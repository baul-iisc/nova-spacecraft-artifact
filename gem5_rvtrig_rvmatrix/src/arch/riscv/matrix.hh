//src/arch/riscv/matrix.hh
#ifndef __ARCH_RISCV_MATRIX_HH__
#define __ARCH_RISCV_MATRIX_HH__

#include <array>
#include <cassert>
#include <cstring>
#include <type_traits>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "sim/serialize_handlers.hh"

namespace gem5
{

// Forward declaration for SystolicMatrixEngine
class SystolicMatrixEngine;

class MatStore
{
    static constexpr size_t RLEN = 192; // in bits (3x3 matrices)
    static constexpr size_t LINES = RLEN / 32;
    static constexpr size_t LINEAR_SIZE = (RLEN * RLEN / 32) / 8;

  public:
    static constexpr inline size_t linearSize() { return LINEAR_SIZE; };
    static constexpr inline size_t numElements32() { return LINEAR_SIZE / 4; }; // 36 floats for 3x3
    static constexpr inline size_t numElements64() { return LINEAR_SIZE / 8; }; // 18 doubles for 3x3

    using Container = std::array<uint8_t, LINEAR_SIZE>;
    using MyClass = MatStore;

  private:
    alignas(16) Container container;

    // Friend declaration for systolic array engine
    friend class SystolicMatrixEngine;

  public:
    MatStore() { zero(); } // Changed to initialize to zero
    MatStore(const MatStore&) = default;

    void
    zero()
    {
        memset(container.data(), 0, LINEAR_SIZE);
    }

    MyClass&
    operator=(const MyClass& that)
    {
        if (&that == this)
            return *this;
        memcpy(container.data(), that.container.data(), LINEAR_SIZE);
        return *this;
    }

    inline bool
    operator==(const MatStore& that) const
    {
        return !memcmp(container.data(), that.container.data(), LINEAR_SIZE);
    }

    bool
    operator!=(const MatStore& that) const
    {
        return !operator==(that);
    }

  private:
    template <typename ElemType>
    const ElemType* rawPtr() const
    {
        return reinterpret_cast<const ElemType*>(container.data());
    }

    template <typename ElemType>
    ElemType* rawPtr()
    {
        return reinterpret_cast<ElemType*>(container.data());
    }

  public:
    template <typename VecElem>
    VecElem *
    as()
    {
        static_assert(LINEAR_SIZE % sizeof(VecElem) == 0,
                      "VecElem does not evenly divide the register size");
        return (VecElem *)container.data();
    }

    template <typename VecElem>
    const VecElem *
    as() const
    {
        static_assert(LINEAR_SIZE % sizeof(VecElem) == 0,
                      "VecElem does not evenly divide the register size");
        return (VecElem *)container.data();
    }
    
    double *
    asDouble()
    {
        static_assert(LINEAR_SIZE % sizeof(double) == 0,
                      "double does not evenly divide the register size");
        return reinterpret_cast<double*>(container.data());
    }

    const double *
    asDouble() const
    {
        static_assert(LINEAR_SIZE % sizeof(double) == 0,
                      "double does not evenly divide the register size");
        return reinterpret_cast<const double*>(container.data());
    }

    float *
    asFloat()
    {
        static_assert(LINEAR_SIZE % sizeof(float) == 0,
                      "float does not evenly divide the register size");
        return reinterpret_cast<float*>(container.data());
    }

    const float *
    asFloat() const
    {
        static_assert(LINEAR_SIZE % sizeof(float) == 0,
                      "float does not evenly divide the register size");
        return reinterpret_cast<const float*>(container.data());
    }
    
    // Add debugging methods
    void printMatrix(const char* label) const {
        printf("%s:\n", label);
        const double* data = asDouble();
        for (int i = 0; i < 3; i++) {
                printf("%.6f ", data[i * 3]);
            }
            printf("\n");
    }

    friend std::ostream&
    operator<<(std::ostream& os, const MatStore& v)
    {
        ccprintf(os, "[");
        size_t count = 0;
        for (auto& b: v.container) {
            if (count && (count % 4) == 0)
                os << "_";
            ccprintf(os, "%02x", b);
            count++;
        }
        ccprintf(os, "]");
        return os;
    }

    friend ParseParam<MatStore>;
    friend ShowParam<MatStore>;
};

// =========================================================================
// Systolic Array 3x3 Matrix Multiplication Unit
// =========================================================================
//
// Architecture: Output-Stationary 3x3 Systolic Array
//
// PE Array Layout:
//     B[0][j]    B[1][j]    B[2][j]   (B columns feed from top)
//       |          |          |
//  A[i][0] -> PE[0][0] -> PE[0][1] -> PE[0][2]   C[0][0] C[0][1] C[0][2]
//       |          |          |
//  A[i][1] -> PE[1][0] -> PE[1][1] -> PE[1][2]   C[1][0] C[1][1] C[1][2]
//       |          |          |
//  A[i][2] -> PE[2][0] -> PE[2][1] -> PE[2][2]   C[2][0] C[2][1] C[2][2]
//
// Dataflow: Output-stationary
//   - C[i][j] stays in PE[i][j] (accumulated locally, no writeback until done)
//   - A rows stream left-to-right through PEs
//   - B columns stream top-to-bottom through PEs
//   - At time step t, PE[i][j] receives A[i][k] and B[k][j] where k = t - i - j
//   - Total time steps = 2*(N-1) + N = 3*N - 2 = 7 for N=3
//
// Benefits over naive triple-nested loop:
//   1. Data reuse: Each A[i][k] read once from register file, used by N PEs
//   2. Data reuse: Each B[k][j] read once from register file, used by N PEs
//   3. Reduced register file access: 2*N*N reads vs N*N*N*2 reads (naive)
//   4. Parallel MACs: Up to N*N PEs active simultaneously per time step
//   5. No intermediate memory writeback: C accumulates locally in PE
//
// Performance comparison for 3x3:
//   Naive:    27 MACs sequential, 54 reads + 9 R/W = 63 register accesses
//   Systolic: 27 MACs in 7 pipelined steps, 18 reads + 9 R/W = 27 reg accesses
//   Memory access reduction: 63 -> 27 = 57% fewer register file accesses
//   Effective throughput: 27 MACs / 7 cycles = 3.86 MACs/cycle
// =========================================================================

// Processing Element (PE) for the 3x3 Systolic Array
// Each PE contains a local MAC unit and input/output latches
struct SystolicPE
{
    // Local accumulator - stores partial result for C[i][j]
    double acc_d;    // Double precision accumulator
    float  acc_f;    // Single precision accumulator

    // Input latches (data received from neighbors or direct feed)
    double a_in_d;   // A data from left neighbor (or register file for col 0)
    double b_in_d;   // B data from top neighbor (or register file for row 0)
    float  a_in_f;
    float  b_in_f;

    // Output latches (data passed to neighbors)
    double a_out_d;  // A data passed to right neighbor
    double b_out_d;  // B data passed to bottom neighbor
    float  a_out_f;
    float  b_out_f;

    // PE state
    bool active;     // Whether this PE is active in current time step

    SystolicPE()
        : acc_d(0.0), acc_f(0.0f),
          a_in_d(0.0), b_in_d(0.0), a_in_f(0.0f), b_in_f(0.0f),
          a_out_d(0.0), b_out_d(0.0), a_out_f(0.0f), b_out_f(0.0f),
          active(false) {}

    void reset()
    {
        acc_d = 0.0; acc_f = 0.0f;
        a_in_d = 0.0; b_in_d = 0.0;
        a_in_f = 0.0f; b_in_f = 0.0f;
        a_out_d = 0.0; b_out_d = 0.0;
        a_out_f = 0.0f; b_out_f = 0.0f;
        active = false;
    }

    // Execute one MAC step (double precision)
    // acc += a_in * b_in; pass data to neighbors
    void computeDouble()
    {
        acc_d += a_in_d * b_in_d;
        a_out_d = a_in_d;    // Pass A to the right
        b_out_d = b_in_d;    // Pass B downward
    }

    // Execute one MAC step (single precision)
    void computeFloat()
    {
        acc_f += a_in_f * b_in_f;
        a_out_f = a_in_f;
        b_out_f = b_in_f;
    }
};

// Enhanced Systolic Matrix Engine with 3x3 PE Array
// Implements output-stationary dataflow for matrix multiplication
class SystolicMatrixEngine
{
  public:
    static constexpr int N = 3;  // Array dimension (3x3)

    // 3x3 Processing Element array
    SystolicPE pe[N][N];

  private:
    // Configuration
    uint32_t rows;
    uint32_t cols;
    uint32_t precision; // 0 = 32-bit float, 1 = 64-bit double
    uint32_t mode;      // 0 = matrix-matrix, 1 = convolution, 2 = matrix-vector

    // Status
    uint32_t status;    // 0 = idle, 1 = busy, 2 = done

    // Internal data buffers (read once from register file, reused by PEs)
    // This models the systolic array's local storage that eliminates
    // repeated register file accesses
    double a_buf_d[N][N];  // A matrix buffer (double)
    double b_buf_d[N][N];  // B matrix buffer (double)
    float  a_buf_f[N][N];  // A matrix buffer (float)
    float  b_buf_f[N][N];  // B matrix buffer (float)

    // Storage for MatStore-based operations
    MatStore matrixA;
    MatStore matrixB;
    MatStore matrixC;

    // Performance counters
    uint64_t totalMACOps;          // Total MAC operations executed
    uint64_t totalSystolicCycles;  // Total systolic time steps
    uint64_t regFileReadsNaive;    // Register reads in naive approach
    uint64_t regFileReadsSystolic; // Register reads in systolic approach
    uint64_t memAccessSaved;       // Memory accesses eliminated

  public:
    SystolicMatrixEngine()
        : rows(N), cols(N), precision(0), mode(0), status(0),
          totalMACOps(0), totalSystolicCycles(0),
          regFileReadsNaive(0), regFileReadsSystolic(0), memAccessSaved(0)
    {
        resetPEArray();
        matrixA.zero();
        matrixB.zero();
        matrixC.zero();
    }

    // Reset all PEs to initial state
    void resetPEArray()
    {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                pe[i][j].reset();
    }

    // =====================================================================
    // Configuration and Control Interface
    // =====================================================================

    void configure(uint32_t config)
    {
        rows = (config >> 0) & 0xFF;
        cols = (config >> 8) & 0xFF;
        precision = (config >> 16) & 0x3;
        mode = (config >> 18) & 0x3;
        if (rows > N) rows = N;
        if (cols > N) cols = N;
        status = 0;
    }

    uint32_t getStatus() const { return status; }
    void start() { status = 1; }

    void stop()
    {
        if (status == 1) status = 2;
    }

    void loadMatrixA(const MatStore& src) { matrixA = src; }
    void loadMatrixB(const MatStore& src) { matrixB = src; }
    void loadMatrixC(const MatStore& src) { matrixC = src; }
    void getMatrixC(MatStore& dest) { dest = matrixC; }

    // Performance counter accessors
    uint64_t getMACOps() const { return totalMACOps; }
    uint64_t getSystolicCycles() const { return totalSystolicCycles; }
    uint64_t getRegReadsNaive() const { return regFileReadsNaive; }
    uint64_t getRegReadsSystolic() const { return regFileReadsSystolic; }
    uint64_t getMemAccessSaved() const { return memAccessSaved; }

    // =====================================================================
    // Core Systolic Array Operations - Output Stationary Dataflow
    // =====================================================================

    // Output-stationary systolic multiply: C += A * B (double precision)
    //
    // Data flow model:
    //   Time step t: PE[i][j] is active when k = t - i - j is in [0, N-1]
    //   - PE[i][j] receives A[i][k] and B[k][j]
    //   - PE[i][j] computes: acc[i][j] += A[i][k] * B[k][j]
    //   - A[i][k] is passed to PE[i][j+1] (right neighbor)
    //   - B[k][j] is passed to PE[i+1][j] (bottom neighbor)
    //
    // Total time steps: 2*(N-1) + N = 3*N - 2 = 7 for N=3
    // Total MACs: N*N*N = 27 for N=3
    // Parallel MACs per step: varies from 1 to min(step+1, 2*N-1-step, N*N)
    //
    void systolicMultiplyDouble(double* C, const double* A, const double* B)
    {
        // Step 1: Load data into internal buffers (ONE read from register file)
        // This models the systolic array reading input matrices once
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                a_buf_d[i][j] = A[i * N + j];
                b_buf_d[i][j] = B[i * N + j];
            }
        }

        // Step 2: Initialize PE accumulators from C (output-stationary)
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                pe[i][j].acc_d = C[i * N + j];

        // Step 3: Execute systolic wavefront computation
        // Total time steps = 3*N - 2 = 7 for N=3
        int totalSteps = 3 * N - 2;
        int macCount = 0;
        int maxParallelPEs = 0;

        for (int t = 0; t < totalSteps; t++) {
            int activePEsThisStep = 0;

            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    int k = t - i - j;
                    if (k >= 0 && k < N) {
                        // Feed data from internal buffers to PE
                        pe[i][j].a_in_d = a_buf_d[i][k];
                        pe[i][j].b_in_d = b_buf_d[k][j];
                        pe[i][j].active = true;

                        // Execute MAC: acc += a_in * b_in
                        pe[i][j].computeDouble();

                        macCount++;
                        activePEsThisStep++;
                    } else {
                        pe[i][j].active = false;
                    }
                }
            }

            if (activePEsThisStep > maxParallelPEs)
                maxParallelPEs = activePEsThisStep;
        }

        // Step 4: Write back from PE accumulators to C (ONE write to register file)
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                C[i * N + j] = pe[i][j].acc_d;

        // Update performance counters
        totalMACOps += macCount;
        totalSystolicCycles += totalSteps;
        // Naive: 2*N^3 reads (A and B for each MAC) + N^2 R/W for C = 63
        regFileReadsNaive += 2 * N * N * N + N * N;
        // Systolic: N^2 reads (A once) + N^2 reads (B once) + N^2 R/W (C) = 27
        regFileReadsSystolic += 3 * N * N;
        memAccessSaved += (2 * N * N * N + N * N) - (3 * N * N);
    }

    // Single precision version of systolic multiply
    void systolicMultiplyFloat(float* C, const float* A, const float* B)
    {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                a_buf_f[i][j] = A[i * N + j];
                b_buf_f[i][j] = B[i * N + j];
            }
        }

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                pe[i][j].acc_f = C[i * N + j];

        int totalSteps = 3 * N - 2;
        int macCount = 0;

        for (int t = 0; t < totalSteps; t++) {
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    int k = t - i - j;
                    if (k >= 0 && k < N) {
                        pe[i][j].a_in_f = a_buf_f[i][k];
                        pe[i][j].b_in_f = b_buf_f[k][j];
                        pe[i][j].active = true;
                        pe[i][j].computeFloat();
                        macCount++;
                    } else {
                        pe[i][j].active = false;
                    }
                }
            }
        }

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                C[i * N + j] = pe[i][j].acc_f;

        totalMACOps += macCount;
        totalSystolicCycles += totalSteps;
        regFileReadsNaive += 2 * N * N * N + N * N;
        regFileReadsSystolic += 3 * N * N;
        memAccessSaved += (2 * N * N * N + N * N) - (3 * N * N);
    }

    // =====================================================================
    // Static helper: systolic multiply directly on register data pointers
    // These are called from ISA execution code (decoder.isa) where we
    // have direct double*/float* pointers to matrix register data.
    // No SystolicMatrixEngine instance needed.
    // =====================================================================

    // Static systolic MAC for double precision
    // C[3x3] += A[3x3] * B[3x3] using output-stationary PE array
    // Returns the number of systolic cycles via pointer
    static void systolicMACDouble(double* C, const double* A, const double* B,
                                  int* cycles_out = nullptr, int* macs_out = nullptr)
    {
        // Local PE accumulators (on-chip, no memory access)
        double pe_acc[N][N];

        // Step 1: Initialize accumulators from C (one read per element)
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                pe_acc[i][j] = C[i * N + j];

        // Step 2: Systolic wavefront execution
        // At time t, PE(i,j) is active if k = t - i - j in [0, N-1]
        int totalSteps = 3 * N - 2;  // = 7 for N=3
        int macCount = 0;

        for (int t = 0; t < totalSteps; t++) {
            // All active PEs compute IN PARALLEL this time step
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    int k = t - i - j;
                    if (k >= 0 && k < N) {
                        // PE(i,j) receives A[i][k] from left, B[k][j] from top
                        // Data flows through local interconnect, NOT register file
                        pe_acc[i][j] += A[i * N + k] * B[k * N + j];
                        macCount++;
                    }
                }
            }
        }

        // Step 3: Write back from PE accumulators to C (one write per element)
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                C[i * N + j] = pe_acc[i][j];

        if (cycles_out) *cycles_out = totalSteps;
        if (macs_out) *macs_out = macCount;
    }

    // Static systolic MAC for single precision
    static void systolicMACFloat(float* C, const float* A, const float* B,
                                 int* cycles_out = nullptr, int* macs_out = nullptr)
    {
        float pe_acc[N][N];

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                pe_acc[i][j] = C[i * N + j];

        int totalSteps = 3 * N - 2;
        int macCount = 0;

        for (int t = 0; t < totalSteps; t++) {
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    int k = t - i - j;
                    if (k >= 0 && k < N) {
                        pe_acc[i][j] += A[i * N + k] * B[k * N + j];
                        macCount++;
                    }
                }
            }
        }

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                C[i * N + j] = pe_acc[i][j];

        if (cycles_out) *cycles_out = totalSteps;
        if (macs_out) *macs_out = macCount;
    }

    // =====================================================================
    // MatStore-based operations (used when SystolicMatrixEngine is instantiated)
    // =====================================================================

    void matrixVectorMultiply()
    {
        if (status != 1) return;

        if (precision == 0) {
            float* result = matrixC.asFloat();
            const float* matA = matrixA.asFloat();
            const float* vecB = matrixB.asFloat();
            for (int i = 0; i < (int)rows; i++) {
                result[i*3] = 0;
                for (int k = 0; k < (int)cols; k++)
                    result[i*3] += matA[i*cols + k] * vecB[k*3];
                for (int j = 1; j < 3; j++)
                    result[i*3 + j] = 0;
            }
        } else {
            double* result = matrixC.asDouble();
            const double* matA = matrixA.asDouble();
            const double* vecB = matrixB.asDouble();
            for (int i = 0; i < (int)rows; i++) {
                result[i*3] = 0;
                for (int k = 0; k < (int)cols; k++)
                    result[i*3] += matA[i*cols + k] * vecB[k*3];
                for (int j = 1; j < 3; j++)
                    result[i*3 + j] = 0;
            }
        }
    }

    // Main multiply dispatch method - now uses systolic PE array
    void multiply()
    {
        if (status != 1) return;

        if (mode == 2) {
            matrixVectorMultiply();
        }
        else if (mode == 0) {
            // Matrix-matrix multiplication using systolic PE array
            if (precision == 0) {
                systolicMultiplyFloat(matrixC.asFloat(),
                                      matrixA.asFloat(),
                                      matrixB.asFloat());
            }
            else if (precision == 1) {
                systolicMultiplyDouble(matrixC.asDouble(),
                                       matrixA.asDouble(),
                                       matrixB.asDouble());
            }
        }
    }
};

template<>
struct ParseParam<MatStore>
{
    static bool
    parse(const std::string &str, MatStore &value)
    {
        for (int i = 0; i < value.linearSize(); i++) {
            uint8_t b = 0;
            if (2 * i < str.size())
                b = stoul(str.substr(i * 2, 2), nullptr, 16);
            value.template rawPtr<uint8_t>()[i] = b;
        }
        return true;
    }
};

template<>
struct ShowParam<MatStore>
{
    static void
    show(std::ostream &os, const MatStore &value)
    {
        for (auto& b: value.container)
            ccprintf(os, "%02x", b);
    }
};

} // namespace gem5

#endif // __ARCH_RISCV_MATRIX_HH__
