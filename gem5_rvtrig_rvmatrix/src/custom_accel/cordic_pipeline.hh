/*
 * NOVA Processor — CORDIC Pipeline Accelerator
 * PhD Research: Chandraboul
 *
 * A fully pipelined CORDIC (COordinate Rotation DIgital Computer) unit
 * with configurable pipeline depth (up to 64 stages, default 32).
 *
 * Supported CORDIC modes:
 *   - Circular Rotation:  sin, cos
 *   - Circular Vectoring: atan, atan2, magnitude/hypot
 *   - Hyperbolic Rotation: exp, sinh, cosh
 *   - Hyperbolic Vectoring: ln, sqrt, tanh (via sinh/cosh)
 *
 * Pipeline architecture:
 *   Stage 0:      Pre-processing (range reduction, mode selection)
 *   Stages 1..N:  CORDIC micro-rotations (N = num_iterations)
 *   Stage N+1:    Post-processing (gain compensation, denormalization)
 *
 * Throughput: 1 result/cycle (fully pipelined)
 * Latency:    N+2 cycles per operation
 */

#ifndef __CUSTOM_ACCEL_CORDIC_PIPELINE_HH__
#define __CUSTOM_ACCEL_CORDIC_PIPELINE_HH__

#include "mem/port.hh"
#include "params/CORDICPipeline.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <array>
#include <cmath>
#include <deque>
#include <vector>

namespace gem5
{

/**
 * CORDICPipeline — Pipelined CORDIC Functional Unit
 *
 * Models a multi-stage CORDIC pipeline that can execute trigonometric,
 * hyperbolic, exponential, logarithmic, magnitude, and sqrt operations.
 * The pipeline is fully pipelined: one new operation can be accepted
 * every cycle, and results emerge after (numIterations + 2) cycles.
 */
class CORDICPipeline : public ClockedObject
{
  public:
    PARAMS(CORDICPipeline);
    CORDICPipeline(const Params &p);
    ~CORDICPipeline();

    /* ===== CORDIC Operation Types ===== */
    enum CORDICOp : int {
        OP_SIN    = 0,
        OP_COS    = 1,
        OP_TAN    = 2,
        OP_ASIN   = 3,
        OP_ACOS   = 4,
        OP_ATAN   = 5,
        OP_ATAN2  = 6,
        OP_EXP    = 7,
        OP_LOG    = 8,
        OP_SINH   = 9,
        OP_COSH   = 10,
        OP_TANH   = 11,
        OP_HYPOT  = 12,    /* magnitude: sqrt(x² + y²) */
        OP_SQRT   = 13,
        NUM_OPS   = 14
    };

    /* ===== CORDIC Modes ===== */
    enum CORDICMode : int {
        MODE_CIRCULAR_ROTATION   = 0,
        MODE_CIRCULAR_VECTORING  = 1,
        MODE_HYPERBOLIC_ROTATION = 2,
        MODE_HYPERBOLIC_VECTORING = 3
    };

    /* ===== Precision ===== */
    enum Precision : int {
        PREC_F32 = 0,
        PREC_F64 = 1
    };

    /* ===== MMIO Port for CPU access ===== */
    class MMIOPort : public ResponsePort
    {
      private:
        CORDICPipeline *accel;
      public:
        MMIOPort(const std::string &name, CORDICPipeline *_accel)
            : ResponsePort(name), accel(_accel) {}

        AddrRangeList getAddrRanges() const override {
            return accel->getAddrRanges();
        }
        Tick recvAtomic(PacketPtr pkt) override {
            return accel->handleMMIO(pkt);
        }
        void recvFunctional(PacketPtr pkt) override {
            accel->handleMMIO(pkt);
        }
        bool recvTimingReq(PacketPtr pkt) override {
            accel->handleMMIO(pkt);
            pkt->makeResponse();
            sendTimingResp(pkt);
            return true;
        }
        void recvRespRetry() override {}
    };

    Port &getPort(const std::string &name,
                  PortID idx = InvalidPortID) override;
    AddrRangeList getAddrRanges() const;

    /* ===== Pipeline Interface ===== */

    /**
     * Submit an operation to the pipeline.
     * Returns the total latency in cycles (numIterations + 2).
     */
    int submitOperation(CORDICOp op, Precision prec,
                        double inputA, double inputB, int coreId);

    /**
     * Advance the pipeline by one cycle.  Called from the clock event.
     */
    void tick();

    /**
     * Check if the pipeline can accept a new operation this cycle.
     */
    bool canAccept() const { return !pipelineFull; }

    /**
     * Retrieve the result for a given core (after completion).
     */
    double getResult(int coreId) const;

    /**
     * Check if a result is ready for a given core.
     */
    bool resultReady(int coreId) const;

    /**
     * Get the pipeline latency for a given operation (in cycles).
     */
    int getOpLatency(CORDICOp op) const;

    /* ===== MMIO Register Offsets ===== */
    static const Addr REG_CTRL        = 0x00;
    static const Addr REG_STATUS      = 0x08;
    static const Addr REG_CORE_ID     = 0x10;
    static const Addr REG_INPUT_A     = 0x18;
    static const Addr REG_INPUT_B     = 0x20;
    static const Addr REG_RESULT      = 0x28;
    static const Addr REG_WAIT_CYCLES = 0x30;
    static const Addr REG_OP_TYPE     = 0x38;
    static const Addr REG_PRECISION   = 0x40;
    static const Addr REG_NUM_STAGES  = 0x48;
    static const Addr REG_PIPE_STATUS = 0x50; /* pipeline occupancy */

    /* Control/Status bits */
    static const uint64_t CTRL_START  = 1 << 0;
    static const uint64_t STATUS_BUSY = 1 << 0;
    static const uint64_t STATUS_DONE = 1 << 1;
    static const uint64_t STATUS_FULL = 1 << 2;

  private:
    /* ===== Pipeline Stage Register ===== */
    struct PipelineStage {
        bool     valid;
        CORDICOp op;
        CORDICMode mode;
        Precision prec;
        int      coreId;
        Tick     enterTime;
        double   x, y, z;     /* CORDIC working registers */
        int      iteration;   /* current CORDIC iteration */
        double   result;
        bool     resultReady;

        PipelineStage()
            : valid(false), op(OP_SIN), mode(MODE_CIRCULAR_ROTATION),
              prec(PREC_F64), coreId(0), enterTime(0),
              x(0), y(0), z(0), iteration(0),
              result(0), resultReady(false) {}
    };

    /* ===== CORDIC Constants ===== */
    static constexpr int MAX_ITERATIONS = 64;

    /* Pre-computed atan(2^-i) for circular mode */
    std::array<double, MAX_ITERATIONS> circularAngles;

    /* Pre-computed atanh(2^-i) for hyperbolic mode */
    std::array<double, MAX_ITERATIONS> hyperbolicAngles;

    /* Circular gain K_n = product(sqrt(1 + 2^(-2i))) for i=0..n-1 */
    double circularGain;     /* ≈ 1.6467602581... */
    double circularGainInv;  /* 1/K_n ≈ 0.6072529350... */

    /* Hyperbolic gain K_h = product(sqrt(1 - 2^(-2i))) for i=1..n */
    double hyperbolicGain;     /* ≈ 0.8281593609... */
    double hyperbolicGainInv;  /* 1/K_h ≈ 1.2074970677... */

    void initLookupTables();

    /* ===== Pipeline data ===== */
    int numIterations;    /* Number of CORDIC iterations per op (default 32) */
    int pipelineDepth;    /* numIterations + 2 (pre + post) */
    int numCores;
    bool pipelineFull;

    /* Pipeline registers — one per stage */
    std::vector<PipelineStage> stages;

    /* Per-core result buffers */
    std::vector<double>  resultBuffers;
    std::vector<bool>    resultValid;
    std::vector<Tick>    waitCycles;

    /* MMIO state */
    Addr mmioBase;
    Addr mmioSize;
    uint64_t regCtrl, regStatus, regCoreId, regOpType, regPrec;
    double regInputA, regInputB;
    MMIOPort mmioPort;

    /* ===== CORDIC Core Algorithm ===== */

    /**
     * Pre-processing: range reduction and initial register setup.
     * Fills stage[0] with x0, y0, z0 for the appropriate mode.
     */
    void preProcess(PipelineStage &s);

    /**
     * One CORDIC micro-rotation iteration.
     * Advances x, y, z by one step.
     */
    void cordicIteration(PipelineStage &s, int i);

    /**
     * Post-processing: gain compensation and mode-specific output.
     */
    void postProcess(PipelineStage &s);

    /**
     * Map operation type → CORDIC mode.
     */
    CORDICMode opToMode(CORDICOp op) const;

    /* ===== Per-operation latency table ===== */
    std::array<int, NUM_OPS> opLatencyTable;
    void initOpLatencies();

    /* ===== MMIO handling ===== */
    Tick handleMMIO(PacketPtr pkt);

    /* ===== Clock event ===== */
    EventFunctionWrapper tickEvent;

    /* ===== Statistics ===== */
    struct CORDICPipelineStats : public statistics::Group
    {
        CORDICPipelineStats(CORDICPipeline *parent);

        statistics::Scalar totalOperations;
        statistics::Scalar pipelineStalls;
        statistics::Scalar totalLatencyCycles;
        statistics::Scalar contentionEvents;
        statistics::Vector perOpCount;       /* per-operation type */
        statistics::Vector perCoreRequests;
        statistics::Vector perCoreWaitCycles;

        /* Per-operation latency distribution */
        statistics::Distribution opLatencyDist;

        /* Pipeline utilization */
        statistics::Scalar pipelineActiveCycles;
        statistics::Scalar pipelineIdleCycles;
    } stats;
};

/* ===== Static helpers for operation name strings ===== */
inline const char* cordicOpName(CORDICPipeline::CORDICOp op)
{
    static const char* names[] = {
        "SIN", "COS", "TAN", "ASIN", "ACOS", "ATAN", "ATAN2",
        "EXP", "LOG", "SINH", "COSH", "TANH", "HYPOT", "SQRT"
    };
    if (op >= 0 && op < CORDICPipeline::NUM_OPS)
        return names[op];
    return "UNKNOWN";
}

} // namespace gem5

#endif // __CUSTOM_ACCEL_CORDIC_PIPELINE_HH__
