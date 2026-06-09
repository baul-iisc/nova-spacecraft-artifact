/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * DianNao-Style Matrix Tile Accelerator
 * 
 * Implements a tiled matrix multiplication accelerator with:
 * - Input Buffer (NBin-style) for matrix A tiles
 * - Weight Buffer (SB-style) for matrix B tiles  
 * - Accumulator Buffer for partial sums (C tiles)
 * - Output Buffer (NBout-style) for results
 * - DMA engine for efficient memory transfers
 * - Hardware tiling - handles arbitrary matrix sizes with 3x3 tiles
 */

#ifndef __CUSTOM_ACCEL_MATRIX_TILE_ACCEL_HH__
#define __CUSTOM_ACCEL_MATRIX_TILE_ACCEL_HH__

#include <queue>
#include <vector>
#include <cstdint>
#include <functional>

#include "mem/port.hh"
#include "params/MatrixTileAccel.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * MatrixTileAccel - DianNao-inspired Tiled Matrix Multiplication Accelerator
 * 
 * Architecture:
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                     DMA Engine                                   │
 * │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
 * │  │ Load Queue  │  │ Store Queue │  │   Arbiter   │              │
 * │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
 * └─────────┼────────────────┼────────────────┼─────────────────────┘
 *           │                │                │
 *           ▼                ▼                ▼
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                     On-Chip SRAM                                 │
 * │  ┌───────────────┐  ┌───────────────┐  ┌───────────────────┐    │
 * │  │  Input Buffer │  │ Weight Buffer │  │ Accumulator Buffer│    │
 * │  │   (A tiles)   │  │   (B tiles)   │  │    (C tiles)      │    │
 * │  │   NBin: 4KB   │  │    SB: 4KB    │  │    AccBuf: 2KB    │    │
 * │  └───────┬───────┘  └───────┬───────┘  └─────────┬─────────┘    │
 * │          │                  │                    │              │
 * │          ▼                  ▼                    ▼              │
 * │  ┌─────────────────────────────────────────────────────────┐    │
 * │  │              Systolic MAC Array (3x3)                   │    │
 * │  │   ┌───┐ ┌───┐ ┌───┐                                     │    │
 * │  │   │MAC│→│MAC│→│MAC│   Row 0                             │    │
 * │  │   └─┬─┘ └─┬─┘ └─┬─┘                                     │    │
 * │  │     ↓     ↓     ↓                                       │    │
 * │  │   ┌───┐ ┌───┐ ┌───┐                                     │    │
 * │  │   │MAC│→│MAC│→│MAC│   Row 1                             │    │
 * │  │   └─┬─┘ └─┬─┘ └─┬─┘                                     │    │
 * │  │     ↓     ↓     ↓                                       │    │
 * │  │   ┌───┐ ┌───┐ ┌───┐                                     │    │
 * │  │   │MAC│→│MAC│→│MAC│   Row 2                             │    │
 * │  │   └───┘ └───┘ └───┘                                     │    │
 * │  └─────────────────────────────────────────────────────────┘    │
 * │          │                                                      │
 * │          ▼                                                      │
 * │  ┌───────────────────┐                                          │
 * │  │   Output Buffer   │                                          │
 * │  │   NBout: 2KB      │                                          │
 * │  └───────────────────┘                                          │
 * └─────────────────────────────────────────────────────────────────┘
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x00: CTRL           - Control register (start, op type, reset)
 * 0x04: STATUS         - Status register (busy, done, error)
 * 0x08: MAT_A_ADDR     - Matrix A base address in memory
 * 0x0C: MAT_B_ADDR     - Matrix B base address in memory
 * 0x10: MAT_C_ADDR     - Matrix C base address in memory (result)
 * 0x14: MAT_DIM_M      - Matrix dimension M (rows of A and C)
 * 0x18: MAT_DIM_N      - Matrix dimension N (cols of B and C)
 * 0x1C: MAT_DIM_K      - Matrix dimension K (cols of A, rows of B)
 * 0x20: STRIDE_A       - Row stride for matrix A (in bytes)
 * 0x24: STRIDE_B       - Row stride for matrix B (in bytes)
 * 0x28: STRIDE_C       - Row stride for matrix C (in bytes)
 * 0x2C: DATA_TYPE      - Data type (0=float, 1=double, 2=int32, 3=int8)
 * 0x30: TILE_CONFIG    - Tile configuration (tile_size, overlap mode)
 * 0x34: PERF_CYCLES    - Performance counter: total cycles
 * 0x38: PERF_TILES     - Performance counter: tiles processed
 * 0x3C: PERF_MACS      - Performance counter: MAC operations
 * 0x40: DMA_STATUS     - DMA engine status
 * 0x44: BUFFER_STATUS  - Buffer occupancy status
 */
class MatrixTileAccel : public ClockedObject
{
  public:
    PARAMS(MatrixTileAccel);
    MatrixTileAccel(const Params &params);
    ~MatrixTileAccel();

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();

  private:
    /**
     * CPU-side port for MMIO register access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        MatrixTileAccel *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, MatrixTileAccel *owner);
        void sendPacket(PacketPtr pkt);
        AddrRangeList getAddrRanges() const override;
        void trySendRetry();

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    /**
     * Memory-side port for DMA transfers
     */
    class MemSidePort : public RequestPort
    {
      private:
        MatrixTileAccel *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, MatrixTileAccel *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Data types supported
    enum class DataType {
        FLOAT32 = 0,
        FLOAT64 = 1,
        INT32 = 2,
        INT8 = 3
    };

    // Operation types
    enum class OpType {
        MATMUL = 0,           // C = A × B
        MATMUL_ACC = 1,       // C += A × B
        MATMUL_TRANSPOSE = 2, // C = A × B^T
        GEMM = 3              // C = alpha*A×B + beta*C
    };

    // DMA transfer types
    enum class DmaType {
        LOAD_A,
        LOAD_B,
        LOAD_C,
        STORE_C
    };

    // DMA request structure
    struct DmaRequest {
        DmaType type;
        Addr addr;
        size_t size;
        int tileRow;
        int tileCol;
        int tileK;
    };

    // Register offsets
    static const Addr REG_CTRL         = 0x00;
    static const Addr REG_STATUS       = 0x04;
    static const Addr REG_MAT_A_ADDR   = 0x08;
    static const Addr REG_MAT_B_ADDR   = 0x0C;
    static const Addr REG_MAT_C_ADDR   = 0x10;
    static const Addr REG_MAT_DIM_M    = 0x14;
    static const Addr REG_MAT_DIM_N    = 0x18;
    static const Addr REG_MAT_DIM_K    = 0x1C;
    static const Addr REG_STRIDE_A     = 0x20;
    static const Addr REG_STRIDE_B     = 0x24;
    static const Addr REG_STRIDE_C     = 0x28;
    static const Addr REG_DATA_TYPE    = 0x2C;
    static const Addr REG_TILE_CONFIG  = 0x30;
    static const Addr REG_PERF_CYCLES  = 0x34;
    static const Addr REG_PERF_TILES   = 0x38;
    static const Addr REG_PERF_MACS    = 0x3C;
    static const Addr REG_DMA_STATUS   = 0x40;
    static const Addr REG_BUF_STATUS   = 0x44;
    static const Addr REG_ALPHA        = 0x48;  // For GEMM
    static const Addr REG_BETA         = 0x4C;  // For GEMM

    // Control bits
    static const uint32_t CTRL_START    = 0x01;
    static const uint32_t CTRL_ABORT    = 0x02;
    static const uint32_t CTRL_RESET    = 0x04;
    static const uint32_t CTRL_OP_MASK  = 0x30;
    static const uint32_t CTRL_OP_SHIFT = 4;

    // Status bits
    static const uint32_t STATUS_IDLE   = 0x00;
    static const uint32_t STATUS_BUSY   = 0x01;
    static const uint32_t STATUS_DONE   = 0x02;
    static const uint32_t STATUS_ERROR  = 0x04;
    static const uint32_t STATUS_DMA_ACTIVE = 0x08;

    // Hardware configuration (from params)
    static constexpr int TILE_SIZE = 3;       // 3x3 tile size
    unsigned inputBufferSize;                 // Input buffer size (bytes)
    unsigned weightBufferSize;                // Weight buffer size (bytes)
    unsigned accumBufferSize;                 // Accumulator buffer size (bytes)
    unsigned outputBufferSize;                // Output buffer size (bytes)
    unsigned macLatency;                      // Cycles per tile MAC operation
    unsigned dmaLatency;                      // Cycles per DMA transfer
    unsigned numTileSlots;                    // Number of tiles that fit in each buffer

    // Internal registers
    uint32_t regCtrl;
    uint32_t regStatus;
    Addr regMatAAddr;
    Addr regMatBAddr;
    Addr regMatCAddr;
    uint32_t regDimM;
    uint32_t regDimN;
    uint32_t regDimK;
    uint32_t regStrideA;
    uint32_t regStrideB;
    uint32_t regStrideC;
    uint32_t regDataType;
    uint32_t regTileConfig;
    uint64_t regPerfCycles;
    uint64_t regPerfTiles;
    uint64_t regPerfMacs;
    uint32_t regDmaStatus;
    uint32_t regBufStatus;
    float regAlpha;
    float regBeta;

    // Current operation state
    OpType currentOp;
    DataType dataType;
    bool busy;
    bool aborted;

    // Tiling state
    int numTilesM;      // Number of tiles in M dimension
    int numTilesN;      // Number of tiles in N dimension
    int numTilesK;      // Number of tiles in K dimension
    int currentTileM;   // Current tile row
    int currentTileN;   // Current tile column
    int currentTileK;   // Current K accumulation step

    // On-chip buffers (DianNao-style)
    // Input buffer (NBin) - stores A tiles
    std::vector<double> inputBuffer;
    std::vector<bool> inputBufferValid;
    
    // Weight buffer (SB) - stores B tiles
    std::vector<double> weightBuffer;
    std::vector<bool> weightBufferValid;
    
    // Accumulator buffer - stores partial sums for C tiles
    std::vector<double> accumBuffer;
    std::vector<bool> accumBufferValid;
    
    // Output buffer (NBout) - stores final C tiles before writeback
    std::vector<double> outputBuffer;
    std::vector<bool> outputBufferValid;

    // DMA engine state
    std::queue<DmaRequest> dmaLoadQueue;
    std::queue<DmaRequest> dmaStoreQueue;
    bool dmaActive;
    DmaRequest currentDmaReq;

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // Events
    EventFunctionWrapper tileComputeEvent;
    EventFunctionWrapper dmaCompleteEvent;
    EventFunctionWrapper operationCompleteEvent;

    // Statistics
    struct AccelStats : public statistics::Group
    {
        AccelStats(MatrixTileAccel *parent);
        statistics::Scalar totalOperations;
        statistics::Scalar totalTilesProcessed;
        statistics::Scalar totalMacOperations;
        statistics::Scalar totalCycles;
        statistics::Scalar dmaBytesLoaded;
        statistics::Scalar dmaBytesStored;
        statistics::Scalar dmaTransfers;
        statistics::Scalar bufferHits;
        statistics::Scalar bufferMisses;
    } stats;

    // Internal methods - Port handling
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    
    // Register access
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);
    uint64_t readReg64(Addr offset);
    void writeReg64(Addr offset, uint64_t value);

    // Operation control
    void startOperation();
    void abortOperation();
    void resetAccelerator();
    void completeOperation();

    // Tiling logic
    void initializeTiling();
    void scheduleNextTile();
    bool allTilesComplete();
    int getTileIndex(int tileM, int tileN);
    
    // Buffer management
    void clearBuffers();
    bool isTileInInputBuffer(int tileM, int tileK);
    bool isTileInWeightBuffer(int tileK, int tileN);
    bool isTileInAccumBuffer(int tileM, int tileN);
    void loadTileToInputBuffer(int tileM, int tileK);
    void loadTileToWeightBuffer(int tileK, int tileN);
    void loadTileToAccumBuffer(int tileM, int tileN);
    void storeTileFromAccumBuffer(int tileM, int tileN);

    // DMA engine
    void enqueueDmaLoad(DmaType type, Addr addr, size_t size, 
                        int tileRow, int tileCol, int tileK);
    void enqueueDmaStore(Addr addr, size_t size, 
                         int tileRow, int tileCol);
    void processDmaQueue();
    void handleDmaComplete();
    Addr calculateTileAddress(Addr baseAddr, uint32_t stride, 
                              int tileRow, int tileCol);

    // Compute engine
    void executeTileMultiply();
    void performTileMAC(int tileM, int tileN, int tileK);
    
    // Data type helpers
    size_t getElementSize();
    size_t getTileDataSize();
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_MATRIX_TILE_ACCEL_HH__

