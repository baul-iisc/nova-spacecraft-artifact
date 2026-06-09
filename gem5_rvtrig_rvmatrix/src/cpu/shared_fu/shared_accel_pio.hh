/*
 * NOVA Processor - Shared Accelerator PIO Device
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * This is a PIO (Programmed I/O) device that models a shared accelerator.
 * When multiple CPUs access it simultaneously:
 * - The first request is served immediately (after operation latency)
 * - Subsequent requests are queued
 * - Contention causes increased response latency
 * 
 * This naturally integrates with gem5's timing model to cause actual
 * CPU stalls when the accelerator is busy.
 */

#ifndef __CPU_SHARED_FU_SHARED_ACCEL_PIO_HH__
#define __CPU_SHARED_FU_SHARED_ACCEL_PIO_HH__

#include <queue>
#include <vector>

#include "dev/io_device.hh"
#include "mem/packet.hh"
#include "params/SharedAccelPio.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * Shared Accelerator as a PIO Device
 * 
 * Register Map:
 * 0x00: CTRL      - Control register (write: start op, read: type)
 * 0x08: STATUS    - Status (0=idle, 1=busy, queue depth in upper bits)
 * 0x10: INPUT0    - Input operand 0
 * 0x18: INPUT1    - Input operand 1
 * 0x20: OUTPUT0   - Output result 0
 * 0x28: OUTPUT1   - Output result 1
 * 0x30: LATENCY   - Current operation latency
 * 0x38: STATS     - Queue stats (total requests)
 * 
 * Operation:
 * 1. Write INPUT0, INPUT1 with operands
 * 2. Write CTRL with operation type (0=sin, 1=cos, 2=mul3x3, etc.)
 * 3. Poll STATUS until idle, or wait for interrupt
 * 4. Read OUTPUT0, OUTPUT1 for results
 */
class SharedAccelPio : public PioDevice
{
  public:
    PARAMS(SharedAccelPio);
    
    SharedAccelPio(const Params &p);
    ~SharedAccelPio();
    
    /** Accelerator type */
    enum class AccelType : uint32_t {
        TRIG = 0,   // Trigonometric
        MAT = 1     // Matrix
    };
    
    /** Operation types */
    enum class OpType : uint32_t {
        // Trig ops
        SIN = 0, COS = 1, TAN = 2, ATAN = 3, ATAN2 = 4, SINCOS = 5,
        // Matrix ops
        MUL3x3 = 16, TRANSPOSE = 17, INVERSE = 18, MATVEC = 19
    };
    
    /** Register offsets */
    static constexpr Addr REG_CTRL     = 0x00;
    static constexpr Addr REG_STATUS   = 0x08;
    static constexpr Addr REG_INPUT0   = 0x10;
    static constexpr Addr REG_INPUT1   = 0x18;
    static constexpr Addr REG_OUTPUT0  = 0x20;
    static constexpr Addr REG_OUTPUT1  = 0x28;
    static constexpr Addr REG_LATENCY  = 0x30;
    static constexpr Addr REG_STATS    = 0x38;
    static constexpr Addr REG_SIZE     = 0x40;
    
    /** PIO interface */
    AddrRangeList getAddrRanges() const override;
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    
    void init() override;
    
  private:
    /** Configuration */
    AccelType accelType;
    int instanceId;
    Addr pioAddr;
    Cycles trigLatency;
    Cycles matLatency;
    size_t maxQueueDepth;
    
    /** Registers */
    uint64_t regInput0;
    uint64_t regInput1;
    uint64_t regOutput0;
    uint64_t regOutput1;
    
    /** State */
    bool busy;
    int currentCpuId;
    OpType currentOp;
    Tick operationStartTick;
    
    /** Pending request queue */
    struct PendingRequest {
        PacketPtr pkt;
        Tick arrivalTime;
        bool isWrite;
    };
    std::queue<PendingRequest> requestQueue;
    
    /** Statistics */
    uint64_t totalRequests;
    uint64_t totalQueuedRequests;
    uint64_t totalBusyCycles;
    uint64_t peakQueueDepth;
    Tick lastIdleTick;
    
    /** Start an operation */
    void startOperation(OpType op, int cpuId);
    
    /** Complete current operation */
    void completeOperation();
    
    /** Process next queued request */
    void processNextRequest();
    
    /** Get latency for an operation */
    Cycles getOpLatency(OpType op) const;
    
    /** Compute result (simplified) */
    void computeResult(OpType op);
    
    /** Events */
    EventFunctionWrapper completeEvent;
    
    /** Statistics */
    struct SharedAccelStats : public statistics::Group
    {
        SharedAccelStats(SharedAccelPio *parent);
        
        statistics::Scalar requests;
        statistics::Scalar queuedRequests;
        statistics::Scalar busyCycles;
        statistics::Scalar avgQueueDepth;
        statistics::Scalar avgWaitCycles;
        statistics::Histogram queueDepthHist;
        statistics::Histogram waitTimeHist;
    } stats;
    
    // Track wait times for stats
    std::vector<Cycles> waitTimes;
    std::vector<size_t> queueDepths;
};

} // namespace gem5

#endif // __CPU_SHARED_FU_SHARED_ACCEL_PIO_HH__


