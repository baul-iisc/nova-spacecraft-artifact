/*
 * NOVA Processor - MMIO-Based Trigonometric Accelerator
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * This is an MMIO-based accelerator that can be shared among cores.
 * It properly models contention through request queuing.
 * 
 * Features:
 * - CORDIC-based trigonometric operations
 * - Request queue for handling concurrent accesses
 * - Configurable processing latency
 * - Statistics for contention analysis
 */

#ifndef __SPACECRAFT_MMIO_TRIG_ACCEL_HH__
#define __SPACECRAFT_MMIO_TRIG_ACCEL_HH__

#include "dev/io_device.hh"
#include "params/MMIOTrigAccel.hh"
#include "sim/eventq.hh"

#include <queue>
#include <cmath>

namespace gem5
{

namespace spacecraft
{

/**
 * MMIO-based Trigonometric Accelerator
 * 
 * Register Map (relative to base address):
 * 0x00: Control/Status register
 * 0x08: Input A (angle or x)
 * 0x10: Input B (y for atan2)
 * 0x18: Output (result)
 * 0x20: Output B (for sincos)
 * 0x28: Precision mode
 * 0x30: Queue depth (read-only)
 */
class MMIOTrigAccel : public BasicPioDevice
{
  public:
    PARAMS(MMIOTrigAccel);
    
    MMIOTrigAccel(const Params &p);
    ~MMIOTrigAccel();
    
    // PIO device interface
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    
    AddrRangeList getAddrRanges() const override;
    
    void init() override;
    
  private:
    // Operation types
    enum class OpType {
        NONE = 0,
        SIN = 1,
        COS = 2,
        SINCOS = 3,
        TAN = 4,
        ATAN = 5,
        ATAN2 = 6,
        SQRT = 7
    };
    
    // Pending request in the queue
    struct PendingRequest {
        OpType op;
        double inputA;
        double inputB;
        Tick requestTime;
        int requesterId;  // Core ID that made the request
    };
    
    // Configuration
    Cycles processingLatency;  // Cycles per operation
    int maxQueueDepth;         // Maximum queue size
    
    // State
    bool isBusy;
    OpType currentOp;
    double inputA;
    double inputB;
    double outputA;
    double outputB;
    bool resultReady;
    
    // Request queue for contention modeling
    std::queue<PendingRequest> requestQueue;
    
    // Statistics
    struct AccelStats : public statistics::Group
    {
        AccelStats(MMIOTrigAccel *parent);
        
        statistics::Scalar totalRequests;
        statistics::Scalar totalSinOps;
        statistics::Scalar totalCosOps;
        statistics::Scalar totalSincosOps;
        statistics::Scalar totalAtanOps;
        statistics::Scalar totalQueuedRequests;
        statistics::Scalar totalQueueWaitCycles;
        statistics::Scalar maxQueueDepthObserved;
        statistics::Distribution queueDepthDist;
        statistics::Histogram requestLatency;
    } stats;
    
    // Event for completing an operation
    EventFunctionWrapper processEvent;
    void processComplete();
    
    // Execute the current operation
    void executeOperation();
    
    // Start processing next request from queue
    void processNextRequest();
    
    // CORDIC-based computations
    double computeSin(double angle);
    double computeCos(double angle);
    double computeTan(double angle);
    double computeAtan(double x);
    double computeAtan2(double y, double x);
    double computeSqrt(double x);
};

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_MMIO_TRIG_ACCEL_HH__

