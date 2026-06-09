/*
 * Shared 3x3 Matrix Accelerator with MMIO and Arbitration
 * PhD Research: Chandraboul
 * 
 * This accelerator is shared among all cores with round-robin arbitration.
 * Accessed via MMIO, tracks contention and wait cycles.
 */

#ifndef __CUSTOM_ACCEL_SHARED_MATRIX_ACCEL_HH__
#define __CUSTOM_ACCEL_SHARED_MATRIX_ACCEL_HH__

#include "mem/port.hh"
#include "params/SharedMatrixAccel.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <queue>
#include <vector>
#include <array>

namespace gem5
{

class SharedMatrixAccel : public ClockedObject
{
  public:
    PARAMS(SharedMatrixAccel);
    SharedMatrixAccel(const Params &p);
    ~SharedMatrixAccel();
    
    /* MMIO Port for CPU access */
    class MMIOPort : public ResponsePort
    {
      private:
        SharedMatrixAccel *accel;
        
      public:
        MMIOPort(const std::string &name, SharedMatrixAccel *_accel)
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
    
    Port &getPort(const std::string &name, PortID idx = InvalidPortID) override;
    AddrRangeList getAddrRanges() const;

    /* MMIO Register offsets */
    static const Addr REG_CTRL = 0x00;
    static const Addr REG_STATUS = 0x08;
    static const Addr REG_CORE_ID = 0x10;
    static const Addr REG_SRC_A = 0x18;
    static const Addr REG_SRC_B = 0x20;
    static const Addr REG_DST = 0x28;
    static const Addr REG_RESULT_BASE = 0x100;
    static const Addr REG_WAIT_CYCLES = 0x200;
    
    /* Control bits */
    static const uint64_t CTRL_START = 1 << 0;
    static const uint64_t STATUS_BUSY = 1 << 0;
    static const uint64_t STATUS_DONE = 1 << 1;

  private:
    Tick handleMMIO(PacketPtr pkt);
    
    /* Request from a core */
    struct MatrixRequest {
        int coreId;
        Tick arrivalTime;
        Tick startTime;
        Addr srcA, srcB, dst;
    };
    
    /* Parameters */
    int numCores;
    Cycles computeLatency;
    Cycles arbitrationLatency;
    Addr mmioBase;
    Addr mmioSize;
    
    /* State */
    std::queue<MatrixRequest*> requestQueue;
    MatrixRequest* currentRequest;
    
    /* Per-core state */
    std::vector<bool> requestComplete;
    std::vector<Tick> waitCycles;
    std::vector<std::array<double, 9>> resultBuffers;
    
    /* MMIO registers */
    uint64_t regCtrl;
    uint64_t regStatus;
    uint64_t regCoreId;
    uint64_t regSrcA, regSrcB, regDst;
    
    MMIOPort mmioPort;
    
    void processQueue();
    void completeOperation();
    
    EventFunctionWrapper processEvent;
    EventFunctionWrapper completeEvent;
    
    /* Statistics - MUST be declared after numCores */
    struct SharedMatrixAccelStats : public statistics::Group
    {
        SharedMatrixAccelStats(SharedMatrixAccel *parent);
        
        statistics::Scalar totalRequests;
        statistics::Scalar completedOperations;
        statistics::Scalar contentionEvents;
        statistics::Scalar totalWaitCycles;
        statistics::Scalar maxQueueDepth;
        statistics::Vector perCoreRequests;
        statistics::Vector perCoreWaitCycles;
    } stats;
};

} // namespace gem5

#endif
