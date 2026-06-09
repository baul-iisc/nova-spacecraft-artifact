/*
 * Shared CORDIC Trigonometric Accelerator with MMIO and Arbitration
 * PhD Research: Chandraboul
 */

#ifndef __CUSTOM_ACCEL_SHARED_CORDIC_ACCEL_HH__
#define __CUSTOM_ACCEL_SHARED_CORDIC_ACCEL_HH__

#include "mem/port.hh"
#include "params/SharedCORDICAccel.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <queue>
#include <vector>

namespace gem5
{

class SharedCORDICAccel : public ClockedObject
{
  public:
    PARAMS(SharedCORDICAccel);
    SharedCORDICAccel(const Params &p);
    ~SharedCORDICAccel();
    
    /* MMIO Port */
    class MMIOPort : public ResponsePort
    {
      private:
        SharedCORDICAccel *accel;
      public:
        MMIOPort(const std::string &name, SharedCORDICAccel *_accel)
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
    static const Addr REG_INPUT_A = 0x18;
    static const Addr REG_INPUT_B = 0x20;
    static const Addr REG_RESULT = 0x28;
    static const Addr REG_WAIT_CYCLES = 0x30;
    
    /* Control/Status bits */
    static const uint64_t CTRL_START = 1 << 0;
    static const uint64_t CTRL_OP_SIN = 0 << 4;
    static const uint64_t CTRL_OP_COS = 1 << 4;
    static const uint64_t CTRL_OP_TAN = 2 << 4;
    static const uint64_t CTRL_OP_ATAN2 = 3 << 4;
    static const uint64_t STATUS_BUSY = 1 << 0;
    static const uint64_t STATUS_DONE = 1 << 1;

  private:
    Tick handleMMIO(PacketPtr pkt);
    
    struct CORDICRequest {
        int coreId;
        Tick arrivalTime;
        Tick startTime;
        int opType;
        double inputA, inputB;
    };
    
    int numCores;
    Cycles computeLatency;
    Cycles arbitrationLatency;
    Addr mmioBase;
    Addr mmioSize;
    
    std::queue<CORDICRequest*> requestQueue;
    CORDICRequest* currentRequest;
    
    std::vector<bool> requestComplete;
    std::vector<Tick> waitCycles;
    std::vector<double> resultBuffers;
    
    uint64_t regCtrl, regStatus, regCoreId;
    double regInputA, regInputB;
    
    MMIOPort mmioPort;
    
    void processQueue();
    void completeOperation();
    
    EventFunctionWrapper processEvent;
    EventFunctionWrapper completeEvent;
    
    /* Statistics - MUST be declared after numCores */
    struct SharedCORDICStats : public statistics::Group
    {
        SharedCORDICStats(SharedCORDICAccel *parent);
        statistics::Scalar totalRequests;
        statistics::Scalar completedOperations;
        statistics::Scalar contentionEvents;
        statistics::Scalar totalWaitCycles;
        statistics::Vector perCoreRequests;
        statistics::Vector perCoreWaitCycles;
    } stats;
};

} // namespace gem5

#endif
