/*
 * Shared CORDIC Accelerator Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/shared_cordic_accel.hh"
#include "base/trace.hh"
#include "debug/SharedCORDICAccel.hh"
#include <cmath>

namespace gem5
{

SharedCORDICAccel::SharedCORDICAccel(const Params &p)
    : ClockedObject(p),
      numCores(p.num_cores),
      computeLatency(p.compute_latency),
      arbitrationLatency(p.arbitration_latency),
      mmioBase(p.mmio_base),
      mmioSize(p.mmio_size),
      currentRequest(nullptr),
      regCtrl(0), regStatus(0), regCoreId(0),
      regInputA(0), regInputB(0),
      mmioPort(name() + ".mmio_port", this),
      processEvent([this]{ processQueue(); }, name() + ".processEvent"),
      completeEvent([this]{ completeOperation(); }, name() + ".completeEvent"),
      stats(this)
{
    requestComplete.resize(numCores, false);
    waitCycles.resize(numCores, 0);
    resultBuffers.resize(numCores, 0.0);
    
    DPRINTF(SharedCORDICAccel, "SharedCORDICAccel created: %d cores, "
            "MMIO base=0x%lx\n", numCores, mmioBase);
}

SharedCORDICAccel::~SharedCORDICAccel()
{
    while (!requestQueue.empty()) {
        delete requestQueue.front();
        requestQueue.pop();
    }
    if (currentRequest) delete currentRequest;
}

Port &
SharedCORDICAccel::getPort(const std::string &name, PortID idx)
{
    if (name == "mmio_port")
        return mmioPort;
    return ClockedObject::getPort(name, idx);
}

AddrRangeList
SharedCORDICAccel::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(mmioBase, mmioBase + mmioSize));
    return ranges;
}

Tick
SharedCORDICAccel::handleMMIO(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;
    
    if (pkt->isRead()) {
        uint64_t data = 0;
        
        switch (offset) {
            case REG_CTRL:
                data = regCtrl;
                break;
            case REG_STATUS:
                data = regStatus;
                if (currentRequest || !requestQueue.empty())
                    data |= STATUS_BUSY;
                break;
            case REG_CORE_ID:
                data = regCoreId;
                break;
            case REG_RESULT:
                if (regCoreId < (uint64_t)numCores) {
                    double val = resultBuffers[regCoreId];
                    memcpy(&data, &val, sizeof(double));
                }
                break;
            case REG_WAIT_CYCLES:
                if (regCoreId < (uint64_t)numCores)
                    data = waitCycles[regCoreId];
                break;
        }
        
        pkt->setData((uint8_t*)&data);
        DPRINTF(SharedCORDICAccel, "MMIO Read: offset=0x%lx, data=0x%lx\n",
                offset, data);
    }
    else if (pkt->isWrite()) {
        uint64_t data;
        pkt->writeData((uint8_t*)&data);
        
        switch (offset) {
            case REG_CTRL:
                regCtrl = data;
                if (data & CTRL_START) {
                    if (regCoreId < (uint64_t)numCores) {
                        CORDICRequest* req = new CORDICRequest();
                        req->coreId = regCoreId;
                        req->arrivalTime = curTick();
                        req->opType = (data >> 4) & 0xF;
                        req->inputA = regInputA;
                        req->inputB = regInputB;
                        
                        requestQueue.push(req);
                        requestComplete[regCoreId] = false;
                        stats.totalRequests++;
                        stats.perCoreRequests[regCoreId]++;
                        
                        if (requestQueue.size() > 1 || currentRequest != nullptr) {
                            stats.contentionEvents++;
                        }
                        
                        if (!processEvent.scheduled() && currentRequest == nullptr) {
                            schedule(processEvent,
                                    curTick() + clockPeriod() * arbitrationLatency);
                        }
                        
                        DPRINTF(SharedCORDICAccel, "Request from Core %lu, op=%d\n",
                                regCoreId, req->opType);
                    }
                    regCtrl = 0;
                }
                break;
            case REG_CORE_ID:
                regCoreId = data;
                break;
            case REG_INPUT_A:
                memcpy(&regInputA, &data, sizeof(double));
                break;
            case REG_INPUT_B:
                memcpy(&regInputB, &data, sizeof(double));
                break;
        }
    }
    
    return clockPeriod();
}

void
SharedCORDICAccel::processQueue()
{
    if (requestQueue.empty() || currentRequest != nullptr) return;
    
    currentRequest = requestQueue.front();
    requestQueue.pop();
    currentRequest->startTime = curTick();
    
    Tick waitTime = currentRequest->startTime - currentRequest->arrivalTime;
    waitCycles[currentRequest->coreId] += waitTime / clockPeriod();
    stats.perCoreWaitCycles[currentRequest->coreId] += waitTime / clockPeriod();
    stats.totalWaitCycles += waitTime / clockPeriod();
    
    regStatus |= STATUS_BUSY;
    
    schedule(completeEvent, curTick() + clockPeriod() * computeLatency);
}

void
SharedCORDICAccel::completeOperation()
{
    if (currentRequest == nullptr) return;
    
    int coreId = currentRequest->coreId;
    
    /* Compute actual result */
    switch (currentRequest->opType) {
        case 0: /* SIN */
            resultBuffers[coreId] = std::sin(currentRequest->inputA);
            break;
        case 1: /* COS */
            resultBuffers[coreId] = std::cos(currentRequest->inputA);
            break;
        case 2: /* TAN */
            resultBuffers[coreId] = std::tan(currentRequest->inputA);
            break;
        case 3: /* ATAN2 */
            resultBuffers[coreId] = std::atan2(currentRequest->inputA,
                                                currentRequest->inputB);
            break;
    }
    
    requestComplete[coreId] = true;
    stats.completedOperations++;
    
    regStatus &= ~STATUS_BUSY;
    regStatus |= STATUS_DONE;
    
    delete currentRequest;
    currentRequest = nullptr;
    
    if (!requestQueue.empty()) {
        schedule(processEvent, curTick() + clockPeriod() * arbitrationLatency);
    }
}

SharedCORDICAccel::SharedCORDICStats::SharedCORDICStats(SharedCORDICAccel *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(), "Total requests"),
      ADD_STAT(completedOperations, statistics::units::Count::get(), "Completed ops"),
      ADD_STAT(contentionEvents, statistics::units::Count::get(), "Contention events"),
      ADD_STAT(totalWaitCycles, statistics::units::Cycle::get(), "Total wait cycles"),
      ADD_STAT(perCoreRequests, statistics::units::Count::get(), "Per-core requests"),
      ADD_STAT(perCoreWaitCycles, statistics::units::Cycle::get(), "Per-core wait")
{
    perCoreRequests.init(parent->numCores);
    perCoreWaitCycles.init(parent->numCores);
}

} // namespace gem5
