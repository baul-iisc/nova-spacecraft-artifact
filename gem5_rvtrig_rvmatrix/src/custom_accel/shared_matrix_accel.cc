/*
 * Shared 3x3 Matrix Accelerator Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/shared_matrix_accel.hh"
#include "base/trace.hh"
#include "debug/SharedMatrixAccel.hh"
#include "sim/system.hh"

namespace gem5
{

SharedMatrixAccel::SharedMatrixAccel(const Params &p)
    : ClockedObject(p),
      numCores(p.num_cores),
      computeLatency(p.compute_latency),
      arbitrationLatency(p.arbitration_latency),
      mmioBase(p.mmio_base),
      mmioSize(p.mmio_size),
      currentRequest(nullptr),
      regCtrl(0), regStatus(0), regCoreId(0),
      regSrcA(0), regSrcB(0), regDst(0),
      mmioPort(name() + ".mmio_port", this),
      processEvent([this]{ processQueue(); }, name() + ".processEvent"),
      completeEvent([this]{ completeOperation(); }, name() + ".completeEvent"),
      stats(this)
{
    requestComplete.resize(numCores, false);
    waitCycles.resize(numCores, 0);
    resultBuffers.resize(numCores);
    
    for (int i = 0; i < numCores; i++) {
        resultBuffers[i].fill(0.0);
    }
    
    DPRINTF(SharedMatrixAccel, "SharedMatrixAccel created: %d cores, "
            "MMIO base=0x%lx, size=0x%lx\n", numCores, mmioBase, mmioSize);
}

SharedMatrixAccel::~SharedMatrixAccel()
{
    while (!requestQueue.empty()) {
        delete requestQueue.front();
        requestQueue.pop();
    }
    if (currentRequest) delete currentRequest;
}

Port &
SharedMatrixAccel::getPort(const std::string &name, PortID idx)
{
    if (name == "mmio_port")
        return mmioPort;
    return ClockedObject::getPort(name, idx);
}

AddrRangeList
SharedMatrixAccel::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(mmioBase, mmioBase + mmioSize));
    return ranges;
}

Tick
SharedMatrixAccel::handleMMIO(PacketPtr pkt)
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
            case REG_WAIT_CYCLES:
                if (regCoreId < (uint64_t)numCores)
                    data = waitCycles[regCoreId];
                break;
            default:
                /* Result buffer read */
                if (offset >= REG_RESULT_BASE && offset < REG_RESULT_BASE + 72) {
                    int idx = (offset - REG_RESULT_BASE) / 8;
                    if (regCoreId < (uint64_t)numCores && idx < 9) {
                        double val = resultBuffers[regCoreId][idx];
                        memcpy(&data, &val, sizeof(double));
                    }
                }
                break;
        }
        
        pkt->setData((uint8_t*)&data);
        DPRINTF(SharedMatrixAccel, "MMIO Read: offset=0x%lx, data=0x%lx\n", 
                offset, data);
    }
    else if (pkt->isWrite()) {
        uint64_t data;
        pkt->writeData((uint8_t*)&data);
        
        switch (offset) {
            case REG_CTRL:
                regCtrl = data;
                if (data & CTRL_START) {
                    /* Submit new request */
                    if (regCoreId < (uint64_t)numCores) {
                        MatrixRequest* req = new MatrixRequest();
                        req->coreId = regCoreId;
                        req->arrivalTime = curTick();
                        req->srcA = regSrcA;
                        req->srcB = regSrcB;
                        req->dst = regDst;
                        
                        requestQueue.push(req);
                        requestComplete[regCoreId] = false;
                        stats.totalRequests++;
                        stats.perCoreRequests[regCoreId]++;
                        
                        if (requestQueue.size() > 1 || currentRequest != nullptr) {
                            stats.contentionEvents++;
                            DPRINTF(SharedMatrixAccel, "Contention! Core %lu queued. "
                                    "Queue depth: %lu\n", regCoreId, requestQueue.size());
                        }
                        
                        if (!processEvent.scheduled() && currentRequest == nullptr) {
                            schedule(processEvent, 
                                    curTick() + clockPeriod() * arbitrationLatency);
                        }
                        
                        DPRINTF(SharedMatrixAccel, "Request from Core %lu. "
                                "Queue depth: %lu\n", regCoreId, requestQueue.size());
                    }
                    regCtrl = 0;  /* Clear start bit */
                }
                break;
            case REG_CORE_ID:
                regCoreId = data;
                break;
            case REG_SRC_A:
                regSrcA = data;
                break;
            case REG_SRC_B:
                regSrcB = data;
                break;
            case REG_DST:
                regDst = data;
                break;
        }
        
        DPRINTF(SharedMatrixAccel, "MMIO Write: offset=0x%lx, data=0x%lx\n",
                offset, data);
    }
    
    return clockPeriod();
}

void
SharedMatrixAccel::processQueue()
{
    if (requestQueue.empty() || currentRequest != nullptr) {
        return;
    }
    
    currentRequest = requestQueue.front();
    requestQueue.pop();
    currentRequest->startTime = curTick();
    
    Tick waitTime = currentRequest->startTime - currentRequest->arrivalTime;
    waitCycles[currentRequest->coreId] += waitTime / clockPeriod();
    stats.perCoreWaitCycles[currentRequest->coreId] += waitTime / clockPeriod();
    stats.totalWaitCycles += waitTime / clockPeriod();
    
    regStatus |= STATUS_BUSY;
    
    DPRINTF(SharedMatrixAccel, "Processing Core %d request. Wait: %lu cycles\n",
            currentRequest->coreId, waitTime / clockPeriod());
    
    schedule(completeEvent, curTick() + clockPeriod() * computeLatency);
}

void
SharedMatrixAccel::completeOperation()
{
    if (currentRequest == nullptr) return;
    
    int coreId = currentRequest->coreId;
    
    /* Simulate matrix multiply (placeholder values) */
    for (int i = 0; i < 9; i++) {
        resultBuffers[coreId][i] = 1.0 + i * 0.1 + coreId * 0.01;
    }
    
    requestComplete[coreId] = true;
    stats.completedOperations++;
    
    regStatus &= ~STATUS_BUSY;
    regStatus |= STATUS_DONE;
    
    DPRINTF(SharedMatrixAccel, "Completed Core %d. Total: %lu\n",
            coreId, stats.completedOperations.value());
    
    delete currentRequest;
    currentRequest = nullptr;
    
    if (!requestQueue.empty()) {
        schedule(processEvent, curTick() + clockPeriod() * arbitrationLatency);
    }
}

SharedMatrixAccel::SharedMatrixAccelStats::SharedMatrixAccelStats(
    SharedMatrixAccel *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(),
               "Total matrix requests"),
      ADD_STAT(completedOperations, statistics::units::Count::get(),
               "Completed operations"),
      ADD_STAT(contentionEvents, statistics::units::Count::get(),
               "Contention events"),
      ADD_STAT(totalWaitCycles, statistics::units::Cycle::get(),
               "Total wait cycles"),
      ADD_STAT(maxQueueDepth, statistics::units::Count::get(),
               "Max queue depth"),
      ADD_STAT(perCoreRequests, statistics::units::Count::get(),
               "Per-core requests"),
      ADD_STAT(perCoreWaitCycles, statistics::units::Cycle::get(),
               "Per-core wait cycles")
{
    perCoreRequests.init(parent->numCores);
    perCoreWaitCycles.init(parent->numCores);
    
    for (int i = 0; i < parent->numCores; i++) {
        perCoreRequests.subname(i, std::string("core") + std::to_string(i));
        perCoreWaitCycles.subname(i, std::string("core") + std::to_string(i));
    }
}

} // namespace gem5
