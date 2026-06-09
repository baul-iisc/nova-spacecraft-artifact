/*
 * NOVA Processor - Shared Functional Unit Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "cpu/shared_fu/shared_functional_unit.hh"

#include "base/trace.hh"
#include "debug/SharedFU.hh"

namespace gem5
{

// ============================================================================
// SharedFunctionalUnit Implementation
// ============================================================================

SharedFunctionalUnit::SharedFunctionalUnit(const Params &p)
    : ClockedObject(p),
      accelType(static_cast<AccelType>(p.accel_type)),
      instanceId(p.instance_id),
      defaultLatency(p.default_latency),
      maxQueueDepth(p.max_queue_depth),
      busy(false),
      currentOwnerCpuId(-1),
      currentOwnerThreadId(-1),
      completionEvent([this]{ handleCompletion(); }, name()),
      stats(this)
{
    DPRINTF(SharedFU, "SharedFunctionalUnit created: type=%d, id=%d, "
            "latency=%d cycles\n", static_cast<int>(accelType), 
            instanceId, defaultLatency);
    
    // Set default latencies based on accelerator type
    switch (accelType) {
      case AccelType::TRIG_ACCEL:
        opLatencies[static_cast<int>(TrigOp::SIN)] = Cycles(15);
        opLatencies[static_cast<int>(TrigOp::COS)] = Cycles(15);
        opLatencies[static_cast<int>(TrigOp::TAN)] = Cycles(20);
        opLatencies[static_cast<int>(TrigOp::ATAN)] = Cycles(18);
        opLatencies[static_cast<int>(TrigOp::ATAN2)] = Cycles(22);
        opLatencies[static_cast<int>(TrigOp::SQRT)] = Cycles(12);
        opLatencies[static_cast<int>(TrigOp::SINCOS)] = Cycles(18);
        break;
        
      case AccelType::MAT_ACCEL:
        opLatencies[static_cast<int>(MatOp::MUL3x3)] = Cycles(50);
        opLatencies[static_cast<int>(MatOp::TRANSPOSE)] = Cycles(9);
        opLatencies[static_cast<int>(MatOp::INVERSE)] = Cycles(100);
        opLatencies[static_cast<int>(MatOp::MATVEC)] = Cycles(27);
        opLatencies[static_cast<int>(MatOp::ADD)] = Cycles(9);
        opLatencies[static_cast<int>(MatOp::SCALE)] = Cycles(9);
        break;
        
      case AccelType::VPU:
        // VPU operations are much longer (image processing)
        break;
        
      case AccelType::NPU:
        // NPU operations depend on layer size
        break;
    }
}

SharedFunctionalUnit::~SharedFunctionalUnit()
{
}

void
SharedFunctionalUnit::init()
{
    ClockedObject::init();
    
    // Initialize per-CPU statistics vectors
    // This requires knowing the number of CPUs, which we'll set to a max
    stats.requestsPerCpu.init(16);  // Support up to 16 CPUs
    stats.stallsPerCpu.init(16);
}

bool
SharedFunctionalUnit::requestAccess(int cpuId, int threadId, 
                                    AccelType reqAccelType, int opType,
                                    std::function<void()> callback)
{
    stats.totalRequests++;
    
    // Record queue depth
    stats.queueDepthDist.sample(waitQueue.size());
    if (waitQueue.size() > static_cast<size_t>(stats.maxQueueDepthSeen.value())) {
        stats.maxQueueDepthSeen = waitQueue.size();
    }
    
    if (cpuId < 16) {
        stats.requestsPerCpu[cpuId]++;
    }
    
    DPRINTF(SharedFU, "Request from CPU%d thread%d: type=%d op=%d, busy=%d, "
            "queue=%d\n", cpuId, threadId, static_cast<int>(reqAccelType), 
            opType, busy, waitQueue.size());
    
    if (!busy) {
        // FU is free - start processing immediately
        busy = true;
        currentOwnerCpuId = cpuId;
        currentOwnerThreadId = threadId;
        
        Cycles latency = getOpLatency(opType);
        
        currentRequest = FURequest{
            cpuId, threadId, reqAccelType, opType,
            curTick(), latency, callback
        };
        
        // Schedule completion
        schedule(completionEvent, clockEdge(latency));
        
        stats.acceptedRequests++;
        stats.totalBusyCycles += latency;
        
        DPRINTF(SharedFU, "  Started immediately, latency=%d cycles\n", latency);
        
        return true;  // Request accepted
        
    } else {
        // FU is busy - queue the request
        if (waitQueue.size() < maxQueueDepth) {
            Cycles latency = getOpLatency(opType);
            
            FURequest req{
                cpuId, threadId, reqAccelType, opType,
                curTick(), latency, callback
            };
            waitQueue.push(req);
            
            stats.queuedRequests++;
            if (cpuId < 16) {
                stats.stallsPerCpu[cpuId]++;
            }
            
            DPRINTF(SharedFU, "  Queued (queue depth now %d)\n", 
                    waitQueue.size());
            
            return false;  // Request queued, caller should stall
            
        } else {
            DPRINTF(SharedFU, "  Queue full! Request rejected\n");
            return false;
        }
    }
}

void
SharedFunctionalUnit::handleCompletion()
{
    DPRINTF(SharedFU, "Operation complete for CPU%d\n", currentOwnerCpuId);
    
    // Record latency
    Tick totalLatency = curTick() - currentRequest.requestTime;
    stats.requestLatencyHist.sample(ticksToCycles(totalLatency));
    
    // Call completion callback
    if (currentRequest.callback) {
        currentRequest.callback();
    }
    
    // Release and process next
    release();
}

void
SharedFunctionalUnit::release()
{
    busy = false;
    currentOwnerCpuId = -1;
    currentOwnerThreadId = -1;
    
    // Process next request if queue is not empty
    if (!waitQueue.empty()) {
        processNextRequest();
    }
}

void
SharedFunctionalUnit::processNextRequest()
{
    FURequest req = waitQueue.front();
    waitQueue.pop();
    
    // Calculate wait time
    Tick waitTime = curTick() - req.requestTime;
    stats.totalQueueWaitCycles += ticksToCycles(waitTime);
    
    DPRINTF(SharedFU, "Processing queued request from CPU%d, waited %d cycles\n",
            req.cpuId, ticksToCycles(waitTime));
    
    busy = true;
    currentOwnerCpuId = req.cpuId;
    currentOwnerThreadId = req.threadId;
    currentRequest = req;
    
    // Schedule completion
    schedule(completionEvent, clockEdge(req.latency));
    
    stats.totalBusyCycles += req.latency;
}

Cycles
SharedFunctionalUnit::getOpLatency(int opType) const
{
    auto it = opLatencies.find(opType);
    if (it != opLatencies.end()) {
        return it->second;
    }
    return defaultLatency;
}

void
SharedFunctionalUnit::setOpLatency(int opType, Cycles latency)
{
    opLatencies[opType] = latency;
}

// Statistics
SharedFunctionalUnit::SharedFUStats::SharedFUStats(SharedFunctionalUnit *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(),
               "Total requests received"),
      ADD_STAT(acceptedRequests, statistics::units::Count::get(),
               "Requests processed immediately (no stall)"),
      ADD_STAT(queuedRequests, statistics::units::Count::get(),
               "Requests that had to wait in queue"),
      ADD_STAT(totalBusyCycles, statistics::units::Cycle::get(),
               "Total cycles the FU was busy"),
      ADD_STAT(totalQueueWaitCycles, statistics::units::Cycle::get(),
               "Total cycles requests spent waiting in queue"),
      ADD_STAT(maxQueueDepthSeen, statistics::units::Count::get(),
               "Maximum queue depth observed"),
      ADD_STAT(queueDepthDist, statistics::units::Count::get(),
               "Distribution of queue depth at request time"),
      ADD_STAT(requestLatencyHist, statistics::units::Cycle::get(),
               "Histogram of total request latency"),
      ADD_STAT(requestsPerCpu, statistics::units::Count::get(),
               "Requests per CPU"),
      ADD_STAT(stallsPerCpu, statistics::units::Count::get(),
               "Stalls per CPU (due to busy FU)")
{
    queueDepthDist
        .init(0, 16, 1)
        .flags(statistics::nozero);
        
    requestLatencyHist
        .init(20)
        .flags(statistics::nozero);
}


// ============================================================================
// SharedAcceleratorPool Implementation
// ============================================================================

SharedAcceleratorPool::SharedAcceleratorPool(const Params &p)
    : SimObject(p),
      sharingMode(static_cast<SharingMode>(p.sharing_mode)),
      numCpus(p.num_cpus),
      stats(this)
{
    DPRINTF(SharedFU, "SharedAcceleratorPool created: mode=%d, cpus=%d\n",
            static_cast<int>(sharingMode), numCpus);
}

SharedAcceleratorPool::~SharedAcceleratorPool()
{
}

void
SharedAcceleratorPool::init()
{
    SimObject::init();
}

void
SharedAcceleratorPool::registerFU(SharedFunctionalUnit* fu)
{
    SharedFunctionalUnit::AccelType type = fu->getAccelType();
    fusByType[type].push_back(fu);
    
    DPRINTF(SharedFU, "Registered FU type=%d, now have %d of this type\n",
            static_cast<int>(type), fusByType[type].size());
}

void
SharedAcceleratorPool::assignDedicatedFU(int cpuId, SharedFunctionalUnit* fu)
{
    dedicatedFUs[cpuId].push_back(fu);
    
    DPRINTF(SharedFU, "Assigned dedicated FU to CPU%d\n", cpuId);
}

SharedFunctionalUnit*
SharedAcceleratorPool::requestFU(int cpuId, int threadId,
                                 SharedFunctionalUnit::AccelType accelType,
                                 int opType,
                                 std::function<void()> callback)
{
    stats.totalRequests++;
    
    SharedFunctionalUnit* fu = selectFU(cpuId, accelType);
    
    if (!fu) {
        DPRINTF(SharedFU, "No FU available for CPU%d type=%d\n",
                cpuId, static_cast<int>(accelType));
        return nullptr;
    }
    
    bool accepted = fu->requestAccess(cpuId, threadId, accelType, opType, callback);
    
    if (accepted) {
        stats.successfulRequests++;
    } else {
        stats.stalledRequests++;
    }
    
    return fu;
}

SharedFunctionalUnit*
SharedAcceleratorPool::selectFU(int cpuId, 
                                SharedFunctionalUnit::AccelType accelType)
{
    switch (sharingMode) {
      case SharingMode::FULLY_DEDICATED:
        // Return dedicated FU for this CPU
        if (dedicatedFUs.count(cpuId)) {
            for (auto* fu : dedicatedFUs[cpuId]) {
                if (fu->getAccelType() == accelType) {
                    return fu;
                }
            }
        }
        return nullptr;
        
      case SharingMode::FULLY_SHARED:
        // Select least-loaded FU of the requested type
        {
            auto it = fusByType.find(accelType);
            if (it == fusByType.end() || it->second.empty()) {
                return nullptr;
            }
            
            // Find FU with shortest queue
            SharedFunctionalUnit* bestFU = nullptr;
            size_t minQueue = SIZE_MAX;
            
            for (auto* fu : it->second) {
                size_t qd = fu->getQueueDepth();
                if (!fu->isBusy()) {
                    return fu;  // Found a free one
                }
                if (qd < minQueue) {
                    minQueue = qd;
                    bestFU = fu;
                }
            }
            
            return bestFU;
        }
        
      case SharingMode::HYBRID:
        // First check dedicated, then shared
        if (dedicatedFUs.count(cpuId)) {
            for (auto* fu : dedicatedFUs[cpuId]) {
                if (fu->getAccelType() == accelType) {
                    return fu;
                }
            }
        }
        // Fall back to shared
        return selectFU(cpuId, accelType);  // Will hit FULLY_SHARED case
        
      case SharingMode::ADAPTIVE:
        // Use dedicated if contention is high, shared otherwise
        // For now, behave like FULLY_SHARED
        return selectFU(cpuId, accelType);
        
      default:
        return nullptr;
    }
}

bool
SharedAcceleratorPool::isAvailable(int cpuId, 
                                   SharedFunctionalUnit::AccelType accelType) const
{
    auto it = fusByType.find(accelType);
    if (it == fusByType.end()) {
        return false;
    }
    
    for (const auto* fu : it->second) {
        if (!fu->isBusy()) {
            return true;
        }
    }
    
    return false;
}

size_t
SharedAcceleratorPool::getTotalQueueDepth(
    SharedFunctionalUnit::AccelType accelType) const
{
    size_t total = 0;
    
    auto it = fusByType.find(accelType);
    if (it != fusByType.end()) {
        for (const auto* fu : it->second) {
            total += fu->getQueueDepth();
        }
    }
    
    return total;
}

void
SharedAcceleratorPool::setSharingMode(SharingMode mode)
{
    sharingMode = mode;
    DPRINTF(SharedFU, "Sharing mode changed to %d\n", static_cast<int>(mode));
}

// Statistics
SharedAcceleratorPool::PoolStats::PoolStats(SharedAcceleratorPool *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(),
               "Total FU requests"),
      ADD_STAT(successfulRequests, statistics::units::Count::get(),
               "Requests that got FU immediately"),
      ADD_STAT(stalledRequests, statistics::units::Count::get(),
               "Requests that caused stalls")
{
}

} // namespace gem5


