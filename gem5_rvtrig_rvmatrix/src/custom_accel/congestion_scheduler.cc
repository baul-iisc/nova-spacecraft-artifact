/*
 * Congestion-Aware Scheduler Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/congestion_scheduler.hh"
#include "base/trace.hh"
#include "debug/CongestionScheduler.hh"
#include "params/ClockedObject.hh"

#include <algorithm>
#include <numeric>

namespace gem5
{

CongestionScheduler::CongestionScheduler(const Params &p)
    : ClockedObject(dynamic_cast<const ClockedObjectParams &>(p)),
      numAccelerators(p.num_accelerators),
      congestionThreshold(p.congestion_threshold),
      maxQueueDepth(p.max_queue_depth),
      starvationTimeout(p.starvation_timeout),
      enablePreemption(p.enable_preemption),
      enableDeadlineScheduling(p.enable_deadline_scheduling),
      nextRequestId(0),
      starvationCheckEvent([this]{ checkStarvation(); }, name() + ".starvCheck"),
      congestionUpdateEvent([this]{ updateCongestionState(); }, name() + ".congUpdate"),
      stats(this)
{
    /* Initialize accelerator states */
    acceleratorStates.resize(numAccelerators);
    for (int i = 0; i < numAccelerators; i++) {
        acceleratorStates[i].accelId = i;
        acceleratorStates[i].busy = false;
        acceleratorStates[i].currentRequestId = -1;
        acceleratorStates[i].busySince = 0;
    }

    /* Initialize congestion state */
    congestionState.currentUtilization = 0.0;
    congestionState.predictedUtilization = 0.0;
    congestionState.queueDepth = 0;
    congestionState.activeRequests = 0;
    congestionState.avgWaitTime = 0;
    congestionState.maxWaitTime = 0;
    congestionState.contentionScore = 1.0;  // Start perfect

    /* Initialize periodic profiles */
    periodicProfiles.resize(NUM_TASK_TYPES);
    for (int i = 0; i < NUM_TASK_TYPES; i++) {
        periodicProfiles[i].type = static_cast<TaskType>(i);
        periodicProfiles[i].period = 0;
        periodicProfiles[i].lastOccurrence = 0;
        periodicProfiles[i].occurrences = 0;
        periodicProfiles[i].avgDuration = 0;
    }

    /* Schedule periodic events */
    scheduleStarvationCheck();
    scheduleCongestionUpdate();

    DPRINTF(CongestionScheduler, "CongestionScheduler created: %d accelerators, "
            "threshold=%.2f\n", numAccelerators, congestionThreshold);
}

CongestionScheduler::~CongestionScheduler()
{
}

int
CongestionScheduler::submitRequest(int coreId, TaskType taskType,
                                   CriticalityLevel criticality,
                                   Tick deadline, int estimatedCycles,
                                   std::function<void(int)> callback)
{
    AcceleratorRequest req;
    req.requestId = nextRequestId++;
    req.coreId = coreId;
    req.taskType = taskType;
    req.criticality = criticality;
    req.submitTime = curTick();
    req.deadline = deadline;
    req.estimatedCycles = estimatedCycles;
    req.callback = callback;

    stats.totalRequests++;
    stats.requestsPerCriticality[criticality]++;
    stats.requestsPerType[taskType]++;

    /* Check queue depth limit */
    if (pendingQueue.size() >= static_cast<size_t>(maxQueueDepth)) {
        DPRINTF(CongestionScheduler, "Queue full, rejecting request %d\n",
                req.requestId);
        return -1;
    }

    /* Store and queue request */
    allRequests[req.requestId] = req;
    pendingQueue.push(req);
    congestionState.queueDepth = pendingQueue.size();

    DPRINTF(CongestionScheduler, "Submitted request %d: core=%d type=%d "
            "crit=%d deadline=%lu\n", req.requestId, coreId, taskType,
            criticality, deadline);

    /* Try to dispatch immediately */
    dispatchNext();

    return req.requestId;
}

bool
CongestionScheduler::cancelRequest(int requestId)
{
    auto it = allRequests.find(requestId);
    if (it == allRequests.end()) {
        return false;
    }

    /* Can only cancel pending requests */
    if (activeRequests.find(requestId) != activeRequests.end()) {
        DPRINTF(CongestionScheduler, "Cannot cancel active request %d\n",
                requestId);
        return false;
    }

    /* Note: Can't efficiently remove from priority_queue,
       mark as cancelled instead */
    stats.cancelledRequests++;
    allRequests.erase(it);

    DPRINTF(CongestionScheduler, "Cancelled request %d\n", requestId);
    return true;
}

CongestionScheduler::CongestionState
CongestionScheduler::getCongestionState() const
{
    return congestionState;
}

Tick
CongestionScheduler::getPredictedWaitTime(CriticalityLevel criticality)
{
    /* Estimate based on queue and criticality */
    int higherPriorityCount = 0;

    /* Count requests with higher criticality */
    for (const auto &pair : allRequests) {
        if (pair.second.criticality > criticality) {
            higherPriorityCount++;
        }
    }

    /* Average service time * position in queue */
    Tick avgServiceTime = 100 * clockPeriod();  // Placeholder
    return higherPriorityCount * avgServiceTime;
}

void
CongestionScheduler::registerAccelerator(int accelId,
    std::function<void(AcceleratorRequest&)> execFunc)
{
    if (accelId >= 0 && accelId < numAccelerators) {
        acceleratorStates[accelId].execFunc = execFunc;
        DPRINTF(CongestionScheduler, "Registered accelerator %d\n", accelId);
    }
}

void
CongestionScheduler::notifyCompletion(int requestId, int accelId)
{
    auto it = activeRequests.find(requestId);
    if (it == activeRequests.end()) {
        DPRINTF(CongestionScheduler, "Unknown completion for request %d\n",
                requestId);
        return;
    }

    AcceleratorRequest &req = it->second;
    Tick completionTime = curTick();
    Tick duration = completionTime - req.submitTime;

    /* Update statistics */
    stats.completedRequests++;
    waitTimeHistory.push_back(duration);
    if (waitTimeHistory.size() > 1000) {
        waitTimeHistory.pop_front();
    }

    /* Check deadline */
    if (req.deadline > 0 && completionTime > req.deadline) {
        stats.deadlineMisses++;
        DPRINTF(CongestionScheduler, "Deadline miss: request %d by %lu ticks\n",
                requestId, completionTime - req.deadline);
    }

    /* Update periodic profile */
    updatePeriodicProfile(req.taskType, duration);

    /* Free accelerator */
    acceleratorStates[accelId].busy = false;
    acceleratorStates[accelId].currentRequestId = -1;
    congestionState.activeRequests--;

    /* Callback */
    if (req.callback) {
        req.callback(requestId);
    }

    /* Cleanup */
    activeRequests.erase(it);
    allRequests.erase(requestId);

    DPRINTF(CongestionScheduler, "Completed request %d on accel %d, "
            "duration=%lu\n", requestId, accelId, duration);

    /* Dispatch next */
    dispatchNext();
}

bool
CongestionScheduler::isCongested() const
{
    return congestionState.currentUtilization > congestionThreshold;
}

double
CongestionScheduler::getReliabilityScore() const
{
    /* Contention affects reliability:
       - High queue depth = lower reliability
       - Deadline misses = lower reliability
       - High wait times = lower reliability */

    double score = 1.0;

    /* Queue depth penalty */
    double queuePenalty = std::min(0.3, 
        congestionState.queueDepth / (double)maxQueueDepth * 0.3);
    score -= queuePenalty;

    /* Utilization penalty */
    double utilPenalty = std::max(0.0, 
        (congestionState.currentUtilization - congestionThreshold) * 0.5);
    score -= utilPenalty;

    /* Deadline miss rate penalty */
    if (stats.totalRequests.value() > 0) {
        double missRate = stats.deadlineMisses.value() / 
                         stats.totalRequests.value();
        score -= missRate * 0.5;
    }

    return std::max(0.0, std::min(1.0, score));
}

void
CongestionScheduler::dispatchNext()
{
    while (!pendingQueue.empty()) {
        /* Find available accelerator */
        int accelId = findAvailableAccelerator();
        if (accelId < 0) {
            DPRINTF(CongestionScheduler, "No accelerators available\n");
            return;
        }

        /* Get highest priority request */
        AcceleratorRequest req = pendingQueue.top();
        pendingQueue.pop();

        /* Check if request was cancelled */
        if (allRequests.find(req.requestId) == allRequests.end()) {
            continue;  // Skip cancelled requests
        }

        /* Dispatch */
        acceleratorStates[accelId].busy = true;
        acceleratorStates[accelId].currentRequestId = req.requestId;
        acceleratorStates[accelId].busySince = curTick();
        activeRequests[req.requestId] = req;
        congestionState.activeRequests++;
        congestionState.queueDepth = pendingQueue.size();

        DPRINTF(CongestionScheduler, "Dispatched request %d to accel %d\n",
                req.requestId, accelId);

        /* Execute on accelerator */
        if (acceleratorStates[accelId].execFunc) {
            acceleratorStates[accelId].execFunc(req);
        }
    }
}

int
CongestionScheduler::findAvailableAccelerator()
{
    for (int i = 0; i < numAccelerators; i++) {
        if (!acceleratorStates[i].busy) {
            return i;
        }
    }
    return -1;
}

void
CongestionScheduler::updateCongestionState()
{
    /* Calculate utilization */
    int busyCount = 0;
    for (const auto &state : acceleratorStates) {
        if (state.busy) {
            busyCount++;
        }
    }
    double util = (numAccelerators > 0) ? 
                  (double)busyCount / numAccelerators : 0.0;

    /* Update history */
    utilizationHistory.push_back(util);
    if (utilizationHistory.size() > 100) {
        utilizationHistory.pop_front();
    }

    /* Current and predicted utilization */
    congestionState.currentUtilization = util;
    congestionState.predictedUtilization = std::accumulate(
        utilizationHistory.begin(), utilizationHistory.end(), 0.0) /
        utilizationHistory.size();

    /* Wait time stats */
    if (!waitTimeHistory.empty()) {
        congestionState.avgWaitTime = std::accumulate(
            waitTimeHistory.begin(), waitTimeHistory.end(), Tick(0)) /
            waitTimeHistory.size();
        congestionState.maxWaitTime = *std::max_element(
            waitTimeHistory.begin(), waitTimeHistory.end());
    }

    /* Reliability score */
    congestionState.contentionScore = getReliabilityScore();

    /* Update stats */
    stats.avgUtilization = congestionState.currentUtilization;
    stats.avgQueueDepth = congestionState.queueDepth;
    stats.reliabilityScore = congestionState.contentionScore;
    stats.waitTimeDistribution.sample(congestionState.avgWaitTime);

    DPRINTF(CongestionScheduler, "Congestion update: util=%.2f queue=%d "
            "reliability=%.2f\n", congestionState.currentUtilization,
            congestionState.queueDepth, congestionState.contentionScore);

    scheduleCongestionUpdate();
}

void
CongestionScheduler::checkStarvation()
{
    Tick now = curTick();

    for (const auto &pair : allRequests) {
        const AcceleratorRequest &req = pair.second;

        /* Skip active requests */
        if (activeRequests.find(req.requestId) != activeRequests.end()) {
            continue;
        }

        /* Check if waiting too long */
        Tick waitTime = now - req.submitTime;
        if (waitTime > starvationTimeout * clockPeriod()) {
            stats.starvationEvents++;
            DPRINTF(CongestionScheduler, "Starvation detected: request %d "
                    "waiting %lu ticks\n", req.requestId, waitTime);
            
            /* Could implement priority boost here */
        }
    }

    scheduleStarvationCheck();
}

void
CongestionScheduler::updatePeriodicProfile(TaskType type, Tick duration)
{
    PeriodicTaskProfile &profile = periodicProfiles[type];
    Tick now = curTick();

    if (profile.lastOccurrence > 0) {
        Tick interval = now - profile.lastOccurrence;
        
        /* Update period estimate (exponential moving average) */
        if (profile.period == 0) {
            profile.period = interval;
        } else {
            profile.period = (profile.period * 7 + interval) / 8;
        }
    }

    profile.lastOccurrence = now;
    profile.occurrences++;

    /* Update duration estimate */
    if (profile.avgDuration == 0) {
        profile.avgDuration = duration;
    } else {
        profile.avgDuration = (profile.avgDuration * 7 + duration) / 8;
    }
}

Tick
CongestionScheduler::predictNextOccurrence(TaskType type)
{
    const PeriodicTaskProfile &profile = periodicProfiles[type];

    if (profile.period == 0 || profile.occurrences < 2) {
        return 0;  // Not enough data
    }

    return profile.lastOccurrence + profile.period;
}

void
CongestionScheduler::scheduleStarvationCheck()
{
    if (!starvationCheckEvent.scheduled()) {
        schedule(starvationCheckEvent, clockEdge(Cycles(1000)));
    }
}

void
CongestionScheduler::scheduleCongestionUpdate()
{
    if (!congestionUpdateEvent.scheduled()) {
        schedule(congestionUpdateEvent, clockEdge(Cycles(100)));
    }
}

CongestionScheduler::CongestionSchedulerStats::CongestionSchedulerStats(
    CongestionScheduler *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(),
               "Total requests submitted"),
      ADD_STAT(completedRequests, statistics::units::Count::get(),
               "Completed requests"),
      ADD_STAT(cancelledRequests, statistics::units::Count::get(),
               "Cancelled requests"),
      ADD_STAT(preemptedRequests, statistics::units::Count::get(),
               "Preempted requests"),
      ADD_STAT(deadlineMisses, statistics::units::Count::get(),
               "Deadline misses"),
      ADD_STAT(starvationEvents, statistics::units::Count::get(),
               "Starvation events"),
      ADD_STAT(requestsPerCriticality, statistics::units::Count::get(),
               "Requests per criticality level"),
      ADD_STAT(requestsPerType, statistics::units::Count::get(),
               "Requests per task type"),
      ADD_STAT(waitTimeDistribution, statistics::units::Tick::get(),
               "Wait time distribution"),
      ADD_STAT(avgQueueDepth, statistics::units::Count::get(),
               "Average queue depth"),
      ADD_STAT(maxQueueDepth, statistics::units::Count::get(),
               "Maximum queue depth"),
      ADD_STAT(avgUtilization, statistics::units::Ratio::get(),
               "Average accelerator utilization"),
      ADD_STAT(reliabilityScore, statistics::units::Ratio::get(),
               "System reliability score")
{
    requestsPerCriticality.init(5);  // 5 criticality levels
    requestsPerType.init(NUM_TASK_TYPES);
    waitTimeDistribution.init(20);

    const char* critNames[] = {"background", "science", "telemetry", 
                               "navigation", "emergency"};
    for (int i = 0; i < 5; i++) {
        requestsPerCriticality.subname(i, critNames[i]);
    }

    const char* typeNames[] = {"matrix_mult", "matrix_inv", "kalman",
                               "attitude", "trig", "fft", "image", "compress"};
    for (int i = 0; i < NUM_TASK_TYPES; i++) {
        requestsPerType.subname(i, typeNames[i]);
    }
}

} // namespace gem5

