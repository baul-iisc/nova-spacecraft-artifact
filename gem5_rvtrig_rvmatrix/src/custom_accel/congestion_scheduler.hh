/*
 * Congestion-Aware Scheduler for Space Workloads
 * PhD Research: Chandraboul
 * 
 * Research Question 2: Congestion-Aware Scheduling
 * - Priority-based accelerator arbitration respecting criticality levels
 * - Predictive congestion modeling for periodic spacecraft tasks
 * - Fault-tolerance mechanisms where contention is a reliability metric
 */

#ifndef __CUSTOM_ACCEL_CONGESTION_SCHEDULER_HH__
#define __CUSTOM_ACCEL_CONGESTION_SCHEDULER_HH__

#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/CongestionScheduler.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <vector>
#include <queue>
#include <map>
#include <functional>

namespace gem5
{

/**
 * CongestionScheduler - Priority-based accelerator arbitration
 * 
 * Features:
 * 1. Criticality-aware priority (Navigation > Telemetry > Science)
 * 2. Deadline-based scheduling for real-time tasks
 * 3. Predictive congestion modeling
 * 4. Starvation prevention
 * 5. Contention as reliability metric
 */
class CongestionScheduler : public ClockedObject
{
  public:
    PARAMS(CongestionScheduler);
    CongestionScheduler(const Params &p);
    ~CongestionScheduler();

    /* Task criticality levels (higher = more critical) */
    enum CriticalityLevel {
        CRIT_BACKGROUND = 0,    // Low priority, can be preempted
        CRIT_SCIENCE = 1,       // Science data processing
        CRIT_TELEMETRY = 2,     // Telemetry and housekeeping
        CRIT_NAVIGATION = 3,    // Navigation and attitude control
        CRIT_EMERGENCY = 4      // Emergency/safe mode operations
    };

    /* Task types */
    enum TaskType {
        TASK_MATRIX_MULT,
        TASK_MATRIX_INV,
        TASK_KALMAN_UPDATE,
        TASK_ATTITUDE_DET,
        TASK_TRIG_COMP,
        TASK_FFT,
        TASK_IMAGE_PROC,
        TASK_COMPRESSION,
        NUM_TASK_TYPES
    };

    /* Accelerator request */
    struct AcceleratorRequest {
        int requestId;
        int coreId;
        TaskType taskType;
        CriticalityLevel criticality;
        Tick submitTime;
        Tick deadline;           // 0 = no deadline
        int estimatedCycles;
        void* context;           // Caller context
        std::function<void(int)> callback;  // Completion callback

        /* Priority comparison for queue */
        bool operator<(const AcceleratorRequest &other) const {
            /* Higher criticality = higher priority */
            if (criticality != other.criticality) {
                return criticality < other.criticality;
            }
            /* Earlier deadline = higher priority */
            if (deadline != other.deadline && deadline > 0 && other.deadline > 0) {
                return deadline > other.deadline;
            }
            /* Earlier submission = higher priority */
            return submitTime > other.submitTime;
        }
    };

    /* Congestion state */
    struct CongestionState {
        double currentUtilization;     // 0.0 - 1.0
        double predictedUtilization;   // Next period
        int queueDepth;
        int activeRequests;
        Tick avgWaitTime;
        Tick maxWaitTime;
        double contentionScore;        // Reliability metric
    };

    /* Submit accelerator request */
    int submitRequest(int coreId, TaskType taskType, 
                     CriticalityLevel criticality,
                     Tick deadline, int estimatedCycles,
                     std::function<void(int)> callback);

    /* Cancel pending request */
    bool cancelRequest(int requestId);

    /* Get congestion state */
    CongestionState getCongestionState() const;

    /* Get predicted wait time for new request */
    Tick getPredictedWaitTime(CriticalityLevel criticality);

    /* Register accelerator availability */
    void registerAccelerator(int accelId, 
                            std::function<void(AcceleratorRequest&)> execFunc);

    /* Notify request completion */
    void notifyCompletion(int requestId, int accelId);

    /* Check if system is congested */
    bool isCongested() const;

    /* Get contention reliability score (0.0 = bad, 1.0 = good) */
    double getReliabilityScore() const;

  private:
    /* Parameters */
    int numAccelerators;
    double congestionThreshold;
    Cycles maxQueueDepth;
    Cycles starvationTimeout;
    bool enablePreemption;
    bool enableDeadlineScheduling;

    /* Request management */
    int nextRequestId;
    std::priority_queue<AcceleratorRequest> pendingQueue;
    std::map<int, AcceleratorRequest> activeRequests;
    std::map<int, AcceleratorRequest> allRequests;

    /* Accelerator state */
    struct AcceleratorState {
        int accelId;
        bool busy;
        int currentRequestId;
        Tick busySince;
        std::function<void(AcceleratorRequest&)> execFunc;
    };
    std::vector<AcceleratorState> acceleratorStates;

    /* Congestion tracking */
    CongestionState congestionState;
    std::deque<double> utilizationHistory;
    std::deque<Tick> waitTimeHistory;

    /* Periodic task modeling */
    struct PeriodicTaskProfile {
        TaskType type;
        Tick period;
        Tick lastOccurrence;
        int occurrences;
        double avgDuration;
    };
    std::vector<PeriodicTaskProfile> periodicProfiles;

    /* Scheduling logic */
    void dispatchNext();
    int findAvailableAccelerator();
    void updateCongestionState();
    void checkStarvation();
    void updatePeriodicProfile(TaskType type, Tick duration);
    Tick predictNextOccurrence(TaskType type);

    /* Events */
    EventFunctionWrapper starvationCheckEvent;
    EventFunctionWrapper congestionUpdateEvent;
    void scheduleStarvationCheck();
    void scheduleCongestionUpdate();

    /* Statistics */
    struct CongestionSchedulerStats : public statistics::Group
    {
        CongestionSchedulerStats(CongestionScheduler *parent);

        statistics::Scalar totalRequests;
        statistics::Scalar completedRequests;
        statistics::Scalar cancelledRequests;
        statistics::Scalar preemptedRequests;
        statistics::Scalar deadlineMisses;
        statistics::Scalar starvationEvents;
        statistics::Vector requestsPerCriticality;
        statistics::Vector requestsPerType;
        statistics::Histogram waitTimeDistribution;
        statistics::Scalar avgQueueDepth;
        statistics::Scalar maxQueueDepth;
        statistics::Scalar avgUtilization;
        statistics::Scalar reliabilityScore;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_CONGESTION_SCHEDULER_HH__

