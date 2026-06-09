/*
 * Adaptive Accelerator Allocation System
 * PhD Research: Chandraboul
 * 
 * Research Question 1: Adaptive Accelerator Allocation Strategies
 * - Dynamically switch between sharing modes based on workload
 * - Predict optimal allocation based on real-time profiling
 * - Mission phase aware allocation (launch, orbit, science)
 */

#ifndef __CUSTOM_ACCEL_ADAPTIVE_ALLOCATOR_HH__
#define __CUSTOM_ACCEL_ADAPTIVE_ALLOCATOR_HH__

#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/AdaptiveAllocator.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <vector>
#include <deque>
#include <map>
#include <functional>

namespace gem5
{

/**
 * AdaptiveAllocator - Dynamic accelerator allocation manager
 * 
 * Features:
 * 1. Runtime switching between shared/dedicated modes
 * 2. Workload profiling and prediction
 * 3. Mission phase awareness
 * 4. Contention monitoring and adaptation
 */
class AdaptiveAllocator : public ClockedObject
{
  public:
    PARAMS(AdaptiveAllocator);
    AdaptiveAllocator(const Params &p);
    ~AdaptiveAllocator();

    /* Allocation modes */
    enum AllocationMode {
        MODE_SHARED,           // All cores share accelerators
        MODE_DEDICATED,        // Each core has dedicated accelerator
        MODE_HYBRID,           // Some shared, some dedicated
        MODE_ADAPTIVE          // Runtime switching
    };

    /* Mission phases (affects allocation strategy) */
    enum MissionPhase {
        PHASE_LAUNCH,          // Critical, low contention
        PHASE_ORBIT_INSERT,    // Critical, medium contention
        PHASE_CRUISE,          // Non-critical, low activity
        PHASE_SCIENCE_OPS,     // High throughput needed
        PHASE_EMERGENCY        // Maximum reliability
    };

    /* Accelerator types */
    enum AcceleratorType {
        ACCEL_MATRIX,
        ACCEL_CORDIC,
        ACCEL_COMPRESSION,
        ACCEL_IMAGE,
        ACCEL_FFT,
        NUM_ACCEL_TYPES
    };

    /* Workload profile for a core */
    struct WorkloadProfile {
        int coreId;
        double matrixIntensity;      // Matrix ops per cycle
        double cordicIntensity;      // Trig ops per cycle
        double averageRequestRate;   // Requests per microsecond
        double contentionRatio;      // Wait time / total time
        Tick lastUpdateTime;
        
        /* Rolling history */
        std::deque<double> requestHistory;
        std::deque<double> contentionHistory;
    };

    /* Allocation decision */
    struct AllocationDecision {
        AllocationMode mode;
        std::vector<int> sharedAccelIds;
        std::map<int, int> dedicatedMapping;  // coreId -> accelId
        double predictedContention;
        double predictedSpeedup;
        Tick decisionTime;
    };

    /* Register accelerator with allocator */
    void registerAccelerator(AcceleratorType type, int id, 
                            std::function<void(AllocationMode)> modeCallback);

    /* Report workload metrics from cores */
    void reportWorkload(int coreId, AcceleratorType type,
                       double requestRate, double waitCycles);

    /* Set current mission phase */
    void setMissionPhase(MissionPhase phase);
    MissionPhase getMissionPhase() const { return currentPhase; }

    /* Get current allocation for a core */
    AllocationMode getAllocation(int coreId, AcceleratorType type);

    /* Force allocation mode (for testing) */
    void forceAllocationMode(AllocationMode mode);

    /* Get allocation recommendation */
    AllocationDecision getRecommendation(AcceleratorType type);

  private:
    /* Parameters */
    int numCores;
    int numAcceleratorsPerType;
    Cycles evaluationInterval;       // How often to re-evaluate
    double contentionThreshold;      // When to switch modes
    double hysteresisMargin;         // Prevent oscillation
    bool enableAdaptive;             // Enable runtime adaptation

    /* State */
    MissionPhase currentPhase;
    std::vector<AllocationMode> currentModes;  // Per accelerator type
    std::vector<WorkloadProfile> workloadProfiles;
    std::deque<AllocationDecision> decisionHistory;

    /* Accelerator registry */
    struct AccelInfo {
        AcceleratorType type;
        int id;
        std::function<void(AllocationMode)> modeCallback;
        bool isShared;
        int assignedCore;  // -1 if shared
    };
    std::vector<AccelInfo> accelerators;

    /* Evaluation logic */
    void evaluateAllocation();
    void applyAllocation(AcceleratorType type, AllocationDecision &decision);
    double predictContention(AcceleratorType type, AllocationMode mode);
    double calculateSharingOverhead(AcceleratorType type);
    AllocationMode selectOptimalMode(AcceleratorType type);

    /* Mission phase policies */
    AllocationMode getMissionPhasePolicy(MissionPhase phase, 
                                         AcceleratorType type);

    /* Events */
    EventFunctionWrapper evaluationEvent;
    void scheduleEvaluation();

    /* Statistics */
    struct AdaptiveAllocatorStats : public statistics::Group
    {
        AdaptiveAllocatorStats(AdaptiveAllocator *parent);

        statistics::Scalar totalEvaluations;
        statistics::Scalar modeChanges;
        statistics::Scalar sharedModeTime;
        statistics::Scalar dedicatedModeTime;
        statistics::Scalar hybridModeTime;
        statistics::Vector perCoreContention;
        statistics::Vector perTypeAllocations;
        statistics::Histogram contentionDistribution;
        statistics::Scalar predictedVsActualError;
        statistics::Scalar adaptationLatency;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_ADAPTIVE_ALLOCATOR_HH__

