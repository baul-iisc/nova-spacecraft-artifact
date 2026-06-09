/*
 * Adaptive Accelerator Allocation System Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/adaptive_allocator.hh"
#include "base/trace.hh"
#include "debug/AdaptiveAllocator.hh"
#include "params/ClockedObject.hh"
#include "sim/system.hh"

#include <algorithm>
#include <numeric>

namespace gem5
{

AdaptiveAllocator::AdaptiveAllocator(const Params &p)
    : ClockedObject(dynamic_cast<const ClockedObjectParams &>(p)),
      numCores(p.num_cores),
      numAcceleratorsPerType(p.num_accelerators_per_type),
      evaluationInterval(p.evaluation_interval),
      contentionThreshold(p.contention_threshold),
      hysteresisMargin(p.hysteresis_margin),
      enableAdaptive(p.enable_adaptive),
      currentPhase(PHASE_CRUISE),
      evaluationEvent([this]{ evaluateAllocation(); }, name() + ".evaluate"),
      stats(this)
{
    /* Initialize per-type modes */
    currentModes.resize(NUM_ACCEL_TYPES, MODE_SHARED);
    
    /* Initialize workload profiles for each core */
    workloadProfiles.resize(numCores);
    for (int i = 0; i < numCores; i++) {
        workloadProfiles[i].coreId = i;
        workloadProfiles[i].matrixIntensity = 0.0;
        workloadProfiles[i].cordicIntensity = 0.0;
        workloadProfiles[i].averageRequestRate = 0.0;
        workloadProfiles[i].contentionRatio = 0.0;
        workloadProfiles[i].lastUpdateTime = 0;
    }

    /* Schedule first evaluation */
    if (enableAdaptive) {
        scheduleEvaluation();
    }

    DPRINTF(AdaptiveAllocator, "AdaptiveAllocator created: %d cores, "
            "%d accels/type, interval=%lu cycles\n",
            numCores, numAcceleratorsPerType, evaluationInterval);
}

AdaptiveAllocator::~AdaptiveAllocator()
{
}

void
AdaptiveAllocator::registerAccelerator(AcceleratorType type, int id,
    std::function<void(AllocationMode)> modeCallback)
{
    AccelInfo info;
    info.type = type;
    info.id = id;
    info.modeCallback = modeCallback;
    info.isShared = true;
    info.assignedCore = -1;

    accelerators.push_back(info);

    DPRINTF(AdaptiveAllocator, "Registered accelerator: type=%d, id=%d\n",
            type, id);
}

void
AdaptiveAllocator::reportWorkload(int coreId, AcceleratorType type,
                                  double requestRate, double waitCycles)
{
    if (coreId < 0 || coreId >= numCores) return;

    WorkloadProfile &profile = workloadProfiles[coreId];
    
    /* Update intensities based on type */
    if (type == ACCEL_MATRIX) {
        profile.matrixIntensity = requestRate;
    } else if (type == ACCEL_CORDIC) {
        profile.cordicIntensity = requestRate;
    }

    /* Update request rate history */
    profile.requestHistory.push_back(requestRate);
    if (profile.requestHistory.size() > 100) {
        profile.requestHistory.pop_front();
    }

    /* Calculate contention ratio */
    double totalCycles = curTick() - profile.lastUpdateTime;
    if (totalCycles > 0) {
        double contention = waitCycles / (totalCycles / clockPeriod());
        profile.contentionHistory.push_back(contention);
        if (profile.contentionHistory.size() > 100) {
            profile.contentionHistory.pop_front();
        }
        
        /* Running average */
        profile.contentionRatio = std::accumulate(
            profile.contentionHistory.begin(),
            profile.contentionHistory.end(), 0.0) / 
            profile.contentionHistory.size();
    }

    profile.lastUpdateTime = curTick();
    stats.perCoreContention[coreId] = profile.contentionRatio;

    DPRINTF(AdaptiveAllocator, "Core %d workload: type=%d rate=%.2f "
            "contention=%.2f%%\n", coreId, type, requestRate, 
            profile.contentionRatio * 100);
}

void
AdaptiveAllocator::setMissionPhase(MissionPhase phase)
{
    if (currentPhase != phase) {
        DPRINTF(AdaptiveAllocator, "Mission phase change: %d -> %d\n",
                currentPhase, phase);
        currentPhase = phase;

        /* Re-evaluate allocation based on new phase */
        if (enableAdaptive) {
            evaluateAllocation();
        }
    }
}

AdaptiveAllocator::AllocationMode
AdaptiveAllocator::getAllocation(int coreId, AcceleratorType type)
{
    return currentModes[type];
}

void
AdaptiveAllocator::forceAllocationMode(AllocationMode mode)
{
    for (int i = 0; i < NUM_ACCEL_TYPES; i++) {
        currentModes[i] = mode;
    }

    /* Notify all accelerators */
    for (auto &accel : accelerators) {
        if (accel.modeCallback) {
            accel.modeCallback(mode);
        }
    }

    DPRINTF(AdaptiveAllocator, "Forced allocation mode: %d\n", mode);
}

AdaptiveAllocator::AllocationDecision
AdaptiveAllocator::getRecommendation(AcceleratorType type)
{
    AllocationDecision decision;
    decision.mode = selectOptimalMode(type);
    decision.predictedContention = predictContention(type, decision.mode);
    decision.predictedSpeedup = 1.0 / (1.0 + decision.predictedContention);
    decision.decisionTime = curTick();

    return decision;
}

void
AdaptiveAllocator::evaluateAllocation()
{
    stats.totalEvaluations++;

    DPRINTF(AdaptiveAllocator, "=== Allocation Evaluation ===\n");

    for (int type = 0; type < NUM_ACCEL_TYPES; type++) {
        AcceleratorType accelType = static_cast<AcceleratorType>(type);
        
        /* Get mission phase policy */
        AllocationMode phasePolicy = getMissionPhasePolicy(currentPhase, 
                                                           accelType);

        /* If in emergency or launch, use phase policy */
        if (currentPhase == PHASE_EMERGENCY || currentPhase == PHASE_LAUNCH) {
            if (currentModes[type] != phasePolicy) {
                AllocationDecision decision;
                decision.mode = phasePolicy;
                decision.decisionTime = curTick();
                applyAllocation(accelType, decision);
            }
            continue;
        }

        /* Calculate current contention */
        double avgContention = 0.0;
        int activeProfiles = 0;
        for (const auto &profile : workloadProfiles) {
            if (!profile.contentionHistory.empty()) {
                avgContention += profile.contentionRatio;
                activeProfiles++;
            }
        }
        if (activeProfiles > 0) {
            avgContention /= activeProfiles;
        }

        stats.contentionDistribution.sample(avgContention * 100);

        /* Determine optimal mode */
        AllocationMode optimalMode = currentModes[type];

        if (avgContention > contentionThreshold + hysteresisMargin) {
            /* High contention: consider switching to dedicated */
            if (currentModes[type] == MODE_SHARED) {
                optimalMode = MODE_DEDICATED;
                DPRINTF(AdaptiveAllocator, "Type %d: High contention (%.2f%%), "
                        "recommending DEDICATED\n", type, avgContention * 100);
            }
        } else if (avgContention < contentionThreshold - hysteresisMargin) {
            /* Low contention: consider switching to shared */
            if (currentModes[type] == MODE_DEDICATED) {
                /* Check if sharing overhead is acceptable */
                double overhead = calculateSharingOverhead(accelType);
                if (overhead < 0.1) {  // Less than 10% overhead
                    optimalMode = MODE_SHARED;
                    DPRINTF(AdaptiveAllocator, "Type %d: Low contention (%.2f%%), "
                            "recommending SHARED\n", type, avgContention * 100);
                }
            }
        }

        /* Apply if mode changed */
        if (optimalMode != currentModes[type]) {
            AllocationDecision decision;
            decision.mode = optimalMode;
            decision.predictedContention = predictContention(accelType, 
                                                             optimalMode);
            decision.decisionTime = curTick();
            applyAllocation(accelType, decision);
        }
    }

    /* Schedule next evaluation */
    scheduleEvaluation();
}

void
AdaptiveAllocator::applyAllocation(AcceleratorType type, 
                                   AllocationDecision &decision)
{
    AllocationMode oldMode = currentModes[type];
    currentModes[type] = decision.mode;
    stats.modeChanges++;

    /* Update mode time stats */
    static Tick lastModeChange = 0;
    Tick elapsed = curTick() - lastModeChange;
    
    switch (oldMode) {
        case MODE_SHARED:
            stats.sharedModeTime += elapsed;
            break;
        case MODE_DEDICATED:
            stats.dedicatedModeTime += elapsed;
            break;
        case MODE_HYBRID:
            stats.hybridModeTime += elapsed;
            break;
        default:
            break;
    }
    lastModeChange = curTick();

    /* Notify accelerators */
    for (auto &accel : accelerators) {
        if (accel.type == type && accel.modeCallback) {
            accel.modeCallback(decision.mode);
        }
    }

    /* Store decision in history */
    decisionHistory.push_back(decision);
    if (decisionHistory.size() > 1000) {
        decisionHistory.pop_front();
    }

    DPRINTF(AdaptiveAllocator, "Applied allocation: type=%d, mode=%d->%d\n",
            type, oldMode, decision.mode);
}

double
AdaptiveAllocator::predictContention(AcceleratorType type, AllocationMode mode)
{
    if (mode == MODE_DEDICATED) {
        return 0.0;  // No contention with dedicated accelerators
    }

    /* Estimate contention based on workload profiles */
    double totalRequestRate = 0.0;
    for (const auto &profile : workloadProfiles) {
        if (type == ACCEL_MATRIX) {
            totalRequestRate += profile.matrixIntensity;
        } else if (type == ACCEL_CORDIC) {
            totalRequestRate += profile.cordicIntensity;
        }
    }

    /* Simple queuing model: contention = arrival_rate / service_rate */
    double serviceRate = 1.0;  // Normalized
    double contention = std::min(0.9, totalRequestRate / serviceRate);

    return contention;
}

double
AdaptiveAllocator::calculateSharingOverhead(AcceleratorType type)
{
    /* Calculate overhead of sharing vs dedicated */
    double overhead = 0.0;

    /* Arbitration overhead */
    double arbitrationCycles = 2.0;  // Typical arbitration latency
    double avgServiceTime = 10.0;    // Average operation time
    overhead += arbitrationCycles / avgServiceTime;

    /* Context switching overhead */
    overhead += 0.01;  // 1% for context

    return overhead;
}

AdaptiveAllocator::AllocationMode
AdaptiveAllocator::selectOptimalMode(AcceleratorType type)
{
    /* Start with mission phase policy */
    AllocationMode baseMode = getMissionPhasePolicy(currentPhase, type);

    /* Calculate metrics */
    double predictedSharedContention = predictContention(type, MODE_SHARED);
    double sharingOverhead = calculateSharingOverhead(type);

    /* Decision logic */
    if (predictedSharedContention > contentionThreshold) {
        return MODE_DEDICATED;
    } else if (sharingOverhead > 0.15) {
        return MODE_DEDICATED;
    } else if (numCores > numAcceleratorsPerType * 2) {
        return MODE_SHARED;  // Not enough accelerators for dedicated
    }

    return baseMode;
}

AdaptiveAllocator::AllocationMode
AdaptiveAllocator::getMissionPhasePolicy(MissionPhase phase, 
                                         AcceleratorType type)
{
    switch (phase) {
        case PHASE_LAUNCH:
        case PHASE_ORBIT_INSERT:
            /* Critical phases: prioritize reliability */
            if (type == ACCEL_MATRIX || type == ACCEL_CORDIC) {
                return MODE_DEDICATED;  // Navigation gets dedicated
            }
            return MODE_SHARED;

        case PHASE_CRUISE:
            /* Low activity: maximize resource sharing */
            return MODE_SHARED;

        case PHASE_SCIENCE_OPS:
            /* High throughput: hybrid for balance */
            return MODE_HYBRID;

        case PHASE_EMERGENCY:
            /* Emergency: all dedicated for determinism */
            return MODE_DEDICATED;

        default:
            return MODE_SHARED;
    }
}

void
AdaptiveAllocator::scheduleEvaluation()
{
    if (!evaluationEvent.scheduled()) {
        schedule(evaluationEvent, clockEdge(evaluationInterval));
    }
}

AdaptiveAllocator::AdaptiveAllocatorStats::AdaptiveAllocatorStats(
    AdaptiveAllocator *parent)
    : statistics::Group(parent),
      ADD_STAT(totalEvaluations, statistics::units::Count::get(),
               "Total allocation evaluations"),
      ADD_STAT(modeChanges, statistics::units::Count::get(),
               "Total mode changes"),
      ADD_STAT(sharedModeTime, statistics::units::Tick::get(),
               "Time in shared mode"),
      ADD_STAT(dedicatedModeTime, statistics::units::Tick::get(),
               "Time in dedicated mode"),
      ADD_STAT(hybridModeTime, statistics::units::Tick::get(),
               "Time in hybrid mode"),
      ADD_STAT(perCoreContention, statistics::units::Ratio::get(),
               "Per-core contention ratio"),
      ADD_STAT(perTypeAllocations, statistics::units::Count::get(),
               "Allocations per type"),
      ADD_STAT(contentionDistribution, statistics::units::Ratio::get(),
               "Contention distribution"),
      ADD_STAT(predictedVsActualError, statistics::units::Ratio::get(),
               "Prediction accuracy"),
      ADD_STAT(adaptationLatency, statistics::units::Cycle::get(),
               "Time to adapt")
{
    perCoreContention.init(parent->numCores);
    perTypeAllocations.init(NUM_ACCEL_TYPES);
    contentionDistribution.init(20);  // 0-100% in 5% buckets

    for (int i = 0; i < parent->numCores; i++) {
        perCoreContention.subname(i, std::string("core") + std::to_string(i));
    }

    const char* typeNames[] = {"matrix", "cordic", "compression", 
                               "image", "fft"};
    for (int i = 0; i < NUM_ACCEL_TYPES; i++) {
        perTypeAllocations.subname(i, typeNames[i]);
    }
}

} // namespace gem5

