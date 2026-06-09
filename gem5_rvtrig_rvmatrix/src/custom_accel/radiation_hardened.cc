/*
 * Radiation-Hardened Accelerator Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/radiation_hardened.hh"
#include "base/trace.hh"
#include "debug/RadiationHardened.hh"
#include "params/ClockedObject.hh"

#include <algorithm>
#include <numeric>
#include <cmath>

namespace gem5
{

RadiationHardenedAccel::RadiationHardenedAccel(const Params &p)
    : ClockedObject(dynamic_cast<const ClockedObjectParams &>(p)),
      numLanes(p.num_lanes),
      defaultMode(static_cast<RedundancyMode>(p.redundancy_mode)),
      seuProbability(p.seu_probability),
      mbuProbability(p.mbu_probability),
      tmrVotingLatency(p.tmr_voting_latency),
      reducedPrecisionBits(p.reduced_precision_bits),
      enableCheckpointing(p.enable_checkpointing),
      currentMode(static_cast<RedundancyMode>(p.redundancy_mode)),
      degradationLevel(DEGRADE_NONE),
      totalErrors(0),
      correctedErrors(0),
      uncorrectedErrors(0),
      nextRequestId(0),
      rng(42),  // Seed for reproducibility
      faultDist(0.0, 1.0),
      stats(this)
{
    /* Initialize lane health */
    laneHealthy.resize(numLanes, true);
    laneErrorCount.resize(numLanes, 0);

    DPRINTF(RadiationHardened, "RadiationHardenedAccel created: %d lanes, "
            "mode=%d, SEU_prob=%.2e\n", numLanes, defaultMode, seuProbability);
}

RadiationHardenedAccel::~RadiationHardenedAccel()
{
}

int
RadiationHardenedAccel::submitCompute(int operationType,
    const std::vector<double> &inputs,
    int precision,
    bool requireTMR,
    std::function<void(std::vector<double>&, ErrorType)> callback)
{
    ComputeRequest req;
    req.requestId = nextRequestId++;
    req.operationType = operationType;
    req.inputs = inputs;
    req.precision = precision;
    req.requireTMR = requireTMR;
    req.callback = callback;

    stats.totalOperations++;

    /* Determine execution mode */
    RedundancyMode execMode = currentMode;
    if (requireTMR && currentMode != MODE_TMR && currentMode != MODE_TMR_WITH_SPARE) {
        if (getActiveLaneCount() >= 3) {
            execMode = MODE_TMR;
        }
    }

    /* Execute based on mode and degradation level */
    std::vector<double> results;
    ErrorType errorType = ERROR_NONE;

    switch (execMode) {
        case MODE_TMR:
        case MODE_TMR_WITH_SPARE: {
            stats.tmrOperations++;
            auto laneResults = executeTMR(req);
            results = voteTMR(laneResults, errorType);
            break;
        }
        case MODE_DMR: {
            stats.dmrOperations++;
            auto [r1, r2] = executeDMR(req);
            if (verifyDMR(r1, r2)) {
                results = r1.outputs;
            } else {
                errorType = ERROR_SEU_DETECTED;
                /* Fall back to single or retry */
                results = r1.outputs;  // Best effort
            }
            break;
        }
        case MODE_NONE:
        default: {
            stats.singleOperations++;
            int laneId = 0;
            for (int i = 0; i < numLanes; i++) {
                if (laneHealthy[i]) {
                    laneId = i;
                    break;
                }
            }
            auto result = executeSingle(req, laneId);
            results = result.outputs;
            errorType = result.errorType;
            break;
        }
    }

    /* Complete request */
    completeRequest(req.requestId, results, errorType);

    return req.requestId;
}

void
RadiationHardenedAccel::setRedundancyMode(RedundancyMode mode)
{
    if (currentMode != mode) {
        /* Check if mode is supported given degradation */
        if (mode == MODE_TMR && getActiveLaneCount() < 3) {
            DPRINTF(RadiationHardened, "Cannot enable TMR: only %d lanes healthy\n",
                    getActiveLaneCount());
            return;
        }

        DPRINTF(RadiationHardened, "Redundancy mode change: %d -> %d\n",
                currentMode, mode);
        currentMode = mode;
    }
}

double
RadiationHardenedAccel::getHealthScore() const
{
    int healthyCount = getActiveLaneCount();
    double baseScore = (double)healthyCount / numLanes;

    /* Penalty for accumulated errors */
    double errorPenalty = std::min(0.3, totalErrors * 0.01);

    /* Penalty for degradation */
    double degradePenalty = degradationLevel * 0.1;

    return std::max(0.0, baseScore - errorPenalty - degradePenalty);
}

void
RadiationHardenedAccel::injectFault(int laneId, ErrorType errorType)
{
    if (laneId < 0 || laneId >= numLanes) return;

    DPRINTF(RadiationHardened, "Injecting fault: lane=%d type=%d\n",
            laneId, errorType);

    laneErrorCount[laneId]++;
    totalErrors++;

    if (errorType == ERROR_LATCHUP || laneErrorCount[laneId] > 10) {
        laneHealthy[laneId] = false;
        stats.latchupEvents++;
        updateDegradationLevel();
    }

    stats.perLaneErrors[laneId]++;
}

bool
RadiationHardenedAccel::attemptRecovery()
{
    stats.recoveryAttempts++;

    bool anyRecovered = false;
    for (int i = 0; i < numLanes; i++) {
        if (!laneHealthy[i] && laneErrorCount[i] < 20) {
            performRecovery(i);
            anyRecovered = true;
        }
    }

    if (anyRecovered) {
        stats.recoverySuccess++;
        updateDegradationLevel();
    }

    return anyRecovered;
}

double
RadiationHardenedAccel::getPerformanceOverhead() const
{
    switch (currentMode) {
        case MODE_TMR:
        case MODE_TMR_WITH_SPARE:
            return 0.15 + (tmrVotingLatency / 10.0);  // ~15% + voting
        case MODE_DMR:
            return 0.05;  // ~5%
        default:
            return 0.0;
    }
}

double
RadiationHardenedAccel::getReliabilityGain() const
{
    switch (currentMode) {
        case MODE_TMR_WITH_SPARE:
            return 0.9999;  // Very high
        case MODE_TMR:
            return 0.999;
        case MODE_DMR:
            return 0.99;
        default:
            return 0.9;  // Base reliability
    }
}

std::vector<RadiationHardenedAccel::TMRLaneResult>
RadiationHardenedAccel::executeTMR(const ComputeRequest &req)
{
    std::vector<TMRLaneResult> results;
    int lanesUsed = 0;

    for (int i = 0; i < numLanes && lanesUsed < 3; i++) {
        if (laneHealthy[i]) {
            auto result = executeSingle(req, i);
            results.push_back(result);
            lanesUsed++;
        }
    }

    stats.tmrVotingCycles += tmrVotingLatency;
    return results;
}

std::vector<double>
RadiationHardenedAccel::voteTMR(const std::vector<TMRLaneResult> &results,
                                ErrorType &detectedError)
{
    if (results.size() < 3) {
        detectedError = ERROR_SEU_DETECTED;
        return results.empty() ? std::vector<double>() : results[0].outputs;
    }

    std::vector<double> voted;
    bool hadError = false;

    /* Vote on each output element */
    size_t outputSize = results[0].outputs.size();
    voted.resize(outputSize);

    for (size_t i = 0; i < outputSize; i++) {
        double v0 = results[0].outputs[i];
        double v1 = results[1].outputs[i];
        double v2 = results[2].outputs[i];

        /* Check for agreement */
        bool agree01 = std::abs(v0 - v1) < 1e-10;
        bool agree02 = std::abs(v0 - v2) < 1e-10;
        bool agree12 = std::abs(v1 - v2) < 1e-10;

        if (agree01 && agree02) {
            voted[i] = v0;  // All agree
        } else if (agree01) {
            voted[i] = v0;  // Lane 2 disagrees
            hadError = true;
        } else if (agree02) {
            voted[i] = v0;  // Lane 1 disagrees
            hadError = true;
        } else if (agree12) {
            voted[i] = v1;  // Lane 0 disagrees
            hadError = true;
        } else {
            /* No agreement - serious error */
            voted[i] = (v0 + v1 + v2) / 3.0;  // Best effort average
            detectedError = ERROR_MBU;
            stats.mbuDetected++;
            return voted;
        }
    }

    if (hadError) {
        detectedError = ERROR_SEU_CORRECTED;
        correctedErrors++;
        stats.seuCorrected++;
    } else {
        detectedError = ERROR_NONE;
    }

    return voted;
}

std::pair<RadiationHardenedAccel::TMRLaneResult, RadiationHardenedAccel::TMRLaneResult>
RadiationHardenedAccel::executeDMR(const ComputeRequest &req)
{
    TMRLaneResult r1, r2;
    int lanesUsed = 0;

    for (int i = 0; i < numLanes && lanesUsed < 2; i++) {
        if (laneHealthy[i]) {
            if (lanesUsed == 0) {
                r1 = executeSingle(req, i);
            } else {
                r2 = executeSingle(req, i);
            }
            lanesUsed++;
        }
    }

    return {r1, r2};
}

bool
RadiationHardenedAccel::verifyDMR(const TMRLaneResult &r1, const TMRLaneResult &r2)
{
    if (r1.outputs.size() != r2.outputs.size()) {
        return false;
    }

    for (size_t i = 0; i < r1.outputs.size(); i++) {
        if (std::abs(r1.outputs[i] - r2.outputs[i]) > 1e-10) {
            stats.seuDetected++;
            return false;
        }
    }

    return true;
}

RadiationHardenedAccel::TMRLaneResult
RadiationHardenedAccel::executeSingle(const ComputeRequest &req, int laneId)
{
    TMRLaneResult result;
    result.laneId = laneId;
    result.hasError = false;
    result.errorType = ERROR_NONE;
    result.computeTime = curTick();

    /* Simulate fault */
    if (simulateFault(laneId)) {
        result.hasError = true;
        result.errorType = ERROR_SEU_DETECTED;
    }

    /* Compute */
    bool hasComputeError = false;
    result.outputs = computeGeneric(req.inputs, req.operationType,
                                   req.precision, hasComputeError);

    /* Apply fault if one occurred */
    if (result.hasError) {
        applyFault(result.outputs, result.errorType);
    }

    result.computeTime = curTick() - result.computeTime;
    return result;
}

std::vector<double>
RadiationHardenedAccel::computeMatrix(const std::vector<double> &inputs,
                                      int precision, bool &hasError)
{
    /* Simulated matrix computation */
    std::vector<double> outputs = inputs;

    /* Apply precision reduction if in degraded mode */
    if (degradationLevel == DEGRADE_REDUCED_PRECISION) {
        double scale = std::pow(2, reducedPrecisionBits);
        for (auto &v : outputs) {
            v = std::round(v * scale) / scale;
        }
    }

    return outputs;
}

std::vector<double>
RadiationHardenedAccel::computeTrig(const std::vector<double> &inputs,
                                    int precision, bool &hasError)
{
    std::vector<double> outputs;
    outputs.reserve(inputs.size());

    for (const auto &v : inputs) {
        outputs.push_back(std::sin(v));  // Example trig
    }

    return outputs;
}

std::vector<double>
RadiationHardenedAccel::computeGeneric(const std::vector<double> &inputs,
                                       int opType, int precision, bool &hasError)
{
    /* Route to appropriate compute function */
    if (opType == 0) {
        return computeMatrix(inputs, precision, hasError);
    } else {
        return computeTrig(inputs, precision, hasError);
    }
}

bool
RadiationHardenedAccel::simulateFault(int laneId)
{
    /* Simulate random fault based on probability */
    double roll = faultDist(rng);

    if (roll < mbuProbability) {
        return true;  // Multi-bit upset
    } else if (roll < seuProbability) {
        return true;  // Single-event upset
    }

    return false;
}

void
RadiationHardenedAccel::applyFault(std::vector<double> &data, ErrorType errorType)
{
    if (data.empty()) return;

    /* Flip a random bit in a random element */
    int idx = rng() % data.size();
    uint64_t *bits = reinterpret_cast<uint64_t*>(&data[idx]);

    if (errorType == ERROR_MBU) {
        /* Multi-bit: flip several bits */
        *bits ^= 0xFF;
    } else {
        /* Single bit */
        int bitPos = rng() % 64;
        *bits ^= (1ULL << bitPos);
    }

    stats.errorDistribution.sample(errorType);
}

void
RadiationHardenedAccel::updateDegradationLevel()
{
    int healthyCount = getActiveLaneCount();

    if (healthyCount >= numLanes) {
        degradationLevel = DEGRADE_NONE;
    } else if (healthyCount >= 3) {
        degradationLevel = DEGRADE_REDUCED_TMR;
    } else if (healthyCount >= 2) {
        degradationLevel = DEGRADE_DMR_ONLY;
        if (currentMode == MODE_TMR) {
            setRedundancyMode(MODE_DMR);
        }
    } else if (healthyCount >= 1) {
        degradationLevel = DEGRADE_SINGLE_LANE;
        setRedundancyMode(MODE_NONE);
    } else {
        degradationLevel = DEGRADE_REDUCED_PRECISION;
        switchToReducedPrecision();
    }

    stats.degradationEvents++;
    DPRINTF(RadiationHardened, "Degradation level: %d (healthy lanes: %d)\n",
            degradationLevel, healthyCount);
}

int
RadiationHardenedAccel::getActiveLaneCount() const
{
    int count = 0;
    for (bool healthy : laneHealthy) {
        if (healthy) count++;
    }
    return count;
}

void
RadiationHardenedAccel::switchToReducedPrecision()
{
    DPRINTF(RadiationHardened, "Switching to reduced precision mode: %d bits\n",
            reducedPrecisionBits);
    stats.precisionReductions++;
}

void
RadiationHardenedAccel::performRecovery(int laneId)
{
    DPRINTF(RadiationHardened, "Attempting recovery for lane %d\n", laneId);

    /* Simple recovery: reset error count and mark healthy */
    laneErrorCount[laneId] = 0;
    laneHealthy[laneId] = true;
}

void
RadiationHardenedAccel::checkpointState()
{
    if (!enableCheckpointing) return;
    DPRINTF(RadiationHardened, "Checkpointing accelerator state\n");
    /* Implementation would save state to memory */
}

void
RadiationHardenedAccel::restoreFromCheckpoint()
{
    if (!enableCheckpointing) return;
    DPRINTF(RadiationHardened, "Restoring from checkpoint\n");
    /* Implementation would restore state */
}

void
RadiationHardenedAccel::completeRequest(int requestId, std::vector<double> &results,
                                        ErrorType errorType)
{
    auto it = pendingRequests.find(requestId);
    if (it != pendingRequests.end()) {
        if (it->second.callback) {
            it->second.callback(results, errorType);
        }
        pendingRequests.erase(it);
    }

    stats.reliabilityScore = getHealthScore();
    stats.avgRedundancyOverhead = getPerformanceOverhead();
}

RadiationHardenedAccel::RadHardStats::RadHardStats(RadiationHardenedAccel *parent)
    : statistics::Group(parent),
      ADD_STAT(totalOperations, statistics::units::Count::get(),
               "Total operations"),
      ADD_STAT(tmrOperations, statistics::units::Count::get(),
               "TMR operations"),
      ADD_STAT(dmrOperations, statistics::units::Count::get(),
               "DMR operations"),
      ADD_STAT(singleOperations, statistics::units::Count::get(),
               "Single-lane operations"),
      ADD_STAT(seuDetected, statistics::units::Count::get(),
               "Single-event upsets detected"),
      ADD_STAT(seuCorrected, statistics::units::Count::get(),
               "Single-event upsets corrected"),
      ADD_STAT(mbuDetected, statistics::units::Count::get(),
               "Multi-bit upsets detected"),
      ADD_STAT(latchupEvents, statistics::units::Count::get(),
               "Latchup events"),
      ADD_STAT(recoveryAttempts, statistics::units::Count::get(),
               "Recovery attempts"),
      ADD_STAT(recoverySuccess, statistics::units::Count::get(),
               "Successful recoveries"),
      ADD_STAT(degradationEvents, statistics::units::Count::get(),
               "Degradation events"),
      ADD_STAT(precisionReductions, statistics::units::Count::get(),
               "Precision reductions"),
      ADD_STAT(perLaneErrors, statistics::units::Count::get(),
               "Errors per lane"),
      ADD_STAT(errorDistribution, statistics::units::Count::get(),
               "Error type distribution"),
      ADD_STAT(avgRedundancyOverhead, statistics::units::Ratio::get(),
               "Average redundancy overhead"),
      ADD_STAT(reliabilityScore, statistics::units::Ratio::get(),
               "System reliability score"),
      ADD_STAT(tmrVotingCycles, statistics::units::Cycle::get(),
               "TMR voting cycles")
{
    perLaneErrors.init(parent->numLanes);
    for (int i = 0; i < parent->numLanes; i++) {
        perLaneErrors.subname(i, std::string("lane") + std::to_string(i));
    }

    errorDistribution.init(5);  // 5 error types
}

} // namespace gem5

