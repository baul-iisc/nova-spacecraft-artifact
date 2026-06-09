/*
 * Radiation-Hardened Accelerator Architecture
 * PhD Research: Chandraboul
 * 
 * Research Question 3: Radiation-Hardened Accelerator Architectures
 * - Triple Modular Redundancy (TMR) for matrix/trig units
 * - Error detection and correction in computation
 * - Performance-reliability tradeoff analysis
 * - Graceful degradation after radiation damage
 */

#ifndef __CUSTOM_ACCEL_RADIATION_HARDENED_HH__
#define __CUSTOM_ACCEL_RADIATION_HARDENED_HH__

#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/RadiationHardenedAccel.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <vector>
#include <functional>
#include <random>
#include <array>

namespace gem5
{

/**
 * RadiationHardenedAccel - Radiation-tolerant accelerator with TMR
 * 
 * Features:
 * 1. Triple Modular Redundancy (TMR) for critical computations
 * 2. Single-Event Upset (SEU) detection and correction
 * 3. Graceful degradation modes
 * 4. Reduced-precision fallback after damage
 * 5. Checkpointing and recovery
 */
class RadiationHardenedAccel : public ClockedObject
{
  public:
    PARAMS(RadiationHardenedAccel);
    RadiationHardenedAccel(const Params &p);
    ~RadiationHardenedAccel();

    /* Redundancy modes */
    enum RedundancyMode {
        MODE_NONE,          // No redundancy (maximum performance)
        MODE_DMR,           // Dual Modular Redundancy (detect only)
        MODE_TMR,           // Triple Modular Redundancy (detect + correct)
        MODE_TMR_WITH_SPARE // TMR + hot spare for recovery
    };

    /* Degradation levels */
    enum DegradationLevel {
        DEGRADE_NONE,           // Fully functional
        DEGRADE_REDUCED_TMR,    // Lost one TMR lane
        DEGRADE_DMR_ONLY,       // Can only do DMR
        DEGRADE_SINGLE_LANE,    // Single lane only
        DEGRADE_REDUCED_PRECISION  // Lower precision mode
    };

    /* Error types */
    enum ErrorType {
        ERROR_NONE,
        ERROR_SEU_CORRECTED,     // Single-event upset, corrected by TMR
        ERROR_SEU_DETECTED,      // Detected but couldn't correct
        ERROR_MBU,               // Multi-bit upset
        ERROR_LATCHUP,           // Latchup event
        ERROR_ACCUMULATED        // Accumulated damage
    };

    /* Computation request */
    struct ComputeRequest {
        int requestId;
        int operationType;       // Matrix mult, trig, etc.
        std::vector<double> inputs;
        int precision;           // Bits of precision
        bool requireTMR;         // Force TMR regardless of mode
        std::function<void(std::vector<double>&, ErrorType)> callback;
    };

    /* TMR lane result */
    struct TMRLaneResult {
        int laneId;
        std::vector<double> outputs;
        bool hasError;
        ErrorType errorType;
        Tick computeTime;
    };

    /* Submit computation */
    int submitCompute(int operationType, 
                     const std::vector<double> &inputs,
                     int precision,
                     bool requireTMR,
                     std::function<void(std::vector<double>&, ErrorType)> callback);

    /* Set redundancy mode */
    void setRedundancyMode(RedundancyMode mode);
    RedundancyMode getRedundancyMode() const { return currentMode; }

    /* Get current health status */
    DegradationLevel getDegradationLevel() const { return degradationLevel; }
    double getHealthScore() const;  // 0.0 (dead) to 1.0 (perfect)

    /* Inject fault for testing */
    void injectFault(int laneId, ErrorType errorType);

    /* Trigger recovery attempt */
    bool attemptRecovery();

    /* Get performance-reliability tradeoff metrics */
    double getPerformanceOverhead() const;
    double getReliabilityGain() const;

  private:
    /* Parameters */
    int numLanes;                    // Number of compute lanes
    RedundancyMode defaultMode;
    double seuProbability;           // SEU probability per operation
    double mbuProbability;           // MBU probability
    Cycles tmrVotingLatency;
    int reducedPrecisionBits;
    bool enableCheckpointing;

    /* State */
    RedundancyMode currentMode;
    DegradationLevel degradationLevel;
    std::vector<bool> laneHealthy;
    std::vector<int> laneErrorCount;
    int totalErrors;
    int correctedErrors;
    int uncorrectedErrors;
    int nextRequestId;

    /* Random number generator for fault simulation */
    std::mt19937 rng;
    std::uniform_real_distribution<double> faultDist;

    /* Pending requests */
    std::map<int, ComputeRequest> pendingRequests;

    /* TMR implementation */
    std::vector<TMRLaneResult> executeTMR(const ComputeRequest &req);
    std::vector<double> voteTMR(const std::vector<TMRLaneResult> &results,
                                ErrorType &detectedError);
    
    /* DMR implementation */
    std::pair<TMRLaneResult, TMRLaneResult> executeDMR(const ComputeRequest &req);
    bool verifyDMR(const TMRLaneResult &r1, const TMRLaneResult &r2);

    /* Single lane execution */
    TMRLaneResult executeSingle(const ComputeRequest &req, int laneId);

    /* Actual compute functions (simulated) */
    std::vector<double> computeMatrix(const std::vector<double> &inputs, 
                                      int precision, bool &hasError);
    std::vector<double> computeTrig(const std::vector<double> &inputs,
                                    int precision, bool &hasError);
    std::vector<double> computeGeneric(const std::vector<double> &inputs,
                                       int opType, int precision, bool &hasError);

    /* Fault injection during computation */
    bool simulateFault(int laneId);
    void applyFault(std::vector<double> &data, ErrorType errorType);

    /* Degradation handling */
    void updateDegradationLevel();
    int getActiveLaneCount() const;
    void switchToReducedPrecision();

    /* Recovery */
    void performRecovery(int laneId);
    void checkpointState();
    void restoreFromCheckpoint();

    /* Completion handling */
    void completeRequest(int requestId, std::vector<double> &results,
                        ErrorType errorType);

    /* Statistics */
    struct RadHardStats : public statistics::Group
    {
        RadHardStats(RadiationHardenedAccel *parent);

        statistics::Scalar totalOperations;
        statistics::Scalar tmrOperations;
        statistics::Scalar dmrOperations;
        statistics::Scalar singleOperations;
        statistics::Scalar seuDetected;
        statistics::Scalar seuCorrected;
        statistics::Scalar mbuDetected;
        statistics::Scalar latchupEvents;
        statistics::Scalar recoveryAttempts;
        statistics::Scalar recoverySuccess;
        statistics::Scalar degradationEvents;
        statistics::Scalar precisionReductions;
        statistics::Vector perLaneErrors;
        statistics::Histogram errorDistribution;
        statistics::Scalar avgRedundancyOverhead;
        statistics::Scalar reliabilityScore;
        statistics::Scalar tmrVotingCycles;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_RADIATION_HARDENED_HH__

