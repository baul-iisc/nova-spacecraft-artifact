/*
 * Mission-Aware Performance Metrics
 * PhD Research: Chandraboul
 * 
 * Research Question 8: Mission-Aware Performance Metrics
 * - Operations-per-joule in realistic thermal environments
 * - Soft-error resilience under particle bombardment
 * - Time-to-recovery after single-event upsets
 * - Mission-specific performance characterization
 */

#ifndef __CUSTOM_ACCEL_MISSION_METRICS_HH__
#define __CUSTOM_ACCEL_MISSION_METRICS_HH__

#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/MissionMetrics.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <vector>
#include <map>
#include <deque>
#include <string>

namespace gem5
{

/**
 * MissionMetrics - Space Mission-Specific Performance Metrics
 * 
 * Beyond speedup - metrics that matter for space missions:
 * 1. Operations per Joule (energy efficiency)
 * 2. Soft-error resilience (MTBF, error rates)
 * 3. Time-to-recovery (fault recovery latency)
 * 4. Deadline compliance (real-time guarantees)
 * 5. Thermal performance (ops under thermal stress)
 */
class MissionMetrics : public ClockedObject
{
  public:
    PARAMS(MissionMetrics);
    MissionMetrics(const Params &p);
    ~MissionMetrics();

    /* Mission phases for context */
    enum MissionPhase {
        PHASE_LAUNCH,
        PHASE_ORBIT_INSERTION,
        PHASE_CRUISE,
        PHASE_APPROACH,
        PHASE_ENTRY_DESCENT_LANDING,
        PHASE_SURFACE_OPS,
        PHASE_SCIENCE,
        PHASE_EMERGENCY,
        NUM_PHASES
    };

    /* Metric categories */
    enum MetricCategory {
        METRIC_PERFORMANCE,
        METRIC_ENERGY,
        METRIC_RELIABILITY,
        METRIC_THERMAL,
        METRIC_REALTIME,
        NUM_CATEGORIES
    };

    /* Real-time task result */
    struct RealtimeTaskResult {
        std::string taskName;
        Tick deadline;
        Tick completionTime;
        bool metDeadline;
        int priority;
    };

    /* Energy measurement point */
    struct EnergyMeasurement {
        Tick timestamp;
        double powerWatts;
        double operations;
        double opsPerJoule;
        double temperature;
    };

    /* Reliability event */
    struct ReliabilityEvent {
        Tick timestamp;
        std::string eventType;
        std::string component;
        Tick recoveryTime;
        bool recovered;
    };

    /* Set current mission phase */
    void setMissionPhase(MissionPhase phase);
    MissionPhase getMissionPhase() const { return currentPhase; }

    /* Report operation completion for energy tracking */
    void reportOperation(double operations, double powerWatts, 
                        double temperature);

    /* Report real-time task completion */
    void reportRealtimeTask(const std::string &taskName, 
                           Tick deadline, Tick completionTime,
                           int priority);

    /* Report reliability event (error, recovery, etc.) */
    void reportReliabilityEvent(const std::string &eventType,
                               const std::string &component,
                               Tick recoveryTime, bool recovered);

    /* Get summary metrics */
    double getOpsPerJoule() const;
    double getMeanTimeBetweenFailures() const;
    double getDeadlineMissRate() const;
    double getAverageRecoveryTime() const;
    double getThermalEfficiency() const;

    /* Get phase-specific metrics */
    double getPhaseOpsPerJoule(MissionPhase phase) const;
    double getPhaseDeadlineMissRate(MissionPhase phase) const;

    /* Generate mission report */
    std::string generateMissionReport() const;

  private:
    /* Parameters */
    std::string missionName;
    double targetOpsPerJoule;
    double targetMTBF;
    double maxDeadlineMissRate;
    double nominalTemperature;

    /* State */
    MissionPhase currentPhase;
    Tick phaseStartTime;
    std::map<MissionPhase, Tick> phaseTime;

    /* Collected data */
    std::deque<EnergyMeasurement> energyHistory;
    std::deque<RealtimeTaskResult> realtimeHistory;
    std::deque<ReliabilityEvent> reliabilityHistory;

    /* Per-phase aggregates */
    std::map<MissionPhase, double> phaseEnergy;
    std::map<MissionPhase, double> phaseOperations;
    std::map<MissionPhase, int> phaseDeadlineMisses;
    std::map<MissionPhase, int> phaseTaskCount;

    /* Thermal tracking */
    double currentTemperature;
    double maxTemperature;
    double minTemperature;
    std::deque<double> temperatureHistory;

    /* Helper functions */
    void updateEnergyStats(const EnergyMeasurement &m);
    void updateRealtimeStats(const RealtimeTaskResult &r);
    void updateReliabilityStats(const ReliabilityEvent &e);
    double calculateThermalDerating(double temperature) const;

    /* Statistics */
    struct MissionMetricsStats : public statistics::Group
    {
        MissionMetricsStats(MissionMetrics *parent);

        /* Energy metrics */
        statistics::Scalar totalOperations;
        statistics::Scalar totalEnergyJoules;
        statistics::Scalar avgOpsPerJoule;
        statistics::Scalar peakOpsPerJoule;
        statistics::Vector phaseOpsPerJoule;

        /* Reliability metrics */
        statistics::Scalar totalErrors;
        statistics::Scalar correctedErrors;
        statistics::Scalar uncorrectedErrors;
        statistics::Scalar meanTimeBetweenFailures;
        statistics::Scalar avgRecoveryTime;
        statistics::Scalar maxRecoveryTime;
        statistics::Vector errorsByComponent;

        /* Real-time metrics */
        statistics::Scalar totalTasks;
        statistics::Scalar deadlinesMet;
        statistics::Scalar deadlinesMissed;
        statistics::Scalar deadlineMissRate;
        statistics::Histogram taskLatencyDist;
        statistics::Vector phaseDeadlineMisses;

        /* Thermal metrics */
        statistics::Scalar avgTemperature;
        statistics::Scalar maxTemperature;
        statistics::Scalar thermalThrottleTime;
        statistics::Scalar thermalEfficiency;
        statistics::Histogram temperatureDist;

        /* Phase tracking */
        statistics::Vector phaseTime;
        statistics::Scalar currentPhase;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_MISSION_METRICS_HH__

