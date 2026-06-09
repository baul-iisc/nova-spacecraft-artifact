/*
 * Mission-Aware Performance Metrics Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/mission_metrics.hh"
#include "base/trace.hh"
#include "debug/MissionMetrics.hh"
#include "params/ClockedObject.hh"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace gem5
{

MissionMetrics::MissionMetrics(const Params &p)
    : ClockedObject(dynamic_cast<const ClockedObjectParams &>(p)),
      missionName(p.mission_name),
      targetOpsPerJoule(p.target_ops_per_joule),
      targetMTBF(p.target_mtbf),
      maxDeadlineMissRate(p.max_deadline_miss_rate),
      nominalTemperature(p.nominal_temperature),
      currentPhase(PHASE_CRUISE),
      phaseStartTime(0),
      currentTemperature(nominalTemperature),
      maxTemperature(nominalTemperature),
      minTemperature(nominalTemperature),
      stats(this)
{
    /* Initialize phase tracking */
    for (int i = 0; i < NUM_PHASES; i++) {
        MissionPhase phase = static_cast<MissionPhase>(i);
        phaseTime[phase] = 0;
        phaseEnergy[phase] = 0;
        phaseOperations[phase] = 0;
        phaseDeadlineMisses[phase] = 0;
        phaseTaskCount[phase] = 0;
    }

    DPRINTF(MissionMetrics, "MissionMetrics created: mission=%s "
            "target_opj=%.2e target_mtbf=%.2f\n",
            missionName.c_str(), targetOpsPerJoule, targetMTBF);
}

MissionMetrics::~MissionMetrics()
{
}

void
MissionMetrics::setMissionPhase(MissionPhase phase)
{
    if (phase == currentPhase) return;

    /* Record time in previous phase */
    Tick elapsed = curTick() - phaseStartTime;
    phaseTime[currentPhase] += elapsed;
    stats.phaseTime[currentPhase] += elapsed;

    DPRINTF(MissionMetrics, "Phase transition: %d -> %d (%.2f ms in phase %d)\n",
            currentPhase, phase, elapsed / 1e9, currentPhase);

    currentPhase = phase;
    phaseStartTime = curTick();
    stats.currentPhase = phase;
}

void
MissionMetrics::reportOperation(double operations, double powerWatts,
                                double temperature)
{
    EnergyMeasurement m;
    m.timestamp = curTick();
    m.powerWatts = powerWatts;
    m.operations = operations;
    m.temperature = temperature;

    /* Calculate ops/joule (assuming 1 tick = 1 ps) */
    double timeSeconds = 1e-12;  // Per tick
    double energy = powerWatts * timeSeconds;
    m.opsPerJoule = (energy > 0) ? operations / energy : 0;

    energyHistory.push_back(m);
    if (energyHistory.size() > 10000) {
        energyHistory.pop_front();
    }

    updateEnergyStats(m);

    /* Track temperature */
    currentTemperature = temperature;
    if (temperature > maxTemperature) maxTemperature = temperature;
    if (temperature < minTemperature) minTemperature = temperature;
    temperatureHistory.push_back(temperature);
    if (temperatureHistory.size() > 1000) {
        temperatureHistory.pop_front();
    }

    /* Update phase-specific tracking */
    phaseOperations[currentPhase] += operations;
    double energyJoules = powerWatts * 1e-12;  // Per tick
    phaseEnergy[currentPhase] += energyJoules;
}

void
MissionMetrics::reportRealtimeTask(const std::string &taskName,
                                   Tick deadline, Tick completionTime,
                                   int priority)
{
    RealtimeTaskResult r;
    r.taskName = taskName;
    r.deadline = deadline;
    r.completionTime = completionTime;
    r.metDeadline = (completionTime <= deadline);
    r.priority = priority;

    realtimeHistory.push_back(r);
    if (realtimeHistory.size() > 10000) {
        realtimeHistory.pop_front();
    }

    updateRealtimeStats(r);

    /* Update phase tracking */
    phaseTaskCount[currentPhase]++;
    if (!r.metDeadline) {
        phaseDeadlineMisses[currentPhase]++;
    }

    DPRINTF(MissionMetrics, "RT Task '%s': deadline=%lu complete=%lu %s\n",
            taskName.c_str(), deadline, completionTime,
            r.metDeadline ? "MET" : "MISSED");
}

void
MissionMetrics::reportReliabilityEvent(const std::string &eventType,
                                       const std::string &component,
                                       Tick recoveryTime, bool recovered)
{
    ReliabilityEvent e;
    e.timestamp = curTick();
    e.eventType = eventType;
    e.component = component;
    e.recoveryTime = recoveryTime;
    e.recovered = recovered;

    reliabilityHistory.push_back(e);
    if (reliabilityHistory.size() > 1000) {
        reliabilityHistory.pop_front();
    }

    updateReliabilityStats(e);

    DPRINTF(MissionMetrics, "Reliability event: type=%s component=%s "
            "recovery=%lu %s\n", eventType.c_str(), component.c_str(),
            recoveryTime, recovered ? "recovered" : "not_recovered");
}

double
MissionMetrics::getOpsPerJoule() const
{
    if (energyHistory.empty()) return 0;

    double totalOps = 0;
    double totalEnergy = 0;

    for (const auto &m : energyHistory) {
        totalOps += m.operations;
        totalEnergy += m.powerWatts * 1e-12;
    }

    return (totalEnergy > 0) ? totalOps / totalEnergy : 0;
}

double
MissionMetrics::getMeanTimeBetweenFailures() const
{
    if (reliabilityHistory.size() < 2) {
        return targetMTBF;  // Not enough data, return target
    }

    Tick totalTime = reliabilityHistory.back().timestamp - 
                     reliabilityHistory.front().timestamp;
    int numFailures = reliabilityHistory.size();

    return static_cast<double>(totalTime) / numFailures / 1e12;  // In seconds
}

double
MissionMetrics::getDeadlineMissRate() const
{
    if (realtimeHistory.empty()) return 0;

    int misses = 0;
    for (const auto &r : realtimeHistory) {
        if (!r.metDeadline) misses++;
    }

    return static_cast<double>(misses) / realtimeHistory.size();
}

double
MissionMetrics::getAverageRecoveryTime() const
{
    if (reliabilityHistory.empty()) return 0;

    Tick totalRecovery = 0;
    int recoveredCount = 0;

    for (const auto &e : reliabilityHistory) {
        if (e.recovered) {
            totalRecovery += e.recoveryTime;
            recoveredCount++;
        }
    }

    return (recoveredCount > 0) ? 
           static_cast<double>(totalRecovery) / recoveredCount : 0;
}

double
MissionMetrics::getThermalEfficiency() const
{
    /* Efficiency relative to nominal temperature */
    double avgTemp = 0;
    if (!temperatureHistory.empty()) {
        avgTemp = std::accumulate(temperatureHistory.begin(),
                                   temperatureHistory.end(), 0.0) /
                  temperatureHistory.size();
    } else {
        avgTemp = nominalTemperature;
    }

    return calculateThermalDerating(avgTemp);
}

double
MissionMetrics::getPhaseOpsPerJoule(MissionPhase phase) const
{
    auto opsIt = phaseOperations.find(phase);
    auto energyIt = phaseEnergy.find(phase);

    if (opsIt == phaseOperations.end() || energyIt == phaseEnergy.end()) {
        return 0;
    }

    double energy = energyIt->second;
    return (energy > 0) ? opsIt->second / energy : 0;
}

double
MissionMetrics::getPhaseDeadlineMissRate(MissionPhase phase) const
{
    auto missIt = phaseDeadlineMisses.find(phase);
    auto countIt = phaseTaskCount.find(phase);

    if (missIt == phaseDeadlineMisses.end() || 
        countIt == phaseTaskCount.end() || countIt->second == 0) {
        return 0;
    }

    return static_cast<double>(missIt->second) / countIt->second;
}

std::string
MissionMetrics::generateMissionReport() const
{
    std::ostringstream report;

    report << "========================================\n";
    report << "   MISSION PERFORMANCE REPORT\n";
    report << "   Mission: " << missionName << "\n";
    report << "========================================\n\n";

    report << "ENERGY EFFICIENCY\n";
    report << "-----------------\n";
    report << std::fixed << std::setprecision(2);
    report << "  Ops/Joule (actual):  " << getOpsPerJoule() << "\n";
    report << "  Ops/Joule (target):  " << targetOpsPerJoule << "\n";
    report << "  Efficiency vs target: " 
           << (getOpsPerJoule() / targetOpsPerJoule * 100) << "%\n\n";

    report << "RELIABILITY\n";
    report << "-----------\n";
    report << "  MTBF (actual):  " << getMeanTimeBetweenFailures() << " s\n";
    report << "  MTBF (target):  " << targetMTBF << " s\n";
    report << "  Avg recovery:   " << getAverageRecoveryTime() / 1e9 << " ms\n\n";

    report << "REAL-TIME PERFORMANCE\n";
    report << "--------------------\n";
    report << "  Deadline miss rate:  " << (getDeadlineMissRate() * 100) << "%\n";
    report << "  Max allowed:         " << (maxDeadlineMissRate * 100) << "%\n";
    report << "  Status: " << (getDeadlineMissRate() <= maxDeadlineMissRate ? 
                               "PASS" : "FAIL") << "\n\n";

    report << "THERMAL PERFORMANCE\n";
    report << "------------------\n";
    report << "  Current temp:    " << currentTemperature << " C\n";
    report << "  Max temp:        " << maxTemperature << " C\n";
    report << "  Thermal efficiency: " << (getThermalEfficiency() * 100) << "%\n\n";

    report << "PHASE BREAKDOWN\n";
    report << "---------------\n";
    const char* phaseNames[] = {"Launch", "Orbit Insertion", "Cruise", 
                                "Approach", "EDL", "Surface Ops", 
                                "Science", "Emergency"};
    for (int i = 0; i < NUM_PHASES; i++) {
        MissionPhase phase = static_cast<MissionPhase>(i);
        auto timeIt = phaseTime.find(phase);
        if (timeIt != phaseTime.end() && timeIt->second > 0) {
            report << "  " << phaseNames[i] << ":\n";
            report << "    Time: " << (timeIt->second / 1e12) << " s\n";
            report << "    Ops/J: " << getPhaseOpsPerJoule(phase) << "\n";
            report << "    DL Miss: " << (getPhaseDeadlineMissRate(phase) * 100) 
                   << "%\n";
        }
    }

    report << "\n========================================\n";

    return report.str();
}

void
MissionMetrics::updateEnergyStats(const EnergyMeasurement &m)
{
    stats.totalOperations += m.operations;
    stats.totalEnergyJoules += m.powerWatts * 1e-12;

    if (stats.totalEnergyJoules.value() > 0) {
        double opj = stats.totalOperations.value() / 
                     stats.totalEnergyJoules.value();
        stats.avgOpsPerJoule = opj;
        if (opj > stats.peakOpsPerJoule.value()) {
            stats.peakOpsPerJoule = opj;
        }
    }

    stats.phaseOpsPerJoule[currentPhase] = getPhaseOpsPerJoule(currentPhase);

    /* Update thermal stats */
    double avgTemp = 0;
    if (!temperatureHistory.empty()) {
        avgTemp = std::accumulate(temperatureHistory.begin(),
                                   temperatureHistory.end(), 0.0) /
                  temperatureHistory.size();
    }
    stats.avgTemperature = avgTemp;
    stats.maxTemperature = maxTemperature;
    stats.thermalEfficiency = getThermalEfficiency();
    stats.temperatureDist.sample(m.temperature);
}

void
MissionMetrics::updateRealtimeStats(const RealtimeTaskResult &r)
{
    stats.totalTasks++;

    if (r.metDeadline) {
        stats.deadlinesMet++;
    } else {
        stats.deadlinesMissed++;
    }

    stats.deadlineMissRate = getDeadlineMissRate();
    stats.taskLatencyDist.sample(r.completionTime);
    stats.phaseDeadlineMisses[currentPhase] = phaseDeadlineMisses[currentPhase];
}

void
MissionMetrics::updateReliabilityStats(const ReliabilityEvent &e)
{
    stats.totalErrors++;

    if (e.recovered) {
        stats.correctedErrors++;
    } else {
        stats.uncorrectedErrors++;
    }

    stats.meanTimeBetweenFailures = getMeanTimeBetweenFailures();
    stats.avgRecoveryTime = getAverageRecoveryTime();

    if (e.recoveryTime > stats.maxRecoveryTime.value()) {
        stats.maxRecoveryTime = e.recoveryTime;
    }
}

double
MissionMetrics::calculateThermalDerating(double temperature) const
{
    /* Simple thermal derating model:
     * 100% efficiency at nominal temperature
     * Linear decrease above nominal
     * Slight decrease below nominal (cold effects)
     */
    double efficiency = 1.0;

    if (temperature > nominalTemperature) {
        /* High temperature derating: 1% per degree above nominal */
        double delta = temperature - nominalTemperature;
        efficiency = std::max(0.5, 1.0 - 0.01 * delta);
    } else if (temperature < nominalTemperature - 20) {
        /* Cold temperature effect: slight decrease */
        efficiency = 0.95;
    }

    return efficiency;
}

MissionMetrics::MissionMetricsStats::MissionMetricsStats(MissionMetrics *parent)
    : statistics::Group(parent),
      ADD_STAT(totalOperations, statistics::units::Count::get(),
               "Total operations"),
      ADD_STAT(totalEnergyJoules, statistics::units::Count::get(),
               "Total energy (Joules)"),
      ADD_STAT(avgOpsPerJoule, statistics::units::Ratio::get(),
               "Average ops per Joule"),
      ADD_STAT(peakOpsPerJoule, statistics::units::Ratio::get(),
               "Peak ops per Joule"),
      ADD_STAT(phaseOpsPerJoule, statistics::units::Ratio::get(),
               "Ops per Joule by phase"),
      ADD_STAT(totalErrors, statistics::units::Count::get(),
               "Total error events"),
      ADD_STAT(correctedErrors, statistics::units::Count::get(),
               "Corrected errors"),
      ADD_STAT(uncorrectedErrors, statistics::units::Count::get(),
               "Uncorrected errors"),
      ADD_STAT(meanTimeBetweenFailures, statistics::units::Second::get(),
               "Mean time between failures"),
      ADD_STAT(avgRecoveryTime, statistics::units::Tick::get(),
               "Average recovery time"),
      ADD_STAT(maxRecoveryTime, statistics::units::Tick::get(),
               "Maximum recovery time"),
      ADD_STAT(errorsByComponent, statistics::units::Count::get(),
               "Errors by component"),
      ADD_STAT(totalTasks, statistics::units::Count::get(),
               "Total real-time tasks"),
      ADD_STAT(deadlinesMet, statistics::units::Count::get(),
               "Deadlines met"),
      ADD_STAT(deadlinesMissed, statistics::units::Count::get(),
               "Deadlines missed"),
      ADD_STAT(deadlineMissRate, statistics::units::Ratio::get(),
               "Deadline miss rate"),
      ADD_STAT(taskLatencyDist, statistics::units::Tick::get(),
               "Task latency distribution"),
      ADD_STAT(phaseDeadlineMisses, statistics::units::Count::get(),
               "Deadline misses by phase"),
      ADD_STAT(avgTemperature, statistics::units::Count::get(),
               "Average temperature (C)"),
      ADD_STAT(maxTemperature, statistics::units::Count::get(),
               "Maximum temperature (C)"),
      ADD_STAT(thermalThrottleTime, statistics::units::Tick::get(),
               "Time spent thermal throttling"),
      ADD_STAT(thermalEfficiency, statistics::units::Ratio::get(),
               "Thermal efficiency"),
      ADD_STAT(temperatureDist, statistics::units::Count::get(),
               "Temperature distribution"),
      ADD_STAT(phaseTime, statistics::units::Tick::get(),
               "Time per phase"),
      ADD_STAT(currentPhase, statistics::units::Count::get(),
               "Current mission phase")
{
    phaseOpsPerJoule.init(NUM_PHASES);
    errorsByComponent.init(8);  // Up to 8 components
    taskLatencyDist.init(20);
    phaseDeadlineMisses.init(NUM_PHASES);
    temperatureDist.init(20);
    phaseTime.init(NUM_PHASES);

    const char* phaseNames[] = {"launch", "orbit_insert", "cruise",
                                "approach", "edl", "surface", 
                                "science", "emergency"};
    for (int i = 0; i < NUM_PHASES; i++) {
        phaseOpsPerJoule.subname(i, phaseNames[i]);
        phaseDeadlineMisses.subname(i, phaseNames[i]);
        phaseTime.subname(i, phaseNames[i]);
    }
}

} // namespace gem5

