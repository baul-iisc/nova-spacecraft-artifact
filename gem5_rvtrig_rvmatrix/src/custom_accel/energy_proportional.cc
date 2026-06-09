/*
 * Energy-Proportional Manager Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/energy_proportional.hh"
#include "base/trace.hh"
#include "debug/EnergyProportional.hh"
#include "params/ClockedObject.hh"

#include <algorithm>
#include <numeric>
#include <cmath>

namespace gem5
{

EnergyProportionalManager::EnergyProportionalManager(const Params &p)
    : ClockedObject(dynamic_cast<const ClockedObjectParams &>(p)),
      totalPowerBudget(p.total_power_budget),
      basePowerConsumption(p.base_power_consumption),
      solarPanelCapacity(p.solar_panel_capacity),
      batteryCapacity(p.battery_capacity),
      thermalLimit(p.thermal_limit),
      enableDVFS(p.enable_dvfs),
      enablePowerGating(p.enable_power_gating),
      dvfsTransitionCycles(p.dvfs_transition_cycles),
      currentTotalPower(basePowerConsumption),
      activeAccelCount(0),
      powerUpdateEvent([this]{ handlePowerUpdate(); }, name() + ".powerUpdate"),
      thermalCheckEvent([this]{ handleThermalCheck(); }, name() + ".thermalCheck"),
      stats(this)
{
    /* Initialize orbital conditions (default: in sun, full battery) */
    currentConditions.solarIncidence = 1.0;
    currentConditions.panelOrientation = 0.8;
    currentConditions.batteryLevel = 1.0;
    currentConditions.thermalState = 1.0;
    currentConditions.inEclipse = false;

    /* Schedule periodic updates */
    schedulePowerUpdate();
    scheduleThermalCheck();

    DPRINTF(EnergyProportional, "EnergyProportionalManager created: "
            "budget=%.2fW, solar=%.2fW, battery=%.2fWh\n",
            totalPowerBudget, solarPanelCapacity, batteryCapacity);
}

EnergyProportionalManager::~EnergyProportionalManager()
{
}

void
EnergyProportionalManager::setOrbitalConditions(const OrbitalConditions &conditions)
{
    bool wasInEclipse = currentConditions.inEclipse;
    currentConditions = conditions;

    /* Track eclipse/sun time */
    if (conditions.inEclipse && !wasInEclipse) {
        DPRINTF(EnergyProportional, "Entering eclipse\n");
    } else if (!conditions.inEclipse && wasInEclipse) {
        DPRINTF(EnergyProportional, "Exiting eclipse\n");
    }

    /* Update power budget based on new conditions */
    enforcePowerBudget();

    DPRINTF(EnergyProportional, "Orbital conditions: solar=%.2f orient=%.2f "
            "battery=%.2f eclipse=%d\n", conditions.solarIncidence,
            conditions.panelOrientation, conditions.batteryLevel,
            conditions.inEclipse);
}

void
EnergyProportionalManager::registerAccelerator(int id, const std::string &accName,
    double maxPower, int priority, bool canPowerGate, Cycles wakeupLatency,
    std::function<void(PowerState, VFPoint)> callback)
{
    AcceleratorPowerProfile profile;
    profile.id = id;
    profile.name = accName;
    profile.currentState = STATE_IDLE;
    profile.maxPower = maxPower;
    profile.currentPower = 0;
    profile.canPowerGate = canPowerGate;
    profile.priority = priority;
    profile.wakeupLatency = wakeupLatency;

    /* Create default VF points */
    VFPoint vf;
    
    /* Off */
    vf = {0.0, 0.0, 0.0, 0.0};
    profile.vfPoints.push_back(vf);
    
    /* Low power (50% performance, 25% power) */
    vf = {0.7, 200, maxPower * 0.25, 0.5};
    profile.vfPoints.push_back(vf);
    
    /* Nominal (100% performance, 100% power) */
    vf = {1.0, 400, maxPower, 1.0};
    profile.vfPoints.push_back(vf);
    
    /* Turbo (120% performance, 150% power) */
    vf = {1.1, 500, maxPower * 1.5, 1.2};
    profile.vfPoints.push_back(vf);

    profile.currentVF = profile.vfPoints[2];  // Start at nominal

    accelerators.push_back(profile);
    accelCallbacks[id] = callback;

    DPRINTF(EnergyProportional, "Registered accelerator: %s (id=%d) "
            "maxPower=%.2fW priority=%d\n", accName.c_str(), id, maxPower, priority);
}

bool
EnergyProportionalManager::requestPower(int accelId, double requestedPower)
{
    /* Check if power is available */
    double available = getAvailablePower();
    if (requestedPower > available) {
        DPRINTF(EnergyProportional, "Power request denied: requested=%.2fW "
                "available=%.2fW\n", requestedPower, available);
        stats.powerGatingEvents++;
        return false;
    }

    /* Allocate power */
    for (auto &accel : accelerators) {
        if (accel.id == accelId) {
            accel.currentPower = requestedPower;
            currentTotalPower += requestedPower;
            activeAccelCount++;

            /* Select appropriate VF point */
            VFPoint vf = selectOptimalVFPoint(accelId, requestedPower);
            applyVFPoint(accelId, vf);

            DPRINTF(EnergyProportional, "Power allocated: accel=%d power=%.2fW\n",
                    accelId, requestedPower);
            return true;
        }
    }

    return false;
}

void
EnergyProportionalManager::releasePower(int accelId)
{
    for (auto &accel : accelerators) {
        if (accel.id == accelId && accel.currentPower > 0) {
            currentTotalPower -= accel.currentPower;
            accel.currentPower = 0;
            activeAccelCount--;

            DPRINTF(EnergyProportional, "Power released: accel=%d\n", accelId);
            return;
        }
    }
}

bool
EnergyProportionalManager::setPowerState(int accelId, PowerState state)
{
    for (auto &accel : accelerators) {
        if (accel.id == accelId) {
            PowerState oldState = accel.currentState;

            /* Check if transition is valid */
            if (state == STATE_TURBO) {
                if (getAvailablePower() < accel.maxPower * 0.5) {
                    DPRINTF(EnergyProportional, "Cannot enter turbo: "
                            "insufficient power\n");
                    return false;
                }
            }

            accel.currentState = state;

            /* Apply appropriate VF point */
            int vfIndex = 0;
            switch (state) {
                case STATE_OFF:
                case STATE_SLEEP:
                    vfIndex = 0;
                    break;
                case STATE_IDLE:
                case STATE_LOW_POWER:
                    vfIndex = 1;
                    break;
                case STATE_NOMINAL:
                    vfIndex = 2;
                    break;
                case STATE_TURBO:
                    vfIndex = 3;
                    break;
            }

            if (vfIndex < (int)accel.vfPoints.size()) {
                applyVFPoint(accelId, accel.vfPoints[vfIndex]);
            }

            DPRINTF(EnergyProportional, "Power state change: accel=%d %d->%d\n",
                    accelId, oldState, state);
            return true;
        }
    }
    return false;
}

EnergyProportionalManager::PowerState
EnergyProportionalManager::getPowerState(int accelId) const
{
    for (const auto &accel : accelerators) {
        if (accel.id == accelId) {
            return accel.currentState;
        }
    }
    return STATE_OFF;
}

double
EnergyProportionalManager::getAvailablePower() const
{
    return calculateAvailablePower() - currentTotalPower;
}

double
EnergyProportionalManager::getCurrentPowerUsage() const
{
    return currentTotalPower;
}

bool
EnergyProportionalManager::setVFPoint(int accelId, int vfIndex)
{
    for (auto &accel : accelerators) {
        if (accel.id == accelId) {
            if (vfIndex >= 0 && vfIndex < (int)accel.vfPoints.size()) {
                VFPoint &vf = accel.vfPoints[vfIndex];

                /* Check power availability */
                double powerDelta = vf.power - accel.currentPower;
                if (powerDelta > 0 && powerDelta > getAvailablePower()) {
                    return false;
                }

                applyVFPoint(accelId, vf);
                stats.dvfsTransitions++;
                return true;
            }
        }
    }
    return false;
}

std::vector<EnergyProportionalManager::VFPoint>
EnergyProportionalManager::getVFPoints(int accelId) const
{
    for (const auto &accel : accelerators) {
        if (accel.id == accelId) {
            return accel.vfPoints;
        }
    }
    return std::vector<VFPoint>();
}

int
EnergyProportionalManager::getMaxSimultaneousAccelerators() const
{
    double available = calculateAvailablePower();
    double avgPower = 0;
    int count = 0;

    for (const auto &accel : accelerators) {
        avgPower += accel.maxPower;
        count++;
    }

    if (count == 0) return 0;
    avgPower /= count;

    return static_cast<int>(available / avgPower);
}

std::vector<int>
EnergyProportionalManager::getActiveAccelerators() const
{
    std::vector<int> active;
    for (const auto &accel : accelerators) {
        if (accel.currentState != STATE_OFF && accel.currentState != STATE_SLEEP) {
            active.push_back(accel.id);
        }
    }
    return active;
}

bool
EnergyProportionalManager::canActivateAccelerator(int accelId) const
{
    for (const auto &accel : accelerators) {
        if (accel.id == accelId) {
            /* Check if enough power available for minimum operation */
            double minPower = accel.vfPoints[1].power;  // Low power point
            return getAvailablePower() >= minPower;
        }
    }
    return false;
}

double
EnergyProportionalManager::getPerformancePerWatt(int accelId) const
{
    for (const auto &accel : accelerators) {
        if (accel.id == accelId) {
            if (accel.currentPower > 0) {
                return accel.currentVF.performance / accel.currentPower;
            }
        }
    }
    return 0.0;
}

double
EnergyProportionalManager::getSystemPerformancePerWatt() const
{
    double totalPerformance = 0;
    double totalPower = basePowerConsumption;

    for (const auto &accel : accelerators) {
        if (accel.currentPower > 0) {
            totalPerformance += accel.currentVF.performance;
            totalPower += accel.currentPower;
        }
    }

    return (totalPower > 0) ? (totalPerformance / totalPower) : 0.0;
}

double
EnergyProportionalManager::calculateAvailablePower() const
{
    double solar = calculateSolarPower();
    double battery = calculateBatteryPower();

    /* Use solar when available, battery as backup */
    double available = solar;
    if (currentConditions.inEclipse || solar < totalPowerBudget * 0.5) {
        available += battery;
    }

    /* Apply thermal limit */
    if (!isWithinThermalLimits()) {
        available *= 0.7;  // Reduce for thermal throttling
    }

    return std::min(available, totalPowerBudget);
}

double
EnergyProportionalManager::calculateSolarPower() const
{
    if (currentConditions.inEclipse) {
        return 0.0;
    }

    return solarPanelCapacity * 
           currentConditions.solarIncidence * 
           currentConditions.panelOrientation;
}

double
EnergyProportionalManager::calculateBatteryPower() const
{
    /* Limit battery draw based on level */
    double maxDraw = totalPowerBudget * 0.5;  // Max 50% from battery
    
    /* Reduce draw as battery depletes */
    if (currentConditions.batteryLevel < 0.3) {
        maxDraw *= currentConditions.batteryLevel / 0.3;
    }

    return maxDraw;
}

EnergyProportionalManager::VFPoint
EnergyProportionalManager::selectOptimalVFPoint(int accelId, double powerAllocation)
{
    for (const auto &accel : accelerators) {
        if (accel.id == accelId) {
            /* Find highest performance VF point within power budget */
            VFPoint best = accel.vfPoints[0];
            for (const auto &vf : accel.vfPoints) {
                if (vf.power <= powerAllocation && vf.performance > best.performance) {
                    best = vf;
                }
            }
            return best;
        }
    }

    VFPoint empty = {0, 0, 0, 0};
    return empty;
}

void
EnergyProportionalManager::applyVFPoint(int accelId, const VFPoint &vf)
{
    for (auto &accel : accelerators) {
        if (accel.id == accelId) {
            /* Update power tracking */
            currentTotalPower -= accel.currentPower;
            accel.currentPower = vf.power;
            currentTotalPower += vf.power;
            accel.currentVF = vf;

            /* Notify accelerator */
            auto it = accelCallbacks.find(accelId);
            if (it != accelCallbacks.end() && it->second) {
                it->second(accel.currentState, vf);
            }

            DPRINTF(EnergyProportional, "Applied VF point: accel=%d "
                    "V=%.2f F=%.0fMHz P=%.2fW perf=%.2f\n",
                    accelId, vf.voltage, vf.frequency, vf.power, vf.performance);
            return;
        }
    }
}

void
EnergyProportionalManager::adjustAllVFPoints()
{
    double available = getAvailablePower();

    if (available < 0) {
        /* Need to reduce power consumption */
        DPRINTF(EnergyProportional, "Power budget exceeded, adjusting VF points\n");

        /* Sort by priority (lower first to reduce) */
        std::vector<AcceleratorPowerProfile*> sorted;
        for (auto &accel : accelerators) {
            if (accel.currentPower > 0) {
                sorted.push_back(&accel);
            }
        }
        std::sort(sorted.begin(), sorted.end(),
                  [](const AcceleratorPowerProfile *a, const AcceleratorPowerProfile *b) {
                      return a->priority < b->priority;
                  });

        /* Reduce power from low priority accelerators */
        for (auto *accel : sorted) {
            if (available >= 0) break;

            /* Find lower VF point */
            for (int i = accel->vfPoints.size() - 1; i >= 0; i--) {
                if (accel->vfPoints[i].power < accel->currentPower) {
                    double saved = accel->currentPower - accel->vfPoints[i].power;
                    applyVFPoint(accel->id, accel->vfPoints[i]);
                    available += saved;
                    stats.dvfsTransitions++;
                    break;
                }
            }
        }
    }
}

void
EnergyProportionalManager::enforcePowerBudget()
{
    double available = calculateAvailablePower();

    if (currentTotalPower > available) {
        adjustAllVFPoints();

        /* If still over budget, power gate low priority accelerators */
        if (currentTotalPower > available && enablePowerGating) {
            for (auto &accel : accelerators) {
                if (currentTotalPower <= available) break;
                if (accel.canPowerGate && accel.priority < 3) {
                    powerGateAccelerator(accel.id);
                }
            }
        }
    }
}

std::vector<int>
EnergyProportionalManager::selectAcceleratorsForBudget(double budget)
{
    std::vector<int> selected;

    /* Sort by priority (higher first) */
    std::vector<AcceleratorPowerProfile*> sorted;
    for (auto &accel : accelerators) {
        sorted.push_back(&accel);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const AcceleratorPowerProfile *a, const AcceleratorPowerProfile *b) {
                  return a->priority > b->priority;
              });

    double remaining = budget - basePowerConsumption;
    for (auto *accel : sorted) {
        double minPower = accel->vfPoints[1].power;  // Low power point
        if (remaining >= minPower) {
            selected.push_back(accel->id);
            remaining -= minPower;
        }
    }

    return selected;
}

void
EnergyProportionalManager::powerGateAccelerator(int accelId)
{
    for (auto &accel : accelerators) {
        if (accel.id == accelId && accel.canPowerGate) {
            currentTotalPower -= accel.currentPower;
            accel.currentPower = 0;
            accel.currentState = STATE_OFF;
            activeAccelCount--;
            stats.powerGatingEvents++;

            DPRINTF(EnergyProportional, "Power gated accelerator %d\n", accelId);
            return;
        }
    }
}

void
EnergyProportionalManager::wakeAccelerator(int accelId)
{
    for (auto &accel : accelerators) {
        if (accel.id == accelId && accel.currentState == STATE_OFF) {
            accel.currentState = STATE_IDLE;
            /* Power will be allocated when work is requested */
            DPRINTF(EnergyProportional, "Waking accelerator %d\n", accelId);
            return;
        }
    }
}

double
EnergyProportionalManager::estimateThermal() const
{
    /* Simple thermal model: power * thermal resistance */
    double thermalResistance = 10.0;  // °C/W
    double ambientTemp = 25.0;  // °C
    
    return ambientTemp + currentTotalPower * thermalResistance * 
           (1.0 / currentConditions.thermalState);
}

bool
EnergyProportionalManager::isWithinThermalLimits() const
{
    return estimateThermal() < thermalLimit;
}

void
EnergyProportionalManager::applyThermalThrottling()
{
    if (!isWithinThermalLimits()) {
        DPRINTF(EnergyProportional, "Thermal throttling activated\n");
        stats.thermalThrottleEvents++;
        adjustAllVFPoints();
    }
}

void
EnergyProportionalManager::schedulePowerUpdate()
{
    if (!powerUpdateEvent.scheduled()) {
        schedule(powerUpdateEvent, clockEdge(Cycles(1000)));
    }
}

void
EnergyProportionalManager::scheduleThermalCheck()
{
    if (!thermalCheckEvent.scheduled()) {
        schedule(thermalCheckEvent, clockEdge(Cycles(10000)));
    }
}

void
EnergyProportionalManager::handlePowerUpdate()
{
    /* Update energy statistics */
    double power = currentTotalPower;
    stats.avgPowerUsage = (stats.avgPowerUsage.value() * 0.99 + power * 0.01);
    if (power > stats.peakPowerUsage.value()) {
        stats.peakPowerUsage = power;
    }

    /* Track eclipse time */
    if (currentConditions.inEclipse) {
        stats.eclipseTime++;
    } else {
        stats.sunTime++;
        stats.solarEnergyHarvested += calculateSolarPower() / 1000.0;  // Wh
    }

    /* Track per-accelerator energy */
    for (size_t i = 0; i < accelerators.size() && i < 16; i++) {
        stats.perAccelEnergy[i] += accelerators[i].currentPower / 1000.0;
        if (accelerators[i].currentPower > 0) {
            stats.perAccelActiveTime[i]++;
        }
    }

    stats.totalEnergyConsumed += power / 1000.0;  // Convert to Wh equivalent
    stats.powerDistribution.sample(power);
    stats.performancePerWatt = getSystemPerformancePerWatt();

    /* Calculate dark silicon ratio */
    int maxAccel = accelerators.size();
    if (maxAccel > 0) {
        stats.darkSiliconRatio = 1.0 - (double)activeAccelCount / maxAccel;
    }

    enforcePowerBudget();
    schedulePowerUpdate();
}

void
EnergyProportionalManager::handleThermalCheck()
{
    applyThermalThrottling();
    scheduleThermalCheck();
}

EnergyProportionalManager::EnergyStats::EnergyStats(
    EnergyProportionalManager *parent)
    : statistics::Group(parent),
      ADD_STAT(totalEnergyConsumed, statistics::units::Count::get(),
               "Total energy consumed (Wh equivalent)"),
      ADD_STAT(solarEnergyHarvested, statistics::units::Count::get(),
               "Solar energy harvested (Wh)"),
      ADD_STAT(batteryEnergyUsed, statistics::units::Count::get(),
               "Battery energy used (Wh)"),
      ADD_STAT(perAccelEnergy, statistics::units::Count::get(),
               "Energy per accelerator"),
      ADD_STAT(perAccelActiveTime, statistics::units::Cycle::get(),
               "Active time per accelerator"),
      ADD_STAT(avgPowerUsage, statistics::units::Count::get(),
               "Average power usage (W)"),
      ADD_STAT(peakPowerUsage, statistics::units::Count::get(),
               "Peak power usage (W)"),
      ADD_STAT(powerGatingEvents, statistics::units::Count::get(),
               "Power gating events"),
      ADD_STAT(dvfsTransitions, statistics::units::Count::get(),
               "DVFS transitions"),
      ADD_STAT(thermalThrottleEvents, statistics::units::Count::get(),
               "Thermal throttle events"),
      ADD_STAT(eclipseTime, statistics::units::Cycle::get(),
               "Time in eclipse"),
      ADD_STAT(sunTime, statistics::units::Cycle::get(),
               "Time in sun"),
      ADD_STAT(powerDistribution, statistics::units::Count::get(),
               "Power usage distribution"),
      ADD_STAT(performancePerWatt, statistics::units::Ratio::get(),
               "System performance per watt"),
      ADD_STAT(darkSiliconRatio, statistics::units::Ratio::get(),
               "Dark silicon ratio")
{
    perAccelEnergy.init(16);
    perAccelActiveTime.init(16);
    powerDistribution.init(20);

    for (int i = 0; i < 16; i++) {
        perAccelEnergy.subname(i, std::string("accel") + std::to_string(i));
        perAccelActiveTime.subname(i, std::string("accel") + std::to_string(i));
    }
}

} // namespace gem5

