/*
 * Energy-Proportional Accelerator Design
 * PhD Research: Chandraboul
 * 
 * Research Question 5: Energy-Proportional Accelerator Design
 * - Dynamic voltage-frequency scaling (DVFS) based on power availability
 * - "Dark silicon" management: how many accelerators can run simultaneously
 * - Performance-per-watt characterization
 * - Solar panel orientation and battery state awareness
 */

#ifndef __CUSTOM_ACCEL_ENERGY_PROPORTIONAL_HH__
#define __CUSTOM_ACCEL_ENERGY_PROPORTIONAL_HH__

#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/EnergyProportionalManager.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <vector>
#include <map>
#include <functional>
#include <deque>

namespace gem5
{

/**
 * EnergyProportionalManager - Power-aware accelerator management
 * 
 * Features:
 * 1. Dynamic voltage-frequency scaling (DVFS)
 * 2. Power budgeting and distribution
 * 3. Dark silicon management
 * 4. Battery/solar awareness
 * 5. Thermal management
 */
class EnergyProportionalManager : public ClockedObject
{
  public:
    PARAMS(EnergyProportionalManager);
    EnergyProportionalManager(const Params &p);
    ~EnergyProportionalManager();

    /* Power states for accelerators */
    enum PowerState {
        STATE_OFF,           // Completely off (0 power)
        STATE_SLEEP,         // Deep sleep (minimal power)
        STATE_IDLE,          // Idle but ready
        STATE_LOW_POWER,     // Reduced VF operation
        STATE_NOMINAL,       // Normal operation
        STATE_TURBO          // Maximum performance (if power available)
    };

    /* Voltage-frequency operating points */
    struct VFPoint {
        double voltage;      // Volts
        double frequency;    // MHz
        double power;        // Watts at this point
        double performance;  // Relative performance (0.0-1.0)
    };

    /* Power source types */
    enum PowerSource {
        SOURCE_SOLAR,
        SOURCE_BATTERY,
        SOURCE_RTG,          // Radioisotope thermoelectric generator
        SOURCE_COMBINATION
    };

    /* Orbital conditions affecting power */
    struct OrbitalConditions {
        double solarIncidence;    // 0.0 (eclipse) to 1.0 (full sun)
        double panelOrientation;  // Effectiveness of solar panels
        double batteryLevel;      // 0.0 to 1.0
        double thermalState;      // Temperature factor
        bool inEclipse;
    };

    /* Accelerator power profile */
    struct AcceleratorPowerProfile {
        int id;
        std::string name;
        PowerState currentState;
        VFPoint currentVF;
        double maxPower;
        double currentPower;
        std::vector<VFPoint> vfPoints;  // Available operating points
        bool canPowerGate;              // Supports complete shutdown
        int priority;                   // Higher = more important
        Cycles wakeupLatency;           // Time to wake from sleep
    };

    /* Set orbital/power conditions */
    void setOrbitalConditions(const OrbitalConditions &conditions);
    OrbitalConditions getOrbitalConditions() const { return currentConditions; }

    /* Register accelerator with power manager */
    void registerAccelerator(int id, const std::string &name,
                            double maxPower, int priority,
                            bool canPowerGate, Cycles wakeupLatency,
                            std::function<void(PowerState, VFPoint)> callback);

    /* Request power for operation */
    bool requestPower(int accelId, double requestedPower);
    void releasePower(int accelId);

    /* Set accelerator power state */
    bool setPowerState(int accelId, PowerState state);
    PowerState getPowerState(int accelId) const;

    /* Get current power budget */
    double getAvailablePower() const;
    double getTotalPowerBudget() const { return totalPowerBudget; }
    double getCurrentPowerUsage() const;

    /* DVFS control */
    bool setVFPoint(int accelId, int vfIndex);
    std::vector<VFPoint> getVFPoints(int accelId) const;

    /* Dark silicon management */
    int getMaxSimultaneousAccelerators() const;
    std::vector<int> getActiveAccelerators() const;
    bool canActivateAccelerator(int accelId) const;

    /* Performance-per-watt metrics */
    double getPerformancePerWatt(int accelId) const;
    double getSystemPerformancePerWatt() const;

  private:
    /* Parameters */
    double totalPowerBudget;         // Watts
    double basePowerConsumption;     // System overhead
    double solarPanelCapacity;       // Maximum solar power
    double batteryCapacity;          // Watt-hours
    double thermalLimit;             // Maximum thermal dissipation
    bool enableDVFS;
    bool enablePowerGating;
    Cycles dvfsTransitionCycles;

    /* State */
    OrbitalConditions currentConditions;
    std::vector<AcceleratorPowerProfile> accelerators;
    std::map<int, std::function<void(PowerState, VFPoint)>> accelCallbacks;
    double currentTotalPower;
    int activeAccelCount;

    /* Power budget calculation */
    double calculateAvailablePower() const;
    double calculateSolarPower() const;
    double calculateBatteryPower() const;

    /* DVFS logic */
    VFPoint selectOptimalVFPoint(int accelId, double powerAllocation);
    void applyVFPoint(int accelId, const VFPoint &vf);
    void adjustAllVFPoints();

    /* Dark silicon management */
    void enforcePowerBudget();
    std::vector<int> selectAcceleratorsForBudget(double budget);
    void powerGateAccelerator(int accelId);
    void wakeAccelerator(int accelId);

    /* Thermal management */
    double estimateThermal() const;
    bool isWithinThermalLimits() const;
    void applyThermalThrottling();

    /* Events */
    EventFunctionWrapper powerUpdateEvent;
    EventFunctionWrapper thermalCheckEvent;
    void schedulePowerUpdate();
    void scheduleThermalCheck();
    void handlePowerUpdate();
    void handleThermalCheck();

    /* Statistics */
    struct EnergyStats : public statistics::Group
    {
        EnergyStats(EnergyProportionalManager *parent);

        statistics::Scalar totalEnergyConsumed;
        statistics::Scalar solarEnergyHarvested;
        statistics::Scalar batteryEnergyUsed;
        statistics::Vector perAccelEnergy;
        statistics::Vector perAccelActiveTime;
        statistics::Scalar avgPowerUsage;
        statistics::Scalar peakPowerUsage;
        statistics::Scalar powerGatingEvents;
        statistics::Scalar dvfsTransitions;
        statistics::Scalar thermalThrottleEvents;
        statistics::Scalar eclipseTime;
        statistics::Scalar sunTime;
        statistics::Histogram powerDistribution;
        statistics::Scalar performancePerWatt;
        statistics::Scalar darkSiliconRatio;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_ENERGY_PROPORTIONAL_HH__

