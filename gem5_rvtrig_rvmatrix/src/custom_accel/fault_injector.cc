/*
 * Fault Injection Framework Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/fault_injector.hh"
#include "debug/FaultInjector.hh"
#include "sim/system.hh"

namespace gem5
{

FaultInjector::FaultInjector(const Params &p)
    : ClockedObject(p),
      faultRate(p.fault_rate),
      enableRandomFaults(p.enable_random_faults),
      maxFaults(p.max_faults),
      watchdogTimeout(p.watchdog_timeout),
      enableRecovery(p.enable_recovery),
      injectedFaults(0),
      random(p.seed),
      randomFaultEvent([this]{ processRandomFault(); }, name() + ".randomFault"),
      watchdogCheckEvent([this]{ checkWatchdog(); }, name() + ".watchdogCheck"),
      stats(this)
{
    DPRINTF(FaultInjector, "FaultInjector created: rate=%f faults/Mcycles, "
            "random=%d, max=%d\n", faultRate, enableRandomFaults, maxFaults);

    if (enableRandomFaults && faultRate > 0) {
        scheduleRandomFault();
    }

    // Schedule periodic watchdog checks
    schedule(watchdogCheckEvent, clockEdge(watchdogTimeout));
}

FaultInjector::~FaultInjector()
{
}

void
FaultInjector::registerAccelerator(const std::string &accelName, int id,
    std::function<void(FaultType, int, uint64_t*)> callback)
{
    AccelInfo info;
    info.name = accelName;
    info.id = id;
    info.callback = callback;
    info.watchdogActive = false;
    info.watchdogStart = 0;
    info.lastActivity = 0;
    
    if ((size_t)id >= accelerators.size()) {
        accelerators.resize(id + 1);
    }
    accelerators[id] = info;

    DPRINTF(FaultInjector, "Registered accelerator: %s (id=%d)\n", 
            info.name.c_str(), id);
}

void
FaultInjector::injectFault(int accelId, FaultType type, int bitPosition)
{
    if (accelId < 0 || (size_t)accelId >= accelerators.size()) {
        DPRINTF(FaultInjector, "Invalid accelerator ID: %d\n", accelId);
        return;
    }

    if (maxFaults >= 0 && injectedFaults >= maxFaults) {
        DPRINTF(FaultInjector, "Max faults reached (%d)\n", maxFaults);
        return;
    }

    FaultEvent fault;
    fault.injectionTime = curTick();
    fault.type = type;
    fault.targetAccel = accelId;
    fault.targetCore = random.random<int>() % 8;  // Random core
    fault.originalValue = 0;
    fault.bitPosition = bitPosition >= 0 ? bitPosition : getRandomBitPosition(64);
    fault.detected = false;
    fault.recovered = false;
    fault.detectionLatency = 0;

    // Apply the fault via callback
    if (accelerators[accelId].callback) {
        accelerators[accelId].callback(type, fault.bitPosition, 
                                       &fault.corruptedValue);
    }

    faultHistory.push_back(fault);
    injectedFaults++;
    stats.totalFaultsInjected++;

    switch (type) {
        case FAULT_BITFLIP_CMD:
        case FAULT_BITFLIP_ADDR:
        case FAULT_BITFLIP_DATA:
            stats.bitflipFaults++;
            break;
        case FAULT_HANG:
            stats.hangFaults++;
            break;
        case FAULT_TIMEOUT:
            stats.timeoutFaults++;
            break;
        default:
            break;
    }

    stats.perAccelFaults[accelId]++;

    DPRINTF(FaultInjector, "FAULT INJECTED: accel=%s type=%d bit=%d "
            "value=0x%lx\n", accelerators[accelId].name.c_str(),
            type, fault.bitPosition, fault.corruptedValue);
}

void
FaultInjector::scheduleRandomFault()
{
    if (!enableRandomFaults || faultRate <= 0) return;

    // Calculate next fault time based on Poisson process
    // Mean time between faults = 1M / faultRate cycles
    double meanInterval = 1000000.0 / faultRate;
    double u = random.random<double>();
    Tick interval = (Tick)(-meanInterval * log(u)) * clockPeriod();

    if (interval < clockPeriod()) {
        interval = clockPeriod();
    }

    if (!randomFaultEvent.scheduled()) {
        schedule(randomFaultEvent, curTick() + interval);
    }
}

void
FaultInjector::processRandomFault()
{
    if (accelerators.empty()) {
        scheduleRandomFault();
        return;
    }

    // Pick random accelerator and fault type
    int accelId = random.random<int>() % accelerators.size();
    FaultType type = getRandomFaultType();

    injectFault(accelId, type);
    scheduleRandomFault();
}

void
FaultInjector::startWatchdog(int accelId, Tick timeout)
{
    if (accelId < 0 || (size_t)accelId >= accelerators.size()) return;

    accelerators[accelId].watchdogActive = true;
    accelerators[accelId].watchdogStart = curTick();
    accelerators[accelId].lastActivity = curTick();

    DPRINTF(FaultInjector, "Watchdog started for %s (timeout=%lu)\n",
            accelerators[accelId].name.c_str(), timeout);
}

void
FaultInjector::petWatchdog(int accelId)
{
    if (accelId < 0 || (size_t)accelId >= accelerators.size()) return;

    if (accelerators[accelId].watchdogActive) {
        accelerators[accelId].lastActivity = curTick();
    }
}

void
FaultInjector::cancelWatchdog(int accelId)
{
    if (accelId < 0 || (size_t)accelId >= accelerators.size()) return;

    accelerators[accelId].watchdogActive = false;
    DPRINTF(FaultInjector, "Watchdog cancelled for %s\n",
            accelerators[accelId].name.c_str());
}

void
FaultInjector::checkWatchdog()
{
    for (auto &accel : accelerators) {
        if (accel.watchdogActive) {
            Tick elapsed = curTick() - accel.lastActivity;
            if (elapsed > clockPeriod() * watchdogTimeout) {
                // Watchdog triggered - accelerator appears hung
                DPRINTF(FaultInjector, "WATCHDOG TRIGGERED: %s hung for %lu cycles\n",
                        accel.name.c_str(), elapsed / clockPeriod());

                // Report as detected hang
                reportFaultDetected(accel.id, FAULT_HANG, elapsed);

                // Attempt recovery if enabled
                if (enableRecovery && accel.callback) {
                    DPRINTF(FaultInjector, "Attempting recovery for %s\n",
                            accel.name.c_str());
                    uint64_t resetCmd = 0xFFFFFFFF;  // Reset command
                    accel.callback(FAULT_NONE, -1, &resetCmd);
                    reportFaultRecovered(accel.id, FAULT_HANG);
                }

                accel.watchdogActive = false;
            }
        }
    }

    // Reschedule periodic check
    schedule(watchdogCheckEvent, clockEdge(watchdogTimeout));
}

void
FaultInjector::reportFaultDetected(int accelId, FaultType type, Tick latency)
{
    stats.faultsDetected++;
    stats.detectionLatency.sample(latency / clockPeriod());

    // Mark in history
    for (auto it = faultHistory.rbegin(); it != faultHistory.rend(); ++it) {
        if (it->targetAccel == accelId && !it->detected) {
            it->detected = true;
            it->detectionLatency = latency;
            break;
        }
    }

    DPRINTF(FaultInjector, "Fault detected: accel=%d type=%d latency=%lu\n",
            accelId, type, latency / clockPeriod());
}

void
FaultInjector::reportFaultRecovered(int accelId, FaultType type)
{
    stats.faultsRecovered++;

    // Mark in history
    for (auto it = faultHistory.rbegin(); it != faultHistory.rend(); ++it) {
        if (it->targetAccel == accelId && it->detected && !it->recovered) {
            it->recovered = true;
            break;
        }
    }

    DPRINTF(FaultInjector, "Fault recovered: accel=%d type=%d\n", accelId, type);
}

uint64_t
FaultInjector::flipBit(uint64_t value, int bitPos)
{
    if (bitPos < 0 || bitPos > 63) return value;
    return value ^ (1ULL << bitPos);
}

int
FaultInjector::getRandomBitPosition(int maxBits)
{
    return random.random<int>() % maxBits;
}

FaultInjector::FaultType
FaultInjector::getRandomFaultType()
{
    int r = random.random<int>() % 100;
    
    // Weighted distribution:
    // 40% - bit flip in command
    // 25% - bit flip in address
    // 20% - bit flip in data
    // 10% - hang
    // 5%  - timeout
    if (r < 40) return FAULT_BITFLIP_CMD;
    else if (r < 65) return FAULT_BITFLIP_ADDR;
    else if (r < 85) return FAULT_BITFLIP_DATA;
    else if (r < 95) return FAULT_HANG;
    else return FAULT_TIMEOUT;
}

FaultInjector::FaultInjectorStats::FaultInjectorStats(FaultInjector *parent)
    : statistics::Group(parent),
      ADD_STAT(totalFaultsInjected, statistics::units::Count::get(),
               "Total faults injected"),
      ADD_STAT(bitflipFaults, statistics::units::Count::get(),
               "Bit-flip faults"),
      ADD_STAT(hangFaults, statistics::units::Count::get(),
               "Hang faults"),
      ADD_STAT(timeoutFaults, statistics::units::Count::get(),
               "Timeout faults"),
      ADD_STAT(faultsDetected, statistics::units::Count::get(),
               "Faults detected"),
      ADD_STAT(faultsRecovered, statistics::units::Count::get(),
               "Faults recovered"),
      ADD_STAT(falsePositives, statistics::units::Count::get(),
               "False positive detections"),
      ADD_STAT(missedFaults, statistics::units::Count::get(),
               "Missed faults (undetected)"),
      ADD_STAT(detectionLatency, statistics::units::Cycle::get(),
               "Detection latency distribution"),
      ADD_STAT(perAccelFaults, statistics::units::Count::get(),
               "Per-accelerator faults")
{
    detectionLatency.init(20);
    perAccelFaults.init(16);  // Max 16 accelerators
    
    for (int i = 0; i < 16; i++) {
        perAccelFaults.subname(i, std::string("accel") + std::to_string(i));
    }
}

} // namespace gem5

