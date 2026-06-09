/*
 * Fault Injection Framework for Spacecraft Accelerators
 * PhD Research: Chandraboul
 * 
 * Implements bit-flip fault injection in accelerator command queues
 * to simulate single-event upsets (SEUs) in space radiation environment.
 */

#ifndef __CUSTOM_ACCEL_FAULT_INJECTOR_HH__
#define __CUSTOM_ACCEL_FAULT_INJECTOR_HH__

#include "base/random.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/FaultInjector.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <queue>
#include <vector>
#include <functional>

namespace gem5
{

/**
 * FaultInjector - Simulates radiation-induced bit flips in accelerator queues
 * 
 * Features:
 * - Random bit-flip injection at configurable rates
 * - Targeted injection at specific bit positions
 * - Watchdog detection of "hung" accelerators
 * - Error recovery simulation
 */
class FaultInjector : public ClockedObject
{
  public:
    PARAMS(FaultInjector);
    FaultInjector(const Params &p);
    ~FaultInjector();

    /* Fault types */
    enum FaultType {
        FAULT_BITFLIP_CMD,      // Bit flip in command register
        FAULT_BITFLIP_ADDR,     // Bit flip in address register
        FAULT_BITFLIP_DATA,     // Bit flip in data register
        FAULT_QUEUE_CORRUPT,    // Corrupt queue pointer
        FAULT_HANG,             // Accelerator hang (infinite loop)
        FAULT_TIMEOUT,          // Response timeout
        FAULT_NONE
    };

    /* Fault injection result */
    struct FaultEvent {
        Tick injectionTime;
        FaultType type;
        int targetAccel;        // Which accelerator
        int targetCore;         // Which core's request
        uint64_t originalValue;
        uint64_t corruptedValue;
        int bitPosition;
        bool detected;
        bool recovered;
        Tick detectionLatency;
    };

    /* Register an accelerator for fault injection */
    void registerAccelerator(const std::string &accelName, int id,
                             std::function<void(FaultType, int, uint64_t*)> callback);

    /* Inject a fault immediately */
    void injectFault(int accelId, FaultType type, int bitPosition = -1);

    /* Schedule random fault injection */
    void scheduleRandomFault();

    /* Check for hung accelerator */
    void startWatchdog(int accelId, Tick timeout);
    void petWatchdog(int accelId);
    void cancelWatchdog(int accelId);

    /* Report detection */
    void reportFaultDetected(int accelId, FaultType type, Tick latency);
    void reportFaultRecovered(int accelId, FaultType type);

    /* Get fault history */
    const std::vector<FaultEvent>& getFaultHistory() const { return faultHistory; }

  private:
    /* Parameters */
    double faultRate;           // Faults per million cycles
    bool enableRandomFaults;
    int maxFaults;
    Cycles watchdogTimeout;
    bool enableRecovery;

    /* State */
    struct AccelInfo {
        std::string name;
        int id;
        std::function<void(FaultType, int, uint64_t*)> callback;
        bool watchdogActive;
        Tick watchdogStart;
        Tick lastActivity;
    };
    std::vector<AccelInfo> accelerators;
    std::vector<FaultEvent> faultHistory;
    int injectedFaults;
    Random random;

    /* Events */
    void processRandomFault();
    void checkWatchdog();
    EventFunctionWrapper randomFaultEvent;
    EventFunctionWrapper watchdogCheckEvent;

    /* Helper functions */
    uint64_t flipBit(uint64_t value, int bitPos);
    int getRandomBitPosition(int maxBits);
    FaultType getRandomFaultType();

    /* Statistics */
    struct FaultInjectorStats : public statistics::Group
    {
        FaultInjectorStats(FaultInjector *parent);

        statistics::Scalar totalFaultsInjected;
        statistics::Scalar bitflipFaults;
        statistics::Scalar hangFaults;
        statistics::Scalar timeoutFaults;
        statistics::Scalar faultsDetected;
        statistics::Scalar faultsRecovered;
        statistics::Scalar falsePositives;
        statistics::Scalar missedFaults;
        statistics::Histogram detectionLatency;
        statistics::Vector perAccelFaults;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_FAULT_INJECTOR_HH__

