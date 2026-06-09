/*
 * Spacecraft Multi-Accelerator SoC Model
 * 
 * Defines all hardware accelerators for a spacecraft processor:
 * - CORDIC (Trigonometric)
 * - Matrix (3x3 Systolic Array)
 * - SpaceWire MAC (Communication)
 * - Reed-Solomon (Error Correction)
 * - AES-256 (Encryption)
 * - Image Compressor (Science Data)
 * - Kalman Filter (State Estimation)
 * 
 * Part of PhD Research: Power-Aware Scheduling for Heterogeneous Accelerators
 */

#ifndef __CPU_POWER_MANAGER_SPACECRAFT_ACCELERATORS_HH__
#define __CPU_POWER_MANAGER_SPACECRAFT_ACCELERATORS_HH__

#include <cstdint>
#include <string>
#include <array>

namespace gem5 {

//==============================================================================
// Accelerator Types
//==============================================================================

enum class AcceleratorType : uint8_t {
    CORDIC          = 0,   // Trigonometric operations (sin, cos, atan)
    MATRIX          = 1,   // 3x3 matrix multiplication (systolic)
    SPACEWIRE_MAC   = 2,   // SpaceWire communication protocol
    REED_SOLOMON    = 3,   // Error correction coding
    AES_256         = 4,   // Encryption/decryption
    IMAGE_COMPRESS  = 5,   // CCSDS image compression
    KALMAN_FILTER   = 6,   // State estimation
    NPU             = 7,   // Neural Processing Unit (TinyML, systolic array)
    VPU             = 8,   // Vision Processing Unit (image/conv/features)
    NUM_ACCELERATORS = 9
};

//==============================================================================
// Power States (DVFS Levels)
//==============================================================================

enum class PowerState : uint8_t {
    OFF         = 0,   // Completely powered off
    SLEEP       = 1,   // Ultra-low power, state retained
    IDLE        = 2,   // Clock gated, quick wake
    LOW_POWER   = 3,   // 50% frequency, 25% power
    NOMINAL     = 4,   // 100% frequency, 100% power
    TURBO       = 5,   // 125% frequency, 150% power (limited duration)
    NUM_STATES  = 6
};

//==============================================================================
// Accelerator Characteristics
//==============================================================================

struct AcceleratorSpec {
    const char* name;
    uint32_t baseLatencyCycles;     // Latency at NOMINAL power state
    double powerMW[6];              // Power (mW) for each PowerState
    uint32_t wakeupLatency[6];      // Cycles to transition TO this state
    double utilizationWeight;        // Priority weight for scheduling
    bool shareableAcrossCores;       // Can be shared?
    uint8_t criticalityLevel;        // 0=best-effort, 1=mission, 2=safety
};

// Accelerator specifications for spacecraft processor
constexpr AcceleratorSpec ACCEL_SPECS[] = {
    // CORDIC - Trigonometric (frequently used by ADCS)
    {
        .name = "CORDIC",
        .baseLatencyCycles = 32,
        .powerMW = {0.0, 0.5, 2.0, 15.0, 45.0, 70.0},  // OFF to TURBO
        .wakeupLatency = {100, 50, 10, 2, 0, 0},
        .utilizationWeight = 0.3,
        .shareableAcrossCores = true,
        .criticalityLevel = 2  // Safety-critical (ADCS)
    },
    // Matrix - 3x3 Systolic Array (GNC, transforms)
    {
        .name = "Matrix3x3",
        .baseLatencyCycles = 27,
        .powerMW = {0.0, 0.8, 3.0, 20.0, 60.0, 95.0},
        .wakeupLatency = {120, 60, 15, 3, 0, 0},
        .utilizationWeight = 0.25,
        .shareableAcrossCores = true,
        .criticalityLevel = 2  // Safety-critical (GNC)
    },
    // SpaceWire MAC - Communication Protocol Engine
    {
        .name = "SpaceWire",
        .baseLatencyCycles = 50,
        .powerMW = {0.0, 1.0, 5.0, 25.0, 75.0, 120.0},
        .wakeupLatency = {200, 100, 25, 5, 0, 0},
        .utilizationWeight = 0.15,
        .shareableAcrossCores = true,
        .criticalityLevel = 1  // Mission-critical
    },
    // Reed-Solomon - Error Correction (CCSDS standard)
    {
        .name = "ReedSolomon",
        .baseLatencyCycles = 100,
        .powerMW = {0.0, 1.5, 8.0, 40.0, 120.0, 190.0},
        .wakeupLatency = {150, 80, 20, 4, 0, 0},
        .utilizationWeight = 0.1,
        .shareableAcrossCores = true,
        .criticalityLevel = 1  // Mission-critical
    },
    // AES-256 - Encryption Engine
    {
        .name = "AES256",
        .baseLatencyCycles = 80,
        .powerMW = {0.0, 1.2, 6.0, 30.0, 90.0, 140.0},
        .wakeupLatency = {100, 50, 12, 3, 0, 0},
        .utilizationWeight = 0.08,
        .shareableAcrossCores = true,
        .criticalityLevel = 2  // Safety-critical (secure comms)
    },
    // Image Compressor - CCSDS 122.0 Wavelet
    {
        .name = "ImageComp",
        .baseLatencyCycles = 200,
        .powerMW = {0.0, 2.0, 10.0, 50.0, 150.0, 240.0},
        .wakeupLatency = {300, 150, 40, 8, 0, 0},
        .utilizationWeight = 0.07,
        .shareableAcrossCores = false,  // Dedicated to imaging core
        .criticalityLevel = 0  // Best-effort (science)
    },
    // Kalman Filter - State Estimation Accelerator
    {
        .name = "KalmanFilter",
        .baseLatencyCycles = 150,
        .powerMW = {0.0, 1.8, 9.0, 45.0, 135.0, 215.0},
        .wakeupLatency = {180, 90, 25, 5, 0, 0},
        .utilizationWeight = 0.05,
        .shareableAcrossCores = true,
        .criticalityLevel = 2  // Safety-critical (navigation)
    },
    // NPU - Neural Processing Unit (TinyML, 8x8 systolic, INT8, weight/act buffers)
    {
        .name = "NPU",
        .baseLatencyCycles = 64,
        .powerMW = {0.0, 1.0, 50.0, 125.0, 500.0, 650.0},  // Match NeuralProcessingUnit.py
        .wakeupLatency = {200, 100, 20, 5, 0, 0},
        .utilizationWeight = 0.2,
        .shareableAcrossCores = true,
        .criticalityLevel = 1  // Mission (AI/autonomy)
    },
    // VPU - Vision Processing Unit (debayering, Sobel/Canny, conv, 16 MACs)
    {
        .name = "VPU",
        .baseLatencyCycles = 100,
        .powerMW = {0.0, 0.5, 10.0, 25.0, 100.0, 130.0},  // Match VisionProcessingUnit.py
        .wakeupLatency = {100, 50, 10, 3, 0, 0},
        .utilizationWeight = 0.15,
        .shareableAcrossCores = true,
        .criticalityLevel = 1  // Mission (vision/navigation)
    }
};

//==============================================================================
// Accelerator State (Runtime)
//==============================================================================

struct AcceleratorState {
    AcceleratorType type;
    PowerState currentPowerState;
    bool busy;
    uint64_t busyUntilTick;
    int ownerCpuId;
    
    // Statistics
    uint64_t totalRequests;
    uint64_t queuedRequests;
    uint64_t totalWaitCycles;
    uint64_t totalActiveCycles;
    double totalEnergyMicroJ;
    
    // Power state transitions
    uint64_t stateTransitions[6];  // Count per target state
    uint64_t timeInState[6];       // Ticks spent in each state
    uint64_t lastStateChangeTime;
    
    AcceleratorState() 
        : type(AcceleratorType::CORDIC)
        , currentPowerState(PowerState::IDLE)
        , busy(false)
        , busyUntilTick(0)
        , ownerCpuId(-1)
        , totalRequests(0)
        , queuedRequests(0)
        , totalWaitCycles(0)
        , totalActiveCycles(0)
        , totalEnergyMicroJ(0.0)
        , lastStateChangeTime(0)
    {
        for (int i = 0; i < 6; i++) {
            stateTransitions[i] = 0;
            timeInState[i] = 0;
        }
    }
};

//==============================================================================
// Frequency/Voltage Scaling Factors
//==============================================================================

struct DVFSLevel {
    const char* name;
    double frequencyRatio;  // Relative to nominal (1.0)
    double voltageRatio;    // Relative to nominal (1.0)
    double powerRatio;      // Power = V^2 * f, normalized
};

constexpr DVFSLevel DVFS_LEVELS[] = {
    {"OFF",       0.00, 0.00, 0.00},
    {"SLEEP",     0.00, 0.50, 0.01},   // Retention voltage only
    {"IDLE",      0.00, 0.80, 0.05},   // Clock gated
    {"LOW_POWER", 0.50, 0.80, 0.25},   // 50% freq, 80% voltage
    {"NOMINAL",   1.00, 1.00, 1.00},   // Reference point
    {"TURBO",     1.25, 1.10, 1.50},   // 125% freq, 110% voltage
};

//==============================================================================
// Helper Functions
//==============================================================================

inline const AcceleratorSpec& getAccelSpec(AcceleratorType type) {
    return ACCEL_SPECS[static_cast<int>(type)];
}

inline const char* getAccelName(AcceleratorType type) {
    return ACCEL_SPECS[static_cast<int>(type)].name;
}

inline const char* getPowerStateName(PowerState state) {
    return DVFS_LEVELS[static_cast<int>(state)].name;
}

inline uint32_t getScaledLatency(AcceleratorType type, PowerState state) {
    const auto& spec = getAccelSpec(type);
    const auto& dvfs = DVFS_LEVELS[static_cast<int>(state)];
    
    if (dvfs.frequencyRatio <= 0.0) {
        return UINT32_MAX;  // Accelerator not running
    }
    
    // Latency inversely proportional to frequency
    return static_cast<uint32_t>(spec.baseLatencyCycles / dvfs.frequencyRatio);
}

inline double getInstantPower(AcceleratorType type, PowerState state) {
    return ACCEL_SPECS[static_cast<int>(type)].powerMW[static_cast<int>(state)];
}

inline uint32_t getWakeupLatency(AcceleratorType type, PowerState from, PowerState to) {
    // Only significant when going to higher power state
    if (to > from) {
        return ACCEL_SPECS[static_cast<int>(type)].wakeupLatency[static_cast<int>(from)];
    }
    return 0;  // Powering down is immediate
}

} // namespace gem5

#endif // __CPU_POWER_MANAGER_SPACECRAFT_ACCELERATORS_HH__
