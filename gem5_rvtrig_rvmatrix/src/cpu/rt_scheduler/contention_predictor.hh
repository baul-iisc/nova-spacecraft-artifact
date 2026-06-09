/*
 * Copyright (c) 2024 NOVA Processor Research
 * Real-Time Scheduling for Spacecraft Accelerators
 *
 * ContentionPredictor: Uses calibrated analytical model to predict
 * accelerator contention overhead for WCET analysis.
 *
 * Model: Overhead% = β × ρ^α × (N-1)^δ / N × (1 + γ × log₂N)
 * Calibrated from gem5 simulations (R² = 0.9986)
 */

#ifndef __CPU_RT_SCHEDULER_CONTENTION_PREDICTOR_HH__
#define __CPU_RT_SCHEDULER_CONTENTION_PREDICTOR_HH__

#include <cmath>
#include <string>
#include <map>

namespace gem5
{

/**
 * ContentionPredictor predicts accelerator contention overhead
 * using the calibrated analytical model from gem5 simulations.
 */
class ContentionPredictor
{
  public:
    // Calibrated model parameters (from thesis analysis)
    static constexpr double BETA = 0.6019;   // Contention sensitivity
    static constexpr double GAMMA = 2.0000;  // Logarithmic scaling
    static constexpr double ALPHA = 3.9868;  // Utilization exponent
    static constexpr double DELTA = 1.8683;  // Core scaling exponent

    // Accelerator latencies in cycles
    static constexpr int TRIG_LATENCY = 15;
    static constexpr int MAT_LATENCY = 45;
    static constexpr int VPU_LATENCY = 100;
    static constexpr int NPU_LATENCY = 200;

    /**
     * Predict contention overhead percentage.
     *
     * @param numCores Number of active CPU cores
     * @param utilization Accelerator utilization (0.0 - 1.0)
     * @return Overhead percentage (0.0 - 100.0)
     */
    static double predictOverhead(int numCores, double utilization)
    {
        if (numCores <= 1 || utilization <= 0.0) {
            return 0.0;
        }

        // Clamp utilization to valid range
        double rho = std::min(std::max(utilization, 0.01), 0.99);

        // Extended analytical model
        double term1 = std::pow(rho, ALPHA);
        double term2 = std::pow(numCores - 1, DELTA) / numCores;
        double term3 = 1.0 + GAMMA * std::log2(numCores);

        return BETA * term1 * term2 * term3;
    }

    /**
     * Predict worst-case execution time including contention.
     *
     * @param baseWCET Base WCET in cycles (without contention)
     * @param numCores Number of active CPU cores
     * @param utilization Accelerator utilization (0.0 - 1.0)
     * @return Adjusted WCET in cycles
     */
    static uint64_t predictWCET(uint64_t baseWCET, int numCores, double utilization)
    {
        double overhead = predictOverhead(numCores, utilization);
        return static_cast<uint64_t>(baseWCET * (1.0 + overhead / 100.0));
    }

    /**
     * Predict contention wait time for a single accelerator request.
     *
     * @param numCores Number of active CPU cores
     * @param utilization Accelerator utilization
     * @param accelLatency Accelerator operation latency in cycles
     * @return Expected wait time in cycles
     */
    static uint64_t predictWaitCycles(int numCores, double utilization, int accelLatency)
    {
        if (numCores <= 1) {
            return 0;
        }

        double overhead = predictOverhead(numCores, utilization);
        // Scale wait time by accelerator latency and overhead
        return static_cast<uint64_t>(accelLatency * overhead / 100.0 * numCores);
    }

    /**
     * Estimate utilization from task parameters.
     *
     * @param accelOpsPerPeriod Number of accelerator operations per task period
     * @param avgLatency Average accelerator latency
     * @param period Task period in cycles
     * @param numCores Number of cores running the task
     * @return Estimated utilization (0.0 - 1.0)
     */
    static double estimateUtilization(int accelOpsPerPeriod, int avgLatency,
                                       uint64_t period, int numCores)
    {
        if (period == 0) return 0.0;

        double totalAccelTime = accelOpsPerPeriod * avgLatency * numCores;
        return std::min(totalAccelTime / period, 0.99);
    }

    /**
     * Get workload-specific utilization based on profiled data.
     *
     * @param workloadName Name of the workload
     * @return Calibrated utilization value
     */
    static double getWorkloadUtilization(const std::string& workloadName)
    {
        // Calibrated from gem5 simulations
        static const std::map<std::string, double> utilizations = {
            {"gnc", 0.40},
            {"adcs", 0.35},
            {"attitude_control", 0.35},
            {"navigation", 0.45},
            {"thermal", 0.10},
            {"imaging", 0.60},
            {"comms", 0.15},
            {"power_management", 0.10},
            {"lunar_landing", 0.14},
            {"orbit_operations", 0.12},
            {"vision_nav", 0.10},
            {"stress_test", 0.89},
            {"default", 0.30}
        };

        auto it = utilizations.find(workloadName);
        if (it != utilizations.end()) {
            return it->second;
        }
        return utilizations.at("default");
    }
};

} // namespace gem5

#endif // __CPU_RT_SCHEDULER_CONTENTION_PREDICTOR_HH__


