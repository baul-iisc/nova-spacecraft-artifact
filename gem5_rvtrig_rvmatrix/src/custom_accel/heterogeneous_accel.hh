/*
 * Heterogeneous Accelerator Network
 * PhD Research: Chandraboul
 * 
 * Research Question 4: Heterogeneous Accelerator Networks
 * - Mix of high-precision (navigation) and low-precision (science) matrix units
 * - Configurable trig accelerators with precision/power tradeoff
 * - Workload routing to appropriate accelerator tiers
 */

#ifndef __CUSTOM_ACCEL_HETEROGENEOUS_ACCEL_HH__
#define __CUSTOM_ACCEL_HETEROGENEOUS_ACCEL_HH__

#include "base/statistics.hh"
#include "base/trace.hh"
#include "params/HeterogeneousAccelNetwork.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <vector>
#include <map>
#include <queue>
#include <functional>

namespace gem5
{

/**
 * HeterogeneousAccelNetwork - Multi-tier accelerator management
 * 
 * Features:
 * 1. Multiple precision tiers (high/medium/low)
 * 2. Power-aware accelerator selection
 * 3. Automatic workload routing based on requirements
 * 4. Runtime precision configuration
 */
class HeterogeneousAccelNetwork : public ClockedObject
{
  public:
    PARAMS(HeterogeneousAccelNetwork);
    HeterogeneousAccelNetwork(const Params &p);
    ~HeterogeneousAccelNetwork();

    /* Precision tiers */
    enum PrecisionTier {
        TIER_HIGH,      // 64-bit double precision (navigation)
        TIER_MEDIUM,    // 32-bit single precision (general)
        TIER_LOW,       // 16-bit half precision (bulk data)
        TIER_ULTRA_LOW, // 8-bit fixed point (ML inference)
        NUM_TIERS
    };

    /* Accelerator types in the network */
    enum AccelType {
        ACCEL_MATRIX_HP,     // High precision matrix
        ACCEL_MATRIX_LP,     // Low precision matrix
        ACCEL_TRIG_HP,       // High precision trig (CORDIC)
        ACCEL_TRIG_LP,       // Low precision trig (LUT-based)
        ACCEL_FFT,           // FFT accelerator
        ACCEL_COMPRESS,      // Compression
        NUM_ACCEL_TYPES
    };

    /* Workload categories */
    enum WorkloadCategory {
        WORKLOAD_NAVIGATION,     // High precision required
        WORKLOAD_ATTITUDE,       // High precision
        WORKLOAD_TELEMETRY,      // Medium precision
        WORKLOAD_SCIENCE_IMAGE,  // Low precision OK
        WORKLOAD_SCIENCE_DATA,   // Low precision OK
        WORKLOAD_ML_INFERENCE,   // Ultra low precision
        NUM_CATEGORIES
    };

    /* Accelerator descriptor */
    struct AcceleratorDesc {
        int id;
        AccelType type;
        PrecisionTier precision;
        int throughput;          // Ops per cycle
        double powerConsumption; // Watts
        double areaFraction;     // Fraction of total die area
        bool available;
        int currentPrecision;    // Runtime configurable
    };

    /* Compute request */
    struct ComputeRequest {
        int requestId;
        WorkloadCategory category;
        int requiredPrecision;   // Minimum bits needed
        double urgency;          // 0.0 (low) to 1.0 (critical)
        double powerBudget;      // Max power allowed
        std::vector<double> inputs;
        std::function<void(int, std::vector<double>&)> callback;
    };

    /* Route workload to best accelerator */
    int routeWorkload(const ComputeRequest &req);

    /* Submit compute request */
    int submitRequest(WorkloadCategory category, int precisionBits,
                     double urgency, double powerBudget,
                     const std::vector<double> &inputs,
                     std::function<void(int, std::vector<double>&)> callback);

    /* Get available accelerators for a workload type */
    std::vector<AcceleratorDesc> getAvailableAccelerators(WorkloadCategory cat);

    /* Configure accelerator precision at runtime */
    bool configurePrecision(int accelId, PrecisionTier tier);

    /* Get power consumption for a workload */
    double estimatePowerConsumption(WorkloadCategory cat, PrecisionTier tier);

    /* Get expected latency */
    Cycles estimateLatency(WorkloadCategory cat, PrecisionTier tier);

    /* Add accelerator to network */
    void addAccelerator(AccelType type, PrecisionTier precision,
                       int throughput, double power, double area);

  private:
    /* Parameters */
    int numHighPrecisionMatrix;
    int numLowPrecisionMatrix;
    int numHighPrecisionTrig;
    int numLowPrecisionTrig;
    double totalPowerBudget;
    bool enableDynamicRouting;

    /* State */
    std::vector<AcceleratorDesc> accelerators;
    int nextRequestId;
    int nextAccelId;
    std::map<int, ComputeRequest> pendingRequests;
    std::map<int, int> requestToAccel;  // Request -> Accelerator mapping

    /* Workload to precision mapping (policy) */
    std::map<WorkloadCategory, PrecisionTier> workloadPrecisionPolicy;

    /* Routing logic */
    int selectAccelerator(const ComputeRequest &req);
    int findBestMatch(AccelType preferredType, PrecisionTier minPrecision,
                     double powerLimit);
    bool meetsRequirements(const AcceleratorDesc &accel, 
                          const ComputeRequest &req);
    double calculateScore(const AcceleratorDesc &accel, 
                         const ComputeRequest &req);

    /* Precision to bits conversion */
    int tierToBits(PrecisionTier tier);
    PrecisionTier bitsToTier(int bits);

    /* Power management */
    double getCurrentPowerUsage();
    bool canAccommodatePower(double additionalPower);

    /* Execution */
    void executeOnAccelerator(int accelId, ComputeRequest &req);
    void completeRequest(int requestId, int accelId, std::vector<double> &results);

    /* Statistics */
    struct HeteroAccelStats : public statistics::Group
    {
        HeteroAccelStats(HeterogeneousAccelNetwork *parent);

        statistics::Scalar totalRequests;
        statistics::Vector requestsPerCategory;
        statistics::Vector requestsPerTier;
        statistics::Vector accelUtilization;
        statistics::Scalar routingDecisions;
        statistics::Scalar precisionDowngrades;
        statistics::Scalar precisionUpgrades;
        statistics::Scalar powerBudgetExceeded;
        statistics::Scalar avgLatency;
        statistics::Scalar avgPowerUsage;
        statistics::Histogram precisionDistribution;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_HETEROGENEOUS_ACCEL_HH__

