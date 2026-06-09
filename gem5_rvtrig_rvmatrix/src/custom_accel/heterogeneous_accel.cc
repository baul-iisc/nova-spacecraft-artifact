/*
 * Heterogeneous Accelerator Network Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/heterogeneous_accel.hh"
#include "base/trace.hh"
#include "debug/HeterogeneousAccel.hh"
#include "params/ClockedObject.hh"

#include <algorithm>
#include <limits>

namespace gem5
{

HeterogeneousAccelNetwork::HeterogeneousAccelNetwork(const Params &p)
    : ClockedObject(dynamic_cast<const ClockedObjectParams &>(p)),
      numHighPrecisionMatrix(p.num_hp_matrix),
      numLowPrecisionMatrix(p.num_lp_matrix),
      numHighPrecisionTrig(p.num_hp_trig),
      numLowPrecisionTrig(p.num_lp_trig),
      totalPowerBudget(p.total_power_budget),
      enableDynamicRouting(p.enable_dynamic_routing),
      nextRequestId(0),
      nextAccelId(0),
      stats(this)
{
    /* Initialize workload precision policies */
    workloadPrecisionPolicy[WORKLOAD_NAVIGATION] = TIER_HIGH;
    workloadPrecisionPolicy[WORKLOAD_ATTITUDE] = TIER_HIGH;
    workloadPrecisionPolicy[WORKLOAD_TELEMETRY] = TIER_MEDIUM;
    workloadPrecisionPolicy[WORKLOAD_SCIENCE_IMAGE] = TIER_LOW;
    workloadPrecisionPolicy[WORKLOAD_SCIENCE_DATA] = TIER_LOW;
    workloadPrecisionPolicy[WORKLOAD_ML_INFERENCE] = TIER_ULTRA_LOW;

    /* Create default accelerator configuration */
    /* High-precision matrix accelerators */
    for (int i = 0; i < numHighPrecisionMatrix; i++) {
        addAccelerator(ACCEL_MATRIX_HP, TIER_HIGH, 9, 2.5, 0.15);
    }

    /* Low-precision matrix accelerators */
    for (int i = 0; i < numLowPrecisionMatrix; i++) {
        addAccelerator(ACCEL_MATRIX_LP, TIER_LOW, 36, 1.0, 0.08);
    }

    /* High-precision trig (CORDIC) */
    for (int i = 0; i < numHighPrecisionTrig; i++) {
        addAccelerator(ACCEL_TRIG_HP, TIER_HIGH, 1, 0.5, 0.03);
    }

    /* Low-precision trig (LUT-based) */
    for (int i = 0; i < numLowPrecisionTrig; i++) {
        addAccelerator(ACCEL_TRIG_LP, TIER_LOW, 4, 0.2, 0.01);
    }

    DPRINTF(HeterogeneousAccel, "HeterogeneousAccelNetwork created: "
            "%d HP matrix, %d LP matrix, %d HP trig, %d LP trig\n",
            numHighPrecisionMatrix, numLowPrecisionMatrix,
            numHighPrecisionTrig, numLowPrecisionTrig);
}

HeterogeneousAccelNetwork::~HeterogeneousAccelNetwork()
{
}

void
HeterogeneousAccelNetwork::addAccelerator(AccelType type, PrecisionTier precision,
                                          int throughput, double power, double area)
{
    AcceleratorDesc desc;
    desc.id = nextAccelId++;
    desc.type = type;
    desc.precision = precision;
    desc.throughput = throughput;
    desc.powerConsumption = power;
    desc.areaFraction = area;
    desc.available = true;
    desc.currentPrecision = tierToBits(precision);

    accelerators.push_back(desc);

    DPRINTF(HeterogeneousAccel, "Added accelerator %d: type=%d prec=%d "
            "throughput=%d power=%.2fW\n", desc.id, type, precision,
            throughput, power);
}

int
HeterogeneousAccelNetwork::routeWorkload(const ComputeRequest &req)
{
    stats.routingDecisions++;

    /* Get policy-recommended precision */
    PrecisionTier policyTier = workloadPrecisionPolicy[req.category];
    PrecisionTier minTier = bitsToTier(req.requiredPrecision);

    /* Use higher of policy and requirement */
    PrecisionTier targetTier = std::max(policyTier, minTier);

    /* Find best accelerator */
    int selectedAccel = selectAccelerator(req);

    if (selectedAccel < 0) {
        DPRINTF(HeterogeneousAccel, "No accelerator available for request %d\n",
                req.requestId);
        return -1;
    }

    /* Check if we had to downgrade */
    if (accelerators[selectedAccel].precision < targetTier) {
        stats.precisionDowngrades++;
        DPRINTF(HeterogeneousAccel, "Precision downgrade: requested %d, got %d\n",
                targetTier, accelerators[selectedAccel].precision);
    }

    return selectedAccel;
}

int
HeterogeneousAccelNetwork::submitRequest(WorkloadCategory category,
    int precisionBits, double urgency, double powerBudget,
    const std::vector<double> &inputs,
    std::function<void(int, std::vector<double>&)> callback)
{
    ComputeRequest req;
    req.requestId = nextRequestId++;
    req.category = category;
    req.requiredPrecision = precisionBits;
    req.urgency = urgency;
    req.powerBudget = powerBudget;
    req.inputs = inputs;
    req.callback = callback;

    stats.totalRequests++;
    stats.requestsPerCategory[category]++;

    /* Route to best accelerator */
    int accelId = routeWorkload(req);
    if (accelId < 0) {
        return -1;  // Failed to route
    }

    /* Store request and execute */
    pendingRequests[req.requestId] = req;
    requestToAccel[req.requestId] = accelId;

    stats.requestsPerTier[accelerators[accelId].precision]++;
    stats.accelUtilization[accelId]++;

    executeOnAccelerator(accelId, req);

    return req.requestId;
}

std::vector<HeterogeneousAccelNetwork::AcceleratorDesc>
HeterogeneousAccelNetwork::getAvailableAccelerators(WorkloadCategory cat)
{
    std::vector<AcceleratorDesc> available;
    PrecisionTier minTier = workloadPrecisionPolicy[cat];

    for (const auto &accel : accelerators) {
        if (accel.available && accel.precision >= minTier) {
            available.push_back(accel);
        }
    }

    return available;
}

bool
HeterogeneousAccelNetwork::configurePrecision(int accelId, PrecisionTier tier)
{
    for (auto &accel : accelerators) {
        if (accel.id == accelId) {
            /* Check if downgrade is allowed */
            if (tier < accel.precision) {
                stats.precisionDowngrades++;
            } else if (tier > accel.precision) {
                stats.precisionUpgrades++;
            }

            accel.currentPrecision = tierToBits(tier);
            DPRINTF(HeterogeneousAccel, "Configured accel %d precision: %d bits\n",
                    accelId, accel.currentPrecision);
            return true;
        }
    }
    return false;
}

double
HeterogeneousAccelNetwork::estimatePowerConsumption(WorkloadCategory cat,
                                                    PrecisionTier tier)
{
    /* Power model based on precision and workload type */
    double basePower = 0.5;  // Watts

    /* Precision scaling (higher precision = more power) */
    double precisionFactor = 1.0 + (tier * 0.5);

    /* Workload type scaling */
    double workloadFactor = 1.0;
    switch (cat) {
        case WORKLOAD_NAVIGATION:
        case WORKLOAD_ATTITUDE:
            workloadFactor = 1.5;  // Continuous operation
            break;
        case WORKLOAD_SCIENCE_IMAGE:
            workloadFactor = 2.0;  // High throughput
            break;
        default:
            workloadFactor = 1.0;
            break;
    }

    return basePower * precisionFactor * workloadFactor;
}

Cycles
HeterogeneousAccelNetwork::estimateLatency(WorkloadCategory cat, PrecisionTier tier)
{
    /* Latency model */
    int baseLatency = 10;

    /* Precision affects latency (lower precision = faster) */
    int precisionLatency = (NUM_TIERS - tier) * 5;

    /* Workload complexity */
    int workloadLatency = 0;
    switch (cat) {
        case WORKLOAD_NAVIGATION:
            workloadLatency = 20;  // Kalman filter, etc.
            break;
        case WORKLOAD_SCIENCE_IMAGE:
            workloadLatency = 50;  // Image processing
            break;
        default:
            workloadLatency = 10;
            break;
    }

    return Cycles(baseLatency + precisionLatency + workloadLatency);
}

int
HeterogeneousAccelNetwork::selectAccelerator(const ComputeRequest &req)
{
    /* Determine preferred accelerator type based on workload */
    AccelType preferredType;
    switch (req.category) {
        case WORKLOAD_NAVIGATION:
        case WORKLOAD_ATTITUDE:
            preferredType = ACCEL_MATRIX_HP;  // Need high precision
            break;
        case WORKLOAD_ML_INFERENCE:
            preferredType = ACCEL_MATRIX_LP;  // Low precision OK
            break;
        default:
            preferredType = ACCEL_MATRIX_LP;
            break;
    }

    /* Find best matching accelerator */
    return findBestMatch(preferredType, bitsToTier(req.requiredPrecision),
                        req.powerBudget);
}

int
HeterogeneousAccelNetwork::findBestMatch(AccelType preferredType,
                                         PrecisionTier minPrecision,
                                         double powerLimit)
{
    int bestAccel = -1;
    double bestScore = -1.0;

    for (size_t i = 0; i < accelerators.size(); i++) {
        const auto &accel = accelerators[i];

        if (!accel.available) continue;
        if (accel.precision < minPrecision) continue;
        if (accel.powerConsumption > powerLimit && powerLimit > 0) continue;

        /* Score based on: type match, precision match, power efficiency */
        double score = 0.0;

        /* Type match bonus */
        if (accel.type == preferredType) {
            score += 10.0;
        } else if ((preferredType == ACCEL_MATRIX_HP && accel.type == ACCEL_MATRIX_LP) ||
                   (preferredType == ACCEL_MATRIX_LP && accel.type == ACCEL_MATRIX_HP)) {
            score += 5.0;  // Same family
        }

        /* Precision match (prefer exact match over overkill) */
        int precisionDiff = accel.precision - minPrecision;
        score += 5.0 - precisionDiff;

        /* Power efficiency */
        score += (10.0 - accel.powerConsumption) * 0.5;

        /* Throughput bonus */
        score += accel.throughput * 0.1;

        if (score > bestScore) {
            bestScore = score;
            bestAccel = i;
        }
    }

    return bestAccel;
}

bool
HeterogeneousAccelNetwork::meetsRequirements(const AcceleratorDesc &accel,
                                             const ComputeRequest &req)
{
    if (!accel.available) return false;
    if (tierToBits(accel.precision) < req.requiredPrecision) return false;
    if (accel.powerConsumption > req.powerBudget && req.powerBudget > 0) return false;
    return true;
}

double
HeterogeneousAccelNetwork::calculateScore(const AcceleratorDesc &accel,
                                          const ComputeRequest &req)
{
    /* Scoring function for accelerator selection */
    double score = 0.0;

    /* Precision match */
    int precBits = tierToBits(accel.precision);
    if (precBits >= req.requiredPrecision) {
        score += 10.0 - (precBits - req.requiredPrecision) * 0.5;
    }

    /* Power efficiency (lower is better within budget) */
    if (req.powerBudget > 0) {
        score += (req.powerBudget - accel.powerConsumption) / req.powerBudget * 5.0;
    }

    /* Urgency matching (use high-throughput for urgent) */
    if (req.urgency > 0.7) {
        score += accel.throughput * 0.2;
    }

    return score;
}

int
HeterogeneousAccelNetwork::tierToBits(PrecisionTier tier)
{
    switch (tier) {
        case TIER_HIGH: return 64;
        case TIER_MEDIUM: return 32;
        case TIER_LOW: return 16;
        case TIER_ULTRA_LOW: return 8;
        default: return 32;
    }
}

HeterogeneousAccelNetwork::PrecisionTier
HeterogeneousAccelNetwork::bitsToTier(int bits)
{
    if (bits >= 64) return TIER_HIGH;
    if (bits >= 32) return TIER_MEDIUM;
    if (bits >= 16) return TIER_LOW;
    return TIER_ULTRA_LOW;
}

double
HeterogeneousAccelNetwork::getCurrentPowerUsage()
{
    double total = 0.0;
    for (const auto &accel : accelerators) {
        if (!accel.available) {  // Busy accelerators consume power
            total += accel.powerConsumption;
        }
    }
    return total;
}

bool
HeterogeneousAccelNetwork::canAccommodatePower(double additionalPower)
{
    return (getCurrentPowerUsage() + additionalPower) <= totalPowerBudget;
}

void
HeterogeneousAccelNetwork::executeOnAccelerator(int accelId, ComputeRequest &req)
{
    accelerators[accelId].available = false;

    DPRINTF(HeterogeneousAccel, "Executing request %d on accel %d\n",
            req.requestId, accelId);

    /* Simulate execution (in real implementation, would schedule completion) */
    std::vector<double> results = req.inputs;  // Placeholder
    completeRequest(req.requestId, accelId, results);
}

void
HeterogeneousAccelNetwork::completeRequest(int requestId, int accelId,
                                           std::vector<double> &results)
{
    auto it = pendingRequests.find(requestId);
    if (it == pendingRequests.end()) return;

    accelerators[accelId].available = true;

    if (it->second.callback) {
        it->second.callback(requestId, results);
    }

    pendingRequests.erase(it);
    requestToAccel.erase(requestId);

    DPRINTF(HeterogeneousAccel, "Completed request %d on accel %d\n",
            requestId, accelId);
}

HeterogeneousAccelNetwork::HeteroAccelStats::HeteroAccelStats(
    HeterogeneousAccelNetwork *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(),
               "Total requests"),
      ADD_STAT(requestsPerCategory, statistics::units::Count::get(),
               "Requests per workload category"),
      ADD_STAT(requestsPerTier, statistics::units::Count::get(),
               "Requests per precision tier"),
      ADD_STAT(accelUtilization, statistics::units::Count::get(),
               "Accelerator utilization"),
      ADD_STAT(routingDecisions, statistics::units::Count::get(),
               "Routing decisions made"),
      ADD_STAT(precisionDowngrades, statistics::units::Count::get(),
               "Precision downgrades"),
      ADD_STAT(precisionUpgrades, statistics::units::Count::get(),
               "Precision upgrades"),
      ADD_STAT(powerBudgetExceeded, statistics::units::Count::get(),
               "Power budget exceeded events"),
      ADD_STAT(avgLatency, statistics::units::Cycle::get(),
               "Average request latency"),
      ADD_STAT(avgPowerUsage, statistics::units::Count::get(),
               "Average power usage"),
      ADD_STAT(precisionDistribution, statistics::units::Count::get(),
               "Precision distribution")
{
    requestsPerCategory.init(NUM_CATEGORIES);
    requestsPerTier.init(NUM_TIERS);
    accelUtilization.init(16);  // Max 16 accelerators
    precisionDistribution.init(4);  // 4 tiers

    const char* catNames[] = {"navigation", "attitude", "telemetry",
                              "science_image", "science_data", "ml_inference"};
    for (int i = 0; i < NUM_CATEGORIES; i++) {
        requestsPerCategory.subname(i, catNames[i]);
    }

    const char* tierNames[] = {"high", "medium", "low", "ultra_low"};
    for (int i = 0; i < NUM_TIERS; i++) {
        requestsPerTier.subname(i, tierNames[i]);
    }
}

} // namespace gem5

