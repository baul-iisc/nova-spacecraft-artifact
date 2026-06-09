/*
 * Space Workload Traffic Generator Implementation
 * PhD Research: Chandraboul
 */

#include "custom_accel/space_traffic_gen.hh"
#include "base/trace.hh"
#include "debug/SpaceTrafficGen.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

SpaceTrafficGen::SpaceTrafficGen(const Params &p)
    : ClockedObject(p),
      primaryWorkload(static_cast<WorkloadType>(p.workload_type)),
      injectionRate(p.injection_rate),
      numCores(p.num_cores),
      enableDeadlines(p.enable_deadlines),
      baseLatency(p.base_latency),
      matrixAccelBase(p.matrix_accel_base),
      cordicAccelBase(p.cordic_accel_base),
      compressionBase(p.compression_base),
      imageCompBase(p.image_comp_base),
      generating(false),
      random(p.seed),
      requestId(0),
      trafficPort(name() + ".traffic_port", this),
      generateEvent([this]{ generateTraffic(); }, name() + ".generate"),
      stats(this)
{
    // Default workload mix for MIXED_REALTIME
    workloadMix = {
        {GNC_ATTITUDE, 0.30},    // 30% attitude control (100Hz)
        {GNC_NAVIGATION, 0.20},  // 20% navigation (10Hz)
        {GNC_ORBIT, 0.10},       // 10% orbit propagation (1Hz)
        {TELEMETRY, 0.15},       // 15% telemetry
        {PAYLOAD_COMPRESS, 0.15},// 15% compression
        {COMMAND, 0.10}          // 10% command processing
    };

    DPRINTF(SpaceTrafficGen, "SpaceTrafficGen created: workload=%d, "
            "rate=%f req/us, cores=%d\n", primaryWorkload, injectionRate, numCores);
}

SpaceTrafficGen::~SpaceTrafficGen()
{
    while (!pendingRequests.empty()) {
        delete pendingRequests.front();
        pendingRequests.pop();
    }
}

Port &
SpaceTrafficGen::getPort(const std::string &name, PortID idx)
{
    if (name == "traffic_port")
        return trafficPort;
    return ClockedObject::getPort(name, idx);
}

void
SpaceTrafficGen::startGeneration()
{
    if (generating) return;
    
    generating = true;
    schedule(generateEvent, curTick());
    
    DPRINTF(SpaceTrafficGen, "Traffic generation started\n");
}

void
SpaceTrafficGen::stopGeneration()
{
    generating = false;
    if (generateEvent.scheduled()) {
        deschedule(generateEvent);
    }
    
    DPRINTF(SpaceTrafficGen, "Traffic generation stopped. "
            "Total: %lu, Completed: %lu\n",
            stats.totalRequests.value(), stats.completedRequests.value());
}

void
SpaceTrafficGen::setWorkloadMix(
    const std::vector<std::pair<WorkloadType, double>> &mix)
{
    workloadMix = mix;
}

void
SpaceTrafficGen::burstTraffic(WorkloadType type, int count)
{
    for (int i = 0; i < count; i++) {
        int coreId = i % numCores;
        TrafficRequest *req = nullptr;
        
        switch (type) {
            case GNC_ATTITUDE:
                req = generateAttitudeRequest(coreId);
                break;
            case GNC_ORBIT:
                req = generateOrbitRequest(coreId);
                break;
            case GNC_NAVIGATION:
                req = generateNavigationRequest(coreId);
                break;
            case PAYLOAD_IMAGE:
                req = generateImageRequest(coreId);
                break;
            case PAYLOAD_COMPRESS:
                req = generateCompressRequest(coreId);
                break;
            default:
                req = generateAttitudeRequest(coreId);
                break;
        }
        
        if (req) {
            pendingRequests.push(req);
            stats.totalRequests++;
        }
    }
    
    // Start sending if not already
    if (!generateEvent.scheduled()) {
        schedule(generateEvent, curTick());
    }
}

void
SpaceTrafficGen::generateTraffic()
{
    if (!generating && pendingRequests.empty()) return;
    
    // Determine which workload to generate
    WorkloadType type;
    if (primaryWorkload == MIXED_REALTIME || primaryWorkload == STRESS_TEST) {
        // Pick based on workload mix
        double r = random.random<double>();
        double cumulative = 0;
        type = GNC_ATTITUDE;  // default
        
        for (const auto &entry : workloadMix) {
            cumulative += entry.second;
            if (r <= cumulative) {
                type = entry.first;
                break;
            }
        }
    } else {
        type = primaryWorkload;
    }
    
    // Select core (round-robin or random based on workload)
    int coreId;
    if (type == COMMAND) {
        coreId = 0;  // Commands always to core 0
    } else if (type == TELEMETRY) {
        coreId = numCores - 1;  // Telemetry to last core
    } else {
        coreId = requestId % numCores;
    }
    
    // Generate appropriate request
    TrafficRequest *req = nullptr;
    switch (type) {
        case GNC_ATTITUDE:
            req = generateAttitudeRequest(coreId);
            break;
        case GNC_ORBIT:
            req = generateOrbitRequest(coreId);
            break;
        case GNC_NAVIGATION:
            req = generateNavigationRequest(coreId);
            break;
        case PAYLOAD_IMAGE:
            req = generateImageRequest(coreId);
            break;
        case PAYLOAD_COMPRESS:
            req = generateCompressRequest(coreId);
            break;
        case TELEMETRY:
            req = generateTelemetryRequest(coreId);
            break;
        case COMMAND:
            req = generateCommandRequest(coreId);
            break;
        default:
            req = generateAttitudeRequest(coreId);
    }
    
    if (req) {
        sendRequest(req);
        stats.totalRequests++;
        stats.perTypeRequests[static_cast<int>(type)]++;
        requestId++;
    }
    
    // Schedule next generation based on injection rate
    if (generating) {
        // injectionRate is requests per microsecond
        // Convert to ticks
        Tick interval;
        if (primaryWorkload == STRESS_TEST) {
            interval = clockPeriod();  // Maximum rate
        } else {
            interval = (Tick)(1000.0 / injectionRate);  // nanoseconds
            // Add some jitter
            interval += (Tick)(random.random<double>() * interval * 0.2);
        }
        
        schedule(generateEvent, curTick() + interval);
    }
}

SpaceTrafficGen::TrafficRequest*
SpaceTrafficGen::generateAttitudeRequest(int coreId)
{
    // Attitude control uses CORDIC for trigonometry
    // 100Hz control loop: sin/cos for attitude, atan2 for sensors
    TrafficRequest *req = new TrafficRequest();
    req->timestamp = curTick();
    req->type = GNC_ATTITUDE;
    req->priority = 10;  // High priority (real-time)
    req->coreId = coreId;
    req->targetAddr = cordicAccelBase;
    req->size = 8;
    req->isRead = false;
    
    // Simulate attitude angle input (radians scaled to fixed-point)
    double angle = random.random<double>() * 2.0 * 3.14159265359;
    req->data = (uint64_t)(angle * 65536.0);
    
    // 10ms deadline for 100Hz loop
    if (enableDeadlines) {
        req->deadline = curTick() + 10000000;  // 10ms in ticks
    }
    
    stats.cordicRequests++;
    return req;
}

SpaceTrafficGen::TrafficRequest*
SpaceTrafficGen::generateOrbitRequest(int coreId)
{
    // Orbital propagation uses matrix operations
    // State transition matrix, rotation matrices
    TrafficRequest *req = new TrafficRequest();
    req->timestamp = curTick();
    req->type = GNC_ORBIT;
    req->priority = 5;  // Medium priority
    req->coreId = coreId;
    req->targetAddr = matrixAccelBase;
    req->size = 8;
    req->isRead = false;
    
    // Command to perform 3x3 matrix multiply
    req->data = 0x01;  // Start command
    
    // 1s deadline for orbit update
    if (enableDeadlines) {
        req->deadline = curTick() + 1000000000;  // 1s
    }
    
    stats.matrixRequests++;
    return req;
}

SpaceTrafficGen::TrafficRequest*
SpaceTrafficGen::generateNavigationRequest(int coreId)
{
    // Navigation filter uses both matrix (Kalman) and CORDIC (angles)
    // Alternate between the two
    TrafficRequest *req = new TrafficRequest();
    req->timestamp = curTick();
    req->type = GNC_NAVIGATION;
    req->priority = 8;
    req->coreId = coreId;
    req->size = 8;
    req->isRead = false;
    
    if (random.random<int>() % 2 == 0) {
        // Kalman filter matrix operation
        req->targetAddr = matrixAccelBase;
        req->data = 0x01;
        stats.matrixRequests++;
    } else {
        // Sensor angle processing
        req->targetAddr = cordicAccelBase;
        req->data = (uint64_t)(random.random<double>() * 65536.0);
        stats.cordicRequests++;
    }
    
    // 100ms deadline for 10Hz filter
    if (enableDeadlines) {
        req->deadline = curTick() + 100000000;
    }
    
    return req;
}

SpaceTrafficGen::TrafficRequest*
SpaceTrafficGen::generateImageRequest(int coreId)
{
    // Image processing - burst of compression requests
    TrafficRequest *req = new TrafficRequest();
    req->timestamp = curTick();
    req->type = PAYLOAD_IMAGE;
    req->priority = 3;
    req->coreId = coreId;
    req->targetAddr = imageCompBase;
    req->size = 8;
    req->isRead = false;
    
    // Image tile index
    req->data = requestId % 256;
    
    // Best effort - no strict deadline
    req->deadline = 0;
    
    return req;
}

SpaceTrafficGen::TrafficRequest*
SpaceTrafficGen::generateCompressRequest(int coreId)
{
    // Data compression for telemetry
    TrafficRequest *req = new TrafficRequest();
    req->timestamp = curTick();
    req->type = PAYLOAD_COMPRESS;
    req->priority = 4;
    req->coreId = coreId;
    req->targetAddr = compressionBase;
    req->size = 8;
    req->isRead = false;
    
    req->data = 0x01;  // Compress command
    req->deadline = 0;
    
    stats.compressionRequests++;
    return req;
}

SpaceTrafficGen::TrafficRequest*
SpaceTrafficGen::generateTelemetryRequest(int coreId)
{
    // Telemetry - read status from accelerators
    TrafficRequest *req = new TrafficRequest();
    req->timestamp = curTick();
    req->type = TELEMETRY;
    req->priority = 6;
    req->coreId = coreId;
    req->size = 8;
    req->isRead = true;  // Read status
    
    // Round-robin through accelerator status registers
    switch (requestId % 4) {
        case 0: req->targetAddr = matrixAccelBase + 0x08; break;
        case 1: req->targetAddr = cordicAccelBase + 0x08; break;
        case 2: req->targetAddr = compressionBase + 0x08; break;
        case 3: req->targetAddr = imageCompBase + 0x08; break;
    }
    
    // 1s deadline
    if (enableDeadlines) {
        req->deadline = curTick() + 1000000000;
    }
    
    return req;
}

SpaceTrafficGen::TrafficRequest*
SpaceTrafficGen::generateCommandRequest(int coreId)
{
    // Ground command - sporadic, high priority
    TrafficRequest *req = new TrafficRequest();
    req->timestamp = curTick();
    req->type = COMMAND;
    req->priority = 15;  // Highest priority
    req->coreId = coreId;
    req->targetAddr = matrixAccelBase + 0x10;  // Command register
    req->size = 8;
    req->isRead = false;
    
    // Simulated command code
    req->data = 0xABCD0000 | (random.random<int>() & 0xFFFF);
    
    // 50ms deadline
    if (enableDeadlines) {
        req->deadline = curTick() + 50000000;
    }
    
    return req;
}

void
SpaceTrafficGen::sendRequest(TrafficRequest *req)
{
    RequestPtr request = std::make_shared<Request>(
        req->targetAddr, req->size, 0, 0);
    
    PacketPtr pkt;
    if (req->isRead) {
        pkt = new Packet(request, MemCmd::ReadReq);
        pkt->allocate();
    } else {
        pkt = new Packet(request, MemCmd::WriteReq);
        pkt->allocate();
        pkt->setLE<uint64_t>(req->data);
    }
    
    // Store request info for latency tracking
    pkt->pushSenderState(new SpaceTrafficGen::TrafficRequestSenderState(req));
    
    if (!trafficPort.sendTimingReq(pkt)) {
        // Port busy, queue for retry
        pendingRequests.push(req);
        delete pkt;
        DPRINTF(SpaceTrafficGen, "Request blocked, queued\n");
    } else {
        DPRINTF(SpaceTrafficGen, "Sent request type=%d addr=0x%lx core=%d\n",
                req->type, req->targetAddr, req->coreId);
    }
}

bool
SpaceTrafficGen::handleResponse(PacketPtr pkt)
{
    auto *state = dynamic_cast<SpaceTrafficGen::TrafficRequestSenderState*>(
        pkt->popSenderState());
    
    if (state && state->req) {
        TrafficRequest *req = state->req;
        Tick latency = curTick() - req->timestamp;
        
        stats.completedRequests++;
        stats.responseLatency.sample(latency / clockPeriod());
        stats.perTypeLatency[static_cast<int>(req->type)] += latency;
        
        // Check deadline
        if (enableDeadlines && req->deadline > 0) {
            if (curTick() <= req->deadline) {
                stats.deadlinesMet++;
            } else {
                stats.deadlinesMissed++;
                DPRINTF(SpaceTrafficGen, "DEADLINE MISSED: type=%d "
                        "latency=%lu deadline=%lu\n",
                        req->type, latency, req->deadline - req->timestamp);
            }
        }
        
        delete req;
        delete state;
    }
    
    delete pkt;
    return true;
}

void
SpaceTrafficGen::retry()
{
    if (!pendingRequests.empty()) {
        TrafficRequest *req = pendingRequests.front();
        pendingRequests.pop();
        sendRequest(req);
    }
}

SpaceTrafficGen::SpaceTrafficGenStats::SpaceTrafficGenStats(SpaceTrafficGen *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(),
               "Total requests generated"),
      ADD_STAT(completedRequests, statistics::units::Count::get(),
               "Completed requests"),
      ADD_STAT(droppedRequests, statistics::units::Count::get(),
               "Dropped requests"),
      ADD_STAT(deadlinesMet, statistics::units::Count::get(),
               "Deadlines met"),
      ADD_STAT(deadlinesMissed, statistics::units::Count::get(),
               "Deadlines missed"),
      ADD_STAT(responseLatency, statistics::units::Cycle::get(),
               "Response latency distribution"),
      ADD_STAT(perTypeRequests, statistics::units::Count::get(),
               "Requests per workload type"),
      ADD_STAT(perTypeLatency, statistics::units::Cycle::get(),
               "Latency per workload type"),
      ADD_STAT(matrixRequests, statistics::units::Count::get(),
               "Matrix accelerator requests"),
      ADD_STAT(cordicRequests, statistics::units::Count::get(),
               "CORDIC accelerator requests"),
      ADD_STAT(compressionRequests, statistics::units::Count::get(),
               "Compression requests")
{
    responseLatency.init(20);
    perTypeRequests.init(9);  // Number of workload types
    perTypeLatency.init(9);
    
    const char* typeNames[] = {
        "GNC_ATTITUDE", "GNC_ORBIT", "GNC_NAVIGATION",
        "PAYLOAD_IMAGE", "PAYLOAD_COMPRESS", "TELEMETRY",
        "COMMAND", "MIXED_REALTIME", "STRESS_TEST"
    };
    
    for (int i = 0; i < 9; i++) {
        perTypeRequests.subname(i, typeNames[i]);
        perTypeLatency.subname(i, typeNames[i]);
    }
}

} // namespace gem5

