/*
 * NOVA Processor - Space-Grade GPU (SpaceGPU) Implementation
 * PhD Research: Futuristic Spacecraft Processor
 * Author: Boul, PhD Scholar, CSA, IISc Bangalore
 */

#include "spacecraft/space_gpu.hh"
#include "base/trace.hh"
#include "debug/SpaceGPU.hh"
#include "mem/packet_access.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

namespace spacecraft
{

SpaceGPU::SpaceGPU(const Params &p)
    : ClockedObject(p),
      dmaPort(name() + ".dma_port", this),
      mmioPort(name() + ".mmio_port", this),
      currentRequest(nullptr),
      fbWidth(p.framebuffer_width),
      fbHeight(p.framebuffer_height),
      fbBase(p.framebuffer_base),
      simdLanes(p.simd_lanes),
      maxTrianglesPerBatch(p.max_triangles_per_batch),
      textureCacheSize(p.texture_cache_size),
      powerUpLatencyCycles(p.power_up_latency),
      sleepWakeLatency(p.sleep_wake_latency),
      triangleSetupLatency(p.triangle_setup_latency),
      pixelLatency(p.pixel_latency),
      textureLatency(p.texture_latency),
      computeBaseLatency(p.compute_base_latency),
      memoryLatency(p.memory_latency),
      powerState(GPUPowerState::IDLE),
      powerOffMW(p.power_off),
      powerSleepMW(p.power_sleep),
      powerIdleMW(p.power_idle),
      powerRenderMW(p.power_render),
      powerComputeMW(p.power_compute),
      powerFullMW(p.power_full),
      processEvent([this]{ startProcessing(); }, name() + ".processEvent"),
      completionEvent([this]{ completeProcessing(); }, name() + ".completionEvent"),
      instanceId(p.instance_id),
      mmioBase(p.mmio_base),
      mmioSize(p.mmio_size)
{
    resetStats();

    DPRINTF(SpaceGPU, "SpaceGPU[%d] initialized: %d SIMD lanes, FB %dx%d\n",
            instanceId, simdLanes, fbWidth, fbHeight);
    DPRINTF(SpaceGPU, "SpaceGPU[%d] MMIO at 0x%lx, size 0x%lx\n",
            instanceId, mmioBase, mmioSize);
}

SpaceGPU::~SpaceGPU()
{
    if (currentRequest) {
        delete currentRequest;
    }
}

Port &SpaceGPU::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma_port") {
        return dmaPort;
    } else if (if_name == "mmio_port") {
        return mmioPort;
    }
    return ClockedObject::getPort(if_name, idx);
}

bool SpaceGPU::submitRequest(GPURequest &req)
{
    if (powerState == GPUPowerState::OFF) {
        DPRINTF(SpaceGPU, "SpaceGPU[%d] is powered off, rejecting request\n",
                instanceId);
        return false;
    }

    req.requestTime = curTick();
    requestQueue.push(req);

    stats.totalRequests++;
    if (requestQueue.size() > stats.maxQueueDepth) {
        stats.maxQueueDepth = requestQueue.size();
    }

    DPRINTF(SpaceGPU, "SpaceGPU[%d] queued request %lu, op=%d, queue depth=%lu\n",
            instanceId, req.requestId, static_cast<int>(req.operation),
            requestQueue.size());

    // Start processing if idle
    if (!currentRequest && !processEvent.scheduled()) {
        schedule(processEvent, curTick());
    }

    return true;
}

void SpaceGPU::resetStats()
{
    stats.totalRequests = 0;
    stats.completedRequests = 0;
    stats.renderOps = 0;
    stats.computeOps = 0;
    stats.vizOps = 0;
    stats.totalCycles = 0;
    stats.busyCycles = 0;
    stats.queuedCycles = 0;
    stats.maxQueueDepth = 0;
    stats.avgLatency = 0.0;
}

void SpaceGPU::powerOff()
{
    if (powerState != GPUPowerState::OFF) {
        DPRINTF(SpaceGPU, "SpaceGPU[%d] powering off\n", instanceId);
        powerState = GPUPowerState::OFF;
    }
}

void SpaceGPU::sleepMode()
{
    if (powerState != GPUPowerState::SLEEP && powerState != GPUPowerState::OFF) {
        DPRINTF(SpaceGPU, "SpaceGPU[%d] entering sleep mode\n", instanceId);
        powerState = GPUPowerState::SLEEP;
    }
}

void SpaceGPU::powerUp()
{
    if (powerState == GPUPowerState::OFF || powerState == GPUPowerState::SLEEP) {
        DPRINTF(SpaceGPU, "SpaceGPU[%d] powering up, latency=%d cycles\n",
                instanceId, powerUpLatencyCycles);
        powerState = GPUPowerState::IDLE;
    }
}

Cycles SpaceGPU::calculateLatency(const GPURequest &req) const
{
    uint64_t pixels = (uint64_t)fbWidth * fbHeight;

    switch (req.operation) {
        case GPUOpType::CLEAR:
            // Clear entire framebuffer: pixels / SIMD lanes
            return Cycles(pixels / simdLanes + 1);

        case GPUOpType::DRAW_TRIANGLES:
        {
            // Triangle setup + rasterization
            uint64_t setupCycles = req.triangleCount * triangleSetupLatency;
            // Average triangle covers ~100 pixels
            uint64_t rasterCycles = (req.triangleCount * 100 * pixelLatency) / simdLanes;
            return Cycles(setupCycles + rasterCycles);
        }

        case GPUOpType::DRAW_POINTS:
            return Cycles(req.vertexCount * pixelLatency);

        case GPUOpType::DRAW_LINES:
            // Average line ~50 pixels
            return Cycles(req.vertexCount * 50 * pixelLatency / simdLanes);

        case GPUOpType::DRAW_TERRAIN:
            // Terrain rendering: heightmap evaluation per pixel
            return Cycles(pixels * 2 / simdLanes + memoryLatency);

        case GPUOpType::DRAW_STARS:
            // Star rendering: per-star projection + draw
            return Cycles(req.vertexCount * (10 + pixelLatency));

        case GPUOpType::THERMAL_MAP:
        case GPUOpType::HAZARD_MAP:
        case GPUOpType::DEPTH_MAP:
            // Visualization: colormap lookup per pixel
            return Cycles(pixels / simdLanes + memoryLatency);

        case GPUOpType::COMPUTE_DISPATCH:
        {
            uint64_t totalWork = req.workgroupX * req.workgroupY;
            return Cycles(totalWork * computeBaseLatency / simdLanes);
        }

        case GPUOpType::PARALLEL_REDUCE:
            // log2(N) stages, each processing N/simdLanes elements
            return Cycles(computeBaseLatency * 10);

        case GPUOpType::HISTOGRAM:
            return Cycles(pixels / simdLanes + computeBaseLatency);

        case GPUOpType::ALPHA_BLEND:
            // Per-pixel blend: 4 multiply + 4 add
            return Cycles(pixels * 2 / simdLanes);

        case GPUOpType::BUFFER_SWAP:
            return Cycles(1);

        case GPUOpType::NOP:
        default:
            return Cycles(1);
    }
}

void SpaceGPU::startProcessing()
{
    if (requestQueue.empty()) {
        return;
    }

    // Get next request
    GPURequest req = requestQueue.front();
    requestQueue.pop();

    currentRequest = new GPURequest(req);

    // Calculate latency
    Cycles latency = calculateLatency(req);

    DPRINTF(SpaceGPU, "SpaceGPU[%d] starting op=%d, latency=%d cycles\n",
            instanceId, static_cast<int>(req.operation), latency);

    // Update statistics and power state
    stats.busyCycles += latency;

    switch (req.operation) {
        case GPUOpType::CLEAR:
        case GPUOpType::DRAW_TRIANGLES:
        case GPUOpType::DRAW_POINTS:
        case GPUOpType::DRAW_LINES:
        case GPUOpType::DRAW_TERRAIN:
        case GPUOpType::DRAW_STARS:
            stats.renderOps++;
            powerState = GPUPowerState::RENDER;
            break;

        case GPUOpType::THERMAL_MAP:
        case GPUOpType::HAZARD_MAP:
        case GPUOpType::DEPTH_MAP:
            stats.vizOps++;
            powerState = GPUPowerState::RENDER;
            break;

        case GPUOpType::COMPUTE_DISPATCH:
        case GPUOpType::PARALLEL_REDUCE:
        case GPUOpType::HISTOGRAM:
            stats.computeOps++;
            powerState = GPUPowerState::COMPUTE;
            break;

        default:
            break;
    }

    // Schedule completion
    schedule(completionEvent, clockEdge(latency));
}

void SpaceGPU::completeProcessing()
{
    assert(currentRequest != nullptr);

    Tick latency = curTick() - currentRequest->requestTime;

    DPRINTF(SpaceGPU, "SpaceGPU[%d] completed request %lu, latency=%lu ticks\n",
            instanceId, currentRequest->requestId, latency);

    stats.completedRequests++;
    stats.totalCycles += latency / clockPeriod();

    // Update average latency
    if (stats.completedRequests > 0) {
        stats.avgLatency = (stats.avgLatency * (stats.completedRequests - 1) + latency)
                           / stats.completedRequests;
    }

    // Call completion callback if provided
    if (currentRequest->callback) {
        currentRequest->callback(true);
    }

    // Return to idle power state
    powerState = GPUPowerState::IDLE;

    delete currentRequest;
    currentRequest = nullptr;

    // Process next request if available
    processNextRequest();
}

void SpaceGPU::processNextRequest()
{
    if (!requestQueue.empty() && !processEvent.scheduled()) {
        schedule(processEvent, curTick());
    }
}

// MMIO Port implementation
AddrRangeList SpaceGPU::GPUMMIOPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(gpu->mmioBase, gpu->mmioBase + gpu->mmioSize));
    return ranges;
}

Tick SpaceGPU::GPUMMIOPort::recvAtomic(PacketPtr pkt)
{
    if (pkt->isRead()) {
        return gpu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        return gpu->handleMMIOWrite(pkt);
    }
    return 1;
}

bool SpaceGPU::GPUMMIOPort::recvTimingReq(PacketPtr pkt)
{
    if (pkt->isRead()) {
        gpu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        gpu->handleMMIOWrite(pkt);
    }

    pkt->makeResponse();
    sendTimingResp(pkt);
    return true;
}

void SpaceGPU::GPUMMIOPort::recvFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        gpu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        gpu->handleMMIOWrite(pkt);
    }
}

void SpaceGPU::GPUMMIOPort::recvRespRetry()
{
    // Handle retry
}

Tick SpaceGPU::handleMMIORead(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;
    uint64_t data = 0;

    switch (offset) {
        case GPUReg::STATUS:
            // Bit 0: busy, Bit 1: queue not empty, Bits 4:2: power state
            data = (currentRequest != nullptr ? 1 : 0) |
                   (!requestQueue.empty() ? 2 : 0) |
                   (static_cast<uint64_t>(powerState) << 2);
            break;

        case GPUReg::FB_WIDTH:
            data = fbWidth;
            break;

        case GPUReg::FB_HEIGHT:
            data = fbHeight;
            break;

        case GPUReg::STATS_TOTAL:
            data = stats.totalRequests;
            break;

        case GPUReg::STATS_BUSY:
            data = stats.busyCycles;
            break;

        case GPUReg::LATENCY:
            data = static_cast<uint64_t>(stats.avgLatency);
            break;

        default:
            DPRINTF(SpaceGPU, "SpaceGPU[%d] unknown MMIO read at offset 0x%lx\n",
                    instanceId, offset);
            break;
    }

    pkt->setLE(data);
    return clockPeriod();
}

Tick SpaceGPU::handleMMIOWrite(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;
    uint64_t data = pkt->getLE<uint64_t>();

    static GPURequest pendingReq;

    switch (offset) {
        case GPUReg::CONTROL:
            if (data == 1) {
                // Start operation
                submitRequest(pendingReq);
                pendingReq = GPURequest();
            } else if (data == 2) {
                // Power off
                powerOff();
            } else if (data == 3) {
                // Power up
                powerUp();
            } else if (data == 4) {
                // Sleep
                sleepMode();
            }
            break;

        case GPUReg::DRAW_CMD:
            pendingReq.operation = static_cast<GPUOpType>(data);
            break;

        case GPUReg::VERTEX_BASE:
            pendingReq.vertexAddr = data;
            break;

        case GPUReg::VERTEX_CNT:
            pendingReq.vertexCount = data;
            break;

        case GPUReg::COMPUTE_CMD:
            pendingReq.operation = static_cast<GPUOpType>(data);
            break;

        case GPUReg::WORKGROUP_X:
            pendingReq.workgroupX = data;
            break;

        case GPUReg::WORKGROUP_Y:
            pendingReq.workgroupY = data;
            break;

        default:
            DPRINTF(SpaceGPU, "SpaceGPU[%d] unknown MMIO write at offset 0x%lx\n",
                    instanceId, offset);
            break;
    }

    return clockPeriod();
}

// DMA Port implementation
bool SpaceGPU::GPUDmaPort::recvTimingResp(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void SpaceGPU::GPUDmaPort::recvReqRetry()
{
    // Handle retry
}

} // namespace spacecraft
} // namespace gem5
