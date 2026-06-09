/*
 * NOVA Processor - Vision Processing Unit (VPU) Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/vision_processing_unit.hh"
#include "base/trace.hh"
#include "debug/VPU.hh"
#include "mem/packet_access.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

namespace spacecraft
{

VisionProcessingUnit::VisionProcessingUnit(const Params &p)
    : ClockedObject(p),
      dmaPort(name() + ".dma_port", this),
      mmioPort(name() + ".mmio_port", this),
      currentRequest(nullptr),
      imageBufferA(BUFFER_SIZE, 0),
      imageBufferB(BUFFER_SIZE, 0),
      activeBuffer(0),
      numMACUnits(p.num_mac_units),
      maxKernelSize(p.max_kernel_size),
      debayerLatency(p.debayer_latency),
      toneMappingLatency(p.tone_mapping_latency),
      gammaLatency(p.gamma_latency),
      sobelLatency(p.sobel_latency),
      cannyLatency(p.canny_latency),
      harrisLatency(p.harris_latency),
      orbLatency(p.orb_latency),
      opticalFlowLatency(p.optical_flow_latency),
      convBaseLatency(p.conv_base_latency),
      scratchpadSize(p.scratchpad_size_kb * 1024),
      poweredOn(true),
      powerUpLatency(p.power_up_latency),
      processEvent([this]{ startProcessing(); }, name() + ".processEvent"),
      completionEvent([this]{ completeProcessing(); }, name() + ".completionEvent"),
      instanceId(p.instance_id),
      mmioBase(p.mmio_base),
      mmioSize(p.mmio_size)
{
    resetStats();
    
    DPRINTF(VPU, "VPU[%d] initialized: %d MAC units, max kernel %dx%d\n",
            instanceId, numMACUnits, maxKernelSize, maxKernelSize);
    DPRINTF(VPU, "VPU[%d] MMIO at 0x%lx, size 0x%lx\n",
            instanceId, mmioBase, mmioSize);
}

VisionProcessingUnit::~VisionProcessingUnit()
{
    if (currentRequest) {
        delete currentRequest;
    }
}

Port &VisionProcessingUnit::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma_port") {
        return dmaPort;
    } else if (if_name == "mmio_port") {
        return mmioPort;
    }
    return ClockedObject::getPort(if_name, idx);
}

bool VisionProcessingUnit::submitRequest(VPURequest &req)
{
    if (!poweredOn) {
        DPRINTF(VPU, "VPU[%d] is powered off, rejecting request\n", instanceId);
        return false;
    }
    
    req.requestTime = curTick();
    requestQueue.push(req);
    
    stats.totalRequests++;
    if (requestQueue.size() > stats.maxQueueDepth) {
        stats.maxQueueDepth = requestQueue.size();
    }
    
    DPRINTF(VPU, "VPU[%d] queued request %lu, op=%d, queue depth=%lu\n",
            instanceId, req.requestId, static_cast<int>(req.operation),
            requestQueue.size());
    
    // Start processing if idle
    if (!currentRequest && !processEvent.scheduled()) {
        schedule(processEvent, curTick());
    }
    
    return true;
}

void VisionProcessingUnit::resetStats()
{
    stats.totalRequests = 0;
    stats.completedRequests = 0;
    stats.debayerOps = 0;
    stats.edgeDetectOps = 0;
    stats.featureExtractOps = 0;
    stats.convolutionOps = 0;
    stats.totalCycles = 0;
    stats.busyCycles = 0;
    stats.queuedCycles = 0;
    stats.maxQueueDepth = 0;
    stats.avgLatency = 0.0;
}

void VisionProcessingUnit::powerGate()
{
    if (poweredOn) {
        DPRINTF(VPU, "VPU[%d] entering power-gated state\n", instanceId);
        poweredOn = false;
    }
}

void VisionProcessingUnit::powerUp()
{
    if (!poweredOn) {
        DPRINTF(VPU, "VPU[%d] powering up, latency=%d cycles\n",
                instanceId, powerUpLatency);
        poweredOn = true;
    }
}

Cycles VisionProcessingUnit::calculateLatency(const VPURequest &req) const
{
    switch (req.operation) {
        case VPUOpType::DEBAYER:
            return debayerLatency;
            
        case VPUOpType::TONE_MAP:
            return toneMappingLatency;
            
        case VPUOpType::GAMMA_CORRECT:
            return gammaLatency;
            
        case VPUOpType::RESIZE:
        case VPUOpType::CROP:
            // Proportional to output size
            return Cycles((req.width * req.height) / numMACUnits);
            
        case VPUOpType::SOBEL_X:
        case VPUOpType::SOBEL_Y:
        case VPUOpType::SOBEL_MAG:
            return sobelLatency;
            
        case VPUOpType::CANNY:
            return cannyLatency;
            
        case VPUOpType::HARRIS_CORNERS:
            return harrisLatency;
            
        case VPUOpType::ORB_FEATURES:
            return orbLatency;
            
        case VPUOpType::OPTICAL_FLOW:
            return opticalFlowLatency;
            
        case VPUOpType::CONV_3X3:
            return calculateConvCycles(req.height, req.width, 
                                       req.channels, req.channels, 3);
            
        case VPUOpType::CONV_5X5:
            return calculateConvCycles(req.height, req.width,
                                       req.channels, req.channels, 5);
            
        case VPUOpType::CONV_7X7:
            return calculateConvCycles(req.height, req.width,
                                       req.channels, req.channels, 7);
            
        case VPUOpType::DEPTHWISE_CONV:
            // Depthwise is cheaper - only K*K per pixel
            return Cycles((req.height * req.width * req.kernelSize * req.kernelSize) 
                         / numMACUnits);
            
        case VPUOpType::HISTOGRAM:
            return Cycles(req.width * req.height / 8);  // 8 pixels per cycle
            
        case VPUOpType::THRESHOLD:
            return Cycles(req.width * req.height / 16);  // 16 pixels per cycle
            
        case VPUOpType::NOP:
        default:
            return Cycles(1);
    }
}

Cycles VisionProcessingUnit::calculateConvCycles(uint32_t H, uint32_t W,
                                                  uint32_t Cin, uint32_t Cout,
                                                  uint32_t K) const
{
    // Total MACs = H * W * Cin * Cout * K * K
    uint64_t totalMACs = (uint64_t)H * W * Cin * Cout * K * K;
    
    // Cycles = MACs / num_mac_units + memory overhead
    uint64_t computeCycles = totalMACs / numMACUnits;
    
    // Memory transfer overhead (input + output + kernel)
    uint64_t inputSize = H * W * Cin;
    uint64_t outputSize = H * W * Cout;
    uint64_t kernelSize = Cin * Cout * K * K;
    uint64_t memoryOverhead = (inputSize + outputSize + kernelSize) / 64;  // 64B per cycle
    
    return Cycles(computeCycles + memoryOverhead + convBaseLatency);
}

void VisionProcessingUnit::startProcessing()
{
    if (requestQueue.empty()) {
        return;
    }
    
    // Get next request
    VPURequest req = requestQueue.front();
    requestQueue.pop();
    
    currentRequest = new VPURequest(req);
    
    // Calculate latency
    Cycles latency = calculateLatency(req);
    
    DPRINTF(VPU, "VPU[%d] starting op=%d, size=%dx%d, latency=%d cycles\n",
            instanceId, static_cast<int>(req.operation),
            req.width, req.height, latency);
    
    // Update statistics
    stats.busyCycles += latency;
    
    switch (req.operation) {
        case VPUOpType::DEBAYER:
        case VPUOpType::TONE_MAP:
        case VPUOpType::GAMMA_CORRECT:
        case VPUOpType::RESIZE:
        case VPUOpType::CROP:
            // Preprocessing ops
            break;
            
        case VPUOpType::SOBEL_X:
        case VPUOpType::SOBEL_Y:
        case VPUOpType::SOBEL_MAG:
        case VPUOpType::CANNY:
            stats.edgeDetectOps++;
            break;
            
        case VPUOpType::HARRIS_CORNERS:
        case VPUOpType::ORB_FEATURES:
        case VPUOpType::OPTICAL_FLOW:
            stats.featureExtractOps++;
            break;
            
        case VPUOpType::CONV_3X3:
        case VPUOpType::CONV_5X5:
        case VPUOpType::CONV_7X7:
        case VPUOpType::DEPTHWISE_CONV:
            stats.convolutionOps++;
            break;
            
        default:
            break;
    }
    
    // Schedule completion
    schedule(completionEvent, clockEdge(latency));
}

void VisionProcessingUnit::completeProcessing()
{
    assert(currentRequest != nullptr);
    
    Tick latency = curTick() - currentRequest->requestTime;
    
    DPRINTF(VPU, "VPU[%d] completed request %lu, latency=%lu ticks\n",
            instanceId, currentRequest->requestId, latency);
    
    stats.completedRequests++;
    stats.totalCycles += latency / clockPeriod();
    
    // Update average latency
    stats.avgLatency = (stats.avgLatency * (stats.completedRequests - 1) + latency) 
                       / stats.completedRequests;
    
    // Call completion callback if provided
    if (currentRequest->callback) {
        currentRequest->callback(true);
    }
    
    // Swap buffers (ping-pong)
    activeBuffer = 1 - activeBuffer;
    
    delete currentRequest;
    currentRequest = nullptr;
    
    // Process next request if available
    processNextRequest();
}

void VisionProcessingUnit::processNextRequest()
{
    if (!requestQueue.empty() && !processEvent.scheduled()) {
        schedule(processEvent, curTick());
    }
}

// MMIO Port implementation
AddrRangeList VisionProcessingUnit::MMIOPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(vpu->mmioBase, vpu->mmioBase + vpu->mmioSize));
    return ranges;
}

Tick VisionProcessingUnit::MMIOPort::recvAtomic(PacketPtr pkt)
{
    if (pkt->isRead()) {
        return vpu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        return vpu->handleMMIOWrite(pkt);
    }
    return 1;
}

bool VisionProcessingUnit::MMIOPort::recvTimingReq(PacketPtr pkt)
{
    if (pkt->isRead()) {
        vpu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        vpu->handleMMIOWrite(pkt);
    }
    
    pkt->makeResponse();
    sendTimingResp(pkt);
    return true;
}

void VisionProcessingUnit::MMIOPort::recvFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        vpu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        vpu->handleMMIOWrite(pkt);
    }
}

void VisionProcessingUnit::MMIOPort::recvRespRetry()
{
    // Handle retry
}

Tick VisionProcessingUnit::handleMMIORead(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;
    uint64_t data = 0;
    
    switch (offset) {
        case VPUReg::STATUS:
            // Bit 0: busy, Bit 1: queue not empty, Bit 2: powered on
            data = (currentRequest != nullptr ? 1 : 0) |
                   (!requestQueue.empty() ? 2 : 0) |
                   (poweredOn ? 4 : 0);
            break;
            
        case VPUReg::RESULT:
            // Return number of keypoints found (for feature extraction)
            data = 0;  // Placeholder
            break;
            
        case VPUReg::LATENCY:
            data = stats.avgLatency;
            break;
            
        case VPUReg::STATS_TOTAL:
            data = stats.totalRequests;
            break;
            
        case VPUReg::STATS_BUSY:
            data = stats.busyCycles;
            break;
            
        default:
            DPRINTF(VPU, "VPU[%d] unknown MMIO read at offset 0x%lx\n",
                    instanceId, offset);
            break;
    }
    
    pkt->setLE(data);
    return clockPeriod();
}

Tick VisionProcessingUnit::handleMMIOWrite(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;
    uint64_t data = pkt->getLE<uint64_t>();
    
    static VPURequest pendingReq;
    
    switch (offset) {
        case VPUReg::CONTROL:
            if (data == 1) {
                // Start operation
                submitRequest(pendingReq);
                pendingReq = VPURequest();
            } else if (data == 2) {
                // Power gate
                powerGate();
            } else if (data == 3) {
                // Power up
                powerUp();
            }
            break;
            
        case VPUReg::OP_TYPE:
            pendingReq.operation = static_cast<VPUOpType>(data);
            break;
            
        case VPUReg::IMG_WIDTH:
            pendingReq.width = data;
            break;
            
        case VPUReg::IMG_HEIGHT:
            pendingReq.height = data;
            break;
            
        case VPUReg::IMG_CHANNELS:
            pendingReq.channels = data;
            break;
            
        case VPUReg::KERNEL_SIZE:
            pendingReq.kernelSize = data;
            break;
            
        case VPUReg::INPUT_ADDR:
            pendingReq.inputAddr = data;
            break;
            
        case VPUReg::OUTPUT_ADDR:
            pendingReq.outputAddr = data;
            break;
            
        case VPUReg::KERNEL_ADDR:
            pendingReq.kernelAddr = data;
            break;
            
        default:
            DPRINTF(VPU, "VPU[%d] unknown MMIO write at offset 0x%lx\n",
                    instanceId, offset);
            break;
    }
    
    return clockPeriod();
}

// DMA Port implementation
bool VisionProcessingUnit::VPUPort::recvTimingResp(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void VisionProcessingUnit::VPUPort::recvReqRetry()
{
    // Handle retry
}

} // namespace spacecraft
} // namespace gem5

