/*
 * NOVA Processor - Neural Processing Unit (NPU) Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/neural_processing_unit.hh"
#include "base/trace.hh"
#include "debug/NPU.hh"
#include "mem/packet_access.hh"
#include "sim/sim_exit.hh"

#include <algorithm>
#include <cmath>

namespace gem5
{

namespace spacecraft
{

NeuralProcessingUnit::NeuralProcessingUnit(const Params &p)
    : ClockedObject(p),
      dmaPort(name() + ".dma_port", this),
      mmioPort(name() + ".mmio_port", this),
      systolicArray(std::make_unique<SystolicArray>(p.array_size, p.array_size)),
      currentRequest(nullptr),
      currentLayerIndex(0),
      weightCache(p.weight_cache_kb * 1024, 0),
      activationScratchpad(p.activation_spad_kb * 1024, 0),
      weightCacheSize(p.weight_cache_kb * 1024),
      activationSpadSize(p.activation_spad_kb * 1024),
      arraySize(p.array_size),
      sparseSupport(p.sparse_support),
      dmaLatencyPerByte(p.dma_latency_per_byte),
      activationLatency(p.activation_latency),
      poolingLatency(p.pooling_latency),
      batchNormLatency(p.batch_norm_latency),
      poweredOn(true),
      powerUpLatency(p.power_up_latency),
      frequencyScale(1.0),
      processEvent([this]{ startProcessing(); }, name() + ".processEvent"),
      layerCompleteEvent([this]{ completeLayer(); }, name() + ".layerCompleteEvent"),
      inferenceCompleteEvent([this]{ completeInference(); }, name() + ".inferenceCompleteEvent"),
      instanceId(p.instance_id),
      mmioBase(p.mmio_base),
      mmioSize(p.mmio_size)
{
    resetStats();
    
    DPRINTF(NPU, "NPU[%d] initialized: %dx%d systolic array, %dKB weight cache, %dKB activation spad\n",
            instanceId, arraySize, arraySize, 
            p.weight_cache_kb, p.activation_spad_kb);
    DPRINTF(NPU, "NPU[%d] MMIO at 0x%lx, size 0x%lx, sparse support=%d\n",
            instanceId, mmioBase, mmioSize, sparseSupport);
}

NeuralProcessingUnit::~NeuralProcessingUnit()
{
    if (currentRequest) {
        delete currentRequest;
    }
}

Port &NeuralProcessingUnit::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma_port") {
        return dmaPort;
    } else if (if_name == "mmio_port") {
        return mmioPort;
    }
    return ClockedObject::getPort(if_name, idx);
}

bool NeuralProcessingUnit::submitInference(NPURequest &req)
{
    if (!poweredOn) {
        DPRINTF(NPU, "NPU[%d] is powered off, rejecting inference\n", instanceId);
        return false;
    }
    
    req.requestTime = curTick();
    requestQueue.push(req);
    
    stats.totalInferences++;
    if (requestQueue.size() > stats.maxQueueDepth) {
        stats.maxQueueDepth = requestQueue.size();
    }
    
    DPRINTF(NPU, "NPU[%d] queued inference request %lu, model=%s, layers=%u\n",
            instanceId, req.requestId, req.modelName.c_str(), req.numLayers);
    
    // Start processing if idle
    if (!currentRequest && !processEvent.scheduled()) {
        schedule(processEvent, curTick());
    }
    
    return true;
}

bool NeuralProcessingUnit::executeLayer(const LayerParams &layer, int coreId)
{
    if (!poweredOn) {
        return false;
    }
    
    // Create a single-layer inference request
    NPURequest req;
    req.requestId = stats.totalInferences++;
    req.coreId = coreId;
    req.numLayers = 1;
    req.layers.push_back(layer);
    req.requestTime = curTick();
    
    return submitInference(req);
}

void NeuralProcessingUnit::resetStats()
{
    stats.totalInferences = 0;
    stats.completedInferences = 0;
    stats.totalLayers = 0;
    stats.convLayers = 0;
    stats.fcLayers = 0;
    stats.poolingLayers = 0;
    stats.activationLayers = 0;
    stats.totalMACs = 0;
    stats.busyCycles = 0;
    stats.idleCycles = 0;
    stats.avgInferenceLatency = 0.0;
    stats.avgUtilization = 0.0;
    stats.maxQueueDepth = 0;
    stats.weightCacheHits = 0;
    stats.weightCacheMisses = 0;
    
    lastPerfCounters = NPUPerfCounters{};
}

void NeuralProcessingUnit::powerGate()
{
    if (poweredOn) {
        DPRINTF(NPU, "NPU[%d] entering power-gated state\n", instanceId);
        poweredOn = false;
    }
}

void NeuralProcessingUnit::powerUp()
{
    if (!poweredOn) {
        DPRINTF(NPU, "NPU[%d] powering up, latency=%d cycles\n",
                instanceId, powerUpLatency);
        poweredOn = true;
    }
}

void NeuralProcessingUnit::setFrequencyScale(float scale)
{
    frequencyScale = std::max(0.1f, std::min(1.0f, scale));
    DPRINTF(NPU, "NPU[%d] frequency scale set to %.2f\n", instanceId, frequencyScale);
}

Cycles NeuralProcessingUnit::calculateLayerLatency(const LayerParams &layer) const
{
    Cycles latency(0);
    
    switch (layer.type) {
        case NPULayerType::CONV_STANDARD:
        case NPULayerType::CONV_DEPTHWISE:
        case NPULayerType::CONV_POINTWISE:
            latency = executeConvLayerLatency(layer);
            break;
            
        case NPULayerType::FULLY_CONNECTED:
            latency = executeFCLayerLatency(layer);
            break;
            
        case NPULayerType::POOLING_MAX:
        case NPULayerType::POOLING_AVG:
            latency = poolingLatency;
            break;
            
        case NPULayerType::ACTIVATION_RELU:
        case NPULayerType::ACTIVATION_RELU6:
        case NPULayerType::ACTIVATION_HARDSWISH:
        case NPULayerType::ACTIVATION_SIGMOID:
        case NPULayerType::SOFTMAX:
            latency = activationLatency;
            break;
            
        case NPULayerType::BATCH_NORM:
            latency = batchNormLatency;
            break;
            
        case NPULayerType::ADD:
        case NPULayerType::CONCAT:
            // Element-wise ops are fast
            latency = Cycles(layer.outputH * layer.outputW * layer.outputC / (arraySize * arraySize));
            break;
            
        default:
            latency = Cycles(1);
    }
    
    // Apply frequency scaling (lower frequency = more cycles)
    return Cycles(static_cast<uint64_t>(latency / frequencyScale));
}

Cycles NeuralProcessingUnit::executeConvLayerLatency(const LayerParams &layer) const
{
    uint64_t macs = calculateConvMACs(layer);
    
    // Effective MACs after sparsity skipping
    if (sparseSupport && layer.sparsity > 0) {
        macs = static_cast<uint64_t>(macs * (1.0 - layer.sparsity));
    }
    
    // Systolic array throughput: arraySize^2 MACs per cycle
    uint64_t macThroughput = arraySize * arraySize;
    Cycles computeCycles(macs / macThroughput);
    
    // Memory transfer overhead
    size_t inputBytes = layer.inputH * layer.inputW * layer.inputC;
    size_t outputBytes = layer.outputH * layer.outputW * layer.outputC;
    size_t weightBytes = layer.kernelH * layer.kernelW * layer.inputC * layer.outputC;
    
    Cycles dmaCycles = estimateDMACycles(inputBytes + outputBytes + weightBytes);
    
    return computeCycles + dmaCycles;
}

Cycles NeuralProcessingUnit::executeFCLayerLatency(const LayerParams &layer) const
{
    uint64_t macs = calculateFCMACs(layer);
    
    if (sparseSupport && layer.sparsity > 0) {
        macs = static_cast<uint64_t>(macs * (1.0 - layer.sparsity));
    }
    
    uint64_t macThroughput = arraySize * arraySize;
    Cycles computeCycles(macs / macThroughput);
    
    // FC layers have significant weight loading overhead
    size_t weightBytes = layer.inputC * layer.outputC;
    Cycles dmaCycles = estimateDMACycles(weightBytes);
    
    return computeCycles + dmaCycles;
}

uint64_t NeuralProcessingUnit::calculateConvMACs(const LayerParams &layer) const
{
    // MACs = H_out * W_out * C_in * C_out * K_h * K_w
    return (uint64_t)layer.outputH * layer.outputW * 
           layer.inputC * layer.outputC * 
           layer.kernelH * layer.kernelW;
}

uint64_t NeuralProcessingUnit::calculateFCMACs(const LayerParams &layer) const
{
    // MACs = input_size * output_size
    return (uint64_t)layer.inputC * layer.outputC;
}

Cycles NeuralProcessingUnit::estimateDMACycles(size_t bytes) const
{
    return Cycles(bytes * static_cast<uint64_t>(dmaLatencyPerByte));
}

void NeuralProcessingUnit::startProcessing()
{
    if (requestQueue.empty()) {
        return;
    }
    
    // Get next request
    NPURequest req = requestQueue.front();
    requestQueue.pop();
    
    currentRequest = new NPURequest(req);
    currentLayerIndex = 0;
    
    // Reset performance counters for this inference
    lastPerfCounters = NPUPerfCounters{};
    
    DPRINTF(NPU, "NPU[%d] starting inference %lu, model=%s, %u layers\n",
            instanceId, req.requestId, req.modelName.c_str(), req.numLayers);
    
    // Start processing first layer
    processLayer();
}

void NeuralProcessingUnit::processLayer()
{
    if (!currentRequest || currentLayerIndex >= currentRequest->layers.size()) {
        // All layers done
        schedule(inferenceCompleteEvent, curTick());
        return;
    }
    
    const LayerParams &layer = currentRequest->layers[currentLayerIndex];
    Cycles latency = calculateLayerLatency(layer);
    
    DPRINTF(NPU, "NPU[%d] processing layer %d/%lu, type=%d, latency=%d cycles\n",
            instanceId, currentLayerIndex, currentRequest->layers.size(),
            static_cast<int>(layer.type), latency);
    
    // Update statistics
    stats.totalLayers++;
    stats.busyCycles += latency;
    
    switch (layer.type) {
        case NPULayerType::CONV_STANDARD:
        case NPULayerType::CONV_DEPTHWISE:
        case NPULayerType::CONV_POINTWISE:
            stats.convLayers++;
            lastPerfCounters.totalMACs += calculateConvMACs(layer);
            break;
            
        case NPULayerType::FULLY_CONNECTED:
            stats.fcLayers++;
            lastPerfCounters.totalMACs += calculateFCMACs(layer);
            break;
            
        case NPULayerType::POOLING_MAX:
        case NPULayerType::POOLING_AVG:
            stats.poolingLayers++;
            break;
            
        case NPULayerType::ACTIVATION_RELU:
        case NPULayerType::ACTIVATION_RELU6:
        case NPULayerType::ACTIVATION_HARDSWISH:
        case NPULayerType::ACTIVATION_SIGMOID:
        case NPULayerType::SOFTMAX:
            stats.activationLayers++;
            break;
            
        default:
            break;
    }
    
    lastPerfCounters.totalCycles += latency;
    
    // Schedule layer completion
    schedule(layerCompleteEvent, clockEdge(latency));
}

void NeuralProcessingUnit::completeLayer()
{
    DPRINTF(NPU, "NPU[%d] completed layer %d\n", instanceId, currentLayerIndex);
    
    currentLayerIndex++;
    
    // Process next layer
    processLayer();
}

void NeuralProcessingUnit::completeInference()
{
    assert(currentRequest != nullptr);
    
    Tick latency = curTick() - currentRequest->requestTime;
    
    DPRINTF(NPU, "NPU[%d] completed inference %lu, latency=%lu ticks, MACs=%lu\n",
            instanceId, currentRequest->requestId, latency, lastPerfCounters.totalMACs);
    
    stats.completedInferences++;
    stats.totalMACs += lastPerfCounters.totalMACs;
    
    // Update average inference latency
    stats.avgInferenceLatency = (stats.avgInferenceLatency * (stats.completedInferences - 1) + latency)
                                / stats.completedInferences;
    
    // Calculate utilization
    lastPerfCounters.utilization = (float)lastPerfCounters.computeCycles / lastPerfCounters.totalCycles;
    
    // Call completion callback if provided
    if (currentRequest->callback) {
        std::vector<float> results;  // Placeholder for actual output
        currentRequest->callback(true, results);
    }
    
    delete currentRequest;
    currentRequest = nullptr;
    currentLayerIndex = 0;
    
    // Process next request if available
    processNextRequest();
}

void NeuralProcessingUnit::processNextRequest()
{
    if (!requestQueue.empty() && !processEvent.scheduled()) {
        schedule(processEvent, curTick());
    }
}

// Systolic Array implementation
void NeuralProcessingUnit::SystolicArray::computeMatMul(
    const std::vector<int8_t>& input,
    std::vector<int32_t>& output)
{
    // Simplified systolic array simulation
    // In real hardware, data flows through the array
    for (int i = 0; i < numCols; i++) {
        accumulators[i] = 0;
        for (int j = 0; j < numRows; j++) {
            if (j < input.size()) {
                accumulators[i] += input[j] * weights[j * numCols + i];
            }
        }
        if (i < output.size()) {
            output[i] = accumulators[i];
        }
    }
}

void NeuralProcessingUnit::SystolicArray::loadWeights(const std::vector<int8_t>& w)
{
    size_t copySize = std::min(w.size(), weights.size());
    std::copy(w.begin(), w.begin() + copySize, weights.begin());
}

// Activation Unit implementation
void NeuralProcessingUnit::ActivationUnit::relu(std::vector<int32_t>& data)
{
    for (auto& val : data) {
        val = std::max(0, val);
    }
}

void NeuralProcessingUnit::ActivationUnit::relu6(std::vector<int32_t>& data)
{
    for (auto& val : data) {
        val = std::min(std::max(0, val), 6 * 256);  // Scaled for quantization
    }
}

void NeuralProcessingUnit::ActivationUnit::hardswish(std::vector<int32_t>& data)
{
    for (auto& val : data) {
        // Approximation: x * relu6(x + 3) / 6
        int32_t relu6_val = std::min(std::max(0, val + 3 * 256), 6 * 256);
        val = (val * relu6_val) / (6 * 256);
    }
}

void NeuralProcessingUnit::ActivationUnit::sigmoid(std::vector<int32_t>& data)
{
    for (auto& val : data) {
        // Piecewise linear approximation
        if (val < -4 * 256) {
            val = 0;
        } else if (val > 4 * 256) {
            val = 256;  // 1.0 in Q8
        } else {
            val = 128 + val / 8;  // Linear approximation
        }
    }
}

void NeuralProcessingUnit::ActivationUnit::softmax(
    std::vector<int32_t>& data, 
    std::vector<float>& output)
{
    // Find max for numerical stability
    int32_t maxVal = *std::max_element(data.begin(), data.end());
    
    float sum = 0.0f;
    output.resize(data.size());
    
    for (size_t i = 0; i < data.size(); i++) {
        output[i] = std::exp((data[i] - maxVal) / 256.0f);
        sum += output[i];
    }
    
    for (auto& val : output) {
        val /= sum;
    }
}

// MMIO Port implementation
AddrRangeList NeuralProcessingUnit::MMIOPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(npu->mmioBase, npu->mmioBase + npu->mmioSize));
    return ranges;
}

Tick NeuralProcessingUnit::MMIOPort::recvAtomic(PacketPtr pkt)
{
    if (pkt->isRead()) {
        return npu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        return npu->handleMMIOWrite(pkt);
    }
    return 1;
}

bool NeuralProcessingUnit::MMIOPort::recvTimingReq(PacketPtr pkt)
{
    if (pkt->isRead()) {
        npu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        npu->handleMMIOWrite(pkt);
    }
    
    pkt->makeResponse();
    sendTimingResp(pkt);
    return true;
}

void NeuralProcessingUnit::MMIOPort::recvFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        npu->handleMMIORead(pkt);
    } else if (pkt->isWrite()) {
        npu->handleMMIOWrite(pkt);
    }
}

void NeuralProcessingUnit::MMIOPort::recvRespRetry()
{
    // Handle retry
}

Tick NeuralProcessingUnit::handleMMIORead(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;
    uint64_t data = 0;
    
    switch (offset) {
        case NPUReg::STATUS:
            data = (currentRequest != nullptr ? 1 : 0) |
                   (!requestQueue.empty() ? 2 : 0) |
                   (poweredOn ? 4 : 0);
            break;
            
        case NPUReg::PERF_MACS:
            data = lastPerfCounters.totalMACs;
            break;
            
        case NPUReg::PERF_CYCLES:
            data = lastPerfCounters.totalCycles;
            break;
            
        case NPUReg::PERF_UTIL:
            data = static_cast<uint64_t>(lastPerfCounters.utilization * 100);
            break;
            
        case NPUReg::STATS_INFER:
            data = stats.completedInferences;
            break;
            
        case NPUReg::STATS_LAYERS:
            data = stats.totalLayers;
            break;
            
        case NPUReg::DVFS_SCALE:
            data = static_cast<uint64_t>(frequencyScale * 100);
            break;
            
        default:
            DPRINTF(NPU, "NPU[%d] unknown MMIO read at offset 0x%lx\n",
                    instanceId, offset);
            break;
    }
    
    pkt->setLE(data);
    return clockPeriod();
}

Tick NeuralProcessingUnit::handleMMIOWrite(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;
    uint64_t data = pkt->getLE<uint64_t>();
    
    static LayerParams pendingLayer;
    
    switch (offset) {
        case NPUReg::CONTROL:
            if (data == 1) {
                // Execute single layer
                executeLayer(pendingLayer, 0);
                pendingLayer = LayerParams{};
            } else if (data == 2) {
                powerGate();
            } else if (data == 3) {
                powerUp();
            }
            break;
            
        case NPUReg::LAYER_TYPE:
            pendingLayer.type = static_cast<NPULayerType>(data);
            break;
            
        case NPUReg::INPUT_H:
            pendingLayer.inputH = data;
            break;
            
        case NPUReg::INPUT_W:
            pendingLayer.inputW = data;
            break;
            
        case NPUReg::INPUT_C:
            pendingLayer.inputC = data;
            break;
            
        case NPUReg::OUTPUT_H:
            pendingLayer.outputH = data;
            break;
            
        case NPUReg::OUTPUT_W:
            pendingLayer.outputW = data;
            break;
            
        case NPUReg::OUTPUT_C:
            pendingLayer.outputC = data;
            break;
            
        case NPUReg::KERNEL_H:
            pendingLayer.kernelH = data;
            break;
            
        case NPUReg::KERNEL_W:
            pendingLayer.kernelW = data;
            break;
            
        case NPUReg::STRIDE:
            pendingLayer.strideH = data & 0xFFFF;
            pendingLayer.strideW = (data >> 16) & 0xFFFF;
            break;
            
        case NPUReg::PADDING:
            pendingLayer.padH = data & 0xFFFF;
            pendingLayer.padW = (data >> 16) & 0xFFFF;
            break;
            
        case NPUReg::INPUT_ADDR:
            pendingLayer.inputAddr = data;
            break;
            
        case NPUReg::OUTPUT_ADDR:
            pendingLayer.outputAddr = data;
            break;
            
        case NPUReg::WEIGHTS_ADDR:
            pendingLayer.weightsAddr = data;
            break;
            
        case NPUReg::BIAS_ADDR:
            pendingLayer.biasAddr = data;
            break;
            
        case NPUReg::PRECISION:
            pendingLayer.precision = static_cast<NPUPrecision>(data);
            break;
            
        case NPUReg::SPARSITY:
            pendingLayer.sparsity = static_cast<float>(data) / 100.0f;
            break;
            
        case NPUReg::DVFS_SCALE:
            setFrequencyScale(static_cast<float>(data) / 100.0f);
            break;
            
        default:
            DPRINTF(NPU, "NPU[%d] unknown MMIO write at offset 0x%lx\n",
                    instanceId, offset);
            break;
    }
    
    return clockPeriod();
}

// DMA Port implementation
bool NeuralProcessingUnit::NPUPort::recvTimingResp(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void NeuralProcessingUnit::NPUPort::recvReqRetry()
{
    // Handle retry
}

} // namespace spacecraft
} // namespace gem5

