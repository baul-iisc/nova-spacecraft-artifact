/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * Enhanced TinyML Accelerator with 32 Processing Elements
 * Implementation file
 */

#include "custom_accel/tiny_ml_32pe.hh"

#include "base/trace.hh"
#include "debug/TinyML32PE.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

// CPUSidePort implementation
TinyML32PE::CPUSidePort::CPUSidePort(const std::string& name, TinyML32PE *owner)
    : ResponsePort(name, owner), owner(owner), needRetry(false), 
      blockedPacket(nullptr)
{
}

void
TinyML32PE::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
TinyML32PE::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
TinyML32PE::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(TinyML32PE, "Sending retry for CPU port\n");
        sendRetryReq();
    }
}

Tick
TinyML32PE::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    return owner->clockPeriod();
}

void
TinyML32PE::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
TinyML32PE::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
TinyML32PE::CPUSidePort::recvRespRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

// MemSidePort implementation
TinyML32PE::MemSidePort::MemSidePort(const std::string& name, TinyML32PE *owner)
    : RequestPort(name, owner), owner(owner), blockedPacket(nullptr)
{
}

void
TinyML32PE::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
TinyML32PE::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
TinyML32PE::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
TinyML32PE::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// Statistics
TinyML32PE::TinyML32PEStats::TinyML32PEStats(TinyML32PE *parent)
    : statistics::Group(parent),
      ADD_STAT(convOps, statistics::units::Count::get(),
               "Number of convolution operations"),
      ADD_STAT(fcOps, statistics::units::Count::get(),
               "Number of fully-connected operations"),
      ADD_STAT(poolOps, statistics::units::Count::get(),
               "Number of pooling operations"),
      ADD_STAT(totalMacOps, statistics::units::Count::get(),
               "Total MAC operations"),
      ADD_STAT(totalCycles, statistics::units::Cycle::get(),
               "Total cycles for all operations"),
      ADD_STAT(inputBytesLoaded, statistics::units::Byte::get(),
               "Total input bytes loaded from memory"),
      ADD_STAT(weightBytesLoaded, statistics::units::Byte::get(),
               "Total weight bytes loaded from memory"),
      ADD_STAT(outputBytesStored, statistics::units::Byte::get(),
               "Total output bytes stored to memory"),
      ADD_STAT(throughput, statistics::units::Rate<
               statistics::units::Count, statistics::units::Cycle>::get(),
               "MAC operations per cycle"),
      ADD_STAT(utilizationRate, statistics::units::Ratio::get(),
               "PE array utilization rate")
{
    throughput = totalMacOps / totalCycles;
    utilizationRate = throughput / NUM_PES;
}

// Main constructor
TinyML32PE::TinyML32PE(const Params &params)
    : ClockedObject(params),
      regCtrl(0), regStatus(STATUS_IDLE),
      regInputBase(0), regWeightBase(0), regOutputBase(0), regBiasBase(0),
      regInputH(0), regInputW(0), regInputC(0), regOutputC(0),
      regKernelH(0), regKernelW(0),
      regStride(0x0101), regPadding(0),
      regDataType(0), regOpType(0), regActivation(0),
      regQuantScale(128), regQuantZero(0),
      regPeConfig(0), regDmaStatus(0),
      regPerfCycles(0), regPerfMacs(0), regPerfBytes(0),
      regInterrupt(0),
      globalBufferSize(params.global_buffer_size),
      weightBufferSize(params.weight_buffer_size),
      accumBufferSize(params.accum_buffer_size),
      peLatency(params.pe_latency),
      dmaLatency(params.dma_latency),
      dataWidth(params.data_width),
      dmaActive(false), dmaBytesRemaining(0), dmaCurrentAddr(0),
      cpuPort(params.name + ".cpu_side", this),
      memPort(params.name + ".mem_side", this),
      addrRange(params.addr_range),
      stats(this),
      computeEvent([this]{ completeOperation(); }, name()),
      dmaEvent([this]{ completeDma(); }, name())
{
    // Initialize PE array
    resetPEArray();
    
    // Allocate on-chip buffers
    globalBuffer.resize(globalBufferSize);
    weightBuffer.resize(weightBufferSize);
    accumBuffer.resize(accumBufferSize / sizeof(int32_t));
    
    DPRINTF(TinyML32PE, "TinyML32PE created with %d PEs (%dx%d)\n",
            NUM_PES, NUM_PE_ROWS, NUM_PE_COLS);
    DPRINTF(TinyML32PE, "  Global buffer: %u bytes\n", globalBufferSize);
    DPRINTF(TinyML32PE, "  Weight buffer: %u bytes\n", weightBufferSize);
    DPRINTF(TinyML32PE, "  Accum buffer: %u bytes\n", accumBufferSize);
}

void
TinyML32PE::startup()
{
    DPRINTF(TinyML32PE, "TinyML32PE starting up\n");
}

Port &
TinyML32PE::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_side") {
        return cpuPort;
    } else if (if_name == "mem_side") {
        return memPort;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

AddrRangeList
TinyML32PE::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(addrRange);
    return ranges;
}

void
TinyML32PE::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

bool
TinyML32PE::handleRequest(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
        DPRINTF(TinyML32PE, "Read from 0x%x: 0x%x\n", offset, value);
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
        DPRINTF(TinyML32PE, "Write to 0x%x: 0x%x\n", offset, value);
    }
    
    pkt->makeResponse();
    cpuPort.sendPacket(pkt);
    return true;
}

bool
TinyML32PE::handleResponse(PacketPtr pkt)
{
    // Handle DMA response
    delete pkt;
    return true;
}

void
TinyML32PE::handleFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        pkt->setLE<uint32_t>(readReg(offset));
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        writeReg(offset, pkt->getLE<uint32_t>());
    }
}

uint32_t
TinyML32PE::readReg(Addr offset)
{
    switch (offset) {
        case REG_CTRL:        return regCtrl;
        case REG_STATUS:      return regStatus;
        case REG_INPUT_BASE:  return regInputBase;
        case REG_WEIGHT_BASE: return regWeightBase;
        case REG_OUTPUT_BASE: return regOutputBase;
        case REG_BIAS_BASE:   return regBiasBase;
        case REG_INPUT_H:     return regInputH;
        case REG_INPUT_W:     return regInputW;
        case REG_INPUT_C:     return regInputC;
        case REG_OUTPUT_C:    return regOutputC;
        case REG_KERNEL_H:    return regKernelH;
        case REG_KERNEL_W:    return regKernelW;
        case REG_STRIDE:      return regStride;
        case REG_PADDING:     return regPadding;
        case REG_DATA_TYPE:   return regDataType;
        case REG_OP_TYPE:     return regOpType;
        case REG_ACTIVATION:  return regActivation;
        case REG_QUANT_SCALE: return regQuantScale;
        case REG_QUANT_ZERO:  return regQuantZero;
        case REG_PE_CONFIG:   return regPeConfig;
        case REG_DMA_STATUS:  return regDmaStatus;
        case REG_PERF_CYCLES: return (uint32_t)regPerfCycles;
        case REG_PERF_MACS:   return (uint32_t)regPerfMacs;
        case REG_PERF_BYTES:  return (uint32_t)regPerfBytes;
        case REG_INTERRUPT:   return regInterrupt;
        case REG_VERSION:     return VERSION;
        default:
            DPRINTF(TinyML32PE, "Unknown register read at 0x%x\n", offset);
            return 0;
    }
}

void
TinyML32PE::writeReg(Addr offset, uint32_t value)
{
    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                regStatus = STATUS_IDLE;
                regPerfCycles = 0;
                regPerfMacs = 0;
                regPerfBytes = 0;
                resetPEArray();
            } else if (value & CTRL_START) {
                startOperation();
            }
            break;
        case REG_INPUT_BASE:  regInputBase = value; break;
        case REG_WEIGHT_BASE: regWeightBase = value; break;
        case REG_OUTPUT_BASE: regOutputBase = value; break;
        case REG_BIAS_BASE:   regBiasBase = value; break;
        case REG_INPUT_H:     regInputH = value; break;
        case REG_INPUT_W:     regInputW = value; break;
        case REG_INPUT_C:     regInputC = value; break;
        case REG_OUTPUT_C:    regOutputC = value; break;
        case REG_KERNEL_H:    regKernelH = value; break;
        case REG_KERNEL_W:    regKernelW = value; break;
        case REG_STRIDE:      regStride = value; break;
        case REG_PADDING:     regPadding = value; break;
        case REG_DATA_TYPE:   regDataType = value; break;
        case REG_OP_TYPE:     regOpType = value; break;
        case REG_ACTIVATION:  regActivation = value; break;
        case REG_QUANT_SCALE: regQuantScale = value; break;
        case REG_QUANT_ZERO:  regQuantZero = value; break;
        case REG_PE_CONFIG:   regPeConfig = value; break;
        case REG_INTERRUPT:   regInterrupt = value; break;
        default:
            DPRINTF(TinyML32PE, "Unknown register write at 0x%x: 0x%x\n", 
                    offset, value);
    }
}

void
TinyML32PE::resetPEArray()
{
    for (unsigned i = 0; i < NUM_PES; i++) {
        peArray[i].accumulator = 0;
        peArray[i].weight = 0;
        peArray[i].busy = false;
        peArray[i].macCount = 0;
    }
}

void
TinyML32PE::startOperation()
{
    if (regStatus & STATUS_BUSY) {
        DPRINTF(TinyML32PE, "Cannot start: already busy\n");
        return;
    }
    
    regStatus = STATUS_BUSY;
    DPRINTF(TinyML32PE, "Starting operation type %d\n", regOpType);
    
    OpType op = static_cast<OpType>(regOpType);
    
    switch (op) {
        case OpType::CONV2D:
            executeConv2D();
            break;
        case OpType::DEPTHWISE_CONV:
            executeDepthwiseConv();
            break;
        case OpType::FULLY_CONNECTED:
            executeFullyConnected();
            break;
        case OpType::MAX_POOL:
            executePooling(true);
            break;
        case OpType::AVG_POOL:
            executePooling(false);
            break;
        case OpType::GLOBAL_AVG_POOL:
            executeGlobalAvgPool();
            break;
        case OpType::ELEMENTWISE_ADD:
            executeElementwise(true);
            break;
        case OpType::ELEMENTWISE_MUL:
            executeElementwise(false);
            break;
        case OpType::SOFTMAX:
            executeSoftmax();
            break;
        case OpType::BATCH_NORM:
            executeBatchNorm();
            break;
        default:
            DPRINTF(TinyML32PE, "Unknown operation type %d\n", regOpType);
            regStatus = STATUS_ERROR;
            return;
    }
}

void
TinyML32PE::executeConv2D()
{
    DPRINTF(TinyML32PE, "Executing Conv2D: %dx%dx%d -> %d channels, kernel %dx%d\n",
            regInputH, regInputW, regInputC, regOutputC, regKernelH, regKernelW);
    
    // Calculate output dimensions
    uint32_t strideH = (regStride >> 8) & 0xFF;
    uint32_t strideW = regStride & 0xFF;
    uint32_t padTop = (regPadding >> 24) & 0xFF;
    uint32_t padLeft = (regPadding >> 8) & 0xFF;
    
    uint32_t outputH = (regInputH + 2 * padTop - regKernelH) / strideH + 1;
    uint32_t outputW = (regInputW + 2 * padLeft - regKernelW) / strideW + 1;
    
    // Calculate computation requirements
    uint64_t totalMACs = (uint64_t)outputH * outputW * regOutputC * 
                         regInputC * regKernelH * regKernelW;
    
    // Calculate cycles (considering PE array parallelism)
    uint64_t tilesNeeded = (regOutputC + NUM_PE_COLS - 1) / NUM_PE_COLS;
    uint64_t macsPerTile = (uint64_t)outputH * outputW * 
                           regInputC * regKernelH * regKernelW;
    uint64_t cyclesPerTile = (macsPerTile + NUM_PE_ROWS - 1) / NUM_PE_ROWS * peLatency;
    uint64_t totalCycles = tilesNeeded * cyclesPerTile;
    
    regPerfMacs += totalMACs;
    regPerfCycles += totalCycles;
    stats.convOps++;
    stats.totalMacOps += totalMACs;
    stats.totalCycles += totalCycles;
    
    // Schedule completion
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::executeDepthwiseConv()
{
    DPRINTF(TinyML32PE, "Executing Depthwise Conv2D: %dx%dx%d, kernel %dx%d\n",
            regInputH, regInputW, regInputC, regKernelH, regKernelW);
    
    uint32_t strideH = (regStride >> 8) & 0xFF;
    uint32_t strideW = regStride & 0xFF;
    uint32_t padTop = (regPadding >> 24) & 0xFF;
    uint32_t padLeft = (regPadding >> 8) & 0xFF;
    
    uint32_t outputH = (regInputH + 2 * padTop - regKernelH) / strideH + 1;
    uint32_t outputW = (regInputW + 2 * padLeft - regKernelW) / strideW + 1;
    
    // Depthwise: each channel processed independently
    uint64_t totalMACs = (uint64_t)outputH * outputW * regInputC * 
                         regKernelH * regKernelW;
    
    uint64_t totalCycles = (totalMACs + NUM_PES - 1) / NUM_PES * peLatency;
    
    regPerfMacs += totalMACs;
    regPerfCycles += totalCycles;
    stats.convOps++;
    stats.totalMacOps += totalMACs;
    stats.totalCycles += totalCycles;
    
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::executeFullyConnected()
{
    DPRINTF(TinyML32PE, "Executing Fully Connected: %d -> %d\n",
            regInputC, regOutputC);
    
    uint64_t totalMACs = (uint64_t)regInputC * regOutputC;
    
    // FC is essentially matrix-vector multiplication
    // Can utilize all PEs for output channel parallelism
    uint64_t tilesNeeded = (regOutputC + NUM_PES - 1) / NUM_PES;
    uint64_t cyclesPerTile = regInputC * peLatency;
    uint64_t totalCycles = tilesNeeded * cyclesPerTile;
    
    regPerfMacs += totalMACs;
    regPerfCycles += totalCycles;
    stats.fcOps++;
    stats.totalMacOps += totalMACs;
    stats.totalCycles += totalCycles;
    
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::executePooling(bool isMax)
{
    DPRINTF(TinyML32PE, "Executing %s Pooling: %dx%dx%d, kernel %dx%d\n",
            isMax ? "Max" : "Avg",
            regInputH, regInputW, regInputC, regKernelH, regKernelW);
    
    uint32_t strideH = (regStride >> 8) & 0xFF;
    uint32_t strideW = regStride & 0xFF;
    
    uint32_t outputH = (regInputH - regKernelH) / strideH + 1;
    uint32_t outputW = (regInputW - regKernelW) / strideW + 1;
    
    // Pooling requires comparisons/additions per window
    uint64_t opsPerWindow = regKernelH * regKernelW;
    uint64_t totalOps = (uint64_t)outputH * outputW * regInputC * opsPerWindow;
    
    // Pooling is memory-bound, estimate cycles
    uint64_t totalCycles = (totalOps + NUM_PES - 1) / NUM_PES * peLatency;
    
    regPerfCycles += totalCycles;
    stats.poolOps++;
    stats.totalCycles += totalCycles;
    
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::executeGlobalAvgPool()
{
    DPRINTF(TinyML32PE, "Executing Global Average Pooling: %dx%dx%d\n",
            regInputH, regInputW, regInputC);
    
    uint64_t opsPerChannel = regInputH * regInputW;
    uint64_t totalOps = opsPerChannel * regInputC;
    
    uint64_t totalCycles = (totalOps + NUM_PES - 1) / NUM_PES * peLatency;
    
    regPerfCycles += totalCycles;
    stats.poolOps++;
    stats.totalCycles += totalCycles;
    
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::executeElementwise(bool isAdd)
{
    DPRINTF(TinyML32PE, "Executing Elementwise %s\n", isAdd ? "Add" : "Mul");
    
    uint64_t totalOps = (uint64_t)regInputH * regInputW * regInputC;
    uint64_t totalCycles = (totalOps + NUM_PES - 1) / NUM_PES * peLatency;
    
    regPerfCycles += totalCycles;
    stats.totalCycles += totalCycles;
    
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::executeSoftmax()
{
    DPRINTF(TinyML32PE, "Executing Softmax over %d elements\n", regInputC);
    
    // Softmax: exp and division approximated with lookup tables
    // Two passes: find max, compute normalized exp
    uint64_t totalCycles = regInputC * 3 * peLatency;  // Approximation
    
    regPerfCycles += totalCycles;
    stats.totalCycles += totalCycles;
    
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::executeBatchNorm()
{
    DPRINTF(TinyML32PE, "Executing Batch Normalization: %dx%dx%d\n",
            regInputH, regInputW, regInputC);
    
    // BN: scale and shift per channel
    uint64_t totalOps = (uint64_t)regInputH * regInputW * regInputC * 2;
    uint64_t totalCycles = (totalOps + NUM_PES - 1) / NUM_PES * peLatency;
    
    regPerfCycles += totalCycles;
    stats.totalCycles += totalCycles;
    
    schedule(computeEvent, curTick() + cyclesToTicks(Cycles(totalCycles)));
}

void
TinyML32PE::completeOperation()
{
    DPRINTF(TinyML32PE, "Operation complete\n");
    
    regStatus = STATUS_DONE;
    regCtrl &= ~CTRL_START;
    
    // Generate interrupt if enabled
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x01;
    }
}

// Activation functions
int8_t
TinyML32PE::applyActivation(int32_t accumulator)
{
    ActivationType act = static_cast<ActivationType>(regActivation);
    
    switch (act) {
        case ActivationType::NONE:
            return quantize(accumulator);
        case ActivationType::RELU:
            return relu(accumulator);
        case ActivationType::RELU6:
            return relu6(accumulator);
        case ActivationType::LEAKY_RELU:
            return leakyRelu(accumulator);
        case ActivationType::SIGMOID:
            return sigmoidApprox(accumulator);
        case ActivationType::TANH:
            return tanhApprox(accumulator);
        case ActivationType::SWISH:
            return swishApprox(accumulator);
        default:
            return quantize(accumulator);
    }
}

int8_t
TinyML32PE::relu(int32_t x)
{
    int32_t result = x > 0 ? x : 0;
    return quantize(result);
}

int8_t
TinyML32PE::relu6(int32_t x)
{
    int32_t scaled6 = 6 * regQuantScale;
    int32_t result = x > 0 ? (x < scaled6 ? x : scaled6) : 0;
    return quantize(result);
}

int8_t
TinyML32PE::leakyRelu(int32_t x)
{
    int32_t result = x > 0 ? x : (x >> 4);  // alpha = 0.0625
    return quantize(result);
}

int8_t
TinyML32PE::sigmoidApprox(int32_t x)
{
    // Piecewise linear approximation of sigmoid
    int32_t absX = x > 0 ? x : -x;
    int32_t scaled = regQuantScale;
    
    if (absX > 4 * scaled) {
        return x > 0 ? 127 : -128;
    } else if (absX > 2 * scaled) {
        return quantize((x >> 2) + (x > 0 ? scaled/2 : -scaled/2));
    } else {
        return quantize((x >> 2) + scaled/2);
    }
}

int8_t
TinyML32PE::tanhApprox(int32_t x)
{
    // Approximation: tanh(x) ≈ 2*sigmoid(2x) - 1
    int32_t sigmoidVal = sigmoidApprox(x * 2);
    return quantize(2 * sigmoidVal - regQuantScale);
}

int8_t
TinyML32PE::swishApprox(int32_t x)
{
    // Swish: x * sigmoid(x)
    int32_t sigVal = sigmoidApprox(x);
    return quantize((x * sigVal) / regQuantScale);
}

int8_t
TinyML32PE::quantize(int32_t accumulator)
{
    int32_t result = (accumulator / (int32_t)regQuantScale) + regQuantZero;
    if (result > 127) return 127;
    if (result < -128) return -128;
    return static_cast<int8_t>(result);
}

int32_t
TinyML32PE::dequantize(int8_t value)
{
    return (static_cast<int32_t>(value) - regQuantZero) * regQuantScale;
}

void
TinyML32PE::startDmaRead(Addr addr, unsigned size)
{
    dmaActive = true;
    dmaBytesRemaining = size;
    dmaCurrentAddr = addr;
    regDmaStatus = STATUS_DMA_BUSY;
    
    schedule(dmaEvent, curTick() + cyclesToTicks(Cycles(dmaLatency)));
}

void
TinyML32PE::startDmaWrite(Addr addr, unsigned size)
{
    dmaActive = true;
    dmaBytesRemaining = size;
    dmaCurrentAddr = addr;
    regDmaStatus = STATUS_DMA_BUSY;
    
    schedule(dmaEvent, curTick() + cyclesToTicks(Cycles(dmaLatency)));
}

void
TinyML32PE::completeDma()
{
    regPerfBytes += dmaBytesRemaining;
    stats.inputBytesLoaded += dmaBytesRemaining;
    
    dmaActive = false;
    dmaBytesRemaining = 0;
    regDmaStatus = 0;
    
    DPRINTF(TinyML32PE, "DMA transfer complete\n");
}

} // namespace gem5



