/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * Tiny ML Accelerator Implementation
 */

#include "custom_accel/tiny_ml_accel.hh"

#include <algorithm>
#include <cmath>

#include "base/trace.hh"
#include "debug/TinyMLAccel.hh"
#include "mem/packet_access.hh"

namespace gem5
{

TinyMLAccel::TinyMLAccel(const Params &params) :
    ClockedObject(params),
    regCtrl(0),
    regStatus(0),
    regInputAddr(0),
    regWeightAddr(0),
    regOutputAddr(0),
    regInputDim(0),
    regWeightDim(0),
    regConfig(0),
    regMacConfig(0),
    regPerfCnt(0),
    regBiasAddr(0),
    regScale(256),  // Default scale factor (1.0 in Q8 format)
    macArrayRows(params.mac_array_rows),
    macArrayCols(params.mac_array_cols),
    sramSize(params.sram_size),
    macLatency(params.mac_latency),
    dmaLatency(params.dma_latency),
    busy(false),
    currentOp(OpType::MATMUL),
    activation(ActivationType::NONE),
    totalMacOps(0),
    cpuPort(params.name + ".cpu_side", this),
    memPort(params.name + ".mem_side", this),
    addrRange(params.addr_range),
    stats(this),
    computeEvent([this]{ completeOperation(); }, name())
{
    DPRINTF(TinyMLAccel, "TinyMLAccel created: %dx%d MAC array, %d KB SRAM\n",
            macArrayRows, macArrayCols, sramSize / 1024);
}

Port &
TinyMLAccel::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_side") {
        return cpuPort;
    } else if (if_name == "mem_side") {
        return memPort;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
TinyMLAccel::startup()
{
    ClockedObject::startup();
    // Notify the bus of our address ranges
    cpuPort.sendRangeChange();
}

// CPUSidePort implementation
TinyMLAccel::CPUSidePort::CPUSidePort(const std::string& name, 
                                      TinyMLAccel *owner) :
    ResponsePort(name), owner(owner), needRetry(false), blockedPacket(nullptr)
{
}

void
TinyMLAccel::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
TinyMLAccel::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
TinyMLAccel::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        sendRetryReq();
    }
}

Tick
TinyMLAccel::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    return owner->clockPeriod();
}

void
TinyMLAccel::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
TinyMLAccel::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
TinyMLAccel::CPUSidePort::recvRespRetry()
{
    assert(blockedPacket != nullptr);
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

// MemSidePort implementation
TinyMLAccel::MemSidePort::MemSidePort(const std::string& name,
                                      TinyMLAccel *owner) :
    RequestPort(name), owner(owner), blockedPacket(nullptr)
{
}

void
TinyMLAccel::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
TinyMLAccel::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
TinyMLAccel::MemSidePort::recvReqRetry()
{
    assert(blockedPacket != nullptr);
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
TinyMLAccel::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// Request handling
bool
TinyMLAccel::handleRequest(PacketPtr pkt)
{
    DPRINTF(TinyMLAccel, "handleRequest for addr %#x\n", pkt->getAddr());

    Addr offset = pkt->getAddr() - addrRange.start();

    if (pkt->isWrite()) {
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
        pkt->makeResponse();
    } else if (pkt->isRead()) {
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
        pkt->makeResponse();
    }

    cpuPort.sendPacket(pkt);
    return true;
}

bool
TinyMLAccel::handleResponse(PacketPtr pkt)
{
    DPRINTF(TinyMLAccel, "handleResponse for addr %#x\n", pkt->getAddr());
    delete pkt;
    return true;
}

void
TinyMLAccel::handleFunctional(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - addrRange.start();
    if (pkt->isWrite()) {
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
    } else if (pkt->isRead()) {
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
    }
}

uint32_t
TinyMLAccel::readReg(Addr offset)
{
    switch (offset) {
        case REG_CTRL:        return regCtrl;
        case REG_STATUS:      return regStatus;
        case REG_INPUT_ADDR:  return regInputAddr;
        case REG_WEIGHT_ADDR: return regWeightAddr;
        case REG_OUTPUT_ADDR: return regOutputAddr;
        case REG_INPUT_DIM:   return regInputDim;
        case REG_WEIGHT_DIM:  return regWeightDim;
        case REG_CONFIG:      return regConfig;
        case REG_MAC_CONFIG:  return regMacConfig;
        case REG_PERF_CNT:    return regPerfCnt;
        case REG_BIAS_ADDR:   return regBiasAddr;
        case REG_SCALE:       return regScale;
        default:
            DPRINTF(TinyMLAccel, "Unknown register read at offset %#x\n", 
                    offset);
            return 0;
    }
}

void
TinyMLAccel::writeReg(Addr offset, uint32_t value)
{
    DPRINTF(TinyMLAccel, "writeReg offset=%#x value=%#x\n", offset, value);

    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                regStatus = 0;
                busy = false;
                totalMacOps = 0;
                regPerfCnt = 0;
            } else if ((value & CTRL_START) && !busy) {
                currentOp = static_cast<OpType>((value & CTRL_OP_MASK) >> 
                                                 CTRL_OP_SHIFT);
                startOperation();
            }
            break;
        case REG_STATUS:
            regStatus &= ~(value & (STATUS_DONE | STATUS_ERROR));
            break;
        case REG_INPUT_ADDR:
            regInputAddr = value;
            break;
        case REG_WEIGHT_ADDR:
            regWeightAddr = value;
            break;
        case REG_OUTPUT_ADDR:
            regOutputAddr = value;
            break;
        case REG_INPUT_DIM:
            regInputDim = value;
            break;
        case REG_WEIGHT_DIM:
            regWeightDim = value;
            break;
        case REG_CONFIG:
            regConfig = value;
            activation = static_cast<ActivationType>(value & 0x0F);
            break;
        case REG_MAC_CONFIG:
            regMacConfig = value;
            break;
        case REG_BIAS_ADDR:
            regBiasAddr = value;
            break;
        case REG_SCALE:
            regScale = value;
            break;
        default:
            DPRINTF(TinyMLAccel, "Unknown register write at offset %#x\n", 
                    offset);
    }
}

void
TinyMLAccel::startOperation()
{
    DPRINTF(TinyMLAccel, "Starting ML operation: type=%d\n", 
            static_cast<int>(currentOp));

    busy = true;
    regStatus = STATUS_BUSY;

    // Decode dimensions
    unsigned inputH = (regInputDim >> 16) & 0xFF;
    unsigned inputW = (regInputDim >> 8) & 0xFF;
    unsigned inputC = regInputDim & 0xFF;

    unsigned outputCh = (regWeightDim >> 16) & 0xFFFF;
    unsigned kernelSize = regWeightDim & 0xFF;

    DPRINTF(TinyMLAccel, "Input: %dx%dx%d, Output channels: %d, Kernel: %d\n",
            inputH, inputW, inputC, outputCh, kernelSize);

    uint64_t macOps = 0;
    Tick computeLatency = 0;

    switch (currentOp) {
        case OpType::MATMUL:
            // Matrix multiply: input [M x K] * weight [K x N] = output [M x N]
            macOps = (uint64_t)inputH * inputW * outputCh;
            executeMatMul();
            stats.matmulOps++;
            break;

        case OpType::CONV2D:
            // 2D Convolution
            macOps = (uint64_t)inputH * inputW * inputC * 
                     outputCh * kernelSize * kernelSize;
            executeConv2D();
            stats.conv2dOps++;
            break;

        case OpType::DEPTHWISE_CONV:
            // Depthwise convolution (less compute than regular conv)
            macOps = (uint64_t)inputH * inputW * inputC * 
                     kernelSize * kernelSize;
            break;

        case OpType::POOLING:
            // Pooling has minimal compute
            macOps = (uint64_t)inputH * inputW * inputC;
            executePooling();
            break;

        case OpType::ACTIVATION:
            // Activation only
            macOps = (uint64_t)inputH * inputW * inputC;
            applyActivation();
            break;

        default:
            regStatus = STATUS_ERROR;
            busy = false;
            return;
    }

    totalMacOps += macOps;
    stats.totalMacOperations += macOps;

    // Calculate latency based on MAC operations and array size
    unsigned totalMacs = macArrayRows * macArrayCols;
    Tick macCycles = (macOps + totalMacs - 1) / totalMacs;
    computeLatency = macCycles * macLatency * clockPeriod();

    // Add DMA latency for data transfer
    unsigned inputBytes = inputH * inputW * inputC;
    unsigned weightBytes = outputCh * inputC * kernelSize * kernelSize;
    Tick dmaTime = ((inputBytes + weightBytes) / 64) * dmaLatency * clockPeriod();

    stats.inputBytesLoaded += inputBytes;
    stats.weightBytesLoaded += weightBytes;

    Tick completionTime = curTick() + computeLatency + dmaTime;
    schedule(computeEvent, completionTime);

    DPRINTF(TinyMLAccel, "Scheduled completion in %llu cycles (%llu MAC ops)\n",
            (computeLatency + dmaTime) / clockPeriod(), macOps);
}

void
TinyMLAccel::executeMatMul()
{
    // Simulated matrix multiplication
    // In real hardware, this uses the systolic array
    unsigned M = (regInputDim >> 16) & 0xFF;
    unsigned K = (regInputDim >> 8) & 0xFF;
    unsigned N = (regWeightDim >> 16) & 0xFFFF;

    outputBuffer.resize(M * N);

    // Simulate computation (actual values don't matter for timing)
    for (unsigned i = 0; i < M; i++) {
        for (unsigned j = 0; j < N; j++) {
            int32_t acc = 0;
            for (unsigned k = 0; k < K; k++) {
                // Simulated MAC: acc += input[i,k] * weight[k,j]
                acc += (int8_t)(i * k + 1) * (int8_t)(k * j + 1);
            }
            // Apply activation and quantize
            outputBuffer[i * N + j] = quantize(acc, regScale, 0);
            if (activation != ActivationType::NONE) {
                outputBuffer[i * N + j] = relu(outputBuffer[i * N + j]);
            }
        }
    }

    stats.outputBytesStored += M * N;
}

void
TinyMLAccel::executeConv2D()
{
    // Simulated 2D convolution
    unsigned H = (regInputDim >> 16) & 0xFF;
    unsigned W = (regInputDim >> 8) & 0xFF;
    unsigned C = regInputDim & 0xFF;
    unsigned outCh = (regWeightDim >> 16) & 0xFFFF;
    unsigned K = regWeightDim & 0xFF;

    // Assuming stride=1, no padding for simplicity
    unsigned outH = H - K + 1;
    unsigned outW = W - K + 1;

    outputBuffer.resize(outH * outW * outCh);

    // Simulate convolution
    for (unsigned oc = 0; oc < outCh; oc++) {
        for (unsigned oh = 0; oh < outH; oh++) {
            for (unsigned ow = 0; ow < outW; ow++) {
                int32_t acc = 0;
                for (unsigned c = 0; c < C; c++) {
                    for (unsigned kh = 0; kh < K; kh++) {
                        for (unsigned kw = 0; kw < K; kw++) {
                            // Simulated MAC
                            acc += 1;  // Placeholder
                        }
                    }
                }
                unsigned idx = oc * outH * outW + oh * outW + ow;
                outputBuffer[idx] = quantize(acc, regScale, 0);
                if (activation == ActivationType::RELU) {
                    outputBuffer[idx] = relu(outputBuffer[idx]);
                }
            }
        }
    }

    stats.outputBytesStored += outH * outW * outCh;
}

void
TinyMLAccel::executePooling()
{
    // Max pooling simulation
    unsigned H = (regInputDim >> 16) & 0xFF;
    unsigned W = (regInputDim >> 8) & 0xFF;
    unsigned C = regInputDim & 0xFF;
    unsigned poolSize = regWeightDim & 0xFF;
    if (poolSize == 0) poolSize = 2;

    unsigned outH = H / poolSize;
    unsigned outW = W / poolSize;

    outputBuffer.resize(outH * outW * C);
    stats.outputBytesStored += outH * outW * C;
}

void
TinyMLAccel::applyActivation()
{
    unsigned size = ((regInputDim >> 16) & 0xFF) * 
                    ((regInputDim >> 8) & 0xFF) * 
                    (regInputDim & 0xFF);

    outputBuffer.resize(size);
    for (unsigned i = 0; i < size; i++) {
        outputBuffer[i] = relu(i);  // Placeholder
    }
    stats.outputBytesStored += size;
}

void
TinyMLAccel::completeOperation()
{
    DPRINTF(TinyMLAccel, "ML operation complete. Total MACs: %llu\n", 
            totalMacOps);

    busy = false;
    regStatus = STATUS_DONE;
    regPerfCnt = totalMacOps & 0xFFFFFFFF;
    regCtrl = 0;
}

// Activation functions (INT8)
int8_t
TinyMLAccel::relu(int32_t x)
{
    if (x < 0) return 0;
    if (x > 127) return 127;
    return static_cast<int8_t>(x);
}

int8_t
TinyMLAccel::relu6(int32_t x)
{
    if (x < 0) return 0;
    if (x > 6) return 6;  // Scaled for INT8
    return static_cast<int8_t>(x);
}

int8_t
TinyMLAccel::sigmoid_approx(int32_t x)
{
    // Piecewise linear approximation of sigmoid
    // Output range: 0-127 (representing 0-1)
    if (x < -128) return 0;
    if (x > 127) return 127;
    // Linear approximation in the middle region
    return static_cast<int8_t>((x + 128) / 2);
}

int8_t
TinyMLAccel::tanh_approx(int32_t x)
{
    // Piecewise linear approximation of tanh
    // Output range: -128 to 127 (representing -1 to 1)
    if (x < -128) return -128;
    if (x > 127) return 127;
    return static_cast<int8_t>(x);
}

int8_t
TinyMLAccel::quantize(int32_t accumulator, int32_t scale, int32_t zero_point)
{
    // Quantize 32-bit accumulator to INT8
    int32_t scaled = (accumulator * scale) >> 8;
    scaled += zero_point;
    
    if (scaled < -128) return -128;
    if (scaled > 127) return 127;
    return static_cast<int8_t>(scaled);
}

AddrRangeList
TinyMLAccel::getAddrRanges() const
{
    return AddrRangeList{addrRange};
}

void
TinyMLAccel::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

// Statistics
TinyMLAccel::MLStats::MLStats(TinyMLAccel *parent) :
    statistics::Group(parent),
    ADD_STAT(matmulOps, statistics::units::Count::get(),
             "Number of matrix multiply operations"),
    ADD_STAT(conv2dOps, statistics::units::Count::get(),
             "Number of 2D convolution operations"),
    ADD_STAT(totalMacOperations, statistics::units::Count::get(),
             "Total MAC operations performed"),
    ADD_STAT(totalCycles, statistics::units::Cycle::get(),
             "Total cycles for ML operations"),
    ADD_STAT(inputBytesLoaded, statistics::units::Byte::get(),
             "Total input bytes loaded from memory"),
    ADD_STAT(weightBytesLoaded, statistics::units::Byte::get(),
             "Total weight bytes loaded from memory"),
    ADD_STAT(outputBytesStored, statistics::units::Byte::get(),
             "Total output bytes stored to memory")
{
}

} // namespace gem5

