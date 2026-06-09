/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * TinyTPU Implementation - TPU-like Matrix Multiply Accelerator
 */

#include "custom_accel/tiny_tpu.hh"

#include <cmath>
#include <algorithm>

#include "base/trace.hh"
#include "debug/TinyTPU.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

TinyTPU::TinyTPU(const Params &params) :
    ClockedObject(params),
    unifiedBufferSize(params.unified_buffer_size),
    weightFifoSize(params.weight_fifo_size),
    accumBufferSize(params.accum_buffer_size),
    mxuLatency(params.mxu_latency),
    dmaLatency(params.dma_latency),
    regCtrl(0),
    regStatus(STATUS_IDLE),
    regInstrAddr(0),
    regWeightAddr(0),
    regInputAddr(0),
    regOutputAddr(0),
    regDimM(0),
    regDimN(0),
    regDimK(0),
    regActivation(0),
    regQuantScale(1),
    regQuantZero(0),
    regPerfCycles(0),
    regPerfMacs(0),
    regPerfTiles(0),
    regMxuConfig(0),
    regBiasAddr(0),
    regDataType(0),
    busy(false),
    aborted(false),
    activationType(ActivationType::NONE),
    dataType(DataType::INT8),
    numTilesM(0),
    numTilesN(0),
    numTilesK(0),
    currentTileM(0),
    currentTileN(0),
    currentTileK(0),
    cpuPort(params.name + ".cpu_side", this),
    memPort(params.name + ".mem_side", this),
    addrRange(params.addr_range),
    mxuComputeEvent([this]{ executeTileMultiply(); }, name()),
    dmaCompleteEvent([this]{ handleDmaComplete(); }, name()),
    operationCompleteEvent([this]{ completeOperation(); }, name()),
    stats(this)
{
    // Allocate buffers
    unifiedBuffer.resize(unifiedBufferSize, 0);
    weightFifo.resize(weightFifoSize, 0);
    accumBuffer.resize(MXU_SIZE * MXU_SIZE, 0);
    outputBuffer.resize(MXU_SIZE * MXU_SIZE, 0);
    
    // Initialize MXU
    initializeMXU();
    
    DPRINTF(TinyTPU, "TinyTPU created:\n");
    DPRINTF(TinyTPU, "  MXU Size: %dx%d systolic array\n", MXU_SIZE, MXU_SIZE);
    DPRINTF(TinyTPU, "  Unified buffer: %d bytes\n", unifiedBufferSize);
    DPRINTF(TinyTPU, "  Weight FIFO: %d bytes\n", weightFifoSize);
    DPRINTF(TinyTPU, "  Accum buffer: %d elements\n", MXU_SIZE * MXU_SIZE);
    DPRINTF(TinyTPU, "  MXU latency: %d cycles\n", mxuLatency);
    DPRINTF(TinyTPU, "  DMA latency: %d cycles\n", dmaLatency);
}

TinyTPU::~TinyTPU()
{
}

void
TinyTPU::startup()
{
    DPRINTF(TinyTPU, "TinyTPU starting up\n");
}

void
TinyTPU::initializeMXU()
{
    for (int i = 0; i < MXU_SIZE; i++) {
        for (int j = 0; j < MXU_SIZE; j++) {
            mxu[i][j].reset();
        }
    }
}

Port &
TinyTPU::getPort(const std::string &if_name, PortID idx)
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
TinyTPU::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(addrRange);
    return ranges;
}

// ============================================================================
// CPU Side Port Implementation
// ============================================================================

TinyTPU::CPUSidePort::CPUSidePort(const std::string& name, TinyTPU *owner) :
    ResponsePort(name), owner(owner), needRetry(false), blockedPacket(nullptr)
{
}

AddrRangeList
TinyTPU::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

Tick
TinyTPU::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    DPRINTF(TinyTPU, "recvAtomic: %s addr 0x%x\n", 
            pkt->cmdString(), pkt->getAddr());
    owner->handleFunctional(pkt);
    return owner->clockPeriod();
}

void
TinyTPU::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(TinyTPU, "recvFunctional: %s addr 0x%x\n",
            pkt->cmdString(), pkt->getAddr());
    owner->handleFunctional(pkt);
}

bool
TinyTPU::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(TinyTPU, "recvTimingReq: %s addr 0x%x\n",
            pkt->cmdString(), pkt->getAddr());
    
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
TinyTPU::CPUSidePort::recvRespRetry()
{
    DPRINTF(TinyTPU, "recvRespRetry\n");
    if (blockedPacket) {
        sendPacket(blockedPacket);
    }
}

void
TinyTPU::CPUSidePort::sendPacket(PacketPtr pkt)
{
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    } else {
        blockedPacket = nullptr;
    }
}

void
TinyTPU::CPUSidePort::trySendRetry()
{
    if (needRetry) {
        needRetry = false;
        sendRetryReq();
    }
}

// ============================================================================
// Memory Side Port Implementation
// ============================================================================

TinyTPU::MemSidePort::MemSidePort(const std::string& name, TinyTPU *owner) :
    RequestPort(name), owner(owner), blockedPacket(nullptr)
{
}

void
TinyTPU::MemSidePort::sendPacket(PacketPtr pkt)
{
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
TinyTPU::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(TinyTPU, "recvTimingResp: %s\n", pkt->cmdString());
    return owner->handleResponse(pkt);
}

void
TinyTPU::MemSidePort::recvReqRetry()
{
    DPRINTF(TinyTPU, "recvReqRetry\n");
    if (blockedPacket) {
        sendPacket(blockedPacket);
        blockedPacket = nullptr;
    }
}

void
TinyTPU::MemSidePort::recvRangeChange()
{
}

// ============================================================================
// Request/Response Handling
// ============================================================================

bool
TinyTPU::handleRequest(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
        pkt->makeResponse();
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
        pkt->makeResponse();
    }
    
    cpuPort.sendPacket(pkt);
    return true;
}

bool
TinyTPU::handleResponse(PacketPtr pkt)
{
    DPRINTF(TinyTPU, "DMA response received\n");
    
    if (!dmaCompleteEvent.scheduled()) {
        schedule(dmaCompleteEvent, clockEdge(Cycles(1)));
    }
    
    delete pkt;
    return true;
}

void
TinyTPU::handleFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
    }
    pkt->makeResponse();
}

// ============================================================================
// Register Access
// ============================================================================

uint32_t
TinyTPU::readReg(Addr offset)
{
    uint32_t value = 0;
    
    switch (offset) {
        case REG_CTRL:         value = regCtrl; break;
        case REG_STATUS:       value = regStatus; break;
        case REG_INSTR_ADDR:   value = (uint32_t)regInstrAddr; break;
        case REG_WEIGHT_ADDR:  value = (uint32_t)regWeightAddr; break;
        case REG_INPUT_ADDR:   value = (uint32_t)regInputAddr; break;
        case REG_OUTPUT_ADDR:  value = (uint32_t)regOutputAddr; break;
        case REG_DIM_M:        value = regDimM; break;
        case REG_DIM_N:        value = regDimN; break;
        case REG_DIM_K:        value = regDimK; break;
        case REG_ACTIVATION:   value = regActivation; break;
        case REG_QUANT_SCALE:  value = regQuantScale; break;
        case REG_QUANT_ZERO:   value = regQuantZero; break;
        case REG_PERF_CYCLES:  value = (uint32_t)regPerfCycles; break;
        case REG_PERF_MACS:    value = (uint32_t)regPerfMacs; break;
        case REG_PERF_TILES:   value = (uint32_t)regPerfTiles; break;
        case REG_MXU_CONFIG:   value = regMxuConfig; break;
        case REG_BIAS_ADDR:    value = (uint32_t)regBiasAddr; break;
        case REG_DATA_TYPE:    value = regDataType; break;
        default:
            DPRINTF(TinyTPU, "Unknown register read at offset 0x%x\n", offset);
    }
    
    DPRINTF(TinyTPU, "Read reg[0x%02x] = 0x%08x\n", offset, value);
    return value;
}

void
TinyTPU::writeReg(Addr offset, uint32_t value)
{
    DPRINTF(TinyTPU, "Write reg[0x%02x] = 0x%08x\n", offset, value);
    
    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                resetTPU();
            } else if (value & CTRL_STOP) {
                stopOperation();
            } else if (value & CTRL_START) {
                startOperation();
            }
            break;
        case REG_INSTR_ADDR:   regInstrAddr = value; break;
        case REG_WEIGHT_ADDR:  regWeightAddr = value; break;
        case REG_INPUT_ADDR:   regInputAddr = value; break;
        case REG_OUTPUT_ADDR:  regOutputAddr = value; break;
        case REG_DIM_M:        regDimM = value; break;
        case REG_DIM_N:        regDimN = value; break;
        case REG_DIM_K:        regDimK = value; break;
        case REG_ACTIVATION:   
            regActivation = value;
            activationType = static_cast<ActivationType>(value);
            break;
        case REG_QUANT_SCALE:  regQuantScale = value; break;
        case REG_QUANT_ZERO:   regQuantZero = value; break;
        case REG_MXU_CONFIG:   regMxuConfig = value; break;
        case REG_BIAS_ADDR:    regBiasAddr = value; break;
        case REG_DATA_TYPE:    
            regDataType = value;
            dataType = static_cast<DataType>(value);
            break;
        default:
            DPRINTF(TinyTPU, "Unknown register write at offset 0x%x\n", offset);
    }
}

// ============================================================================
// Operation Control
// ============================================================================

void
TinyTPU::startOperation()
{
    if (busy) {
        DPRINTF(TinyTPU, "Cannot start: TPU is busy\n");
        return;
    }
    
    DPRINTF(TinyTPU, "Starting TPU operation:\n");
    DPRINTF(TinyTPU, "  Dimensions: M=%d, N=%d, K=%d\n", regDimM, regDimN, regDimK);
    DPRINTF(TinyTPU, "  Weights @ 0x%lx\n", regWeightAddr);
    DPRINTF(TinyTPU, "  Inputs  @ 0x%lx\n", regInputAddr);
    DPRINTF(TinyTPU, "  Outputs @ 0x%lx\n", regOutputAddr);
    DPRINTF(TinyTPU, "  Activation: %d\n", static_cast<int>(activationType));
    
    busy = true;
    aborted = false;
    regStatus = STATUS_BUSY;
    regPerfCycles = 0;
    regPerfMacs = 0;
    regPerfTiles = 0;
    
    // Initialize tiling
    initializeTiling();
    
    // Initialize MXU
    initializeMXU();
    
    // Start processing
    scheduleNextTile();
    
    stats.totalOperations++;
}

void
TinyTPU::stopOperation()
{
    DPRINTF(TinyTPU, "Stopping TPU operation\n");
    aborted = true;
    busy = false;
    regStatus = STATUS_IDLE;
}

void
TinyTPU::resetTPU()
{
    DPRINTF(TinyTPU, "Resetting TPU\n");
    
    stopOperation();
    initializeMXU();
    
    regCtrl = 0;
    regStatus = STATUS_IDLE;
    regPerfCycles = 0;
    regPerfMacs = 0;
    regPerfTiles = 0;
    
    // Clear buffers
    std::fill(unifiedBuffer.begin(), unifiedBuffer.end(), 0);
    std::fill(weightFifo.begin(), weightFifo.end(), 0);
    std::fill(accumBuffer.begin(), accumBuffer.end(), 0);
    std::fill(outputBuffer.begin(), outputBuffer.end(), 0);
}

void
TinyTPU::completeOperation()
{
    DPRINTF(TinyTPU, "TPU operation complete:\n");
    DPRINTF(TinyTPU, "  Tiles: %lu, MACs: %lu, Cycles: %lu\n",
            regPerfTiles, regPerfMacs, regPerfCycles);
    
    busy = false;
    regStatus = STATUS_DONE;
    
    stats.totalTilesProcessed += regPerfTiles;
    stats.totalMacOperations += regPerfMacs;
    stats.totalCycles += regPerfCycles;
}

// ============================================================================
// Tiling Logic
// ============================================================================

void
TinyTPU::initializeTiling()
{
    numTilesM = (regDimM + TILE_SIZE - 1) / TILE_SIZE;
    numTilesN = (regDimN + TILE_SIZE - 1) / TILE_SIZE;
    numTilesK = (regDimK + TILE_SIZE - 1) / TILE_SIZE;
    
    DPRINTF(TinyTPU, "Tiling: %dx%dx%d tiles (tile size %d)\n",
            numTilesM, numTilesN, numTilesK, TILE_SIZE);
    
    currentTileM = 0;
    currentTileN = 0;
    currentTileK = 0;
}

void
TinyTPU::scheduleNextTile()
{
    if (aborted) return;
    
    if (allTilesComplete()) {
        schedule(operationCompleteEvent, clockEdge(Cycles(1)));
        return;
    }
    
    DPRINTF(TinyTPU, "Processing tile [%d,%d,%d]\n",
            currentTileM, currentTileN, currentTileK);
    
    // Schedule MXU computation
    schedule(mxuComputeEvent, clockEdge(Cycles(mxuLatency + dmaLatency)));
}

bool
TinyTPU::allTilesComplete()
{
    return (currentTileM >= numTilesM);
}

// ============================================================================
// Systolic Array Operations
// ============================================================================

void
TinyTPU::loadWeightsToMXU(int tileK, int tileN)
{
    // In weight-stationary dataflow, weights are loaded once
    // and stay in the PEs while inputs stream through
    
    DPRINTF(TinyTPU, "Loading weights for tile [K=%d, N=%d]\n", tileK, tileN);
    
    // For simulation, we initialize weights in the PEs
    // In real hardware, this would be DMA from weight memory
    for (int i = 0; i < MXU_SIZE; i++) {
        for (int j = 0; j < MXU_SIZE; j++) {
            // Calculate global indices
            int k = tileK * TILE_SIZE + i;
            int n = tileN * TILE_SIZE + j;
            
            if (k < (int)regDimK && n < (int)regDimN) {
                // Simulated weight value (in real hw, loaded from memory)
                mxu[i][j].weight = (k * regDimN + n) % 127;
                mxu[i][j].weight_valid = true;
            } else {
                mxu[i][j].weight = 0;
                mxu[i][j].weight_valid = false;
            }
        }
    }
    
    stats.weightBytesLoaded += TILE_SIZE * TILE_SIZE;
}

void
TinyTPU::streamInputsThroughMXU(int tileM, int tileK)
{
    // Inputs stream through the systolic array
    // Each row of inputs enters from the left
    
    DPRINTF(TinyTPU, "Streaming inputs for tile [M=%d, K=%d]\n", tileM, tileK);
    
    // Simulate systolic execution
    // In real hardware, this happens over multiple cycles with pipelining
    for (int cycle = 0; cycle < MXU_SIZE + MXU_SIZE - 1; cycle++) {
        // Each PE processes based on its position
        for (int i = 0; i < MXU_SIZE; i++) {
            for (int j = 0; j < MXU_SIZE; j++) {
                int input_cycle = cycle - j;  // Skewed input
                if (input_cycle >= 0 && input_cycle < MXU_SIZE) {
                    // Calculate global index
                    int m = tileM * TILE_SIZE + i;
                    int k = tileK * TILE_SIZE + input_cycle;
                    
                    if (m < (int)regDimM && k < (int)regDimK) {
                        // Simulated input (in real hw, from unified buffer)
                        int8_t input = (m * regDimK + k) % 127;
                        mxu[i][j].compute(input);
                    }
                }
            }
        }
    }
    
    stats.inputBytesLoaded += TILE_SIZE * TILE_SIZE;
}

void
TinyTPU::drainAccumulators(int tileM, int tileN)
{
    // Copy accumulators to accumulator buffer
    // Apply activation function and quantization
    
    DPRINTF(TinyTPU, "Draining accumulators for tile [M=%d, N=%d]\n", tileM, tileN);
    
    for (int i = 0; i < MXU_SIZE; i++) {
        for (int j = 0; j < MXU_SIZE; j++) {
            int idx = i * MXU_SIZE + j;
            
            // Get accumulated value
            int32_t acc = mxu[i][j].accumulator;
            
            // Apply activation function
            int8_t output = applyActivation(acc);
            
            // Store to output buffer
            outputBuffer[idx] = output;
            
            // Reset accumulator for next tile
            mxu[i][j].accumulator = 0;
        }
    }
    
    stats.activationsComputed += TILE_SIZE * TILE_SIZE;
    stats.outputBytesStored += TILE_SIZE * TILE_SIZE;
}

void
TinyTPU::executeTileMultiply()
{
    if (aborted) return;
    
    DPRINTF(TinyTPU, "Executing tile multiply [%d,%d,%d]\n",
            currentTileM, currentTileN, currentTileK);
    
    // TPU Weight-Stationary Dataflow:
    // 1. Load weights into systolic array (done once per K-N tile)
    // 2. Stream inputs through array
    // 3. Accumulate partial products
    
    if (currentTileK == 0) {
        // First K tile: initialize accumulators to zero
        for (int i = 0; i < MXU_SIZE; i++) {
            for (int j = 0; j < MXU_SIZE; j++) {
                mxu[i][j].accumulator = 0;
            }
        }
    }
    
    // Load weights for this tile
    loadWeightsToMXU(currentTileK, currentTileN);
    
    // Stream inputs and compute
    streamInputsThroughMXU(currentTileM, currentTileK);
    
    // Update performance counters
    regPerfTiles++;
    regPerfMacs += TILE_SIZE * TILE_SIZE * TILE_SIZE;
    regPerfCycles += mxuLatency;
    
    // Move to next tile
    currentTileK++;
    
    if (currentTileK >= numTilesK) {
        // Finished all K tiles for this output tile
        // Drain accumulators with activation
        drainAccumulators(currentTileM, currentTileN);
        
        // Move to next output tile
        currentTileK = 0;
        currentTileN++;
        
        if (currentTileN >= numTilesN) {
            currentTileN = 0;
            currentTileM++;
        }
    }
    
    // Schedule next tile
    scheduleNextTile();
}

// ============================================================================
// Activation Functions
// ============================================================================

int8_t
TinyTPU::applyActivation(int32_t value)
{
    switch (activationType) {
        case ActivationType::RELU:
            return relu(value);
        case ActivationType::RELU6:
            return relu6(value);
        case ActivationType::SIGMOID:
            return sigmoid_approx(value);
        case ActivationType::TANH:
            return tanh_approx(value);
        case ActivationType::LEAKY_RELU:
            return leaky_relu(value);
        case ActivationType::NONE:
        default:
            return quantize(value);
    }
}

int8_t
TinyTPU::relu(int32_t x)
{
    int32_t result = (x > 0) ? x : 0;
    return quantize(result);
}

int8_t
TinyTPU::relu6(int32_t x)
{
    int32_t result = (x < 0) ? 0 : ((x > 6 * regQuantScale) ? 6 * regQuantScale : x);
    return quantize(result);
}

int8_t
TinyTPU::sigmoid_approx(int32_t x)
{
    // Piecewise linear approximation
    if (x < -4 * (int32_t)regQuantScale) return -128 + regQuantZero;
    if (x > 4 * (int32_t)regQuantScale) return 127;
    // Linear region
    return quantize(x / 8 + regQuantScale / 2);
}

int8_t
TinyTPU::tanh_approx(int32_t x)
{
    // Piecewise linear approximation  
    if (x < -2 * (int32_t)regQuantScale) return -128 + regQuantZero;
    if (x > 2 * (int32_t)regQuantScale) return 127;
    return quantize(x / 2);
}

int8_t
TinyTPU::leaky_relu(int32_t x)
{
    int32_t result = (x > 0) ? x : (x / 8);  // alpha = 0.125
    return quantize(result);
}

int8_t
TinyTPU::quantize(int32_t accumulator)
{
    // Apply scale and zero point
    int32_t scaled = (accumulator * (int32_t)regQuantScale) >> 16;
    scaled += regQuantZero;
    
    // Clamp to INT8 range
    if (scaled < -128) scaled = -128;
    if (scaled > 127) scaled = 127;
    
    return static_cast<int8_t>(scaled);
}

// ============================================================================
// DMA Operations
// ============================================================================

void
TinyTPU::loadWeightTile(int tileK, int tileN)
{
    DPRINTF(TinyTPU, "DMA: Loading weight tile [%d,%d]\n", tileK, tileN);
    // In real implementation, this would initiate DMA transfer
}

void
TinyTPU::loadInputTile(int tileM, int tileK)
{
    DPRINTF(TinyTPU, "DMA: Loading input tile [%d,%d]\n", tileM, tileK);
    // In real implementation, this would initiate DMA transfer
}

void
TinyTPU::storeOutputTile(int tileM, int tileN)
{
    DPRINTF(TinyTPU, "DMA: Storing output tile [%d,%d]\n", tileM, tileN);
    // In real implementation, this would initiate DMA transfer
}

void
TinyTPU::handleDmaComplete()
{
    DPRINTF(TinyTPU, "DMA transfer complete\n");
}

// ============================================================================
// Statistics
// ============================================================================

TinyTPU::TPUStats::TPUStats(TinyTPU *parent) :
    statistics::Group(parent),
    ADD_STAT(totalOperations, statistics::units::Count::get(),
             "Total TPU operations executed"),
    ADD_STAT(totalTilesProcessed, statistics::units::Count::get(),
             "Total tiles processed"),
    ADD_STAT(totalMacOperations, statistics::units::Count::get(),
             "Total MAC operations"),
    ADD_STAT(totalCycles, statistics::units::Cycle::get(),
             "Total cycles spent in computation"),
    ADD_STAT(weightBytesLoaded, statistics::units::Byte::get(),
             "Total weight bytes loaded"),
    ADD_STAT(inputBytesLoaded, statistics::units::Byte::get(),
             "Total input bytes loaded"),
    ADD_STAT(outputBytesStored, statistics::units::Byte::get(),
             "Total output bytes stored"),
    ADD_STAT(activationsComputed, statistics::units::Count::get(),
             "Total activations computed")
{
}

} // namespace gem5

