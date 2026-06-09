/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * DianNao-Style Matrix Tile Accelerator Implementation
 */

#include "custom_accel/matrix_tile_accel.hh"

#include <cmath>
#include <algorithm>

#include "base/trace.hh"
#include "debug/MatrixTileAccel.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

MatrixTileAccel::MatrixTileAccel(const Params &params) :
    ClockedObject(params),
    inputBufferSize(params.input_buffer_size),
    weightBufferSize(params.weight_buffer_size),
    accumBufferSize(params.accum_buffer_size),
    outputBufferSize(params.output_buffer_size),
    macLatency(params.mac_latency),
    dmaLatency(params.dma_latency),
    regCtrl(0),
    regStatus(STATUS_IDLE),
    regMatAAddr(0),
    regMatBAddr(0),
    regMatCAddr(0),
    regDimM(0),
    regDimN(0),
    regDimK(0),
    regStrideA(0),
    regStrideB(0),
    regStrideC(0),
    regDataType(0),
    regTileConfig(0),
    regPerfCycles(0),
    regPerfTiles(0),
    regPerfMacs(0),
    regDmaStatus(0),
    regBufStatus(0),
    regAlpha(1.0f),
    regBeta(0.0f),
    currentOp(OpType::MATMUL),
    dataType(DataType::FLOAT64),
    busy(false),
    aborted(false),
    numTilesM(0),
    numTilesN(0),
    numTilesK(0),
    currentTileM(0),
    currentTileN(0),
    currentTileK(0),
    dmaActive(false),
    cpuPort(params.name + ".cpu_side", this),
    memPort(params.name + ".mem_side", this),
    addrRange(params.addr_range),
    tileComputeEvent([this]{ executeTileMultiply(); }, name()),
    dmaCompleteEvent([this]{ handleDmaComplete(); }, name()),
    operationCompleteEvent([this]{ completeOperation(); }, name()),
    stats(this)
{
    // Calculate number of tile slots based on buffer sizes
    // For double precision: 9 doubles per tile = 72 bytes
    size_t tileSize = TILE_SIZE * TILE_SIZE * sizeof(double);
    numTileSlots = std::min({
        inputBufferSize / tileSize,
        weightBufferSize / tileSize,
        accumBufferSize / tileSize
    });
    
    // Allocate buffers
    // Each buffer can hold numTileSlots tiles worth of data
    size_t elementsPerTile = TILE_SIZE * TILE_SIZE;
    inputBuffer.resize(numTileSlots * elementsPerTile, 0.0);
    inputBufferValid.resize(numTileSlots, false);
    
    weightBuffer.resize(numTileSlots * elementsPerTile, 0.0);
    weightBufferValid.resize(numTileSlots, false);
    
    accumBuffer.resize(numTileSlots * elementsPerTile, 0.0);
    accumBufferValid.resize(numTileSlots, false);
    
    outputBuffer.resize(numTileSlots * elementsPerTile, 0.0);
    outputBufferValid.resize(numTileSlots, false);
    
    DPRINTF(MatrixTileAccel, "MatrixTileAccel created:\n");
    DPRINTF(MatrixTileAccel, "  Input buffer: %d bytes (%d tile slots)\n", 
            inputBufferSize, numTileSlots);
    DPRINTF(MatrixTileAccel, "  Weight buffer: %d bytes\n", weightBufferSize);
    DPRINTF(MatrixTileAccel, "  Accum buffer: %d bytes\n", accumBufferSize);
    DPRINTF(MatrixTileAccel, "  MAC latency: %d cycles\n", macLatency);
    DPRINTF(MatrixTileAccel, "  DMA latency: %d cycles\n", dmaLatency);
}

MatrixTileAccel::~MatrixTileAccel()
{
}

void
MatrixTileAccel::startup()
{
    DPRINTF(MatrixTileAccel, "MatrixTileAccel starting up\n");
}

Port &
MatrixTileAccel::getPort(const std::string &if_name, PortID idx)
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
MatrixTileAccel::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(addrRange);
    return ranges;
}

void
MatrixTileAccel::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

// ============================================================================
// CPU Side Port Implementation
// ============================================================================

MatrixTileAccel::CPUSidePort::CPUSidePort(const std::string& name, 
                                           MatrixTileAccel *owner) :
    ResponsePort(name), owner(owner), needRetry(false), blockedPacket(nullptr)
{
}

AddrRangeList
MatrixTileAccel::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

Tick
MatrixTileAccel::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    DPRINTF(MatrixTileAccel, "recvAtomic: %s addr 0x%x\n", 
            pkt->cmdString(), pkt->getAddr());
    owner->handleFunctional(pkt);
    return owner->clockPeriod();
}

void
MatrixTileAccel::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(MatrixTileAccel, "recvFunctional: %s addr 0x%x\n",
            pkt->cmdString(), pkt->getAddr());
    owner->handleFunctional(pkt);
}

bool
MatrixTileAccel::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(MatrixTileAccel, "recvTimingReq: %s addr 0x%x\n",
            pkt->cmdString(), pkt->getAddr());
    
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
MatrixTileAccel::CPUSidePort::recvRespRetry()
{
    DPRINTF(MatrixTileAccel, "recvRespRetry\n");
    if (blockedPacket) {
        sendPacket(blockedPacket);
    }
}

void
MatrixTileAccel::CPUSidePort::sendPacket(PacketPtr pkt)
{
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    } else {
        blockedPacket = nullptr;
    }
}

void
MatrixTileAccel::CPUSidePort::trySendRetry()
{
    if (needRetry) {
        needRetry = false;
        sendRetryReq();
    }
}

// ============================================================================
// Memory Side Port Implementation
// ============================================================================

MatrixTileAccel::MemSidePort::MemSidePort(const std::string& name,
                                           MatrixTileAccel *owner) :
    RequestPort(name), owner(owner), blockedPacket(nullptr)
{
}

void
MatrixTileAccel::MemSidePort::sendPacket(PacketPtr pkt)
{
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
MatrixTileAccel::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(MatrixTileAccel, "recvTimingResp: %s\n", pkt->cmdString());
    return owner->handleResponse(pkt);
}

void
MatrixTileAccel::MemSidePort::recvReqRetry()
{
    DPRINTF(MatrixTileAccel, "recvReqRetry\n");
    if (blockedPacket) {
        sendPacket(blockedPacket);
        blockedPacket = nullptr;
    }
}

void
MatrixTileAccel::MemSidePort::recvRangeChange()
{
    // Do nothing
}

// ============================================================================
// Request/Response Handling
// ============================================================================

bool
MatrixTileAccel::handleRequest(PacketPtr pkt)
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
MatrixTileAccel::handleResponse(PacketPtr pkt)
{
    DPRINTF(MatrixTileAccel, "DMA response received\n");
    
    // Handle DMA completion
    if (!dmaCompleteEvent.scheduled()) {
        schedule(dmaCompleteEvent, clockEdge(Cycles(1)));
    }
    
    delete pkt;
    return true;
}

void
MatrixTileAccel::handleFunctional(PacketPtr pkt)
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
MatrixTileAccel::readReg(Addr offset)
{
    uint32_t value = 0;
    
    switch (offset) {
        case REG_CTRL:         value = regCtrl; break;
        case REG_STATUS:       value = regStatus; break;
        case REG_MAT_A_ADDR:   value = (uint32_t)regMatAAddr; break;
        case REG_MAT_B_ADDR:   value = (uint32_t)regMatBAddr; break;
        case REG_MAT_C_ADDR:   value = (uint32_t)regMatCAddr; break;
        case REG_MAT_DIM_M:    value = regDimM; break;
        case REG_MAT_DIM_N:    value = regDimN; break;
        case REG_MAT_DIM_K:    value = regDimK; break;
        case REG_STRIDE_A:     value = regStrideA; break;
        case REG_STRIDE_B:     value = regStrideB; break;
        case REG_STRIDE_C:     value = regStrideC; break;
        case REG_DATA_TYPE:    value = regDataType; break;
        case REG_TILE_CONFIG:  value = regTileConfig; break;
        case REG_PERF_CYCLES:  value = (uint32_t)regPerfCycles; break;
        case REG_PERF_TILES:   value = (uint32_t)regPerfTiles; break;
        case REG_PERF_MACS:    value = (uint32_t)regPerfMacs; break;
        case REG_DMA_STATUS:   value = regDmaStatus; break;
        case REG_BUF_STATUS:   value = regBufStatus; break;
        default:
            DPRINTF(MatrixTileAccel, "Unknown register read at offset 0x%x\n", offset);
    }
    
    DPRINTF(MatrixTileAccel, "Read reg[0x%02x] = 0x%08x\n", offset, value);
    return value;
}

void
MatrixTileAccel::writeReg(Addr offset, uint32_t value)
{
    DPRINTF(MatrixTileAccel, "Write reg[0x%02x] = 0x%08x\n", offset, value);
    
    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                resetAccelerator();
            } else if (value & CTRL_ABORT) {
                abortOperation();
            } else if (value & CTRL_START) {
                currentOp = static_cast<OpType>((value & CTRL_OP_MASK) >> CTRL_OP_SHIFT);
                startOperation();
            }
            break;
        case REG_MAT_A_ADDR:   regMatAAddr = value; break;
        case REG_MAT_B_ADDR:   regMatBAddr = value; break;
        case REG_MAT_C_ADDR:   regMatCAddr = value; break;
        case REG_MAT_DIM_M:    regDimM = value; break;
        case REG_MAT_DIM_N:    regDimN = value; break;
        case REG_MAT_DIM_K:    regDimK = value; break;
        case REG_STRIDE_A:     regStrideA = value; break;
        case REG_STRIDE_B:     regStrideB = value; break;
        case REG_STRIDE_C:     regStrideC = value; break;
        case REG_DATA_TYPE:    
            regDataType = value; 
            dataType = static_cast<DataType>(value);
            break;
        case REG_TILE_CONFIG:  regTileConfig = value; break;
        default:
            DPRINTF(MatrixTileAccel, "Unknown register write at offset 0x%x\n", offset);
    }
}

// ============================================================================
// Operation Control
// ============================================================================

void
MatrixTileAccel::startOperation()
{
    if (busy) {
        DPRINTF(MatrixTileAccel, "Cannot start: accelerator is busy\n");
        return;
    }
    
    DPRINTF(MatrixTileAccel, "Starting matrix operation:\n");
    DPRINTF(MatrixTileAccel, "  Op type: %d\n", static_cast<int>(currentOp));
    DPRINTF(MatrixTileAccel, "  Dimensions: M=%d, N=%d, K=%d\n", regDimM, regDimN, regDimK);
    DPRINTF(MatrixTileAccel, "  Matrix A @ 0x%lx, stride=%d\n", regMatAAddr, regStrideA);
    DPRINTF(MatrixTileAccel, "  Matrix B @ 0x%lx, stride=%d\n", regMatBAddr, regStrideB);
    DPRINTF(MatrixTileAccel, "  Matrix C @ 0x%lx, stride=%d\n", regMatCAddr, regStrideC);
    
    busy = true;
    aborted = false;
    regStatus = STATUS_BUSY;
    regPerfCycles = 0;
    regPerfTiles = 0;
    regPerfMacs = 0;
    
    // Initialize tiling
    initializeTiling();
    
    // Clear buffers for new operation
    clearBuffers();
    
    // Start processing tiles
    scheduleNextTile();
    
    stats.totalOperations++;
}

void
MatrixTileAccel::abortOperation()
{
    DPRINTF(MatrixTileAccel, "Aborting operation\n");
    aborted = true;
    busy = false;
    regStatus = STATUS_IDLE;
    
    // Clear DMA queues
    while (!dmaLoadQueue.empty()) dmaLoadQueue.pop();
    while (!dmaStoreQueue.empty()) dmaStoreQueue.pop();
    dmaActive = false;
}

void
MatrixTileAccel::resetAccelerator()
{
    DPRINTF(MatrixTileAccel, "Resetting accelerator\n");
    
    abortOperation();
    
    regCtrl = 0;
    regStatus = STATUS_IDLE;
    regPerfCycles = 0;
    regPerfTiles = 0;
    regPerfMacs = 0;
    
    clearBuffers();
}

void
MatrixTileAccel::completeOperation()
{
    DPRINTF(MatrixTileAccel, "Operation complete: %lu tiles, %lu MACs, %lu cycles\n",
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
MatrixTileAccel::initializeTiling()
{
    // Calculate number of tiles in each dimension
    // Use ceiling division to handle non-multiple dimensions
    numTilesM = (regDimM + TILE_SIZE - 1) / TILE_SIZE;
    numTilesN = (regDimN + TILE_SIZE - 1) / TILE_SIZE;
    numTilesK = (regDimK + TILE_SIZE - 1) / TILE_SIZE;
    
    DPRINTF(MatrixTileAccel, "Tiling: %dx%dx%d tiles (tile size %d)\n",
            numTilesM, numTilesN, numTilesK, TILE_SIZE);
    
    // Start from first tile
    currentTileM = 0;
    currentTileN = 0;
    currentTileK = 0;
}

void
MatrixTileAccel::scheduleNextTile()
{
    if (aborted) return;
    
    // Check if we have more tiles to process
    if (allTilesComplete()) {
        // All tiles done, write back remaining results
        if (!dmaStoreQueue.empty()) {
            processDmaQueue();
        } else {
            schedule(operationCompleteEvent, clockEdge(Cycles(1)));
        }
        return;
    }
    
    DPRINTF(MatrixTileAccel, "Processing tile [%d,%d,%d]\n",
            currentTileM, currentTileN, currentTileK);
    
    // Load required tiles if not in buffers
    bool needLoad = false;
    
    // Check A tile (row currentTileM, col currentTileK)
    if (!isTileInInputBuffer(currentTileM, currentTileK)) {
        loadTileToInputBuffer(currentTileM, currentTileK);
        needLoad = true;
    }
    
    // Check B tile (row currentTileK, col currentTileN)
    if (!isTileInWeightBuffer(currentTileK, currentTileN)) {
        loadTileToWeightBuffer(currentTileK, currentTileN);
        needLoad = true;
    }
    
    // Check C tile (need accumulator)
    if (currentTileK == 0 && currentOp != OpType::MATMUL_ACC) {
        // First K tile: initialize accumulator to zero
        int idx = getTileIndex(currentTileM, currentTileN) % numTileSlots;
        for (int i = 0; i < TILE_SIZE * TILE_SIZE; i++) {
            accumBuffer[idx * TILE_SIZE * TILE_SIZE + i] = 0.0;
        }
        accumBufferValid[idx] = true;
    } else if (currentTileK == 0 && currentOp == OpType::MATMUL_ACC) {
        // MATMUL_ACC: load existing C values
        if (!isTileInAccumBuffer(currentTileM, currentTileN)) {
            loadTileToAccumBuffer(currentTileM, currentTileN);
            needLoad = true;
        }
    }
    
    if (needLoad) {
        // Process DMA queue first
        processDmaQueue();
    } else {
        // All data ready, schedule compute
        schedule(tileComputeEvent, clockEdge(Cycles(macLatency)));
    }
}

bool
MatrixTileAccel::allTilesComplete()
{
    return (currentTileM >= numTilesM);
}

int
MatrixTileAccel::getTileIndex(int tileM, int tileN)
{
    return tileM * numTilesN + tileN;
}

// ============================================================================
// Buffer Management
// ============================================================================

void
MatrixTileAccel::clearBuffers()
{
    std::fill(inputBufferValid.begin(), inputBufferValid.end(), false);
    std::fill(weightBufferValid.begin(), weightBufferValid.end(), false);
    std::fill(accumBufferValid.begin(), accumBufferValid.end(), false);
    std::fill(outputBufferValid.begin(), outputBufferValid.end(), false);
}

bool
MatrixTileAccel::isTileInInputBuffer(int tileM, int tileK)
{
    // Simple direct-mapped buffer for now
    int idx = (tileM * numTilesK + tileK) % numTileSlots;
    return inputBufferValid[idx];
}

bool
MatrixTileAccel::isTileInWeightBuffer(int tileK, int tileN)
{
    int idx = (tileK * numTilesN + tileN) % numTileSlots;
    return weightBufferValid[idx];
}

bool
MatrixTileAccel::isTileInAccumBuffer(int tileM, int tileN)
{
    int idx = getTileIndex(tileM, tileN) % numTileSlots;
    return accumBufferValid[idx];
}

void
MatrixTileAccel::loadTileToInputBuffer(int tileM, int tileK)
{
    Addr addr = calculateTileAddress(regMatAAddr, regStrideA, tileM, tileK);
    size_t size = getTileDataSize();
    enqueueDmaLoad(DmaType::LOAD_A, addr, size, tileM, tileK, 0);
}

void
MatrixTileAccel::loadTileToWeightBuffer(int tileK, int tileN)
{
    Addr addr = calculateTileAddress(regMatBAddr, regStrideB, tileK, tileN);
    size_t size = getTileDataSize();
    enqueueDmaLoad(DmaType::LOAD_B, addr, size, tileK, tileN, 0);
}

void
MatrixTileAccel::loadTileToAccumBuffer(int tileM, int tileN)
{
    Addr addr = calculateTileAddress(regMatCAddr, regStrideC, tileM, tileN);
    size_t size = getTileDataSize();
    enqueueDmaLoad(DmaType::LOAD_C, addr, size, tileM, tileN, 0);
}

void
MatrixTileAccel::storeTileFromAccumBuffer(int tileM, int tileN)
{
    Addr addr = calculateTileAddress(regMatCAddr, regStrideC, tileM, tileN);
    size_t size = getTileDataSize();
    enqueueDmaStore(addr, size, tileM, tileN);
}

Addr
MatrixTileAccel::calculateTileAddress(Addr baseAddr, uint32_t stride,
                                       int tileRow, int tileCol)
{
    // Calculate address of tile[tileRow, tileCol]
    // tileRow * TILE_SIZE gives the starting row
    // tileCol * TILE_SIZE gives the starting column
    size_t elemSize = getElementSize();
    Addr rowOffset = tileRow * TILE_SIZE * stride;
    Addr colOffset = tileCol * TILE_SIZE * elemSize;
    return baseAddr + rowOffset + colOffset;
}

// ============================================================================
// DMA Engine
// ============================================================================

void
MatrixTileAccel::enqueueDmaLoad(DmaType type, Addr addr, size_t size,
                                 int tileRow, int tileCol, int tileK)
{
    DmaRequest req;
    req.type = type;
    req.addr = addr;
    req.size = size;
    req.tileRow = tileRow;
    req.tileCol = tileCol;
    req.tileK = tileK;
    dmaLoadQueue.push(req);
    
    DPRINTF(MatrixTileAccel, "Enqueued DMA load: type=%d addr=0x%lx size=%d\n",
            static_cast<int>(type), addr, size);
}

void
MatrixTileAccel::enqueueDmaStore(Addr addr, size_t size,
                                  int tileRow, int tileCol)
{
    DmaRequest req;
    req.type = DmaType::STORE_C;
    req.addr = addr;
    req.size = size;
    req.tileRow = tileRow;
    req.tileCol = tileCol;
    req.tileK = 0;
    dmaStoreQueue.push(req);
    
    DPRINTF(MatrixTileAccel, "Enqueued DMA store: addr=0x%lx size=%d\n", addr, size);
}

void
MatrixTileAccel::processDmaQueue()
{
    if (dmaActive) return;
    
    // Prioritize loads over stores (for performance)
    if (!dmaLoadQueue.empty()) {
        currentDmaReq = dmaLoadQueue.front();
        dmaLoadQueue.pop();
        dmaActive = true;
        regDmaStatus |= STATUS_DMA_ACTIVE;
        
        // For functional simulation, just mark buffer as valid
        // In timing mode, this would be a real memory access
        int idx;
        switch (currentDmaReq.type) {
            case DmaType::LOAD_A:
                idx = (currentDmaReq.tileRow * numTilesK + currentDmaReq.tileCol) % numTileSlots;
                inputBufferValid[idx] = true;
                stats.dmaBytesLoaded += currentDmaReq.size;
                break;
            case DmaType::LOAD_B:
                idx = (currentDmaReq.tileRow * numTilesN + currentDmaReq.tileCol) % numTileSlots;
                weightBufferValid[idx] = true;
                stats.dmaBytesLoaded += currentDmaReq.size;
                break;
            case DmaType::LOAD_C:
                idx = getTileIndex(currentDmaReq.tileRow, currentDmaReq.tileCol) % numTileSlots;
                accumBufferValid[idx] = true;
                stats.dmaBytesLoaded += currentDmaReq.size;
                break;
            default:
                break;
        }
        
        stats.dmaTransfers++;
        
        // Schedule DMA completion
        schedule(dmaCompleteEvent, clockEdge(Cycles(dmaLatency)));
        
    } else if (!dmaStoreQueue.empty()) {
        currentDmaReq = dmaStoreQueue.front();
        dmaStoreQueue.pop();
        dmaActive = true;
        regDmaStatus |= STATUS_DMA_ACTIVE;
        
        stats.dmaBytesStored += currentDmaReq.size;
        stats.dmaTransfers++;
        
        // Schedule DMA completion
        schedule(dmaCompleteEvent, clockEdge(Cycles(dmaLatency)));
    }
}

void
MatrixTileAccel::handleDmaComplete()
{
    DPRINTF(MatrixTileAccel, "DMA complete: type=%d\n", static_cast<int>(currentDmaReq.type));
    
    dmaActive = false;
    regDmaStatus &= ~STATUS_DMA_ACTIVE;
    
    // Check if more DMA pending
    if (!dmaLoadQueue.empty() || !dmaStoreQueue.empty()) {
        processDmaQueue();
    } else if (currentDmaReq.type != DmaType::STORE_C) {
        // All loads done, check if ready to compute
        if (isTileInInputBuffer(currentTileM, currentTileK) &&
            isTileInWeightBuffer(currentTileK, currentTileN)) {
            schedule(tileComputeEvent, clockEdge(Cycles(macLatency)));
        }
    } else {
        // Store complete, check if operation is done
        if (allTilesComplete() && dmaStoreQueue.empty()) {
            schedule(operationCompleteEvent, clockEdge(Cycles(1)));
        }
    }
}

// ============================================================================
// Compute Engine
// ============================================================================

void
MatrixTileAccel::executeTileMultiply()
{
    if (aborted) return;
    
    DPRINTF(MatrixTileAccel, "Executing tile multiply [%d,%d,%d]\n",
            currentTileM, currentTileN, currentTileK);
    
    // Perform the actual tile MAC operation
    performTileMAC(currentTileM, currentTileN, currentTileK);
    
    regPerfTiles++;
    regPerfMacs += TILE_SIZE * TILE_SIZE * TILE_SIZE;  // 27 MACs for 3x3
    regPerfCycles += macLatency;
    
    // Move to next K tile
    currentTileK++;
    
    if (currentTileK >= numTilesK) {
        // Finished accumulating for this C tile
        // Store result back to memory
        storeTileFromAccumBuffer(currentTileM, currentTileN);
        
        // Move to next C tile
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

void
MatrixTileAccel::performTileMAC(int tileM, int tileN, int tileK)
{
    // Get buffer indices
    int aIdx = (tileM * numTilesK + tileK) % numTileSlots;
    int bIdx = (tileK * numTilesN + tileN) % numTileSlots;
    int cIdx = getTileIndex(tileM, tileN) % numTileSlots;
    
    // For this simulation, we compute using the buffer data
    // In reality, the data would be loaded from memory via DMA
    // Here we simulate the 3x3 tile MAC: C_tile += A_tile × B_tile
    
    double* A = &inputBuffer[aIdx * TILE_SIZE * TILE_SIZE];
    double* B = &weightBuffer[bIdx * TILE_SIZE * TILE_SIZE];
    double* C = &accumBuffer[cIdx * TILE_SIZE * TILE_SIZE];
    
    // 3x3 matrix multiply-accumulate
    for (int i = 0; i < TILE_SIZE; i++) {
        for (int j = 0; j < TILE_SIZE; j++) {
            double sum = C[i * TILE_SIZE + j];
            for (int k = 0; k < TILE_SIZE; k++) {
                sum += A[i * TILE_SIZE + k] * B[k * TILE_SIZE + j];
            }
            C[i * TILE_SIZE + j] = sum;
        }
    }
    
    DPRINTF(MatrixTileAccel, "Tile MAC complete: C[%d,%d] accumulated with A[%d,%d] x B[%d,%d]\n",
            tileM, tileN, tileM, tileK, tileK, tileN);
}

// ============================================================================
// Data Type Helpers
// ============================================================================

size_t
MatrixTileAccel::getElementSize()
{
    switch (dataType) {
        case DataType::FLOAT32: return 4;
        case DataType::FLOAT64: return 8;
        case DataType::INT32:   return 4;
        case DataType::INT8:    return 1;
        default: return 8;
    }
}

size_t
MatrixTileAccel::getTileDataSize()
{
    return TILE_SIZE * TILE_SIZE * getElementSize();
}

// ============================================================================
// Statistics
// ============================================================================

MatrixTileAccel::AccelStats::AccelStats(MatrixTileAccel *parent) :
    statistics::Group(parent),
    ADD_STAT(totalOperations, statistics::units::Count::get(),
             "Total matrix operations executed"),
    ADD_STAT(totalTilesProcessed, statistics::units::Count::get(),
             "Total tiles processed"),
    ADD_STAT(totalMacOperations, statistics::units::Count::get(),
             "Total MAC operations"),
    ADD_STAT(totalCycles, statistics::units::Cycle::get(),
             "Total cycles spent in computation"),
    ADD_STAT(dmaBytesLoaded, statistics::units::Byte::get(),
             "Total bytes loaded via DMA"),
    ADD_STAT(dmaBytesStored, statistics::units::Byte::get(),
             "Total bytes stored via DMA"),
    ADD_STAT(dmaTransfers, statistics::units::Count::get(),
             "Total DMA transfers"),
    ADD_STAT(bufferHits, statistics::units::Count::get(),
             "Buffer hits (data already in buffer)"),
    ADD_STAT(bufferMisses, statistics::units::Count::get(),
             "Buffer misses (required DMA load)")
{
}

} // namespace gem5

