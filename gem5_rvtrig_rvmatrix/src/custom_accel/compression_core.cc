/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * Compression Core Implementation
 */

#include "custom_accel/compression_core.hh"

#include "base/trace.hh"
#include "debug/CompressionCore.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

CompressionCore::CompressionCore(const Params &params) :
    ClockedObject(params),
    regCtrl(0),
    regStatus(0),
    regSrcAddr(0),
    regDstAddr(0),
    regLength(0),
    regOutLen(0),
    regConfig(0),
    regStats(0),
    kParam(params.k_param),
    blockSize(params.block_size),
    compressionLatency(params.compression_latency),
    busy(false),
    bytesProcessed(0),
    cpuPort(params.name + ".cpu_side", this),
    memPort(params.name + ".mem_side", this),
    addrRange(params.addr_range),
    stats(this),
    compressionEvent([this]{ completeOperation(); }, name())
{
    DPRINTF(CompressionCore, "CompressionCore created at %s\n", 
            addrRange.to_string());
}

Port &
CompressionCore::getPort(const std::string &if_name, PortID idx)
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
CompressionCore::startup()
{
    ClockedObject::startup();
    // Notify the bus of our address ranges
    cpuPort.sendRangeChange();
}

// CPUSidePort implementation
CompressionCore::CPUSidePort::CPUSidePort(const std::string& name, 
                                          CompressionCore *owner) :
    ResponsePort(name), owner(owner), needRetry(false), blockedPacket(nullptr)
{
}

void
CompressionCore::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
CompressionCore::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
CompressionCore::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(CompressionCore, "Sending retry req\n");
        sendRetryReq();
    }
}

Tick
CompressionCore::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    return owner->clockPeriod();
}

void
CompressionCore::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
CompressionCore::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
CompressionCore::CPUSidePort::recvRespRetry()
{
    assert(blockedPacket != nullptr);
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

// MemSidePort implementation
CompressionCore::MemSidePort::MemSidePort(const std::string& name,
                                          CompressionCore *owner) :
    RequestPort(name), owner(owner), blockedPacket(nullptr)
{
}

void
CompressionCore::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
CompressionCore::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
CompressionCore::MemSidePort::recvReqRetry()
{
    assert(blockedPacket != nullptr);
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
CompressionCore::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// Main request handling
bool
CompressionCore::handleRequest(PacketPtr pkt)
{
    DPRINTF(CompressionCore, "handleRequest for addr %#x\n", pkt->getAddr());

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

    // Schedule response
    cpuPort.sendPacket(pkt);
    return true;
}

bool
CompressionCore::handleResponse(PacketPtr pkt)
{
    DPRINTF(CompressionCore, "handleResponse for addr %#x\n", pkt->getAddr());
    delete pkt;
    return true;
}

void
CompressionCore::handleFunctional(PacketPtr pkt)
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
CompressionCore::readReg(Addr offset)
{
    switch (offset) {
        case REG_CTRL:      return regCtrl;
        case REG_STATUS:    return regStatus;
        case REG_SRC_ADDR:  return regSrcAddr;
        case REG_DST_ADDR:  return regDstAddr;
        case REG_LENGTH:    return regLength;
        case REG_OUT_LEN:   return regOutLen;
        case REG_CONFIG:    return regConfig;
        case REG_STATS:     return regStats;
        default:
            DPRINTF(CompressionCore, "Unknown register read at offset %#x\n", 
                    offset);
            return 0;
    }
}

void
CompressionCore::writeReg(Addr offset, uint32_t value)
{
    DPRINTF(CompressionCore, "writeReg offset=%#x value=%#x\n", offset, value);

    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                // Reset the accelerator
                regStatus = 0;
                busy = false;
                inputBuffer.clear();
                outputBuffer.clear();
            } else if ((value & CTRL_START) && !busy) {
                // Start compression/decompression
                if (value & CTRL_COMPRESS) {
                    startCompression();
                } else {
                    startDecompression();
                }
            }
            break;
        case REG_STATUS:
            // Status is read-only, but allow clearing done/error flags
            regStatus &= ~(value & (STATUS_DONE | STATUS_ERROR));
            break;
        case REG_SRC_ADDR:
            regSrcAddr = value;
            break;
        case REG_DST_ADDR:
            regDstAddr = value;
            break;
        case REG_LENGTH:
            regLength = value;
            break;
        case REG_CONFIG:
            regConfig = value;
            // Update k-parameter from config
            kParam = (value >> 0) & 0xFF;
            blockSize = (value >> 8) & 0xFFFF;
            if (blockSize == 0) blockSize = 64;  // Default block size
            break;
        default:
            DPRINTF(CompressionCore, "Unknown register write at offset %#x\n", 
                    offset);
    }
}

void
CompressionCore::startCompression()
{
    DPRINTF(CompressionCore, "Starting compression: src=%#x len=%u\n",
            regSrcAddr, regLength);

    busy = true;
    regStatus = STATUS_BUSY;
    bytesProcessed = 0;

    // For simulation, we model the compression with fixed latency
    // In a real implementation, this would involve DMA transfers
    
    // Create simulated input data (in real hw, this comes from memory)
    inputBuffer.resize(regLength);
    for (unsigned i = 0; i < regLength; i++) {
        inputBuffer[i] = (i * 17 + 31) & 0xFF;  // Simulated data pattern
    }

    // Perform compression
    riceCompress(inputBuffer, outputBuffer);

    // Calculate latency based on data size and block size
    unsigned numBlocks = (regLength + blockSize - 1) / blockSize;
    Tick completionTime = curTick() + 
                          numBlocks * compressionLatency * clockPeriod();

    schedule(compressionEvent, completionTime);
    stats.compressionOps++;
    stats.bytesCompressed += regLength;
}

void
CompressionCore::startDecompression()
{
    DPRINTF(CompressionCore, "Starting decompression: src=%#x len=%u\n",
            regSrcAddr, regLength);

    busy = true;
    regStatus = STATUS_BUSY;

    // For simulation purposes
    inputBuffer.resize(regLength);
    riceDecompress(inputBuffer, outputBuffer, regLength * 2);

    unsigned numBlocks = (regLength + blockSize - 1) / blockSize;
    Tick completionTime = curTick() + 
                          numBlocks * compressionLatency * clockPeriod();

    schedule(compressionEvent, completionTime);
    stats.decompressionOps++;
    stats.bytesDecompressed += regLength;
}

void
CompressionCore::completeOperation()
{
    DPRINTF(CompressionCore, "Compression/decompression complete. "
            "Output size: %zu bytes\n", outputBuffer.size());

    busy = false;
    regStatus = STATUS_DONE;
    regOutLen = outputBuffer.size();

    // Calculate compression ratio (fixed point, 8.8 format)
    if (regLength > 0) {
        regStats = (outputBuffer.size() * 256) / regLength;
    }

    // Clear control register
    regCtrl = 0;
}

// Rice compression implementation (simplified for simulation)
void
CompressionCore::riceCompress(const std::vector<uint8_t>& input,
                              std::vector<uint8_t>& output)
{
    output.clear();
    
    if (input.empty()) return;

    // Simplified Rice coding for simulation
    // Real CCSDS implementation would be more complex
    uint32_t bitBuffer = 0;
    int bitCount = 0;

    auto writeBits = [&](uint32_t value, int bits) {
        bitBuffer = (bitBuffer << bits) | (value & ((1 << bits) - 1));
        bitCount += bits;
        while (bitCount >= 8) {
            bitCount -= 8;
            output.push_back((bitBuffer >> bitCount) & 0xFF);
        }
    };

    // Write header with original size
    writeBits(input.size() >> 24, 8);
    writeBits(input.size() >> 16, 8);
    writeBits(input.size() >> 8, 8);
    writeBits(input.size(), 8);

    // Encode each sample
    uint8_t predictor = 0;
    for (uint8_t sample : input) {
        // Prediction residual (simple delta coding)
        int16_t residual = sample - predictor;
        predictor = sample;

        // Map to unsigned (zig-zag encoding)
        uint16_t mapped = (residual < 0) ? 
                          (-residual * 2 - 1) : (residual * 2);

        // Rice coding
        uint16_t quotient = mapped >> kParam;
        uint16_t remainder = mapped & ((1 << kParam) - 1);

        // Unary code for quotient (limited for simulation)
        quotient = std::min(quotient, (uint16_t)15);
        for (uint16_t i = 0; i < quotient; i++) {
            writeBits(1, 1);
        }
        writeBits(0, 1);

        // Binary code for remainder
        writeBits(remainder, kParam);
    }

    // Flush remaining bits
    if (bitCount > 0) {
        output.push_back((bitBuffer << (8 - bitCount)) & 0xFF);
    }

    DPRINTF(CompressionCore, "Compressed %zu bytes to %zu bytes (ratio: %.2f)\n",
            input.size(), output.size(), 
            (float)output.size() / input.size());
}

void
CompressionCore::riceDecompress(const std::vector<uint8_t>& input,
                                std::vector<uint8_t>& output,
                                size_t originalSize)
{
    output.clear();
    output.reserve(originalSize);

    // Simplified decompression (inverse of compression)
    // For simulation purposes, just output expanded data
    for (size_t i = 0; i < originalSize && i < input.size() * 2; i++) {
        output.push_back((i < input.size()) ? input[i] : 0);
    }
}

AddrRangeList
CompressionCore::getAddrRanges() const
{
    return AddrRangeList{addrRange};
}

void
CompressionCore::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

// Statistics
CompressionCore::CompressionStats::CompressionStats(CompressionCore *parent) :
    statistics::Group(parent),
    ADD_STAT(compressionOps, statistics::units::Count::get(),
             "Number of compression operations"),
    ADD_STAT(decompressionOps, statistics::units::Count::get(),
             "Number of decompression operations"),
    ADD_STAT(bytesCompressed, statistics::units::Byte::get(),
             "Total bytes compressed"),
    ADD_STAT(bytesDecompressed, statistics::units::Byte::get(),
             "Total bytes decompressed"),
    ADD_STAT(totalCycles, statistics::units::Cycle::get(),
             "Total cycles spent on compression/decompression"),
    ADD_STAT(avgCompressionRatio, statistics::units::Ratio::get(),
             "Average compression ratio")
{
    avgCompressionRatio = bytesCompressed / (bytesDecompressed + 1);
}

} // namespace gem5

