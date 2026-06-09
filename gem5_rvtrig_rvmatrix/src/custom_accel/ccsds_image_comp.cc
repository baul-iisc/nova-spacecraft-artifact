/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * CCSDS 122.0 Image Compression Accelerator Implementation
 */

#include "custom_accel/ccsds_image_comp.hh"

#include <algorithm>
#include <cmath>

#include "base/trace.hh"
#include "debug/CCSDSImageComp.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

// CPUSidePort implementation
CCSDSImageComp::CPUSidePort::CPUSidePort(const std::string& name, 
                                          CCSDSImageComp *owner)
    : ResponsePort(name, owner), owner(owner), needRetry(false),
      blockedPacket(nullptr)
{
}

void
CCSDSImageComp::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
CCSDSImageComp::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
CCSDSImageComp::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(CCSDSImageComp, "Sending retry for CPU port\n");
        sendRetryReq();
    }
}

Tick
CCSDSImageComp::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    return owner->clockPeriod();
}

void
CCSDSImageComp::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
CCSDSImageComp::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
CCSDSImageComp::CPUSidePort::recvRespRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

// MemSidePort implementation
CCSDSImageComp::MemSidePort::MemSidePort(const std::string& name, 
                                          CCSDSImageComp *owner)
    : RequestPort(name, owner), owner(owner), blockedPacket(nullptr)
{
}

void
CCSDSImageComp::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
CCSDSImageComp::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
CCSDSImageComp::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
CCSDSImageComp::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// Statistics
CCSDSImageComp::ImageCompStats::ImageCompStats(CCSDSImageComp *parent)
    : statistics::Group(parent),
      ADD_STAT(imagesCompressed, statistics::units::Count::get(),
               "Number of images compressed"),
      ADD_STAT(imagesDecompressed, statistics::units::Count::get(),
               "Number of images decompressed"),
      ADD_STAT(tilesProcessed, statistics::units::Count::get(),
               "Number of tiles processed"),
      ADD_STAT(totalPixels, statistics::units::Count::get(),
               "Total pixels processed"),
      ADD_STAT(totalInputBytes, statistics::units::Byte::get(),
               "Total input bytes"),
      ADD_STAT(totalOutputBytes, statistics::units::Byte::get(),
               "Total output bytes"),
      ADD_STAT(totalCycles, statistics::units::Cycle::get(),
               "Total cycles for all operations"),
      ADD_STAT(avgCompressionRatio, statistics::units::Ratio::get(),
               "Average compression ratio"),
      ADD_STAT(throughput, statistics::units::Rate<
               statistics::units::Count, statistics::units::Cycle>::get(),
               "Pixels per cycle throughput")
{
    avgCompressionRatio = totalInputBytes / totalOutputBytes;
    throughput = totalPixels / totalCycles;
}

// Main constructor
CCSDSImageComp::CCSDSImageComp(const Params &params)
    : ClockedObject(params),
      regCtrl(0), regStatus(STATUS_IDLE), regMode(0),
      regInputBase(0), regOutputBase(0),
      regImgWidth(0), regImgHeight(0), regBitDepth(8),
      regSegmentSize(64), regDwtLevels(3),
      regWaveletType(0),  // Default: 9/7 lossy
      regTargetBpp(0x0100),  // 1.0 bpp default
      regOutputLen(0), regCompRatio(0), regQuality(0),
      regTileWidth(256), regTileHeight(256),
      regHeaderConfig(0),
      regPerfPixels(0), regPerfCycles(0),
      regPerfBytesIn(0), regPerfBytesOut(0),
      regInterrupt(0),
      dwtLatency(params.dwt_latency),
      bpeLatency(params.bpe_latency),
      lineBufferSize(params.line_buffer_size),
      maxTileSize(params.max_tile_size),
      cpuPort(params.name + ".cpu_side", this),
      memPort(params.name + ".mem_side", this),
      addrRange(params.addr_range),
      stats(this),
      processEvent([this]{ completeOperation(); }, name())
{
    // Pre-allocate buffers
    inputBuffer.resize(maxTileSize);
    coeffBuffer.resize(maxTileSize);
    outputBuffer.resize(maxTileSize);
    dwtLineBuffer.resize(lineBufferSize);
    
    DPRINTF(CCSDSImageComp, "CCSDS Image Compression Accelerator created\n");
    DPRINTF(CCSDSImageComp, "  DWT latency: %d, BPE latency: %d\n",
            dwtLatency, bpeLatency);
    DPRINTF(CCSDSImageComp, "  Max tile size: %d pixels\n", maxTileSize);
}

void
CCSDSImageComp::startup()
{
    DPRINTF(CCSDSImageComp, "CCSDS Image Comp starting up\n");
}

Port &
CCSDSImageComp::getPort(const std::string &if_name, PortID idx)
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
CCSDSImageComp::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(addrRange);
    return ranges;
}

void
CCSDSImageComp::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

bool
CCSDSImageComp::handleRequest(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
        DPRINTF(CCSDSImageComp, "Read from 0x%x: 0x%x\n", offset, value);
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
        DPRINTF(CCSDSImageComp, "Write to 0x%x: 0x%x\n", offset, value);
    }
    
    pkt->makeResponse();
    cpuPort.sendPacket(pkt);
    return true;
}

bool
CCSDSImageComp::handleResponse(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void
CCSDSImageComp::handleFunctional(PacketPtr pkt)
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
CCSDSImageComp::readReg(Addr offset)
{
    switch (offset) {
        case REG_CTRL:          return regCtrl;
        case REG_STATUS:        return regStatus;
        case REG_MODE:          return regMode;
        case REG_INPUT_BASE:    return regInputBase;
        case REG_OUTPUT_BASE:   return regOutputBase;
        case REG_IMG_WIDTH:     return regImgWidth;
        case REG_IMG_HEIGHT:    return regImgHeight;
        case REG_BIT_DEPTH:     return regBitDepth;
        case REG_SEGMENT_SIZE:  return regSegmentSize;
        case REG_DWT_LEVELS:    return regDwtLevels;
        case REG_WAVELET_TYPE:  return regWaveletType;
        case REG_TARGET_BPP:    return regTargetBpp;
        case REG_OUTPUT_LEN:    return regOutputLen;
        case REG_COMP_RATIO:    return regCompRatio;
        case REG_QUALITY:       return regQuality;
        case REG_TILE_WIDTH:    return regTileWidth;
        case REG_TILE_HEIGHT:   return regTileHeight;
        case REG_HEADER_CONFIG: return regHeaderConfig;
        case REG_PERF_PIXELS:   return (uint32_t)regPerfPixels;
        case REG_PERF_CYCLES:   return (uint32_t)regPerfCycles;
        case REG_PERF_BYTES_IN: return (uint32_t)regPerfBytesIn;
        case REG_PERF_BYTES_OUT:return (uint32_t)regPerfBytesOut;
        case REG_INTERRUPT:     return regInterrupt;
        case REG_VERSION:       return VERSION;
        default:
            DPRINTF(CCSDSImageComp, "Unknown register read at 0x%x\n", offset);
            return 0;
    }
}

void
CCSDSImageComp::writeReg(Addr offset, uint32_t value)
{
    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                regStatus = STATUS_IDLE;
                regPerfPixels = 0;
                regPerfCycles = 0;
                regPerfBytesIn = 0;
                regPerfBytesOut = 0;
            } else if (value & CTRL_START) {
                startOperation();
            }
            break;
        case REG_MODE:          regMode = value; break;
        case REG_INPUT_BASE:    regInputBase = value; break;
        case REG_OUTPUT_BASE:   regOutputBase = value; break;
        case REG_IMG_WIDTH:     regImgWidth = value; break;
        case REG_IMG_HEIGHT:    regImgHeight = value; break;
        case REG_BIT_DEPTH:     regBitDepth = value; break;
        case REG_SEGMENT_SIZE:  regSegmentSize = value; break;
        case REG_DWT_LEVELS:    regDwtLevels = std::min(value, 5u); break;
        case REG_WAVELET_TYPE:  regWaveletType = value; break;
        case REG_TARGET_BPP:    regTargetBpp = value; break;
        case REG_TILE_WIDTH:    regTileWidth = value; break;
        case REG_TILE_HEIGHT:   regTileHeight = value; break;
        case REG_HEADER_CONFIG: regHeaderConfig = value; break;
        case REG_INTERRUPT:     regInterrupt = value; break;
        default:
            DPRINTF(CCSDSImageComp, "Unknown register write at 0x%x: 0x%x\n",
                    offset, value);
    }
}

void
CCSDSImageComp::startOperation()
{
    if (regStatus & STATUS_BUSY) {
        DPRINTF(CCSDSImageComp, "Cannot start: already busy\n");
        return;
    }
    
    // Validate parameters
    if (regImgWidth == 0 || regImgHeight == 0) {
        DPRINTF(CCSDSImageComp, "Invalid image dimensions\n");
        regStatus = STATUS_ERROR;
        return;
    }
    
    regStatus = STATUS_BUSY;
    Mode mode = static_cast<Mode>(regMode);
    
    DPRINTF(CCSDSImageComp, "Starting operation mode %d on %dx%d image\n",
            regMode, regImgWidth, regImgHeight);
    
    switch (mode) {
        case Mode::COMPRESS:
            executeCompress();
            break;
        case Mode::DECOMPRESS:
            executeDecompress();
            break;
        case Mode::DWT_ONLY:
            executeDwtOnly();
            break;
        case Mode::BPE_ONLY:
            executeBpeOnly();
            break;
        default:
            DPRINTF(CCSDSImageComp, "Unknown mode %d\n", regMode);
            regStatus = STATUS_ERROR;
            return;
    }
}

void
CCSDSImageComp::executeCompress()
{
    DPRINTF(CCSDSImageComp, "Compressing %dx%dx%d image, %d DWT levels\n",
            regImgWidth, regImgHeight, regBitDepth, regDwtLevels);
    
    uint64_t pixels = (uint64_t)regImgWidth * regImgHeight;
    uint64_t inputBytes = pixels * ((regBitDepth + 7) / 8);
    
    // Estimate cycles for compression
    uint64_t cycles = estimateCycles(pixels, regDwtLevels);
    
    // Estimate output size based on target bpp
    uint32_t targetBits = (pixels * regTargetBpp) >> 8;  // Fixed point
    regOutputLen = (targetBits + 7) / 8;
    regOutputLen += 32;  // Header overhead
    
    // Compute compression ratio (fixed point 8.8)
    if (regOutputLen > 0) {
        regCompRatio = (inputBytes << 8) / regOutputLen;
    }
    
    // Update performance counters
    regPerfPixels += pixels;
    regPerfCycles += cycles;
    regPerfBytesIn += inputBytes;
    regPerfBytesOut += regOutputLen;
    
    stats.imagesCompressed++;
    stats.totalPixels += pixels;
    stats.totalInputBytes += inputBytes;
    stats.totalOutputBytes += regOutputLen;
    stats.totalCycles += cycles;
    
    // Calculate tiles
    unsigned numTilesX = (regImgWidth + regTileWidth - 1) / regTileWidth;
    unsigned numTilesY = (regImgHeight + regTileHeight - 1) / regTileHeight;
    stats.tilesProcessed += numTilesX * numTilesY;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSImageComp::executeDecompress()
{
    DPRINTF(CCSDSImageComp, "Decompressing to %dx%dx%d image\n",
            regImgWidth, regImgHeight, regBitDepth);
    
    uint64_t pixels = (uint64_t)regImgWidth * regImgHeight;
    
    // Decompression cycles: BPE decode + inverse DWT
    uint64_t cycles = estimateCycles(pixels, regDwtLevels);
    cycles += pixels / regSegmentSize * bpeLatency * 2;  // BPE decode more expensive
    
    regOutputLen = pixels * ((regBitDepth + 7) / 8);
    
    regPerfPixels += pixels;
    regPerfCycles += cycles;
    regPerfBytesOut += regOutputLen;
    
    stats.imagesDecompressed++;
    stats.totalPixels += pixels;
    stats.totalOutputBytes += regOutputLen;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSImageComp::executeDwtOnly()
{
    DPRINTF(CCSDSImageComp, "DWT transform only: %dx%d, %d levels\n",
            regImgWidth, regImgHeight, regDwtLevels);
    
    uint64_t pixels = (uint64_t)regImgWidth * regImgHeight;
    
    // DWT cycles only
    uint64_t cycles = pixels * dwtLatency * regDwtLevels;
    
    regOutputLen = pixels * 4;  // 32-bit coefficients
    
    regPerfCycles += cycles;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSImageComp::executeBpeOnly()
{
    DPRINTF(CCSDSImageComp, "BPE encoding only: %dx%d\n",
            regImgWidth, regImgHeight);
    
    uint64_t coeffs = (uint64_t)regImgWidth * regImgHeight;
    
    // BPE cycles
    uint64_t segments = (coeffs + regSegmentSize - 1) / regSegmentSize;
    uint64_t cycles = segments * bpeLatency;
    
    regPerfCycles += cycles;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSImageComp::completeOperation()
{
    DPRINTF(CCSDSImageComp, "Operation complete, output: %u bytes\n", regOutputLen);
    
    regStatus = STATUS_DONE;
    regCtrl &= ~CTRL_START;
    
    // Generate interrupt if enabled
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x01;
    }
}

unsigned
CCSDSImageComp::estimateCycles(unsigned pixels, unsigned levels)
{
    unsigned cycles = 0;
    
    // DWT cycles: each level processes decreasing number of coefficients
    unsigned size = pixels;
    for (unsigned l = 0; l < levels; l++) {
        cycles += size * dwtLatency;
        size = size / 4;  // Each level reduces to 1/4 at LL subband
    }
    
    // BPE cycles
    unsigned segments = (pixels + regSegmentSize - 1) / regSegmentSize;
    cycles += segments * bpeLatency;
    
    // Add memory transfer overhead
    cycles += pixels / 64;  // Burst transfers
    
    return cycles;
}

uint32_t
CCSDSImageComp::estimateCompressedSize()
{
    uint64_t pixels = (uint64_t)regImgWidth * regImgHeight;
    uint32_t targetBits = (pixels * regTargetBpp) >> 8;
    return (targetBits + 7) / 8 + 32;  // Plus header
}

void
CCSDSImageComp::dwtForward2D(std::vector<int32_t>& data, 
                              unsigned width, unsigned height)
{
    // Perform 2D DWT by applying 1D transforms to rows then columns
    WaveletType wt = static_cast<WaveletType>(regWaveletType);
    
    for (unsigned level = 0; level < regDwtLevels; level++) {
        unsigned levelWidth = width >> level;
        unsigned levelHeight = height >> level;
        
        // Transform rows
        for (unsigned y = 0; y < levelHeight; y++) {
            if (wt == WaveletType::LOSSLESS_5_3) {
                dwtForward1D_53(&data[y * width], levelWidth);
            } else {
                // For 9/7, convert to float, transform, convert back
                for (unsigned x = 0; x < levelWidth; x++) {
                    dwtLineBuffer[x] = static_cast<float>(data[y * width + x]);
                }
                dwtForward1D_97(dwtLineBuffer.data(), levelWidth);
                for (unsigned x = 0; x < levelWidth; x++) {
                    data[y * width + x] = static_cast<int32_t>(dwtLineBuffer[x]);
                }
            }
        }
        
        // Transform columns
        for (unsigned x = 0; x < levelWidth; x++) {
            // Extract column
            for (unsigned y = 0; y < levelHeight; y++) {
                dwtLineBuffer[y] = static_cast<float>(data[y * width + x]);
            }
            
            if (wt == WaveletType::LOSSLESS_5_3) {
                std::vector<int32_t> col(levelHeight);
                for (unsigned y = 0; y < levelHeight; y++) {
                    col[y] = static_cast<int32_t>(dwtLineBuffer[y]);
                }
                dwtForward1D_53(col.data(), levelHeight);
                for (unsigned y = 0; y < levelHeight; y++) {
                    data[y * width + x] = col[y];
                }
            } else {
                dwtForward1D_97(dwtLineBuffer.data(), levelHeight);
                for (unsigned y = 0; y < levelHeight; y++) {
                    data[y * width + x] = static_cast<int32_t>(dwtLineBuffer[y]);
                }
            }
        }
    }
}

void
CCSDSImageComp::dwtForward1D_97(float* data, unsigned length)
{
    if (length < 2) return;
    
    // Lifting steps for 9/7 wavelet
    unsigned half = length / 2;
    
    // Predict 1
    for (unsigned i = 0; i < half; i++) {
        unsigned j = 2 * i + 1;
        float left = data[2 * i];
        float right = (j + 1 < length) ? data[j + 1] : data[j - 1];
        data[j] += ALPHA * (left + right);
    }
    
    // Update 1
    for (unsigned i = 0; i < half; i++) {
        unsigned j = 2 * i;
        float left = (j > 0) ? data[j - 1] : data[j + 1];
        float right = data[j + 1];
        data[j] += BETA * (left + right);
    }
    
    // Predict 2
    for (unsigned i = 0; i < half; i++) {
        unsigned j = 2 * i + 1;
        float left = data[2 * i];
        float right = (j + 1 < length) ? data[j + 1] : data[j - 1];
        data[j] += GAMMA * (left + right);
    }
    
    // Update 2
    for (unsigned i = 0; i < half; i++) {
        unsigned j = 2 * i;
        float left = (j > 0) ? data[j - 1] : data[j + 1];
        float right = data[j + 1];
        data[j] += DELTA * (left + right);
    }
    
    // Scale and deinterleave
    std::vector<float> temp(length);
    for (unsigned i = 0; i < half; i++) {
        temp[i] = data[2 * i] * SCALE_LO;
        temp[half + i] = data[2 * i + 1] * SCALE_HI;
    }
    std::copy(temp.begin(), temp.end(), data);
}

void
CCSDSImageComp::dwtForward1D_53(int32_t* data, unsigned length)
{
    if (length < 2) return;
    
    unsigned half = length / 2;
    
    // Predict (odd samples)
    for (unsigned i = 0; i < half; i++) {
        unsigned j = 2 * i + 1;
        int32_t left = data[2 * i];
        int32_t right = (j + 1 < length) ? data[j + 1] : data[j - 1];
        data[j] -= (left + right) / 2;
    }
    
    // Update (even samples)
    for (unsigned i = 0; i < half; i++) {
        unsigned j = 2 * i;
        int32_t left = (j > 0) ? data[j - 1] : data[j + 1];
        int32_t right = data[j + 1];
        data[j] += (left + right + 2) / 4;
    }
    
    // Deinterleave
    std::vector<int32_t> temp(length);
    for (unsigned i = 0; i < half; i++) {
        temp[i] = data[2 * i];
        temp[half + i] = data[2 * i + 1];
    }
    std::copy(temp.begin(), temp.end(), data);
}

void
CCSDSImageComp::bpeEncode(const std::vector<int32_t>& coeffs,
                           std::vector<uint8_t>& output)
{
    output.clear();
    
    // CCSDS 122.0 BPE: segment-based encoding
    unsigned numSegments = (coeffs.size() + regSegmentSize - 1) / regSegmentSize;
    
    for (unsigned seg = 0; seg < numSegments; seg++) {
        unsigned start = seg * regSegmentSize;
        unsigned count = std::min((size_t)regSegmentSize, coeffs.size() - start);
        encodeSegment(&coeffs[start], count, output);
    }
}

void
CCSDSImageComp::encodeSegment(const int32_t* coeffs, unsigned count,
                               std::vector<uint8_t>& output)
{
    // Simplified BPE segment encoding
    // In real implementation: DC coding, bit-plane iteration, etc.
    
    // Find maximum absolute value to determine bit depth
    int32_t maxVal = 0;
    for (unsigned i = 0; i < count; i++) {
        int32_t absVal = coeffs[i] >= 0 ? coeffs[i] : -coeffs[i];
        if (absVal > maxVal) maxVal = absVal;
    }
    
    // Determine number of bit planes
    unsigned bitPlanes = 0;
    while ((1u << bitPlanes) <= (unsigned)maxVal) bitPlanes++;
    
    // Header byte: number of bit planes
    output.push_back(bitPlanes & 0xFF);
    
    // Encode each coefficient using Golomb-Rice
    unsigned bitPos = 0;
    unsigned k = 4;  // Rice parameter
    for (unsigned i = 0; i < count; i++) {
        golombRiceEncode(coeffs[i], k, output, bitPos);
    }
}

void
CCSDSImageComp::golombRiceEncode(int32_t value, unsigned k,
                                  std::vector<uint8_t>& output, 
                                  unsigned& bitPos)
{
    // Map signed to unsigned (zigzag encoding)
    uint32_t uval = value >= 0 ? 2 * value : (-2 * value - 1);
    
    // Split into quotient and remainder
    uint32_t quot = uval >> k;
    uint32_t rem = uval & ((1 << k) - 1);
    
    // Encode quotient in unary (simplified: just store length)
    // In real implementation, this would be bit-by-bit
    while (output.size() <= (bitPos + quot + k + 7) / 8) {
        output.push_back(0);
    }
    
    bitPos += quot + 1 + k;
}

void
CCSDSImageComp::writeHeader(std::vector<uint8_t>& output)
{
    // CCSDS 122.0 header
    output.push_back(0x00);  // Header ID
    output.push_back((regImgWidth >> 8) & 0xFF);
    output.push_back(regImgWidth & 0xFF);
    output.push_back((regImgHeight >> 8) & 0xFF);
    output.push_back(regImgHeight & 0xFF);
    output.push_back(regBitDepth & 0x1F);
    output.push_back(((regDwtLevels & 0x7) << 4) | (regWaveletType & 0x1));
    output.push_back(regSegmentSize & 0xFF);
}

} // namespace gem5



