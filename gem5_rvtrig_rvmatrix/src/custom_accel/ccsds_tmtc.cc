/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * CCSDS TM/TC Protocol Accelerator Implementation
 */

#include "custom_accel/ccsds_tmtc.hh"

#include "base/trace.hh"
#include "debug/CCSDSTmTc.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

// CPUSidePort implementation
CCSDSTmTc::CPUSidePort::CPUSidePort(const std::string& name, CCSDSTmTc *owner)
    : ResponsePort(name, owner), owner(owner), needRetry(false),
      blockedPacket(nullptr)
{
}

void
CCSDSTmTc::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
CCSDSTmTc::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
CCSDSTmTc::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(CCSDSTmTc, "Sending retry for CPU port\n");
        sendRetryReq();
    }
}

Tick
CCSDSTmTc::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    return owner->clockPeriod();
}

void
CCSDSTmTc::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
CCSDSTmTc::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
CCSDSTmTc::CPUSidePort::recvRespRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

// MemSidePort implementation
CCSDSTmTc::MemSidePort::MemSidePort(const std::string& name, CCSDSTmTc *owner)
    : RequestPort(name, owner), owner(owner), blockedPacket(nullptr)
{
}

void
CCSDSTmTc::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
CCSDSTmTc::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
CCSDSTmTc::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
CCSDSTmTc::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// Statistics
CCSDSTmTc::TMTCStats::TMTCStats(CCSDSTmTc *parent)
    : statistics::Group(parent),
      ADD_STAT(tmFramesProcessed, statistics::units::Count::get(),
               "Number of TM frames processed"),
      ADD_STAT(tcFramesProcessed, statistics::units::Count::get(),
               "Number of TC frames processed"),
      ADD_STAT(packetsAssembled, statistics::units::Count::get(),
               "Number of space packets assembled"),
      ADD_STAT(packetsExtracted, statistics::units::Count::get(),
               "Number of space packets extracted"),
      ADD_STAT(crcErrors, statistics::units::Count::get(),
               "Number of CRC errors detected"),
      ADD_STAT(rsCorrections, statistics::units::Count::get(),
               "Number of RS error corrections"),
      ADD_STAT(syncLosses, statistics::units::Count::get(),
               "Number of sync marker losses"),
      ADD_STAT(totalCycles, statistics::units::Cycle::get(),
               "Total cycles for all operations"),
      ADD_STAT(bytesProcessed, statistics::units::Byte::get(),
               "Total bytes processed"),
      ADD_STAT(throughput, statistics::units::Rate<
               statistics::units::Byte, statistics::units::Cycle>::get(),
               "Processing throughput")
{
    throughput = bytesProcessed / totalCycles;
}

// Main constructor
CCSDSTmTc::CCSDSTmTc(const Params &params)
    : ClockedObject(params),
      regCtrl(0), regStatus(STATUS_IDLE), regMode(0),
      regScid(0), regVcid(0),
      regInputBase(0), regOutputBase(0),
      regInputLen(0), regOutputLen(0),
      regFrameCount(0),
      regTmConfig(0), regTcConfig(0),
      regRsConfig(0), regConvConfig(0),
      regCrcResult(0), regErrCount(0),
      regSyncPattern(TM_SYNC_MARKER),
      regRandSeed(0xFF),
      regAuthKey(0),
      regPerfFrames(0), regPerfCycles(0), regPerfErrors(0),
      regInterrupt(0),
      frameLatency(params.frame_latency),
      rsLatency(params.rs_latency),
      convLatency(params.conv_latency),
      cpuPort(params.name + ".cpu_side", this),
      memPort(params.name + ".mem_side", this),
      addrRange(params.addr_range),
      stats(this),
      processEvent([this]{ completeOperation(); }, name())
{
    // Initialize buffers
    inputBuffer.resize(2048);
    outputBuffer.resize(4096);  // Larger for encoded data
    rsEncoder.resize(256 * RS_SYMBOLS);
    convState.resize(64);
    
    // Initialize lookup tables
    initRsEncoder();
    initRandomizer();
    
    DPRINTF(CCSDSTmTc, "CCSDS TM/TC Accelerator created\n");
    DPRINTF(CCSDSTmTc, "  SCID: %d, Frame latency: %d cycles\n",
            regScid, frameLatency);
}

void
CCSDSTmTc::startup()
{
    DPRINTF(CCSDSTmTc, "CCSDS TM/TC Accelerator starting up\n");
}

Port &
CCSDSTmTc::getPort(const std::string &if_name, PortID idx)
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
CCSDSTmTc::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(addrRange);
    return ranges;
}

void
CCSDSTmTc::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

bool
CCSDSTmTc::handleRequest(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
        DPRINTF(CCSDSTmTc, "Read from 0x%x: 0x%x\n", offset, value);
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
        DPRINTF(CCSDSTmTc, "Write to 0x%x: 0x%x\n", offset, value);
    }
    
    pkt->makeResponse();
    cpuPort.sendPacket(pkt);
    return true;
}

bool
CCSDSTmTc::handleResponse(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void
CCSDSTmTc::handleFunctional(PacketPtr pkt)
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
CCSDSTmTc::readReg(Addr offset)
{
    switch (offset) {
        case REG_CTRL:          return regCtrl;
        case REG_STATUS:        return regStatus;
        case REG_MODE:          return regMode;
        case REG_SCID:          return regScid;
        case REG_VCID:          return regVcid;
        case REG_INPUT_BASE:    return regInputBase;
        case REG_OUTPUT_BASE:   return regOutputBase;
        case REG_INPUT_LEN:     return regInputLen;
        case REG_OUTPUT_LEN:    return regOutputLen;
        case REG_FRAME_COUNT:   return regFrameCount;
        case REG_TM_CONFIG:     return regTmConfig;
        case REG_TC_CONFIG:     return regTcConfig;
        case REG_RS_CONFIG:     return regRsConfig;
        case REG_CONV_CONFIG:   return regConvConfig;
        case REG_CRC_RESULT:    return regCrcResult;
        case REG_ERR_COUNT:     return regErrCount;
        case REG_SYNC_PATTERN:  return regSyncPattern;
        case REG_RAND_SEED:     return regRandSeed;
        case REG_AUTH_KEY_L:    return regAuthKey & 0xFFFFFFFF;
        case REG_AUTH_KEY_H:    return (regAuthKey >> 32) & 0xFFFFFFFF;
        case REG_PERF_FRAMES:   return (uint32_t)regPerfFrames;
        case REG_PERF_CYCLES:   return (uint32_t)regPerfCycles;
        case REG_PERF_ERRORS:   return (uint32_t)regPerfErrors;
        case REG_INTERRUPT:     return regInterrupt;
        case REG_VERSION:       return VERSION;
        default:
            DPRINTF(CCSDSTmTc, "Unknown register read at 0x%x\n", offset);
            return 0;
    }
}

void
CCSDSTmTc::writeReg(Addr offset, uint32_t value)
{
    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                regStatus = STATUS_IDLE;
                regPerfFrames = 0;
                regPerfCycles = 0;
                regPerfErrors = 0;
                regErrCount = 0;
                initRandomizer();
            } else if (value & CTRL_START) {
                startOperation();
            }
            break;
        case REG_MODE:          regMode = value; break;
        case REG_SCID:          regScid = value & 0x3FF; break;  // 10 bits
        case REG_VCID:          regVcid = value & 0x3F; break;   // 6 bits
        case REG_INPUT_BASE:    regInputBase = value; break;
        case REG_OUTPUT_BASE:   regOutputBase = value; break;
        case REG_INPUT_LEN:     regInputLen = value; break;
        case REG_TM_CONFIG:     regTmConfig = value; break;
        case REG_TC_CONFIG:     regTcConfig = value; break;
        case REG_RS_CONFIG:     regRsConfig = value; break;
        case REG_CONV_CONFIG:   regConvConfig = value; break;
        case REG_SYNC_PATTERN:  regSyncPattern = value; break;
        case REG_RAND_SEED:     regRandSeed = value; initRandomizer(); break;
        case REG_AUTH_KEY_L:    regAuthKey = (regAuthKey & 0xFFFFFFFF00000000ULL) | value; break;
        case REG_AUTH_KEY_H:    regAuthKey = (regAuthKey & 0xFFFFFFFF) | ((uint64_t)value << 32); break;
        case REG_INTERRUPT:     regInterrupt = value; break;
        default:
            DPRINTF(CCSDSTmTc, "Unknown register write at 0x%x: 0x%x\n",
                    offset, value);
    }
}

void
CCSDSTmTc::startOperation()
{
    if (regStatus & STATUS_BUSY) {
        DPRINTF(CCSDSTmTc, "Cannot start: already busy\n");
        return;
    }
    
    regStatus = STATUS_BUSY;
    Mode mode = static_cast<Mode>(regMode);
    
    DPRINTF(CCSDSTmTc, "Starting operation mode %d\n", regMode);
    
    switch (mode) {
        case Mode::TM_ENCODE:
            processTmEncode();
            break;
        case Mode::TM_DECODE:
            processTmDecode();
            break;
        case Mode::TC_DECODE:
            processTcDecode();
            break;
        case Mode::TC_ENCODE:
            processTcEncode();
            break;
        case Mode::PACKET_ASSEMBLE:
            assembleSpacePacket();
            break;
        case Mode::PACKET_EXTRACT:
            extractSpacePacket();
            break;
        default:
            DPRINTF(CCSDSTmTc, "Unknown mode %d\n", regMode);
            regStatus = STATUS_ERROR;
            return;
    }
}

void
CCSDSTmTc::processTmEncode()
{
    DPRINTF(CCSDSTmTc, "Processing TM Encode: %d bytes\n", regInputLen);
    
    // Simulate TM frame encoding process
    // 1. Add frame header (6 bytes)
    // 2. Add data field
    // 3. Add CRC (2 bytes)
    // 4. RS encoding (adds 32 bytes)
    // 5. Convolutional encoding (doubles size)
    // 6. Randomization
    // 7. Add sync marker (4 bytes)
    
    uint64_t cycles = 0;
    
    // CRC computation cycles
    cycles += regInputLen;
    
    // RS encoding cycles
    bool rsEnabled = (regRsConfig & 0x01);
    if (rsEnabled) {
        cycles += rsLatency;
    }
    
    // Convolutional encoding cycles
    bool convEnabled = (regConvConfig & 0x01);
    if (convEnabled) {
        cycles += convLatency;
    }
    
    // Calculate output length
    uint32_t outputLen = regInputLen + 8;  // Header + CRC
    if (rsEnabled) outputLen += RS_SYMBOLS;
    if (convEnabled) outputLen *= 2;
    outputLen += 4;  // Sync marker
    
    regOutputLen = outputLen;
    regPerfFrames++;
    regPerfCycles += cycles;
    regFrameCount++;
    
    stats.tmFramesProcessed++;
    stats.bytesProcessed += regInputLen;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSTmTc::processTmDecode()
{
    DPRINTF(CCSDSTmTc, "Processing TM Decode: %d bytes\n", regInputLen);
    
    uint64_t cycles = frameLatency;
    
    // Reverse process: sync, derandomize, Viterbi decode, RS decode, CRC check
    bool convEnabled = (regConvConfig & 0x01);
    if (convEnabled) {
        cycles += convLatency * 2;  // Viterbi is more expensive
    }
    
    bool rsEnabled = (regRsConfig & 0x01);
    if (rsEnabled) {
        cycles += rsLatency * 2;
    }
    
    // CRC verification
    cycles += regInputLen / 2;
    
    regPerfCycles += cycles;
    stats.tmFramesProcessed++;
    stats.bytesProcessed += regInputLen;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSTmTc::processTcDecode()
{
    DPRINTF(CCSDSTmTc, "Processing TC Decode: %d bytes\n", regInputLen);
    
    uint64_t cycles = frameLatency;
    
    // TC decoding: sync detection, BCH decode, CRC check, frame extraction
    cycles += regInputLen;  // Sync search
    cycles += 50;           // BCH decoding (TC uses BCH, not RS)
    cycles += regInputLen;  // CRC verification
    
    regPerfCycles += cycles;
    stats.tcFramesProcessed++;
    stats.bytesProcessed += regInputLen;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSTmTc::processTcEncode()
{
    DPRINTF(CCSDSTmTc, "Processing TC Encode: %d bytes\n", regInputLen);
    
    // TC encoding (for ground station simulation or loopback testing)
    uint64_t cycles = frameLatency;
    cycles += regInputLen;  // CRC computation
    cycles += 50;           // BCH encoding
    
    regOutputLen = regInputLen + 2 + 7;  // CRC + BCH parity
    
    regPerfCycles += cycles;
    stats.tcFramesProcessed++;
    stats.bytesProcessed += regInputLen;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSTmTc::assembleSpacePacket()
{
    DPRINTF(CCSDSTmTc, "Assembling Space Packet: %d bytes payload\n", regInputLen);
    
    // CCSDS Space Packet assembly
    // Primary header: 6 bytes
    // Secondary header: optional (depends on config)
    // Data field
    
    uint64_t cycles = 10;  // Header assembly
    cycles += regInputLen; // Data copy
    
    // Build packet header
    uint16_t header0 = 0;
    header0 |= (0 << 13);           // Version (always 0)
    header0 |= (1 << 12);           // Type (1=telemetry)
    header0 |= (1 << 11);           // Secondary header flag
    header0 |= (regScid & 0x7FF);   // APID
    
    uint16_t header1 = 0;
    header1 |= (3 << 14);           // Sequence flags (11=unsegmented)
    header1 |= (regFrameCount & 0x3FFF);  // Sequence count
    
    uint16_t header2 = regInputLen - 1;  // Data length - 1
    
    regOutputLen = 6 + regInputLen;  // Header + payload
    regFrameCount++;
    
    regPerfCycles += cycles;
    stats.packetsAssembled++;
    stats.bytesProcessed += regInputLen;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSTmTc::extractSpacePacket()
{
    DPRINTF(CCSDSTmTc, "Extracting Space Packet from %d bytes\n", regInputLen);
    
    uint64_t cycles = regInputLen;  // Parse through data
    
    // Validate packet structure
    // Extract header fields
    // Output data field
    
    if (regInputLen < 7) {
        regStatus = STATUS_ERROR;
        DPRINTF(CCSDSTmTc, "Input too short for space packet\n");
        return;
    }
    
    regOutputLen = regInputLen - 6;  // Remove header
    
    regPerfCycles += cycles;
    stats.packetsExtracted++;
    stats.bytesProcessed += regInputLen;
    stats.totalCycles += cycles;
    
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(cycles)));
}

void
CCSDSTmTc::completeOperation()
{
    DPRINTF(CCSDSTmTc, "Operation complete\n");
    
    regStatus = STATUS_DONE;
    regCtrl &= ~CTRL_START;
    
    // Generate interrupt if enabled
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x01;
    }
}

void
CCSDSTmTc::rsEncode(std::vector<uint8_t>& data)
{
    // Reed-Solomon RS(255,223) encoding
    // Adds 32 check symbols
    DPRINTF(CCSDSTmTc, "RS encoding %zu bytes\n", data.size());
    
    // Simplified: append placeholder parity bytes
    // In real HW, this would use GF(2^8) polynomial arithmetic
    size_t originalSize = data.size();
    data.resize(originalSize + RS_SYMBOLS);
    
    // Generate check symbols (simplified XOR-based)
    for (unsigned i = 0; i < RS_SYMBOLS; i++) {
        uint8_t check = 0;
        for (size_t j = 0; j < originalSize; j++) {
            check ^= data[j] ^ rsEncoder[i * 256 + data[j]];
        }
        data[originalSize + i] = check;
    }
}

bool
CCSDSTmTc::rsDecode(std::vector<uint8_t>& data, unsigned& corrections)
{
    // Simplified RS decoding
    // Returns true if successful, corrections contains number of fixes
    corrections = 0;
    
    if (data.size() < RS_SYMBOLS) {
        return false;
    }
    
    // In real implementation, this would use Berlekamp-Massey algorithm
    // Can correct up to 16 symbol errors
    
    // Remove check symbols from output
    data.resize(data.size() - RS_SYMBOLS);
    return true;
}

void
CCSDSTmTc::convEncode(const std::vector<uint8_t>& input, 
                       std::vector<uint8_t>& output)
{
    // Convolutional encoding: rate 1/2, K=7
    // Each input bit produces 2 output bits
    DPRINTF(CCSDSTmTc, "Convolutional encoding %zu bytes\n", input.size());
    
    output.resize(input.size() * 2);
    
    uint8_t state = 0;  // 6-bit shift register state
    size_t outIdx = 0;
    
    for (size_t i = 0; i < input.size(); i++) {
        for (int bit = 7; bit >= 0; bit--) {
            uint8_t inBit = (input[i] >> bit) & 1;
            state = ((state << 1) | inBit) & 0x7F;
            
            // G1 = 1111001 (0x79), G2 = 1011011 (0x5B)
            uint8_t g1 = __builtin_popcount(state & 0x79) & 1;
            uint8_t g2 = __builtin_popcount(state & 0x5B) & 1;
            
            size_t byteIdx = outIdx / 8;
            int bitPos = 7 - (outIdx % 8);
            
            if (byteIdx < output.size()) {
                output[byteIdx] |= (g1 << bitPos);
                if (bitPos > 0) {
                    output[byteIdx] |= (g2 << (bitPos - 1));
                } else if (byteIdx + 1 < output.size()) {
                    output[byteIdx + 1] |= (g2 << 7);
                }
            }
            outIdx += 2;
        }
    }
}

void
CCSDSTmTc::randomize(std::vector<uint8_t>& data)
{
    // CCSDS pseudorandom sequence XOR
    DPRINTF(CCSDSTmTc, "Randomizing %zu bytes\n", data.size());
    
    for (size_t i = 0; i < data.size(); i++) {
        data[i] ^= randRegister[i % 32];
        
        // Update LFSR for next byte
        // Polynomial: x^8 + x^7 + x^5 + x^3 + 1
        if ((i % 32) == 31) {
            uint8_t feedback = randRegister[0];
            for (int j = 0; j < 31; j++) {
                randRegister[j] = randRegister[j + 1];
            }
            randRegister[31] = feedback ^ randRegister[6] ^ randRegister[4] ^ randRegister[2];
        }
    }
}

void
CCSDSTmTc::derandomize(std::vector<uint8_t>& data)
{
    // Derandomization is the same as randomization (XOR is self-inverse)
    initRandomizer();  // Reset to known state
    randomize(data);
}

uint16_t
CCSDSTmTc::computeCrc16(const std::vector<uint8_t>& data)
{
    // CRC-16-CCITT
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < data.size(); i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc ^ 0xFFFF;
}

bool
CCSDSTmTc::verifyCrc16(const std::vector<uint8_t>& data, uint16_t expected)
{
    uint16_t computed = computeCrc16(data);
    
    if (computed != expected) {
        regErrCount++;
        regPerfErrors++;
        stats.crcErrors++;
        return false;
    }
    return true;
}

void
CCSDSTmTc::initRsEncoder()
{
    // Initialize RS encoder lookup table
    // Using GF(2^8) with primitive polynomial x^8 + x^4 + x^3 + x^2 + 1
    
    for (unsigned i = 0; i < RS_SYMBOLS; i++) {
        for (unsigned j = 0; j < 256; j++) {
            // Simplified: use basic multiplication
            rsEncoder[i * 256 + j] = (uint8_t)((i * j) ^ (j >> 1));
        }
    }
}

void
CCSDSTmTc::initRandomizer()
{
    // Initialize CCSDS randomizer sequence
    // LFSR initialized with seed
    uint8_t seed = regRandSeed;
    
    for (int i = 0; i < 32; i++) {
        randRegister[i] = seed;
        // Simple LFSR step
        seed = (seed << 1) | ((seed >> 7) ^ ((seed >> 5) & 1));
    }
}

bool
CCSDSTmTc::syncSearch(const std::vector<uint8_t>& data, uint32_t pattern)
{
    // Search for sync marker in data
    if (data.size() < 4) return false;
    
    for (size_t i = 0; i <= data.size() - 4; i++) {
        uint32_t word = ((uint32_t)data[i] << 24) |
                        ((uint32_t)data[i+1] << 16) |
                        ((uint32_t)data[i+2] << 8) |
                        data[i+3];
        if (word == pattern) {
            return true;
        }
    }
    
    stats.syncLosses++;
    return false;
}

} // namespace gem5



