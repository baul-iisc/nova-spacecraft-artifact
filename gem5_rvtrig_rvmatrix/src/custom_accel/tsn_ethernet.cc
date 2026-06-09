/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * TSN Ethernet Controller Implementation
 */

#include "custom_accel/tsn_ethernet.hh"

#include "base/trace.hh"
#include "debug/TSNEthernet.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

// CPUSidePort implementation
TSNEthernet::CPUSidePort::CPUSidePort(const std::string& name, 
                                       TSNEthernet *owner)
    : ResponsePort(name, owner), owner(owner), needRetry(false),
      blockedPacket(nullptr)
{
}

void
TSNEthernet::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
TSNEthernet::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
TSNEthernet::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(TSNEthernet, "Sending retry for CPU port\n");
        sendRetryReq();
    }
}

Tick
TSNEthernet::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    return owner->clockPeriod();
}

void
TSNEthernet::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
TSNEthernet::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
TSNEthernet::CPUSidePort::recvRespRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

// MemSidePort implementation
TSNEthernet::MemSidePort::MemSidePort(const std::string& name, 
                                       TSNEthernet *owner)
    : RequestPort(name, owner), owner(owner), blockedPacket(nullptr)
{
}

void
TSNEthernet::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
TSNEthernet::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
TSNEthernet::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
TSNEthernet::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// Statistics
TSNEthernet::TsnStats::TsnStats(TSNEthernet *parent)
    : statistics::Group(parent),
      ADD_STAT(txFrames, statistics::units::Count::get(),
               "Number of frames transmitted"),
      ADD_STAT(rxFrames, statistics::units::Count::get(),
               "Number of frames received"),
      ADD_STAT(txBytes, statistics::units::Byte::get(),
               "Total bytes transmitted"),
      ADD_STAT(rxBytes, statistics::units::Byte::get(),
               "Total bytes received"),
      ADD_STAT(crcErrors, statistics::units::Count::get(),
               "Number of CRC errors"),
      ADD_STAT(gateDrops, statistics::units::Count::get(),
               "Frames dropped by Time-Aware Shaper"),
      ADD_STAT(psfpDrops, statistics::units::Count::get(),
               "Frames dropped by PSFP filtering"),
      ADD_STAT(frerDuplicates, statistics::units::Count::get(),
               "Duplicate frames eliminated by FRER"),
      ADD_STAT(ptpSyncEvents, statistics::units::Count::get(),
               "Number of PTP sync events"),
      ADD_STAT(ptpOffsetNs, statistics::units::Second::get(),
               "Current PTP clock offset"),
      ADD_STAT(totalCycles, statistics::units::Cycle::get(),
               "Total cycles for all operations"),
      ADD_STAT(throughput, statistics::units::Rate<
               statistics::units::Byte, statistics::units::Cycle>::get(),
               "Network throughput (bytes per cycle)")
{
    throughput = (txBytes + rxBytes) / totalCycles;
}

// Main constructor
TSNEthernet::TSNEthernet(const Params &params)
    : ClockedObject(params),
      regCtrl(0), regStatus(STATUS_TX_IDLE | STATUS_LINK_UP),
      regMacAddr(params.mac_address),
      regTxBase(0), regTxLen(0), regTxDescBase(0), regTxDescTail(0),
      regRxBase(0), regRxLen(0), regRxDescBase(0), regRxDescHead(0),
      regLinkSpeed(params.link_speed), regLinkStatus(STATUS_LINK_UP),
      regPtpCtrl(0), regPtpStatus(0), regPtpTime(0),
      regPtpOffset(0), regPtpRateAdj(0),
      regPtpTxTs(0), regPtpRxTs(0),
      regTasCtrl(0), regTasStatus(0),
      regTasCycleTime(1000000),  // Default 1ms cycle
      regTasBaseTime(0), regTasGclLen(0), regTasGclBase(0),
      regFrerCtrl(0), regFrerStatus(0), regFrerStreams(0), regFrerStreamBase(0),
      regPsfpCtrl(0), regPsfpStatus(0), regPsfpFilterCnt(0),
      regPerfTxFrames(0), regPerfRxFrames(0),
      regPerfTxBytes(0), regPerfRxBytes(0), regPerfErrors(0),
      regInterrupt(0),
      linkSpeedMbps(params.link_speed),
      txFifoDepth(params.tx_fifo_depth),
      rxFifoDepth(params.rx_fifo_depth),
      maxFrameSize(params.max_frame_size),
      gclMaxEntries(params.gcl_max_entries),
      currentGclIndex(0), currentGateStates(0xFF),
      cpuPort(params.name + ".cpu_side", this),
      memPort(params.name + ".mem_side", this),
      addrRange(params.addr_range),
      stats(this),
      txEvent([this]{ processTx(); }, name()),
      rxEvent([this]{ processRx(); }, name()),
      ptpEvent([this]{ ptpProcess(); }, name()),
      tasEvent([this]{ tasProcess(); }, name())
{
    // Initialize TAS queues
    regTasQueue.fill(0xFF);  // All gates open initially
    
    DPRINTF(TSNEthernet, "TSN Ethernet Controller created\n");
    DPRINTF(TSNEthernet, "  MAC: %012lx\n", regMacAddr);
    DPRINTF(TSNEthernet, "  Link speed: %d Mbps\n", linkSpeedMbps);
    DPRINTF(TSNEthernet, "  GCL max entries: %d\n", gclMaxEntries);
}

void
TSNEthernet::startup()
{
    DPRINTF(TSNEthernet, "TSN Ethernet starting up\n");
    
    // Start PTP time keeping
    if (!ptpEvent.scheduled()) {
        schedule(ptpEvent, curTick() + cyclesToTicks(Cycles(1000)));
    }
}

Port &
TSNEthernet::getPort(const std::string &if_name, PortID idx)
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
TSNEthernet::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(addrRange);
    return ranges;
}

void
TSNEthernet::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

bool
TSNEthernet::handleRequest(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
        DPRINTF(TSNEthernet, "Read from 0x%x: 0x%x\n", offset, value);
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
        DPRINTF(TSNEthernet, "Write to 0x%x: 0x%x\n", offset, value);
    }
    
    pkt->makeResponse();
    cpuPort.sendPacket(pkt);
    return true;
}

bool
TSNEthernet::handleResponse(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void
TSNEthernet::handleFunctional(PacketPtr pkt)
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
TSNEthernet::readReg(Addr offset)
{
    // Basic registers
    if (offset < 0x100) {
        switch (offset) {
            case REG_CTRL:         return regCtrl;
            case REG_STATUS:       return regStatus;
            case REG_MAC_ADDR_L:   return regMacAddr & 0xFFFFFFFF;
            case REG_MAC_ADDR_H:   return (regMacAddr >> 32) & 0xFFFF;
            case REG_TX_BASE:      return regTxBase;
            case REG_TX_LEN:       return regTxLen;
            case REG_TX_DESC_BASE: return regTxDescBase;
            case REG_TX_DESC_TAIL: return regTxDescTail;
            case REG_RX_BASE:      return regRxBase;
            case REG_RX_LEN:       return regRxLen;
            case REG_RX_DESC_BASE: return regRxDescBase;
            case REG_RX_DESC_HEAD: return regRxDescHead;
            case REG_LINK_SPEED:   return regLinkSpeed;
            case REG_LINK_STATUS:  return regLinkStatus;
            default: break;
        }
    }
    // PTP registers
    else if (offset >= 0x100 && offset < 0x200) {
        switch (offset) {
            case REG_PTP_CTRL:     return regPtpCtrl;
            case REG_PTP_STATUS:   return regPtpStatus;
            case REG_PTP_TIME_L:   return regPtpTime & 0xFFFFFFFF;
            case REG_PTP_TIME_H:   return (regPtpTime >> 32) & 0xFFFFFFFF;
            case REG_PTP_OFFSET_L: return regPtpOffset & 0xFFFFFFFF;
            case REG_PTP_OFFSET_H: return (regPtpOffset >> 32) & 0xFFFFFFFF;
            case REG_PTP_RATE_ADJ: return regPtpRateAdj;
            case REG_PTP_TX_TS_L:  return regPtpTxTs & 0xFFFFFFFF;
            case REG_PTP_TX_TS_H:  return (regPtpTxTs >> 32) & 0xFFFFFFFF;
            case REG_PTP_RX_TS_L:  return regPtpRxTs & 0xFFFFFFFF;
            case REG_PTP_RX_TS_H:  return (regPtpRxTs >> 32) & 0xFFFFFFFF;
            default: break;
        }
    }
    // TAS registers
    else if (offset >= 0x200 && offset < 0x300) {
        switch (offset) {
            case REG_TAS_CTRL:       return regTasCtrl;
            case REG_TAS_STATUS:     return regTasStatus;
            case REG_TAS_CYCLE_TIME: return regTasCycleTime;
            case REG_TAS_BASE_TIME_L:return regTasBaseTime & 0xFFFFFFFF;
            case REG_TAS_BASE_TIME_H:return (regTasBaseTime >> 32) & 0xFFFFFFFF;
            case REG_TAS_GCL_LEN:    return regTasGclLen;
            case REG_TAS_GCL_BASE:   return regTasGclBase;
            default:
                // Queue registers
                if (offset >= REG_TAS_QUEUE_BASE && 
                    offset < REG_TAS_QUEUE_BASE + NUM_TRAFFIC_CLASSES * 4) {
                    unsigned idx = (offset - REG_TAS_QUEUE_BASE) / 4;
                    return regTasQueue[idx];
                }
                break;
        }
    }
    // FRER registers
    else if (offset >= 0x300 && offset < 0x400) {
        switch (offset) {
            case REG_FRER_CTRL:        return regFrerCtrl;
            case REG_FRER_STATUS:      return regFrerStatus;
            case REG_FRER_STREAMS:     return regFrerStreams;
            case REG_FRER_STREAM_BASE: return regFrerStreamBase;
            default: break;
        }
    }
    // PSFP registers
    else if (offset >= 0x400 && offset < 0x500) {
        switch (offset) {
            case REG_PSFP_CTRL:       return regPsfpCtrl;
            case REG_PSFP_STATUS:     return regPsfpStatus;
            case REG_PSFP_FILTER_CNT: return regPsfpFilterCnt;
            default: break;
        }
    }
    // Performance registers
    else if (offset >= 0x500) {
        switch (offset) {
            case REG_PERF_TX_FRAMES: return (uint32_t)regPerfTxFrames;
            case REG_PERF_RX_FRAMES: return (uint32_t)regPerfRxFrames;
            case REG_PERF_TX_BYTES:  return (uint32_t)regPerfTxBytes;
            case REG_PERF_RX_BYTES:  return (uint32_t)regPerfRxBytes;
            case REG_PERF_ERRORS:    return (uint32_t)regPerfErrors;
            case REG_INTERRUPT:      return regInterrupt;
            case REG_VERSION:        return VERSION;
            default: break;
        }
    }
    
    DPRINTF(TSNEthernet, "Unknown register read at 0x%x\n", offset);
    return 0;
}

void
TSNEthernet::writeReg(Addr offset, uint32_t value)
{
    // Basic registers
    if (offset < 0x100) {
        switch (offset) {
            case REG_CTRL:
                regCtrl = value;
                if (value & CTRL_RESET) {
                    regStatus = STATUS_TX_IDLE | STATUS_LINK_UP;
                    regPerfTxFrames = 0;
                    regPerfRxFrames = 0;
                    regPerfTxBytes = 0;
                    regPerfRxBytes = 0;
                }
                if ((value & CTRL_TX_EN) && regTxLen > 0) {
                    startTx();
                }
                break;
            case REG_MAC_ADDR_L:
                regMacAddr = (regMacAddr & 0xFFFF00000000ULL) | value;
                break;
            case REG_MAC_ADDR_H:
                regMacAddr = (regMacAddr & 0xFFFFFFFF) | ((uint64_t)(value & 0xFFFF) << 32);
                break;
            case REG_TX_BASE:      regTxBase = value; break;
            case REG_TX_LEN:       regTxLen = value; break;
            case REG_TX_DESC_BASE: regTxDescBase = value; break;
            case REG_TX_DESC_TAIL: regTxDescTail = value; break;
            case REG_RX_BASE:      regRxBase = value; break;
            case REG_RX_DESC_BASE: regRxDescBase = value; break;
            default: break;
        }
    }
    // PTP registers
    else if (offset >= 0x100 && offset < 0x200) {
        switch (offset) {
            case REG_PTP_CTRL:     regPtpCtrl = value; break;
            case REG_PTP_OFFSET_L: regPtpOffset = (regPtpOffset & 0xFFFFFFFF00000000LL) | value; break;
            case REG_PTP_OFFSET_H: regPtpOffset = (regPtpOffset & 0xFFFFFFFF) | ((int64_t)value << 32); break;
            case REG_PTP_RATE_ADJ: regPtpRateAdj = value; break;
            default: break;
        }
    }
    // TAS registers
    else if (offset >= 0x200 && offset < 0x300) {
        switch (offset) {
            case REG_TAS_CTRL:
                regTasCtrl = value;
                if (value & 0x01) {
                    loadGcl();
                    if (!tasEvent.scheduled()) {
                        schedule(tasEvent, curTick() + cyclesToTicks(Cycles(100)));
                    }
                }
                break;
            case REG_TAS_CYCLE_TIME:  regTasCycleTime = value; break;
            case REG_TAS_BASE_TIME_L: regTasBaseTime = (regTasBaseTime & 0xFFFFFFFF00000000ULL) | value; break;
            case REG_TAS_BASE_TIME_H: regTasBaseTime = (regTasBaseTime & 0xFFFFFFFF) | ((uint64_t)value << 32); break;
            case REG_TAS_GCL_LEN:     regTasGclLen = value; break;
            case REG_TAS_GCL_BASE:    regTasGclBase = value; break;
            default: break;
        }
    }
    // FRER registers
    else if (offset >= 0x300 && offset < 0x400) {
        switch (offset) {
            case REG_FRER_CTRL:        regFrerCtrl = value; break;
            case REG_FRER_STREAMS:     regFrerStreams = value; break;
            case REG_FRER_STREAM_BASE: regFrerStreamBase = value; break;
            default: break;
        }
    }
    // PSFP registers
    else if (offset >= 0x400 && offset < 0x500) {
        switch (offset) {
            case REG_PSFP_CTRL:       regPsfpCtrl = value; break;
            case REG_PSFP_FILTER_CNT: regPsfpFilterCnt = value; break;
            default: break;
        }
    }
    // Performance registers
    else if (offset >= 0x500) {
        switch (offset) {
            case REG_INTERRUPT: regInterrupt = value; break;
            default: break;
        }
    }
    else {
        DPRINTF(TSNEthernet, "Unknown register write at 0x%x: 0x%x\n",
                offset, value);
    }
}

void
TSNEthernet::startTx()
{
    if (!(regStatus & STATUS_TX_IDLE)) {
        DPRINTF(TSNEthernet, "TX already active\n");
        return;
    }
    
    regStatus &= ~STATUS_TX_IDLE;
    
    DPRINTF(TSNEthernet, "Starting TX: %d bytes\n", regTxLen);
    
    // Capture TX timestamp if PTP enabled
    if (regPtpCtrl & 0x01) {
        ptpCaptureTimestamp(true);
    }
    
    // Calculate TX time based on link speed
    // Time in ns = (bytes * 8) / (speed in Mbps) * 1000
    unsigned txTimeNs = (regTxLen * 8 * 1000) / linkSpeedMbps;
    unsigned txCycles = txTimeNs / 10;  // Assuming 100MHz clock (10ns period)
    
    schedule(txEvent, curTick() + cyclesToTicks(Cycles(txCycles)));
}

void
TSNEthernet::processTx()
{
    DPRINTF(TSNEthernet, "TX complete\n");
    
    regPerfTxFrames++;
    regPerfTxBytes += regTxLen;
    stats.txFrames++;
    stats.txBytes += regTxLen;
    
    completeTx();
}

void
TSNEthernet::completeTx()
{
    regStatus |= STATUS_TX_IDLE;
    
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x01;
    }
}

void
TSNEthernet::processRx()
{
    if (rxQueue.empty()) return;
    
    auto& frame = rxQueue.front();
    
    // Apply PSFP filtering
    if ((regPsfpCtrl & 0x01) && !psfpFilterFrame(frame)) {
        DPRINTF(TSNEthernet, "Frame dropped by PSFP\n");
        stats.psfpDrops++;
        rxQueue.pop();
        return;
    }
    
    // Check for FRER duplicates
    if ((regFrerCtrl & 0x01) && frerCheckDuplicate(frame)) {
        DPRINTF(TSNEthernet, "Duplicate frame eliminated by FRER\n");
        stats.frerDuplicates++;
        rxQueue.pop();
        return;
    }
    
    // Capture RX timestamp
    if (regPtpCtrl & 0x01) {
        ptpCaptureTimestamp(false);
    }
    
    regRxLen = frame.size();
    regPerfRxFrames++;
    regPerfRxBytes += frame.size();
    stats.rxFrames++;
    stats.rxBytes += frame.size();
    
    rxQueue.pop();
    completeRx();
}

void
TSNEthernet::completeRx()
{
    regStatus |= STATUS_RX_READY;
    
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x02;
    }
    
    DPRINTF(TSNEthernet, "RX complete: %d bytes\n", regRxLen);
}

void
TSNEthernet::externalRx(const std::vector<uint8_t>& frame)
{
    if (rxQueue.size() >= rxFifoDepth) {
        DPRINTF(TSNEthernet, "RX FIFO full, dropping frame\n");
        regPerfErrors++;
        return;
    }
    
    rxQueue.push(frame);
    
    if (!rxEvent.scheduled()) {
        schedule(rxEvent, curTick() + cyclesToTicks(Cycles(10)));
    }
}

void
TSNEthernet::ptpProcess()
{
    ptpUpdateTime();
    
    // Reschedule for next update
    schedule(ptpEvent, curTick() + cyclesToTicks(Cycles(1000)));
}

void
TSNEthernet::ptpUpdateTime()
{
    // Increment PTP time by elapsed cycles (in nanoseconds)
    // Assuming 100MHz clock = 10ns per cycle, 1000 cycles = 10000ns = 10us
    uint64_t incrementNs = 10000;
    
    // Apply rate adjustment (in parts per billion)
    if (regPtpRateAdj != 0) {
        int64_t adjustment = (int64_t)incrementNs * regPtpRateAdj / 1000000000LL;
        incrementNs += adjustment;
    }
    
    regPtpTime += incrementNs;
}

void
TSNEthernet::ptpCaptureTimestamp(bool isTx)
{
    uint64_t ts = getPtpTime();
    
    if (isTx) {
        regPtpTxTs = ts;
        DPRINTF(TSNEthernet, "TX timestamp captured: %lu ns\n", ts);
    } else {
        regPtpRxTs = ts;
        DPRINTF(TSNEthernet, "RX timestamp captured: %lu ns\n", ts);
    }
}

uint64_t
TSNEthernet::getPtpTime()
{
    return regPtpTime + regPtpOffset;
}

void
TSNEthernet::tasProcess()
{
    if (!(regTasCtrl & 0x01)) {
        return;  // TAS disabled
    }
    
    tasUpdateGates();
    
    // Calculate time to next GCL entry
    unsigned nextInterval = 1000;  // Default 1us
    if (currentGclIndex < gateControlList.size()) {
        nextInterval = gateControlList[currentGclIndex].timeInterval / 10;  // Convert ns to cycles
    }
    
    if (nextInterval > 0) {
        schedule(tasEvent, curTick() + cyclesToTicks(Cycles(nextInterval)));
    }
}

void
TSNEthernet::tasUpdateGates()
{
    if (gateControlList.empty()) {
        currentGateStates = 0xFF;  // All open
        return;
    }
    
    currentGclIndex = (currentGclIndex + 1) % gateControlList.size();
    currentGateStates = gateControlList[currentGclIndex].gateStates;
    
    // Update queue gate status
    for (unsigned i = 0; i < NUM_TRAFFIC_CLASSES; i++) {
        regTasQueue[i] = (currentGateStates >> i) & 1;
    }
    
    regTasStatus = (regTasStatus & 0xFFFFFF00) | currentGclIndex;
    
    DPRINTF(TSNEthernet, "TAS update: GCL[%d] = 0x%02x\n",
            currentGclIndex, currentGateStates);
}

bool
TSNEthernet::tasCheckGate(unsigned trafficClass)
{
    if (!(regTasCtrl & 0x01)) {
        return true;  // TAS disabled, all gates open
    }
    
    if (trafficClass >= NUM_TRAFFIC_CLASSES) {
        return false;
    }
    
    bool open = (currentGateStates >> trafficClass) & 1;
    if (!open) {
        stats.gateDrops++;
    }
    return open;
}

void
TSNEthernet::loadGcl()
{
    gateControlList.clear();
    
    // In real implementation, would read from memory at regTasGclBase
    // For simulation, create default GCL
    for (unsigned i = 0; i < regTasGclLen && i < gclMaxEntries; i++) {
        GclEntry entry;
        entry.gateStates = 0xFF;  // All gates open
        entry.timeInterval = regTasCycleTime / regTasGclLen;
        gateControlList.push_back(entry);
    }
    
    currentGclIndex = 0;
    regTasStatus |= 0x01;  // GCL loaded
    
    DPRINTF(TSNEthernet, "GCL loaded: %zu entries\n", gateControlList.size());
}

bool
TSNEthernet::frerCheckDuplicate(const std::vector<uint8_t>& frame)
{
    // Simplified duplicate detection
    // In real implementation, would check sequence numbers per stream
    return false;  // No duplicate
}

void
TSNEthernet::frerReplicate(std::vector<uint8_t>& frame)
{
    // Frame replication for redundancy
    // Would be handled by the network fabric in real implementation
}

bool
TSNEthernet::psfpFilterFrame(const std::vector<uint8_t>& frame)
{
    // Simplified stream filtering
    // In real implementation, would check against configured filters
    
    if (frame.size() < 14) {
        return false;  // Too short
    }
    
    // Check against max frame size
    if (frame.size() > maxFrameSize) {
        return false;
    }
    
    return true;  // Pass
}

bool
TSNEthernet::psfpCheckRate(unsigned streamId)
{
    // Rate limiting per stream
    // Simplified: always pass
    return true;
}

unsigned
TSNEthernet::selectQueue(const std::vector<uint8_t>& frame)
{
    // Extract priority from VLAN tag or use default
    if (frame.size() >= 16 && frame[12] == 0x81 && frame[13] == 0x00) {
        // VLAN tagged frame
        unsigned pcp = (frame[14] >> 5) & 0x07;
        return pcp;
    }
    
    return 0;  // Default to lowest priority
}

uint32_t
TSNEthernet::computeCrc32(const uint8_t* data, unsigned length)
{
    // Ethernet CRC-32
    uint32_t crc = 0xFFFFFFFF;
    
    for (unsigned i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    
    return ~crc;
}

} // namespace gem5



