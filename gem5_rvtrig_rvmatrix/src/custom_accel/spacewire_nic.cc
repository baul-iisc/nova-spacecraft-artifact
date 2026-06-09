/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * SpaceWire Network Interface Controller Implementation
 */

#include "custom_accel/spacewire_nic.hh"

#include "base/trace.hh"
#include "debug/SpaceWireNIC.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

// CPUSidePort implementation
SpaceWireNIC::CPUSidePort::CPUSidePort(const std::string& name, 
                                        SpaceWireNIC *owner)
    : ResponsePort(name, owner), owner(owner), needRetry(false),
      blockedPacket(nullptr)
{
}

void
SpaceWireNIC::CPUSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingResp(pkt)) {
        blockedPacket = pkt;
    }
}

AddrRangeList
SpaceWireNIC::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
SpaceWireNIC::CPUSidePort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(SpaceWireNIC, "Sending retry for CPU port\n");
        sendRetryReq();
    }
}

Tick
SpaceWireNIC::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    return owner->clockPeriod();
}

void
SpaceWireNIC::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
SpaceWireNIC::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
SpaceWireNIC::CPUSidePort::recvRespRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

// MemSidePort implementation
SpaceWireNIC::MemSidePort::MemSidePort(const std::string& name, 
                                        SpaceWireNIC *owner)
    : RequestPort(name, owner), owner(owner), blockedPacket(nullptr)
{
}

void
SpaceWireNIC::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send when blocked");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

bool
SpaceWireNIC::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt);
}

void
SpaceWireNIC::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr, "Got retry but no blocked packet");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
SpaceWireNIC::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// Statistics
SpaceWireNIC::SpwStats::SpwStats(SpaceWireNIC *parent)
    : statistics::Group(parent),
      ADD_STAT(txPackets, statistics::units::Count::get(),
               "Number of packets transmitted"),
      ADD_STAT(rxPackets, statistics::units::Count::get(),
               "Number of packets received"),
      ADD_STAT(txBytes, statistics::units::Byte::get(),
               "Total bytes transmitted"),
      ADD_STAT(rxBytes, statistics::units::Byte::get(),
               "Total bytes received"),
      ADD_STAT(parityErrors, statistics::units::Count::get(),
               "Number of parity errors"),
      ADD_STAT(disconnects, statistics::units::Count::get(),
               "Number of link disconnects"),
      ADD_STAT(escapeErrors, statistics::units::Count::get(),
               "Number of escape sequence errors"),
      ADD_STAT(creditErrors, statistics::units::Count::get(),
               "Number of credit/flow control errors"),
      ADD_STAT(rmapCommands, statistics::units::Count::get(),
               "Number of RMAP commands processed"),
      ADD_STAT(timeCodes, statistics::units::Count::get(),
               "Number of time-codes processed"),
      ADD_STAT(totalCycles, statistics::units::Cycle::get(),
               "Total cycles for all operations"),
      ADD_STAT(avgPacketSize, statistics::units::Byte::get(),
               "Average packet size"),
      ADD_STAT(throughput, statistics::units::Rate<
               statistics::units::Byte, statistics::units::Cycle>::get(),
               "Link throughput (bytes per cycle)")
{
    avgPacketSize = (txBytes + rxBytes) / (txPackets + rxPackets);
    throughput = (txBytes + rxBytes) / totalCycles;
}

// Main constructor
SpaceWireNIC::SpaceWireNIC(const Params &params)
    : ClockedObject(params),
      regCtrl(0), regStatus(STATUS_TX_IDLE),
      regLinkCtrl(0), regLinkStatus(0),
      regTxBase(0), regTxLen(0), regTxDesc(0),
      regRxBase(0), regRxLen(0), regRxDesc(0),
      regNodeAddr(params.node_address),
      regTimeCode(0),
      regCreditCnt(8),  // Default 8 credits
      regErrCnt(0), regDisconnectCnt(0),
      regRmapCtrl(0), regRmapKey(params.rmap_key), regRmapStatus(0),
      regDmaCtrl(0), regDmaStatus(0),
      regPerfTxPkt(0), regPerfRxPkt(0),
      regPerfTxBytes(0), regPerfRxBytes(0),
      regInterrupt(0),
      linkSpeed(params.link_speed),
      txFifoDepth(params.tx_fifo_depth),
      rxFifoDepth(params.rx_fifo_depth),
      maxPacketSize(params.max_packet_size),
      linkState(LinkState::ERROR_RESET),
      creditCount(8),
      cpuPort(params.name + ".cpu_side", this),
      memPort(params.name + ".mem_side", this),
      addrRange(params.addr_range),
      stats(this),
      txEvent([this]{ processTx(); }, name()),
      rxEvent([this]{ processRx(); }, name()),
      linkEvent([this]{ linkStateMachine(); }, name())
{
    DPRINTF(SpaceWireNIC, "SpaceWire NIC created\n");
    DPRINTF(SpaceWireNIC, "  Node address: %d\n", regNodeAddr);
    DPRINTF(SpaceWireNIC, "  Link speed: %d Mbps\n", linkSpeed);
    DPRINTF(SpaceWireNIC, "  TX FIFO: %d, RX FIFO: %d\n", 
            txFifoDepth, rxFifoDepth);
}

void
SpaceWireNIC::startup()
{
    DPRINTF(SpaceWireNIC, "SpaceWire NIC starting up\n");
    linkState = LinkState::ERROR_RESET;
}

Port &
SpaceWireNIC::getPort(const std::string &if_name, PortID idx)
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
SpaceWireNIC::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(addrRange);
    return ranges;
}

void
SpaceWireNIC::sendRangeChange()
{
    cpuPort.sendRangeChange();
}

bool
SpaceWireNIC::handleRequest(PacketPtr pkt)
{
    if (pkt->isRead()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = readReg(offset);
        pkt->setLE<uint32_t>(value);
        DPRINTF(SpaceWireNIC, "Read from 0x%x: 0x%x\n", offset, value);
    } else if (pkt->isWrite()) {
        Addr offset = pkt->getAddr() - addrRange.start();
        uint32_t value = pkt->getLE<uint32_t>();
        writeReg(offset, value);
        DPRINTF(SpaceWireNIC, "Write to 0x%x: 0x%x\n", offset, value);
    }
    
    pkt->makeResponse();
    cpuPort.sendPacket(pkt);
    return true;
}

bool
SpaceWireNIC::handleResponse(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void
SpaceWireNIC::handleFunctional(PacketPtr pkt)
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
SpaceWireNIC::readReg(Addr offset)
{
    switch (offset) {
        case REG_CTRL:           return regCtrl;
        case REG_STATUS:         return regStatus;
        case REG_LINK_CTRL:      return regLinkCtrl;
        case REG_LINK_STATUS:    return regLinkStatus;
        case REG_TX_BASE:        return regTxBase;
        case REG_TX_LEN:         return regTxLen;
        case REG_TX_DESC:        return regTxDesc;
        case REG_RX_BASE:        return regRxBase;
        case REG_RX_LEN:         return regRxLen;
        case REG_RX_DESC:        return regRxDesc;
        case REG_NODE_ADDR:      return regNodeAddr;
        case REG_TIME_CODE:      return regTimeCode;
        case REG_CREDIT_CNT:     return regCreditCnt;
        case REG_ERR_CNT:        return regErrCnt;
        case REG_DISCONNECT_CNT: return regDisconnectCnt;
        case REG_RMAP_CTRL:      return regRmapCtrl;
        case REG_RMAP_KEY:       return regRmapKey;
        case REG_RMAP_STATUS:    return regRmapStatus;
        case REG_DMA_CTRL:       return regDmaCtrl;
        case REG_DMA_STATUS:     return regDmaStatus;
        case REG_PERF_TX_PKT:    return (uint32_t)regPerfTxPkt;
        case REG_PERF_RX_PKT:    return (uint32_t)regPerfRxPkt;
        case REG_PERF_TX_BYTES:  return (uint32_t)regPerfTxBytes;
        case REG_PERF_RX_BYTES:  return (uint32_t)regPerfRxBytes;
        case REG_INTERRUPT:      return regInterrupt;
        case REG_VERSION:        return VERSION;
        default:
            DPRINTF(SpaceWireNIC, "Unknown register read at 0x%x\n", offset);
            return 0;
    }
}

void
SpaceWireNIC::writeReg(Addr offset, uint32_t value)
{
    switch (offset) {
        case REG_CTRL:
            regCtrl = value;
            if (value & CTRL_RESET) {
                resetLink();
            }
            if (value & CTRL_LINK_START) {
                startLink();
            }
            if ((value & CTRL_TX_EN) && regTxLen > 0) {
                startTx();
            }
            break;
        case REG_LINK_CTRL:      regLinkCtrl = value; break;
        case REG_TX_BASE:        regTxBase = value; break;
        case REG_TX_LEN:         regTxLen = value; break;
        case REG_TX_DESC:        regTxDesc = value; break;
        case REG_RX_BASE:        regRxBase = value; break;
        case REG_RX_DESC:        regRxDesc = value; break;
        case REG_NODE_ADDR:      regNodeAddr = value & 0xFF; break;
        case REG_TIME_CODE:
            regTimeCode = value & 0xFF;
            sendTimeCode(regTimeCode);
            break;
        case REG_RMAP_CTRL:      regRmapCtrl = value; break;
        case REG_RMAP_KEY:       regRmapKey = value & 0xFF; break;
        case REG_DMA_CTRL:       regDmaCtrl = value; break;
        case REG_INTERRUPT:      regInterrupt = value; break;
        default:
            DPRINTF(SpaceWireNIC, "Unknown register write at 0x%x: 0x%x\n",
                    offset, value);
    }
}

void
SpaceWireNIC::linkStateMachine()
{
    DPRINTF(SpaceWireNIC, "Link state machine: %d\n", (int)linkState);
    
    switch (linkState) {
        case LinkState::ERROR_RESET:
            // Reset state - wait for error to clear
            linkState = LinkState::ERROR_WAIT;
            schedule(linkEvent, curTick() + cyclesToTicks(Cycles(100)));
            break;
            
        case LinkState::ERROR_WAIT:
            // Wait state - after error, wait before trying again
            linkState = LinkState::READY;
            schedule(linkEvent, curTick() + cyclesToTicks(Cycles(100)));
            break;
            
        case LinkState::READY:
            // Ready to start - wait for link enable
            if (regCtrl & CTRL_LINK_EN) {
                linkState = LinkState::STARTED;
                schedule(linkEvent, curTick() + cyclesToTicks(Cycles(50)));
            }
            break;
            
        case LinkState::STARTED:
            // Link started - send NULLs
            linkState = LinkState::CONNECTING;
            schedule(linkEvent, curTick() + cyclesToTicks(Cycles(50)));
            break;
            
        case LinkState::CONNECTING:
            // Connecting - exchange NULLs and FCTs
            linkState = LinkState::RUN;
            regLinkStatus |= STATUS_LINK_UP;
            regStatus |= STATUS_LINK_UP;
            DPRINTF(SpaceWireNIC, "Link is now UP\n");
            break;
            
        case LinkState::RUN:
            // Running - normal operation
            regLinkStatus |= STATUS_LINK_UP;
            break;
    }
}

void
SpaceWireNIC::startLink()
{
    DPRINTF(SpaceWireNIC, "Starting link\n");
    
    if (linkState == LinkState::READY || 
        linkState == LinkState::ERROR_RESET ||
        linkState == LinkState::ERROR_WAIT) {
        
        regCtrl |= CTRL_LINK_EN;
        linkState = LinkState::STARTED;
        schedule(linkEvent, curTick() + cyclesToTicks(Cycles(10)));
    }
}

void
SpaceWireNIC::stopLink()
{
    DPRINTF(SpaceWireNIC, "Stopping link\n");
    
    linkState = LinkState::ERROR_RESET;
    regLinkStatus &= ~STATUS_LINK_UP;
    regStatus &= ~STATUS_LINK_UP;
    regDisconnectCnt++;
    stats.disconnects++;
}

void
SpaceWireNIC::resetLink()
{
    DPRINTF(SpaceWireNIC, "Resetting link\n");
    
    linkState = LinkState::ERROR_RESET;
    regStatus = STATUS_TX_IDLE;
    regLinkStatus = 0;
    creditCount = 8;
    regCreditCnt = 8;
    regErrCnt = 0;
    
    // Clear queues
    while (!txQueue.empty()) txQueue.pop();
    while (!rxQueue.empty()) rxQueue.pop();
}

void
SpaceWireNIC::startTx()
{
    if (!(regStatus & STATUS_TX_IDLE)) {
        DPRINTF(SpaceWireNIC, "TX already active\n");
        return;
    }
    
    if (linkState != LinkState::RUN) {
        DPRINTF(SpaceWireNIC, "Link not up, cannot TX\n");
        regStatus |= STATUS_ERROR;
        return;
    }
    
    regStatus &= ~STATUS_TX_IDLE;
    regStatus |= STATUS_TX_ACTIVE;
    
    DPRINTF(SpaceWireNIC, "Starting TX: %d bytes from 0x%x\n",
            regTxLen, regTxBase);
    
    // Calculate transmission time based on link speed
    unsigned txTime = calculateTxTime(regTxLen);
    schedule(txEvent, curTick() + cyclesToTicks(Cycles(txTime)));
}

void
SpaceWireNIC::processTx()
{
    DPRINTF(SpaceWireNIC, "Processing TX complete\n");
    
    // Update statistics
    regPerfTxPkt++;
    regPerfTxBytes += regTxLen;
    stats.txPackets++;
    stats.txBytes += regTxLen;
    
    completeTx();
}

void
SpaceWireNIC::completeTx()
{
    regStatus &= ~STATUS_TX_ACTIVE;
    regStatus |= STATUS_TX_IDLE;
    
    // Generate interrupt if enabled
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x01;  // TX complete interrupt
    }
    
    DPRINTF(SpaceWireNIC, "TX complete\n");
}

void
SpaceWireNIC::processRx()
{
    if (rxQueue.empty()) {
        regStatus &= ~STATUS_RX_ACTIVE;
        return;
    }
    
    auto& packet = rxQueue.front();
    
    DPRINTF(SpaceWireNIC, "Processing RX: %zu bytes\n", packet.size());
    
    // Check if it's an RMAP command
    if (packet.size() > 1 && packet[1] == 0x01) {
        processRmapCommand(packet);
    }
    
    // Update statistics
    regPerfRxPkt++;
    regPerfRxBytes += packet.size();
    regRxLen = packet.size();
    stats.rxPackets++;
    stats.rxBytes += packet.size();
    
    rxQueue.pop();
    completeRx();
}

void
SpaceWireNIC::completeRx()
{
    regStatus &= ~STATUS_RX_ACTIVE;
    regStatus |= STATUS_RX_READY;
    
    // Generate interrupt if enabled
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x02;  // RX complete interrupt
    }
    
    DPRINTF(SpaceWireNIC, "RX complete, %d bytes received\n", regRxLen);
}

void
SpaceWireNIC::externalRx(const std::vector<uint8_t>& packet)
{
    if (linkState != LinkState::RUN) {
        DPRINTF(SpaceWireNIC, "Link not up, dropping RX packet\n");
        return;
    }
    
    if (rxQueue.size() >= rxFifoDepth) {
        DPRINTF(SpaceWireNIC, "RX FIFO full, dropping packet\n");
        return;
    }
    
    rxQueue.push(packet);
    
    if (!(regStatus & STATUS_RX_ACTIVE)) {
        regStatus |= STATUS_RX_ACTIVE;
        schedule(rxEvent, curTick() + cyclesToTicks(Cycles(10)));
    }
}

void
SpaceWireNIC::processRmapCommand(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 16) {
        DPRINTF(SpaceWireNIC, "RMAP packet too short\n");
        return;
    }
    
    DPRINTF(SpaceWireNIC, "Processing RMAP command\n");
    stats.rmapCommands++;
    
    RmapHeader cmd;
    cmd.destAddr = packet[0];
    cmd.protocolId = packet[1];
    cmd.instruction = packet[2];
    cmd.destKey = packet[3];
    cmd.srcAddr = packet[4];
    cmd.transId = (packet[5] << 8) | packet[6];
    cmd.extAddr = packet[7];
    cmd.addr = (packet[8] << 24) | (packet[9] << 16) | 
               (packet[10] << 8) | packet[11];
    cmd.length = (packet[12] << 16) | (packet[13] << 8) | packet[14];
    
    if (!validateRmapCommand(cmd)) {
        sendRmapReply(cmd, 0x01, {});  // Error status
        return;
    }
    
    // Process based on command type
    if (cmd.instruction & 0x20) {
        // Write command
        DPRINTF(SpaceWireNIC, "RMAP Write: addr=0x%x, len=%d\n",
                cmd.addr, cmd.length);
        sendRmapReply(cmd, 0x00, {});  // Success
    } else {
        // Read command
        DPRINTF(SpaceWireNIC, "RMAP Read: addr=0x%x, len=%d\n",
                cmd.addr, cmd.length);
        std::vector<uint8_t> data(cmd.length, 0);  // Dummy data
        sendRmapReply(cmd, 0x00, data);
    }
}

void
SpaceWireNIC::sendRmapReply(const RmapHeader& cmd, uint8_t status,
                             const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> reply;
    
    reply.push_back(cmd.srcAddr);     // Destination (original source)
    reply.push_back(0x01);            // Protocol ID
    reply.push_back((cmd.instruction & 0x3C) | 0x0C);  // Reply instruction
    reply.push_back(status);          // Status
    reply.push_back(regNodeAddr);     // Source (this node)
    reply.push_back((cmd.transId >> 8) & 0xFF);
    reply.push_back(cmd.transId & 0xFF);
    
    // Add data if this is a read reply
    if (!(cmd.instruction & 0x20) && status == 0x00) {
        reply.push_back((data.size() >> 16) & 0xFF);
        reply.push_back((data.size() >> 8) & 0xFF);
        reply.push_back(data.size() & 0xFF);
        reply.insert(reply.end(), data.begin(), data.end());
    }
    
    // Add CRC
    reply.push_back(computeCrc(reply.data(), reply.size()));
    
    txQueue.push(reply);
}

bool
SpaceWireNIC::validateRmapCommand(const RmapHeader& cmd)
{
    // Check protocol ID
    if (cmd.protocolId != 0x01) {
        return false;
    }
    
    // Check destination key
    if (cmd.destKey != regRmapKey && regRmapKey != 0) {
        return false;
    }
    
    // Check destination address
    if (cmd.destAddr != regNodeAddr && cmd.destAddr != 0xFE) {
        return false;
    }
    
    return true;
}

void
SpaceWireNIC::processTimeCode(uint8_t code)
{
    regTimeCode = code;
    stats.timeCodes++;
    
    DPRINTF(SpaceWireNIC, "Received time-code: %d\n", code);
    
    // Generate interrupt if enabled
    if (regCtrl & CTRL_IRQ_EN) {
        regInterrupt |= 0x04;  // Time-code interrupt
    }
}

void
SpaceWireNIC::sendTimeCode(uint8_t code)
{
    if (linkState != LinkState::RUN) {
        return;
    }
    
    DPRINTF(SpaceWireNIC, "Sending time-code: %d\n", code);
    stats.timeCodes++;
}

uint8_t
SpaceWireNIC::computeCrc(const uint8_t* data, unsigned length)
{
    // SpaceWire CRC (same as RMAP CRC)
    uint8_t crc = 0;
    
    for (unsigned i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;  // Polynomial: x^8 + x^2 + x + 1
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

unsigned
SpaceWireNIC::calculateTxTime(unsigned bytes)
{
    // SpaceWire uses 10-bit characters (8 data + parity + flag)
    // Plus overhead for EOP and flow control
    unsigned bits = bytes * 10 + 20;  // +20 for overhead
    
    // Cycles based on link speed and clock frequency
    // Assuming 100 MHz clock, scale by link speed
    unsigned cyclesPerMbit = 100;  // At 100 MHz, 100 cycles per Mbit at 1 Mbps
    unsigned cycles = (bits * cyclesPerMbit) / linkSpeed;
    
    return cycles > 0 ? cycles : 1;
}

} // namespace gem5



