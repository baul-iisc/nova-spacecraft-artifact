/**
 * @file cxl_interface.cc
 * @brief Implementation of CXL Interface Controller
 *
 * PhD Research: Chandraboul
 */

#include "custom_accel/cxl_interface.hh"

#include "base/trace.hh"
#include "debug/CXLInterface.hh"
#include "mem/packet_access.hh"

namespace gem5
{

CXLInterface::CXLInterface(const Params &p) :
    ClockedObject(p),
    cpuPort("cpu_side", this),
    addrRange(p.addr_range),
    defaultLinkSpeed(p.link_speed),
    defaultLinkWidth(p.link_width),
    accessLatency(p.access_latency),
    processEvent([this]{ processTransactions(); }, name()),
    statusReg(0),
    controlReg(0),
    stats(this)
{
    // Initialize memory ports
    for (unsigned i = 0; i < NUM_CXL_PORTS; i++) {
        std::string portName = csprintf("mem_port[%d]", i);
        memPorts[i] = new MemSidePort(portName, this, i);
        
        // Initialize port state
        ports[i].enabled = false;
        ports[i].linkUp = false;
        ports[i].linkSpeed = defaultLinkSpeed;
        ports[i].linkWidth = defaultLinkWidth;
        ports[i].txBytes = 0;
        ports[i].rxBytes = 0;
        ports[i].errors = 0;
    }

    DPRINTF(CXLInterface, "CXLInterface created with %d ports, "
            "link speed %d GT/s, width x%d\n",
            NUM_CXL_PORTS, defaultLinkSpeed, defaultLinkWidth);
}

CXLInterface::~CXLInterface()
{
    for (auto port : memPorts) {
        delete port;
    }
}

void
CXLInterface::init()
{
    ClockedObject::init();
    
    // Enable ports by default
    for (unsigned i = 0; i < NUM_CXL_PORTS; i++) {
        ports[i].enabled = true;
        ports[i].linkUp = true;  // Assume link is up
    }
    
    statusReg |= STATUS_ENABLED;
    
    DPRINTF(CXLInterface, "CXLInterface initialized\n");
}

Port&
CXLInterface::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "cpu_side") {
        return cpuPort;
    }
    if (if_name == "mem_side" && idx < NUM_CXL_PORTS) {
        return *memPorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

uint64_t
CXLInterface::read(Addr addr)
{
    Addr offset = addr - addrRange.start();
    
    if (offset < REG_PORT_BASE) {
        // Global registers
        switch (offset) {
          case REG_STATUS:
            return statusReg;
          case REG_CONTROL:
            return controlReg;
          default:
            return 0;
        }
    } else {
        // Per-port registers
        unsigned portId = (offset - REG_PORT_BASE) / REG_PORT_SIZE;
        Addr portOffset = (offset - REG_PORT_BASE) % REG_PORT_SIZE;
        
        if (portId < NUM_CXL_PORTS) {
            CXLPort& port = ports[portId];
            
            switch (portOffset) {
              case REG_PORT_STATUS:
                return (port.enabled ? PORT_ENABLED : 0) |
                       (port.linkUp ? PORT_LINK_UP : 0) |
                       (port.errors > 0 ? PORT_ERROR : 0);
              case REG_PORT_CONTROL:
                return port.enabled ? 1 : 0;
              case REG_PORT_LINK:
                return (port.linkSpeed << 8) | port.linkWidth;
              case REG_PORT_TX_COUNT:
                return port.txBytes;
              case REG_PORT_RX_COUNT:
                return port.rxBytes;
              default:
                return 0;
            }
        }
    }
    
    return 0;
}

void
CXLInterface::write(Addr addr, uint64_t data)
{
    Addr offset = addr - addrRange.start();
    
    if (offset < REG_PORT_BASE) {
        // Global registers
        switch (offset) {
          case REG_CONTROL:
            controlReg = data;
            if (data & 1) {
                statusReg |= STATUS_ENABLED;
                DPRINTF(CXLInterface, "CXL interface enabled\n");
            } else {
                statusReg &= ~STATUS_ENABLED;
                DPRINTF(CXLInterface, "CXL interface disabled\n");
            }
            break;
          default:
            break;
        }
    } else {
        // Per-port registers
        unsigned portId = (offset - REG_PORT_BASE) / REG_PORT_SIZE;
        Addr portOffset = (offset - REG_PORT_BASE) % REG_PORT_SIZE;
        
        if (portId < NUM_CXL_PORTS) {
            CXLPort& port = ports[portId];
            
            switch (portOffset) {
              case REG_PORT_CONTROL:
                port.enabled = (data & 1) != 0;
                DPRINTF(CXLInterface, "Port %d %s\n", portId,
                        port.enabled ? "enabled" : "disabled");
                break;
              case REG_PORT_LINK:
                port.linkSpeed = (data >> 8) & 0xFF;
                port.linkWidth = data & 0xFF;
                DPRINTF(CXLInterface, "Port %d link: %d GT/s x%d\n",
                        portId, port.linkSpeed, port.linkWidth);
                break;
              default:
                break;
            }
        }
    }
}

void
CXLInterface::processTransactions()
{
    // Process pending transactions on all ports
    for (unsigned i = 0; i < NUM_CXL_PORTS; i++) {
        if (!ports[i].enabled || !ports[i].linkUp)
            continue;
            
        // Process TX queue
        while (!txQueues[i].empty()) {
            PacketPtr pkt = txQueues[i].front();
            txQueues[i].pop();
            
            ports[i].txBytes += pkt->getSize();
            stats.totalTxBytes++;
            stats.portTxBytes[i]++;
            stats.totalTransactions++;
        }
        
        // Process RX queue
        while (!rxQueues[i].empty()) {
            PacketPtr pkt = rxQueues[i].front();
            rxQueues[i].pop();
            
            ports[i].rxBytes += pkt->getSize();
            stats.totalRxBytes++;
            stats.portRxBytes[i]++;
        }
    }
    
    // Reschedule if needed
    if (!processEvent.scheduled()) {
        schedule(processEvent, curTick() + cyclesToTicks(accessLatency));
    }
}

// CPU-side port implementation
CXLInterface::CPUSidePort::CPUSidePort(const std::string& name,
                                        CXLInterface* owner) :
    ResponsePort(name),
    owner(owner)
{
}

AddrRangeList
CXLInterface::CPUSidePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(owner->addrRange);
    return ranges;
}

bool
CXLInterface::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (pkt->isRead()) {
        pkt->setLE<uint32_t>(static_cast<uint32_t>(owner->read(pkt->getAddr())));
    } else if (pkt->isWrite()) {
        owner->write(pkt->getAddr(), pkt->getLE<uint32_t>());
    }
    pkt->makeResponse();
    return true;
}

Tick
CXLInterface::CPUSidePort::recvAtomic(PacketPtr pkt)
{
    if (pkt->isRead()) {
        pkt->setLE<uint32_t>(static_cast<uint32_t>(owner->read(pkt->getAddr())));
    } else if (pkt->isWrite()) {
        owner->write(pkt->getAddr(), pkt->getLE<uint32_t>());
    }
    pkt->makeResponse();
    return owner->clockPeriod();
}

void
CXLInterface::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        pkt->setLE<uint32_t>(static_cast<uint32_t>(owner->read(pkt->getAddr())));
    } else if (pkt->isWrite()) {
        owner->write(pkt->getAddr(), pkt->getLE<uint32_t>());
    }
    pkt->makeResponse();
}

// Memory-side port implementation
CXLInterface::MemSidePort::MemSidePort(const std::string& name,
                                        CXLInterface* owner,
                                        unsigned id) :
    RequestPort(name),
    owner(owner),
    portId(id)
{
}

bool
CXLInterface::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    owner->rxQueues[portId].push(pkt);
    owner->stats.memoryAccesses++;
    return true;
}

void
CXLInterface::MemSidePort::recvReqRetry()
{
    // Handle retry
}

// Statistics
CXLInterface::CXLStats::CXLStats(CXLInterface* parent) :
    statistics::Group(parent),
    ADD_STAT(totalTxBytes, statistics::units::Byte::get(),
             "Total bytes transmitted"),
    ADD_STAT(totalRxBytes, statistics::units::Byte::get(),
             "Total bytes received"),
    ADD_STAT(totalTransactions, statistics::units::Count::get(),
             "Total CXL transactions"),
    ADD_STAT(cacheCoherentAccesses, statistics::units::Count::get(),
             "Cache-coherent memory accesses"),
    ADD_STAT(memoryAccesses, statistics::units::Count::get(),
             "CXL memory accesses"),
    ADD_STAT(errors, statistics::units::Count::get(),
             "Total errors"),
    ADD_STAT(portTxBytes, statistics::units::Byte::get(),
             "TX bytes per port"),
    ADD_STAT(portRxBytes, statistics::units::Byte::get(),
             "RX bytes per port")
{
    portTxBytes.init(CXLInterface::NUM_CXL_PORTS);
    portRxBytes.init(CXLInterface::NUM_CXL_PORTS);
}

} // namespace gem5

