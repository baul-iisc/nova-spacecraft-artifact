/**
 * @file debug_interface.cc
 * @brief Implementation of Hardware Debug Interface Controller
 *
 * PhD Research: Chandraboul
 */

#include "custom_accel/debug_interface.hh"

#include "base/trace.hh"
#include "debug/DebugInterface.hh"
#include "mem/packet_access.hh"

namespace gem5
{

DebugInterface::DebugInterface(const Params &p) :
    ClockedObject(p),
    cpuPort("cpu_side", this),
    memPort("mem_side", this),
    numBreakpoints(p.num_breakpoints),
    numCores(p.num_cores),
    addrRange(p.addr_range),
    statusReg(0),
    controlReg(0),
    memAccessAddr(0),
    memAccessData(0),
    memAccessCtrl(0),
    regSelectReg(0),
    regDataReg(0),
    stats(this)
{
    // Initialize breakpoints
    for (unsigned i = 0; i < MAX_BREAKPOINTS; i++) {
        breakpoints[i].enabled = false;
        breakpoints[i].type = BreakpointType::EXECUTE;
        breakpoints[i].address = 0;
        breakpoints[i].addressMask = ~0ULL;
        breakpoints[i].size = 4;
        breakpoints[i].hitCount = 0;
        breakpoints[i].targetHits = 0;
    }

    // Initialize core states
    for (unsigned i = 0; i < MAX_CORES; i++) {
        coreHalted[i] = false;
        coreSingleStep[i] = false;
    }

    DPRINTF(DebugInterface, "DebugInterface created with %d breakpoints, "
            "%d cores\n", numBreakpoints, numCores);
}

DebugInterface::~DebugInterface()
{
}

void
DebugInterface::init()
{
    ClockedObject::init();
    DPRINTF(DebugInterface, "DebugInterface initialized\n");
}

Port&
DebugInterface::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "cpu_side") {
        return cpuPort;
    }
    if (if_name == "mem_side") {
        return memPort;
    }
    return ClockedObject::getPort(if_name, idx);
}

bool
DebugInterface::checkBreakpoint(Addr addr, BreakpointType type, unsigned coreId)
{
    if (!(statusReg & STATUS_ENABLED)) {
        return false;
    }

    for (unsigned i = 0; i < numBreakpoints; i++) {
        Breakpoint& bp = breakpoints[i];
        
        if (!bp.enabled) continue;
        if (bp.type != type) continue;
        
        // Check address match (with mask)
        if ((addr & bp.addressMask) == (bp.address & bp.addressMask)) {
            bp.hitCount++;
            
            // Check if we should actually break
            if (bp.targetHits == 0 || bp.hitCount >= bp.targetHits) {
                DPRINTF(DebugInterface, "Breakpoint %d hit at addr 0x%lx "
                        "(core %d, type %d)\n", i, addr, coreId, (int)type);
                
                stats.breakpointHits++;
                statusReg |= STATUS_BREAKPOINT;
                
                // Halt the core
                haltCore(coreId);
                return true;
            }
        }
    }
    
    return false;
}

bool
DebugInterface::isCoreHalted(unsigned coreId) const
{
    if (coreId >= numCores) return false;
    return coreHalted[coreId];
}

void
DebugInterface::haltCore(unsigned coreId)
{
    if (coreId >= numCores) return;
    
    coreHalted[coreId] = true;
    statusReg |= STATUS_HALTED;
    stats.halts++;
    
    DPRINTF(DebugInterface, "Core %d halted\n", coreId);
}

void
DebugInterface::resumeCore(unsigned coreId)
{
    if (coreId >= numCores) return;
    
    coreHalted[coreId] = false;
    coreSingleStep[coreId] = false;
    
    // Check if any core is still halted
    bool anyHalted = false;
    for (unsigned i = 0; i < numCores; i++) {
        if (coreHalted[i]) anyHalted = true;
    }
    if (!anyHalted) {
        statusReg &= ~STATUS_HALTED;
    }
    
    stats.resumes++;
    DPRINTF(DebugInterface, "Core %d resumed\n", coreId);
}

void
DebugInterface::singleStep(unsigned coreId)
{
    if (coreId >= numCores) return;
    
    coreSingleStep[coreId] = true;
    coreHalted[coreId] = false;  // Temporarily resume
    
    stats.singleSteps++;
    DPRINTF(DebugInterface, "Core %d single-stepping\n", coreId);
}

uint64_t
DebugInterface::read(Addr addr)
{
    Addr offset = addr - addrRange.start();
    
    // Global registers
    if (offset < REG_BP_BASE) {
        switch (offset) {
          case REG_STATUS:
            return statusReg;
          case REG_CONTROL:
            return controlReg;
          case REG_HALT_STATUS:
            {
                uint32_t halted = 0;
                for (unsigned i = 0; i < numCores; i++) {
                    if (coreHalted[i]) halted |= (1 << i);
                }
                return halted;
            }
          case REG_MEM_ADDR:
            return memAccessAddr;
          case REG_MEM_DATA:
            stats.memoryReads++;
            return memAccessData;
          case REG_MEM_CTRL:
            return memAccessCtrl;
          case REG_REG_SEL:
            return regSelectReg;
          case REG_REG_DATA:
            stats.registerReads++;
            return regDataReg;
          default:
            return 0;
        }
    }
    // Breakpoint registers
    else if (offset < REG_MEM_ADDR) {
        unsigned bpId = (offset - REG_BP_BASE) / REG_BP_SIZE;
        Addr bpOffset = (offset - REG_BP_BASE) % REG_BP_SIZE;
        
        if (bpId < numBreakpoints) {
            Breakpoint& bp = breakpoints[bpId];
            
            switch (bpOffset) {
              case REG_BP_CTRL:
                {
                    uint32_t ctrl = bp.enabled ? BP_ENABLE : 0;
                    switch (bp.type) {
                      case BreakpointType::EXECUTE:
                        ctrl |= BP_TYPE_EXEC;
                        break;
                      case BreakpointType::READ:
                        ctrl |= BP_TYPE_READ;
                        break;
                      case BreakpointType::WRITE:
                        ctrl |= BP_TYPE_WRITE;
                        break;
                      case BreakpointType::ACCESS:
                        ctrl |= BP_TYPE_ACCESS;
                        break;
                    }
                    return ctrl;
                }
              case REG_BP_ADDR:
                return bp.address;
              case REG_BP_MASK:
                return bp.addressMask;
              case REG_BP_COUNT:
                return bp.hitCount;
              default:
                return 0;
            }
        }
    }
    
    return 0;
}

void
DebugInterface::write(Addr addr, uint64_t data)
{
    Addr offset = addr - addrRange.start();
    
    // Global registers
    if (offset < REG_BP_BASE) {
        switch (offset) {
          case REG_CONTROL:
            controlReg = data;
            
            if (data & CTRL_ENABLE) {
                statusReg |= STATUS_ENABLED;
                DPRINTF(DebugInterface, "Debug interface enabled\n");
            } else {
                statusReg &= ~STATUS_ENABLED;
            }
            
            if (data & CTRL_HALT_ALL) {
                for (unsigned i = 0; i < numCores; i++) {
                    haltCore(i);
                }
            }
            
            if (data & CTRL_RESUME_ALL) {
                for (unsigned i = 0; i < numCores; i++) {
                    resumeCore(i);
                }
            }
            
            if (data & CTRL_RESET) {
                statusReg &= ~(STATUS_BREAKPOINT | STATUS_STEP_DONE);
            }
            break;
            
          case REG_HALT_CONTROL:
            // Individual core halt/resume control
            for (unsigned i = 0; i < numCores; i++) {
                if (data & (1 << i)) {
                    haltCore(i);
                }
            }
            break;
            
          case REG_STEP_CONTROL:
            // Single-step control
            for (unsigned i = 0; i < numCores; i++) {
                if (data & (1 << i)) {
                    singleStep(i);
                }
            }
            break;
            
          case REG_MEM_ADDR:
            memAccessAddr = data;
            break;
            
          case REG_MEM_DATA:
            memAccessData = data;
            stats.memoryWrites++;
            break;
            
          case REG_MEM_CTRL:
            memAccessCtrl = data;
            // Trigger memory operation based on control
            if (data & 1) {
                performMemoryRead();
            } else if (data & 2) {
                performMemoryWrite();
            }
            break;
            
          case REG_REG_SEL:
            regSelectReg = data;
            break;
            
          case REG_REG_DATA:
            regDataReg = data;
            stats.registerWrites++;
            break;
            
          default:
            break;
        }
    }
    // Breakpoint registers
    else if (offset < REG_MEM_ADDR) {
        unsigned bpId = (offset - REG_BP_BASE) / REG_BP_SIZE;
        Addr bpOffset = (offset - REG_BP_BASE) % REG_BP_SIZE;
        
        if (bpId < numBreakpoints) {
            Breakpoint& bp = breakpoints[bpId];
            
            switch (bpOffset) {
              case REG_BP_CTRL:
                bp.enabled = (data & BP_ENABLE) != 0;
                switch ((data & BP_TYPE_MASK) >> 1) {
                  case 0:
                    bp.type = BreakpointType::EXECUTE;
                    break;
                  case 1:
                    bp.type = BreakpointType::READ;
                    break;
                  case 2:
                    bp.type = BreakpointType::WRITE;
                    break;
                  case 3:
                    bp.type = BreakpointType::ACCESS;
                    break;
                }
                DPRINTF(DebugInterface, "Breakpoint %d %s, type=%d\n",
                        bpId, bp.enabled ? "enabled" : "disabled",
                        (int)bp.type);
                break;
                
              case REG_BP_ADDR:
                bp.address = data;
                DPRINTF(DebugInterface, "Breakpoint %d address=0x%lx\n",
                        bpId, bp.address);
                break;
                
              case REG_BP_MASK:
                bp.addressMask = data;
                break;
                
              case REG_BP_COUNT:
                bp.hitCount = 0;  // Reset count on write
                bp.targetHits = data;
                break;
                
              default:
                break;
            }
        }
    }
}

void
DebugInterface::performMemoryRead()
{
    // In a full implementation, this would issue a memory read
    // through the memory port and store the result in memAccessData
    DPRINTF(DebugInterface, "Memory read from 0x%lx\n", memAccessAddr);
}

void
DebugInterface::performMemoryWrite()
{
    // In a full implementation, this would issue a memory write
    // through the memory port
    DPRINTF(DebugInterface, "Memory write to 0x%lx, data=0x%lx\n",
            memAccessAddr, memAccessData);
}

// CPU-side port implementation
DebugInterface::CPUSidePort::CPUSidePort(const std::string& name,
                                          DebugInterface* owner) :
    ResponsePort(name),
    owner(owner)
{
}

AddrRangeList
DebugInterface::CPUSidePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(owner->addrRange);
    return ranges;
}

bool
DebugInterface::CPUSidePort::recvTimingReq(PacketPtr pkt)
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
DebugInterface::CPUSidePort::recvAtomic(PacketPtr pkt)
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
DebugInterface::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        pkt->setLE<uint32_t>(static_cast<uint32_t>(owner->read(pkt->getAddr())));
    } else if (pkt->isWrite()) {
        owner->write(pkt->getAddr(), pkt->getLE<uint32_t>());
    }
    pkt->makeResponse();
}

// Memory-side port implementation
DebugInterface::MemSidePort::MemSidePort(const std::string& name,
                                          DebugInterface* owner) :
    RequestPort(name),
    owner(owner)
{
}

bool
DebugInterface::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // Handle memory response
    if (pkt->isRead()) {
        owner->memAccessData = pkt->getLE<uint32_t>();
    }
    return true;
}

void
DebugInterface::MemSidePort::recvReqRetry()
{
    // Handle retry
}

// Statistics
DebugInterface::DebugStats::DebugStats(DebugInterface* parent) :
    statistics::Group(parent),
    ADD_STAT(breakpointHits, statistics::units::Count::get(),
             "Total breakpoint hits"),
    ADD_STAT(memoryReads, statistics::units::Count::get(),
             "Debug memory reads"),
    ADD_STAT(memoryWrites, statistics::units::Count::get(),
             "Debug memory writes"),
    ADD_STAT(registerReads, statistics::units::Count::get(),
             "Debug register reads"),
    ADD_STAT(registerWrites, statistics::units::Count::get(),
             "Debug register writes"),
    ADD_STAT(singleSteps, statistics::units::Count::get(),
             "Single-step executions"),
    ADD_STAT(halts, statistics::units::Count::get(),
             "Core halts"),
    ADD_STAT(resumes, statistics::units::Count::get(),
             "Core resumes")
{
}

} // namespace gem5

