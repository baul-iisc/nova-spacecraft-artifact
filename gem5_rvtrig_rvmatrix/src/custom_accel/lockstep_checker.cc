/**
 * @file lockstep_checker.cc
 * @brief Implementation of Dual-core Lockstep (DLS) Checker
 *
 * PhD Research: Chandraboul
 */

#include "custom_accel/lockstep_checker.hh"

#include "base/trace.hh"
#include "debug/LockstepChecker.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

LockstepChecker::LockstepChecker(const Params &p) :
    ClockedObject(p),
    numPairs(p.num_pairs),
    comparisonLatency(p.comparison_latency),
    recoveryLatency(p.recovery_latency),
    enableRecovery(p.enable_recovery),
    strictMode(p.strict_mode),
    processEvent([this]{ processEventHandler(); }, name()),
    addrRange(p.addr_range),
    statusReg(0),
    controlReg(0),
    stats(this)
{
    // Create CPU ports for all cores
    for (unsigned i = 0; i < NUM_CORES; i++) {
        std::string portName = csprintf("cpu_port[%d]", i);
        cpuPorts.push_back(new CPUSidePort(portName, this, i));
    }

    // Initialize DLS pairs
    // Pair 0: Core 0 (primary) + Core 1 (checker)
    // Pair 1: Core 2 (primary) + Core 3 (checker)
    // Pair 2: Core 4 (primary) + Core 5 (checker)
    // Pair 3: Core 6 (primary) + Core 7 (checker)
    for (unsigned i = 0; i < numPairs; i++) {
        dlsPairs[i].primaryCoreId = i * 2;
        dlsPairs[i].checkerCoreId = i * 2 + 1;
        dlsPairs[i].synchronized = false;
        dlsPairs[i].lastMatchedSeq = 0;
        dlsPairs[i].comparisons = 0;
        dlsPairs[i].mismatches = 0;
        dlsPairs[i].recoveries = 0;
    }

    DPRINTF(LockstepChecker, "LockstepChecker created with %u DLS pairs\n",
            numPairs);
}

LockstepChecker::~LockstepChecker()
{
    for (auto port : cpuPorts) {
        delete port;
    }
}

void
LockstepChecker::init()
{
    ClockedObject::init();
    
    // Schedule initial processing event
    if (!processEvent.scheduled()) {
        schedule(processEvent, curTick() + cyclesToTicks(Cycles(1)));
    }
    
    DPRINTF(LockstepChecker, "LockstepChecker initialized\n");
}

Port&
LockstepChecker::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "cpu_port" && idx < cpuPorts.size()) {
        return *cpuPorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

bool
LockstepChecker::isPrimaryCore(unsigned coreId) const
{
    return (coreId % 2) == 0;
}

bool
LockstepChecker::isCheckerCore(unsigned coreId) const
{
    return (coreId % 2) == 1;
}

int
LockstepChecker::getPairId(unsigned coreId) const
{
    return coreId / 2;
}

void
LockstepChecker::recordCommit(unsigned coreId, const CommitInfo& info)
{
    if (!(statusReg & STATUS_ENABLED)) {
        return;
    }

    int pairId = getPairId(coreId);
    if (pairId < 0 || pairId >= static_cast<int>(numPairs)) {
        return;
    }

    DLSPair& pair = dlsPairs[pairId];
    
    if (isPrimaryCore(coreId)) {
        if (pair.primaryQueue.size() < COMMIT_QUEUE_SIZE) {
            pair.primaryQueue.push(info);
            DPRINTF(LockstepChecker, "Pair %d: Primary commit PC=0x%lx seq=%lu\n",
                    pairId, info.pc, info.seqNum);
        }
    } else {
        if (pair.checkerQueue.size() < COMMIT_QUEUE_SIZE) {
            pair.checkerQueue.push(info);
            DPRINTF(LockstepChecker, "Pair %d: Checker commit PC=0x%lx seq=%lu\n",
                    pairId, info.pc, info.seqNum);
        }
    }

    // Process commits immediately if both queues have data
    if (!pair.primaryQueue.empty() && !pair.checkerQueue.empty()) {
        processCommits(pairId);
    }
}

void
LockstepChecker::processCommits(unsigned pairId)
{
    if (pairId >= numPairs) return;
    
    DLSPair& pair = dlsPairs[pairId];

    while (!pair.primaryQueue.empty() && !pair.checkerQueue.empty()) {
        CommitInfo& primary = pair.primaryQueue.front();
        CommitInfo& checker = pair.checkerQueue.front();

        // Ensure sequence numbers match (may need resync)
        if (primary.seqNum != checker.seqNum) {
            // Try to resynchronize
            if (primary.seqNum < checker.seqNum) {
                pair.primaryQueue.pop();
                continue;
            } else {
                pair.checkerQueue.pop();
                continue;
            }
        }

        // Compare the commits
        pair.comparisons++;
        stats.totalComparisons++;

        if (compareCommits(primary, checker)) {
            // Match - instruction verified
            pair.lastMatchedSeq = primary.seqNum;
            pair.synchronized = true;
            DPRINTF(LockstepChecker, "Pair %d: Match seq=%lu\n",
                    pairId, primary.seqNum);
        } else {
            // Mismatch - fault detected!
            pair.mismatches++;
            stats.totalMismatches++;
            stats.pairMismatches[pairId]++;
            
            DPRINTF(LockstepChecker, 
                    "Pair %d: MISMATCH at PC=0x%lx seq=%lu! "
                    "Primary result=0x%lx, Checker result=0x%lx\n",
                    pairId, primary.pc, primary.seqNum,
                    primary.result, checker.result);

            statusReg |= STATUS_MISMATCH;
            handleMismatch(pairId, primary, checker);
        }

        pair.primaryQueue.pop();
        pair.checkerQueue.pop();
    }
}

bool
LockstepChecker::compareCommits(const CommitInfo& a, const CommitInfo& b) const
{
    // Compare PC
    if (a.pc != b.pc) return false;
    
    // Compare instruction encoding (optional in non-strict mode)
    if (strictMode && a.inst != b.inst) return false;
    
    // Compare result
    if (a.result != b.result) return false;
    
    // Compare memory operations
    if (a.isMemOp != b.isMemOp) return false;
    
    if (a.isMemOp) {
        if (a.addr != b.addr) return false;
        if (a.isStore && a.data != b.data) return false;
    }
    
    return true;
}

bool
LockstepChecker::handleMismatch(unsigned pairId, const CommitInfo& primary,
                                 const CommitInfo& checker)
{
    if (!enableRecovery) {
        // Just report, don't recover
        return false;
    }

    statusReg |= STATUS_RECOVERY;
    
    // Trigger recovery
    triggerRecovery(pairId);
    
    dlsPairs[pairId].recoveries++;
    stats.totalRecoveries++;
    
    DPRINTF(LockstepChecker, "Pair %d: Recovery initiated\n", pairId);
    
    statusReg &= ~STATUS_RECOVERY;
    statusReg &= ~STATUS_MISMATCH;
    
    return true;
}

void
LockstepChecker::triggerRecovery(unsigned pairId)
{
    DLSPair& pair = dlsPairs[pairId];
    
    // Clear commit queues
    while (!pair.primaryQueue.empty()) pair.primaryQueue.pop();
    while (!pair.checkerQueue.empty()) pair.checkerQueue.pop();
    
    // Mark as needing resync
    pair.synchronized = false;
    
    // In a real implementation, this would:
    // 1. Signal the CPU to flush the pipeline
    // 2. Roll back to last known good state
    // 3. Re-execute from that point
    
    DPRINTF(LockstepChecker, 
            "Pair %d: Pipeline flush and re-execution triggered\n", pairId);
}

void
LockstepChecker::processEventHandler()
{
    // Periodic processing of all pairs
    for (unsigned i = 0; i < numPairs; i++) {
        processCommits(i);
    }
    
    // Reschedule
    schedule(processEvent, curTick() + cyclesToTicks(Cycles(comparisonLatency)));
}

uint64_t
LockstepChecker::read(Addr addr)
{
    Addr offset = addr - addrRange.start();
    
    switch (offset) {
      case REG_STATUS:
        return statusReg;
        
      case REG_CONTROL:
        return controlReg;
        
      default:
        // Per-pair registers
        if (offset >= REG_PAIR_STATUS && offset < REG_MISMATCH_COUNT) {
            unsigned pairId = (offset - REG_PAIR_STATUS) / 4;
            if (pairId < numPairs) {
                return (dlsPairs[pairId].synchronized ? 1 : 0);
            }
        }
        else if (offset >= REG_MISMATCH_COUNT && offset < REG_RECOVERY_COUNT) {
            unsigned pairId = (offset - REG_MISMATCH_COUNT) / 4;
            if (pairId < numPairs) {
                return dlsPairs[pairId].mismatches;
            }
        }
        else if (offset >= REG_RECOVERY_COUNT && offset < REG_LAST_MISMATCH) {
            unsigned pairId = (offset - REG_RECOVERY_COUNT) / 4;
            if (pairId < numPairs) {
                return dlsPairs[pairId].recoveries;
            }
        }
        break;
    }
    
    return 0;
}

void
LockstepChecker::write(Addr addr, uint64_t data)
{
    Addr offset = addr - addrRange.start();
    
    switch (offset) {
      case REG_CONTROL:
        controlReg = data;
        // Bit 0: Enable
        if (data & 1) {
            statusReg |= STATUS_ENABLED;
        } else {
            statusReg &= ~STATUS_ENABLED;
        }
        DPRINTF(LockstepChecker, "Control register set to 0x%lx\n", data);
        break;
        
      default:
        break;
    }
}

// Port implementation
LockstepChecker::CPUSidePort::CPUSidePort(const std::string& name,
                                           LockstepChecker* owner,
                                           unsigned core_id) :
    ResponsePort(name),
    owner(owner),
    coreId(core_id)
{
}

AddrRangeList
LockstepChecker::CPUSidePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(owner->addrRange);
    return ranges;
}

bool
LockstepChecker::CPUSidePort::recvTimingReq(PacketPtr pkt)
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
LockstepChecker::CPUSidePort::recvAtomic(PacketPtr pkt)
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
LockstepChecker::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    if (pkt->isRead()) {
        pkt->setLE<uint32_t>(static_cast<uint32_t>(owner->read(pkt->getAddr())));
    } else if (pkt->isWrite()) {
        owner->write(pkt->getAddr(), pkt->getLE<uint32_t>());
    }
    pkt->makeResponse();
}

// Statistics
LockstepChecker::LockstepStats::LockstepStats(LockstepChecker* parent) :
    statistics::Group(parent),
    ADD_STAT(totalComparisons, statistics::units::Count::get(),
             "Total instruction comparisons"),
    ADD_STAT(totalMismatches, statistics::units::Count::get(),
             "Total mismatches detected"),
    ADD_STAT(totalRecoveries, statistics::units::Count::get(),
             "Total successful recoveries"),
    ADD_STAT(falsePositives, statistics::units::Count::get(),
             "False positive mismatch detections"),
    ADD_STAT(mismatchLatency, statistics::units::Cycle::get(),
             "Distribution of mismatch detection latency"),
    ADD_STAT(pairMismatches, statistics::units::Count::get(),
             "Mismatches per DLS pair")
{
    pairMismatches.init(LockstepChecker::MAX_DLS_PAIRS);
    mismatchLatency.init(1, 100, 10);
}

} // namespace gem5

