/*
 * NOVA Processor - Shared FU Interface Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "cpu/shared_fu/shared_fu_interface.hh"

#include "base/trace.hh"
#include "debug/SharedFU.hh"

namespace gem5
{

SharedFUInterface::SharedFUInterface(SharedAcceleratorPool* pool, int cpuId)
    : pool(pool),
      cpuId(cpuId),
      waitingForFU(false),
      currentWaitType(SharedFunctionalUnit::AccelType::TRIG_ACCEL)
{
    DPRINTF(SharedFU, "SharedFUInterface created for CPU%d\n", cpuId);
}

bool
SharedFUInterface::canExecuteTrig(int op) const
{
    if (!pool) {
        // No pool configured - operations always proceed (dedicated mode)
        return true;
    }
    
    return pool->isAvailable(cpuId, SharedFunctionalUnit::AccelType::TRIG_ACCEL);
}

bool
SharedFUInterface::canExecuteMat(int op) const
{
    if (!pool) {
        return true;
    }
    
    return pool->isAvailable(cpuId, SharedFunctionalUnit::AccelType::MAT_ACCEL);
}

Cycles
SharedFUInterface::requestTrigOp(int threadId, int op,
                                 std::function<void()> onComplete)
{
    if (!pool) {
        // No pool - return fixed latency for dedicated FU
        // These match the TrigAccel latencies
        static const Cycles trigLatencies[] = {
            Cycles(15),  // SIN
            Cycles(15),  // COS
            Cycles(20),  // TAN
            Cycles(18),  // ATAN
            Cycles(22),  // ATAN2
            Cycles(12),  // SQRT
            Cycles(18)   // SINCOS
        };
        
        int opIdx = (op >= 0 && op < 7) ? op : 0;
        return trigLatencies[opIdx];
    }
    
    // Request through pool
    auto* fu = pool->requestFU(cpuId, threadId,
                               SharedFunctionalUnit::AccelType::TRIG_ACCEL,
                               op, [this, onComplete]() {
                                   waitingForFU = false;
                                   if (onComplete) onComplete();
                               });
    
    if (fu) {
        if (!fu->isBusy() || fu->getCurrentOwner() == cpuId) {
            // We got immediate access
            return Cycles(15);  // Will be overridden by actual FU latency
        } else {
            // We're queued - must stall
            waitingForFU = true;
            currentWaitType = SharedFunctionalUnit::AccelType::TRIG_ACCEL;
            
            DPRINTF(SharedFU, "CPU%d stalling for TrigAccel (queue=%d)\n",
                    cpuId, fu->getQueueDepth());
            
            return Cycles(0);  // Indicate stall required
        }
    }
    
    // No FU available at all
    return Cycles(0);
}

Cycles
SharedFUInterface::requestMatOp(int threadId, int op,
                                std::function<void()> onComplete)
{
    if (!pool) {
        // No pool - return fixed latency for dedicated FU
        static const Cycles matLatencies[] = {
            Cycles(50),   // MUL3x3
            Cycles(9),    // TRANSPOSE
            Cycles(100),  // INVERSE
            Cycles(27),   // MATVEC
            Cycles(9),    // ADD
            Cycles(9)     // SCALE
        };
        
        int opIdx = (op >= 0 && op < 6) ? op : 0;
        return matLatencies[opIdx];
    }
    
    // Request through pool
    auto* fu = pool->requestFU(cpuId, threadId,
                               SharedFunctionalUnit::AccelType::MAT_ACCEL,
                               op, [this, onComplete]() {
                                   waitingForFU = false;
                                   if (onComplete) onComplete();
                               });
    
    if (fu) {
        if (!fu->isBusy() || fu->getCurrentOwner() == cpuId) {
            return Cycles(50);  // Default MatAccel latency
        } else {
            waitingForFU = true;
            currentWaitType = SharedFunctionalUnit::AccelType::MAT_ACCEL;
            
            DPRINTF(SharedFU, "CPU%d stalling for MatAccel (queue=%d)\n",
                    cpuId, fu->getQueueDepth());
            
            return Cycles(0);
        }
    }
    
    return Cycles(0);
}

} // namespace gem5


