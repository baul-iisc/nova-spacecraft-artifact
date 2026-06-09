/*
 * NOVA Processor - Shared FU Interface for CPU
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * This header provides the interface that CPU models use to access
 * shared functional units. It supports:
 * 
 * 1. Checking if a shared FU is available
 * 2. Requesting access to a shared FU
 * 3. Stalling the CPU when FU is busy
 * 4. Resuming when FU becomes available
 */

#ifndef __CPU_SHARED_FU_SHARED_FU_INTERFACE_HH__
#define __CPU_SHARED_FU_SHARED_FU_INTERFACE_HH__

#include "cpu/shared_fu/shared_functional_unit.hh"
#include "sim/sim_object.hh"

#include <functional>

namespace gem5
{

/**
 * Interface for CPU to access shared functional units.
 * 
 * This class provides a simple API for CPU models to use shared
 * accelerators with proper stalling when the FU is busy.
 */
class SharedFUInterface
{
  public:
    /**
     * Constructor.
     * @param pool Pointer to the shared accelerator pool
     * @param cpuId CPU ID for this interface
     */
    SharedFUInterface(SharedAcceleratorPool* pool, int cpuId);
    
    ~SharedFUInterface() = default;
    
    /**
     * Check if a trigonometric operation can proceed immediately.
     * @param op Operation type (0=SIN, 1=COS, etc.)
     * @return true if FU is free, false if would need to stall
     */
    bool canExecuteTrig(int op) const;
    
    /**
     * Check if a matrix operation can proceed immediately.
     * @param op Operation type
     * @return true if FU is free, false if would need to stall
     */
    bool canExecuteMat(int op) const;
    
    /**
     * Request a trigonometric operation.
     * 
     * @param threadId Thread making the request
     * @param op Operation type
     * @param onComplete Callback when operation completes
     * @return Cycles until completion (0 if queued - must stall)
     */
    Cycles requestTrigOp(int threadId, int op, 
                         std::function<void()> onComplete);
    
    /**
     * Request a matrix operation.
     * 
     * @param threadId Thread making the request
     * @param op Operation type
     * @param onComplete Callback when operation completes
     * @return Cycles until completion (0 if queued - must stall)
     */
    Cycles requestMatOp(int threadId, int op,
                        std::function<void()> onComplete);
    
    /**
     * Check if currently waiting for a shared FU.
     */
    bool isWaiting() const { return waitingForFU; }
    
    /**
     * Mark that we're done waiting (called from completion callback).
     */
    void clearWaiting() { waitingForFU = false; }
    
    /**
     * Get the pool.
     */
    SharedAcceleratorPool* getPool() { return pool; }
    
  private:
    SharedAcceleratorPool* pool;
    int cpuId;
    bool waitingForFU;
    SharedFunctionalUnit::AccelType currentWaitType;
};


/**
 * Macros for instruction execution with shared FU
 * 
 * These can be used in instruction implementations to check
 * and request shared FU access.
 */
#define SHARED_FU_TRIG_CHECK(iface, op) \
    if (!(iface)->canExecuteTrig(op)) { \
        return GenericISA::DelaySlot; \
    }

#define SHARED_FU_MAT_CHECK(iface, op) \
    if (!(iface)->canExecuteMat(op)) { \
        return GenericISA::DelaySlot; \
    }


} // namespace gem5

#endif // __CPU_SHARED_FU_SHARED_FU_INTERFACE_HH__


