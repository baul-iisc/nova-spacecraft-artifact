/**
 * NOVA Processor - Vision Dispatcher Header
 * PhD Research: Futuristic Spacecraft Processor
 */

#ifndef __CPU_NOVA_SIMD_NOVA_VISION_DISPATCHER_HH__
#define __CPU_NOVA_SIMD_NOVA_VISION_DISPATCHER_HH__

#include <queue>

#include "cpu/nova_simd/nova_simd_unit.hh"
#include "params/NOVAVisionDispatcher.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * NOVA Vision Dispatcher - Dispatches vision workloads to SIMD units.
 */
class NOVAVisionDispatcher : public SimObject
{
  private:
    std::queue<VisionTask> pendingTasks;
    int maxQueueDepth;
    int tileWidth;
    int tileHeight;
    
  public:
    PARAMS(NOVAVisionDispatcher);
    
    NOVAVisionDispatcher(const Params &p);
    
    void dispatchImageProcessing(uint64_t srcAddr, uint64_t dstAddr,
                                  uint32_t width, uint32_t height,
                                  VisionOpType opType);
    
    void tileAndDispatch(const VisionTask &task);
};

} // namespace gem5

#endif // __CPU_NOVA_SIMD_NOVA_VISION_DISPATCHER_HH__
