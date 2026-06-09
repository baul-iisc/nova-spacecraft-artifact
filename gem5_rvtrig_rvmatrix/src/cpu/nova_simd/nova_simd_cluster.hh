/**
 * NOVA Processor - SIMD Cluster Header
 * PhD Research: Futuristic Spacecraft Processor
 */

#ifndef __CPU_NOVA_SIMD_NOVA_SIMD_CLUSTER_HH__
#define __CPU_NOVA_SIMD_NOVA_SIMD_CLUSTER_HH__

#include <vector>

#include "cpu/nova_simd/nova_simd_unit.hh"
#include "params/NOVASIMDCluster.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * NOVA SIMD Cluster - Group of SIMD units sharing L2 cache.
 */
class NOVASIMDCluster : public SimObject
{
  private:
    std::vector<NOVASIMDUnit*> simdUnits;
    int numUnits;
    
  public:
    PARAMS(NOVASIMDCluster);
    
    NOVASIMDCluster(const Params &p);
    
    bool submitTask(const VisionTask &task);
    int selectUnit();
};

} // namespace gem5

#endif // __CPU_NOVA_SIMD_NOVA_SIMD_CLUSTER_HH__
