/**
 * NOVA Processor - GPU-like SIMD Vision Accelerator
 * PhD Research: Futuristic Spacecraft Processor
 *
 * C++ Implementation of the NOVA SIMD unit.
 */

#include "cpu/nova_simd/nova_simd_unit.hh"
#include "cpu/nova_simd/nova_simd_cluster.hh"
#include "cpu/nova_simd/nova_vision_dispatcher.hh"

#include "base/trace.hh"
#include "debug/NOVASIMDUnit.hh"

namespace gem5
{

// ============== NOVASIMDUnit Implementation ==============

NOVASIMDUnit::NOVASIMDUnit(const Params &p) :
    SimObject(p),
    numPEs(p.num_pes),
    simdWidth(p.simd_width),
    warpSize(p.warp_size),
    maxWarps(p.max_warps),
    localMemorySize(p.local_memory_size),
    activeWarps(0),
    stats(this)
{
    // Initialize processing elements
    for (int i = 0; i < numPEs; i++) {
        processingElements.emplace_back(i);
    }
    
    // Initialize warps
    for (int i = 0; i < maxWarps; i++) {
        warps.emplace_back(i);
    }
    
    // Initialize local memory
    localMemory.resize(localMemorySize, 0);
    
    DPRINTF(NOVASIMDUnit, "NOVA SIMD Unit initialized: %d PEs, %d-bit SIMD, %d warps\n",
            numPEs, simdWidth, maxWarps);
}

NOVASIMDUnit::~NOVASIMDUnit()
{
}

bool NOVASIMDUnit::submitTask(const VisionTask &task)
{
    if (taskQueue.size() >= 64) {
        DPRINTF(NOVASIMDUnit, "Task queue full, rejecting task\n");
        return false;
    }
    
    taskQueue.push(task);
    stats.totalTasks++;
    
    DPRINTF(NOVASIMDUnit, "Task submitted: type=%d, size=%dx%d\n",
            static_cast<int>(task.opType), task.width, task.height);
    
    return true;
}

int NOVASIMDUnit::allocateWarp()
{
    for (int i = 0; i < maxWarps; i++) {
        if (!warps[i].active) {
            warps[i].active = true;
            warps[i].waiting = false;
            warps[i].programCounter = 0;
            activeWarps++;
            DPRINTF(NOVASIMDUnit, "Allocated warp %d, active warps: %d\n", i, activeWarps);
            return i;
        }
    }
    return -1;  // No free warps
}

void NOVASIMDUnit::freeWarp(int warpId)
{
    if (warpId >= 0 && warpId < maxWarps && warps[warpId].active) {
        warps[warpId].active = false;
        activeWarps--;
        DPRINTF(NOVASIMDUnit, "Freed warp %d, active warps: %d\n", warpId, activeWarps);
    }
}

void NOVASIMDUnit::tick()
{
    stats.totalCycles++;
    
    // Process queued tasks
    if (!taskQueue.empty() && activeWarps < maxWarps) {
        VisionTask task = taskQueue.front();
        taskQueue.pop();
        
        int warpId = allocateWarp();
        if (warpId >= 0) {
            // Execute based on operation type
            switch (task.opType) {
                case VisionOpType::CONV_3x3:
                case VisionOpType::CONV_5x5:
                case VisionOpType::CONV_7x7:
                    executeConvolution(task);
                    stats.convolutionOps++;
                    break;
                    
                case VisionOpType::HARRIS_CORNER:
                case VisionOpType::FAST_CORNER:
                case VisionOpType::ORB_FEATURE:
                    executeHarrisCorner(task);
                    stats.featureOps++;
                    break;
                    
                case VisionOpType::OPTICAL_FLOW_LK:
                    executeOpticalFlow(task);
                    stats.flowOps++;
                    break;
                    
                case VisionOpType::SOBEL_EDGE:
                case VisionOpType::GAUSSIAN_BLUR:
                    executeSobelEdge(task);
                    stats.filterOps++;
                    break;
                    
                default:
                    stats.filterOps++;
                    break;
            }
            
            freeWarp(warpId);
        }
    } else if (activeWarps == 0) {
        stats.idleCycles++;
    }
    
    // Execute active warps
    for (int i = 0; i < maxWarps; i++) {
        if (warps[i].active && !warps[i].waiting) {
            executeWarp(i);
        }
    }
}

void NOVASIMDUnit::executeWarp(int warpId)
{
    DPRINTF(NOVASIMDUnit, "Executing warp %d at PC %d\n", 
            warpId, warps[warpId].programCounter);
    
    // Simulate SIMD execution across all PEs
    for (int i = 0; i < numPEs; i++) {
        if (warps[warpId].activeMask & (1 << i)) {
            processingElements[i].fp32Ops++;
        }
    }
    
    warps[warpId].programCounter++;
}

void NOVASIMDUnit::executeConvolution(const VisionTask &task)
{
    DPRINTF(NOVASIMDUnit, "Executing convolution: %dx%d kernel on %dx%d image\n",
            task.kernelSize, task.kernelSize, task.width, task.height);
    
    // Simulate convolution operations
    uint64_t ops = task.width * task.height * task.kernelSize * task.kernelSize;
    for (int i = 0; i < numPEs; i++) {
        processingElements[i].fp32Ops += ops / numPEs;
    }
}

void NOVASIMDUnit::executeHarrisCorner(const VisionTask &task)
{
    DPRINTF(NOVASIMDUnit, "Executing Harris corner detection on %dx%d image\n",
            task.width, task.height);
    
    // Harris corner detection involves gradient computation and corner response
    uint64_t ops = task.width * task.height * 20;  // Approximate ops per pixel
    for (int i = 0; i < numPEs; i++) {
        processingElements[i].fp32Ops += ops / numPEs;
    }
}

void NOVASIMDUnit::executeOpticalFlow(const VisionTask &task)
{
    DPRINTF(NOVASIMDUnit, "Executing optical flow on %dx%d image\n",
            task.width, task.height);
    
    // Lucas-Kanade optical flow
    uint64_t ops = task.width * task.height * 50;  // Approximate ops per pixel
    for (int i = 0; i < numPEs; i++) {
        processingElements[i].fp32Ops += ops / numPEs;
    }
}

void NOVASIMDUnit::executeSobelEdge(const VisionTask &task)
{
    DPRINTF(NOVASIMDUnit, "Executing Sobel edge detection on %dx%d image\n",
            task.width, task.height);
    
    // Sobel uses 3x3 convolution for gradient
    uint64_t ops = task.width * task.height * 18;  // 9 ops for Gx + 9 for Gy
    for (int i = 0; i < numPEs; i++) {
        processingElements[i].fp32Ops += ops / numPEs;
    }
}

void NOVASIMDUnit::loadToLocalMemory(uint64_t addr, size_t size)
{
    DPRINTF(NOVASIMDUnit, "Loading %lu bytes to local memory from 0x%lx\n", size, addr);
}

void NOVASIMDUnit::storeFromLocalMemory(uint64_t addr, size_t size)
{
    DPRINTF(NOVASIMDUnit, "Storing %lu bytes from local memory to 0x%lx\n", size, addr);
}

void NOVASIMDUnit::printStats() const
{
}

uint64_t NOVASIMDUnit::getTotalOps() const
{
    uint64_t total = 0;
    for (const auto &pe : processingElements) {
        total += pe.fp32Ops + pe.fp64Ops + pe.intOps;
    }
    return total;
}

double NOVASIMDUnit::getUtilization() const
{
    uint64_t totalCycles = stats.totalCycles.value();
    uint64_t idleCycles = stats.idleCycles.value();
    
    if (totalCycles == 0) return 0.0;
    return 1.0 - (static_cast<double>(idleCycles) / totalCycles);
}

// Statistics implementation
NOVASIMDUnit::NOVASIMDStats::NOVASIMDStats(NOVASIMDUnit *parent)
    : statistics::Group(parent),
      ADD_STAT(totalTasks, statistics::units::Count::get(),
               "Total vision tasks processed"),
      ADD_STAT(convolutionOps, statistics::units::Count::get(),
               "Convolution operations"),
      ADD_STAT(featureOps, statistics::units::Count::get(),
               "Feature detection operations"),
      ADD_STAT(flowOps, statistics::units::Count::get(),
               "Optical flow operations"),
      ADD_STAT(filterOps, statistics::units::Count::get(),
               "Filter operations"),
      ADD_STAT(totalCycles, statistics::units::Cycle::get(),
               "Total execution cycles"),
      ADD_STAT(idleCycles, statistics::units::Cycle::get(),
               "Idle cycles"),
      ADD_STAT(stallCycles, statistics::units::Cycle::get(),
               "Stall cycles"),
      ADD_STAT(utilization, statistics::units::Ratio::get(),
               "SIMD unit utilization"),
      ADD_STAT(avgTaskLatency, statistics::units::Cycle::get(),
               "Average task latency")
{
    utilization = (totalCycles - idleCycles) / totalCycles;
    avgTaskLatency = totalCycles / totalTasks;
}

// ============== NOVASIMDCluster Implementation ==============

NOVASIMDCluster::NOVASIMDCluster(const Params &p) :
    SimObject(p),
    numUnits(p.num_simd_units)
{
}

bool NOVASIMDCluster::submitTask(const VisionTask &task)
{
    int unitId = selectUnit();
    if (unitId >= 0 && unitId < (int)simdUnits.size()) {
        return simdUnits[unitId]->submitTask(task);
    }
    return false;
}

int NOVASIMDCluster::selectUnit()
{
    static int nextUnit = 0;
    int selected = nextUnit;
    nextUnit = (nextUnit + 1) % numUnits;
    return selected;
}

// ============== NOVAVisionDispatcher Implementation ==============

NOVAVisionDispatcher::NOVAVisionDispatcher(const Params &p) :
    SimObject(p),
    maxQueueDepth(p.max_queue_depth),
    tileWidth(p.tile_width),
    tileHeight(p.tile_height)
{
}

void NOVAVisionDispatcher::dispatchImageProcessing(uint64_t srcAddr, uint64_t dstAddr,
                                                    uint32_t width, uint32_t height,
                                                    VisionOpType opType)
{
    VisionTask task;
    task.srcAddr = srcAddr;
    task.dstAddr = dstAddr;
    task.width = width;
    task.height = height;
    task.opType = opType;
    
    if (width <= (uint32_t)tileWidth && height <= (uint32_t)tileHeight) {
        pendingTasks.push(task);
    } else {
        tileAndDispatch(task);
    }
}

void NOVAVisionDispatcher::tileAndDispatch(const VisionTask &task)
{
    for (uint32_t y = 0; y < task.height; y += tileHeight) {
        for (uint32_t x = 0; x < task.width; x += tileWidth) {
            VisionTask tile;
            tile.opType = task.opType;
            tile.width = std::min((uint32_t)tileWidth, task.width - x);
            tile.height = std::min((uint32_t)tileHeight, task.height - y);
            tile.srcAddr = task.srcAddr + y * task.width + x;
            tile.dstAddr = task.dstAddr + y * task.width + x;
            tile.priority = task.priority;
            
            pendingTasks.push(tile);
        }
    }
}

} // namespace gem5
