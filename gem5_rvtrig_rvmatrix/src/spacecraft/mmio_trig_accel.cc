/*
 * NOVA Processor - MMIO-Based Trigonometric Accelerator Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/mmio_trig_accel.hh"

#include "base/trace.hh"
#include "debug/TrigAccel.hh"
#include "mem/packet_access.hh"

#include <cmath>

namespace gem5
{

namespace spacecraft
{

MMIOTrigAccel::MMIOTrigAccel(const Params &p)
    : BasicPioDevice(p, 0x100),  // 256 bytes of register space
      processingLatency(p.processing_latency),
      maxQueueDepth(p.max_queue_depth),
      isBusy(false),
      currentOp(OpType::NONE),
      inputA(0.0),
      inputB(0.0),
      outputA(0.0),
      outputB(0.0),
      resultReady(false),
      stats(this),
      processEvent([this]{ processComplete(); }, name())
{
    DPRINTF(TrigAccel, "MMIOTrigAccel created at address 0x%lx\n", 
            pioAddr);
}

MMIOTrigAccel::~MMIOTrigAccel()
{
}

void
MMIOTrigAccel::init()
{
    BasicPioDevice::init();
    DPRINTF(TrigAccel, "MMIOTrigAccel initialized\n");
}

AddrRangeList
MMIOTrigAccel::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

Tick
MMIOTrigAccel::read(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - pioAddr;
    
    DPRINTF(TrigAccel, "Read from offset 0x%lx\n", offset);
    
    uint64_t data = 0;
    
    switch (offset) {
      case 0x00:  // Control/Status
        // Bits: [0]=busy, [1]=done, [2]=error
        data = (isBusy ? 0x1 : 0x0) | (resultReady ? 0x2 : 0x0);
        break;
        
      case 0x18:  // Output A (result)
        data = *reinterpret_cast<uint64_t*>(&outputA);
        break;
        
      case 0x20:  // Output B (for sincos)
        data = *reinterpret_cast<uint64_t*>(&outputB);
        break;
        
      case 0x30:  // Queue depth
        data = requestQueue.size();
        break;
        
      default:
        data = 0;
        break;
    }
    
    pkt->setLE(data);
    pkt->makeResponse();
    
    return pioDelay;
}

Tick
MMIOTrigAccel::write(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - pioAddr;
    uint64_t data = pkt->getLE<uint64_t>();
    
    DPRINTF(TrigAccel, "Write 0x%lx to offset 0x%lx\n", data, offset);
    
    switch (offset) {
      case 0x00:  // Control register
        if (data & 0x01) {  // Start bit
            // Extract operation type from bits 4-7
            int opCode = (data >> 4) & 0x0F;
            OpType op = static_cast<OpType>(opCode);
            
            stats.totalRequests++;
            
            // Update operation-specific stats
            switch (op) {
              case OpType::SIN: stats.totalSinOps++; break;
              case OpType::COS: stats.totalCosOps++; break;
              case OpType::SINCOS: stats.totalSincosOps++; break;
              case OpType::ATAN:
              case OpType::ATAN2: stats.totalAtanOps++; break;
              default: break;
            }
            
            // Record queue depth
            size_t qd = requestQueue.size();
            stats.queueDepthDist.sample(qd);
            if (qd > static_cast<size_t>(stats.maxQueueDepthObserved.value())) {
                stats.maxQueueDepthObserved = qd;
            }
            
            if (isBusy) {
                // Accelerator is busy - queue the request
                if (requestQueue.size() < static_cast<size_t>(maxQueueDepth)) {
                    PendingRequest req;
                    req.op = op;
                    req.inputA = inputA;
                    req.inputB = inputB;
                    req.requestTime = curTick();
                    req.requesterId = 0;  // Could be extended to track core ID
                    
                    requestQueue.push(req);
                    stats.totalQueuedRequests++;
                    
                    DPRINTF(TrigAccel, "Request queued, queue depth = %d\n", 
                            requestQueue.size());
                } else {
                    DPRINTF(TrigAccel, "Queue full! Request dropped\n");
                }
            } else {
                // Start immediately
                currentOp = op;
                isBusy = true;
                resultReady = false;
                
                // Schedule completion
                schedule(processEvent, curTick() + 
                         cyclesToTicks(processingLatency));
                
                DPRINTF(TrigAccel, "Started operation %d, latency = %d cycles\n",
                        opCode, processingLatency);
            }
        }
        
        if (data & 0x8000) {  // Reset bit
            isBusy = false;
            resultReady = false;
            currentOp = OpType::NONE;
            while (!requestQueue.empty()) requestQueue.pop();
            DPRINTF(TrigAccel, "Accelerator reset\n");
        }
        break;
        
      case 0x08:  // Input A
        inputA = *reinterpret_cast<double*>(&data);
        break;
        
      case 0x10:  // Input B
        inputB = *reinterpret_cast<double*>(&data);
        break;
        
      case 0x28:  // Precision mode (ignored for now)
        break;
        
      default:
        break;
    }
    
    pkt->makeResponse();
    return pioDelay;
}

void
MMIOTrigAccel::processComplete()
{
    DPRINTF(TrigAccel, "Operation complete\n");
    
    // Execute the operation
    executeOperation();
    
    resultReady = true;
    isBusy = false;
    
    // Process next request if queue is not empty
    if (!requestQueue.empty()) {
        processNextRequest();
    }
}

void
MMIOTrigAccel::executeOperation()
{
    switch (currentOp) {
      case OpType::SIN:
        outputA = computeSin(inputA);
        break;
        
      case OpType::COS:
        outputA = computeCos(inputA);
        break;
        
      case OpType::SINCOS:
        outputA = computeSin(inputA);
        outputB = computeCos(inputA);
        break;
        
      case OpType::TAN:
        outputA = computeTan(inputA);
        break;
        
      case OpType::ATAN:
        outputA = computeAtan(inputA);
        break;
        
      case OpType::ATAN2:
        outputA = computeAtan2(inputB, inputA);
        break;
        
      case OpType::SQRT:
        outputA = computeSqrt(inputA);
        break;
        
      default:
        outputA = 0.0;
        break;
    }
    
    DPRINTF(TrigAccel, "Result: outputA = %f, outputB = %f\n", 
            outputA, outputB);
}

void
MMIOTrigAccel::processNextRequest()
{
    PendingRequest req = requestQueue.front();
    requestQueue.pop();
    
    // Calculate wait time for statistics
    Tick waitTime = curTick() - req.requestTime;
    stats.totalQueueWaitCycles += ticksToCycles(waitTime);
    
    // Set up the new operation
    inputA = req.inputA;
    inputB = req.inputB;
    currentOp = req.op;
    isBusy = true;
    resultReady = false;
    
    // Schedule completion
    schedule(processEvent, curTick() + cyclesToTicks(processingLatency));
    
    DPRINTF(TrigAccel, "Processing queued request, waited %d cycles\n",
            ticksToCycles(waitTime));
}

// CORDIC implementations (simplified - use standard lib for now)
double MMIOTrigAccel::computeSin(double angle)
{
    return std::sin(angle);
}

double MMIOTrigAccel::computeCos(double angle)
{
    return std::cos(angle);
}

double MMIOTrigAccel::computeTan(double angle)
{
    return std::tan(angle);
}

double MMIOTrigAccel::computeAtan(double x)
{
    return std::atan(x);
}

double MMIOTrigAccel::computeAtan2(double y, double x)
{
    return std::atan2(y, x);
}

double MMIOTrigAccel::computeSqrt(double x)
{
    return std::sqrt(x);
}

// Statistics
MMIOTrigAccel::AccelStats::AccelStats(MMIOTrigAccel *parent)
    : statistics::Group(parent),
      ADD_STAT(totalRequests, statistics::units::Count::get(),
               "Total number of requests"),
      ADD_STAT(totalSinOps, statistics::units::Count::get(),
               "Total sin operations"),
      ADD_STAT(totalCosOps, statistics::units::Count::get(),
               "Total cos operations"),
      ADD_STAT(totalSincosOps, statistics::units::Count::get(),
               "Total sincos operations"),
      ADD_STAT(totalAtanOps, statistics::units::Count::get(),
               "Total atan/atan2 operations"),
      ADD_STAT(totalQueuedRequests, statistics::units::Count::get(),
               "Requests that had to wait in queue (contention)"),
      ADD_STAT(totalQueueWaitCycles, statistics::units::Cycle::get(),
               "Total cycles spent waiting in queue"),
      ADD_STAT(maxQueueDepthObserved, statistics::units::Count::get(),
               "Maximum queue depth observed"),
      ADD_STAT(queueDepthDist, statistics::units::Count::get(),
               "Distribution of queue depth at request time"),
      ADD_STAT(requestLatency, statistics::units::Cycle::get(),
               "Histogram of request latency")
{
    queueDepthDist
        .init(0, 16, 1)
        .flags(statistics::nozero);
        
    requestLatency
        .init(20)
        .flags(statistics::nozero);
}

} // namespace spacecraft
} // namespace gem5

