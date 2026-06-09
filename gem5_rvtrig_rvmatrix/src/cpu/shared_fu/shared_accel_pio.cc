/*
 * NOVA Processor - Shared Accelerator PIO Device Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "cpu/shared_fu/shared_accel_pio.hh"

#include <cmath>

#include "base/trace.hh"
#include "debug/SharedFU.hh"
#include "mem/packet_access.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

SharedAccelPio::SharedAccelPio(const Params &p)
    : PioDevice(p),
      accelType(static_cast<AccelType>(p.accel_type)),
      instanceId(p.instance_id),
      pioAddr(p.pio_addr),
      trigLatency(Cycles(p.trig_latency)),
      matLatency(Cycles(p.mat_latency)),
      maxQueueDepth(p.max_queue_depth),
      regInput0(0),
      regInput1(0),
      regOutput0(0),
      regOutput1(0),
      busy(false),
      currentCpuId(-1),
      currentOp(OpType::SIN),
      operationStartTick(0),
      totalRequests(0),
      totalQueuedRequests(0),
      totalBusyCycles(0),
      peakQueueDepth(0),
      lastIdleTick(0),
      completeEvent([this]{ completeOperation(); }, name()),
      stats(this)
{
    DPRINTF(SharedFU, "SharedAccelPio created: type=%d, id=%d, addr=0x%lx\n",
            static_cast<int>(accelType), instanceId, pioAddr);
}

SharedAccelPio::~SharedAccelPio()
{
}

void
SharedAccelPio::init()
{
    PioDevice::init();
    DPRINTF(SharedFU, "SharedAccelPio[%d] initialized at 0x%lx\n",
            instanceId, pioAddr);
}

AddrRangeList
SharedAccelPio::getAddrRanges() const
{
    return { AddrRange(pioAddr, pioAddr + REG_SIZE) };
}

Tick
SharedAccelPio::read(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - pioAddr;
    uint64_t data = 0;
    
    DPRINTF(SharedFU, "Read: addr=0x%lx, offset=0x%lx\n", 
            pkt->getAddr(), offset);
    
    switch (offset) {
      case REG_CTRL:
        // Return accelerator type
        data = static_cast<uint64_t>(accelType);
        break;
        
      case REG_STATUS:
        // Status: busy in bit 0, queue depth in upper bits
        data = (busy ? 1 : 0) | (requestQueue.size() << 8);
        break;
        
      case REG_INPUT0:
        data = regInput0;
        break;
        
      case REG_INPUT1:
        data = regInput1;
        break;
        
      case REG_OUTPUT0:
        data = regOutput0;
        break;
        
      case REG_OUTPUT1:
        data = regOutput1;
        break;
        
      case REG_LATENCY:
        data = static_cast<uint64_t>(getOpLatency(currentOp));
        break;
        
      case REG_STATS:
        data = totalRequests | (totalQueuedRequests << 32);
        break;
        
      default:
        warn("SharedAccelPio: Invalid read offset 0x%lx\n", offset);
        data = 0;
        break;
    }
    
    pkt->setLE<uint64_t>(data);
    pkt->makeAtomicResponse();
    
    // Read always returns immediately
    return clockPeriod();
}

Tick
SharedAccelPio::write(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - pioAddr;
    uint64_t data = pkt->getLE<uint64_t>();
    
    DPRINTF(SharedFU, "Write: addr=0x%lx, offset=0x%lx, data=0x%lx\n",
            pkt->getAddr(), offset, data);
    
    totalRequests++;
    stats.requests++;
    
    switch (offset) {
      case REG_CTRL:
        {
            // Start operation
            OpType op = static_cast<OpType>(data & 0xFF);
            int cpuId = (data >> 8) & 0xFF;
            
            if (busy) {
                // Queue the request
                if (requestQueue.size() < maxQueueDepth) {
                    PendingRequest req;
                    req.pkt = pkt;
                    req.arrivalTime = curTick();
                    req.isWrite = true;
                    requestQueue.push(req);
                    
                    totalQueuedRequests++;
                    stats.queuedRequests++;
                    queueDepths.push_back(requestQueue.size());
                    
                    if (requestQueue.size() > peakQueueDepth) {
                        peakQueueDepth = requestQueue.size();
                    }
                    
                    DPRINTF(SharedFU, "Request queued: op=%d, depth=%ld\n",
                            static_cast<int>(op), requestQueue.size());
                    
                    // Return large latency - will be updated when complete
                    Cycles queueLatency = Cycles(requestQueue.size() * 
                                                 getOpLatency(op));
                    return clockPeriod() * queueLatency;
                } else {
                    // Queue full - reject (should not happen in well-designed sw)
                    warn("SharedAccelPio: Queue full, request dropped!\n");
                }
            } else {
                // Start immediately
                startOperation(op, cpuId);
            }
        }
        break;
        
      case REG_INPUT0:
        regInput0 = data;
        break;
        
      case REG_INPUT1:
        regInput1 = data;
        break;
        
      default:
        warn("SharedAccelPio: Invalid write offset 0x%lx\n", offset);
        break;
    }
    
    pkt->makeAtomicResponse();
    
    // Return operation latency
    if (busy && offset == REG_CTRL) {
        return clockPeriod() * getOpLatency(currentOp);
    }
    return clockPeriod();
}

void
SharedAccelPio::startOperation(OpType op, int cpuId)
{
    busy = true;
    currentCpuId = cpuId;
    currentOp = op;
    operationStartTick = curTick();
    
    DPRINTF(SharedFU, "Starting operation: op=%d, cpu=%d, latency=%ld cycles\n",
            static_cast<int>(op), cpuId, getOpLatency(op));
    
    // Schedule completion
    Tick completionTick = curTick() + clockPeriod() * getOpLatency(op);
    if (!completeEvent.scheduled()) {
        schedule(completeEvent, completionTick);
    }
}

void
SharedAccelPio::completeOperation()
{
    DPRINTF(SharedFU, "Operation complete: op=%d, cpu=%d\n",
            static_cast<int>(currentOp), currentCpuId);
    
    // Compute result
    computeResult(currentOp);
    
    // Track busy time
    totalBusyCycles += curTick() - operationStartTick;
    stats.busyCycles += (curTick() - operationStartTick) / clockPeriod();
    
    busy = false;
    currentCpuId = -1;
    lastIdleTick = curTick();
    
    // Process next queued request
    processNextRequest();
}

void
SharedAccelPio::processNextRequest()
{
    if (requestQueue.empty()) {
        return;
    }
    
    PendingRequest req = requestQueue.front();
    requestQueue.pop();
    
    // Calculate wait time
    Cycles waitCycles = Cycles((curTick() - req.arrivalTime) / clockPeriod());
    waitTimes.push_back(waitCycles);
    
    DPRINTF(SharedFU, "Processing queued request, waited %ld cycles\n",
            static_cast<uint64_t>(waitCycles));
    
    // Extract operation from packet
    if (req.isWrite) {
        uint64_t data = req.pkt->getLE<uint64_t>();
        OpType op = static_cast<OpType>(data & 0xFF);
        int cpuId = (data >> 8) & 0xFF;
        startOperation(op, cpuId);
    }
}

Cycles
SharedAccelPio::getOpLatency(OpType op) const
{
    switch (op) {
      case OpType::SIN:
      case OpType::COS:
      case OpType::TAN:
      case OpType::ATAN:
      case OpType::ATAN2:
      case OpType::SINCOS:
        return trigLatency;
        
      case OpType::MUL3x3:
        return matLatency;
      case OpType::TRANSPOSE:
        return Cycles(matLatency / 4);
      case OpType::INVERSE:
        return Cycles(matLatency * 2);
      case OpType::MATVEC:
        return Cycles(matLatency / 2);
        
      default:
        return trigLatency;
    }
}

void
SharedAccelPio::computeResult(OpType op)
{
    // Simplified computation for simulation purposes
    double input = *reinterpret_cast<double*>(&regInput0);
    double output;
    
    switch (op) {
      case OpType::SIN:
        output = std::sin(input);
        break;
      case OpType::COS:
        output = std::cos(input);
        break;
      case OpType::TAN:
        output = std::tan(input);
        break;
      case OpType::ATAN:
        output = std::atan(input);
        break;
      case OpType::ATAN2:
        {
            double input2 = *reinterpret_cast<double*>(&regInput1);
            output = std::atan2(input, input2);
        }
        break;
      case OpType::SINCOS:
        output = std::sin(input);
        regOutput1 = *reinterpret_cast<uint64_t*>(&output);
        output = std::cos(input);
        break;
      case OpType::MUL3x3:
      case OpType::TRANSPOSE:
      case OpType::INVERSE:
      case OpType::MATVEC:
        // Matrix ops - simplified
        output = input;
        break;
      default:
        output = input;
        break;
    }
    
    regOutput0 = *reinterpret_cast<uint64_t*>(&output);
}

// Statistics
SharedAccelPio::SharedAccelStats::SharedAccelStats(SharedAccelPio *parent)
    : statistics::Group(parent),
      ADD_STAT(requests, statistics::units::Count::get(),
               "Total accelerator requests"),
      ADD_STAT(queuedRequests, statistics::units::Count::get(),
               "Requests that were queued due to contention"),
      ADD_STAT(busyCycles, statistics::units::Cycle::get(),
               "Total cycles the accelerator was busy"),
      ADD_STAT(avgQueueDepth, statistics::units::Ratio::get(),
               "Average queue depth when requests arrived"),
      ADD_STAT(avgWaitCycles, statistics::units::Cycle::get(),
               "Average wait cycles for queued requests"),
      ADD_STAT(queueDepthHist, statistics::units::Count::get(),
               "Histogram of queue depths"),
      ADD_STAT(waitTimeHist, statistics::units::Cycle::get(),
               "Histogram of wait times")
{
    queueDepthHist.init(16);  // 0-15 queue depth
    waitTimeHist.init(20);    // Wait time buckets
}

} // namespace gem5


