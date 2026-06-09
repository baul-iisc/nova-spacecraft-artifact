/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010-2013,2015,2017-2018, 2020-2021 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/simple/timing.hh"

#include <atomic>
#include <cstdlib>

#include "arch/generic/decoder.hh"
#include "base/compiler.hh"
#include "cpu/exetrace.hh"
#include "debug/Config.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/HtmCpu.hh"
#include "debug/Mwait.hh"
#include "debug/SimpleCPU.hh"
#include "enums/OpClass.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/BaseTimingSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

namespace gem5
{

// Forward declare the operation counters (defined later)
extern std::atomic<uint64_t> trigOpsDetected;
extern std::atomic<uint64_t> matOpsDetected;
extern std::atomic<uint64_t> vpuOpsDetected;
extern std::atomic<uint64_t> npuOpsDetected;
extern std::atomic<uint64_t> gpuOpsDetected;

void
TimingSimpleCPU::init()
{
    BaseSimpleCPU::init();
    
    // NOVA: Reset shared FU stats at simulation start (only once)
    static bool initialized = false;
    if (!initialized) {
        std::lock_guard<std::mutex> lock(sharedFUMutex);
        
        // Read configuration from environment variables
        const char* trigEnv = std::getenv("NOVA_NUM_TRIG_ACCELS");
        const char* matEnv = std::getenv("NOVA_NUM_MAT_ACCELS");
        
        if (trigEnv) {
            numTrigAccels = std::atoi(trigEnv);
        }
        if (matEnv) {
            numMatAccels = std::atoi(matEnv);
        }
        
        // Read VPU and NPU configuration from environment
        const char* vpuEnv = std::getenv("NOVA_NUM_VPU_ACCELS");
        const char* npuEnv = std::getenv("NOVA_NUM_NPU_ACCELS");
        const char* gpuEnv = std::getenv("NOVA_NUM_GPU_ACCELS");
        
        if (vpuEnv) {
            numVpuAccels = std::atoi(vpuEnv);
        }
        if (npuEnv) {
            numNpuAccels = std::atoi(npuEnv);
        }
        if (gpuEnv) {
            numGpuAccels = std::atoi(gpuEnv);
        }

        // Read per-type latency overrides from environment
        const char* vpuLatEnv = std::getenv("NOVA_VPU_LATENCY");
        const char* npuLatEnv = std::getenv("NOVA_NPU_LATENCY");
        const char* gpuLatEnv = std::getenv("NOVA_GPU_LATENCY");
        
        if (vpuLatEnv) {
            vpuLatency = Cycles(std::atoi(vpuLatEnv));
        }
        if (npuLatEnv) {
            npuLatency = Cycles(std::atoi(npuLatEnv));
        }
        if (gpuLatEnv) {
            gpuLatency = Cycles(std::atoi(gpuLatEnv));
        }

        // IEEE-Transactions-on-Computers artifact mode: pin each accelerator type
        // to a single blocking instance regardless of NOVA_NUM_* (latency env vars
        // above still apply). Use for reproducible baseline / McPAT-style accounting.
        const char *ieeeTc = std::getenv("GEM5_IEEE_TC_ARTIFACT");
        if (ieeeTc && ieeeTc[0] == '1' && ieeeTc[1] == '\0') {
            numTrigAccels = 1;
            numMatAccels = 1;
            numVpuAccels = 1;
            numNpuAccels = 1;
            numGpuAccels = 1;
            warn("GEM5_IEEE_TC_ARTIFACT=1: one instance per accelerator type "
                 "(Trig/Mat/VPU/NPU/GPU); NOVA_NUM_* instance counts ignored.\n");
        }
        
        // Set per-type instance counts
        sharedFUState[0].numInstances = numTrigAccels;
        sharedFUState[1].numInstances = numMatAccels;
        sharedFUState[2].numInstances = numVpuAccels;
        sharedFUState[3].numInstances = numNpuAccels;
        sharedFUState[4].numInstances = numGpuAccels;
        
        for (auto& state : sharedFUState) {
            // Clamp numInstances to valid range
            if (state.numInstances < 1) state.numInstances = 1;
            if (state.numInstances > MAX_ACCEL_INSTANCES)
                state.numInstances = MAX_ACCEL_INSTANCES;
            // Initialize per-instance busy timestamps
            for (int i = 0; i < MAX_ACCEL_INSTANCES; i++) {
                state.instanceBusyUntil[i] = 0;
            }
            state.totalRequests = 0;
            state.queuedRequests = 0;
            state.totalWaitCycles = 0;
            state.totalWaitTicks = 0;
            state.totalStallTicks = 0;
            state.contentionRangeStart = 0;
            state.contentionRangeEnd = 0;
            state.totalContentionTicks = 0;
        }
        trigOpsDetected = 0;
        matOpsDetected = 0;
        vpuOpsDetected = 0;
        npuOpsDetected = 0;
        gpuOpsDetected = 0;
        initialized = true;
        warn("NOVA v3.1: MULTI-INSTANCE SHARED ACCELERATORS (BUS-ATTACHED, BLOCKING)\n");
        warn("NOVA v3.1: TrigAccels=%d (lat=%d), MatAccels=%d (lat=%d), VPU=%d (lat=%d), NPU=%d (lat=%d), GPU=%d (lat=%d)\n",
             numTrigAccels, (int)trigLatency, numMatAccels, (int)matLatency,
             numVpuAccels, (int)vpuLatency, numNpuAccels, (int)npuLatency,
             numGpuAccels, (int)gpuLatency);
        warn("NOVA v3.1: Each type has independent parallel instances for contention modeling\n");
    }
}

void
TimingSimpleCPU::TimingCPUPort::TickEvent::schedule(PacketPtr _pkt, Tick t)
{
    pkt = _pkt;
    cpu->schedule(this, t);
}

TimingSimpleCPU::TimingSimpleCPU(const BaseTimingSimpleCPUParams &p)
    : BaseSimpleCPU(p), fetchTranslation(this), icachePort(this),
      dcachePort(this), ifetch_pkt(NULL), dcache_pkt(NULL), previousCycle(0),
      fetchEvent([this]{ fetch(); }, name()),
      sharedFUResumeEvent([this]{ resumeAfterSharedFUStall(); }, name())
{
    _status = Idle;
    
    // NOVA v2.0: Initialize shared FU stall state
    sharedFUStalled = false;
    stalledForFUType = -1;
    stallUntilTick = 0;
    pendingStallTicks = 0;
}



TimingSimpleCPU::~TimingSimpleCPU()
{
}

DrainState
TimingSimpleCPU::drain()
{
    // Deschedule any power gating event (if any)
    deschedulePowerGatingEvent();

    if (switchedOut())
        return DrainState::Drained;

    if (_status == Idle ||
        (_status == BaseSimpleCPU::Running && isCpuDrained())) {
        DPRINTF(Drain, "No need to drain.\n");
        activeThreads.clear();
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "Requesting drain.\n");

        // The fetch event can become descheduled if a drain didn't
        // succeed on the first attempt. We need to reschedule it if
        // the CPU is waiting for a microcode routine to complete.
        if (_status == BaseSimpleCPU::Running && !fetchEvent.scheduled())
            schedule(fetchEvent, clockEdge());

        return DrainState::Draining;
    }
}

void
TimingSimpleCPU::drainResume()
{
    assert(!fetchEvent.scheduled());
    if (switchedOut())
        return;

    DPRINTF(SimpleCPU, "Resume\n");
    verifyMemoryMode();

    assert(!threadContexts.empty());

    _status = BaseSimpleCPU::Idle;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (threadInfo[tid]->thread->status() == ThreadContext::Active) {
            threadInfo[tid]->execContextStats.notIdleFraction = 1;

            activeThreads.push_back(tid);

            _status = BaseSimpleCPU::Running;

            // Fetch if any threads active
            if (!fetchEvent.scheduled()) {
                schedule(fetchEvent, nextCycle());
            }
        } else {
            threadInfo[tid]->execContextStats.notIdleFraction = 0;
        }
    }

    // Reschedule any power gating event (if any)
    schedulePowerGatingEvent();
}

bool
TimingSimpleCPU::tryCompleteDrain()
{
    if (drainState() != DrainState::Draining)
        return false;

    DPRINTF(Drain, "tryCompleteDrain.\n");
    if (!isCpuDrained())
        return false;

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    signalDrainDone();

    return true;
}

void
TimingSimpleCPU::switchOut()
{
    SimpleExecContext& t_info = *threadInfo[curThread];
    [[maybe_unused]] SimpleThread* thread = t_info.thread;

    // hardware transactional memory
    // Cannot switch out the CPU in the middle of a transaction
    assert(!t_info.inHtmTransactionalState());

    BaseSimpleCPU::switchOut();

    assert(!fetchEvent.scheduled());
    assert(_status == BaseSimpleCPU::Running || _status == Idle);
    assert(!t_info.stayAtPC);
    assert(thread->pcState().microPC() == 0);

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);
}


void
TimingSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseSimpleCPU::takeOverFrom(oldCPU);

    previousCycle = curCycle();
}

void
TimingSimpleCPU::verifyMemoryMode() const
{
    if (!system->isTimingMode()) {
        fatal("The timing CPU requires the memory system to be in "
              "'timing' mode.\n");
    }
}

void
TimingSimpleCPU::activateContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "ActivateContext %d\n", thread_num);

    assert(thread_num < numThreads);

    threadInfo[thread_num]->execContextStats.notIdleFraction = 1;
    if (_status == BaseSimpleCPU::Idle)
        _status = BaseSimpleCPU::Running;

    // kick things off by initiating the fetch of the next instruction
    if (!fetchEvent.scheduled())
        schedule(fetchEvent, clockEdge(Cycles(0)));

    if (std::find(activeThreads.begin(), activeThreads.end(), thread_num)
         == activeThreads.end()) {
        activeThreads.push_back(thread_num);
    }

    BaseCPU::activateContext(thread_num);
}


void
TimingSimpleCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num < numThreads);
    activeThreads.remove(thread_num);

    // hardware transactional memory
    // Cannot suspend context in the middle of a transaction.
    assert(!threadInfo[curThread]->inHtmTransactionalState());

    if (_status == Idle)
        return;

    // The CPU may be in various states when a thread exits via tgkill/exit:
    //   Running, DcacheWaitResponse, DTBWaitResponse, DcacheRetry,
    //   IcacheWaitResponse, or stalled for a shared FU.
    // All of these are valid states from which a thread can be suspended.
    // (The original assertion only allowed Running, causing aborts when
    //  threads exit while the CPU has an outstanding memory request.)

    threadInfo[thread_num]->execContextStats.notIdleFraction = 0;

    // If this CPU was stalled waiting for a shared accelerator, clean up
    if (sharedFUStalled) {
        sharedFUStalled = false;
        stalledForFUType = -1;
        stallUntilTick = 0;
        if (sharedFUResumeEvent.scheduled()) {
            deschedule(sharedFUResumeEvent);
        }
    }

    if (activeThreads.empty()) {
        _status = Idle;

        if (fetchEvent.scheduled()) {
            deschedule(fetchEvent);
        }
    }

    BaseCPU::suspendContext(thread_num);
}

bool
TimingSimpleCPU::handleReadPacket(PacketPtr pkt)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const RequestPtr &req = pkt->req;

    // hardware transactional memory
    // sanity check
    if (req->isHTMCmd()) {
        assert(!req->isLocalAccess());
    }

    // We're about the issues a locked load, so tell the monitor
    // to start caring about this address
    if (pkt->isRead() && pkt->req->isLLSC()) {
        thread->getIsaPtr()->handleLockedRead(pkt->req);
    }
    if (req->isLocalAccess()) {
        Cycles delay = req->localAccessor(thread->getTC(), pkt);
        new IprEvent(pkt, this, clockEdge(delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else if (!dcachePort.sendTimingReq(pkt)) {
        _status = DcacheRetry;
        dcache_pkt = pkt;
    } else {
        _status = DcacheWaitResponse;
        // memory system takes ownership of packet
        dcache_pkt = NULL;
    }
    return dcache_pkt == NULL;
}

void
TimingSimpleCPU::sendData(const RequestPtr &req, uint8_t *data, uint64_t *res,
                          bool read)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    PacketPtr pkt = buildPacket(req, read);
    pkt->dataDynamic<uint8_t>(data);

    // hardware transactional memory
    // If the core is in transactional mode or if the request is HtmCMD
    // to abort a transaction, the packet should reflect that it is
    // transactional and also contain a HtmUid for debugging.
    const bool is_htm_speculative = t_info.inHtmTransactionalState();
    if (is_htm_speculative || req->isHTMAbort()) {
        pkt->setHtmTransactional(t_info.getHtmTransactionUid());
    }
    if (req->isHTMAbort())
        DPRINTF(HtmCpu, "htmabort htmUid=%u\n", t_info.getHtmTransactionUid());

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt->makeResponse();
        completeDataAccess(pkt);
    } else if (read) {
        handleReadPacket(pkt);
    } else {
        bool do_access = true;  // flag to suppress cache access

        if (req->isLLSC()) {
            do_access = thread->getIsaPtr()->handleLockedWrite(
                    req, dcachePort.cacheBlockMask);
        } else if (req->isCondSwap()) {
            assert(res);
            req->setExtraData(*res);
        }

        if (do_access) {
            dcache_pkt = pkt;
            handleWritePacket();
            threadSnoop(pkt, curThread);
        } else {
            _status = DcacheWaitResponse;
            completeDataAccess(pkt);
        }
    }
}

void
TimingSimpleCPU::sendSplitData(const RequestPtr &req1, const RequestPtr &req2,
                               const RequestPtr &req, uint8_t *data, bool read)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    PacketPtr pkt1, pkt2;
    buildSplitPacket(pkt1, pkt2, req1, req2, req, data, read);

    // hardware transactional memory
    // HTM commands should never use SplitData
    assert(!req1->isHTMCmd() && !req2->isHTMCmd());

    // If the thread is executing transactionally,
    // reflect this in the packets.
    if (t_info.inHtmTransactionalState()) {
        pkt1->setHtmTransactional(t_info.getHtmTransactionUid());
        pkt2->setHtmTransactional(t_info.getHtmTransactionUid());
    }

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt1->makeResponse();
        completeDataAccess(pkt1);
    } else if (read) {
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleReadPacket(pkt1)) {
            send_state->clearFromParent();
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleReadPacket(pkt2)) {
                send_state->clearFromParent();
            }
        }
    } else {
        dcache_pkt = pkt1;
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleWritePacket()) {
            send_state->clearFromParent();
            dcache_pkt = pkt2;
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleWritePacket()) {
                send_state->clearFromParent();
            }
        }
    }
}

void
TimingSimpleCPU::translationFault(const Fault &fault)
{
    // fault may be NoFault in cases where a fault is suppressed,
    // for instance prefetches.
    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if ((fault != NoFault) && traceData) {
        traceFault();
    }

    postExecute();

    advanceInst(fault);
}

PacketPtr
TimingSimpleCPU::buildPacket(const RequestPtr &req, bool read)
{
    return read ? Packet::createRead(req) : Packet::createWrite(req);
}

void
TimingSimpleCPU::buildSplitPacket(PacketPtr &pkt1, PacketPtr &pkt2,
        const RequestPtr &req1, const RequestPtr &req2, const RequestPtr &req,
        uint8_t *data, bool read)
{
    pkt1 = pkt2 = NULL;

    assert(!req1->isLocalAccess() && !req2->isLocalAccess());

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        pkt1 = buildPacket(req, read);
        return;
    }

    pkt1 = buildPacket(req1, read);
    pkt2 = buildPacket(req2, read);

    PacketPtr pkt = new Packet(req, pkt1->cmd.responseCommand());

    pkt->dataDynamic<uint8_t>(data);
    pkt1->dataStatic<uint8_t>(data);
    pkt2->dataStatic<uint8_t>(data + req1->getSize());

    SplitMainSenderState * main_send_state = new SplitMainSenderState;
    pkt->senderState = main_send_state;
    main_send_state->fragments[0] = pkt1;
    main_send_state->fragments[1] = pkt2;
    main_send_state->outstanding = 2;
    pkt1->senderState = new SplitFragmentSenderState(pkt, 0);
    pkt2->senderState = new SplitFragmentSenderState(pkt, 1);
}

Fault
TimingSimpleCPU::initiateMemRead(Addr addr, unsigned size,
                                 Request::Flags flags,
                                 const std::vector<bool>& byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Fault fault;
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Read;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId(), pc, thread->contextId());
    req->setByteEnable(byte_enable);

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;
    if (split_addr > addr) {
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, new uint8_t[size],
                                      NULL, mode);
        DataTranslation<TimingSimpleCPU *> *trans1 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 0);
        DataTranslation<TimingSimpleCPU *> *trans2 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 1);

        thread->mmu->translateTiming(req1, thread->getTC(), trans1, mode);
        thread->mmu->translateTiming(req2, thread->getTC(), trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, new uint8_t[size], NULL, mode);
        DataTranslation<TimingSimpleCPU *> *translation
            = new DataTranslation<TimingSimpleCPU *>(this, state);
        thread->mmu->translateTiming(req, thread->getTC(), translation, mode);
    }

    return NoFault;
}

bool
TimingSimpleCPU::handleWritePacket()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const RequestPtr &req = dcache_pkt->req;
    if (req->isLocalAccess()) {
        Cycles delay = req->localAccessor(thread->getTC(), dcache_pkt);
        new IprEvent(dcache_pkt, this, clockEdge(delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else if (!dcachePort.sendTimingReq(dcache_pkt)) {
        _status = DcacheRetry;
    } else {
        _status = DcacheWaitResponse;
        // memory system takes ownership of packet
        dcache_pkt = NULL;
    }
    return dcache_pkt == NULL;
}

Fault
TimingSimpleCPU::writeMem(uint8_t *data, unsigned size,
                          Addr addr, Request::Flags flags, uint64_t *res,
                          const std::vector<bool>& byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    uint8_t *newData = new uint8_t[size];
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Write;

    if (data == NULL) {
        assert(flags & Request::STORE_NO_DATA);
        // This must be a cache block cleaning request
        memset(newData, 0, size);
    } else {
        memcpy(newData, data, size);
    }

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId(), pc, thread->contextId());
    req->setByteEnable(byte_enable);

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;

    // TODO: TimingSimpleCPU doesn't support arbitrarily long multi-line mem.
    // accesses yet

    if (split_addr > addr) {
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, newData, res, mode);
        DataTranslation<TimingSimpleCPU *> *trans1 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 0);
        DataTranslation<TimingSimpleCPU *> *trans2 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 1);

        thread->mmu->translateTiming(req1, thread->getTC(), trans1, mode);
        thread->mmu->translateTiming(req2, thread->getTC(), trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, newData, res, mode);
        DataTranslation<TimingSimpleCPU *> *translation =
            new DataTranslation<TimingSimpleCPU *>(this, state);
        thread->mmu->translateTiming(req, thread->getTC(), translation, mode);
    }

    // Translation faults will be returned via finishTranslation()
    return NoFault;
}

Fault
TimingSimpleCPU::initiateMemAMO(Addr addr, unsigned size,
                                Request::Flags flags,
                                AtomicOpFunctorPtr amo_op)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Fault fault;
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Write;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(addr, size, flags,
                            dataRequestorId(), pc, thread->contextId(),
                            std::move(amo_op));

    assert(req->hasAtomicOpFunctor());

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);

    // AMO requests that access across a cache line boundary are not
    // allowed since the cache does not guarantee AMO ops to be executed
    // atomically in two cache lines
    // For ISAs such as x86 that requires AMO operations to work on
    // accesses that cross cache-line boundaries, the cache needs to be
    // modified to support locking both cache lines to guarantee the
    // atomicity.
    if (split_addr > addr) {
        panic("AMO requests should not access across a cache line boundary\n");
    }

    _status = DTBWaitResponse;

    WholeTranslationState *state =
        new WholeTranslationState(req, new uint8_t[size], NULL, mode);
    DataTranslation<TimingSimpleCPU *> *translation
        = new DataTranslation<TimingSimpleCPU *>(this, state);
    thread->mmu->translateTiming(req, thread->getTC(), translation, mode);

    return NoFault;
}

void
TimingSimpleCPU::threadSnoop(PacketPtr pkt, ThreadID sender)
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (tid != sender) {
            if (getCpuAddrMonitor(tid)->doMonitor(pkt)) {
                wakeup(tid);
            }
            threadInfo[tid]->thread->getIsaPtr()->handleLockedSnoop(pkt,
                    dcachePort.cacheBlockMask);
        }
    }
}

void
TimingSimpleCPU::finishTranslation(WholeTranslationState *state)
{
    _status = BaseSimpleCPU::Running;

    if (state->getFault() != NoFault) {
        if (state->isPrefetch()) {
            state->setNoFault();
        }
        delete [] state->data;
        state->deleteReqs();
        translationFault(state->getFault());
    } else {
        if (!state->isSplit) {
            sendData(state->mainReq, state->data, state->res,
                     state->mode == BaseMMU::Read);
        } else {
            sendSplitData(state->sreqLow, state->sreqHigh, state->mainReq,
                          state->data, state->mode == BaseMMU::Read);
        }
    }

    delete state;
}


void
TimingSimpleCPU::fetch()
{
    // NOVA v2.0: Apply pending stall from shared FU contention
    // If we have accumulated stall ticks, delay this fetch
    if (pendingStallTicks > 0) {
        Tick stallTime = pendingStallTicks;
        pendingStallTicks = 0;  // Clear pending stall
        
        DPRINTF(SimpleCPU, "NOVA v2: Applying %lu stall ticks before fetch\n", stallTime);
        
        // Reschedule this fetch for later
        if (!fetchEvent.scheduled()) {
            schedule(fetchEvent, curTick() + stallTime);
        } else {
            reschedule(fetchEvent, curTick() + stallTime, true);
        }
        return;  // Will resume via fetchEvent
    }
    
    // Change thread if multi-threaded
    swapActiveThread();

    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    DPRINTF(SimpleCPU, "Fetch\n");

    if (!curStaticInst || !curStaticInst->isDelayedCommit()) {
        checkForInterrupts();
        checkPcEventQueue();
    }

    // We must have just got suspended by a PC event
    if (_status == Idle)
        return;

    MicroPC upc = thread->pcState().microPC();
    bool needToFetch = !isRomMicroPC(upc) && !curMacroStaticInst;

    if (needToFetch) {
        _status = BaseSimpleCPU::Running;
        RequestPtr ifetch_req = std::make_shared<Request>();
        ifetch_req->taskId(taskId());
        ifetch_req->setContext(thread->contextId());
        setupFetchRequest(ifetch_req);
        DPRINTF(SimpleCPU, "Translating address %#x\n", ifetch_req->getVaddr());
        thread->mmu->translateTiming(ifetch_req, thread->getTC(),
                &fetchTranslation, BaseMMU::Execute);
    } else {
        _status = IcacheWaitResponse;
        completeIfetch(NULL);

        updateCycleCounts();
        updateCycleCounters(BaseCPU::CPU_STATE_ON);
    }
}


void
TimingSimpleCPU::sendFetch(const Fault &fault, const RequestPtr &req,
                           ThreadContext *tc)
{
    auto &decoder = threadInfo[curThread]->thread->decoder;

    if (fault == NoFault) {
        DPRINTF(SimpleCPU, "Sending fetch for addr %#x(pa: %#x)\n",
                req->getVaddr(), req->getPaddr());
        ifetch_pkt = new Packet(req, MemCmd::ReadReq);
        ifetch_pkt->dataStatic(decoder->moreBytesPtr());
        DPRINTF(SimpleCPU, " -- pkt addr: %#x\n", ifetch_pkt->getAddr());

        if (!icachePort.sendTimingReq(ifetch_pkt)) {
            // Need to wait for retry
            _status = IcacheRetry;
        } else {
            // Need to wait for cache to respond
            _status = IcacheWaitResponse;
            // ownership of packet transferred to memory system
            ifetch_pkt = NULL;
        }
    } else {
        DPRINTF(SimpleCPU, "Translation of addr %#x faulted\n", req->getVaddr());
        // fetch fault: advance directly to next instruction (fault handler)
        _status = BaseSimpleCPU::Running;
        advanceInst(fault);
    }

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);
}


void
TimingSimpleCPU::advanceInst(const Fault &fault)
{
    SimpleExecContext &t_info = *threadInfo[curThread];

    if (_status == Faulting)
        return;

    if (fault != NoFault) {
        // hardware transactional memory
        // If a fault occurred within a transaction
        // ensure that the transaction aborts
        if (t_info.inHtmTransactionalState() &&
            !std::dynamic_pointer_cast<GenericHtmFailureFault>(fault)) {
            DPRINTF(HtmCpu, "fault (%s) occurred - "
                "replacing with HTM abort fault htmUid=%u\n",
                fault->name(), t_info.getHtmTransactionUid());

            Fault tmfault = std::make_shared<GenericHtmFailureFault>(
                t_info.getHtmTransactionUid(),
                HtmFailureFaultCause::EXCEPTION);

            advancePC(tmfault);
            reschedule(fetchEvent, clockEdge(), true);
            _status = Faulting;
            return;
        }

        DPRINTF(SimpleCPU, "Fault occured. Handling the fault\n");

        advancePC(fault);

        // A syscall fault could suspend this CPU (e.g., futex_wait)
        // If the _status is not Idle, schedule an event to fetch the next
        // instruction after 'stall' ticks.
        // If the cpu has been suspended (i.e., _status == Idle), another
        // cpu will wake this cpu up later.
        if (_status != Idle) {
            DPRINTF(SimpleCPU, "Scheduling fetch event after the Fault\n");

            Tick stall = std::dynamic_pointer_cast<SyscallRetryFault>(fault) ?
                         clockEdge(syscallRetryLatency) : clockEdge();
            reschedule(fetchEvent, stall, true);
            _status = Faulting;
        }

        return;
    }

    if (!t_info.stayAtPC)
        advancePC(fault);

    if (tryCompleteDrain())
        return;

    serviceInstCountEvents();

    if (_status == BaseSimpleCPU::Running) {
        // kick off fetch of next instruction... callback from icache
        // response will cause that instruction to be executed,
        // keeping the CPU running.
        fetch();
    }
}


void
TimingSimpleCPU::completeIfetch(PacketPtr pkt)
{
    SimpleExecContext& t_info = *threadInfo[curThread];

    DPRINTF(SimpleCPU, "Complete ICache Fetch for addr %#x\n", pkt ?
            pkt->getAddr() : 0);

    // received a response from the icache: execute the received
    // instruction
    panic_if(pkt && pkt->isError(), "Instruction fetch (%s) failed: %s",
            pkt->getAddrRange().to_string(), pkt->print());
    
    // NOVA v3.1 FIX: Instead of hard assert, handle the case where
    // _status != IcacheWaitResponse gracefully. This can occur when
    // a shared FU stall + resume creates a timing window where a
    // duplicate completeIfetch callback fires.
    if (_status != IcacheWaitResponse) {
        warn_once("NOVA v3.1 FIX: completeIfetch called with _status=%d "
                  "(expected IcacheWaitResponse=%d) at tick %lu, CPU%d. "
                  "sharedFUStalled=%d. Recovering by skipping duplicate.\n",
                  (int)_status, (int)IcacheWaitResponse, curTick(), cpuId(),
                  (int)sharedFUStalled);
        // If we're stalled for a shared FU, the resume event will handle
        // the instruction execution. Skip this duplicate callback.
        if (pkt) {
            delete pkt;
        }
        return;
    }

    _status = BaseSimpleCPU::Running;

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if (pkt)
        pkt->req->setAccessLatency();


    preExecute();

    // hardware transactional memory
    if (curStaticInst && curStaticInst->isHtmStart()) {
        // if this HtmStart is not within a transaction,
        // then assign it a new htmTransactionUid
        if (!t_info.inHtmTransactionalState())
            t_info.newHtmTransactionUid();
        SimpleThread* thread = t_info.thread;
        thread->htmTransactionStarts++;
        DPRINTF(HtmCpu, "htmTransactionStarts++=%u\n",
            thread->htmTransactionStarts);
    }

    if (curStaticInst && curStaticInst->isMemRef()) {
        // load or store: just send to dcache
        Fault fault = curStaticInst->initiateAcc(&t_info, traceData);

        // If we're not running now the instruction will complete in a dcache
        // response callback or the instruction faulted and has started an
        // ifetch
        if (_status == BaseSimpleCPU::Running) {
            if (fault != NoFault && traceData) {
                traceFault();
            }

            postExecute();
            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;
            advanceInst(fault);
        }
    } else if (curStaticInst) {
        // non-memory instruction: execute completely now
        
        // NOVA v3.0: SHARED ACCELERATOR MODEL
        // Check if this instruction uses a shared functional unit
        // If accelerator is busy, STALL THE CPU until it's free
        bool usesAccel = usesSharedFU(curStaticInst);
        int fuType = -1;
        Tick stallTicks = 0;
        
        if (usesAccel) {
            fuType = getSharedFUType(curStaticInst);
            // tryAcquireSharedFU returns stall ticks if FU is busy
            stallTicks = tryAcquireSharedFU(fuType);
            
            // NOVA v3.0: If accelerator is busy, BLOCK execution
            // Schedule resume event for when accelerator is available
            if (stallTicks > 0) {
                DPRINTF(SimpleCPU, "NOVA v3: CPU%d BLOCKED waiting %lu ticks for "
                        "shared accelerator type %d\n", cpuId(), stallTicks, fuType);
                
                // Store the pending instruction state for resume
                stalledForFUType = fuType;
                sharedFUStalled = true;
                stallUntilTick = curTick() + stallTicks;
                
                // NOVA v3.1 FIX: Set _status to a "busy" state so that
                // no other code path (activateContext, syscall resume, etc.)
                // can schedule a conflicting fetchEvent while we're stalled.
                // IcacheWaitResponse is reused here as a "blocked" indicator
                // since the CPU must not fetch until the stall resolves.
                _status = IcacheWaitResponse;
                
                // Deschedule any pending fetch to prevent duplicate
                // completeIfetch callbacks that would hit the
                // assert(_status == IcacheWaitResponse) on re-entry.
                if (fetchEvent.scheduled()) {
                    deschedule(fetchEvent);
                }
                
                // Schedule resume event for when accelerator is available
                // This uses the proper resume path that executes without re-acquiring
                if (!sharedFUResumeEvent.scheduled()) {
                    schedule(sharedFUResumeEvent, curTick() + stallTicks);
                } else {
                    reschedule(sharedFUResumeEvent, curTick() + stallTicks, true);
                }
                
                // Don't execute now - resumeAfterSharedFUStall will handle it
                return;
            }
        }
        
        // Accelerator is available (or instruction doesn't use accelerator)
        // Execute the instruction now
        Fault fault = curStaticInst->execute(&t_info, traceData);

        // keep an instruction count
        if (fault == NoFault)
            countInst();
        else if (traceData) {
            traceFault();
        }

        postExecute();
        // @todo remove me after debugging with legion done
        if (curStaticInst && (!curStaticInst->isMicroop() ||
                curStaticInst->isFirstMicroop()))
            instCnt++;
        
        // NOVA: Release shared FU after operation completes
        // The accelerator latency is modeled by the stall above
        if (usesAccel && fuType >= 0) {
            releaseSharedFU(fuType);
        }
        
        advanceInst(fault);
    } else {
        advanceInst(NoFault);
    }

    if (pkt) {
        delete pkt;
    }
}

void
TimingSimpleCPU::IcachePort::ITickEvent::process()
{
    cpu->completeIfetch(pkt);
}

bool
TimingSimpleCPU::IcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "Received fetch response %#x\n", pkt->getAddr());

    // hardware transactional memory
    // Currently, there is no support for tracking instruction fetches
    // in an transaction's read set.
    if (pkt->htmTransactionFailedInCache()) {
        panic("HTM transactional support for"
              " instruction stream not yet supported\n");
    }

    // we should only ever see one response per cycle since we only
    // issue a new request once this response is sunk
    assert(!tickEvent.scheduled());
    // delay processing of returned data until next CPU clock edge
    tickEvent.schedule(pkt, cpu->clockEdge());

    return true;
}

void
TimingSimpleCPU::IcachePort::recvReqRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->ifetch_pkt != NULL);
    assert(cpu->_status == IcacheRetry);
    PacketPtr tmp = cpu->ifetch_pkt;
    if (sendTimingReq(tmp)) {
        cpu->_status = IcacheWaitResponse;
        cpu->ifetch_pkt = NULL;
    }
}

void
TimingSimpleCPU::completeDataAccess(PacketPtr pkt)
{
    // hardware transactional memory

    SimpleExecContext *t_info = threadInfo[curThread];
    [[maybe_unused]] const bool is_htm_speculative =
        t_info->inHtmTransactionalState();

    // received a response from the dcache: complete the load or store
    // instruction
    panic_if(pkt->isError(), "Data access (%s) failed: %s",
            pkt->getAddrRange().to_string(), pkt->print());
    assert(_status == DcacheWaitResponse || _status == DTBWaitResponse ||
           pkt->req->getFlags().isSet(Request::NO_ACCESS));

    pkt->req->setAccessLatency();

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if (pkt->senderState) {
        // hardware transactional memory
        // There shouldn't be HtmCmds occurring in multipacket requests
        if (pkt->req->isHTMCmd()) {
            panic("unexpected HTM case");
        }

        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt->senderState);
        assert(send_state);
        PacketPtr big_pkt = send_state->bigPkt;
        delete send_state;

        if (pkt->isHtmTransactional()) {
            assert(is_htm_speculative);

            big_pkt->setHtmTransactional(
                pkt->getHtmTransactionUid()
            );
        }

        if (pkt->htmTransactionFailedInCache()) {
            assert(is_htm_speculative);
            big_pkt->setHtmTransactionFailedInCache(
                pkt->getHtmTransactionFailedInCacheRC()
            );
        }

        delete pkt;

        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);
        // Record the fact that this packet is no longer outstanding.
        assert(main_send_state->outstanding != 0);
        main_send_state->outstanding--;

        if (main_send_state->outstanding) {
            return;
        } else {
            delete main_send_state;
            big_pkt->senderState = NULL;
            pkt = big_pkt;
        }
    }

    _status = BaseSimpleCPU::Running;

    Fault fault;

    // hardware transactional memory
    // sanity checks
    // ensure htmTransactionUids are equivalent
    if (pkt->isHtmTransactional())
        assert (pkt->getHtmTransactionUid() ==
                t_info->getHtmTransactionUid());

    // can't have a packet that fails a transaction while not in a transaction
    if (pkt->htmTransactionFailedInCache())
        assert(is_htm_speculative);

    // shouldn't fail through stores because this would be inconsistent w/ O3
    // which cannot fault after the store has been sent to memory
    if (pkt->htmTransactionFailedInCache() && !pkt->isWrite()) {
        const HtmCacheFailure htm_rc =
            pkt->getHtmTransactionFailedInCacheRC();
        DPRINTF(HtmCpu, "HTM abortion in cache (rc=%s) detected htmUid=%u\n",
            htmFailureToStr(htm_rc), pkt->getHtmTransactionUid());

        // Currently there are only two reasons why a transaction would
        // fail in the memory subsystem--
        // (1) A transactional line was evicted from the cache for
        //     space (or replacement policy) reasons.
        // (2) Another core/device requested a cache line that is in this
        //     transaction's read/write set that is incompatible with the
        //     HTM's semantics, e.g. another core requesting exclusive access
        //     of a line in this core's read set.
        if (htm_rc == HtmCacheFailure::FAIL_SELF) {
            fault = std::make_shared<GenericHtmFailureFault>(
                t_info->getHtmTransactionUid(),
                HtmFailureFaultCause::SIZE);
        } else if (htm_rc == HtmCacheFailure::FAIL_REMOTE) {
            fault = std::make_shared<GenericHtmFailureFault>(
                t_info->getHtmTransactionUid(),
                HtmFailureFaultCause::MEMORY);
        } else {
            panic("HTM - unhandled rc %s", htmFailureToStr(htm_rc));
        }
    } else {
        fault = curStaticInst->completeAcc(pkt, t_info,
                                     traceData);
    }

    // hardware transactional memory
    // Track HtmStop instructions,
    // e.g. instructions which commit a transaction.
    if (curStaticInst && curStaticInst->isHtmStop()) {
        t_info->thread->htmTransactionStops++;
        DPRINTF(HtmCpu, "htmTransactionStops++=%u\n",
            t_info->thread->htmTransactionStops);
    }

    // keep an instruction count
    if (fault == NoFault)
        countInst();
    else if (traceData) {
        traceFault();
    }

    delete pkt;

    postExecute();

    advanceInst(fault);
}

void
TimingSimpleCPU::updateCycleCounts()
{
    const Cycles delta(curCycle() - previousCycle);

    baseStats.numCycles += delta;

    previousCycle = curCycle();
}

void
TimingSimpleCPU::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }

    // Making it uniform across all CPUs:
    // The CPUs need to be woken up only on an invalidation packet
    // (when using caches) or on an incoming write packet (when not
    // using caches) It is not necessary to wake up the processor on
    // all incoming packets
    if (pkt->isInvalidate() || pkt->isWrite()) {
        for (auto &t_info : cpu->threadInfo) {
            t_info->thread->getIsaPtr()->handleLockedSnoop(pkt,
                    cacheBlockMask);
        }
    } else if (pkt->req && pkt->req->isTlbiExtSync()) {
        // We received a TLBI_EXT_SYNC request.
        // In a detailed sim we would wait for memory ops to complete,
        // but in our simple case we just respond immediately
        auto reply_req = Request::createMemManagement(
            Request::TLBI_EXT_SYNC_COMP,
            cpu->dataRequestorId());

        // Extra Data = the transaction ID of the Sync we're completing
        reply_req->setExtraData(pkt->req->getExtraData());
        PacketPtr reply_pkt = Packet::createRead(reply_req);

        // TODO - reserve some credit for these responses?
        if (!sendTimingReq(reply_pkt)) {
            panic("Couldn't send TLBI_EXT_SYNC_COMP message");
        }
    }
}

void
TimingSimpleCPU::DcachePort::recvFunctionalSnoop(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }
}

bool
TimingSimpleCPU::DcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "Received load/store response %#x\n", pkt->getAddr());

    // The timing CPU is not really ticked, instead it relies on the
    // memory system (fetch and load/store) to set the pace.
    if (!tickEvent.scheduled()) {
        // Delay processing of returned data until next CPU clock edge
        tickEvent.schedule(pkt, cpu->clockEdge());
        return true;
    } else {
        // In the case of a split transaction and a cache that is
        // faster than a CPU we could get two responses in the
        // same tick, delay the second one
        if (!retryRespEvent.scheduled())
            cpu->schedule(retryRespEvent, cpu->clockEdge(Cycles(1)));
        return false;
    }
}

void
TimingSimpleCPU::DcachePort::DTickEvent::process()
{
    cpu->completeDataAccess(pkt);
}

void
TimingSimpleCPU::DcachePort::recvReqRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->dcache_pkt != NULL);
    assert(cpu->_status == DcacheRetry);
    PacketPtr tmp = cpu->dcache_pkt;
    if (tmp->senderState) {
        // This is a packet from a split access.
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(tmp->senderState);
        assert(send_state);
        PacketPtr big_pkt = send_state->bigPkt;

        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);

        if (sendTimingReq(tmp)) {
            // If we were able to send without retrying, record that fact
            // and try sending the other fragment.
            send_state->clearFromParent();
            int other_index = main_send_state->getPendingFragment();
            if (other_index > 0) {
                tmp = main_send_state->fragments[other_index];
                cpu->dcache_pkt = tmp;
                if ((big_pkt->isRead() && cpu->handleReadPacket(tmp)) ||
                        (big_pkt->isWrite() && cpu->handleWritePacket())) {
                    main_send_state->fragments[other_index] = NULL;
                }
            } else {
                cpu->_status = DcacheWaitResponse;
                // memory system takes ownership of packet
                cpu->dcache_pkt = NULL;
            }
        }
    } else if (sendTimingReq(tmp)) {
        cpu->_status = DcacheWaitResponse;
        // memory system takes ownership of packet
        cpu->dcache_pkt = NULL;
    }
}

TimingSimpleCPU::IprEvent::IprEvent(Packet *_pkt, TimingSimpleCPU *_cpu,
    Tick t)
    : pkt(_pkt), cpu(_cpu)
{
    cpu->schedule(this, t);
}

void
TimingSimpleCPU::IprEvent::process()
{
    cpu->completeDataAccess(pkt);
}

const char *
TimingSimpleCPU::IprEvent::description() const
{
    return "Timing Simple CPU Delay IPR event";
}


void
TimingSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

Fault
TimingSimpleCPU::initiateMemMgmtCmd(Request::Flags flags)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const Addr addr = 0x0ul;
    const Addr pc = thread->pcState().instAddr();
    const int size = 8;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId());

    req->setPC(pc);
    req->setContext(thread->contextId());
    req->taskId(taskId());
    req->setInstCount(t_info.numInst);

    assert(req->isHTMCmd() || req->isTlbiCmd());

    // Use the payload as a sanity check,
    // the memory subsystem will clear allocated data
    uint8_t *data = new uint8_t[size];
    assert(data);
    uint64_t rc = 0xdeadbeeflu;
    memcpy (data, &rc, size);

    // debugging output
    if (req->isHTMCmd()) {
        if (req->isHTMStart())
            DPRINTF(HtmCpu, "HTMstart htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else if (req->isHTMCommit())
            DPRINTF(HtmCpu, "HTMcommit htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else if (req->isHTMCancel())
            DPRINTF(HtmCpu, "HTMcancel htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else
            panic("initiateMemMgmtCmd: unknown HTM CMD");
    }

    sendData(req, data, nullptr, true);

    return NoFault;
}

void
TimingSimpleCPU::htmSendAbortSignal(ThreadID tid, uint64_t htm_uid,
                                    HtmFailureFaultCause cause)
{
    SimpleExecContext& t_info = *threadInfo[tid];
    SimpleThread* thread = t_info.thread;

    const Addr addr = 0x0ul;
    const Addr pc = thread->pcState().instAddr();
    const int size = 8;
    const Request::Flags flags =
        Request::PHYSICAL|Request::STRICT_ORDER|Request::HTM_ABORT;

    if (traceData)
        traceData->setMem(addr, size, flags);

    // notify l1 d-cache (ruby) that core has aborted transaction

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId());

    req->setPC(pc);
    req->setContext(thread->contextId());
    req->taskId(taskId());
    req->setInstCount(t_info.numInst);
    req->setHtmAbortCause(cause);

    assert(req->isHTMAbort());

    uint8_t *data = new uint8_t[size];
    assert(data);
    uint64_t rc = 0lu;
    memcpy (data, &rc, size);

    sendData(req, data, nullptr, true);
}

/*
 * =============================================================================
 * NOVA Processor - Shared Functional Unit Implementation v2.0
 * PhD Research: Spacecraft multicore processor with shared accelerators
 * 
 * v2.0: Implements ACTUAL timing delays via event-based stall mechanism
 * =============================================================================
 */

// Static member definitions
std::array<TimingSimpleCPU::SharedFUState, 5> TimingSimpleCPU::sharedFUState;
std::mutex TimingSimpleCPU::sharedFUMutex;
int TimingSimpleCPU::numTrigAccels = 1;
int TimingSimpleCPU::numMatAccels = 1;
int TimingSimpleCPU::numVpuAccels = 1;
int TimingSimpleCPU::numNpuAccels = 1;
int TimingSimpleCPU::numGpuAccels = 1;
Cycles TimingSimpleCPU::trigLatency = Cycles(32);
Cycles TimingSimpleCPU::matLatency = Cycles(27);
Cycles TimingSimpleCPU::vpuLatency = Cycles(100);
Cycles TimingSimpleCPU::npuLatency = Cycles(200);
Cycles TimingSimpleCPU::gpuLatency = Cycles(150);

void
TimingSimpleCPU::configureSharedFU(int numTrig, int numMat, 
                                   Cycles trigLat, Cycles matLat)
{
    std::lock_guard<std::mutex> lock(sharedFUMutex);
    numTrigAccels = numTrig;
    numMatAccels = numMat;
    trigLatency = trigLat;
    matLatency = matLat;
    
    // Set per-type instance counts
    sharedFUState[0].numInstances = numTrigAccels;
    sharedFUState[1].numInstances = numMatAccels;
    sharedFUState[2].numInstances = numVpuAccels;
    sharedFUState[3].numInstances = numNpuAccels;
    sharedFUState[4].numInstances = numGpuAccels;
    
    // Reset state
    for (auto& state : sharedFUState) {
        if (state.numInstances < 1) state.numInstances = 1;
        if (state.numInstances > MAX_ACCEL_INSTANCES)
            state.numInstances = MAX_ACCEL_INSTANCES;
        for (int i = 0; i < MAX_ACCEL_INSTANCES; i++) {
            state.instanceBusyUntil[i] = 0;
        }
        state.totalRequests = 0;
        state.queuedRequests = 0;
        state.totalWaitCycles = 0;
        state.totalWaitTicks = 0;
        state.totalStallTicks = 0;
        state.contentionRangeStart = 0;
        state.contentionRangeEnd = 0;
        state.totalContentionTicks = 0;
    }
    
    warn("NOVA v2.0: Shared FU configured - Trig:%d(lat=%d), Mat:%d(lat=%d)\n",
         numTrig, (int)trigLat, numMat, (int)matLat);
}

// Counter for tracking accelerator operations detected
std::atomic<uint64_t> trigOpsDetected{0};
std::atomic<uint64_t> matOpsDetected{0};
std::atomic<uint64_t> vpuOpsDetected{0};
std::atomic<uint64_t> npuOpsDetected{0};
std::atomic<uint64_t> gpuOpsDetected{0};

// Resume after shared FU stall - called when the FU becomes available
void
TimingSimpleCPU::resumeAfterSharedFUStall()
{
    if (!sharedFUStalled) {
        // Already resumed (e.g., thread was suspended), nothing to do
        return;
    }
    
    DPRINTF(SimpleCPU, "NOVA v3.1: CPU%d resuming after FU%d stall at tick %lu\n",
            cpuId(), stalledForFUType, curTick());
    
    // Clear stall state
    sharedFUStalled = false;
    int fuType = stalledForFUType;
    stalledForFUType = -1;
    stallUntilTick = 0;
    
    // NOVA v3.1 FIX: Transition back to Running before executing.
    // completeIfetch() set _status = IcacheWaitResponse when stalling;
    // we must restore Running so that advanceInst() -> fetch() works
    // correctly and the normal CPU pipeline state machine is maintained.
    _status = BaseSimpleCPU::Running;
    
    // Now execute the instruction that was waiting
    SimpleExecContext &t_info = *threadInfo[curThread];
    
    if (!curStaticInst) {
        // No instruction to execute, just resume fetch
        if (!fetchEvent.scheduled()) {
            schedule(fetchEvent, clockEdge());
        }
        return;
    }
    
    // Execute the instruction now that FU is available
    Fault fault = curStaticInst->execute(&t_info, traceData);
    
    // keep an instruction count
    if (fault == NoFault)
        countInst();
    else if (traceData) {
        traceFault();
    }
    
    postExecute();
    
    if (curStaticInst && (!curStaticInst->isMicroop() ||
            curStaticInst->isFirstMicroop()))
        instCnt++;
    
    // Release shared FU after operation
    if (fuType >= 0) {
        releaseSharedFU(fuType);
    }
    
    advanceInst(fault);
}

bool
TimingSimpleCPU::usesSharedFU(const StaticInstPtr &inst) const
{
    if (!inst)
        return false;
    
    OpClass opClass = inst->opClass();
    
    // Trigonometric operations -> FU type 0
    // Core trig: sin, cos, tan, asin, acos, atan, atan2
    // Extended CORDIC: exp, log, sinh, cosh, tanh, hypot, sqrt
    if (opClass == enums::FloatSin || opClass == enums::FloatCos ||
        opClass == enums::FloatTan || opClass == enums::FloataSin ||
        opClass == enums::FloataCos || opClass == enums::FloataTan ||
        opClass == enums::FloataTan2 ||
        opClass == enums::FloatExp || opClass == enums::FloatLog ||
        opClass == enums::FloatSinh || opClass == enums::FloatCosh ||
        opClass == enums::FloatTanh || opClass == enums::FloatHypot ||
        opClass == enums::FloatSqrt ||
        opClass == enums::SimdFloatSin || opClass == enums::SimdFloatCos ||
        opClass == enums::SimdFloatTan) {
        trigOpsDetected++;
        return true;
    }
    
    // Matrix operations -> FU type 1
    if (opClass == enums::Matrix || opClass == enums::MatrixMov ||
        opClass == enums::MatrixOP) {
        matOpsDetected++;
        return true;
    }
    
    // VPU operations (custom instructions) -> FU type 2
    // VPU uses SIMD operations for vision processing (convolution, etc.)
    if (opClass == enums::SimdMisc || 
        opClass == enums::SimdMult || opClass == enums::SimdMultAcc ||
        opClass == enums::SimdAdd || opClass == enums::SimdShift ||
        opClass == enums::SimdSqrt || opClass == enums::SimdReduceAdd ||
        opClass == enums::SimdFloatSqrt || opClass == enums::SimdFloatDiv) {
        vpuOpsDetected++;
        return true;
    }
    
    // NPU operations (neural network accelerator) -> FU type 3
    // NPU uses MAC-heavy operations detected via accumulator patterns
    if (opClass == enums::SimdFloatMultAcc || opClass == enums::SimdFloatMult ||
        opClass == enums::SimdFloatMisc) {
        npuOpsDetected++;
        return true;
    }
    
    // GPU operations (SpaceGPU render/compute) -> FU type 4
    // GPU uses floating point add, compare, convert, and ALU ops for rendering
    if (opClass == enums::SimdFloatAdd || opClass == enums::SimdFloatCmp ||
        opClass == enums::SimdFloatCvt || opClass == enums::SimdFloatAlu ||
        opClass == enums::SimdCvt || opClass == enums::SimdFloatMatMultAcc ||
        opClass == enums::SimdReduceAlu || opClass == enums::SimdReduceCmp) {
        gpuOpsDetected++;
        return true;
    }
    
    return false;
}

int
TimingSimpleCPU::getSharedFUType(const StaticInstPtr &inst) const
{
    if (!inst)
        return 0;
    
    OpClass opClass = inst->opClass();
    
    // Trigonometric operations -> type 0
    // Core trig: sin, cos, tan, asin, acos, atan, atan2
    // Extended CORDIC: exp, log, sinh, cosh, tanh, hypot, sqrt
    if (opClass == enums::FloatSin || opClass == enums::FloatCos ||
        opClass == enums::FloatTan || opClass == enums::FloataSin ||
        opClass == enums::FloataCos || opClass == enums::FloataTan ||
        opClass == enums::FloataTan2 ||
        opClass == enums::FloatExp || opClass == enums::FloatLog ||
        opClass == enums::FloatSinh || opClass == enums::FloatCosh ||
        opClass == enums::FloatTanh || opClass == enums::FloatHypot ||
        opClass == enums::FloatSqrt ||
        opClass == enums::SimdFloatSin || opClass == enums::SimdFloatCos ||
        opClass == enums::SimdFloatTan) {
        return 0;  // TRIG
    }
    
    // Matrix operations -> type 1
    if (opClass == enums::Matrix || opClass == enums::MatrixMov ||
        opClass == enums::MatrixOP) {
        return 1;  // MAT
    }
    
    // VPU operations -> type 2
    if (opClass == enums::SimdMisc || 
        opClass == enums::SimdMult || opClass == enums::SimdMultAcc ||
        opClass == enums::SimdAdd || opClass == enums::SimdShift ||
        opClass == enums::SimdSqrt || opClass == enums::SimdReduceAdd ||
        opClass == enums::SimdFloatSqrt || opClass == enums::SimdFloatDiv) {
        return 2;  // VPU
    }
    
    // NPU operations -> type 3
    if (opClass == enums::SimdFloatMultAcc || opClass == enums::SimdFloatMult ||
        opClass == enums::SimdFloatMisc) {
        return 3;  // NPU
    }
    
    return 0;
}

Cycles
TimingSimpleCPU::getSharedFULatency(int fuType) const
{
    switch (fuType) {
      case 0: return trigLatency;  // TRIG
      case 1: return matLatency;   // MAT
      case 2: return vpuLatency;   // VPU
      case 3: return npuLatency;   // NPU
      case 4: return gpuLatency;   // GPU
      default: return trigLatency;
    }
}

Tick
TimingSimpleCPU::tryAcquireSharedFU(int fuType)
{
    std::lock_guard<std::mutex> lock(sharedFUMutex);
    
    SharedFUState& state = sharedFUState[fuType];
    state.totalRequests++;
    
    Tick now = curTick();
    Cycles opLatency = getSharedFULatency(fuType);
    Tick opLatencyTicks = cyclesToTicks(opLatency);
    
    // Use the per-instance count stored in SharedFUState
    int numAccels = state.numInstances;
    
    // NOVA v3.1: MULTI-INSTANCE SHARED ACCELERATOR MODEL
    // Each accelerator type has numAccels independent instances.
    // Find the instance that will be free earliest.
    
    int bestInstance = 0;
    Tick earliestFree = state.instanceBusyUntil[0];
    for (int i = 1; i < numAccels; i++) {
        if (state.instanceBusyUntil[i] < earliestFree) {
            earliestFree = state.instanceBusyUntil[i];
            bestInstance = i;
        }
    }
    
    Tick totalStallTicks = 0;
    
    if (earliestFree > now) {
        // All instances are busy - must wait for the earliest one to finish
        Tick waitTicks = earliestFree - now;
        
        // Track contention statistics
        state.queuedRequests++;
        state.totalWaitCycles += ticksToCycles(waitTicks);
        state.totalStallTicks += waitTicks;
        state.totalWaitTicks += waitTicks;
        
        // Track contentious period (cycles with queue > 0)
        Tick newBusyEnd = earliestFree + opLatencyTicks;
        if (now > state.contentionRangeEnd) {
            // Previous contentious period ended, finalize it
            if (state.contentionRangeEnd > state.contentionRangeStart) {
                state.totalContentionTicks += 
                    (state.contentionRangeEnd - state.contentionRangeStart);
            }
            // Start new contentious period
            state.contentionRangeStart = now;
            state.contentionRangeEnd = newBusyEnd;
        } else {
            // Extend current contentious period
            if (newBusyEnd > state.contentionRangeEnd) {
                state.contentionRangeEnd = newBusyEnd;
            }
        }
        
        // Total stall = wait for instance to free + our operation latency
        totalStallTicks = waitTicks + opLatencyTicks;
        
        // Reserve the instance: it becomes busy after current op finishes + our latency
        state.instanceBusyUntil[bestInstance] = earliestFree + opLatencyTicks;
        
        DPRINTF(SimpleCPU, "NOVA v3.1: CPU%d CONTENTION on FU%d inst%d/%d, "
                "wait %lu + op %lu = %lu ticks\n",
                cpuId(), fuType, bestInstance, numAccels,
                waitTicks, opLatencyTicks, totalStallTicks);
    } else {
        // At least one instance is free - use it, stall only for operation latency
        state.instanceBusyUntil[bestInstance] = now + opLatencyTicks;
        
        // Still stall for the operation latency (accelerator takes time)
        totalStallTicks = opLatencyTicks;
        
        DPRINTF(SimpleCPU, "NOVA v3.1: CPU%d acquired FU%d inst%d/%d, "
                "op latency %lu ticks\n",
                cpuId(), fuType, bestInstance, numAccels, opLatencyTicks);
    }
    
    return totalStallTicks;  // Return total stall (wait + operation)
}

void
TimingSimpleCPU::releaseSharedFU(int fuType)
{
    // In the multi-instance model, release is implicit: instanceBusyUntil
    // timestamps naturally expire. No explicit release needed since
    // tryAcquireSharedFU checks timestamps directly.
    // This function is kept for API compatibility and potential future use.
}

void
TimingSimpleCPU::printSharedFUStats()
{
    std::lock_guard<std::mutex> lock(sharedFUMutex);
    
    const char* names[] = {"TrigAccel", "MatAccel", "VPU", "NPU", "GPU"};
    
    warn("====== NOVA v3.1 Multi-Instance Shared Accelerator Statistics ======\n");
    warn("Detected operations: Trig=%lu, Mat=%lu, VPU=%lu, NPU=%lu, GPU=%lu\n", 
         (unsigned long)trigOpsDetected, (unsigned long)matOpsDetected,
         (unsigned long)vpuOpsDetected, (unsigned long)npuOpsDetected,
         (unsigned long)gpuOpsDetected);
    
    uint64_t totalStallTicks = 0;
    for (int i = 0; i < 5; i++) {
        SharedFUState& state = sharedFUState[i];
        if (state.totalRequests > 0) {
            double queueRatio = (double)state.queuedRequests / state.totalRequests * 100.0;
            double avgWait = state.queuedRequests > 0 ? 
                (double)state.totalWaitCycles / state.queuedRequests : 0;
            
            // Finalize any pending contentious period first
            if (state.contentionRangeEnd > state.contentionRangeStart) {
                state.totalContentionTicks += 
                    (state.contentionRangeEnd - state.contentionRangeStart);
                state.contentionRangeStart = 0;
                state.contentionRangeEnd = 0;
            }
            
            // avgQueueDepth = totalWaitTicks / totalContentionTicks
            double avgQueueDepth = state.totalContentionTicks > 0 ?
                (double)state.totalWaitTicks / (double)state.totalContentionTicks : 0.0;
            
            int currentAccels = state.numInstances;
            int recommendedAccels = currentAccels + (int)std::ceil(avgQueueDepth);
            
            warn("%s (%d instances): requests=%lu, queued=%lu (%.1f%%), "
                 "avgWait=%.1f cycles, contentionTicks=%lu, "
                 "avgQueueDepth=%.3f, recommendedAccels=%d\n",
                 names[i], currentAccels,
                 (unsigned long)state.totalRequests, 
                 (unsigned long)state.queuedRequests, queueRatio, avgWait,
                 (unsigned long)state.totalContentionTicks, avgQueueDepth,
                 recommendedAccels);
            
            totalStallTicks += state.totalStallTicks;
        }
    }
    warn("Total stall ticks from contention: %lu\n", (unsigned long)totalStallTicks);
    warn("============================================\n");
}

// Call this automatically at program exit to print stats
static struct SharedFUStatsReporter {
    ~SharedFUStatsReporter() {
        TimingSimpleCPU::printSharedFUStats();
    }
} sharedFUStatsReporter;

} // namespace gem5
