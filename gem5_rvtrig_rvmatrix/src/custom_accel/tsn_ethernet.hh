/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * Time-Sensitive Networking (TSN) Ethernet Controller
 * 
 * Implements IEEE 802.1 TSN standards for deterministic networking:
 * - IEEE 802.1AS-2020 Time synchronization (gPTP)
 * - IEEE 802.1Qbv-2015 Time-aware shaper (TAS)
 * - IEEE 802.1Qci-2017 Per-stream filtering and policing
 * - IEEE 802.1CB-2017 Frame replication and elimination
 * 
 * Designed for next-generation spacecraft avionics networks
 */

#ifndef __CUSTOM_ACCEL_TSN_ETHERNET_HH__
#define __CUSTOM_ACCEL_TSN_ETHERNET_HH__

#include <array>
#include <queue>
#include <vector>
#include <cstdint>

#include "mem/port.hh"
#include "params/TSNEthernet.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * TSNEthernet - Time-Sensitive Networking Ethernet Controller
 * 
 * Key TSN Features:
 * 1. Time Synchronization (802.1AS-2020)
 *    - gPTP (generalized Precision Time Protocol)
 *    - Sub-microsecond synchronization
 *    
 * 2. Time-Aware Shaper (802.1Qbv)
 *    - Gate Control List (GCL) for traffic scheduling
 *    - Up to 8 traffic classes
 *    - Deterministic latency
 *    
 * 3. Per-Stream Filtering (802.1Qci)
 *    - Stream identification
 *    - Ingress rate limiting
 *    - Frame filtering
 *    
 * 4. Frame Replication (802.1CB)
 *    - Redundancy for critical streams
 *    - Seamless redundancy
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x000: CTRL            - Control register
 * 0x004: STATUS          - Status register
 * 0x008: MAC_ADDR_L      - MAC address low 32 bits
 * 0x00C: MAC_ADDR_H      - MAC address high 16 bits
 * 0x010: TX_BASE         - TX buffer base address
 * 0x014: TX_LEN          - TX frame length
 * 0x018: TX_DESC_BASE    - TX descriptor ring base
 * 0x01C: TX_DESC_TAIL    - TX descriptor tail pointer
 * 0x020: RX_BASE         - RX buffer base address
 * 0x024: RX_LEN          - RX frame length
 * 0x028: RX_DESC_BASE    - RX descriptor ring base
 * 0x02C: RX_DESC_HEAD    - RX descriptor head pointer
 * 0x030: LINK_SPEED      - Link speed (10/100/1000 Mbps)
 * 0x034: LINK_STATUS     - PHY link status
 * 
 * TSN Registers:
 * 0x100: PTP_CTRL        - PTP/gPTP control
 * 0x104: PTP_STATUS      - PTP status
 * 0x108: PTP_TIME_L      - PTP time low 32 bits (ns)
 * 0x10C: PTP_TIME_H      - PTP time high 32 bits (s)
 * 0x110: PTP_OFFSET_L    - PTP offset low
 * 0x114: PTP_OFFSET_H    - PTP offset high
 * 0x118: PTP_RATE_ADJ    - PTP rate adjustment (ppb)
 * 0x11C: PTP_TX_TS_L     - TX timestamp low
 * 0x120: PTP_TX_TS_H     - TX timestamp high
 * 0x124: PTP_RX_TS_L     - RX timestamp low
 * 0x128: PTP_RX_TS_H     - RX timestamp high
 * 
 * 0x200: TAS_CTRL        - Time-Aware Shaper control
 * 0x204: TAS_STATUS      - TAS status
 * 0x208: TAS_CYCLE_TIME  - GCL cycle time (ns)
 * 0x20C: TAS_BASE_TIME_L - Base time low
 * 0x210: TAS_BASE_TIME_H - Base time high
 * 0x214: TAS_GCL_LEN     - Gate Control List length
 * 0x218: TAS_GCL_BASE    - GCL memory base address
 * 0x21C-0x23C: TAS_QUEUE[8] - Queue gate status
 * 
 * 0x300: FRER_CTRL       - Frame Replication control
 * 0x304: FRER_STATUS     - FRER status
 * 0x308: FRER_STREAMS    - Number of configured streams
 * 0x30C: FRER_STREAM_BASE- Stream configuration base
 * 
 * 0x400: PSFP_CTRL       - Per-Stream Filtering control
 * 0x404: PSFP_STATUS     - PSFP status
 * 0x408: PSFP_FILTER_CNT - Active filters count
 * 
 * 0x500: PERF_TX_FRAMES  - TX frame counter
 * 0x504: PERF_RX_FRAMES  - RX frame counter
 * 0x508: PERF_TX_BYTES   - TX byte counter
 * 0x50C: PERF_RX_BYTES   - RX byte counter
 * 0x510: PERF_ERRORS     - Error counter
 * 0x514: INTERRUPT       - Interrupt status/control
 * 0x518: VERSION         - Controller version
 */
class TSNEthernet : public ClockedObject
{
  private:
    /**
     * CPU-side port for MMIO access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        TSNEthernet *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, TSNEthernet *owner);
        void sendPacket(PacketPtr pkt);
        AddrRangeList getAddrRanges() const override;
        void trySendRetry();

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    /**
     * Memory-side port for DMA
     */
    class MemSidePort : public RequestPort
    {
      private:
        TSNEthernet *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, TSNEthernet *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Traffic classes (8 priority levels)
    static const unsigned NUM_TRAFFIC_CLASSES = 8;

    // Gate Control List entry
    struct GclEntry {
        uint8_t gateStates;      // Bit mask for 8 gates
        uint32_t timeInterval;   // Duration in ns
    };

    // Stream identification
    struct StreamId {
        uint64_t macDest;
        uint16_t vlanId;
        uint8_t priority;
        bool enabled;
    };

    // TX/RX Descriptor
    struct Descriptor {
        uint32_t bufAddr;
        uint32_t length;
        uint32_t status;
        uint32_t timestamp;
    };

    // Register offsets - Basic
    static const Addr REG_CTRL           = 0x000;
    static const Addr REG_STATUS         = 0x004;
    static const Addr REG_MAC_ADDR_L     = 0x008;
    static const Addr REG_MAC_ADDR_H     = 0x00C;
    static const Addr REG_TX_BASE        = 0x010;
    static const Addr REG_TX_LEN         = 0x014;
    static const Addr REG_TX_DESC_BASE   = 0x018;
    static const Addr REG_TX_DESC_TAIL   = 0x01C;
    static const Addr REG_RX_BASE        = 0x020;
    static const Addr REG_RX_LEN         = 0x024;
    static const Addr REG_RX_DESC_BASE   = 0x028;
    static const Addr REG_RX_DESC_HEAD   = 0x02C;
    static const Addr REG_LINK_SPEED     = 0x030;
    static const Addr REG_LINK_STATUS    = 0x034;

    // Register offsets - PTP
    static const Addr REG_PTP_CTRL       = 0x100;
    static const Addr REG_PTP_STATUS     = 0x104;
    static const Addr REG_PTP_TIME_L     = 0x108;
    static const Addr REG_PTP_TIME_H     = 0x10C;
    static const Addr REG_PTP_OFFSET_L   = 0x110;
    static const Addr REG_PTP_OFFSET_H   = 0x114;
    static const Addr REG_PTP_RATE_ADJ   = 0x118;
    static const Addr REG_PTP_TX_TS_L    = 0x11C;
    static const Addr REG_PTP_TX_TS_H    = 0x120;
    static const Addr REG_PTP_RX_TS_L    = 0x124;
    static const Addr REG_PTP_RX_TS_H    = 0x128;

    // Register offsets - TAS
    static const Addr REG_TAS_CTRL       = 0x200;
    static const Addr REG_TAS_STATUS     = 0x204;
    static const Addr REG_TAS_CYCLE_TIME = 0x208;
    static const Addr REG_TAS_BASE_TIME_L = 0x20C;
    static const Addr REG_TAS_BASE_TIME_H = 0x210;
    static const Addr REG_TAS_GCL_LEN    = 0x214;
    static const Addr REG_TAS_GCL_BASE   = 0x218;
    static const Addr REG_TAS_QUEUE_BASE = 0x21C;

    // Register offsets - FRER
    static const Addr REG_FRER_CTRL      = 0x300;
    static const Addr REG_FRER_STATUS    = 0x304;
    static const Addr REG_FRER_STREAMS   = 0x308;
    static const Addr REG_FRER_STREAM_BASE = 0x30C;

    // Register offsets - PSFP
    static const Addr REG_PSFP_CTRL      = 0x400;
    static const Addr REG_PSFP_STATUS    = 0x404;
    static const Addr REG_PSFP_FILTER_CNT = 0x408;

    // Register offsets - Performance
    static const Addr REG_PERF_TX_FRAMES = 0x500;
    static const Addr REG_PERF_RX_FRAMES = 0x504;
    static const Addr REG_PERF_TX_BYTES  = 0x508;
    static const Addr REG_PERF_RX_BYTES  = 0x50C;
    static const Addr REG_PERF_ERRORS    = 0x510;
    static const Addr REG_INTERRUPT      = 0x514;
    static const Addr REG_VERSION        = 0x518;

    // Control bits
    static const uint32_t CTRL_TX_EN     = 0x01;
    static const uint32_t CTRL_RX_EN     = 0x02;
    static const uint32_t CTRL_PROMISC   = 0x04;
    static const uint32_t CTRL_LOOPBACK  = 0x08;
    static const uint32_t CTRL_RESET     = 0x80;
    static const uint32_t CTRL_IRQ_EN    = 0x100;

    // Status bits
    static const uint32_t STATUS_TX_IDLE     = 0x01;
    static const uint32_t STATUS_RX_READY    = 0x02;
    static const uint32_t STATUS_LINK_UP     = 0x04;
    static const uint32_t STATUS_PTP_SYNC    = 0x08;
    static const uint32_t STATUS_TAS_ACTIVE  = 0x10;

    // Internal registers - Basic
    uint32_t regCtrl;
    uint32_t regStatus;
    uint64_t regMacAddr;
    Addr regTxBase;
    uint32_t regTxLen;
    Addr regTxDescBase;
    uint32_t regTxDescTail;
    Addr regRxBase;
    uint32_t regRxLen;
    Addr regRxDescBase;
    uint32_t regRxDescHead;
    uint32_t regLinkSpeed;
    uint32_t regLinkStatus;

    // Internal registers - PTP
    uint32_t regPtpCtrl;
    uint32_t regPtpStatus;
    uint64_t regPtpTime;
    int64_t regPtpOffset;
    int32_t regPtpRateAdj;
    uint64_t regPtpTxTs;
    uint64_t regPtpRxTs;

    // Internal registers - TAS
    uint32_t regTasCtrl;
    uint32_t regTasStatus;
    uint32_t regTasCycleTime;
    uint64_t regTasBaseTime;
    uint32_t regTasGclLen;
    Addr regTasGclBase;
    std::array<uint32_t, NUM_TRAFFIC_CLASSES> regTasQueue;

    // Internal registers - FRER
    uint32_t regFrerCtrl;
    uint32_t regFrerStatus;
    uint32_t regFrerStreams;
    Addr regFrerStreamBase;

    // Internal registers - PSFP
    uint32_t regPsfpCtrl;
    uint32_t regPsfpStatus;
    uint32_t regPsfpFilterCnt;

    // Performance counters
    uint64_t regPerfTxFrames;
    uint64_t regPerfRxFrames;
    uint64_t regPerfTxBytes;
    uint64_t regPerfRxBytes;
    uint64_t regPerfErrors;
    uint32_t regInterrupt;

    // Hardware parameters
    unsigned linkSpeedMbps;
    unsigned txFifoDepth;
    unsigned rxFifoDepth;
    unsigned maxFrameSize;
    unsigned gclMaxEntries;

    // Gate Control List
    std::vector<GclEntry> gateControlList;
    unsigned currentGclIndex;
    uint8_t currentGateStates;

    // Traffic queues (one per traffic class)
    std::array<std::queue<std::vector<uint8_t>>, NUM_TRAFFIC_CLASSES> txQueues;
    std::queue<std::vector<uint8_t>> rxQueue;

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // Statistics
    struct TsnStats : public statistics::Group
    {
        TsnStats(TSNEthernet *parent);
        statistics::Scalar txFrames;
        statistics::Scalar rxFrames;
        statistics::Scalar txBytes;
        statistics::Scalar rxBytes;
        statistics::Scalar crcErrors;
        statistics::Scalar gateDrops;        // Frames dropped by TAS
        statistics::Scalar psfpDrops;        // Frames dropped by PSFP
        statistics::Scalar frerDuplicates;   // Duplicates eliminated by FRER
        statistics::Scalar ptpSyncEvents;
        statistics::Scalar ptpOffsetNs;      // Current PTP offset
        statistics::Scalar totalCycles;      // For throughput calculation
        statistics::Formula throughput;
    } stats;

    // Events
    EventFunctionWrapper txEvent;
    EventFunctionWrapper rxEvent;
    EventFunctionWrapper ptpEvent;
    EventFunctionWrapper tasEvent;

    // Internal methods
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);

    // Ethernet operations
    void startTx();
    void processTx();
    void completeTx();
    void processRx();
    void completeRx();

    // PTP operations
    void ptpProcess();
    void ptpUpdateTime();
    void ptpCaptureTimestamp(bool isTx);
    uint64_t getPtpTime();

    // TAS operations
    void tasProcess();
    void tasUpdateGates();
    bool tasCheckGate(unsigned trafficClass);
    void loadGcl();

    // FRER operations
    bool frerCheckDuplicate(const std::vector<uint8_t>& frame);
    void frerReplicate(std::vector<uint8_t>& frame);

    // PSFP operations
    bool psfpFilterFrame(const std::vector<uint8_t>& frame);
    bool psfpCheckRate(unsigned streamId);

    // Utility methods
    unsigned selectQueue(const std::vector<uint8_t>& frame);
    uint32_t computeCrc32(const uint8_t* data, unsigned length);

  public:
    PARAMS(TSNEthernet);
    TSNEthernet(const Params &params);

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();

    // External interface for network simulation
    void externalRx(const std::vector<uint8_t>& frame);
    bool isLinkUp() const { return regLinkStatus & STATUS_LINK_UP; }

    // Version information
    static const uint32_t VERSION = 0x01000000;  // v1.0.0.0
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_TSN_ETHERNET_HH__



