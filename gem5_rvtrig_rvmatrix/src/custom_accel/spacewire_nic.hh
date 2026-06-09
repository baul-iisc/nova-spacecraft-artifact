/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * SpaceWire Network Interface Controller
 * 
 * Implements ECSS-E-ST-50-12C SpaceWire standard
 * High-speed serial data link for spacecraft systems
 * 
 * Features:
 * - Configurable link speeds (2-400 Mbps)
 * - RMAP (Remote Memory Access Protocol) support
 * - Time-code distribution
 * - Multiple virtual channels
 * - Error detection and recovery
 */

#ifndef __CUSTOM_ACCEL_SPACEWIRE_NIC_HH__
#define __CUSTOM_ACCEL_SPACEWIRE_NIC_HH__

#include <array>
#include <queue>
#include <vector>
#include <cstdint>

#include "mem/port.hh"
#include "params/SpaceWireNIC.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * SpaceWireNIC - SpaceWire Network Interface Controller
 * 
 * Implements ECSS-E-ST-50-12C SpaceWire Links, Nodes, Routers standard
 * 
 * SpaceWire Features:
 * - Point-to-point serial data links
 * - Data-Strobe (DS) encoding
 * - Credit-based flow control
 * - Error detection via parity
 * - Link initialization and recovery
 * 
 * Protocol Support:
 * - SpaceWire packet transfer
 * - RMAP (ECSS-E-ST-50-52C) for remote memory access
 * - CCSDS packet encapsulation
 * - Time-code distribution
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x000: CTRL           - Control register
 * 0x004: STATUS         - Status register
 * 0x008: LINK_CTRL      - Link control (speed, enable)
 * 0x00C: LINK_STATUS    - Link status (connected, errors)
 * 0x010: TX_BASE        - Transmit buffer base address
 * 0x014: TX_LEN         - Transmit packet length
 * 0x018: TX_DESC        - Transmit descriptor address
 * 0x01C: RX_BASE        - Receive buffer base address
 * 0x020: RX_LEN         - Received packet length (read-only)
 * 0x024: RX_DESC        - Receive descriptor address
 * 0x028: NODE_ADDR      - SpaceWire node logical address
 * 0x02C: TIME_CODE      - Time-code value
 * 0x030: CREDIT_CNT     - Credit counter
 * 0x034: ERR_CNT        - Error counter
 * 0x038: DISCONNECT_CNT - Disconnect counter
 * 0x03C: RMAP_CTRL      - RMAP protocol control
 * 0x040: RMAP_KEY       - RMAP destination key
 * 0x044: RMAP_STATUS    - RMAP command status
 * 0x048: DMA_CTRL       - DMA engine control
 * 0x04C: DMA_STATUS     - DMA engine status
 * 0x050: PERF_TX_PKT    - TX packets counter
 * 0x054: PERF_RX_PKT    - RX packets counter
 * 0x058: PERF_TX_BYTES  - TX bytes counter
 * 0x05C: PERF_RX_BYTES  - RX bytes counter
 * 0x060: INTERRUPT      - Interrupt status/control
 * 0x064: VERSION        - Controller version
 */
class SpaceWireNIC : public ClockedObject
{
  private:
    /**
     * CPU-side port for MMIO access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        SpaceWireNIC *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, SpaceWireNIC *owner);
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
        SpaceWireNIC *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, SpaceWireNIC *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Link states (ECSS-E-ST-50-12C state machine)
    enum class LinkState : uint8_t {
        ERROR_RESET = 0,
        ERROR_WAIT = 1,
        READY = 2,
        STARTED = 3,
        CONNECTING = 4,
        RUN = 5
    };

    // RMAP command types
    enum class RmapCmd : uint8_t {
        READ_SINGLE = 0x4C,
        READ_INCREMENT = 0x4D,
        WRITE_SINGLE = 0x6C,
        WRITE_INCREMENT = 0x6D,
        WRITE_VERIFY = 0x5C,
        REPLY = 0x0C
    };

    // SpaceWire character types
    enum class CharType : uint8_t {
        DATA = 0,
        EOP = 1,        // End of Packet
        EEP = 2,        // Error End of Packet  
        ESC = 3,        // Escape character
        FCT = 4,        // Flow Control Token
        NULL_CHAR = 5,  // NULL character
        TIME_CODE = 6   // Time-code
    };

    // Register offsets
    static const Addr REG_CTRL           = 0x000;
    static const Addr REG_STATUS         = 0x004;
    static const Addr REG_LINK_CTRL      = 0x008;
    static const Addr REG_LINK_STATUS    = 0x00C;
    static const Addr REG_TX_BASE        = 0x010;
    static const Addr REG_TX_LEN         = 0x014;
    static const Addr REG_TX_DESC        = 0x018;
    static const Addr REG_RX_BASE        = 0x01C;
    static const Addr REG_RX_LEN         = 0x020;
    static const Addr REG_RX_DESC        = 0x024;
    static const Addr REG_NODE_ADDR      = 0x028;
    static const Addr REG_TIME_CODE      = 0x02C;
    static const Addr REG_CREDIT_CNT     = 0x030;
    static const Addr REG_ERR_CNT        = 0x034;
    static const Addr REG_DISCONNECT_CNT = 0x038;
    static const Addr REG_RMAP_CTRL      = 0x03C;
    static const Addr REG_RMAP_KEY       = 0x040;
    static const Addr REG_RMAP_STATUS    = 0x044;
    static const Addr REG_DMA_CTRL       = 0x048;
    static const Addr REG_DMA_STATUS     = 0x04C;
    static const Addr REG_PERF_TX_PKT    = 0x050;
    static const Addr REG_PERF_RX_PKT    = 0x054;
    static const Addr REG_PERF_TX_BYTES  = 0x058;
    static const Addr REG_PERF_RX_BYTES  = 0x05C;
    static const Addr REG_INTERRUPT      = 0x060;
    static const Addr REG_VERSION        = 0x064;

    // Control bits
    static const uint32_t CTRL_TX_EN     = 0x01;
    static const uint32_t CTRL_RX_EN     = 0x02;
    static const uint32_t CTRL_LINK_EN   = 0x04;
    static const uint32_t CTRL_AUTO_START = 0x08;
    static const uint32_t CTRL_LINK_START = 0x10;
    static const uint32_t CTRL_RESET     = 0x80;
    static const uint32_t CTRL_IRQ_EN    = 0x100;

    // Status bits
    static const uint32_t STATUS_TX_IDLE     = 0x01;
    static const uint32_t STATUS_TX_ACTIVE   = 0x02;
    static const uint32_t STATUS_RX_READY    = 0x04;
    static const uint32_t STATUS_RX_ACTIVE   = 0x08;
    static const uint32_t STATUS_LINK_UP     = 0x10;
    static const uint32_t STATUS_ERROR       = 0x20;
    static const uint32_t STATUS_DISCONNECT  = 0x40;
    static const uint32_t STATUS_PARITY_ERR  = 0x80;

    // SpaceWire packet header
    struct SpwPacketHeader {
        uint8_t destAddr;       // Destination logical address
        uint8_t protocol;       // Protocol ID
        uint8_t srcAddr;        // Source logical address
        uint16_t length;        // Payload length
    };

    // RMAP header
    struct RmapHeader {
        uint8_t destAddr;
        uint8_t protocolId;     // 0x01 for RMAP
        uint8_t instruction;
        uint8_t destKey;
        uint8_t srcAddr;
        uint16_t transId;
        uint8_t extAddr;
        uint32_t addr;
        uint32_t length;
    };

    // Transmit/Receive descriptor
    struct Descriptor {
        uint32_t addr;          // Buffer address
        uint32_t length;        // Packet length
        uint32_t status;        // Status flags
        uint32_t next;          // Next descriptor address
    };

    // Internal registers
    uint32_t regCtrl;
    uint32_t regStatus;
    uint32_t regLinkCtrl;
    uint32_t regLinkStatus;
    Addr regTxBase;
    uint32_t regTxLen;
    Addr regTxDesc;
    Addr regRxBase;
    uint32_t regRxLen;
    Addr regRxDesc;
    uint8_t regNodeAddr;
    uint8_t regTimeCode;
    uint32_t regCreditCnt;
    uint32_t regErrCnt;
    uint32_t regDisconnectCnt;
    uint32_t regRmapCtrl;
    uint8_t regRmapKey;
    uint32_t regRmapStatus;
    uint32_t regDmaCtrl;
    uint32_t regDmaStatus;
    uint64_t regPerfTxPkt;
    uint64_t regPerfRxPkt;
    uint64_t regPerfTxBytes;
    uint64_t regPerfRxBytes;
    uint32_t regInterrupt;

    // Hardware parameters
    unsigned linkSpeed;         // Link speed in Mbps
    unsigned txFifoDepth;       // TX FIFO depth
    unsigned rxFifoDepth;       // RX FIFO depth
    unsigned maxPacketSize;     // Maximum packet size

    // Link state machine
    LinkState linkState;
    unsigned creditCount;       // Available credits (flow control)

    // Packet buffers
    std::queue<std::vector<uint8_t>> txQueue;
    std::queue<std::vector<uint8_t>> rxQueue;

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // Statistics
    struct SpwStats : public statistics::Group
    {
        SpwStats(SpaceWireNIC *parent);
        statistics::Scalar txPackets;
        statistics::Scalar rxPackets;
        statistics::Scalar txBytes;
        statistics::Scalar rxBytes;
        statistics::Scalar parityErrors;
        statistics::Scalar disconnects;
        statistics::Scalar escapeErrors;
        statistics::Scalar creditErrors;
        statistics::Scalar rmapCommands;
        statistics::Scalar timeCodes;
        statistics::Scalar totalCycles;   // For throughput calculation
        statistics::Formula avgPacketSize;
        statistics::Formula throughput;
    } stats;

    // Events
    EventFunctionWrapper txEvent;
    EventFunctionWrapper rxEvent;
    EventFunctionWrapper linkEvent;

    // Internal methods
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);

    // Link operations
    void linkStateMachine();
    void startLink();
    void stopLink();
    void resetLink();

    // Transmit operations
    void startTx();
    void processTx();
    void completeTx();
    bool sendPacket(const std::vector<uint8_t>& data);

    // Receive operations
    void processRx();
    void completeRx();
    bool receivePacket(std::vector<uint8_t>& data);

    // RMAP operations
    void processRmapCommand(const std::vector<uint8_t>& packet);
    void sendRmapReply(const RmapHeader& cmd, uint8_t status,
                       const std::vector<uint8_t>& data);
    bool validateRmapCommand(const RmapHeader& cmd);

    // Time-code operations
    void processTimeCode(uint8_t code);
    void sendTimeCode(uint8_t code);

    // Utility methods
    uint8_t computeCrc(const uint8_t* data, unsigned length);
    unsigned calculateTxTime(unsigned bytes);

  public:
    PARAMS(SpaceWireNIC);
    SpaceWireNIC(const Params &params);

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();

    // External interface for link simulation
    void externalRx(const std::vector<uint8_t>& packet);
    bool isLinkUp() const { return linkState == LinkState::RUN; }

    // Version information
    static const uint32_t VERSION = 0x01000000;  // v1.0.0.0
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_SPACEWIRE_NIC_HH__



