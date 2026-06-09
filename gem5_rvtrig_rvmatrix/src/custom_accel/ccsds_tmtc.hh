/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * CCSDS TM/TC (Telemetry/Telecommand) Protocol Accelerator
 * 
 * Implements CCSDS space packet protocol standards:
 * - CCSDS 132.0-B-2 TM Space Data Link Protocol
 * - CCSDS 232.0-B-3 TC Space Data Link Protocol  
 * - CCSDS 131.0-B-3 TM Synchronization and Channel Coding
 * - CCSDS 231.0-B-3 TC Synchronization and Channel Coding
 * - CCSDS 133.0-B-1 Space Packet Protocol
 */

#ifndef __CUSTOM_ACCEL_CCSDS_TMTC_HH__
#define __CUSTOM_ACCEL_CCSDS_TMTC_HH__

#include <array>
#include <queue>
#include <vector>
#include <cstdint>

#include "mem/port.hh"
#include "params/CCSDSTmTc.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * CCSDSTmTc - Hardware accelerator for CCSDS TM/TC protocol processing
 * 
 * This accelerator offloads spacecraft communication protocol processing:
 * 
 * Telemetry (TM) Path:
 * - Space Packet assembly/disassembly
 * - Transfer Frame generation
 * - Virtual Channel multiplexing
 * - Reed-Solomon encoding (RS(255,223))
 * - LDPC encoding (optional)
 * - Convolutional encoding (rate 1/2, K=7)
 * - Randomization (CCSDS pseudorandom sequence)
 * - CRC-16 (CCITT) computation
 * 
 * Telecommand (TC) Path:
 * - Frame synchronization detection (ASM)
 * - Derandomization
 * - BCH decoding (TC frames)
 * - CRC verification
 * - Transfer Frame extraction
 * - Command extraction and validation
 * - Command authentication (optional SDLS)
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x000: CTRL           - Control register
 * 0x004: STATUS         - Status register
 * 0x008: MODE           - Operating mode (TM/TC)
 * 0x00C: SCID           - Spacecraft ID (10 bits)
 * 0x010: VCID           - Virtual Channel ID
 * 0x014: INPUT_BASE     - Input buffer base address
 * 0x018: OUTPUT_BASE    - Output buffer base address
 * 0x01C: INPUT_LEN      - Input data length
 * 0x020: OUTPUT_LEN     - Output data length (read-only)
 * 0x024: FRAME_COUNT    - Frame sequence counter
 * 0x028: TM_CONFIG      - TM configuration
 * 0x02C: TC_CONFIG      - TC configuration
 * 0x030: RS_CONFIG      - Reed-Solomon configuration
 * 0x034: CONV_CONFIG    - Convolutional encoder config
 * 0x038: CRC_RESULT     - CRC computation result
 * 0x03C: ERR_COUNT      - Error counter
 * 0x040: SYNC_PATTERN   - Synchronization marker pattern
 * 0x044: RAND_SEED      - Randomizer seed
 * 0x048: AUTH_KEY_L     - Authentication key (low 32 bits)
 * 0x04C: AUTH_KEY_H     - Authentication key (high 32 bits)
 * 0x050: PERF_FRAMES    - Performance: frames processed
 * 0x054: PERF_CYCLES    - Performance: total cycles
 * 0x058: PERF_ERRORS    - Performance: errors detected
 * 0x05C: INTERRUPT      - Interrupt status/control
 * 0x060: VERSION        - Accelerator version
 */
class CCSDSTmTc : public ClockedObject
{
  private:
    /**
     * CPU-side port for MMIO access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        CCSDSTmTc *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, CCSDSTmTc *owner);
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
        CCSDSTmTc *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, CCSDSTmTc *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Operating modes
    enum class Mode : uint8_t {
        IDLE = 0,
        TM_ENCODE = 1,      // Telemetry encoding (spacecraft -> ground)
        TM_DECODE = 2,      // Telemetry decoding (for local processing)
        TC_DECODE = 3,      // Telecommand decoding (ground -> spacecraft)
        TC_ENCODE = 4,      // Telecommand encoding (for test/simulation)
        PACKET_ASSEMBLE = 5,// Space packet assembly
        PACKET_EXTRACT = 6  // Space packet extraction
    };

    // Frame types
    enum class FrameType : uint8_t {
        TM_TRANSFER = 0,
        TC_TRANSFER = 1,
        AOS_TRANSFER = 2,
        PROXIMITY_LINK = 3
    };

    // Coding types
    enum class CodingType : uint8_t {
        UNCODED = 0,
        RS_ONLY = 1,        // Reed-Solomon only
        CONV_ONLY = 2,      // Convolutional only
        RS_CONV = 3,        // RS + Convolutional (concatenated)
        LDPC = 4,           // LDPC (for high rate)
        TURBO = 5           // Turbo codes
    };

    // Register offsets
    static const Addr REG_CTRL          = 0x000;
    static const Addr REG_STATUS        = 0x004;
    static const Addr REG_MODE          = 0x008;
    static const Addr REG_SCID          = 0x00C;
    static const Addr REG_VCID          = 0x010;
    static const Addr REG_INPUT_BASE    = 0x014;
    static const Addr REG_OUTPUT_BASE   = 0x018;
    static const Addr REG_INPUT_LEN     = 0x01C;
    static const Addr REG_OUTPUT_LEN    = 0x020;
    static const Addr REG_FRAME_COUNT   = 0x024;
    static const Addr REG_TM_CONFIG     = 0x028;
    static const Addr REG_TC_CONFIG     = 0x02C;
    static const Addr REG_RS_CONFIG     = 0x030;
    static const Addr REG_CONV_CONFIG   = 0x034;
    static const Addr REG_CRC_RESULT    = 0x038;
    static const Addr REG_ERR_COUNT     = 0x03C;
    static const Addr REG_SYNC_PATTERN  = 0x040;
    static const Addr REG_RAND_SEED     = 0x044;
    static const Addr REG_AUTH_KEY_L    = 0x048;
    static const Addr REG_AUTH_KEY_H    = 0x04C;
    static const Addr REG_PERF_FRAMES   = 0x050;
    static const Addr REG_PERF_CYCLES   = 0x054;
    static const Addr REG_PERF_ERRORS   = 0x058;
    static const Addr REG_INTERRUPT     = 0x05C;
    static const Addr REG_VERSION       = 0x060;

    // Control bits
    static const uint32_t CTRL_START    = 0x01;
    static const uint32_t CTRL_ABORT    = 0x02;
    static const uint32_t CTRL_RESET    = 0x80;
    static const uint32_t CTRL_IRQ_EN   = 0x100;

    // Status bits
    static const uint32_t STATUS_IDLE       = 0x00;
    static const uint32_t STATUS_BUSY       = 0x01;
    static const uint32_t STATUS_DONE       = 0x02;
    static const uint32_t STATUS_ERROR      = 0x04;
    static const uint32_t STATUS_CRC_ERROR  = 0x08;
    static const uint32_t STATUS_SYNC_LOST  = 0x10;
    static const uint32_t STATUS_AUTH_FAIL  = 0x20;

    // CCSDS constants
    static const uint32_t TM_SYNC_MARKER = 0x1ACFFC1D;
    static const uint32_t TC_SYNC_MARKER = 0xFAF320;
    static const unsigned TM_FRAME_SIZE = 1115;     // Including RS check symbols
    static const unsigned TC_FRAME_SIZE = 64;       // Variable, this is max
    static const unsigned RS_SYMBOLS = 32;          // RS(255,223) parity symbols
    static const uint16_t CRC16_POLY = 0x1021;      // CCITT polynomial

    // Internal registers
    uint32_t regCtrl;
    uint32_t regStatus;
    uint32_t regMode;
    uint32_t regScid;
    uint32_t regVcid;
    Addr regInputBase;
    Addr regOutputBase;
    uint32_t regInputLen;
    uint32_t regOutputLen;
    uint32_t regFrameCount;
    uint32_t regTmConfig;
    uint32_t regTcConfig;
    uint32_t regRsConfig;
    uint32_t regConvConfig;
    uint32_t regCrcResult;
    uint32_t regErrCount;
    uint32_t regSyncPattern;
    uint32_t regRandSeed;
    uint64_t regAuthKey;
    uint64_t regPerfFrames;
    uint64_t regPerfCycles;
    uint64_t regPerfErrors;
    uint32_t regInterrupt;

    // Hardware parameters
    unsigned frameLatency;          // Cycles per frame processing
    unsigned rsLatency;             // Cycles for RS encoding/decoding
    unsigned convLatency;           // Cycles for convolutional coding

    // Internal buffers
    std::vector<uint8_t> inputBuffer;
    std::vector<uint8_t> outputBuffer;
    std::vector<uint8_t> rsEncoder;     // RS encoder lookup table
    std::vector<uint8_t> convState;     // Convolutional encoder state

    // Randomizer state (CCSDS PN sequence)
    uint8_t randRegister[32];

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // Statistics
    struct TMTCStats : public statistics::Group
    {
        TMTCStats(CCSDSTmTc *parent);
        statistics::Scalar tmFramesProcessed;
        statistics::Scalar tcFramesProcessed;
        statistics::Scalar packetsAssembled;
        statistics::Scalar packetsExtracted;
        statistics::Scalar crcErrors;
        statistics::Scalar rsCorrections;
        statistics::Scalar syncLosses;
        statistics::Scalar totalCycles;
        statistics::Scalar bytesProcessed;
        statistics::Formula throughput;
    } stats;

    // Events
    EventFunctionWrapper processEvent;

    // Internal methods
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);

    // Processing methods
    void startOperation();
    void processTmEncode();
    void processTmDecode();
    void processTcDecode();
    void processTcEncode();
    void assembleSpacePacket();
    void extractSpacePacket();
    void completeOperation();

    // Protocol processing helpers
    void buildTmFrame(const std::vector<uint8_t>& data);
    void buildTcFrame(const std::vector<uint8_t>& data);
    bool syncSearch(const std::vector<uint8_t>& data, uint32_t pattern);
    
    // Coding operations
    void rsEncode(std::vector<uint8_t>& data);
    bool rsDecode(std::vector<uint8_t>& data, unsigned& corrections);
    void convEncode(const std::vector<uint8_t>& input, std::vector<uint8_t>& output);
    void convDecode(const std::vector<uint8_t>& input, std::vector<uint8_t>& output);
    void randomize(std::vector<uint8_t>& data);
    void derandomize(std::vector<uint8_t>& data);
    
    // CRC operations
    uint16_t computeCrc16(const std::vector<uint8_t>& data);
    bool verifyCrc16(const std::vector<uint8_t>& data, uint16_t expected);

    // Initialize lookup tables
    void initRsEncoder();
    void initRandomizer();

  public:
    PARAMS(CCSDSTmTc);
    CCSDSTmTc(const Params &params);

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();

    // Version information
    static const uint32_t VERSION = 0x01000000;  // v1.0.0.0
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_CCSDS_TMTC_HH__



