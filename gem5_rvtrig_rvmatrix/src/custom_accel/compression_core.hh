/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * Compression Core for Spacecraft Telemetry Data
 * Implements CCSDS Rice-like compression algorithm
 */

#ifndef __CUSTOM_ACCEL_COMPRESSION_CORE_HH__
#define __CUSTOM_ACCEL_COMPRESSION_CORE_HH__

#include <queue>
#include <vector>

#include "mem/port.hh"
#include "params/CompressionCore.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * CompressionCore - A memory-mapped compression accelerator for spacecraft
 * 
 * This accelerator implements CCSDS Rice-like compression suitable for:
 * - Telemetry (TM) data compression
 * - Science data compression
 * - Uplink data decompression
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x00: CTRL     - Control register (start/stop/mode)
 * 0x04: STATUS   - Status register (busy/done/error)
 * 0x08: SRC_ADDR - Source data address
 * 0x0C: DST_ADDR - Destination address for compressed data
 * 0x10: LENGTH   - Input data length (bytes)
 * 0x14: OUT_LEN  - Output compressed length (read-only)
 * 0x18: CONFIG   - Compression configuration (k-parameter, block size)
 * 0x1C: STATS    - Compression statistics (ratio)
 */
class CompressionCore : public ClockedObject
{
  private:
    /**
     * CPU-side port for receiving memory-mapped requests
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        CompressionCore *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, CompressionCore *owner);
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
     * Memory-side port for DMA access to main memory
     */
    class MemSidePort : public RequestPort
    {
      private:
        CompressionCore *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, CompressionCore *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Register offsets
    static const Addr REG_CTRL      = 0x00;
    static const Addr REG_STATUS    = 0x04;
    static const Addr REG_SRC_ADDR  = 0x08;
    static const Addr REG_DST_ADDR  = 0x0C;
    static const Addr REG_LENGTH    = 0x10;
    static const Addr REG_OUT_LEN   = 0x14;
    static const Addr REG_CONFIG    = 0x18;
    static const Addr REG_STATS     = 0x1C;

    // Control register bits
    static const uint32_t CTRL_START      = 0x01;
    static const uint32_t CTRL_COMPRESS   = 0x02;  // 1=compress, 0=decompress
    static const uint32_t CTRL_RESET      = 0x80;

    // Status register bits
    static const uint32_t STATUS_BUSY     = 0x01;
    static const uint32_t STATUS_DONE     = 0x02;
    static const uint32_t STATUS_ERROR    = 0x04;

    // Internal registers
    uint32_t regCtrl;
    uint32_t regStatus;
    Addr regSrcAddr;
    Addr regDstAddr;
    uint32_t regLength;
    uint32_t regOutLen;
    uint32_t regConfig;
    uint32_t regStats;

    // Compression parameters
    unsigned kParam;           // Rice k-parameter (bits for remainder)
    unsigned blockSize;        // Block size for compression
    unsigned compressionLatency;  // Cycles per block

    // Internal state
    bool busy;
    std::vector<uint8_t> inputBuffer;
    std::vector<uint8_t> outputBuffer;
    unsigned bytesProcessed;

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;

    // Base address for MMIO
    AddrRange addrRange;

    // Statistics
    struct CompressionStats : public statistics::Group
    {
        CompressionStats(CompressionCore *parent);
        statistics::Scalar compressionOps;
        statistics::Scalar decompressionOps;
        statistics::Scalar bytesCompressed;
        statistics::Scalar bytesDecompressed;
        statistics::Scalar totalCycles;
        statistics::Formula avgCompressionRatio;
    } stats;

    // Event for compression completion
    EventFunctionWrapper compressionEvent;

    // Internal methods
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);
    void startCompression();
    void startDecompression();
    void completeOperation();

    // Rice compression/decompression
    void riceCompress(const std::vector<uint8_t>& input, 
                      std::vector<uint8_t>& output);
    void riceDecompress(const std::vector<uint8_t>& input,
                        std::vector<uint8_t>& output,
                        size_t originalSize);

  public:
    PARAMS(CompressionCore);
    CompressionCore(const Params &params);

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_COMPRESSION_CORE_HH__

