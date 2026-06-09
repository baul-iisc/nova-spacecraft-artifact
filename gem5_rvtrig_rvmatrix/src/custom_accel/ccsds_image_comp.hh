/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * CCSDS 122.0 Image Compression Accelerator
 * 
 * Implements CCSDS 122.0-B-2 Image Data Compression standard
 * Designed for spacecraft payload data compression:
 * - Earth observation imagery
 * - Science instrument data
 * - Surveillance imagery
 * 
 * Features:
 * - Discrete Wavelet Transform (DWT) using 9/7 or 5/3 filters
 * - Bit-Plane Encoder (BPE) with segment-based coding
 * - Configurable compression ratio
 * - Lossless and lossy modes
 * - Support for 8-16 bit samples
 */

#ifndef __CUSTOM_ACCEL_CCSDS_IMAGE_COMP_HH__
#define __CUSTOM_ACCEL_CCSDS_IMAGE_COMP_HH__

#include <array>
#include <queue>
#include <vector>
#include <cstdint>

#include "mem/port.hh"
#include "params/CCSDSImageComp.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * CCSDSImageComp - CCSDS 122.0-B-2 Image Compression Accelerator
 * 
 * Implements the CCSDS image compression standard optimized for:
 * - High-throughput satellite imaging
 * - Science data compression
 * - Onboard autonomous processing
 * 
 * Processing Pipeline:
 * 1. 2D Discrete Wavelet Transform (DWT)
 *    - 3-level decomposition
 *    - 9/7 float filter (lossy) or 5/3 integer filter (lossless)
 * 
 * 2. Bit-Plane Encoder (BPE)
 *    - Segment-based encoding
 *    - DC coefficient coding
 *    - AC coefficient coding
 *    - Arithmetic/Golomb-Rice entropy coding
 * 
 * 3. Rate Control
 *    - Target bits-per-pixel control
 *    - Quality layers for progressive transmission
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x000: CTRL           - Control register
 * 0x004: STATUS         - Status register  
 * 0x008: MODE           - Operating mode (compress/decompress)
 * 0x00C: INPUT_BASE     - Input image base address
 * 0x010: OUTPUT_BASE    - Output compressed data address
 * 0x014: IMG_WIDTH      - Image width in pixels
 * 0x018: IMG_HEIGHT     - Image height in pixels
 * 0x01C: BIT_DEPTH      - Sample bit depth (8-16)
 * 0x020: SEGMENT_SIZE   - Segment size for BPE
 * 0x024: DWT_LEVELS     - Number of DWT decomposition levels
 * 0x028: WAVELET_TYPE   - Wavelet filter (0=9/7 lossy, 1=5/3 lossless)
 * 0x02C: TARGET_BPP     - Target bits-per-pixel (fixed point 8.8)
 * 0x030: OUTPUT_LEN     - Compressed output length (read-only)
 * 0x034: COMP_RATIO     - Achieved compression ratio (read-only)
 * 0x038: QUALITY        - Quality metric (PSNR approximation)
 * 0x03C: TILE_WIDTH     - Tile width for tiled processing
 * 0x040: TILE_HEIGHT    - Tile height for tiled processing
 * 0x044: HEADER_CONFIG  - Header configuration options
 * 0x048: PERF_PIXELS    - Performance: pixels processed
 * 0x04C: PERF_CYCLES    - Performance: total cycles
 * 0x050: PERF_BYTES_IN  - Performance: input bytes
 * 0x054: PERF_BYTES_OUT - Performance: output bytes
 * 0x058: INTERRUPT      - Interrupt status/control
 * 0x05C: VERSION        - Accelerator version
 */
class CCSDSImageComp : public ClockedObject
{
  private:
    /**
     * CPU-side port for MMIO access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        CCSDSImageComp *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, CCSDSImageComp *owner);
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
        CCSDSImageComp *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, CCSDSImageComp *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Operating modes
    enum class Mode : uint8_t {
        IDLE = 0,
        COMPRESS = 1,
        DECOMPRESS = 2,
        DWT_ONLY = 3,           // DWT transform only
        BPE_ONLY = 4            // BPE encoding only
    };

    // Wavelet filter types
    enum class WaveletType : uint8_t {
        LOSSY_9_7 = 0,          // 9/7 biorthogonal (lossy)
        LOSSLESS_5_3 = 1        // 5/3 integer wavelet (lossless)
    };

    // Register offsets
    static const Addr REG_CTRL          = 0x000;
    static const Addr REG_STATUS        = 0x004;
    static const Addr REG_MODE          = 0x008;
    static const Addr REG_INPUT_BASE    = 0x00C;
    static const Addr REG_OUTPUT_BASE   = 0x010;
    static const Addr REG_IMG_WIDTH     = 0x014;
    static const Addr REG_IMG_HEIGHT    = 0x018;
    static const Addr REG_BIT_DEPTH     = 0x01C;
    static const Addr REG_SEGMENT_SIZE  = 0x020;
    static const Addr REG_DWT_LEVELS    = 0x024;
    static const Addr REG_WAVELET_TYPE  = 0x028;
    static const Addr REG_TARGET_BPP    = 0x02C;
    static const Addr REG_OUTPUT_LEN    = 0x030;
    static const Addr REG_COMP_RATIO    = 0x034;
    static const Addr REG_QUALITY       = 0x038;
    static const Addr REG_TILE_WIDTH    = 0x03C;
    static const Addr REG_TILE_HEIGHT   = 0x040;
    static const Addr REG_HEADER_CONFIG = 0x044;
    static const Addr REG_PERF_PIXELS   = 0x048;
    static const Addr REG_PERF_CYCLES   = 0x04C;
    static const Addr REG_PERF_BYTES_IN = 0x050;
    static const Addr REG_PERF_BYTES_OUT = 0x054;
    static const Addr REG_INTERRUPT     = 0x058;
    static const Addr REG_VERSION       = 0x05C;

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
    static const uint32_t STATUS_DWT_DONE   = 0x08;
    static const uint32_t STATUS_BPE_DONE   = 0x10;

    // DWT filter coefficients (9/7 filter)
    static constexpr float ALPHA = -1.586134342f;
    static constexpr float BETA = -0.05298011854f;
    static constexpr float GAMMA = 0.8829110762f;
    static constexpr float DELTA = 0.4435068522f;
    static constexpr float SCALE_LO = 1.149604398f;
    static constexpr float SCALE_HI = 1.0f / 1.149604398f;

    // 5/3 filter coefficients (integers for lossless)
    static constexpr int LIFT_5_3_PREDICT = -1;
    static constexpr int LIFT_5_3_UPDATE = 1;

    // Internal registers
    uint32_t regCtrl;
    uint32_t regStatus;
    uint32_t regMode;
    Addr regInputBase;
    Addr regOutputBase;
    uint32_t regImgWidth;
    uint32_t regImgHeight;
    uint32_t regBitDepth;
    uint32_t regSegmentSize;
    uint32_t regDwtLevels;
    uint32_t regWaveletType;
    uint32_t regTargetBpp;      // Fixed point 8.8
    uint32_t regOutputLen;
    uint32_t regCompRatio;      // Fixed point 8.8
    uint32_t regQuality;        // PSNR estimate
    uint32_t regTileWidth;
    uint32_t regTileHeight;
    uint32_t regHeaderConfig;
    uint64_t regPerfPixels;
    uint64_t regPerfCycles;
    uint64_t regPerfBytesIn;
    uint64_t regPerfBytesOut;
    uint32_t regInterrupt;

    // Hardware parameters (configurable)
    unsigned dwtLatency;        // Cycles per DWT coefficient
    unsigned bpeLatency;        // Cycles per BPE segment
    unsigned lineBufferSize;    // Line buffer size for DWT
    unsigned maxTileSize;       // Maximum tile size

    // Internal buffers
    std::vector<int16_t> inputBuffer;       // Input image
    std::vector<int32_t> coeffBuffer;       // DWT coefficients
    std::vector<uint8_t> outputBuffer;      // Compressed output
    std::vector<float> dwtLineBuffer;       // Line buffer for DWT

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // Statistics
    struct ImageCompStats : public statistics::Group
    {
        ImageCompStats(CCSDSImageComp *parent);
        statistics::Scalar imagesCompressed;
        statistics::Scalar imagesDecompressed;
        statistics::Scalar tilesProcessed;
        statistics::Scalar totalPixels;
        statistics::Scalar totalInputBytes;
        statistics::Scalar totalOutputBytes;
        statistics::Scalar totalCycles;
        statistics::Formula avgCompressionRatio;
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

    // Main processing methods
    void startOperation();
    void executeCompress();
    void executeDecompress();
    void executeDwtOnly();
    void executeBpeOnly();
    void completeOperation();

    // DWT operations
    void dwtForward2D(std::vector<int32_t>& data, unsigned width, unsigned height);
    void dwtInverse2D(std::vector<int32_t>& data, unsigned width, unsigned height);
    void dwtForward1D_97(float* data, unsigned length);
    void dwtInverse1D_97(float* data, unsigned length);
    void dwtForward1D_53(int32_t* data, unsigned length);
    void dwtInverse1D_53(int32_t* data, unsigned length);

    // BPE operations
    void bpeEncode(const std::vector<int32_t>& coeffs, 
                   std::vector<uint8_t>& output);
    void bpeDecode(const std::vector<uint8_t>& input,
                   std::vector<int32_t>& coeffs);
    void encodeSegment(const int32_t* coeffs, unsigned count,
                       std::vector<uint8_t>& output);
    void decodeSegment(const uint8_t* input, unsigned& inputLen,
                       int32_t* coeffs, unsigned count);

    // Entropy coding
    void golombRiceEncode(int32_t value, unsigned k, 
                          std::vector<uint8_t>& output, unsigned& bitPos);
    int32_t golombRiceDecode(const uint8_t* input, unsigned& bitPos, unsigned k);

    // Header operations
    void writeHeader(std::vector<uint8_t>& output);
    bool readHeader(const std::vector<uint8_t>& input, unsigned& offset);

    // Utility methods
    unsigned estimateCycles(unsigned pixels, unsigned levels);
    uint32_t estimateCompressedSize();

  public:
    PARAMS(CCSDSImageComp);
    CCSDSImageComp(const Params &params);

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();

    // Version information
    static const uint32_t VERSION = 0x01000000;  // v1.0.0.0
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_CCSDS_IMAGE_COMP_HH__



