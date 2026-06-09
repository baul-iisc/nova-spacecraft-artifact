/**
 * @file cxl_interface.hh
 * @brief CXL (Compute Express Link) Interface Controller
 *
 * PhD Research: Chandraboul
 *
 * This module implements a CXL interface controller supporting:
 * - CXL.io: PCIe-like I/O protocol
 * - CXL.cache: Cache coherent access to host memory
 * - CXL.mem: Memory access protocol for attached memory
 *
 * Features:
 * - 4 CXL ports for external connectivity
 * - Cache-coherent memory expansion
 * - Low-latency memory access
 * - Support for CXL 2.0 features
 */

#ifndef __CUSTOM_ACCEL_CXL_INTERFACE_HH__
#define __CUSTOM_ACCEL_CXL_INTERFACE_HH__

#include <array>
#include <queue>
#include <vector>

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/CXLInterface.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * CXL Transaction types
 */
enum class CXLTransType {
    CXL_IO_READ,
    CXL_IO_WRITE,
    CXL_CACHE_READ,
    CXL_CACHE_WRITE,
    CXL_MEM_READ,
    CXL_MEM_WRITE,
    CXL_CACHE_SNOOP,
    CXL_CACHE_RESPONSE
};

/**
 * CXL Port state
 */
struct CXLPort {
    bool enabled;
    bool linkUp;
    unsigned linkSpeed;      // GT/s
    unsigned linkWidth;      // x1, x2, x4, x8, x16
    uint64_t txBytes;
    uint64_t rxBytes;
    uint64_t errors;
};

/**
 * CXL Interface Controller
 */
class CXLInterface : public ClockedObject
{
  public:
    PARAMS(CXLInterface);
    CXLInterface(const Params &p);
    ~CXLInterface();

    void init() override;

    Port& getPort(const std::string& if_name,
                  PortID idx = InvalidPortID) override;

    // Number of CXL ports
    static constexpr unsigned NUM_CXL_PORTS = 4;

  protected:
    /**
     * CPU-side port for control/status access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        CXLInterface* owner;

      public:
        CPUSidePort(const std::string& name, CXLInterface* owner);

        AddrRangeList getAddrRanges() const override;
        bool recvTimingReq(PacketPtr pkt) override;
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvRespRetry() override {}
    };

    /**
     * Memory-side port for CXL memory access
     */
    class MemSidePort : public RequestPort
    {
      private:
        CXLInterface* owner;
        unsigned portId;

      public:
        MemSidePort(const std::string& name, CXLInterface* owner, unsigned id);

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    };

    CPUSidePort cpuPort;
    std::array<MemSidePort*, NUM_CXL_PORTS> memPorts;

    // Port configurations
    std::array<CXLPort, NUM_CXL_PORTS> ports;

    // Configuration
    AddrRange addrRange;
    unsigned defaultLinkSpeed;
    unsigned defaultLinkWidth;
    Cycles accessLatency;

    // Transaction queues per port
    std::array<std::queue<PacketPtr>, NUM_CXL_PORTS> txQueues;
    std::array<std::queue<PacketPtr>, NUM_CXL_PORTS> rxQueues;

    // Register interface
    static constexpr Addr REG_STATUS        = 0x00;
    static constexpr Addr REG_CONTROL       = 0x04;
    static constexpr Addr REG_PORT_BASE     = 0x10;  // Per-port registers
    static constexpr Addr REG_PORT_SIZE     = 0x20;
    static constexpr Addr REG_PORT_STATUS   = 0x00;
    static constexpr Addr REG_PORT_CONTROL  = 0x04;
    static constexpr Addr REG_PORT_LINK     = 0x08;
    static constexpr Addr REG_PORT_TX_COUNT = 0x10;
    static constexpr Addr REG_PORT_RX_COUNT = 0x18;

    // Status bits
    static constexpr uint32_t STATUS_ENABLED = 1 << 0;
    static constexpr uint32_t STATUS_ERROR   = 1 << 1;

    // Port status bits
    static constexpr uint32_t PORT_ENABLED   = 1 << 0;
    static constexpr uint32_t PORT_LINK_UP   = 1 << 1;
    static constexpr uint32_t PORT_ERROR     = 1 << 2;

    uint32_t statusReg;
    uint32_t controlReg;

    uint64_t read(Addr addr);
    void write(Addr addr, uint64_t data);

    // Processing
    void processTransactions();
    EventFunctionWrapper processEvent;

  protected:
    struct CXLStats : public statistics::Group
    {
        CXLStats(CXLInterface* parent);

        statistics::Scalar totalTxBytes;
        statistics::Scalar totalRxBytes;
        statistics::Scalar totalTransactions;
        statistics::Scalar cacheCoherentAccesses;
        statistics::Scalar memoryAccesses;
        statistics::Scalar errors;
        statistics::Vector portTxBytes;
        statistics::Vector portRxBytes;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_CXL_INTERFACE_HH__

