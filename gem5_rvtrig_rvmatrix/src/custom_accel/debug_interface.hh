/**
 * @file debug_interface.hh
 * @brief Hardware Debug Interface Controller
 *
 * PhD Research: Chandraboul
 *
 * This module implements a hardware debug interface supporting:
 * - Breakpoint setting and management
 * - Memory read/write access
 * - Register read/write access
 * - Single-step execution control
 * - Run/halt control
 * - Debug triggers
 */

#ifndef __CUSTOM_ACCEL_DEBUG_INTERFACE_HH__
#define __CUSTOM_ACCEL_DEBUG_INTERFACE_HH__

#include <array>
#include <set>
#include <vector>

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/DebugInterface.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Breakpoint types
 */
enum class BreakpointType {
    EXECUTE,        // Instruction execution breakpoint
    READ,           // Memory read breakpoint
    WRITE,          // Memory write breakpoint
    ACCESS          // Memory access (read or write) breakpoint
};

/**
 * Breakpoint entry
 */
struct Breakpoint {
    bool enabled;
    BreakpointType type;
    Addr address;
    Addr addressMask;       // For address range matching
    unsigned size;          // Size for memory breakpoints
    unsigned hitCount;
    unsigned targetHits;    // Break after N hits (0 = break immediately)
};

/**
 * Debug Interface Controller
 */
class DebugInterface : public ClockedObject
{
  public:
    PARAMS(DebugInterface);
    DebugInterface(const Params &p);
    ~DebugInterface();

    void init() override;

    Port& getPort(const std::string& if_name,
                  PortID idx = InvalidPortID) override;

    // Maximum number of breakpoints
    static constexpr unsigned MAX_BREAKPOINTS = 8;
    
    // Maximum number of cores supported
    static constexpr unsigned MAX_CORES = 8;

    /**
     * Check if an address hits a breakpoint
     */
    bool checkBreakpoint(Addr addr, BreakpointType type, unsigned coreId);

    /**
     * Check if a core is halted
     */
    bool isCoreHalted(unsigned coreId) const;

    /**
     * Halt a specific core
     */
    void haltCore(unsigned coreId);

    /**
     * Resume a specific core
     */
    void resumeCore(unsigned coreId);

    /**
     * Single-step a core
     */
    void singleStep(unsigned coreId);

  protected:
    /**
     * CPU-side port for control/status access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        DebugInterface* owner;

      public:
        CPUSidePort(const std::string& name, DebugInterface* owner);

        AddrRangeList getAddrRanges() const override;
        bool recvTimingReq(PacketPtr pkt) override;
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvRespRetry() override {}
    };

    /**
     * Memory-side port for memory access operations
     */
    class MemSidePort : public RequestPort
    {
      private:
        DebugInterface* owner;

      public:
        MemSidePort(const std::string& name, DebugInterface* owner);

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    };

    CPUSidePort cpuPort;
    MemSidePort memPort;

    // Breakpoints
    std::array<Breakpoint, MAX_BREAKPOINTS> breakpoints;
    unsigned numBreakpoints;

    // Core states
    std::array<bool, MAX_CORES> coreHalted;
    std::array<bool, MAX_CORES> coreSingleStep;
    unsigned numCores;

    // Configuration
    AddrRange addrRange;

    // Register offsets
    static constexpr Addr REG_STATUS          = 0x00;
    static constexpr Addr REG_CONTROL         = 0x04;
    static constexpr Addr REG_HALT_STATUS     = 0x08;
    static constexpr Addr REG_HALT_CONTROL    = 0x0C;
    static constexpr Addr REG_STEP_CONTROL    = 0x10;
    static constexpr Addr REG_BP_BASE         = 0x40;   // Breakpoint registers start
    static constexpr Addr REG_BP_SIZE         = 0x10;   // Size per breakpoint
    static constexpr Addr REG_BP_CTRL         = 0x00;   // Offset within BP: control
    static constexpr Addr REG_BP_ADDR         = 0x04;   // Offset within BP: address
    static constexpr Addr REG_BP_MASK         = 0x08;   // Offset within BP: mask
    static constexpr Addr REG_BP_COUNT        = 0x0C;   // Offset within BP: hit count
    static constexpr Addr REG_MEM_ADDR        = 0x100;  // Memory access address
    static constexpr Addr REG_MEM_DATA        = 0x108;  // Memory access data
    static constexpr Addr REG_MEM_CTRL        = 0x110;  // Memory access control
    static constexpr Addr REG_REG_SEL         = 0x120;  // Register select (core + reg)
    static constexpr Addr REG_REG_DATA        = 0x128;  // Register data

    // Status bits
    static constexpr uint32_t STATUS_ENABLED     = 1 << 0;
    static constexpr uint32_t STATUS_HALTED      = 1 << 1;
    static constexpr uint32_t STATUS_BREAKPOINT  = 1 << 2;
    static constexpr uint32_t STATUS_STEP_DONE   = 1 << 3;

    // Control bits
    static constexpr uint32_t CTRL_ENABLE        = 1 << 0;
    static constexpr uint32_t CTRL_HALT_ALL      = 1 << 1;
    static constexpr uint32_t CTRL_RESUME_ALL    = 1 << 2;
    static constexpr uint32_t CTRL_RESET         = 1 << 3;

    // Breakpoint control bits
    static constexpr uint32_t BP_ENABLE          = 1 << 0;
    static constexpr uint32_t BP_TYPE_MASK       = 0x3 << 1;
    static constexpr uint32_t BP_TYPE_EXEC       = 0x0 << 1;
    static constexpr uint32_t BP_TYPE_READ       = 0x1 << 1;
    static constexpr uint32_t BP_TYPE_WRITE      = 0x2 << 1;
    static constexpr uint32_t BP_TYPE_ACCESS     = 0x3 << 1;

    uint32_t statusReg;
    uint32_t controlReg;
    Addr memAccessAddr;
    uint64_t memAccessData;
    uint32_t memAccessCtrl;
    uint32_t regSelectReg;
    uint64_t regDataReg;

    uint64_t read(Addr addr);
    void write(Addr addr, uint64_t data);

    // Memory access helpers
    void performMemoryRead();
    void performMemoryWrite();

  protected:
    struct DebugStats : public statistics::Group
    {
        DebugStats(DebugInterface* parent);

        statistics::Scalar breakpointHits;
        statistics::Scalar memoryReads;
        statistics::Scalar memoryWrites;
        statistics::Scalar registerReads;
        statistics::Scalar registerWrites;
        statistics::Scalar singleSteps;
        statistics::Scalar halts;
        statistics::Scalar resumes;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_DEBUG_INTERFACE_HH__

