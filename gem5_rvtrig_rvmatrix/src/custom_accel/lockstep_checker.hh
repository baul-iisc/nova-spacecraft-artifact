/**
 * @file lockstep_checker.hh
 * @brief Dual-core Lockstep (DLS) Checker for Spacecraft Fault Tolerance
 *
 * PhD Research: Chandraboul
 *
 * This module implements a Dual-core Lockstep checker that:
 * - Monitors instruction execution results from paired cores
 * - Compares results between primary and secondary (checker) cores
 * - On mismatch: signals pipeline flush and re-execution
 * - Provides fault detection and recovery statistics
 *
 * DLS Architecture:
 * - 8 cores = 4 DLS pairs
 * - Each pair: Primary Core + Checker Core
 * - Both cores execute same instruction stream
 * - Results compared at commit stage
 * - Mismatch triggers recovery
 */

#ifndef __CUSTOM_ACCEL_LOCKSTEP_CHECKER_HH__
#define __CUSTOM_ACCEL_LOCKSTEP_CHECKER_HH__

#include <queue>
#include <vector>
#include <array>

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/LockstepChecker.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * Structure to hold instruction commit information for comparison
 */
struct CommitInfo {
    uint64_t pc;              // Program counter
    uint64_t inst;            // Instruction encoding
    uint64_t result;          // Execution result (register value)
    uint64_t addr;            // Memory address (if applicable)
    uint64_t data;            // Memory data (if applicable)
    bool isMemOp;             // Is this a memory operation
    bool isStore;             // Is this a store operation
    uint64_t seqNum;          // Instruction sequence number
    Tick commitTick;          // Tick when instruction committed
};

/**
 * Structure for a DLS pair
 */
struct DLSPair {
    unsigned primaryCoreId;    // Primary core ID
    unsigned checkerCoreId;    // Checker/secondary core ID
    
    // Commit queues for comparison
    std::queue<CommitInfo> primaryQueue;
    std::queue<CommitInfo> checkerQueue;
    
    // Status
    bool synchronized;         // Are both cores synchronized
    uint64_t lastMatchedSeq;   // Last matched sequence number
    
    // Statistics per pair
    uint64_t comparisons;      // Total comparisons
    uint64_t mismatches;       // Total mismatches detected
    uint64_t recoveries;       // Successful recoveries
};

/**
 * Lockstep Checker SimObject
 */
class LockstepChecker : public ClockedObject
{
  public:
    PARAMS(LockstepChecker);
    LockstepChecker(const Params &p);
    ~LockstepChecker();

    /**
     * Initialize the checker with the specified number of DLS pairs
     */
    void init() override;

    /**
     * Record a committed instruction from a core
     * Called by CPU models at instruction commit
     */
    void recordCommit(unsigned coreId, const CommitInfo& info);

    /**
     * Check if a core is a primary core in a DLS pair
     */
    bool isPrimaryCore(unsigned coreId) const;

    /**
     * Check if a core is a checker core in a DLS pair
     */
    bool isCheckerCore(unsigned coreId) const;

    /**
     * Get the pair ID for a given core
     */
    int getPairId(unsigned coreId) const;

    /**
     * Signal that a mismatch requires pipeline flush
     * Returns true if re-execution should occur
     */
    bool handleMismatch(unsigned pairId, const CommitInfo& primary,
                        const CommitInfo& checker);

    /**
     * Get the port for CPU-side connections
     */
    Port& getPort(const std::string& if_name,
                  PortID idx = InvalidPortID) override;

    // Configuration
    static constexpr unsigned MAX_DLS_PAIRS = 4;
    static constexpr unsigned NUM_CORES = 8;
    static constexpr unsigned COMMIT_QUEUE_SIZE = 64;

  protected:
    /**
     * CPU-side port for receiving commit information
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        LockstepChecker* owner;
        unsigned coreId;

      public:
        CPUSidePort(const std::string& name, LockstepChecker* owner, 
                    unsigned core_id);

        AddrRangeList getAddrRanges() const override;
        bool recvTimingReq(PacketPtr pkt) override;
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvRespRetry() override {}
    };

    // Ports for each core
    std::vector<CPUSidePort*> cpuPorts;

    // DLS Pairs
    std::array<DLSPair, MAX_DLS_PAIRS> dlsPairs;
    unsigned numPairs;

    // Configuration
    unsigned comparisonLatency;   // Cycles to compare results
    unsigned recoveryLatency;     // Cycles to recover from mismatch
    bool enableRecovery;          // Enable automatic recovery
    bool strictMode;              // Strict comparison mode

    /**
     * Process commits and perform comparison
     */
    void processCommits(unsigned pairId);

    /**
     * Compare two commit records
     */
    bool compareCommits(const CommitInfo& a, const CommitInfo& b) const;

    /**
     * Trigger recovery for a pair
     */
    void triggerRecovery(unsigned pairId);

    /**
     * Event for periodic comparison processing
     */
    EventFunctionWrapper processEvent;
    void processEventHandler();

    // Memory-mapped register interface
    AddrRange addrRange;
    
    // Register offsets
    static constexpr Addr REG_STATUS         = 0x00;  // Status register
    static constexpr Addr REG_CONTROL        = 0x04;  // Control register
    static constexpr Addr REG_PAIR_STATUS    = 0x08;  // Per-pair status base
    static constexpr Addr REG_MISMATCH_COUNT = 0x20;  // Mismatch counter base
    static constexpr Addr REG_RECOVERY_COUNT = 0x30;  // Recovery counter base
    static constexpr Addr REG_LAST_MISMATCH  = 0x40;  // Last mismatch info base

    // Status bits
    static constexpr uint32_t STATUS_ENABLED     = 1 << 0;
    static constexpr uint32_t STATUS_SYNCHRONIZED = 1 << 1;
    static constexpr uint32_t STATUS_MISMATCH    = 1 << 2;
    static constexpr uint32_t STATUS_RECOVERY    = 1 << 3;

    uint32_t statusReg;
    uint32_t controlReg;

    /**
     * Handle memory-mapped read
     */
    uint64_t read(Addr addr);

    /**
     * Handle memory-mapped write
     */
    void write(Addr addr, uint64_t data);

  protected:
    // Statistics
    struct LockstepStats : public statistics::Group
    {
        LockstepStats(LockstepChecker* parent);

        statistics::Scalar totalComparisons;
        statistics::Scalar totalMismatches;
        statistics::Scalar totalRecoveries;
        statistics::Scalar falsePositives;
        statistics::Distribution mismatchLatency;
        statistics::Vector pairMismatches;
    } stats;
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_LOCKSTEP_CHECKER_HH__

