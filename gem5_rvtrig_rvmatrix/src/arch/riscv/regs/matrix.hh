//src/arch/riscv/regs/matrix.hh
#ifndef __ARCH_RISCV_REGS_MATRIX_HH__
#define __ARCH_RISCV_REGS_MATRIX_HH__

#include <cstdint>
#include <string>
#include <vector>

#include "arch/riscv/types.hh"
#include "arch/riscv/matrix.hh"
#include "base/bitunion.hh"
#include "cpu/reg_class.hh"
#include "debug/MatRegs.hh"

namespace gem5
{

namespace RiscvISA
{

constexpr unsigned MaxMLenInBits = 192; // For 3x3 matrices
constexpr unsigned MaxMLenInBytes = MaxMLenInBits >> 3;

using MatRegContainer = gem5::MatStore;
using mreg_t = MatRegContainer;

const int NumMatrixStandardRegs = 32;
const int NumAccumulatorRegs = 8;
const int NumSystolicArrayRegs = 4; // Control registers for systolic array
const int NumMatrixInternalRegs = 0;
const int NumMatrixRegs = NumMatrixStandardRegs + NumAccumulatorRegs + 
                          NumSystolicArrayRegs + NumMatrixInternalRegs;

// Define start of accumulator registers
const int ACC_REG_START = NumMatrixStandardRegs;
// Define start of systolic array control registers
const int SYS_REG_START = ACC_REG_START + NumAccumulatorRegs;

const std::vector<std::string> MatrixRegNames = {
     "m0",   "m1",   "m2",   "m3",   "m4",   "m5",   "m6",   "m7",
     "m8",   "m9",  "m10",  "m11",  "m12",  "m13",  "m14",  "m15",
    "m16",  "m17",  "m18",  "m19",  "m20",  "m21",  "m22",  "m23",
    "m24",  "m25",  "m26",  "m27",  "m28",  "m29",  "m30",  "m31",
    "acc0", "acc1", "acc2", "acc3", "acc4", "acc5", "acc6", "acc7",
    "sys0", "sys1", "sys2", "sys3" // Systolic array control registers
};

// Definitions for systolic array control registers
const int SYS_CONFIG_REG = SYS_REG_START + 0;     // Configuration register
const int SYS_STATUS_REG = SYS_REG_START + 1;     // Status register
const int SYS_RESULT_REG = SYS_REG_START + 2;     // Result register
const int SYS_CONTROL_REG = SYS_REG_START + 3;    // Control register

// Bit definitions for systolic array status register
constexpr uint32_t SYS_STATUS_IDLE      = 0;      // Systolic array is idle
constexpr uint32_t SYS_STATUS_BUSY      = 1;      // Systolic array is busy
constexpr uint32_t SYS_STATUS_DONE      = 2;      // Computation complete
constexpr uint32_t SYS_STATUS_ERROR     = 3;      // Error occurred

// Bit definitions for systolic array control register
constexpr uint32_t SYS_CTRL_START       = 1 << 0; // Start computation
constexpr uint32_t SYS_CTRL_STOP        = 1 << 1; // Stop computation
constexpr uint32_t SYS_CTRL_RESET       = 1 << 2; // Reset systolic array
constexpr uint32_t SYS_CTRL_INT_EN      = 1 << 3; // Enable interrupts

static inline TypedRegClassOps<RiscvISA::MatRegContainer> matRegClassOps;

// Function to map register index to appropriate physical register index
// This function is critical for accumulator register operations
static inline int mapMatrixReg(int regIdx, bool isAccumulator) {
    if (isAccumulator) {
        // For accumulator operations, add the offset
        return ACC_REG_START + regIdx;
    } else if (regIdx >= ACC_REG_START && regIdx < SYS_REG_START) {
        // This is already an accumulator register index
        return regIdx;
    } else if (regIdx >= SYS_REG_START && regIdx < NumMatrixRegs) {
        // This is already a systolic array register index
        return regIdx;
    } else {
        // Normal matrix register
        return regIdx;
    }
}

// Helper to check if a register index refers to an accumulator register
static inline bool isAccumulatorReg(int regIdx) {
    return (regIdx >= ACC_REG_START && regIdx < SYS_REG_START);
}

// For backward compatibility
static inline int mapAccumulatorReg(int regIdx) {
    return mapMatrixReg(regIdx, true);
}

inline constexpr RegClass matRegClass = 
    RegClass(MatRegClass, MatRegClassName, NumMatrixRegs, debug::MatRegs).
        ops(matRegClassOps).
        regType<MatRegContainer>();

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_MATRIX_HH__
