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

constexpr unsigned MaxMLenInBits = 256;
constexpr unsigned MaxMLenInBytes = MaxMLenInBits >> 3;

using MatRegContainer = gem5::MatStore;
using mreg_t = MatRegContainer;

const int NumMatrixStandardRegs = 32;
const int NumMatrixInternalRegs = 0;
const int NumMatrixRegs = NumMatrixStandardRegs + NumMatrixInternalRegs;

const std::vector<std::string> MatrixRegNames = {
     "m0",   "m1",   "m2",   "m3",   "m4",   "m5",   "m6",   "m7",
     "m8",   "m9",  "m10",  "m11",  "m12",  "m13",  "m14",  "m15",
    "m16",  "m17",  "m18",  "m19",  "m20",  "m21",  "m22",  "m23",
    "m24",  "m25",  "m26",  "m27",  "m28",  "m29",  "m30",  "m31"
};

static inline TypedRegClassOps<RiscvISA::MatRegContainer> matRegClassOps;

inline constexpr RegClass matRegClass = 
    RegClass(MatRegClass, MatRegClassName, NumMatrixRegs, debug::MatRegs).
        ops(matRegClassOps).
        regType<MatRegContainer>();

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_MATRIX_HH__
