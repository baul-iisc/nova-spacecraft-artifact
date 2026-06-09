/*
 * NOVA Processor - Vision Processing Unit (VPU) Instructions
 * PhD Research: Futuristic Spacecraft Processor with VPU Accelerator
 *
 * This file defines the VPU instruction classes for gem5 simulation.
 * VPU instructions use OpClass::SimdMisc for contention tracking.
 */

#ifndef __ARCH_RISCV_INSTS_VPU_HH__
#define __ARCH_RISCV_INSTS_VPU_HH__

#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

/**
 * Base class for VPU instructions
 */
class VpuOp : public RiscvStaticInst
{
  protected:
    VpuOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : RiscvStaticInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

/**
 * VPU Convolution instruction
 * Performs 2D convolution on image data using hardware acceleration
 */
class VpuConv2d : public VpuOp
{
  public:
    VpuConv2d(ExtMachInst _machInst)
        : VpuOp("vpu.conv2d", _machInst, OpClass::SimdMisc)
    {}

    Fault execute(ExecContext *, trace::InstRecord *) const override;
};

/**
 * VPU Sobel edge detection
 */
class VpuSobel : public VpuOp
{
  public:
    VpuSobel(ExtMachInst _machInst)
        : VpuOp("vpu.sobel", _machInst, OpClass::SimdMisc)
    {}

    Fault execute(ExecContext *, trace::InstRecord *) const override;
};

/**
 * VPU Harris corner detection
 */
class VpuHarris : public VpuOp
{
  public:
    VpuHarris(ExtMachInst _machInst)
        : VpuOp("vpu.harris", _machInst, OpClass::SimdMisc)
    {}

    Fault execute(ExecContext *, trace::InstRecord *) const override;
};

/**
 * VPU ORB feature extraction
 */
class VpuOrb : public VpuOp
{
  public:
    VpuOrb(ExtMachInst _machInst)
        : VpuOp("vpu.orb", _machInst, OpClass::SimdMisc)
    {}

    Fault execute(ExecContext *, trace::InstRecord *) const override;
};

/**
 * VPU Optical flow computation
 */
class VpuFlow : public VpuOp
{
  public:
    VpuFlow(ExtMachInst _machInst)
        : VpuOp("vpu.flow", _machInst, OpClass::SimdMisc)
    {}

    Fault execute(ExecContext *, trace::InstRecord *) const override;
};

/**
 * VPU Feature matching
 */
class VpuMatch : public VpuOp
{
  public:
    VpuMatch(ExtMachInst _machInst)
        : VpuOp("vpu.match", _machInst, OpClass::SimdMisc)
    {}

    Fault execute(ExecContext *, trace::InstRecord *) const override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_INSTS_VPU_HH__





