//src/arch/riscv/insts/matrix.hh
#ifndef __ARCH_RISCV_INSTS_MATRIX_HH__
#define __ARCH_RISCV_INSTS_MATRIX_HH__

#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/matrix.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/utility.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

/**
 * Base class for Matrix operations
 */
class MatrixOp : public RiscvStaticInst
{
  protected:
    MatrixOp(const char *mnem, ExtMachInst _extMachInst, OpClass __opClass)
        : RiscvStaticInst(mnem, _extMachInst, __opClass)
    {
        this->flags[IsMatrix] = true;
    };

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixNonSplitInst : public MatrixOp
{
  protected:
    MatrixNonSplitInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixOp(mnem, _machInst, __opClass)
    {}
};

class MatrixMacroInst : public RiscvMacroInst
{
  protected:
    MatrixMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : RiscvMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsMacroop] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixMicroInst : public RiscvMicroInst
{
  protected:
    uint32_t microIdx;

    MatrixMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : RiscvMicroInst(mnem, _machInst, __opClass),
        microIdx(_microIdx)
    {
        this->flags[IsMicroop] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixArithMacroInst : public MatrixMacroInst {
  protected:
    MatrixArithMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}
};

class MatrixArithMicroInst : public MatrixMicroInst {
  protected:
    MatrixArithMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}
};

class MatrixArithDoubleMacroInst : public MatrixMacroInst {
  protected:
    MatrixArithDoubleMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}
};

class MatrixArithDoubleMicroInst : public MatrixMicroInst {
  protected:
    MatrixArithDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}
};

// TODO: Unused
class MatrixUnaryArithMacroInst : public MatrixArithMacroInst {
  protected:
    MatrixUnaryArithMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixArithMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixUnaryArithMicroInst : public MatrixArithMicroInst {
  protected:
    MatrixUnaryArithMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixArithMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixArithLineMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixArithLineMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx
                          ), 
          memAccessFlags(0)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixArithLineDoubleMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixArithLineDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx
                          ), 
          memAccessFlags(0)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};
class MatrixMemMacroInst : public MatrixMacroInst {
  protected:
    MatrixMemMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}
};

class MatrixMemMicroInst : public MatrixMicroInst {
  protected:
    MatrixMemMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}
};

class MatrixLoadMacroInst : public MatrixMemMacroInst {
  protected:
    MatrixLoadMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsLoad] = true;
    }
};

class MatrixLoadMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixLoadMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsLoad] = true;
    }
};

class MatrixStoreMacroInst : public MatrixMemMacroInst {
  protected:
    MatrixStoreMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixStoreMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixStoreMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixLoadDoubleMacroInst : public MatrixMemMacroInst {
  protected:
    MatrixLoadDoubleMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsLoad] = true;
    }
};

class MatrixLoadDoubleMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixLoadDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsLoad] = true;
    }
};

class MatrixStoreDoubleMacroInst : public MatrixMemMacroInst {
  protected:
    MatrixStoreDoubleMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixStoreDoubleMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixStoreDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};
class MatrixMoveMacroInst : public MatrixMacroInst {
  protected:
    MatrixMoveMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixMoveMicroInst : public MatrixMicroInst {
  protected:
    MatrixMoveMicroInst(const char* mnem, ExtMachInst _machInst,
                        OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Systolic Array Configuration and Control
class SystolicConfigMacroInst : public MatrixMacroInst {
  protected:
    SystolicConfigMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicConfigMicroInst : public MatrixMicroInst {
  protected:
    SystolicConfigMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicControlMacroInst : public MatrixMacroInst {
  protected:
    SystolicControlMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicControlMicroInst : public MatrixMicroInst {
  protected:
    SystolicControlMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicStatusMacroInst : public MatrixMacroInst {
  protected:
    SystolicStatusMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicStatusMicroInst : public MatrixMicroInst {
  protected:
    SystolicStatusMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Systolic Memory Operations
class SystolicLoadMacroInst : public MatrixMemMacroInst {
  protected:
    SystolicLoadMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsLoad] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicLoadMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    SystolicLoadMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsLoad] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicStoreMacroInst : public MatrixMemMacroInst {
  protected:
    SystolicStoreMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicStoreMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    SystolicStoreMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicLoadDoubleMacroInst : public MatrixMemMacroInst {
  protected:
    SystolicLoadDoubleMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsLoad] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicLoadDoubleMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    SystolicLoadDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsLoad] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicStoreDoubleMacroInst : public MatrixMemMacroInst {
  protected:
    SystolicStoreDoubleMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicStoreDoubleMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    SystolicStoreDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Systolic Matrix Operations
class SystolicMmacMacroInst : public MatrixMacroInst {
  protected:
    SystolicMmacMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicMmacMicroInst : public MatrixMicroInst {
  protected:
    SystolicMmacMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicMmacDoubleMacroInst : public MatrixMacroInst {
  protected:
    SystolicMmacDoubleMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class SystolicMmacDoubleMicroInst : public MatrixMicroInst {
  protected:
    SystolicMmacDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Add these class definitions to src/arch/riscv/insts/matrix.hh
// before the closing namespace brace

// Matrix-Vector Load Operation Classes
class MatrixVectorLoadMacroInst : public MatrixMemMacroInst {
  protected:
    MatrixVectorLoadMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsLoad] = true;
    }
};

class MatrixVectorLoadMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixVectorLoadMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsLoad] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Matrix-Vector Store Operation Classes
class MatrixVectorStoreMacroInst : public MatrixMemMacroInst {
  protected:
    MatrixVectorStoreMacroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass)
        : MatrixMemMacroInst(mnem, _machInst, __opClass)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixVectorStoreMicroInst : public MatrixMicroInst {
  protected:
    Request::Flags memAccessFlags;
    
    MatrixVectorStoreMicroInst(const char* mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx), 
        memAccessFlags(0)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Matrix-Vector Arithmetic Operation Base Classes
class MatrixVectorArithMacroInst : public MatrixMacroInst {
  protected:
    MatrixVectorArithMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}
};

class MatrixVectorArithMicroInst : public MatrixMicroInst {
  protected:
    MatrixVectorArithMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}
};

// Matrix-Vector Multiply Operation Classes
class MatrixVectorMulMacroInst : public MatrixVectorArithMacroInst {
  protected:
    MatrixVectorMulMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixVectorArithMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixVectorMulMicroInst : public MatrixVectorArithMicroInst {
  protected:
    MatrixVectorMulMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixVectorArithMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Matrix-Vector Multiply-Accumulate Operation Classes
class MatrixVectorMacMacroInst : public MatrixVectorArithMacroInst {
  protected:
    MatrixVectorMacMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixVectorArithMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixVectorMacMicroInst : public MatrixVectorArithMicroInst {
  protected:
    MatrixVectorMacMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixVectorArithMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Matrix-Vector Add Operation Classes
class MatrixVectorAddMacroInst : public MatrixVectorArithMacroInst {
  protected:
    MatrixVectorAddMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixVectorArithMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixVectorAddMicroInst : public MatrixVectorArithMicroInst {
  protected:
    MatrixVectorAddMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixVectorArithMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Matrix-Vector Subtract Operation Classes
class MatrixVectorSubMacroInst : public MatrixVectorArithMacroInst {
  protected:
    MatrixVectorSubMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : MatrixVectorArithMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixVectorSubMicroInst : public MatrixVectorArithMicroInst {
  protected:
    MatrixVectorSubMicroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microIdx)
        : MatrixVectorArithMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Add these class definitions to src/arch/riscv/insts/matrix.hh
// Matrix Inversion Operation Classes - Make sure these inherit from existing classes properly

class MatrixInvMacroInst : public MatrixMacroInst {
  protected:
    MatrixInvMacroInst(const char* mnem, ExtMachInst _machInst,
                       OpClass __opClass)
        : MatrixMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixInvMicroInst : public MatrixMicroInst {
  protected:
    MatrixInvMicroInst(const char* mnem, ExtMachInst _machInst,
                       OpClass __opClass, uint32_t _microIdx)
        : MatrixMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// Single-precision specific implementations
class MatrixInvFloatMacroInst : public MatrixInvMacroInst {
  protected:
    MatrixInvFloatMacroInst(const char* mnem, ExtMachInst _machInst,
                           OpClass __opClass)
        : MatrixInvMacroInst(mnem, _machInst, __opClass)
    {}
};

class MatrixInvFloatMicroInst : public MatrixInvMicroInst {
  protected:
    MatrixInvFloatMicroInst(const char* mnem, ExtMachInst _machInst,
                           OpClass __opClass, uint32_t _microIdx)
        : MatrixInvMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}
};

// Double-precision specific implementations
class MatrixInvDoubleMacroInst : public MatrixInvMacroInst {
  protected:
    MatrixInvDoubleMacroInst(const char* mnem, ExtMachInst _machInst,
                            OpClass __opClass)
        : MatrixInvMacroInst(mnem, _machInst, __opClass)
    {}
};

class MatrixInvDoubleMicroInst : public MatrixInvMicroInst {
  protected:
    MatrixInvDoubleMicroInst(const char* mnem, ExtMachInst _machInst,
                            OpClass __opClass, uint32_t _microIdx)
        : MatrixInvMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}
};

// Matrix-Vector Multiply-Accumulate with Accumulator Operation Classes
class MatrixVectorMacAccMacroInst : public MatrixVectorArithMacroInst {
  protected:
    MatrixVectorMacAccMacroInst(const char* mnem, ExtMachInst _machInst,
                             OpClass __opClass)
        : MatrixVectorArithMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class MatrixVectorMacAccMicroInst : public MatrixVectorArithMicroInst {
  protected:
    MatrixVectorMacAccMicroInst(const char* mnem, ExtMachInst _machInst,
                             OpClass __opClass, uint32_t _microIdx)
        : MatrixVectorArithMicroInst(mnem, _machInst, __opClass, _microIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_INSTS_MATRIX_HH__
