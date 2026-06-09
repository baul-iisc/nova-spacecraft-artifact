#include "arch/riscv/insts/matrix.hh"

#include <sstream>
#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/regs/matrix.hh"
#include "arch/riscv/utility.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

std::string
MatrixOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixStoreMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1)) << ", " 
        << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
MatrixStoreMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1)) << ", " 
        << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
MatrixArithLineMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}
std::string
MatrixStoreDoubleMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1)) << ", " 
        << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
MatrixStoreDoubleMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1)) << ", " 
        << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
MatrixArithLineDoubleMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixUnaryArithMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0));
    return ss.str();
}

std::string
MatrixUnaryArithMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0));
    return ss.str();
}

std::string
MatrixMoveMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0));
    return ss.str();
}

std::string
MatrixMoveMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0));
    return ss.str();
}

//Add these methods to the matrix.cc file

// Systolic Config/Control/Status
std::string
SystolicConfigMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(0));
    return ss.str();
}

std::string
SystolicConfigMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(0));
    return ss.str();
}

std::string
SystolicControlMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic;
    return ss.str();
}

std::string
SystolicControlMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic;
    return ss.str();
}

std::string
SystolicStatusMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    return ss.str();
}

std::string
SystolicStatusMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    return ss.str();
}

// Systolic Memory Operations
std::string
SystolicLoadMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
SystolicLoadMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
SystolicStoreMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(srcRegIdx(0)) << ", " 
       << registerName(srcRegIdx(1)) << ", "
       << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
SystolicStoreMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(srcRegIdx(0)) << ", " 
       << registerName(srcRegIdx(1)) << ", "
       << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
SystolicLoadDoubleMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
SystolicLoadDoubleMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
SystolicStoreDoubleMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(srcRegIdx(0)) << ", " 
       << registerName(srcRegIdx(1)) << ", "
       << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
SystolicStoreDoubleMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(srcRegIdx(0)) << ", " 
       << registerName(srcRegIdx(1)) << ", "
       << registerName(srcRegIdx(2));
    return ss.str();
}

// Systolic Matrix Operations
std::string
SystolicMmacMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
SystolicMmacMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
SystolicMmacDoubleMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
SystolicMmacDoubleMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
       << registerName(destRegIdx(0)) << ", " 
       << registerName(srcRegIdx(0)) << ", "
       << registerName(srcRegIdx(1));
    return ss.str();
}

// Add these implementations to src/arch/riscv/insts/matrix.cc
// before the closing namespace braces

std::string
MatrixVectorLoadMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorStoreMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1)) << ", " 
        << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
MatrixVectorStoreMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1)) << ", " 
        << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
MatrixVectorMulMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorMulMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorMacMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorMacMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorAddMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorAddMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorSubMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorSubMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixInvMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0));
    return ss.str();
}

std::string
MatrixInvMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0));
    return ss.str();
}

std::string
MatrixVectorMacAccMacroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}

std::string
MatrixVectorMacAccMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const {
    std::stringstream ss;
    ss << mnemonic << ' ' 
        << registerName(destRegIdx(0)) << ", " 
        << registerName(srcRegIdx(0)) << ", " 
        << registerName(srcRegIdx(1));
    return ss.str();
}
} // namespace RiscvISA
} // namespace gem5
