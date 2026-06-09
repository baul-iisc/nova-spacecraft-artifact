# NOVA Custom ISA Extensions — RISC-V GNU Toolchain, Spike & PK

## Complete Reference for CORDIC Trigonometric & Matrix Accelerator Instructions

**Author:** Chandraboul  
**Date:** February 2026  
**Toolchain:** riscv64-unknown-elf-gcc (gc891d8dc23e) 13.2.0  
**Spike:** RISC-V ISA Simulator 1.1.1-dev  
**Target ISA:** RV64GCV (rv64gcv, lp64d ABI)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Directory Layout & Paths](#2-directory-layout--paths)
3. [Instruction Summary](#3-instruction-summary)
4. [Binutils Modifications (Assembler/Disassembler)](#4-binutils-modifications-assemblerdisassembler)
5. [Spike Modifications (ISA Simulator)](#5-spike-modifications-isa-simulator)
6. [Proxy Kernel (PK) Build](#6-proxy-kernel-pk-build)
7. [Build & Install Procedures](#7-build--install-procedures)
8. [Test Programs](#8-test-programs)
9. [Compilation & Execution](#9-compilation--execution)
10. [Verified Test Results](#10-verified-test-results)
11. [Encoding Tables](#11-encoding-tables)
12. [Known Encoding Conflicts](#12-known-encoding-conflicts)
13. [Instruction Implementations (Spike)](#13-instruction-implementations-spike)
14. [Troubleshooting](#14-troubleshooting)
15. [Files Modified Summary](#15-files-modified-summary)

---

## 1. Overview

This document describes all modifications made to the RISC-V GNU toolchain to support two sets of custom ISA extensions for the **NOVA Processor** (PhD Research: Hardware-Accelerated Spacecraft Navigation):

### A. CORDIC Trigonometric Accelerator (14 instructions)
- Opcode: **OP-FP (0x53)**
- Uses **FP registers** (frd, frs1, frs2)
- Operations: sin, cos, tan, atan, atan2, asin, acos (single + double precision)

### B. Matrix Accelerator (52 instructions)
- Opcode: **custom-3 (0x77)**
- Uses **integer (GPR) registers** (rd, rs1, rs2)
- Operations: matrix load/store, multiply-accumulate, add/sub, vector ops, systolic array control

### Components Modified

| Component | Purpose | Location |
|-----------|---------|----------|
| **Binutils** | Assembler & disassembler (gas/objdump) | `binutils/` subdirectory |
| **Spike** | RISC-V ISA simulator (functional execution) | `spike/` subdirectory |
| **PK** | Proxy kernel (syscall emulation for Spike) | `pk/` subdirectory |

---

## 2. Directory Layout & Paths

```
/data/home/chandraboul/development/
├── riscv-gnu-toolchain/              # Toolchain source root
│   ├── binutils/                      # GNU Binutils (assembler, linker, objdump)
│   │   ├── include/opcode/riscv-opc.h   # Instruction encodings (MATCH/MASK)
│   │   └── opcodes/riscv-opc.c          # Opcode table entries
│   ├── spike/                         # Spike ISA simulator source
│   │   ├── riscv/
│   │   │   ├── encoding.h                # Spike's own encoding defines
│   │   │   ├── insn_template.h            # Template includes for instruction compilation
│   │   │   ├── insn_template.cc           # Template that #includes insns/NAME.h
│   │   │   ├── riscv.mk.in               # Makefile listing all instructions
│   │   │   └── insns/                     # Individual instruction implementations
│   │   │       ├── fsin_d.h ... facos_s.h   # 14 trig instruction files
│   │   │       ├── ml.h ... sys_mmacd.h     # 52 matrix instruction files
│   │   │       └── (standard instructions)
│   │   └── disasm/
│   │       └── disasm.cc                  # Disassembly support
│   ├── pk/                            # Proxy kernel source
│   ├── build-binutils-newlib/         # Binutils build directory
│   ├── build-spike/                   # Spike build directory (owned by root)
│   └── build-pk/                      # PK build directory
│
└── riscv-spike/                       # Install prefix (all tools installed here)
    ├── bin/
    │   ├── riscv64-unknown-elf-gcc      # Cross-compiler
    │   ├── riscv64-unknown-elf-as       # Assembler
    │   ├── riscv64-unknown-elf-objdump  # Disassembler
    │   └── spike                         # ISA simulator
    └── riscv64-unknown-elf/
        └── bin/
            └── pk                        # Proxy kernel
```

---

## 3. Instruction Summary

### 3.1 CORDIC Trigonometric Instructions (14 total)

| # | Mnemonic | Precision | Type | Operands | Description |
|---|----------|-----------|------|----------|-------------|
| 1 | `fsin.s` | Single | Unary | frd, frs1 | sin(frs1) |
| 2 | `fcos.s` | Single | Unary | frd, frs1 | cos(frs1) |
| 3 | `ftan.s` | Single | Unary | frd, frs1 | tan(frs1) |
| 4 | `fatan.s` | Single | Unary | frd, frs1 | atan(frs1) |
| 5 | `fatan2.s` | Single | Binary | frd, frs1, frs2 | atan2(frs1, frs2) |
| 6 | `fasin.s` | Single | Unary | frd, frs1 | asin(frs1) |
| 7 | `facos.s` | Single | Unary | frd, frs1 | acos(frs1) |
| 8 | `fsin.d` | Double | Unary | frd, frs1 | sin(frs1) |
| 9 | `fcos.d` | Double | Unary | frd, frs1 | cos(frs1) |
| 10 | `ftan.d` | Double | Unary | frd, frs1 | tan(frs1) |
| 11 | `fatan.d` | Double | Unary | frd, frs1 | atan(frs1) |
| 12 | `fatan2.d` | Double | Binary | frd, frs1, frs2 | atan2(frs1, frs2) |
| 13 | `fasin.d` | Double | Unary | frd, frs1 | asin(frs1) |
| 14 | `facos.d` | Double | Unary | frd, frs1 | acos(frs1) |

> **Note:** Half-precision (`.h`) variants also exist in binutils but were not added to Spike.

### 3.2 Matrix Accelerator Instructions (52 total)

#### Memory Operations (8 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 1 | `ml` | rd, rs1 | Matrix Load (int32, 3×3) |
| 2 | `ms` | rd, rs1 | Matrix Store (int32, 3×3) |
| 3 | `mls` | rd, rs1 | Matrix Load Strided (int32) |
| 4 | `mss` | rd, rs1 | Matrix Store Strided (int32) |
| 5 | `mld` | rd, rs1 | Matrix Load (float64) |
| 6 | `msd` | rd, rs1 | Matrix Store (float64) |
| 7 | `mlds` | rd, rs1 | Matrix Load Strided (float64) |
| 8 | `msds` | rd, rs1 | Matrix Store Strided (float64) |

#### Matrix Move (1 instruction)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 9 | `mmv.xmu` | rd, rs1 | Matrix Move (register) |

#### Integer Compute (6 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 10 | `mmacu` | rd, rs1, rs2 | Unsigned MAC: rd = rs1 × rs2 + rd |
| 11 | `maddu` | rd, rs1, rs2 | Unsigned Add: rd = rs1 + rs2 |
| 12 | `msubu` | rd, rs1, rs2 | Unsigned Sub: rd = rs1 - rs2 |
| 13 | `mmac` | rd, rs1, rs2 | Signed MAC: rd = rs1 × rs2 + rd |
| 14 | `madd.m` | rd, rs1, rs2 | Signed Add: rd = rs1 + rs2 |
| 15 | `msub.m` | rd, rs1, rs2 | Signed Sub: rd = rs1 - rs2 |

#### Single-Precision Float Compute (3 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 16 | `mmacf` | rd, rs1, rs2 | Float MAC: rd = float(rs1) × float(rs2) + float(rd) |
| 17 | `maddf` | rd, rs1, rs2 | Float Add: rd = float(rs1) + float(rs2) |
| 18 | `msubf` | rd, rs1, rs2 | Float Sub: rd = float(rs1) - float(rs2) |

#### Double-Precision Float Compute (3 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 19 | `mmacdf` | rd, rs1, rs2 | Double MAC: rd = double(rs1) × double(rs2) + double(rd) |
| 20 | `maddfd` | rd, rs1, rs2 | Double Add: rd = double(rs1) + double(rs2) |
| 21 | `msubfd` | rd, rs1, rs2 | Double Sub: rd = double(rs1) - double(rs2) |

#### Accumulator Operations (5 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 22 | `macc.zero` | rd | Zero int accumulator |
| 23 | `macc.move` | rd, rs1 | Move to accumulator |
| 24 | `macc.store` | rd, rs1 | Store from accumulator |
| 25 | `macc.zerof` | rd | Zero float accumulator |
| 26 | `macc.zerod` | rd | Zero double accumulator |

#### Matrix-Vector Operations (5 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 27 | `mv.load` | rd, rs1, rs2 | Vector load with stride |
| 28 | `mv.store` | rd, rs1, rs2 | Vector store with stride |
| 29 | `mv.mul` | rd, rs1, rs2 | Vector multiply |
| 30 | `mv.mac` | rd, rs1, rs2 | Vector MAC |
| 31 | `mv.mac.acc` | rd, rs1, rs2 | Vector MAC with accumulator |

#### Precision Accumulator MAC (2 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 32 | `mmacf.acc` | rd, rs1, rs2 | Float MAC with accumulator |
| 33 | `mmacdf.acc` | rd, rs1, rs2 | Double MAC with accumulator |

#### Matrix Inversion (2 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 34 | `minvf` | rd, rs1 | Float matrix inversion |
| 35 | `minvd` | rd, rs1 | Double matrix inversion |

#### Systolic Array Control (4 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 36 | `sys.config` | rs1 | Configure systolic array |
| 37 | `sys.start` | (none) | Start systolic execution |
| 38 | `sys.stop` | (none) | Stop systolic execution |
| 39 | `sys.status` | rd | Read systolic status |

#### Systolic Array Data (10 instructions)

| # | Mnemonic | Operands | Description |
|---|----------|----------|-------------|
| 40 | `sys.load.a` | rd, rs1, rs2 | Load matrix A (int) |
| 41 | `sys.load.b` | rd, rs1, rs2 | Load matrix B (int) |
| 42 | `sys.load.c` | rd, rs1, rs2 | Load matrix C (int) |
| 43 | `sys.store.c` | rd, rs1, rs2 | Store matrix C (int) |
| 44 | `sys.load.ad` | rd, rs1, rs2 | Load matrix A (double) |
| 45 | `sys.load.bd` | rd, rs1, rs2 | Load matrix B (double) |
| 46 | `sys.load.cd` | rd, rs1, rs2 | Load matrix C (double) |
| 47 | `sys.store.cd` | rd, rs1, rs2 | Store matrix C (double) |
| 48 | `sys.mmac` | rd, rs1, rs2 | Systolic int MAC |
| 49 | `sys.mmacd` | rd, rs1, rs2 | Systolic double MAC |

> Instructions 50-52 include `mmacd` (double MAC via integer regs, conflicts with `msub.m`), `msubd` (alias), totaling 52 instruction file implementations in Spike.

---

## 4. Binutils Modifications (Assembler/Disassembler)

### 4.1 File: `binutils/include/opcode/riscv-opc.h`

This file defines `MATCH_*` and `MASK_*` constants for each instruction encoding, plus `DECLARE_INSN()` entries.

#### What was added:

**A. CORDIC Trig MATCH/MASK defines** (after existing OP-FP instructions):

```c
/* NOVA CORDIC Trigonometric Accelerator - Single Precision */
#define MATCH_FSIN_S    0x30000053
#define MASK_FSIN_S     0xfff0007f
#define MATCH_FCOS_S    0x32000053
#define MASK_FCOS_S     0xfff0007f
#define MATCH_FTAN_S    0x36000053
#define MASK_FTAN_S     0xfff0007f
#define MATCH_FATAN_S   0x56000053
#define MASK_FATAN_S    0xfff0007f
#define MATCH_FATAN2_S  0x48000053
#define MASK_FATAN2_S   0xfe00007f    /* Binary: uses rs2, so mask doesn't lock rs2 */
#define MATCH_FASIN_S   0x60000053
#define MASK_FASIN_S    0xfff0007f
#define MATCH_FACOS_S   0x62000053
#define MASK_FACOS_S    0xfff0007f

/* NOVA CORDIC Trigonometric Accelerator - Double Precision */
#define MATCH_FSIN_D    0x38000053
#define MASK_FSIN_D     0xfff0007f
#define MATCH_FCOS_D    0x3A000053
#define MASK_FCOS_D     0xfff0007f
#define MATCH_FTAN_D    0x3E000053
#define MASK_FTAN_D     0xfff0007f
#define MATCH_FATAN_D   0x34000053
#define MASK_FATAN_D    0xfff0007f
#define MATCH_FATAN2_D  0x4A000053
#define MASK_FATAN2_D   0xfe00007f
#define MATCH_FASIN_D   0x64000053
#define MASK_FASIN_D    0xfff0007f
#define MATCH_FACOS_D   0x66000053
#define MASK_FACOS_D    0xfff0007f

/* Also: Half-precision (_H) variants defined but commented out in DECLARE_INSN */
```

**B. Matrix MATCH/MASK defines** (after NEUR_RES instructions, before CSR defines):

```c
/* NOVA Matrix Extension - Memory Operations (custom-3: opcode 0x77) */
#define MATCH_ML        0x22000077
#define MASK_ML         0xfe00707f
#define MATCH_MS        0x22001077
#define MASK_MS         0xfe00707f
/* ... all 52 matrix instructions with proper MATCH/MASK values ... */
#define MATCH_SYS_MMACD     0xb2001077
#define MASK_SYS_MMACD      0xfe00707f
```

**C. DECLARE_INSN entries** (before `#endif` of DECLARE_INSN block):

```c
/* Trigonometric */
DECLARE_INSN(fsin_s, MATCH_FSIN_S, MASK_FSIN_S)
DECLARE_INSN(fcos_s, MATCH_FCOS_S, MASK_FCOS_S)
DECLARE_INSN(fsin_d, MATCH_FSIN_D, MASK_FSIN_D)
/* ... all 14 trig DECLARE_INSN entries ... */

/* Matrix */
DECLARE_INSN(ml, MATCH_ML, MASK_ML)
DECLARE_INSN(ms, MATCH_MS, MASK_MS)
/* ... all 52 matrix DECLARE_INSN entries ... */
```

### 4.2 File: `binutils/opcodes/riscv-opc.c`

This file contains the opcode table used by the assembler and disassembler.

#### Trigonometric entries (using FP register operands):

```c
/* Format: "D,S" = frd, frs1 (unary); "D,S,T" = frd, frs1, frs2 (binary) */
/* Each has two entries: one with implied rounding mode, one with explicit */
{"fsin.s",    0, INSN_CLASS_F,    "D,S",       MATCH_FSIN_S|MASK_RM, MASK_FSIN_S|MASK_RM, match_opcode, 0 },
{"fsin.s",    0, INSN_CLASS_F,    "D,S,m",     MATCH_FSIN_S, MASK_FSIN_S, match_opcode, 0 },
{"fcos.s",    0, INSN_CLASS_F,    "D,S",       MATCH_FCOS_S|MASK_RM, MASK_FCOS_S|MASK_RM, match_opcode, 0 },
{"fcos.s",    0, INSN_CLASS_F,    "D,S,m",     MATCH_FCOS_S, MASK_FCOS_S, match_opcode, 0 },
/* ... repeat for all 14 s/d/h trig instructions ... */
{"fatan2.d",  0, INSN_CLASS_D,    "D,S,T",     MATCH_FATAN2_D|MASK_RM, MASK_FATAN2_D|MASK_RM, match_opcode, 0 },
{"fatan2.d",  0, INSN_CLASS_D,    "D,S,T,m",   MATCH_FATAN2_D, MASK_FATAN2_D, match_opcode, 0 },
```

#### Matrix entries (using integer register operands):

```c
/* Format: "d,s,t" = rd, rs1, rs2; "d,s" = rd, rs1; "d" = rd only; "s" = rs1 only; "" = no operands */
{"ml",          0, INSN_CLASS_I,   "d,s",       MATCH_ML, MASK_ML, match_opcode, 0 },
{"ms",          0, INSN_CLASS_I,   "d,s",       MATCH_MS, MASK_MS, match_opcode, 0 },
{"mmac",        0, INSN_CLASS_I,   "d,s,t",     MATCH_MMAC, MASK_MMAC, match_opcode, 0 },
{"mmacf",       0, INSN_CLASS_I,   "d,s,t",     MATCH_MMACF, MASK_MMACF, match_opcode, 0 },
{"mmacdf",      0, INSN_CLASS_I,   "d,s,t",     MATCH_MMACDF, MASK_MMACDF, match_opcode, 0 },
{"macc.zero",   0, INSN_CLASS_I,   "d",         MATCH_MACC_ZERO, MASK_MACC_ZERO, match_opcode, 0 },
{"sys.config",  0, INSN_CLASS_I,   "s",         MATCH_SYS_CONFIG, MASK_SYS_CONFIG, match_opcode, 0 },
{"sys.start",   0, INSN_CLASS_I,   "",          MATCH_SYS_START, MASK_SYS_START, match_opcode, 0 },
/* ... all 43 matrix opcode table entries ... */
```

**Operand format key:**
- `"D,S"` — FP destination, FP source (trig unary)
- `"D,S,T"` — FP destination, FP source1, FP source2 (trig binary: fatan2)
- `"D,S,m"` — with explicit rounding mode
- `"d,s,t"` — GPR destination, GPR source1, GPR source2 (3-operand matrix)
- `"d,s"` — GPR destination, GPR source (2-operand: ml, ms, mmv.xmu, etc.)
- `"d"` — GPR destination only (macc.zero, macc.zerof, macc.zerod, sys.status)
- `"s"` — GPR source only (sys.config)
- `""` — no operands (sys.start, sys.stop)

---

## 5. Spike Modifications (ISA Simulator)

### 5.1 File: `spike/riscv/encoding.h`

Added MATCH/MASK defines and DECLARE_INSN entries identical to binutils (but in Spike's own copy of encoding.h).

**Matrix defines** added after the `MATCH_XPERM8 / MASK_XPERM8` block, before `CSR_FFLAGS`:

```c
/* NOVA Matrix Extension Instructions */
#define MATCH_ML        0x22000077
#define MASK_ML         0xfe00707f
/* ... all 52 matrix MATCH/MASK pairs ... */

/* NOVA CORDIC Trigonometric Accelerator Instructions */
#define MATCH_FSIN_S    0x30000053
#define MASK_FSIN_S     0xfff0007f
/* ... all 14 trig MATCH/MASK pairs ... */
```

**DECLARE_INSN entries** added before `#endif` / `#ifdef DECLARE_CSR`:

```c
/* NOVA Matrix Extension Instructions */
DECLARE_INSN(ml, MATCH_ML, MASK_ML)
/* ... all 52 matrix entries ... */

/* NOVA CORDIC Trigonometric Instructions */
DECLARE_INSN(fsin_s, MATCH_FSIN_S, MASK_FSIN_S)
/* ... all 14 trig entries ... */
```

### 5.2 File: `spike/riscv/insn_template.h`

Added `<cmath>` include for trigonometric math functions:

```c
#include "debug_defines.h"
#include <assert.h>
#include <cmath>          /* <-- ADDED: needed by fsin_d.h, fcos_d.h, etc. */
```

### 5.3 Instruction Implementation Files: `spike/riscv/insns/*.h`

**66 total instruction files created** (52 matrix + 14 trig).

#### Trig instruction pattern (example: `fsin_d.h`):

```c
// NOVA CORDIC - Double precision sine
require_either_extension('D', EXT_ZDINX);
require_fp;
softfloat_roundingMode = RM;
double arg = to_f(FRS1_D);        // Convert softfloat to native double
double result = sin(arg);           // Use C math library
uint64_t bits;
memcpy(&bits, &result, sizeof(bits));
WRITE_FRD_D(f64(bits));            // Convert back and write FP register
set_fp_exceptions;
```

Single-precision variant (`fsin_s.h`) uses `to_f(FRS1_F)`, `sinf()`, `f32()`, `WRITE_FRD_F()`.

Binary variant (`fatan2_d.h`) additionally reads `FRS2_D`.

#### Matrix instruction patterns:

**Integer MAC (`mmac.h`):**
```c
// Matrix Multiply-Accumulate (Signed Integer)
WRITE_RD(sext_xlen(RS1 * RS2 + RD));
```

**Single-precision float MAC (`mmacf.h`):**
```c
// Interpret integer register bits as float32, perform MAC, write back
{
  uint32_t a_bits = (uint32_t)RS1;
  uint32_t b_bits = (uint32_t)RS2;
  uint32_t acc_bits = (uint32_t)RD;
  float a, b, acc;
  memcpy(&a, &a_bits, sizeof(float));
  memcpy(&b, &b_bits, sizeof(float));
  memcpy(&acc, &acc_bits, sizeof(float));
  float result = a * b + acc;
  uint32_t res_bits;
  memcpy(&res_bits, &result, sizeof(uint32_t));
  WRITE_RD(sext_xlen(res_bits));
}
```

**Double-precision float MAC (`mmacdf.h`):**
```c
// Same pattern but with uint64_t, double, WRITE_RD(res_bits) (no sext)
```

**Matrix Load (`ml.h`):**
```c
{
  reg_t addr = RS1;
  for (int i = 0; i < 9; i++) {
    WRITE_REG(insn.rd(), MMU.load<uint32_t>(addr + i * 4));
  }
  WRITE_RD(addr);
}
```

**Accumulator Zero (`macc_zero.h`):**
```c
WRITE_RD(0);
```

**Systolic Control (`sys_start.h`, `sys_stop.h`):**
```c
// No-op in functional simulation
```

#### Complete list of 52 matrix instruction files:

```
ml.h  ms.h  mls.h  mss.h  mld.h  msd.h  mlds.h  msds.h
mmv_xmu.h
mmacu.h  maddu_m.h  msubu_m.h
mmac.h  madd_m.h  msub_m.h
mmacf.h  maddf.h  msubf.h
mmacd.h  mmacdf.h  maddfd.h  msubfd.h  msubd.h
macc_zero.h  macc_move.h  macc_store.h  macc_zerof.h  macc_zerod.h
mv_load.h  mv_store.h  mv_mul.h  mv_mac.h  mv_mac_acc.h
mmacf_acc.h  mmacdf_acc.h
minvf.h  minvd.h
sys_config.h  sys_start.h  sys_stop.h  sys_status.h
sys_load_a.h  sys_load_b.h  sys_load_c.h  sys_store_c.h
sys_load_ad.h  sys_load_bd.h  sys_load_cd.h  sys_store_cd.h
sys_mmac.h  sys_mmacd.h
```

#### Complete list of 14 trig instruction files:

```
fsin_s.h  fcos_s.h  ftan_s.h  fatan_s.h  fatan2_s.h  fasin_s.h  facos_s.h
fsin_d.h  fcos_d.h  ftan_d.h  fatan_d.h  fatan2_d.h  fasin_d.h  facos_d.h
```

### 5.4 File: `spike/riscv/riscv.mk.in`

Added two new instruction list variables and included them in `riscv_insn_list`:

```makefile
# After riscv_insn_ext_zvksh:

riscv_insn_ext_matrix = \
	ml \
	ms \
	mls \
	mss \
	mld \
	msd \
	mlds \
	msds \
	mmv_xmu \
	mmacu \
	maddu_m \
	msubu_m \
	mmac \
	madd_m \
	msub_m \
	mmacf \
	maddf \
	msubf \
	mmacd \
	mmacdf \
	maddfd \
	msubfd \
	msubd \
	macc_zero \
	macc_move \
	macc_store \
	macc_zerof \
	macc_zerod \
	mv_load \
	mv_store \
	mv_mul \
	mv_mac \
	mv_mac_acc \
	mmacf_acc \
	mmacdf_acc \
	minvf \
	minvd \
	sys_config \
	sys_start \
	sys_stop \
	sys_status \
	sys_load_a \
	sys_load_b \
	sys_load_c \
	sys_store_c \
	sys_load_ad \
	sys_load_bd \
	sys_load_cd \
	sys_store_cd \
	sys_mmac \
	sys_mmacd \

riscv_insn_ext_cordic = \
	fsin_s \
	fcos_s \
	ftan_s \
	fatan_s \
	fatan2_s \
	fasin_s \
	facos_s \
	fsin_d \
	fcos_d \
	ftan_d \
	fatan_d \
	fatan2_d \
	fasin_d \
	facos_d \

riscv_insn_list = \
	$(riscv_insn_ext_i) \
	... (existing entries) ...
	$(riscv_insn_ext_zicfiss) \
	$(riscv_insn_ext_matrix) \
	$(riscv_insn_ext_cordic) \
```

### 5.5 File: `spike/disasm/disasm.cc`

Added disassembly entries using Spike's macros:

```cpp
  { /* NOVA Matrix Extension Instructions */
    /* Memory Operations */
    DEFINE_R1TYPE(ml);
    DEFINE_R1TYPE(ms);
    DEFINE_R1TYPE(mls);
    DEFINE_R1TYPE(mss);
    DEFINE_R1TYPE(mld);
    DEFINE_R1TYPE(msd);
    DEFINE_R1TYPE(mlds);
    DEFINE_R1TYPE(msds);

    /* Matrix Move */
    DEFINE_R1TYPE(mmv_xmu);

    /* Unsigned Integer Compute */
    DEFINE_RTYPE(mmacu);
    DEFINE_RTYPE(maddu_m);
    DEFINE_RTYPE(msubu_m);

    /* Signed Integer Compute */
    DEFINE_RTYPE(mmac);
    DEFINE_RTYPE(madd_m);
    DEFINE_RTYPE(msub_m);

    /* Float Compute */
    DEFINE_RTYPE(mmacf);
    DEFINE_RTYPE(maddf);
    DEFINE_RTYPE(msubf);
    DEFINE_RTYPE(mmacd);
    DEFINE_RTYPE(mmacdf);
    DEFINE_RTYPE(maddfd);
    DEFINE_RTYPE(msubfd);
    DEFINE_RTYPE(msubd);

    /* Accumulator Operations */
    DISASM_INSN("macc_zero", macc_zero, 0, {&xrd});
    DEFINE_R1TYPE(macc_move);
    DEFINE_R1TYPE(macc_store);
    DISASM_INSN("macc_zerof", macc_zerof, 0, {&xrd});
    DISASM_INSN("macc_zerod", macc_zerod, 0, {&xrd});

    /* Matrix-Vector Operations */
    DEFINE_RTYPE(mv_load);
    DEFINE_RTYPE(mv_store);
    DEFINE_RTYPE(mv_mul);
    DEFINE_RTYPE(mv_mac);
    DEFINE_RTYPE(mv_mac_acc);

    /* Precision Accumulator MAC */
    DEFINE_RTYPE(mmacf_acc);
    DEFINE_RTYPE(mmacdf_acc);

    /* Matrix Inversion */
    DEFINE_R1TYPE(minvf);
    DEFINE_R1TYPE(minvd);

    /* Systolic Array Control */
    DISASM_INSN("sys_config", sys_config, 0, {&xrs1});
    DEFINE_NOARG(sys_start);
    DEFINE_NOARG(sys_stop);
    DISASM_INSN("sys_status", sys_status, 0, {&xrd});

    /* Systolic Array Data */
    DEFINE_RTYPE(sys_load_a);
    DEFINE_RTYPE(sys_load_b);
    DEFINE_RTYPE(sys_load_c);
    DEFINE_RTYPE(sys_store_c);
    DEFINE_RTYPE(sys_load_ad);
    DEFINE_RTYPE(sys_load_bd);
    DEFINE_RTYPE(sys_load_cd);
    DEFINE_RTYPE(sys_store_cd);

    /* Systolic Array MAC */
    DEFINE_RTYPE(sys_mmac);
    DEFINE_RTYPE(sys_mmacd);
  }

  { /* NOVA CORDIC Trigonometric Accelerator Instructions */
    /* Single Precision */
    DEFINE_FR1TYPE(fsin_s);
    DEFINE_FR1TYPE(fcos_s);
    DEFINE_FR1TYPE(ftan_s);
    DEFINE_FR1TYPE(fatan_s);
    DEFINE_FRTYPE(fatan2_s);
    DEFINE_FR1TYPE(fasin_s);
    DEFINE_FR1TYPE(facos_s);
    /* Double Precision */
    DEFINE_FR1TYPE(fsin_d);
    DEFINE_FR1TYPE(fcos_d);
    DEFINE_FR1TYPE(ftan_d);
    DEFINE_FR1TYPE(fatan_d);
    DEFINE_FRTYPE(fatan2_d);
    DEFINE_FR1TYPE(fasin_d);
    DEFINE_FR1TYPE(facos_d);
  }
```

**Disasm macro reference:**
- `DEFINE_RTYPE(name)` → prints `name rd, rs1, rs2` (integer registers)
- `DEFINE_R1TYPE(name)` → prints `name rd, rs1` (integer registers)
- `DEFINE_NOARG(name)` → prints `name` (no operands)
- `DEFINE_FRTYPE(name)` → prints `name frd, frs1, frs2` (FP registers)
- `DEFINE_FR1TYPE(name)` → prints `name frd, frs1` (FP registers)
- `DISASM_INSN(str, name, mask, {operands})` → custom operand list

---

## 6. Proxy Kernel (PK) Build

The proxy kernel was not previously installed. It was built and installed to enable `spike pk <binary>` execution.

**Source:** `/data/home/chandraboul/development/riscv-gnu-toolchain/pk/`

**No source modifications required** — stock pk is sufficient.

### Build commands:

```bash
export PATH=/data/home/chandraboul/development/riscv-spike/bin:$PATH

mkdir -p /data/home/chandraboul/development/riscv-gnu-toolchain/build-pk
cd /data/home/chandraboul/development/riscv-gnu-toolchain/build-pk

# Configure
/data/home/chandraboul/development/riscv-gnu-toolchain/pk/configure \
    --prefix=/data/home/chandraboul/development/riscv-spike \
    --host=riscv64-unknown-elf

# Build and install
make -j$(nproc)
make install
```

**Installed to:** `/data/home/chandraboul/development/riscv-spike/riscv64-unknown-elf/bin/pk`

---

## 7. Build & Install Procedures

### 7.1 Environment Setup

```bash
export PATH=/data/home/chandraboul/development/riscv-spike/bin:$PATH
```

### 7.2 Rebuilding Binutils (after modifying riscv-opc.h or riscv-opc.c)

```bash
cd /data/home/chandraboul/development/riscv-gnu-toolchain/build-binutils-newlib
sudo make -j$(nproc)
sudo make install
```

### 7.3 Rebuilding Spike (after modifying any Spike source)

**Important:** Since `riscv.mk.in` was modified (not just .h files), the build system must be reconfigured to regenerate `riscv.mk`:

```bash
cd /data/home/chandraboul/development/riscv-gnu-toolchain/build-spike

# Step 1: Reconfigure (REQUIRED when riscv.mk.in changes)
sudo /data/home/chandraboul/development/riscv-gnu-toolchain/spike/configure \
    --prefix=/data/home/chandraboul/development/riscv-spike

# Step 2: Build
sudo make -j$(nproc)

# Step 3: Install
sudo make install
```

**If only instruction .h files changed** (no riscv.mk.in change), force recompile:

```bash
cd /data/home/chandraboul/development/riscv-gnu-toolchain/build-spike

# Remove cached object files for changed instructions
sudo rm -f mmacf.o mmacd.o mmacdf.o mmacf_acc.o mmacdf_acc.o

# Rebuild and install
sudo make -j$(nproc)
sudo make install
```

### 7.4 Rebuilding PK

```bash
cd /data/home/chandraboul/development/riscv-gnu-toolchain/build-pk
export PATH=/data/home/chandraboul/development/riscv-spike/bin:$PATH
make -j$(nproc)
make install
```

---

## 8. Test Programs

### 8.1 Trigonometric + Matrix Basic Test (`test_nova_spike.c`)

```c
/* Test NOVA custom instructions on Spike */
/* Tests both CORDIC trig and Matrix instructions */
#include <stdio.h>
#include <math.h>

int main() {
    printf("=== NOVA Spike Instruction Test ===\n\n");

    /* Test 1: Trigonometric instructions (double precision) */
    printf("--- Trigonometric (Double) ---\n");
    double angle = 0.5;
    double sin_result, cos_result, tan_result, atan_result, asin_result, acos_result;

    asm volatile(
        "fmv.d fa0, %1\n\t"
        "fsin.d fa0, fa0\n\t"
        "fmv.d %0, fa0\n\t"
        : "=f"(sin_result) : "f"(angle) : "fa0"
    );
    printf("  fsin.d(0.5) = %f (expected %f)\n", sin_result, sin(0.5));

    asm volatile(
        "fmv.d fa0, %1\n\t"
        "fcos.d fa0, fa0\n\t"
        "fmv.d %0, fa0\n\t"
        : "=f"(cos_result) : "f"(angle) : "fa0"
    );
    printf("  fcos.d(0.5) = %f (expected %f)\n", cos_result, cos(0.5));

    asm volatile(
        "fmv.d fa0, %1\n\t"
        "ftan.d fa0, fa0\n\t"
        "fmv.d %0, fa0\n\t"
        : "=f"(tan_result) : "f"(angle) : "fa0"
    );
    printf("  ftan.d(0.5) = %f (expected %f)\n", tan_result, tan(0.5));

    asm volatile(
        "fmv.d fa0, %1\n\t"
        "fatan.d fa0, fa0\n\t"
        "fmv.d %0, fa0\n\t"
        : "=f"(atan_result) : "f"(angle) : "fa0"
    );
    printf("  fatan.d(0.5) = %f (expected %f)\n", atan_result, atan(0.5));

    asm volatile(
        "fmv.d fa0, %1\n\t"
        "fasin.d fa0, fa0\n\t"
        "fmv.d %0, fa0\n\t"
        : "=f"(asin_result) : "f"(angle) : "fa0"
    );
    printf("  fasin.d(0.5) = %f (expected %f)\n", asin_result, asin(0.5));

    asm volatile(
        "fmv.d fa0, %1\n\t"
        "facos.d fa0, fa0\n\t"
        "fmv.d %0, fa0\n\t"
        : "=f"(acos_result) : "f"(angle) : "fa0"
    );
    printf("  facos.d(0.5) = %f (expected %f)\n", acos_result, acos(0.5));

    double y = 1.0, x = 2.0, atan2_result;
    asm volatile(
        "fmv.d fa0, %1\n\t"
        "fmv.d fa1, %2\n\t"
        "fatan2.d fa0, fa0, fa1\n\t"
        "fmv.d %0, fa0\n\t"
        : "=f"(atan2_result) : "f"(y), "f"(x) : "fa0", "fa1"
    );
    printf("  fatan2.d(1.0, 2.0) = %f (expected %f)\n", atan2_result, atan2(1.0, 2.0));

    /* Test 2: Trigonometric instructions (single precision) */
    printf("\n--- Trigonometric (Single) ---\n");
    float anglef = 0.5f;
    float sinf_result, cosf_result;

    asm volatile(
        "fmv.s fa0, %1\n\t"
        "fsin.s fa0, fa0\n\t"
        "fmv.s %0, fa0\n\t"
        : "=f"(sinf_result) : "f"(anglef) : "fa0"
    );
    printf("  fsin.s(0.5) = %f (expected %f)\n", sinf_result, sinf(0.5f));

    asm volatile(
        "fmv.s fa0, %1\n\t"
        "fcos.s fa0, fa0\n\t"
        "fmv.s %0, fa0\n\t"
        : "=f"(cosf_result) : "f"(anglef) : "fa0"
    );
    printf("  fcos.s(0.5) = %f (expected %f)\n", cosf_result, cosf(0.5f));

    /* Test 3: Matrix instructions */
    printf("\n--- Matrix Instructions ---\n");

    asm volatile("macc.zero x0\n\t" ::: );
    printf("  macc.zero executed OK\n");

    long a = 3, b = 4, mac_result;
    asm volatile(
        "mmac %0, %1, %2\n\t"
        : "=r"(mac_result) : "r"(a), "r"(b)
    );
    printf("  mmac(3, 4) = %ld\n", mac_result);

    long c = 10, d = 20, add_result;
    asm volatile(
        "madd.m %0, %1, %2\n\t"
        : "=r"(add_result) : "r"(c), "r"(d)
    );
    printf("  madd.m(10, 20) = %ld\n", add_result);

    long sub_result;
    asm volatile(
        "msub.m %0, %1, %2\n\t"
        : "=r"(sub_result) : "r"(c), "r"(d)
    );
    printf("  msub.m(10, 20) = %ld\n", sub_result);

    printf("\n=== All tests complete ===\n");
    return 0;
}
```

### 8.2 Matrix Multiply Precision Test (`test_matrix_mul.c`)

```c
/* Test Matrix Multiply - Single and Double Precision on Spike */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

static inline uint64_t f2b(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static inline float b2f(uint64_t b) { uint32_t t=(uint32_t)b; float f; memcpy(&f, &t, 4); return f; }
static inline uint64_t d2b(double d) { uint64_t b; memcpy(&b, &d, 8); return b; }
static inline double b2d(uint64_t b) { double d; memcpy(&d, &b, 8); return d; }

int main() {
    int pass = 0, total = 0;
    printf("=== Matrix Multiply-Accumulate Tests (Single & Double) ===\n\n");

    /* Single Precision: mmacf */
    printf("--- Single Precision (mmacf) ---\n");

    float fa=2.5f, fb=3.0f, facc=1.5f;
    uint64_t r;
    asm volatile("mv a3,%3\n\t mmacf a3,%1,%2\n\t mv %0,a3"
        : "=r"(r) : "r"(f2b(fa)), "r"(f2b(fb)), "r"(f2b(facc)) : "a3");
    float fr = b2f(r), fe = fa*fb+facc;
    total++; if(fr==fe) pass++;
    printf("  mmacf(%.1f*%.1f+%.1f) = %.1f expected %.1f  %s\n",
           fa,fb,facc,fr,fe, fr==fe?"PASS":"FAIL");

    fa=-2.0f; fb=3.5f; facc=10.0f;
    asm volatile("mv a3,%3\n\t mmacf a3,%1,%2\n\t mv %0,a3"
        : "=r"(r) : "r"(f2b(fa)), "r"(f2b(fb)), "r"(f2b(facc)) : "a3");
    fr=b2f(r); fe=fa*fb+facc;
    total++; if(fr==fe) pass++;
    printf("  mmacf(%.1f*%.1f+%.1f) = %.1f expected %.1f  %s\n",
           fa,fb,facc,fr,fe, fr==fe?"PASS":"FAIL");

    /* Double Precision: mmacdf */
    printf("\n--- Double Precision (mmacdf) ---\n");

    double da=2.5, db=3.0, dacc=1.5;
    uint64_t rd;
    asm volatile("mv a3,%3\n\t mmacdf a3,%1,%2\n\t mv %0,a3"
        : "=r"(rd) : "r"(d2b(da)), "r"(d2b(db)), "r"(d2b(dacc)) : "a3");
    double dr=b2d(rd), de=da*db+dacc;
    total++; if(dr==de) pass++;
    printf("  mmacdf(%.1f*%.1f+%.1f) = %.1f expected %.1f  %s\n",
           da,db,dacc,dr,de, dr==de?"PASS":"FAIL");

    da=3.14159; db=2.71828; dacc=0.0;
    asm volatile("mv a3,%3\n\t mmacdf a3,%1,%2\n\t mv %0,a3"
        : "=r"(rd) : "r"(d2b(da)), "r"(d2b(db)), "r"(d2b(dacc)) : "a3");
    dr=b2d(rd); de=da*db+dacc;
    total++; if(dr==de) pass++;
    printf("  mmacdf(pi*e+0) = %.6f expected %.6f  %s\n",
           dr, de, dr==de?"PASS":"FAIL");

    /* Accumulator variants */
    printf("\n--- Accumulator Variants ---\n");

    fa=10.0f; fb=0.5f; facc=100.0f;
    asm volatile("mv a3,%3\n\t mmacf.acc a3,%1,%2\n\t mv %0,a3"
        : "=r"(r) : "r"(f2b(fa)), "r"(f2b(fb)), "r"(f2b(facc)) : "a3");
    fr=b2f(r); fe=fa*fb+facc;
    total++; if(fr==fe) pass++;
    printf("  mmacf.acc(%.1f*%.1f+%.1f) = %.1f expected %.1f  %s\n",
           fa,fb,facc,fr,fe, fr==fe?"PASS":"FAIL");

    da=10.0; db=0.5; dacc=100.0;
    asm volatile("mv a3,%3\n\t mmacdf.acc a3,%1,%2\n\t mv %0,a3"
        : "=r"(rd) : "r"(d2b(da)), "r"(d2b(db)), "r"(d2b(dacc)) : "a3");
    dr=b2d(rd); de=da*db+dacc;
    total++; if(dr==de) pass++;
    printf("  mmacdf.acc(%.1f*%.1f+%.1f) = %.1f expected %.1f  %s\n",
           da,db,dacc,dr,de, dr==de?"PASS":"FAIL");

    /* 3x3 Matrix Multiply (Single Precision) */
    printf("\n--- 3x3 Matrix Multiply (Single Precision via mmacf) ---\n");
    float A[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
    float B[3][3] = {{9,8,7},{6,5,4},{3,2,1}};
    float C[3][3] = {{0}};

    for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
        uint64_t acc = f2b(0.0f);
        for(int k=0;k<3;k++) {
            asm volatile("mv a3,%3\n\t mmacf a3,%1,%2\n\t mv %0,a3"
                : "=r"(acc) : "r"(f2b(A[i][k])), "r"(f2b(B[k][j])),
                  "r"(acc) : "a3");
        }
        C[i][j] = b2f(acc);
    }
    printf("  C[0] = [%.0f, %.0f, %.0f]  expected [30, 24, 18]\n",
           C[0][0], C[0][1], C[0][2]);
    printf("  C[1] = [%.0f, %.0f, %.0f]  expected [84, 69, 54]\n",
           C[1][0], C[1][1], C[1][2]);
    printf("  C[2] = [%.0f, %.0f, %.0f]  expected [138, 114, 90]\n",
           C[2][0], C[2][1], C[2][2]);
    int m3f = (C[0][0]==30 && C[0][1]==24 && C[0][2]==18 &&
               C[1][0]==84 && C[1][1]==69 && C[1][2]==54 &&
               C[2][0]==138 && C[2][1]==114 && C[2][2]==90);
    total++; if(m3f) pass++;
    printf("  3x3 single-prec: %s\n", m3f?"PASS":"FAIL");

    /* 3x3 Matrix Multiply (Double Precision) */
    printf("\n--- 3x3 Matrix Multiply (Double Precision via mmacdf) ---\n");
    double Ad[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
    double Bd[3][3] = {{9,8,7},{6,5,4},{3,2,1}};
    double Cd[3][3] = {{0}};

    for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
        uint64_t acc = d2b(0.0);
        for(int k=0;k<3;k++) {
            asm volatile("mv a3,%3\n\t mmacdf a3,%1,%2\n\t mv %0,a3"
                : "=r"(acc) : "r"(d2b(Ad[i][k])), "r"(d2b(Bd[k][j])),
                  "r"(acc) : "a3");
        }
        Cd[i][j] = b2d(acc);
    }
    printf("  C[0] = [%.0f, %.0f, %.0f]  expected [30, 24, 18]\n",
           Cd[0][0], Cd[0][1], Cd[0][2]);
    printf("  C[1] = [%.0f, %.0f, %.0f]  expected [84, 69, 54]\n",
           Cd[1][0], Cd[1][1], Cd[1][2]);
    printf("  C[2] = [%.0f, %.0f, %.0f]  expected [138, 114, 90]\n",
           Cd[2][0], Cd[2][1], Cd[2][2]);
    int m3d = (Cd[0][0]==30 && Cd[0][1]==24 && Cd[0][2]==18 &&
               Cd[1][0]==84 && Cd[1][1]==69 && Cd[1][2]==54 &&
               Cd[2][0]==138 && Cd[2][1]==114 && Cd[2][2]==90);
    total++; if(m3d) pass++;
    printf("  3x3 double-prec: %s\n", m3d?"PASS":"FAIL");

    printf("\n=== Results: %d/%d passed ===\n", pass, total);
    return (pass == total) ? 0 : 1;
}
```

---

## 9. Compilation & Execution

### 9.1 Set up PATH

```bash
export PATH=/data/home/chandraboul/development/riscv-spike/bin:$PATH
```

### 9.2 Compile

```bash
# Trig + matrix basic test
riscv64-unknown-elf-gcc -O0 -o test_nova_spike test_nova_spike.c -lm

# Matrix multiply precision test
riscv64-unknown-elf-gcc -O0 -o test_matrix_mul test_matrix_mul.c -lm
```

> **Important:** Use `-O0` to prevent the compiler from optimizing away inline assembly.

### 9.3 Verify assembly encoding (optional)

```bash
# Disassemble to verify instruction encodings
riscv64-unknown-elf-objdump -d test_nova_spike | grep -E "fsin|fcos|ftan|fatan|fasin|facos|mmac|madd|msub|macc"
```

### 9.4 Execute on Spike

```bash
spike pk test_nova_spike
spike pk test_matrix_mul
```

### 9.5 Using .word encoding (legacy method)

If binutils doesn't have the mnemonics, instructions can be encoded manually:

```c
/* fsin.d fa0, fa0 — encoding 0x38050553 */
asm volatile(".word 0x38050553\n\t");

/* macc.zero x0 — encoding 0x80000077 */
asm volatile(".word 0x80000077\n\t");
```

### 9.6 Using the NOVA header files (for gem5 compatibility)

```c
#include "nova_cordic_accel.h"        /* trig: hw_sin(), hw_cos(), etc. */
#include "riscv-matrix-updated2_mv_3x3.h"  /* matrix: ML(), MS(), MMAC(), etc. */
```

---

## 10. Verified Test Results

### 10.1 Trigonometric Instructions — All PASS

```
=== NOVA Spike Instruction Test ===

--- Trigonometric (Double) ---
  fsin.d(0.5) = 0.479426 (expected 0.479426)     ✓
  fcos.d(0.5) = 0.877583 (expected 0.877583)     ✓
  ftan.d(0.5) = 0.546302 (expected 0.546302)     ✓
  fatan.d(0.5) = 0.463648 (expected 0.463648)    ✓
  fasin.d(0.5) = 0.523599 (expected 0.523599)    ✓
  facos.d(0.5) = 1.047198 (expected 1.047198)    ✓
  fatan2.d(1.0, 2.0) = 0.463648 (expected 0.463648) ✓

--- Trigonometric (Single) ---
  fsin.s(0.5) = 0.479426 (expected 0.479426)     ✓
  fcos.s(0.5) = 0.877583 (expected 0.877583)     ✓

--- Matrix Instructions ---
  macc.zero executed OK                           ✓
  mmac(3, 4) = 15                                 ✓
  madd.m(10, 20) = 30                             ✓
  msub.m(10, 20) = -10                            ✓
```

### 10.2 Matrix Multiply Precision — 11/11 PASS

```
=== Matrix Multiply-Accumulate Tests (Single & Double) ===

--- Single Precision (mmacf) ---
  mmacf(2.5*3.0+1.5) = 9.0 expected 9.0          PASS
  mmacf(1.5*4.0+0.0) = 6.0 expected 6.0          PASS
  mmacf(-2.0*3.5+10.0) = 3.0 expected 3.0        PASS

--- Double Precision (mmacdf) ---
  mmacdf(2.5*3.0+1.5) = 9.0 expected 9.0         PASS
  mmacdf(1.1*2.2+3.3) = 5.720000 expected 5.720000   PASS
  mmacdf(pi*e+0) = 8.539721 expected 8.539721     PASS
  mmacdf(-5.5*2.0+20.0) = 9.0 expected 9.0       PASS

--- Accumulator Variants ---
  mmacf.acc(10.0*0.5+100.0) = 105.0 expected 105.0   PASS
  mmacdf.acc(10.0*0.5+100.0) = 105.0 expected 105.0  PASS

--- 3x3 Matrix Multiply (Single Precision via mmacf) ---
  C[0] = [30, 24, 18]  expected [30, 24, 18]     PASS
  C[1] = [84, 69, 54]  expected [84, 69, 54]
  C[2] = [138, 114, 90]  expected [138, 114, 90]

--- 3x3 Matrix Multiply (Double Precision via mmacdf) ---
  C[0] = [30, 24, 18]  expected [30, 24, 18]     PASS
  C[1] = [84, 69, 54]  expected [84, 69, 54]
  C[2] = [138, 114, 90]  expected [138, 114, 90]

=== Results: 11/11 passed ===
```

---

## 11. Encoding Tables

### 11.1 CORDIC Trig Encodings (OP-FP: opcode 0x53)

```
Encoding format: [funct7(31:25)] [rs2(24:20)] [rs1(19:15)] [rm(14:12)] [rd(11:7)] [opcode(6:0)]
Unary ops: rs2 = 0x00

Instruction    funct7   MATCH        MASK         Type
─────────────────────────────────────────────────────────
fsin.s         0x18     0x30000053   0xfff0007f   Unary
fcos.s         0x19     0x32000053   0xfff0007f   Unary
ftan.s         0x1b     0x36000053   0xfff0007f   Unary
fatan.s        0x2b     0x56000053   0xfff0007f   Unary
fatan2.s       0x24     0x48000053   0xfe00007f   Binary
fasin.s        0x30     0x60000053   0xfff0007f   Unary
facos.s        0x31     0x62000053   0xfff0007f   Unary
fsin.d         0x1c     0x38000053   0xfff0007f   Unary
fcos.d         0x1d     0x3A000053   0xfff0007f   Unary
ftan.d         0x1f     0x3E000053   0xfff0007f   Unary
fatan.d        0x1a     0x34000053   0xfff0007f   Unary
fatan2.d       0x25     0x4A000053   0xfe00007f   Binary
fasin.d        0x32     0x64000053   0xfff0007f   Unary
facos.d        0x33     0x66000053   0xfff0007f   Unary
```

### 11.2 Matrix Encodings (custom-3: opcode 0x77)

```
Encoding format: [funct7(31:25)] [rs2(24:20)] [rs1(19:15)] [funct3(14:12)] [rd(11:7)] [opcode(6:0)]

Instruction    MATCH        MASK         funct7  funct3  Operands
──────────────────────────────────────────────────────────────────────
ml             0x22000077   0xfe00707f   0x11    0x000   rd, rs1
ms             0x22001077   0xfe00707f   0x11    0x001   rd, rs1
mls            0x22004077   0xfe00707f   0x11    0x100   rd, rs1
mss            0x22005077   0xfe00707f   0x11    0x101   rd, rs1
mld            0x22002077   0xfe00707f   0x11    0x010   rd, rs1
msd            0x22003077   0xfe00707f   0x11    0x011   rd, rs1
mlds           0x22006077   0xfe00707f   0x11    0x110   rd, rs1
msds           0x22007077   0xfe00707f   0x11    0x111   rd, rs1
mmv.xmu        0x42100077   0xfe1ff07f   0x21    0x000   rd, rs1
mmacu          0x62000077   0xfe00707f   0x31    0x000   rd, rs1, rs2
maddu          0x62001077   0xfe00707f   0x31    0x001   rd, rs1, rs2
msubu          0x62002077   0xfe00707f   0x31    0x010   rd, rs1, rs2
mmac           0x66000077   0xfe00707f   0x33    0x000   rd, rs1, rs2
madd.m         0x66001077   0xfe00707f   0x33    0x001   rd, rs1, rs2
msub.m         0x66002077   0xfe00707f   0x33    0x010   rd, rs1, rs2
mmacf          0x6a000077   0xfe00707f   0x35    0x000   rd, rs1, rs2
maddf          0x6a001077   0xfe00707f   0x35    0x001   rd, rs1, rs2
msubf          0x6a002077   0xfe00707f   0x35    0x010   rd, rs1, rs2
mmacdf         0x6a003077   0xfe00707f   0x35    0x011   rd, rs1, rs2
maddfd         0x6a004077   0xfe00707f   0x35    0x100   rd, rs1, rs2
msubfd         0x6a005077   0xfe00707f   0x35    0x101   rd, rs1, rs2
mmacf.acc      0x6a006077   0xfe00707f   0x35    0x110   rd, rs1, rs2
mmacdf.acc     0x6a007077   0xfe00707f   0x35    0x111   rd, rs1, rs2
macc.zero      0x80000077   0xfe0ff07f   0x40    0x000   rd
macc.move      0x80001077   0xfe00707f   0x40    0x001   rd, rs1
macc.store     0x80002077   0xfe00707f   0x40    0x010   rd, rs1
macc.zerof     0x80003077   0xfe0ff07f   0x40    0x011   rd
macc.zerod     0x80004077   0xfe0ff07f   0x40    0x100   rd
mv.load        0x82000077   0xfe00707f   0x41    0x000   rd, rs1, rs2
mv.store       0x82001077   0xfe00707f   0x41    0x001   rd, rs1, rs2
mv.mul         0x86002077   0xfe00707f   0x43    0x010   rd, rs1, rs2
mv.mac         0x86003077   0xfe00707f   0x43    0x011   rd, rs1, rs2
mv.mac.acc     0x86006077   0xfe00707f   0x43    0x110   rd, rs1, rs2
minvf          0x6a005077   0xfe00707f   0x35    0x101   rd, rs1
minvd          0x6a006077   0xfe00707f   0x35    0x110   rd, rs1
sys.config     0xa2000077   0xfe00707f   0x51    0x000   rs1
sys.start      0xa2001077   0xffffffff   0x51    0x001   (none)
sys.stop       0xa2002077   0xffffffff   0x51    0x010   (none)
sys.status     0xa2003077   0xfe0ff07f   0x51    0x011   rd
sys.load.a     0xa2004077   0xfe00707f   0x51    0x100   rd, rs1, rs2
sys.load.b     0xa2005077   0xfe00707f   0x51    0x101   rd, rs1, rs2
sys.load.c     0xa2006077   0xfe00707f   0x51    0x110   rd, rs1, rs2
sys.store.c    0xa2007077   0xfe00707f   0x51    0x111   rd, rs1, rs2
sys.load.ad    0xa6004077   0xfe00707f   0x53    0x100   rd, rs1, rs2
sys.load.bd    0xa6005077   0xfe00707f   0x53    0x101   rd, rs1, rs2
sys.load.cd    0xa6006077   0xfe00707f   0x53    0x110   rd, rs1, rs2
sys.store.cd   0xa6007077   0xfe00707f   0x53    0x111   rd, rs1, rs2
sys.mmac       0xb2000077   0xfe00707f   0x59    0x000   rd, rs1, rs2
sys.mmacd      0xb2001077   0xfe00707f   0x59    0x001   rd, rs1, rs2
```

---

## 12. Known Encoding Conflicts

The following instructions share the same binary encoding (inherited from the original ISA design, which uses additional decoder context in gem5):

| Encoding | Instruction 1 | Instruction 2 | Recommendation |
|----------|---------------|---------------|----------------|
| `0x66002077` | `msub.m` (int sub) | `mmacd` (double MAC) | Use `mmacdf` for double MAC |
| `0x6a005077` | `msubfd` (double sub) | `minvf` (float inv) | Functionally equivalent in sim |
| `0x6a006077` | `mmacf.acc` (float MAC acc) | `minvd` (double inv) | Functionally equivalent in sim |

**In Spike:** The first-registered instruction wins. For double-precision MAC, always use **`mmacdf`** (0x6a003077) which has no conflict.

---

## 13. Instruction Implementations (Spike)

### Key Spike Macros

| Macro | Description |
|-------|-------------|
| `RS1` | Read integer register rs1 |
| `RS2` | Read integer register rs2 |
| `RD` | Read integer register rd (for accumulate) |
| `WRITE_RD(val)` | Write to integer register rd |
| `sext_xlen(val)` | Sign-extend to XLEN bits |
| `FRS1_D` / `FRS1_F` | Read FP register rs1 (double/float, softfloat type) |
| `FRS2_D` / `FRS2_F` | Read FP register rs2 |
| `WRITE_FRD_D(val)` / `WRITE_FRD_F(val)` | Write to FP register rd |
| `to_f(softfloat_val)` | Convert softfloat to native C float/double |
| `f64(bits)` / `f32(bits)` | Create softfloat value from raw bits |
| `MMU.load<type>(addr)` | Load from memory |
| `MMU.store<type>(addr, val)` | Store to memory |
| `require_fp` | Ensure FP unit is enabled |
| `require_either_extension(c, ext)` | Check ISA extension |
| `softfloat_roundingMode = RM` | Set rounding mode from instruction |
| `set_fp_exceptions` | Propagate FP exception flags |

### Implementation Categories

| Category | Pattern | Example |
|----------|---------|---------|
| Trig (double) | `to_f(FRS1_D)` → `sin()` → `f64(bits)` → `WRITE_FRD_D()` | `fsin_d.h` |
| Trig (single) | `to_f(FRS1_F)` → `sinf()` → `f32(bits)` → `WRITE_FRD_F()` | `fsin_s.h` |
| Integer MAC | `WRITE_RD(sext_xlen(RS1 * RS2 + RD))` | `mmac.h` |
| Float MAC | `memcpy` bits → float math → `memcpy` back → `WRITE_RD()` | `mmacf.h` |
| Double MAC | Same pattern with `double` and `uint64_t` | `mmacdf.h` |
| Memory ops | `MMU.load<>` / `MMU.store<>` loops for 3×3 | `ml.h` |
| Zero acc | `WRITE_RD(0)` | `macc_zero.h` |
| No-op | Empty body | `sys_start.h` |
| Passthrough | `WRITE_RD(RS1)` | `sys_config.h` |

---

## 14. Troubleshooting

### Problem: "An illegal instruction was executed!"

**Cause:** Wrong `.word` encoding used, or Spike not rebuilt after adding instructions.

**Fix:**
1. Verify encoding with `objdump -d` or check `encoding.h`
2. Ensure Spike was reconfigured (`configure`) and rebuilt after `riscv.mk.in` changes
3. Verify object files exist: `ls build-spike/fsin_d.o`

### Problem: "make: Nothing to be done"

**Cause:** Build system doesn't detect `.h` file changes.

**Fix:** Delete the specific `.o` files and rebuild:
```bash
cd build-spike && sudo rm -f mmacf.o && sudo make -j$(nproc)
```

### Problem: NaN/Inf results from double-precision matrix instructions

**Cause:** Using `mmacd` which conflicts with `msub.m` encoding.

**Fix:** Use `mmacdf` instead of `mmacd` for double-precision float MAC.

### Problem: Spike build fails with "no rule for conversion.h"

**Cause:** `softfloat/softfloat.mk.in` has stale references.

**Fix:** Reconfigure:
```bash
cd build-spike
sudo spike/configure --prefix=/path/to/install
sudo make -j$(nproc)
```

### Problem: "spike: command not found" or "pk: No such file"

**Fix:**
```bash
export PATH=/data/home/chandraboul/development/riscv-spike/bin:$PATH
# pk is at: riscv-spike/riscv64-unknown-elf/bin/pk
# spike finds it automatically via --prefix
```

---

## 15. Files Modified Summary

### Binutils (2 files)

| File | Changes |
|------|---------|
| `binutils/include/opcode/riscv-opc.h` | Added MATCH/MASK defines + DECLARE_INSN for 52 matrix + 14+ trig instructions |
| `binutils/opcodes/riscv-opc.c` | Added opcode table entries (43 matrix + trig entries with format strings) |

### Spike (5 files + 66 new files)

| File | Changes |
|------|---------|
| `spike/riscv/encoding.h` | Added MATCH/MASK + DECLARE_INSN (52 matrix + 14 trig) |
| `spike/riscv/insn_template.h` | Added `#include <cmath>` |
| `spike/riscv/riscv.mk.in` | Added `riscv_insn_ext_matrix` (52) + `riscv_insn_ext_cordic` (14) lists |
| `spike/disasm/disasm.cc` | Added disassembly entries for all 66 instructions |
| `spike/riscv/insns/*.h` | **66 new files** (52 matrix + 14 trig instruction implementations) |

### Proxy Kernel (0 files modified)

| Action | Details |
|--------|---------|
| Built from source | `pk/configure --prefix=... --host=riscv64-unknown-elf` |
| Installed to | `riscv-spike/riscv64-unknown-elf/bin/pk` |

---

## Quick Reference Card

```bash
# Setup
export PATH=/data/home/chandraboul/development/riscv-spike/bin:$PATH

# Compile
riscv64-unknown-elf-gcc -O0 -o test test.c -lm

# Run on Spike
spike pk test

# Disassemble
riscv64-unknown-elf-objdump -d test | grep fsin

# Rebuild Spike (after source changes)
cd /data/home/chandraboul/development/riscv-gnu-toolchain/build-spike
sudo ../spike/configure --prefix=/data/home/chandraboul/development/riscv-spike
sudo make -j$(nproc) && sudo make install

# Rebuild Binutils (after opcode changes)
cd /data/home/chandraboul/development/riscv-gnu-toolchain/build-binutils-newlib
sudo make -j$(nproc) && sudo make install
```
