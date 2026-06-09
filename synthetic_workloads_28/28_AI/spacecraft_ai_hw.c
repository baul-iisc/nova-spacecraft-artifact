/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright 2025 the authors of IEEE TC paper TC-2025-09-0830
 * ("Characterizing and Accelerating Spacecraft Onboard Workloads
 *  on RISC-V Platform").  Licensed under the Apache License,
 * Version 2.0; see LICENSE at the root of this repository.
 *
 * Synthetic workload derived for academic research from
 * production spacecraft onboard flight software characteristics.
 */
/*
 * SYNTHETIC BENCHMARK WORKLOAD
 *
 * This synthetic workload is derived for academic research from
 * production spacecraft onboard flight software characteristics.
 * The algorithmic structure and computational characteristics (BCE —
 * Basic Computational Elements) are representative of real spacecraft
 * onboard flight software, but all mission-specific details, proprietary
 * algorithms, and program identifiers have been removed or replaced
 * with sanitized, non-mission-specific synthetic equivalents.
 *
 * Part of the Spacecraft Onboard Computing Benchmark Suite for
 * RISC-V accelerator architecture evaluation.
 */
/* ═══════════════════════════════════════════════════════════════
 * SYSTOLIC ARRAY VERSION — SYS_MMACD (C += A × B, no transpose)
 * Systolic-matrix variant produced from the MMACDF source file using convert_to_systolic.py
 * ═══════════════════════════════════════════════════════════════ */

/*
 * spacecraft_ai_hw.c -- Unity build with CORDIC trig + AME matrix
 *
 * Combined variant:
 * - sinf/cosf redirected to CORDIC hw_sin/hw_cos
 * - mat_mul (15x15 float) replaced with AME 3x3 tiled SYS_MMACD
 *
 * Built as a single compilation unit that #includes all original sources.
 */

/* Pull in system headers via the project header first */
#include "spacecraft_ai.h"
#include "../flight_compliance.h"


/* ═══════════════════════════════════════════════════════════════════
 * NOVA CORDIC Trigonometric Accelerator — Inline Definitions
 * Uses assembler mnemonics (requires NOVA-modified riscv-gnu-toolchain)
 * ═══════════════════════════════════════════════════════════════════ */
static inline double hw_sin(double x) {
    double r; asm volatile("fsin.d %0, %1" : "=f"(r) : "f"(x)); return r;
}
static inline double hw_cos(double x) {
    double r; asm volatile("fcos.d %0, %1" : "=f"(r) : "f"(x)); return r;
}
static inline double hw_tan(double x) {
    double r; asm volatile("ftan.d %0, %1" : "=f"(r) : "f"(x)); return r;
}
static inline double hw_atan(double x) {
    double r; asm volatile("fatan.d %0, %1" : "=f"(r) : "f"(x)); return r;
}
static inline double hw_atan2(double y, double x) {
    double r; asm volatile("fatan2.d %0, %1, %2" : "=f"(r) : "f"(y), "f"(x)); return r;
}
static inline double hw_asin(double x) {
    double r; asm volatile("fasin.d %0, %1" : "=f"(r) : "f"(x)); return r;
}
static inline double hw_acos(double x) {
    double r; asm volatile("facos.d %0, %1" : "=f"(r) : "f"(x)); return r;
}
static inline double hw_sqrt(double x) {
    double r; asm volatile("fsqrt.d %0, %1" : "=f"(r) : "f"(x)); return r;
}
/* AME 3x3 tile matrix accelerator */

/* ═══════════════════════════════════════════════════════════════════
 * NOVA 3×3 Matrix Extension — Inline Macro Definitions
 * Uses assembler mnemonics (requires NOVA-modified riscv-gnu-toolchain)
 *
 * Matrix registers are encoded via integer register names (x0–x31).
 * The hardware decodes opcode 0x77 and routes to the matrix register
 * file, so "mlds x0, t1, t0" loads into matrix register M0.
 * ═══════════════════════════════════════════════════════════════════ */

/* Matrix register aliases (use integer register names for encoding) */
#define M0  "x0"
#define M1  "x1"
#define M2  "x2"
#define M3  "x3"

/* Integer register aliases for address / stride operands */
#define T0  "t0"
#define T1  "t1"
#define T2  "t2"
#define T3  "t3"

/* Matrix instruction macros (assembler mnemonics) */
#define MLDS(md, rs1, rs2)      "mlds "      md ", " rs1 ", " rs2 "\n\t"
#define MSDS(md, rs1, rs2)      "msds "      md ", " rs1 ", " rs2 "\n\t"
#define MZERO(md)               "mmv.xmu "   md ", x0\n\t"
#define SYS_MMACD(md, rs1, rs2) "sys.mmacd " md ", " rs1 ", " rs2 "\n\t"
#define MMACDF(md, rs1, rs2)    "mmacdf "    md ", " rs1 ", " rs2 "\n\t"
#define MINVD(md, ms1)          "minvd "     md ", " ms1 "\n\t"
#define MADDFD(md, ms1, ms2)    "maddfd "    md ", " ms1 ", " ms2 "\n\t"
#define MSUBFD(md, ms1, ms2)    "msubfd "    md ", " ms1 ", " ms2 "\n\t"
/* Redirect float trig to CORDIC (double precision, cast at boundary) */
#undef sinf
#undef cosf
#define sinf(x) (float)hw_sin((double)(x))
#define cosf(x) (float)hw_cos((double)(x))

/* Hide the original mat_mul definition during math.c inclusion */
#define mat_mul mat_mul_sw_hidden
#include "spacecraft_ai_math.c"
#undef mat_mul

/* AME 3x3 tiled mat_mul for SC_EKF_STATE_DIM = 15 (5 tiles of 3) */
#define N SC_EKF_STATE_DIM
void mat_mul(const float A[N][N],
 const float B[N][N],
 float C[N][N]) {
 /* ── Optimized: pre-transpose B as doubles → direct MLDS for BT ── */
 const int n = N;
 const int P = 3;
 const long s24 = 24;
 for (int i = 0; i < n; i++)
 for (int j = 0; j < n; j++)
 C[i][j] = 0.0f;

 const long stride_bt = n * (long)sizeof(double);
 const long stride_n = stride_bt;
 double B_d[N * N];
 for (int ii = 0; ii < n; ii++)
 for (int jj = 0; jj < n; jj++)
 B_d[ii * n + jj] = (double)B[ii][jj];

 for (int i = 0; i < n; i += P) {
 int mr = (i + P <= n) ? P : n - i;
 for (int j = 0; j < n; j += P) {
 int nr = (j + P <= n) ? P : n - j;
 asm volatile(MZERO(M2));
 for (int kk = 0; kk < n; kk += P) {
 int kr = (kk + P <= n) ? P : n - kk;
 /* Gather A tile with float→double conversion (required) */
 double At[9] = {0};
 for (int r = 0; r < mr; r++)
 for (int c = 0; c < kr; c++)
 At[r * P + c] = (double)A[i + r][kk + c];

 /* Load BT tile directly if full, gather if boundary */
 if (nr == P && kr == P) {
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride_bt), "r"(At),
 "r"(stride_n), "r"(&B_d[kk * n + j])
 : "t0", "t1", "memory"
 );
 } else {
 double b_pad[9] = {0};
 for (int r = 0; r < nr; r++)
 for (int c = 0; c < kr; c++)
 b_pad[r * P + c] = B_d[(j + r) * n + kk + c];
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(s24), "r"(At),
 "r"(s24), "r"(b_pad)
 : "t0", "t1", "memory"
 );
 }
 }
 /* Store result tile and convert double→float */
 double Ct[9];
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(s24), "r"(Ct)
 : "t0", "t1", "memory"
 );
 for (int r = 0; r < mr; r++)
 for (int c = 0; c < nr; c++)
 C[i + r][j + c] = (float)Ct[r * P + c];
 }
 }
}

#undef N

/* Rest of the source files */
#include "spacecraft_ai_weights.c"
#include "spacecraft_ai_workloads.c"
#include "spacecraft_ai_main.c"
