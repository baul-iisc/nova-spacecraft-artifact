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
 * spacecraft_ai_hw_mat.c -- Unity build with AME matrix acceleration
 *
 * mat_mul (15x15 float EKF matrix multiply) replaced with AME 3x3
 * tiled MLDS/SYS_MMACD/MSDS inline assembly. Float<->double conversion
 * at tile boundaries (AME registers are double-only).
 * N=15 = 5 tiles of 3 -- no edge handling needed.
 *
 * Built as a single compilation unit that #includes all original sources.
 */

/* Pull in system headers via the project header first */
#include "spacecraft_ai.h"
#include "../flight_compliance.h"

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
