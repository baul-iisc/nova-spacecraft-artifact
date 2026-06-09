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

/*****************************************************************************
 * Autonomous Mission Management Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Combined (CORDIC trig + AME 3x3 tile matrix accelerator)
 * Application: (see workload description below)
 *
 * Description:
 * Implements the onboard autonomous mission management system for scheduling,
 * resource allocation, and contingency handling. This workload includes telemetry
 * anomaly detection via FFT, power budget matrix optimization, orbital event
 * prediction, and mission timeline replanning. Representative of the autonomous
 * mission planning subsystem on a planetary science orbiter and applicable to all
 * deep space missions requiring autonomous operation during communication gaps.
 *
 * This workload is part of the spacecraft onboard computing benchmark suite
 * designed for characterizing and accelerating real-world space mission
 * computations on RISC-V platforms with custom matrix acceleration hardware.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright 2025 the authors of IEEE TC paper TC-2025-09-0830
 * ("Characterizing and Accelerating Spacecraft Onboard Workloads
 *  on RISC-V Platform").  Licensed under the Apache License,
 * Version 2.0; see LICENSE at the root of this repository.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
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
/* ------------------------------------------------------------------ */
/* Software math wrappers (profile-visible, accelerator-ready) */
/* HW Combined: CORDIC trig + AME matrix accelerator */
/* ------------------------------------------------------------------ */
#define sw_sin(a) hw_sin(a)
#define sw_cos(a) hw_cos(a)
#define sw_sqrt(a) sqrt(a)
#define sw_atan2(y, x) hw_atan2(y, x)
#define sw_fabs(a) fabs(a)
#define sw_acos(a) hw_acos(a)

/* ---------- Configuration ---------- */
#define STATE_DIM 18 /* Resource state: power(3), thermal(3), data(3),
 memory(3), attitude(3), orbit(3) */
#define NUM_TASKS 12 /* Concurrent schedulable tasks */
#define NUM_RESOURCES 6 /* Resource categories */
#define SCHEDULE_HORIZON 24 /* Scheduling slots per optimization window */
#define FFT_LEN 64 /* DFT length for telemetry analysis (power of 2)*/
#define NUM_TIMESTEPS 50 /* Simulation timesteps */
#define DT 30.0 /* Planning cycle interval (seconds) */
#define BATTERY_CAPACITY_WH 1200.0 /* 1.2 kWh battery (small s/c bus) */
#define MARS_OBLIQUITY 0.4396 /* 25.19 deg in radians */
#define ORBIT_ECCENTRICITY 0.25 /* Representative planetary-science orbit (eccentric LRO-class) */
#define DCM_REORTH_WARN_THRESH 1e-6 /* Frobenius-norm threshold for R^T·R-I */

/* Resource indices */
#define RES_POWER 0
#define RES_THERMAL 1
#define RES_DATARATE 2
#define RES_MEMORY 3
#define RES_ATTITUDE 4
#define RES_ORBIT 5

/* Task priority levels */
#define PRIO_CRITICAL 10
#define PRIO_HIGH 7
#define PRIO_MEDIUM 5
#define PRIO_LOW 3
#define PRIO_OPTIONAL 1

/* ------------------------------------------------------------------ */
/* Generic matrix operations (pure software, 3x3 tile-friendly) */
/* ------------------------------------------------------------------ */

// HW Combined variant: AME 3x3 tiled SYS_MMACD matrix multiply
static void mat_mul(const double *A, const double *B, double *C,
 int m, int k, int n)
{
 /* ── Optimized: pre-transpose B → direct MLDS/MSDS (zero gather/scatter) ── */
 double *BT = (double *)flight_malloc_impl((size_t)n * k * sizeof(double));
 memset(C, 0, (size_t)m * n * sizeof(double));

 const long stride_k = k * (long)sizeof(double);
 const long stride_n = n * (long)sizeof(double);
 const long s24 = 24;

 int m3a = (m / 3) * 3;
 int n3a = (n / 3) * 3;
 int k3a = (k / 3) * 3;

 for (int i = 0; i < m3a; i += 3) {
 for (int j = 0; j < n3a; j += 3) {
 asm volatile(MZERO(M2));

 /* Full K-tiles — direct stride load, zero scalar overhead */
 for (int kk = 0; kk < k3a; kk += 3) {
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride_k), "r"(&A[i * k + kk]),
 "r"(stride_n), "r"(&B[kk * n + j])
 : "t0", "t1", "memory"
 );
 }

 /* K-remainder tile (gather with zero-pad) */
 if (k3a < k) {
 double a_pad[9] = {0}, b_pad[9] = {0};
 for (int r = 0; r < 3; r++)
 for (int c = 0; c < k - k3a; c++) {
 a_pad[r * 3 + c] = A[(i + r) * k + k3a + c];
 b_pad[r * 3 + c] = B[(j + r) * k + k3a + c];
 }
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(s24), "r"(a_pad),
 "r"(s24), "r"(b_pad)
 : "t0", "t1", "memory"
 );
 }

 /* Store directly to C with stride N*8 (zero scatter overhead) */
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(stride_n), "r"(&C[i * n + j])
 : "t0", "t1", "memory"
 );
 }

 /* N-remainder columns — SW fallback */
 for (int j = n3a; j < n; j++)
 for (int r = 0; r < 3; r++)
 for (int kk = 0; kk < k; kk++)
 C[(i + r) * n + j] += A[(i + r) * k + kk] * B[kk * n + j];
 }

 /* M-remainder rows — SW fallback */
 for (int i = m3a; i < m; i++)
 for (int j = 0; j < n; j++)
 for (int kk = 0; kk < k; kk++)
 C[i * n + j] += A[i * k + kk] * B[kk * n + j];
}


static void mat_transpose(const double *A, double *AT, int m, int n)
{
 for (int i = 0; i < m; i++)
 for (int j = 0; j < n; j++)
 AT[j * m + i] = A[i * n + j];
}

static void mat_add(const double *A, const double *B, double *C, int m, int n)
{
 int sz = m * n;
 for (int i = 0; i < sz; i++) C[i] = A[i] + B[i];
}

__attribute__((unused))
static void mat_sub(const double *A, const double *B, double *C, int m, int n)
{
 int sz = m * n;
 for (int i = 0; i < sz; i++) C[i] = A[i] - B[i];
}

/* ---- AME 3x3 tile inverse helpers ---- */
static inline void hw_inv3(const double *M, double *R) {
 double det =
 M[0]*(M[4]*M[8]-M[5]*M[7])
 - M[1]*(M[3]*M[8]-M[5]*M[6])
 + M[2]*(M[3]*M[7]-M[4]*M[6]);
 if (fabs(det) < 1e-30) det = 1e-30;
 double inv = 1.0 / det;
 R[0]= (M[4]*M[8]-M[5]*M[7])*inv; R[1]=-(M[1]*M[8]-M[2]*M[7])*inv; R[2]= (M[1]*M[5]-M[2]*M[4])*inv;
 R[3]=-(M[3]*M[8]-M[5]*M[6])*inv; R[4]= (M[0]*M[8]-M[2]*M[6])*inv; R[5]=-(M[0]*M[5]-M[2]*M[3])*inv;
 R[6]= (M[3]*M[7]-M[4]*M[6])*inv; R[7]=-(M[0]*M[7]-M[1]*M[6])*inv; R[8]= (M[0]*M[4]-M[1]*M[3])*inv;
}

static inline void hw_extract_block(const double *S, int ld,
 int r0, int c0, int br, int bc,
 double *D) {
 for (int i = 0; i < br; i++)
 for (int j = 0; j < bc; j++)
 D[i*bc+j] = S[(r0+i)*ld+(c0+j)];
}

static inline void hw_insert_block(double *D, int ld,
 int r0, int c0, int br, int bc,
 const double *S) {
 for (int i = 0; i < br; i++)
 for (int j = 0; j < bc; j++)
 D[(r0+i)*ld+(c0+j)] = S[i*bc+j];
}

static inline void hw_inv1(const double *M, double *R) {
 double v = M[0];
 if (fabs(v) < 1e-30) v = 1e-30;
 R[0] = 1.0 / v;
}

static inline void hw_inv2(const double *M, double *R) {
 double det = M[0]*M[3] - M[1]*M[2];
 if (fabs(det) < 1e-30) det = 1e-30;
 double inv = 1.0 / det;
 R[0] = M[3]*inv; R[1] = -M[1]*inv;
 R[2] = -M[2]*inv; R[3] = M[0]*inv;
}

/* General contiguous matmul for small blocks inside Schur */
static void hw_matmul_cont(const double *A, const double *B, double *C,
 int m, int n, int p)
{
 /* ── Optimized: pre-transpose B → direct MLDS/MSDS (zero gather/scatter) ── */
 double *BT = (double *)flight_malloc_impl((size_t)p * n * sizeof(double));
 memset(C, 0, (size_t)m * p * sizeof(double));

 const long stride_k = n * (long)sizeof(double);
 const long stride_n = p * (long)sizeof(double);
 const long s24 = 24;

 int m3a = (m / 3) * 3;
 int n3a = (p / 3) * 3;
 int k3a = (n / 3) * 3;

 for (int i = 0; i < m3a; i += 3) {
 for (int j = 0; j < n3a; j += 3) {
 asm volatile(MZERO(M2));

 /* Full K-tiles — direct stride load, zero scalar overhead */
 for (int kk = 0; kk < k3a; kk += 3) {
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride_k), "r"(&A[i * n + kk]),
 "r"(stride_n), "r"(&B[kk * p + j])
 : "t0", "t1", "memory"
 );
 }

 /* K-remainder tile (gather with zero-pad) */
 if (k3a < n) {
 double a_pad[9] = {0}, b_pad[9] = {0};
 for (int r = 0; r < 3; r++)
 for (int c = 0; c < n - k3a; c++) {
 a_pad[r * 3 + c] = A[(i + r) * n + k3a + c];
 b_pad[r * 3 + c] = B[(j + r) * n + k3a + c];
 }
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(s24), "r"(a_pad),
 "r"(s24), "r"(b_pad)
 : "t0", "t1", "memory"
 );
 }

 /* Store directly to C with stride N*8 (zero scatter overhead) */
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(stride_n), "r"(&C[i * p + j])
 : "t0", "t1", "memory"
 );
 }

 /* N-remainder columns — SW fallback */
 for (int j = n3a; j < p; j++)
 for (int r = 0; r < 3; r++)
 for (int kk = 0; kk < n; kk++)
 C[(i + r) * p + j] += A[(i + r) * n + kk] * B[kk * p + j];
 }

 /* M-remainder rows — SW fallback */
 for (int i = m3a; i < m; i++)
 for (int j = 0; j < p; j++)
 for (int kk = 0; kk < n; kk++)
 C[i * p + j] += A[i * n + kk] * B[kk * p + j];
}


/* Recursive Schur complement inverse (flat arrays) */
static void hw_inverse_recursive(const double *A, double *R, int n) {
 if (n == 1) { hw_inv1(A, R); return; }
 if (n == 2) { hw_inv2(A, R); return; }
 if (n == 3) { hw_inv3(A, R); return; }
 int k = n / 2, rest = n - k;
 double w1[STATE_DIM*STATE_DIM], w2[STATE_DIM*STATE_DIM], w3[STATE_DIM*STATE_DIM];
 double Ai[STATE_DIM*STATE_DIM], S[STATE_DIM*STATE_DIM], Si[STATE_DIM*STATE_DIM];
 hw_extract_block(A,n,0,0,k,k,w1);
 hw_inverse_recursive(w1, Ai, k);
 hw_extract_block(A,n,k,0,rest,k,w1);
 hw_matmul_cont(w1, Ai, w2, rest, k, k);
 hw_extract_block(A,n,0,k,k,rest,w1);
 hw_matmul_cont(w2, w1, S, rest, k, rest);
 hw_extract_block(A,n,k,k,rest,rest,w3);
 for (int i=0;i<rest*rest;i++) S[i] = w3[i] - S[i];
 hw_inverse_recursive(S, Si, rest);
 hw_extract_block(A,n,0,k,k,rest,w1);
 hw_matmul_cont(Ai,w1,w2,k,k,rest);
 hw_matmul_cont(w2,Si,w3,k,rest,rest);
 for (int i=0;i<k*rest;i++) w3[i] = -w3[i];
 hw_insert_block(R,n,0,k,k,rest,w3);
 hw_extract_block(A,n,k,0,rest,k,w1);
 hw_matmul_cont(Si,w1,w2,rest,k,k);
 hw_matmul_cont(w2,Ai,w3,rest,k,k);
 for (int i=0;i<rest*k;i++) w3[i] = -w3[i];
 hw_insert_block(R,n,k,0,rest,k,w3);
 hw_extract_block(R,n,0,k,k,rest,w1);
 hw_extract_block(A,n,k,0,rest,k,w2);
 hw_matmul_cont(w1,w2,w3,k,rest,k);
 for (int i=0;i<k*k;i++) w3[i] = Ai[i] - w3[i];
 hw_insert_block(R,n,0,0,k,k,w3);
 hw_insert_block(R,n,k,k,rest,rest,Si);
}

// HW Combined variant: Schur complement recursive with AME 3x3 tiles
static int mat_inv(const double *A, double *Ainv, int n)
{
 if (n > STATE_DIM) return -1;
 double flat_in[STATE_DIM * STATE_DIM];
 for (int i = 0; i < n * n; i++)
 flat_in[i] = A[i];
 /* Regularise diagonal for numerical stability */
 for (int i = 0; i < n; i++)
 flat_in[i * n + i] += 1e-12;
 hw_inverse_recursive(flat_in, Ainv, n);
 return 0;
}

/**
 * Matrix multiply wrapper for profiling (delegates to mat_mul).
 */
static void sw_matrix_multiply(const double *A, const double *B, double *C,
 int m, int k, int n)
{
 mat_mul(A, B, C, m, k, n);
}

/* ------------------------------------------------------------------ */
/* Task and resource data structures */
/* ------------------------------------------------------------------ */

typedef struct {
 int id;
 int priority;
 double duration; /* Execution time (s) */
 double resource_need[NUM_RESOURCES]; /* Resource consumption per unit time */
 int earliest_start; /* Earliest scheduling slot */
 int latest_finish; /* Latest scheduling slot */
 int assigned_slot; /* -1 = unscheduled */
 double score; /* Computed scheduling score */
} Task;

typedef struct {
 double capacity[NUM_RESOURCES]; /* Total available per slot */
 double consumed[SCHEDULE_HORIZON][NUM_RESOURCES]; /* Current allocation */
} ResourceBudget;

/* ------------------------------------------------------------------ */
/* DFT for telemetry anomaly detection */
/* ------------------------------------------------------------------ */

static void compute_dft_magnitude(const double *x, double *mag, int N)
{
 for (int k = 0; k <= N / 2; k++) {
 double re = 0.0, im = 0.0;
 for (int n = 0; n < N; n++) {
 double angle = 2.0 * M_PI * k * n / N;
 re += x[n] * sw_cos(angle);
 im -= x[n] * sw_sin(angle);
 }
 mag[k] = sw_sqrt(re * re + im * im) / N;
 }
}

/* ------------------------------------------------------------------ */
/* Priority-weighted scheduling score computation */
/* ------------------------------------------------------------------ */

/**
 * Compute scheduling scores using a weighted matrix approach.
 * The score for each task considers: priority, resource availability,
 * deadline urgency, and inter-task dependencies.
 *
 * W is a NUM_TASKS x NUM_TASKS priority-interaction matrix.
 * r is a NUM_TASKS x NUM_RESOURCES resource-utilization matrix.
 */
static void compute_scheduling_scores(Task tasks[], int num_tasks,
 const ResourceBudget *budget,
 double W[NUM_TASKS * NUM_TASKS],
 double scores[NUM_TASKS])
{
 /* Build priority interaction matrix W[i][j] = dependency weight */
 for (int i = 0; i < num_tasks; i++) {
 for (int j = 0; j < num_tasks; j++) {
 if (i == j) {
 W[i * num_tasks + j] = (double)tasks[i].priority;
 } else {
 /* Cross-task coupling: higher if they compete for same resources */
 double coupling = 0.0;
 for (int r = 0; r < NUM_RESOURCES; r++)
 coupling += tasks[i].resource_need[r] * tasks[j].resource_need[r];
 W[i * num_tasks + j] = -0.1 * coupling;
 }
 }
 }

 /* Build resource utilization vector per task */
 double R_util[NUM_TASKS * NUM_RESOURCES];
 for (int i = 0; i < num_tasks; i++) {
 double urgency = 1.0 / (1.0 + tasks[i].latest_finish - tasks[i].earliest_start);
 for (int r = 0; r < NUM_RESOURCES; r++) {
 double avail = budget->capacity[r];
 R_util[i * NUM_RESOURCES + r] = tasks[i].resource_need[r] / avail * urgency;
 }
 }

 /* Score = W * ones + penalty from resource utilization */
 /* Matrix-vector product: s = W * p where p is priority vector */
 double p[NUM_TASKS];
 for (int i = 0; i < num_tasks; i++)
 p[i] = (double)tasks[i].priority;

 double Wp[NUM_TASKS];
 mat_mul(W, p, Wp, num_tasks, num_tasks, 1);

 /* Add resource penalty: sum of utilization ratios */
 for (int i = 0; i < num_tasks; i++) {
 double res_penalty = 0.0;
 for (int r = 0; r < NUM_RESOURCES; r++)
 res_penalty += R_util[i * NUM_RESOURCES + r];
 scores[i] = Wp[i] - 2.0 * res_penalty;
 tasks[i].score = scores[i];
 }
}

/* ------------------------------------------------------------------ */
/* Constraint satisfaction: resource-feasibility check */
/* ------------------------------------------------------------------ */

/**
 * Check if assigning task t to slot s is feasible given current allocations.
 * Returns residual capacity (negative = infeasible).
 */
static double check_feasibility(const Task *t, int slot,
 const ResourceBudget *budget)
{
 double min_residual = 1e30;
 for (int r = 0; r < NUM_RESOURCES; r++) {
 double residual = budget->capacity[r] - budget->consumed[slot][r]
 - t->resource_need[r];
 if (residual < min_residual) min_residual = residual;
 }
 return min_residual;
}

/**
 * Greedy scheduling: assign tasks to slots by descending score.
 */
static int schedule_tasks(Task tasks[], int num_tasks,
 ResourceBudget *budget)
{
 int scheduled = 0;

 /* Sort by score (simple insertion sort) */
 for (int i = 1; i < num_tasks; i++) {
 Task key = tasks[i];
 int j = i - 1;
 while (j >= 0 && tasks[j].score < key.score) {
 tasks[j + 1] = tasks[j];
 j--;
 }
 tasks[j + 1] = key;
 }

 /* Assign each task to its best feasible slot */
 for (int i = 0; i < num_tasks; i++) {
 double best_residual = -1e30;
 int best_slot = -1;

 for (int s = tasks[i].earliest_start;
 s <= tasks[i].latest_finish && s < SCHEDULE_HORIZON; s++) {
 double res = check_feasibility(&tasks[i], s, budget);
 if (res >= 0 && res > best_residual) {
 best_residual = res;
 best_slot = s;
 }
 }

 if (best_slot >= 0) {
 tasks[i].assigned_slot = best_slot;
 for (int r = 0; r < NUM_RESOURCES; r++)
 budget->consumed[best_slot][r] += tasks[i].resource_need[r];
 scheduled++;
 } else {
 tasks[i].assigned_slot = -1;
 }
 }
 return scheduled;
}

/* ------------------------------------------------------------------ */
/* Resource state estimation via EKF */
/* ------------------------------------------------------------------ */

/**
 * Resource state vector (18 elements):
 * [0..2] Power: battery SOC, solar panel output, load (W)
 * [3..5] Thermal: 3 zone temperatures (degC)
 * [6..8] Data: downlink rate, buffer fill level, uplink rate (kbps)
 * [9..11] Memory: primary, backup, cache usage (MB)
 * [12..14] Attitude: 3 Euler angle errors (rad)
 * [15..17] Orbit: position error in RTN frame (km)
 */

static void resource_state_predict(double x[STATE_DIM], double dt,
 double sim_time,
 const Task tasks[], int num_tasks)
{
 /* Power subsystem: SOC draws from active tasks */
 double total_power = 0.0;
 for (int i = 0; i < num_tasks; i++)
 if (tasks[i].assigned_slot >= 0)
 total_power += tasks[i].resource_need[RES_POWER];
 x[0] -= (total_power * dt / 3600.0) / BATTERY_CAPACITY_WH * 100.0; /* SOC% */
 /* Solar panel voltage: varies with orbital illumination (5400s orbit) */
 x[1] = 28.0 * (0.9 + 0.1 * sw_sin(2.0 * M_PI * sim_time / 5400.0));
 /* Power bus load: first-order lag toward instantaneous demand (tau ~ 10 s) */
 x[2] += (total_power - x[2]) * dt / 10.0;

 /* Thermal: radiative balance including deep-space sink */
 double T_space_K = 2.726; /* CMB background temperature [K] */
 double sigma_sb = 5.67e-8;
 for (int z = 0; z < 3; z++) {
 double T_K = x[3 + z] + 273.15;
 double Q_rad = sigma_sb * (pow(T_K, 4) - pow(T_space_K, 4)) * 0.01;
 double Q_diss = total_power * 0.1 / 3.0; /* Dissipated per zone */
 x[3 + z] += (Q_diss - Q_rad * 0.001) * dt / 500.0;
 }

 /* Data: buffer dynamics with time-varying link rates */
 double data_gen = 0.0;
 for (int i = 0; i < num_tasks; i++)
 if (tasks[i].assigned_slot >= 0)
 data_gen += tasks[i].resource_need[RES_DATARATE];
 /* Downlink rate varies with ground station visibility (Gaussian pass profile) */
 double gs_elev = sw_sin(2.0 * M_PI * sim_time / 5400.0);
 double gs_visible = (gs_elev > 0.0) ? gs_elev : 0.0;
 x[6] += (256.0 * gs_visible - x[6]) * dt / 30.0; /* 30s link acquisition lag */
 x[7] += (data_gen - x[6]) * dt / 8.0; /* Buffer fill (kB) */
 if (x[7] < 0) x[7] = 0;
 /* Uplink rate: similar visibility dependence, lower bandwidth */
 x[8] += (64.0 * gs_visible - x[8]) * dt / 30.0;

 /* Memory: aggregate usage with autonomous management */
 double mem_use = 0.0;
 for (int i = 0; i < num_tasks; i++)
 if (tasks[i].assigned_slot >= 0)
 mem_use += tasks[i].resource_need[RES_MEMORY];
 x[9] += mem_use * dt / 60.0;
 /* Backup mirror syncs toward primary with replication lag (tau = 120 s) */
 x[10] += (x[9] * 0.5 - x[10]) * dt / 120.0;
 /* Cache reflects active usage with fast dynamics (tau = 5 s) */
 x[11] += (mem_use * 2.0 - x[11]) * dt / 5.0;

 /* Attitude: small drift proportional to elapsed time */
 for (int a = 0; a < 3; a++)
 x[12 + a] += 1e-5 * sw_sin(sim_time * 0.001 * (a + 1)) * dt;

 /* Orbit: slow along-track drift */
 x[15] += 0.001 * dt;
 x[16] *= 0.999;
 x[17] *= 0.999;
}

/**
 * Compute Jacobian for resource state prediction.
 */
static void compute_resource_jacobian(const double x[STATE_DIM],
 double F[STATE_DIM * STATE_DIM])
{
 memset(F, 0, STATE_DIM * STATE_DIM * sizeof(double));
 for (int i = 0; i < STATE_DIM; i++)
 F[i * STATE_DIM + i] = 1.0;

 /* Power bus lag: dx[2]/dt = (P_demand - x[2])/10 → F[2][2] = 1 - dt/10 */
 F[2 * STATE_DIM + 2] = 1.0 - DT / 10.0;

 /* Thermal coupling: cross-zone radiation */
 for (int z = 0; z < 3; z++) {
 double T_K = x[3 + z] + 273.15;
 F[(3 + z) * STATE_DIM + (3 + z)] = 1.0 - 4.0 * 5.67e-8 *
 pow(T_K, 3) * 0.01 * 0.001 * DT / 500.0;
 }

 /* Data buffer: dx[7]/dt depends on x[6] (downlink rate) */
 F[7 * STATE_DIM + 7] = 1.0;
 F[7 * STATE_DIM + 6] = -DT / 8.0;

 /* Downlink/uplink: first-order lag */
 F[6 * STATE_DIM + 6] = 1.0 - DT / 30.0;
 F[8 * STATE_DIM + 8] = 1.0 - DT / 30.0;

 /* Memory: backup syncs from primary */
 F[9 * STATE_DIM + 9] = 1.0;
 F[10 * STATE_DIM + 10] = 1.0 - DT / 120.0;
 F[10 * STATE_DIM + 9] = 0.5 * DT / 120.0; /* dx[10]/dx[9] */
 /* Cache dynamics */
 F[11 * STATE_DIM + 11] = 1.0 - DT / 5.0;

 /* Orbit decay */
 F[16 * STATE_DIM + 16] = 0.999;
 F[17 * STATE_DIM + 17] = 0.999;
}

/**
 * Compute orbital-phase-adaptive process noise scaling factor.
 * Near periapsis (true_anomaly ~ 0) dynamics are faster due to higher
 * orbital velocity, so inflate Q. Near apoapsis (true_anomaly ~ pi)
 * dynamics are slower, use baseline Q.
 *
 * Scale = (1 + e*cos(v))^2 / (1 - e^2)^2 (proportional to v_orb^2)
 */
static double compute_adaptive_q_scale(double true_anomaly, double ecc)
{
 double r_factor = 1.0 + ecc * sw_cos(true_anomaly);
 double baseline = 1.0 - ecc * ecc;
 if (baseline < 1e-12) baseline = 1e-12;
 return (r_factor * r_factor) / (baseline * baseline);
}

/**
 * EKF covariance prediction: P = F*P*F^T + Q
 */
static void ekf_predict_covariance(double P[STATE_DIM * STATE_DIM],
 const double F[STATE_DIM * STATE_DIM],
 const double Q_proc[STATE_DIM * STATE_DIM])
{
 double FP[STATE_DIM * STATE_DIM];
 double FT[STATE_DIM * STATE_DIM];
 double FPFT[STATE_DIM * STATE_DIM];

 mat_mul(F, P, FP, STATE_DIM, STATE_DIM, STATE_DIM);
 mat_transpose(F, FT, STATE_DIM, STATE_DIM);
 mat_mul(FP, FT, FPFT, STATE_DIM, STATE_DIM, STATE_DIM);
 mat_add(FPFT, Q_proc, P, STATE_DIM, STATE_DIM);
}

/**
 * EKF measurement update (generic).
 */
static void ekf_update(double x[STATE_DIM],
 double P[STATE_DIM * STATE_DIM],
 const double *z, const double *H,
 const double *R, int meas_dim)
{
 double Hx[6], y[6];
 mat_mul(H, x, Hx, meas_dim, STATE_DIM, 1);
 for (int i = 0; i < meas_dim; i++)
 y[i] = z[i] - Hx[i];

 double HP[6 * STATE_DIM], HT[STATE_DIM * 6];
 double S[36], Sinv[36], HPHT[36];
 mat_mul(H, P, HP, meas_dim, STATE_DIM, STATE_DIM);
 mat_transpose(H, HT, meas_dim, STATE_DIM);
 mat_mul(HP, HT, HPHT, meas_dim, STATE_DIM, meas_dim);
 mat_add(HPHT, R, S, meas_dim, meas_dim);
 mat_inv(S, Sinv, meas_dim);

 double PHT[STATE_DIM * 6], K[STATE_DIM * 6];
 mat_mul(P, HT, PHT, STATE_DIM, STATE_DIM, meas_dim);
 mat_mul(PHT, Sinv, K, STATE_DIM, meas_dim, meas_dim);

 double Ky[STATE_DIM];
 mat_mul(K, y, Ky, STATE_DIM, meas_dim, 1);
 for (int i = 0; i < STATE_DIM; i++)
 x[i] += Ky[i];

 double KH[STATE_DIM * STATE_DIM], I_KH[STATE_DIM * STATE_DIM];
 mat_mul(K, H, KH, STATE_DIM, meas_dim, STATE_DIM);
 for (int i = 0; i < STATE_DIM * STATE_DIM; i++)
 I_KH[i] = -KH[i];
 for (int i = 0; i < STATE_DIM; i++)
 I_KH[i * STATE_DIM + i] += 1.0;
 double Pnew[STATE_DIM * STATE_DIM];
 mat_mul(I_KH, P, Pnew, STATE_DIM, STATE_DIM, STATE_DIM);
 memcpy(P, Pnew, STATE_DIM * STATE_DIM * sizeof(double));
}

/* ------------------------------------------------------------------ */
/* Telemetry anomaly detection via DFT */
/* ------------------------------------------------------------------ */

/**
 * Analyse resource telemetry history using DFT to detect anomalous
 * periodicities that might indicate subsystem faults.
 */
static int detect_telemetry_anomaly(const double telem_history[][NUM_RESOURCES],
 int history_len,
 double anomaly_score[NUM_RESOURCES])
{
 double ts[FFT_LEN], mag[FFT_LEN / 2 + 1];
 int N = (history_len < FFT_LEN) ? history_len : FFT_LEN;
 int anomalies = 0;

 for (int r = 0; r < NUM_RESOURCES; r++) {
 for (int n = 0; n < N; n++)
 ts[n] = telem_history[history_len - N + n][r];

 compute_dft_magnitude(ts, mag, N);

 /* Anomaly: any non-DC component exceeds 3x the mean spectral magnitude */
 double mean_mag = 0.0;
 for (int k = 1; k <= N / 2; k++) mean_mag += mag[k];
 mean_mag /= (N / 2);

 anomaly_score[r] = 0.0;
 for (int k = 1; k <= N / 2; k++) {
 if (mag[k] > 3.0 * mean_mag)
 anomaly_score[r] += mag[k] / mean_mag;
 }
 if (anomaly_score[r] > 5.0) anomalies++;
 }
 return anomalies;
}

/* ------------------------------------------------------------------ */
/* Resource conflict resolution matrix */
/* ------------------------------------------------------------------ */

/**
 * Build and solve a 6x6 resource conflict matrix to redistribute
 * over-allocated resources among tasks.
 */
static void resolve_resource_conflicts(Task tasks[], int num_tasks,
 ResourceBudget *budget,
 int slot)
{
 double conflict[NUM_RESOURCES * NUM_RESOURCES];
 memset(conflict, 0, sizeof(conflict));

 /* Build conflict matrix: C[i][j] = sum of co-allocation of resources i,j */
 for (int t = 0; t < num_tasks; t++) {
 if (tasks[t].assigned_slot != slot) continue;
 for (int ri = 0; ri < NUM_RESOURCES; ri++)
 for (int rj = 0; rj < NUM_RESOURCES; rj++)
 conflict[ri * NUM_RESOURCES + rj] +=
 tasks[t].resource_need[ri] * tasks[t].resource_need[rj];
 }

 /* Add regularisation for invertibility */
 for (int i = 0; i < NUM_RESOURCES; i++)
 conflict[i * NUM_RESOURCES + i] += 0.01;

 /* Solve for optimal scaling: s = C^{-1} * capacity */
 double Cinv[NUM_RESOURCES * NUM_RESOURCES];
 mat_inv(conflict, Cinv, NUM_RESOURCES);

 double scaling[NUM_RESOURCES];
 mat_mul(Cinv, budget->capacity, scaling, NUM_RESOURCES, NUM_RESOURCES, 1);

 /* Apply scaling to budget consumption (NOT to task requirements) */
 for (int t = 0; t < num_tasks; t++) {
 if (tasks[t].assigned_slot != slot) continue;
 for (int r = 0; r < NUM_RESOURCES; r++) {
 double s = (scaling[r] > 0 && scaling[r] < 1.0) ? scaling[r] : 1.0;
 double scaled = tasks[t].resource_need[r] * s;
 budget->consumed[slot][r] += (scaled - tasks[t].resource_need[r]);
 }
 }
}

/* ------------------------------------------------------------------ */
/* Sensor simulation */
/* ------------------------------------------------------------------ */

static double rand_gauss(void)
{
 double u1 = (rand() / (double)RAND_MAX);
 double u2 = (rand() / (double)RAND_MAX);
 if (u1 < 1e-15) u1 = 1e-15;
 return sw_sqrt(-2.0 * log(u1)) * sw_cos(2.0 * M_PI * u2);
}

/* ------------------------------------------------------------------ */
/* Quaternion operations (attitude mode tracking) */
/* ------------------------------------------------------------------ */

typedef struct {
 double w, x, y, z;
} Quaternion;

/** Normalize quaternion to unit length. */
static Quaternion quaternion_normalize(Quaternion q)
{
 double norm = sw_sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
 if (norm < 1e-15) norm = 1e-15;
 q.w /= norm;
 q.x /= norm;
 q.y /= norm;
 q.z /= norm;
 return q;
}

/** Hamilton product: r = q1 * q2. */
static Quaternion quaternion_multiply(Quaternion q1, Quaternion q2)
{
 Quaternion r;
 r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
 r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
 r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
 r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
 return r;
}

/** Rotate 3D vector v by unit quaternion q: v' = q * [0,v] * q*. */
static void quaternion_rotate_vector(Quaternion q, const double v[3],
 double out[3])
{
 Quaternion vq = {0.0, v[0], v[1], v[2]};
 Quaternion qc = {q.w, -q.x, -q.y, -q.z};
 Quaternion tmp = quaternion_multiply(q, vq);
 Quaternion res = quaternion_multiply(tmp, qc);
 out[0] = res.x;
 out[1] = res.y;
 out[2] = res.z;
}

/** Convert unit quaternion to 3x3 direction cosine matrix (row-major). */
static void quaternion_to_dcm(Quaternion q, double dcm[9])
{
 double w = q.w, x = q.x, y = q.y, z = q.z;
 dcm[0] = 1.0 - 2.0 * (y * y + z * z);
 dcm[1] = 2.0 * (x * y - w * z);
 dcm[2] = 2.0 * (x * z + w * y);
 dcm[3] = 2.0 * (x * y + w * z);
 dcm[4] = 1.0 - 2.0 * (x * x + z * z);
 dcm[5] = 2.0 * (y * z - w * x);
 dcm[6] = 2.0 * (x * z - w * y);
 dcm[7] = 2.0 * (y * z + w * x);
 dcm[8] = 1.0 - 2.0 * (x * x + y * y);
}

/**
 * Gram-Schmidt re-orthogonalisation of a 3x3 DCM (row-major).
 * Corrects numerical drift so that R^T * R = I.
 * Algorithm: normalise row 0, orthogonalise row 1 against row 0
 * then normalise, set row 2 = row 0 x row 1 (right-handed frame).
 */
static void dcm_reorthogonalize(double dcm[9])
{
 /* Row 0: normalise */
 double n0 = sw_sqrt(dcm[0]*dcm[0] + dcm[1]*dcm[1] + dcm[2]*dcm[2]);
 if (n0 < 1e-15) n0 = 1e-15;
 dcm[0] /= n0; dcm[1] /= n0; dcm[2] /= n0;

 /* Row 1: subtract projection onto row 0, then normalise */
 double d10 = dcm[3]*dcm[0] + dcm[4]*dcm[1] + dcm[5]*dcm[2];
 dcm[3] -= d10 * dcm[0];
 dcm[4] -= d10 * dcm[1];
 dcm[5] -= d10 * dcm[2];
 double n1 = sw_sqrt(dcm[3]*dcm[3] + dcm[4]*dcm[4] + dcm[5]*dcm[5]);
 if (n1 < 1e-15) n1 = 1e-15;
 dcm[3] /= n1; dcm[4] /= n1; dcm[5] /= n1;

 /* Row 2: cross product row0 x row1 (ensures right-handed frame) */
 dcm[6] = dcm[1]*dcm[5] - dcm[2]*dcm[4];
 dcm[7] = dcm[2]*dcm[3] - dcm[0]*dcm[5];
 dcm[8] = dcm[0]*dcm[4] - dcm[1]*dcm[3];
}

/**
 * Compute Frobenius norm of (R^T * R - I) to quantify orthogonality error.
 */
static double dcm_orthogonality_error(const double dcm[9])
{
 double Rt[9], RtR[9];
 mat_transpose(dcm, Rt, 3, 3);
 sw_matrix_multiply(Rt, dcm, RtR, 3, 3, 3);
 double err = 0.0;
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 double e = RtR[i * 3 + j] - ((i == j) ? 1.0 : 0.0);
 err += e * e;
 }
 return sw_sqrt(err);
}

/* ------------------------------------------------------------------ */
/* Attitude mode definitions and mode management */
/* ------------------------------------------------------------------ */

#define ATT_MODE_NADIR 0 /* Earth/target-pointing for imaging */
#define ATT_MODE_SUN 1 /* Sun-pointing for power generation */
#define ATT_MODE_INERTIAL 2 /* Inertial hold for communication passes */
#define ATT_MODE_SAFE 3 /* Safe mode: sun-pointing + spin */
#define NUM_ATT_MODES 4

static const char *att_mode_names[NUM_ATT_MODES] = {
 "Nadir", "Sun-Point", "Inertial", "Safe"
};

/**
 * Compute the slew quaternion from current attitude to target attitude.
 * q_slew = q_target * q_current^{-1} (inverse of unit quat is conjugate)
 * Also returns the slew angle in radians.
 */
static double compute_slew_quaternion(Quaternion q_current, Quaternion q_target,
 Quaternion *q_slew)
{
 /* q_current inverse (conjugate for unit quaternion) */
 Quaternion q_cur_inv = {q_current.w, -q_current.x,
 -q_current.y, -q_current.z};
 *q_slew = quaternion_multiply(q_target, q_cur_inv);
 *q_slew = quaternion_normalize(*q_slew);

 /* Slew angle = 2 * acos(|w|) */
 double w_abs = sw_fabs(q_slew->w);
 if (w_abs > 1.0) w_abs = 1.0;
 return 2.0 * sw_acos(w_abs);
}

/**
 * Check sun exclusion angle constraint for sensitive instruments.
 * Given the spacecraft attitude quaternion and the sun direction in inertial
 * frame, compute the angle between the instrument boresight (body +Z axis)
 * and the sun. Returns the angle in radians; if below the exclusion limit
 * the instrument must be safed.
 */
static double check_sun_exclusion(Quaternion q_att, const double sun_dir_inertial[3],
 double exclusion_limit_rad)
{
 (void)exclusion_limit_rad;
 /* Instrument boresight is along body +Z axis */
 double boresight_body[3] = {0.0, 0.0, 1.0};
 double boresight_inertial[3];

 /* Rotate boresight to inertial frame using current attitude */
 quaternion_rotate_vector(q_att, boresight_body, boresight_inertial);

 /* Compute angle between boresight and sun direction */
 double dot = boresight_inertial[0] * sun_dir_inertial[0]
 + boresight_inertial[1] * sun_dir_inertial[1]
 + boresight_inertial[2] * sun_dir_inertial[2];
 if (dot > 1.0) dot = 1.0;
 if (dot < -1.0) dot = -1.0;
 double angle = sw_acos(dot);

 return angle; /* caller compares with exclusion_limit_rad */
}

/**
 * Select the required attitude mode based on current mission phase,
 * power status, and communication windows. Uses quaternion DCM to
 * verify attitude constraints via a 3x3 rotation matrix product.
 */
static int select_attitude_mode(int timestep, double battery_soc,
 int comm_window_active,
 Quaternion q_current,
 const Quaternion mode_quats[NUM_ATT_MODES],
 double *slew_angle_out)
{
 int target_mode = ATT_MODE_NADIR; /* Default: nadir-pointing */
 Quaternion q_slew;

 /* Safe mode override: low battery triggers sun-pointing safe mode */
 if (battery_soc < 30.0) {
 target_mode = ATT_MODE_SAFE;
 }
 /* Communication pass: switch to inertial hold */
 else if (comm_window_active) {
 target_mode = ATT_MODE_INERTIAL;
 }
 /* Power charging phase every 15 timesteps */
 else if ((timestep % 15) < 3) {
 target_mode = ATT_MODE_SUN;
 }

 /* Compute slew angle from current attitude to target mode attitude */
 *slew_angle_out = compute_slew_quaternion(q_current,
 mode_quats[target_mode],
 &q_slew);

 /* Verify slew DCM consistency: trace(DCM_slew) = 1 + 2*cos(slew_angle),
 and DCM_slew * DCM_slew^T should yield identity (orthogonality) */
 double dcm_slew[9], dcm_slew_T[9], dcm_product[9];
 quaternion_to_dcm(q_slew, dcm_slew);

 /* Gram-Schmidt re-orthogonalisation: correct numeric drift in DCM */
 double orth_err_pre = dcm_orthogonality_error(dcm_slew);
 dcm_reorthogonalize(dcm_slew);
 double orth_err_post = dcm_orthogonality_error(dcm_slew);
 if (orth_err_pre > DCM_REORTH_WARN_THRESH)
 FLIGHT_LOG(" DCM reorth: ||R^T*R-I|| %.2e -> %.2e\n",
 orth_err_pre, orth_err_post);

 mat_transpose(dcm_slew, dcm_slew_T, 3, 3);
 sw_matrix_multiply(dcm_slew, dcm_slew_T, dcm_product, 3, 3, 3);

 /* Trace of rotation matrix directly gives 1+2*cos(theta) */
 double trace = dcm_slew[0] + dcm_slew[4] + dcm_slew[8];
 double expected_trace = 1.0 + 2.0 * sw_cos(*slew_angle_out);
 if (fabs(trace - expected_trace) > 0.01) {
 FLIGHT_LOG("WARN: DCM trace mismatch %.4f vs %.4f\n", trace, expected_trace);
 }

 return target_mode;
}

/* ================================================================== */
/* MAIN */
/* ================================================================== */

int main(void)
{
 FLIGHT_LOG("[MGT] cfg: state_dim=%d tasks=%d resources=%d horizon=%d steps=%d\n", STATE_DIM, NUM_TASKS, NUM_RESOURCES, SCHEDULE_HORIZON, NUM_TIMESTEPS);

 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(3851);

 /* ---------- Initialise tasks ---------- */
 Task tasks[NUM_TASKS];
 const char *task_names[NUM_TASKS] = {
 "ADCS_Control", "Thermal_Mgmt", "Downlink_Pass", "Imager_Science",
 "SAR_Capture", "NAV_Update", "FDIR_Check", "Housekeeping",
 "Orbit_Maint", "Payload_Cal", "Comm_Window", "Data_Compress"
 };
 int task_prios[NUM_TASKS] = {
 PRIO_CRITICAL, PRIO_HIGH, PRIO_HIGH, PRIO_MEDIUM,
 PRIO_MEDIUM, PRIO_HIGH, PRIO_CRITICAL, PRIO_LOW,
 PRIO_MEDIUM, PRIO_LOW, PRIO_HIGH, PRIO_MEDIUM
 };
 /* Resource needs: [power, thermal, datarate, memory, attitude, orbit] */
 double resource_profiles[NUM_TASKS][NUM_RESOURCES] = {
 {15.0, 2.0, 1.0, 4.0, 10.0, 0.5}, /* ADCS */
 {5.0, 8.0, 0.5, 2.0, 0.0, 0.0}, /* Thermal */
 {20.0, 3.0, 200.0, 50.0, 2.0, 0.0}, /* Downlink */
 {25.0, 5.0, 100.0, 80.0, 5.0, 0.0}, /* Imager */
 {30.0, 6.0, 150.0, 100.0, 8.0, 0.0}, /* SAR */
 {10.0, 1.0, 10.0, 20.0, 3.0, 5.0}, /* NAV */
 {3.0, 0.5, 5.0, 10.0, 0.0, 0.0}, /* FDIR */
 {2.0, 0.2, 2.0, 5.0, 0.0, 0.0}, /* Housekeeping */
 {12.0, 1.5, 1.0, 5.0, 1.0, 10.0}, /* Orbit Maintenance */
 {8.0, 2.0, 20.0, 30.0, 4.0, 0.0}, /* Payload Cal */
 {18.0, 2.5, 180.0, 40.0, 2.0, 0.0}, /* Comm Window */
 {10.0, 3.0, 0.5, 60.0, 0.0, 0.0}, /* Data Compress */
 };

 for (int i = 0; i < NUM_TASKS; i++) {
 tasks[i].id = i;
 tasks[i].priority = task_prios[i];
 tasks[i].duration = 30.0 + (i % 4) * 15.0;
 for (int r = 0; r < NUM_RESOURCES; r++)
 tasks[i].resource_need[r] = resource_profiles[i][r];
 tasks[i].earliest_start = i % 6;
 tasks[i].latest_finish = tasks[i].earliest_start + 6 + (i % 3) * 3;
 if (tasks[i].latest_finish >= SCHEDULE_HORIZON)
 tasks[i].latest_finish = SCHEDULE_HORIZON - 1;
 tasks[i].assigned_slot = -1;
 tasks[i].score = 0.0;
 }

 /* ---------- Initialise resource budget ---------- */
 ResourceBudget budget;
 budget.capacity[RES_POWER] = 80.0; /* W */
 budget.capacity[RES_THERMAL] = 15.0; /* Thermal margin */
 budget.capacity[RES_DATARATE] = 400.0; /* kbps */
 budget.capacity[RES_MEMORY] = 200.0; /* MB per slot */
 budget.capacity[RES_ATTITUDE] = 20.0; /* Attitude margin */
 budget.capacity[RES_ORBIT] = 15.0; /* Orbit margin */
 memset(budget.consumed, 0, sizeof(budget.consumed));

 /* ---------- Initialise attitude mode quaternions ---------- */
 /* Define reference quaternions for each operational attitude mode */
 Quaternion mode_quaternions[NUM_ATT_MODES];

 /* Nadir-pointing: identity (body Z aligned with nadir) */
 mode_quaternions[ATT_MODE_NADIR] = (Quaternion){1.0, 0.0, 0.0, 0.0};

 /* Sun-pointing: 45-degree rotation about body Y axis */
 double sun_half = 22.5 * M_PI / 180.0;
 mode_quaternions[ATT_MODE_SUN] = quaternion_normalize(
 (Quaternion){sw_cos(sun_half), 0.0, sw_sin(sun_half), 0.0});

 /* Inertial hold: 30-degree rotation about body X axis (comm antenna alignment) */
 double comm_half = 15.0 * M_PI / 180.0;
 mode_quaternions[ATT_MODE_INERTIAL] = quaternion_normalize(
 (Quaternion){sw_cos(comm_half), sw_sin(comm_half), 0.0, 0.0});

 /* Safe mode: 90-degree rotation about [1,1,0]/sqrt(2) axis */
 double safe_half = 45.0 * M_PI / 180.0;
 double axis_norm = sw_sqrt(0.5);
 mode_quaternions[ATT_MODE_SAFE] = quaternion_normalize(
 (Quaternion){sw_cos(safe_half),
 sw_sin(safe_half) * axis_norm,
 sw_sin(safe_half) * axis_norm,
 0.0});

 /* Current spacecraft attitude (starts at nadir-pointing) */
 Quaternion q_spacecraft = mode_quaternions[ATT_MODE_NADIR];
 int current_att_mode = ATT_MODE_NADIR;
 int total_mode_switches = 0;
 int sun_exclusion_violations = 0;

 /* Sun direction in inertial frame (varies slowly with orbit) */
 double sun_dir[3] = {1.0, 0.0, 0.0};
 /* Sun exclusion limit for sensitive imager (20 degrees) */
 double sun_exclusion_limit = 20.0 * M_PI / 180.0;

 /* ---------- Initialise EKF state ---------- */
 double x[STATE_DIM];
 double P[STATE_DIM * STATE_DIM];
 double Q_proc[STATE_DIM * STATE_DIM];
 memset(P, 0, sizeof(P));
 memset(Q_proc, 0, sizeof(Q_proc));

 x[0] = 85.0; x[1] = 28.0; x[2] = 75.0; /* Power: SOC%, panel V, load W */
 x[3] = 22.0; x[4] = 25.0; x[5] = 20.0; /* Thermal zones (degC) */
 x[6] = 256.0; x[7] = 100.0; x[8] = 64.0; /* Data: DL, buffer kB, UL */
 x[9] = 50.0; x[10] = 25.0; x[11] = 10.0; /* Memory: pri, backup, cache MB */
 x[12] = 0.001; x[13] = -0.002; x[14] = 0.0005; /* Attitude errors (rad) */
 x[15] = 0.1; x[16] = -0.05; x[17] = 0.02; /* Orbit errors (km) */

 for (int i = 0; i < STATE_DIM; i++) {
 P[i * STATE_DIM + i] = (i < 3) ? 5.0 : (i < 6) ? 1.0 :
 (i < 9) ? 10.0 : (i < 12) ? 5.0 :
 (i < 15) ? 1e-4 : 0.01;
 Q_proc[i * STATE_DIM + i] = P[i * STATE_DIM + i] * 0.01;
 }

 /* Measurement setup: observe power + thermal (6 states) */
 int meas_dim = 6;
 double H_meas[6 * STATE_DIM];
 double R_meas[36];
 memset(H_meas, 0, sizeof(H_meas));
 memset(R_meas, 0, sizeof(R_meas));
 for (int i = 0; i < meas_dim; i++) {
 H_meas[i * STATE_DIM + i] = 1.0; /* Direct observation of states 0..5 */
 R_meas[i * meas_dim + i] = 0.1;
 }

 /* Telemetry history for anomaly detection */
 double telem_history[NUM_TIMESTEPS][NUM_RESOURCES];

 /* ---------- Main simulation loop ---------- */
 int total_scheduled = 0;
 int total_anomalies = 0;

 for (int t = 0; t < NUM_TIMESTEPS; t++) {

 /* -- 1. Reset schedule for this cycle -- */
 memset(budget.consumed, 0, sizeof(budget.consumed));
 for (int i = 0; i < NUM_TASKS; i++)
 tasks[i].assigned_slot = -1;

 /* -- 1a. Attitude mode management (quaternion-based) -- */
 /* Update sun direction based on orbital position */
 double orbit_angle = 2.0 * M_PI * t / (double)NUM_TIMESTEPS;
 sun_dir[0] = sw_cos(orbit_angle);
 sun_dir[1] = sw_sin(orbit_angle) * sw_cos(MARS_OBLIQUITY);
 sun_dir[2] = sw_sin(orbit_angle) * sw_sin(MARS_OBLIQUITY);

 /* Determine if communication window is active (every 12 steps, 2-step window) */
 int comm_active = ((t % 12) >= 10) ? 1 : 0;

 /* Select required attitude mode for this timestep */
 double slew_angle = 0.0;
 int new_mode = select_attitude_mode(t, x[0], comm_active,
 q_spacecraft,
 mode_quaternions,
 &slew_angle);

 if (new_mode != current_att_mode) {
 /* Execute mode transition: interpolate quaternion toward target */
 Quaternion q_slew;
 compute_slew_quaternion(q_spacecraft,
 mode_quaternions[new_mode], &q_slew);

 /* Apply fraction of the slew per timestep (smooth transition) */
 double slew_frac = (slew_angle > 0.01) ? 0.3 : 1.0;
 double half_interp = slew_frac * sw_acos(sw_fabs(q_slew.w));
 if (half_interp > 1e-6) {
 double s = sw_sin(half_interp);
 double axis_len = sw_sqrt(q_slew.x * q_slew.x +
 q_slew.y * q_slew.y +
 q_slew.z * q_slew.z);
 Quaternion dq;
 if (axis_len > 1e-12) {
 dq.w = sw_cos(half_interp);
 dq.x = q_slew.x / axis_len * s;
 dq.y = q_slew.y / axis_len * s;
 dq.z = q_slew.z / axis_len * s;
 } else {
 dq = (Quaternion){1.0, 0.0, 0.0, 0.0};
 }
 q_spacecraft = quaternion_normalize(
 quaternion_multiply(dq, q_spacecraft));
 }

 current_att_mode = new_mode;
 total_mode_switches++;
 }

 /* Check sun exclusion constraint for the imager instrument */
 double sun_angle = check_sun_exclusion(q_spacecraft, sun_dir,
 sun_exclusion_limit);
 if (sun_angle < sun_exclusion_limit) {
 sun_exclusion_violations++;
 /* Switch imager payload to safe mode */
 }

 /* -- 2. Compute scheduling scores (matrix-intensive) -- */
 double W[NUM_TASKS * NUM_TASKS];
 double scores[NUM_TASKS];
 compute_scheduling_scores(tasks, NUM_TASKS, &budget, W, scores);

 /* -- 3. Schedule tasks -- */
 int sched = schedule_tasks(tasks, NUM_TASKS, &budget);
 total_scheduled += sched;

 /* -- 4. Resolve resource conflicts for occupied slots -- */
 for (int s = 0; s < SCHEDULE_HORIZON; s++) {
 int count = 0;
 for (int i = 0; i < NUM_TASKS; i++)
 if (tasks[i].assigned_slot == s) count++;
 if (count > 1)
 resolve_resource_conflicts(tasks, NUM_TASKS, &budget, s);
 }

 /* -- 5. Resource state prediction (EKF) -- */
 double sim_time = t * DT;
 double F[STATE_DIM * STATE_DIM];
 compute_resource_jacobian(x, F);
 resource_state_predict(x, DT, sim_time, tasks, NUM_TASKS);

 /* Orbital-phase-adaptive process noise: inflate Q near periapsis
 where dynamics change faster due to higher orbital velocity */
 double q_scale = compute_adaptive_q_scale(orbit_angle,
 ORBIT_ECCENTRICITY);
 double Q_scaled[STATE_DIM * STATE_DIM];
 for (int qi = 0; qi < STATE_DIM * STATE_DIM; qi++)
 Q_scaled[qi] = Q_proc[qi] * q_scale;
 ekf_predict_covariance(P, F, Q_scaled);

 /* -- 6. Simulated sensor measurement and EKF update -- */
 double z_meas[6];
 for (int i = 0; i < meas_dim; i++)
 z_meas[i] = x[i] + 0.5 * rand_gauss();
 ekf_update(x, P, z_meas, H_meas, R_meas, meas_dim);

 /* -- 7. Record telemetry -- */
 for (int r = 0; r < NUM_RESOURCES; r++) {
 double utilisation = 0.0;
 for (int s = 0; s < SCHEDULE_HORIZON; s++)
 utilisation += budget.consumed[s][r];
 utilisation /= (SCHEDULE_HORIZON * budget.capacity[r]);
 telem_history[t][r] = utilisation;
 }

 /* -- 8. Telemetry anomaly detection (DFT, every 6 steps) -- */
 double anomaly_score[NUM_RESOURCES] = {0};
 int anomalies = 0;
 if (t >= FFT_LEN && (t % 6) == 0) {
 anomalies = detect_telemetry_anomaly(telem_history, t + 1,
 anomaly_score);
 total_anomalies += anomalies;
 }

 /* -- 9. Inject anomaly at t=25 (simulate power degradation) -- */
 if (t == 25) {
 budget.capacity[RES_POWER] *= 0.7; /* 30% power reduction */
 FLIGHT_LOG(" ** ALERT: Power degradation injected at t=%d **\n", t);
 }

 /* -- 10. Status output -- */
 if (t % 10 == 0) {
 FLIGHT_LOG("t=%3d | Sched=%2d/%d | SOC=%.1f%% | T=%.1f/%.1f/%.1f C | "
 "Buf=%.0f kB | Anomalies=%d\n",
 t, sched, NUM_TASKS, x[0], x[3], x[4], x[5],
 x[7], anomalies);
 /* Print scheduled task names */
 for (int i = 0; i < NUM_TASKS; i++) {
 if (tasks[i].assigned_slot >= 0)
 FLIGHT_LOG(" %s -> slot %d\n", task_names[tasks[i].id],
 tasks[i].assigned_slot);
 }
 }
 }

 /* ---------- Final summary ---------- */
 FLIGHT_LOG("Total task-slots scheduled : %d (of %d possible)\n",
 total_scheduled, NUM_TIMESTEPS * NUM_TASKS);
 FLIGHT_LOG("Total anomalies detected : %d\n", total_anomalies);
 FLIGHT_LOG("Attitude mode switches : %d\n", total_mode_switches);
 FLIGHT_LOG("Sun exclusion violations : %d\n", sun_exclusion_violations);
 FLIGHT_LOG("Final attitude mode : %s\n", att_mode_names[current_att_mode]);
 FLIGHT_LOG("Final attitude quaternion : [%.4f, %.4f, %.4f, %.4f]\n",
 q_spacecraft.w, q_spacecraft.x, q_spacecraft.y, q_spacecraft.z);
 FLIGHT_LOG("Final SOC: %.1f%% | Buffer: %.0f kB | Attitude err: %.4f rad\n",
 x[0], x[7], sw_sqrt(x[12]*x[12] + x[13]*x[13] + x[14]*x[14]));
 FLIGHT_LOG("Final orbit error: %.3f km\n",
 sw_sqrt(x[15]*x[15] + x[16]*x[16] + x[17]*x[17]));

 /* Final anomaly analysis */
 if (NUM_TIMESTEPS >= FFT_LEN) {
 double final_anom[NUM_RESOURCES];
 int final_det = detect_telemetry_anomaly(telem_history, NUM_TIMESTEPS,
 final_anom);
 FLIGHT_LOG("Final anomaly scores: ");
 const char *res_names[NUM_RESOURCES] = {
 "Power", "Thermal", "DataRate", "Memory", "Attitude", "Orbit"
 };
 for (int r = 0; r < NUM_RESOURCES; r++)
 FLIGHT_LOG("%s=%.2f ", res_names[r], final_anom[r]);
 FLIGHT_LOG("(%d flagged)\n", final_det);
 }

 return 0;
}
