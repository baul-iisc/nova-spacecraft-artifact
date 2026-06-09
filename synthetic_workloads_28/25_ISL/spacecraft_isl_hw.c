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
 * Inter-Satellite Link Management Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements inter-satellite link (ISL) acquisition, tracking, and communication
 * management. This workload computes antenna pointing geometry using orbital
 * mechanics, FFT-based Doppler compensation, link budget matrices, and signal
 * quality analysis. Applicable to navigation satellite constellation cross-links, the
 * tracking and data relay satellite system, and future LEO broadband communication
 * satellite constellations with inter-satellite mesh connectivity.
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
/* AME 3×3 tile matrix accelerator */

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
/* Redirect double trig to CORDIC hardware */
#define sin(x) hw_sin(x)
#define cos(x) hw_cos(x)
#define atan2(y,x) hw_atan2(y,x)

/* ---------- Configuration ---------- */
#define STATE_DIM 18 /* Relative orbit state (6) + attitude (6) +
 link params (3) + Doppler/timing (3) */
#define NUM_NEIGHBOURS 6 /* Number of visible neighbouring satellites */
#define ANTENNA_ELEMENTS 9 /* 3x3 phased array elements */
#define FFT_LEN 64 /* DFT length for Doppler estimation */
#define NUM_TIMESTEPS 50 /* Simulation timesteps */
#define DT 10.0 /* Update interval (seconds) */

/* Dormand-Prince RK45 adaptive step control */
#define DP45_TOL 1e-9 /* Truncation error tolerance (m) */
#define DP45_SAFETY 0.9 /* Safety factor for step adjustment */
#define DP45_MIN_STEP 0.1 /* Minimum sub-step (seconds) */
#define DP45_MAX_STEP 30.0 /* Maximum sub-step (seconds) */

/* Wavelet scratch (static pool — MISRA 21.3) */
#define WAVELET_SIGNAL_LEN 32

/* Physical constants */
#define MU_EARTH 3.986004418e14 /* Earth GM (m^3/s^2) */
#define R_EARTH 6378137.0 /* Earth equatorial radius (m) */
#define J2 1.08263e-3 /* Earth J2 oblateness */
#define C_LIGHT 2.99792458e8 /* Speed of light (m/s) */
#define K_BOLTZMANN 1.380649e-23 /* Boltzmann constant (J/K) */
#define FREQ_KA 26.5e9 /* Ka-band centre frequency (Hz) */

/* ------------------------------------------------------------------ */
/* Generic matrix / vector operations (3x3 tile-friendly) */
/* ------------------------------------------------------------------ */

/* AME 3×3 tiled SYS_MMACD matrix multiply */
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

/* ---- AME 3×3 tile inverse helpers ---- */
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

/* AME tiled matmul for Schur complement inner multiplies */
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

/* AME Schur complement recursive inverse */
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

static double vec_norm(const double *v, int n)
{
 double s = 0.0;
 for (int i = 0; i < n; i++) s += v[i] * v[i];
 return sqrt(s);
}

static double vec_dot(const double *a, const double *b, int n)
{
 double s = 0.0;
 for (int i = 0; i < n; i++) s += a[i] * b[i];
 return s;
}

__attribute__((unused))
static void vec_cross(const double a[3], const double b[3], double c[3])
{
 c[0] = a[1] * b[2] - a[2] * b[1];
 c[1] = a[2] * b[0] - a[0] * b[2];
 c[2] = a[0] * b[1] - a[1] * b[0];
}

/* ------------------------------------------------------------------ */
/* Orbital mechanics: J2-perturbed two-body propagation */
/* ------------------------------------------------------------------ */

/**
 * Compute gravitational acceleration with J2 perturbation.
 * pos: ECI position [m], acc: output acceleration [m/s^2]
 */
static void j2_acceleration(const double pos[3], double acc[3])
{
 double r = vec_norm(pos, 3);
 double r2 = r * r;
 double z2_r2 = (pos[2] * pos[2]) / r2;

 double mu_r3 = MU_EARTH / (r2 * r);
 double j2_factor = 1.5 * J2 * (R_EARTH * R_EARTH) / r2;

 acc[0] = -mu_r3 * pos[0] * (1.0 + j2_factor * (1.0 - 5.0 * z2_r2));
 acc[1] = -mu_r3 * pos[1] * (1.0 + j2_factor * (1.0 - 5.0 * z2_r2));
 acc[2] = -mu_r3 * pos[2] * (1.0 + j2_factor * (3.0 - 5.0 * z2_r2));
}

/**
 * RK4 orbit propagation for dt seconds.
 * state = [x, y, z, vx, vy, vz]
 */
static void propagate_orbit_rk4(double state[6], double dt)
{
 double k1[6], k2[6], k3[6], k4[6];
 double tmp[6], acc[3];

 /* k1 */
 j2_acceleration(state, acc);
 k1[0] = state[3]; k1[1] = state[4]; k1[2] = state[5];
 k1[3] = acc[0]; k1[4] = acc[1]; k1[5] = acc[2];

 /* k2 */
 for (int i = 0; i < 6; i++) tmp[i] = state[i] + 0.5 * dt * k1[i];
 j2_acceleration(tmp, acc);
 k2[0] = tmp[3]; k2[1] = tmp[4]; k2[2] = tmp[5];
 k2[3] = acc[0]; k2[4] = acc[1]; k2[5] = acc[2];

 /* k3 */
 for (int i = 0; i < 6; i++) tmp[i] = state[i] + 0.5 * dt * k2[i];
 j2_acceleration(tmp, acc);
 k3[0] = tmp[3]; k3[1] = tmp[4]; k3[2] = tmp[5];
 k3[3] = acc[0]; k3[4] = acc[1]; k3[5] = acc[2];

 /* k4 */
 for (int i = 0; i < 6; i++) tmp[i] = state[i] + dt * k3[i];
 j2_acceleration(tmp, acc);
 k4[0] = tmp[3]; k4[1] = tmp[4]; k4[2] = tmp[5];
 k4[3] = acc[0]; k4[4] = acc[1]; k4[5] = acc[2];

 for (int i = 0; i < 6; i++)
 state[i] += dt / 6.0 * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
}

/**
 * Dormand-Prince RK4(5) adaptive-step orbit propagation.
 * Integrates state for exactly `dt_total` seconds using internal sub-steps
 * whose size is controlled by the local truncation error estimate.
 *
 * The 4th- and 5th-order solutions share 6 function evaluations (FSAL),
 * and the difference gives the embedded error estimate.
 */
static void propagate_orbit_dp45(double state[6], double dt_total)
{
 /* Dormand-Prince Butcher tableau (only the coefficients we need) */
 static const double a2 = 1.0/5.0;
 /* b coefficients for 5th-order solution */
 static const double b51 = 35.0/384.0;
 static const double b53 = 500.0/1113.0;
 static const double b54 = 125.0/192.0;
 static const double b55 = -2187.0/6784.0;
 static const double b56 = 11.0/84.0;
 /* e = b5 - b4 (error coefficients) */
 static const double e1 = 71.0/57600.0;
 static const double e3 = -71.0/16695.0;
 static const double e4 = 71.0/1920.0;
 static const double e5 = -17253.0/339200.0;
 static const double e6 = 22.0/525.0;
 static const double e7 = -1.0/40.0;

 double t_done = 0.0;
 double h = dt_total; /* initial step = full interval */
 if (h > DP45_MAX_STEP) h = DP45_MAX_STEP;

 while (t_done < dt_total) {
 /* Clamp step so we land exactly on dt_total */
 if (t_done + h > dt_total) h = dt_total - t_done;
 if (h < DP45_MIN_STEP) h = DP45_MIN_STEP;

 double k1[6], k2[6], k3[6], k4[6], k5[6], k6[6], k7[6];
 double tmp[6], acc[3];

 /* k1 */
 j2_acceleration(state, acc);
 for (int i = 0; i < 3; i++) { k1[i] = state[3+i]; k1[3+i] = acc[i]; }

 /* k2 */
 for (int i = 0; i < 6; i++) tmp[i] = state[i] + h * a2 * k1[i];
 j2_acceleration(tmp, acc);
 for (int i = 0; i < 3; i++) { k2[i] = tmp[3+i]; k2[3+i] = acc[i]; }

 /* k3 */
 for (int i = 0; i < 6; i++)
 tmp[i] = state[i] + h * (3.0/40.0 * k1[i] + 9.0/40.0 * k2[i]);
 j2_acceleration(tmp, acc);
 for (int i = 0; i < 3; i++) { k3[i] = tmp[3+i]; k3[3+i] = acc[i]; }

 /* k4 */
 for (int i = 0; i < 6; i++)
 tmp[i] = state[i] + h * (44.0/45.0 * k1[i] - 56.0/15.0 * k2[i]
 + 32.0/9.0 * k3[i]);
 j2_acceleration(tmp, acc);
 for (int i = 0; i < 3; i++) { k4[i] = tmp[3+i]; k4[3+i] = acc[i]; }

 /* k5 */
 for (int i = 0; i < 6; i++)
 tmp[i] = state[i] + h * (19372.0/6561.0 * k1[i]
 - 25360.0/2187.0 * k2[i]
 + 64448.0/6561.0 * k3[i]
 - 212.0/729.0 * k4[i]);
 j2_acceleration(tmp, acc);
 for (int i = 0; i < 3; i++) { k5[i] = tmp[3+i]; k5[3+i] = acc[i]; }

 /* k6 */
 for (int i = 0; i < 6; i++)
 tmp[i] = state[i] + h * (9017.0/3168.0 * k1[i]
 - 355.0/33.0 * k2[i]
 + 46732.0/5247.0 * k3[i]
 + 49.0/176.0 * k4[i]
 - 5103.0/18656.0 * k5[i]);
 j2_acceleration(tmp, acc);
 for (int i = 0; i < 3; i++) { k6[i] = tmp[3+i]; k6[3+i] = acc[i]; }

 /* 5th-order solution (for state update) */
 double y5[6];
 for (int i = 0; i < 6; i++)
 y5[i] = state[i] + h * (b51*k1[i] + b53*k3[i] + b54*k4[i]
 + b55*k5[i] + b56*k6[i]);

 /* k7 (FSAL: evaluated at the 5th-order result) */
 j2_acceleration(y5, acc);
 for (int i = 0; i < 3; i++) { k7[i] = y5[3+i]; k7[3+i] = acc[i]; }

 /* Error estimate: ||e|| = max_i |h * (e1*k1 + e3*k3 + ... + e7*k7)| */
 double err_max = 0.0;
 for (int i = 0; i < 6; i++) {
 double ei = fabs(h * (e1*k1[i] + e3*k3[i] + e4*k4[i]
 + e5*k5[i] + e6*k6[i] + e7*k7[i]));
 if (ei > err_max) err_max = ei;
 }

 if (err_max < 1e-30) err_max = 1e-30; /* Avoid div-by-zero */

 if (err_max <= DP45_TOL || h <= DP45_MIN_STEP) {
 /* Accept step */
 memcpy(state, y5, 6 * sizeof(double));
 t_done += h;

 /* Grow step for next iteration */
 double factor = DP45_SAFETY * pow(DP45_TOL / err_max, 0.2);
 if (factor > 5.0) factor = 5.0;
 h *= factor;
 if (h > DP45_MAX_STEP) h = DP45_MAX_STEP;
 } else {
 /* Reject step: shrink and retry */
 double factor = DP45_SAFETY * pow(DP45_TOL / err_max, 0.25);
 if (factor < 0.2) factor = 0.2;
 h *= factor;
 if (h < DP45_MIN_STEP) h = DP45_MIN_STEP;
 }
 }
}

/**
 * Compute State Transition Matrix (STM) via numerical differentiation.
 * STM is 6x6, representing sensitivity of final state to initial state.
 */
static void compute_orbit_stm(const double state0[6], double dt,
 double STM[36])
{
 double eps = 1.0; /* perturbation for position / velocity */
 double state_p[6], state_m[6];

 for (int j = 0; j < 6; j++) {
 double ej = (j < 3) ? eps : eps * 1e-3;

 memcpy(state_p, state0, 6 * sizeof(double));
 memcpy(state_m, state0, 6 * sizeof(double));
 state_p[j] += ej;
 state_m[j] -= ej;

 propagate_orbit_rk4(state_p, dt);
 propagate_orbit_rk4(state_m, dt);

 for (int i = 0; i < 6; i++)
 STM[i * 6 + j] = (state_p[i] - state_m[i]) / (2.0 * ej);
 }
}

/* ------------------------------------------------------------------ */
/* Direction Cosine Matrix (DCM) operations */
/* ------------------------------------------------------------------ */

/**
 * Build DCM from body frame to ECI using Euler angles (3-2-1 sequence).
 */
static void euler_to_dcm(double roll, double pitch, double yaw,
 double dcm[3][3])
{
 double cr = cos(roll), sr = sin(roll);
 double cp = cos(pitch), sp = sin(pitch);
 double cy = cos(yaw), sy = sin(yaw);

 dcm[0][0] = cp * cy;
 dcm[0][1] = cp * sy;
 dcm[0][2] = -sp;
 dcm[1][0] = sr * sp * cy - cr * sy;
 dcm[1][1] = sr * sp * sy + cr * cy;
 dcm[1][2] = sr * cp;
 dcm[2][0] = cr * sp * cy + sr * sy;
 dcm[2][1] = cr * sp * sy - sr * cy;
 dcm[2][2] = cr * cp;
}

/**
 * Apply 3x3 DCM to a vector: v_out = DCM * v_in
 */
static void dcm_rotate(const double dcm[3][3], const double v_in[3],
 double v_out[3])
{
 for (int i = 0; i < 3; i++) {
 v_out[i] = 0.0;
 for (int j = 0; j < 3; j++)
 v_out[i] += dcm[i][j] * v_in[j];
 }
}

/* ------------------------------------------------------------------ */
/* Phased array beam steering */
/* ------------------------------------------------------------------ */

/**
 * Compute 3x3 beam weight matrix for a planar phased array.
 * az, el: desired beam direction in azimuth/elevation (rad).
 * lambda: wavelength (m).
 * d: element spacing (m), nominally lambda/2.
 *
 * Each element weight w[i][j] = exp(j * phase_shift) where phase_shift
 * depends on the element position and desired beam direction.
 */
static void compute_beam_weights(double az, double el, double lambda,
 double d, double w_re[3][3],
 double w_im[3][3])
{
 double kx = 2.0 * M_PI / lambda * sin(el) * cos(az);
 double ky = 2.0 * M_PI / lambda * sin(el) * sin(az);

 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 double phase = kx * (i - 1) * d + ky * (j - 1) * d;
 w_re[i][j] = cos(phase);
 w_im[i][j] = sin(phase);
 }
 }
}

/**
 * Compute array factor (antenna gain pattern) for a given direction.
 * Returns normalised gain in dB.
 */
static double compute_array_factor(const double w_re[3][3],
 const double w_im[3][3],
 double theta, double phi,
 double lambda, double d)
{
 double kx = 2.0 * M_PI / lambda * sin(theta) * cos(phi);
 double ky = 2.0 * M_PI / lambda * sin(theta) * sin(phi);

 double sum_re = 0.0, sum_im = 0.0;
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 double phase = kx * (i - 1) * d + ky * (j - 1) * d;
 double e_re = cos(phase), e_im = sin(phase);
 /* Complex multiply: w * e */
 sum_re += w_re[i][j] * e_re - w_im[i][j] * e_im;
 sum_im += w_re[i][j] * e_im + w_im[i][j] * e_re;
 }
 }
 double power = (sum_re * sum_re + sum_im * sum_im) /
 (ANTENNA_ELEMENTS * ANTENNA_ELEMENTS);
 return 10.0 * log10(power + 1e-30);
}

/* ------------------------------------------------------------------ */
/* Link budget computation */
/* ------------------------------------------------------------------ */

typedef struct {
 double range_m; /* Slant range (m) */
 double tx_power_dBW; /* Transmit power (dBW) */
 double tx_gain_dB; /* Transmit antenna gain (dB) */
 double rx_gain_dB; /* Receive antenna gain (dB) */
 double frequency_Hz; /* Carrier frequency (Hz) */
 double bandwidth_Hz; /* Channel bandwidth (Hz) */
 double noise_temp_K; /* Receiver noise temperature (K) */
 double data_rate_bps; /* Required data rate (bps) */
} LinkParams;

static double compute_fspl(double range_m, double freq_Hz)
{
 /* Free-Space Path Loss (dB) */
 return 20.0 * log10(range_m) + 20.0 * log10(freq_Hz)
 + 20.0 * log10(4.0 * M_PI / C_LIGHT);
}

static double compute_link_margin(const LinkParams *lp, double beam_gain_dB)
{
 double fspl = compute_fspl(lp->range_m, lp->frequency_Hz);
 double EIRP = lp->tx_power_dBW + lp->tx_gain_dB + beam_gain_dB;
 double rx_power = EIRP - fspl + lp->rx_gain_dB;

 /* Noise power */
 double N0_dBW = 10.0 * log10(K_BOLTZMANN * lp->noise_temp_K);
 double noise_dBW = N0_dBW + 10.0 * log10(lp->bandwidth_Hz);

 double SNR_dB = rx_power - noise_dBW;

 /* Required Eb/N0 for QPSK ~9.6 dB at BER=1e-6 */
 double required_EbN0 = 9.6;
 double actual_EbN0 = SNR_dB - 10.0 * log10(lp->data_rate_bps / lp->bandwidth_Hz);

 return actual_EbN0 - required_EbN0; /* Link margin (dB) */
}

/* ------------------------------------------------------------------ */
/* Doppler estimation via DFT */
/* ------------------------------------------------------------------ */

/**
 * Estimate Doppler shift from received carrier samples.
 * Returns estimated Doppler frequency in Hz.
 */
static double estimate_doppler_dft(const double *samples, int N,
 double sample_rate)
{
 double mag[FFT_LEN / 2 + 1];
 int Nuse = (N < FFT_LEN) ? N : FFT_LEN;

 /* DFT */
 for (int k = 0; k <= Nuse / 2; k++) {
 double re = 0.0, im = 0.0;
 for (int n = 0; n < Nuse; n++) {
 double angle = 2.0 * M_PI * k * n / Nuse;
 re += samples[n] * cos(angle);
 im -= samples[n] * sin(angle);
 }
 mag[k] = sqrt(re * re + im * im);
 }

 /* Find peak */
 int peak_k = 1;
 double peak_val = 0.0;
 for (int k = 1; k <= Nuse / 2; k++) {
 if (mag[k] > peak_val) {
 peak_val = mag[k];
 peak_k = k;
 }
 }

 return (double)peak_k * sample_rate / Nuse;
}

/* ------------------------------------------------------------------ */
/* Relative orbit & range computation */
/* ------------------------------------------------------------------ */

/**
 * Compute relative position, velocity, range, and range rate between
 * the host satellite and a neighbour.
 */
static void compute_relative_state(const double host[6],
 const double neighbour[6],
 double rel_pos[3], double rel_vel[3],
 double *range, double *range_rate)
{
 for (int i = 0; i < 3; i++) {
 rel_pos[i] = neighbour[i] - host[i];
 rel_vel[i] = neighbour[i + 3] - host[i + 3];
 }
 *range = vec_norm(rel_pos, 3);
 if (*range > 0.0)
 *range_rate = vec_dot(rel_pos, rel_vel, 3) / (*range);
 else
 *range_rate = 0.0;
}

/**
 * Check Earth-body blockage: returns 1 if line-of-sight is blocked.
 */
static int is_earth_blocked(const double pos_a[3], const double pos_b[3])
{
 /* Parametric closest approach of line to Earth centre */
 double d[3];
 for (int i = 0; i < 3; i++) d[i] = pos_b[i] - pos_a[i];
 double t = -vec_dot(pos_a, d, 3) / (vec_dot(d, d, 3) + 1e-30);
 if (t < 0.0) t = 0.0;
 if (t > 1.0) t = 1.0;
 double closest[3];
 for (int i = 0; i < 3; i++) closest[i] = pos_a[i] + t * d[i];
 return (vec_norm(closest, 3) < R_EARTH) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* EKF for link state estimation */
/* ------------------------------------------------------------------ */

/**
 * State vector layout (18 elements):
 * [0..2] Relative position (RTN frame, m)
 * [3..5] Relative velocity (RTN frame, m/s)
 * [6..8] Host attitude errors (Euler angles, rad)
 * [9..11] Link quality: SNR, BER_log, margin (dB)
 * [12..14] Doppler: shift_Hz, rate_Hz/s, jerk_Hz/s^2
 * [15..17] Timing: propagation_delay_ns, clock_offset_ns, drift_ns/s
 */

static void link_state_predict(double x[STATE_DIM], double dt)
{
 /* Relative orbit: CW (Hill) equations linearised */
 double n = sqrt(MU_EARTH / pow(R_EARTH + 550e3, 3)); /* Mean motion at 550 km */
 double x0 = x[0], y0 = x[1], z0 = x[2];
 double vx0 = x[3], vy0 = x[4], vz0 = x[5];

 /* Hill's equations solution */
 double sn = sin(n * dt), cn = cos(n * dt);
 x[0] = (4.0 - 3.0 * cn) * x0 + sn / n * vx0 + 2.0 / n * (1.0 - cn) * vy0;
 x[1] = 6.0 * (sn - n * dt) * x0 + y0 + 2.0 / n * (cn - 1.0) * vx0
 + (4.0 * sn - 3.0 * n * dt) / n * vy0;
 x[2] = z0 * cn + vz0 / n * sn;
 x[3] = 3.0 * n * sn * x0 + cn * vx0 + 2.0 * sn * vy0;
 x[4] = 6.0 * n * (cn - 1.0) * x0 - 2.0 * sn * vx0 + (4.0 * cn - 3.0) * vy0;
 x[5] = -z0 * n * sn + vz0 * cn;

 /* Attitude: small angle drift */
 for (int i = 0; i < 3; i++)
 x[6 + i] *= 0.98; /* Slow convergence */

 /* Link quality: slow variation */
 x[9] += 0.01 * sin(n * dt); /* SNR fluctuation */
 x[10] *= 1.001; /* BER degradation */
 x[11] = x[9] - 9.6; /* Margin = SNR - required */

 /* Doppler: kinematic model */
 x[12] += x[13] * dt + 0.5 * x[14] * dt * dt;
 x[13] += x[14] * dt;

 /* Timing: drift model */
 x[15] = vec_norm(x, 3) / C_LIGHT * 1e9; /* Propagation delay (ns) */
 x[16] += x[17] * dt;
}

/**
 * Build Jacobian for link state (18x18).
 */
static void build_link_jacobian(const double x[STATE_DIM] __attribute__((unused)),
 double dt,
 double F[STATE_DIM * STATE_DIM])
{
 memset(F, 0, STATE_DIM * STATE_DIM * sizeof(double));

 double n = sqrt(MU_EARTH / pow(R_EARTH + 550e3, 3));
 double sn = sin(n * dt), cn = cos(n * dt);

 /* CW equations partial derivatives (6x6 block) */
 F[0 * STATE_DIM + 0] = 4.0 - 3.0 * cn;
 F[0 * STATE_DIM + 3] = sn / n;
 F[0 * STATE_DIM + 4] = 2.0 / n * (1.0 - cn);
 F[1 * STATE_DIM + 0] = 6.0 * (sn - n * dt);
 F[1 * STATE_DIM + 1] = 1.0;
 F[1 * STATE_DIM + 3] = 2.0 / n * (cn - 1.0);
 F[1 * STATE_DIM + 4] = (4.0 * sn - 3.0 * n * dt) / n;
 F[2 * STATE_DIM + 2] = cn;
 F[2 * STATE_DIM + 5] = sn / n;
 F[3 * STATE_DIM + 0] = 3.0 * n * sn;
 F[3 * STATE_DIM + 3] = cn;
 F[3 * STATE_DIM + 4] = 2.0 * sn;
 F[4 * STATE_DIM + 0] = 6.0 * n * (cn - 1.0);
 F[4 * STATE_DIM + 3] = -2.0 * sn;
 F[4 * STATE_DIM + 4] = 4.0 * cn - 3.0;
 F[5 * STATE_DIM + 2] = -n * sn;
 F[5 * STATE_DIM + 5] = cn;

 /* Attitude block */
 for (int i = 6; i < 9; i++)
 F[i * STATE_DIM + i] = 0.98;

 /* Link quality */
 F[9 * STATE_DIM + 9] = 1.0;
 F[10 * STATE_DIM + 10] = 1.001;
 F[11 * STATE_DIM + 9] = 1.0;
 F[11 * STATE_DIM + 11] = 0.0;

 /* Doppler */
 F[12 * STATE_DIM + 12] = 1.0;
 F[12 * STATE_DIM + 13] = dt;
 F[12 * STATE_DIM + 14] = 0.5 * dt * dt;
 F[13 * STATE_DIM + 13] = 1.0;
 F[13 * STATE_DIM + 14] = dt;
 F[14 * STATE_DIM + 14] = 1.0;

 /* Timing */
 F[15 * STATE_DIM + 15] = 0.0; /* Recomputed from position */
 F[16 * STATE_DIM + 16] = 1.0;
 F[16 * STATE_DIM + 17] = dt;
 F[17 * STATE_DIM + 17] = 1.0;
}

/**
 * EKF covariance prediction.
 */
static void ekf_predict_covariance(double P[STATE_DIM * STATE_DIM],
 const double F[STATE_DIM * STATE_DIM],
 const double Q[STATE_DIM * STATE_DIM])
{
 double FP[STATE_DIM * STATE_DIM];
 double FT[STATE_DIM * STATE_DIM];
 double FPFT[STATE_DIM * STATE_DIM];

 mat_mul(F, P, FP, STATE_DIM, STATE_DIM, STATE_DIM);
 mat_transpose(F, FT, STATE_DIM, STATE_DIM);
 mat_mul(FP, FT, FPFT, STATE_DIM, STATE_DIM, STATE_DIM);
 mat_add(FPFT, Q, P, STATE_DIM, STATE_DIM);
}

/**
 * EKF measurement update.
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
/* Handover decision */
/* ------------------------------------------------------------------ */

typedef struct {
 int id;
 double state[6]; /* ECI position + velocity */
 double range;
 double range_rate;
 double link_margin_dB;
 double doppler_Hz;
 int visible;
 double handover_score;
} Neighbour;

static double rand_gauss(void)
{
 double u1 = (rand() / (double)RAND_MAX);
 double u2 = (rand() / (double)RAND_MAX);
 if (u1 < 1e-15) u1 = 1e-15;
 return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ------------------------------------------------------------------ */
/* BCE: Wavelet (9%) — Haar lifting wavelet for Doppler analysis */
/* ------------------------------------------------------------------ */

#define WAVELET_SIGNAL_LEN 32 /* Power-of-2, fits ISL signal buffers */

static double wav_scratch[WAVELET_SIGNAL_LEN]; /* Static scratch (MISRA 21.3) */

/**
 * In-place 1-D Haar lifting wavelet decomposition (one level).
 * After: data[0..len/2-1] = approximation, data[len/2..len-1] = detail.
 */
static void haar_lifting_decompose(double *data, int len)
{
 if (len < 2) return;
 int half = len / 2;
 double *tmp = wav_scratch; /* Static BSS scratch (MISRA 21.3) */

 /* Split into even / odd */
 for (int i = 0; i < half; i++) {
 tmp[i] = data[2 * i]; /* even (approx) */
 tmp[half + i] = data[2 * i + 1]; /* odd (detail) */
 }

 /* Predict: detail -= approx */
 for (int i = 0; i < half; i++)
 tmp[half + i] -= tmp[i];

 /* Update: approx += detail / 2 */
 for (int i = 0; i < half; i++)
 tmp[i] += tmp[half + i] * 0.5;

 memcpy(data, tmp, len * sizeof(double));
}

/**
 * In-place 1-D Haar lifting wavelet reconstruction (one level).
 */
__attribute__((unused))
static void haar_lifting_reconstruct(double *data, int len)
{
 if (len < 2) return;
 int half = len / 2;
 double *tmp = wav_scratch; /* Static BSS scratch (MISRA 21.3) */

 /* Undo update */
 for (int i = 0; i < half; i++)
 data[i] -= data[half + i] * 0.5;

 /* Undo predict */
 for (int i = 0; i < half; i++)
 data[half + i] += data[i];

 /* Merge even / odd back */
 for (int i = 0; i < half; i++) {
 tmp[2 * i] = data[i];
 tmp[2 * i + 1] = data[half + i];
 }
 memcpy(data, tmp, len * sizeof(double));
}

/**
 * Multi-resolution Doppler estimation using wavelet coefficients.
 * Decomposes a Doppler-shift time series into 3 levels and estimates
 * the dominant Doppler at each scale from the detail energy.
 *
 * Returns the estimated Doppler frequency from the finest scale.
 */
static double wavelet_doppler_estimate(const double *doppler_series,
 int len, double sample_rate)
{
 double *buf = wav_scratch; /* Static BSS scratch (MISRA 21.3) */
 memcpy(buf, doppler_series, len * sizeof(double));

 double scale_energy[4] = {0}; /* Up to 3 detail levels + approx */
 int cur = len;

 /* Forward DWT — 3 levels */
 for (int lev = 0; lev < 3 && cur >= 2; lev++) {
 haar_lifting_decompose(buf, cur);
 /* Energy of detail coefficients at this level */
 int half = cur / 2;
 for (int i = half; i < cur; i++)
 scale_energy[lev] += buf[i] * buf[i];
 scale_energy[lev] /= half;
 cur /= 2;
 }
 /* Approximation energy */
 for (int i = 0; i < cur; i++)
 scale_energy[3] += buf[i] * buf[i];
 scale_energy[3] /= cur;

 /* Find dominant scale */
 int dom = 0;
 double max_e = scale_energy[0];
 for (int i = 1; i < 4; i++) {
 if (scale_energy[i] > max_e) {
 max_e = scale_energy[i];
 dom = i;
 }
 }

 /* Frequency band centre for the dominant scale:
 * Level 0 detail: [fs/4, fs/2], Level 1: [fs/8, fs/4], etc. */
 double fs = sample_rate;
 double f_est;
 if (dom < 3)
 f_est = fs / pow(2.0, dom + 2) * 1.5; /* Centre of band */
 else
 f_est = fs / pow(2.0, 4); /* Lowest approx band */

 return f_est;
}

/* ------------------------------------------------------------------ */
/* BCE: ECC (9%) — Hamming(7,4) block encoder / decoder */
/* ------------------------------------------------------------------ */

/*
 * Hamming(7,4) parity-check and generator matrices.
 * G = [I_4 | P^T], H = [P | I_3]
 * P = | 1 1 0 | (rows correspond to data bits d0..d3)
 * | 0 1 1 |
 * | 1 0 1 |
 * | 1 1 1 |
 */

/**
 * Encode a 4-bit nibble into a 7-bit Hamming codeword.
 * data: 4 data bits (LSB-first in bits 0..3 of uint8_t).
 * Returns 7-bit codeword in bits 0..6.
 */
static uint8_t hamming74_encode(uint8_t data)
{
 uint8_t d0 = (data >> 0) & 1;
 uint8_t d1 = (data >> 1) & 1;
 uint8_t d2 = (data >> 2) & 1;
 uint8_t d3 = (data >> 3) & 1;

 /* Parity bits: p0 = d0^d2^d3, p1 = d0^d1^d3, p2 = d1^d2^d3 */
 uint8_t p0 = d0 ^ d2 ^ d3;
 uint8_t p1 = d0 ^ d1 ^ d3;
 uint8_t p2 = d1 ^ d2 ^ d3;

 /* Codeword: [d0 d1 d2 d3 p0 p1 p2] */
 return (d0) | (d1 << 1) | (d2 << 2) | (d3 << 3)
 | (p0 << 4) | (p1 << 5) | (p2 << 6);
}

/**
 * Compute 3-bit syndrome for a 7-bit received word.
 * Non-zero syndrome indicates an error; its value encodes the error position.
 */
static uint8_t hamming74_syndrome(uint8_t codeword)
{
 uint8_t b[7];
 for (int i = 0; i < 7; i++)
 b[i] = (codeword >> i) & 1;

 /* Syndrome: s = H * r^T */
 uint8_t s0 = b[0] ^ b[2] ^ b[3] ^ b[4];
 uint8_t s1 = b[0] ^ b[1] ^ b[3] ^ b[5];
 uint8_t s2 = b[1] ^ b[2] ^ b[3] ^ b[6];

 return (s0) | (s1 << 1) | (s2 << 2);
}

/**
 * Decode a 7-bit Hamming codeword, correcting up to 1 bit error.
 * Returns the corrected 4-bit data nibble.
 */
static uint8_t hamming74_decode(uint8_t codeword)
{
 uint8_t syndrome = hamming74_syndrome(codeword);

 if (syndrome != 0) {
 /* Syndrome encodes the 1-indexed error position in the column
 * ordering of H = [P | I_3].
 * Map syndrome to bit position:
 * syndrome 1(001)->bit4(p0), 2(010)->bit5(p1), 3(011)->bit0(d0),
 * 4(100)->bit6(p2), 5(101)->bit2(d2), 6(110)->bit1(d1), 7(111)->bit3(d3)
 */
 static const int pos_map[8] = {-1, 4, 5, 0, 6, 2, 1, 3};
 int epos = pos_map[syndrome & 0x07];
 if (epos >= 0 && epos < 7)
 codeword ^= (1 << epos); /* Correct single-bit error */
 }

 return codeword & 0x0F; /* Extract data bits [d0..d3] */
}

/**
 * Encode a block of bytes using Hamming(7,4).
 * Each input byte yields two 7-bit codewords (low nibble + high nibble),
 * packed into 2 output bytes (with 2 unused bits).
 *
 * data_in : input buffer (data_len bytes)
 * code_out: output buffer (must hold at least 2 * data_len bytes)
 * Returns the number of codeword bytes written.
 */
static int ecc_encode_block(const uint8_t *data_in, int data_len,
 uint8_t *code_out)
{
 int out_idx = 0;
 for (int i = 0; i < data_len; i++) {
 uint8_t lo = data_in[i] & 0x0F;
 uint8_t hi = (data_in[i] >> 4) & 0x0F;
 code_out[out_idx++] = hamming74_encode(lo);
 code_out[out_idx++] = hamming74_encode(hi);
 }
 return out_idx;
}

/**
 * Decode a Hamming(7,4)-coded block with error correction.
 * code_in : encoded codewords (code_len bytes, pairs per original byte)
 * data_out: recovered data (code_len / 2 bytes)
 * Returns corrected error count.
 */
static int ecc_decode_block(const uint8_t *code_in, int code_len,
 uint8_t *data_out)
{
 int errors_corrected = 0;
 int out_idx = 0;
 for (int i = 0; i + 1 < code_len; i += 2) {
 /* Check and correct each codeword */
 uint8_t s_lo = hamming74_syndrome(code_in[i]);
 uint8_t s_hi = hamming74_syndrome(code_in[i + 1]);
 if (s_lo) errors_corrected++;
 if (s_hi) errors_corrected++;

 uint8_t lo = hamming74_decode(code_in[i]);
 uint8_t hi = hamming74_decode(code_in[i + 1]);
 data_out[out_idx++] = lo | (hi << 4);
 }
 return errors_corrected;
}

/* ------------------------------------------------------------------ */
/* BCE: ECC outer layer — RS(15,11) over GF(2^4) */
/* Provides burst-error resilience for high-speed Ka-band ISL links. */
/* RS(15,11) can correct up to t=2 symbol errors per 15-symbol block, */
/* where each symbol is a 4-bit nibble ∈ GF(16). */
/* ------------------------------------------------------------------ */

/**
 * GF(2^4) arithmetic with primitive polynomial x^4 + x + 1 (0x13).
 * Field elements are 0..15.
 */
static const uint8_t gf16_exp[32] = { /* α^i for i=0..14, then wraps */
 1,2,4,8,3,6,12,11,5,10,7,14,15,13,9, /* α^0..α^14 */
 1,2,4,8,3,6,12,11,5,10,7,14,15,13,9 /* repeat */
};
static const uint8_t gf16_log[16] = { /* log_α(x) for x=0..15; log(0)=255 */
 255, 0,1,4,2,8,5,10,3,14,9,7,6,13,11,12
};

static uint8_t gf16_mul(uint8_t a, uint8_t b)
{
 if (a == 0 || b == 0) return 0;
 return gf16_exp[gf16_log[a] + gf16_log[b]];
}

static uint8_t gf16_inv(uint8_t a)
{
 if (a == 0) return 0; /* undefined, guard */
 return gf16_exp[15 - gf16_log[a]]; /* α^(15-log(a)) mod 15 = α^(-log(a)) */
}

/* Generator polynomial for RS(15,11): g(x) = Π_{i=1..4} (x − α^i)
 * Precomputed coefficients g[0..4] (g[4]=1 leading, stored implicitly) */
static const uint8_t rs_gen[4] = {6, 9, 6, 4}; /* g0..g3 in GF(16) */

/**
 * RS(15,11) encoder. Data: 11 symbols in sym[0..10], parity stored in sym[11..14].
 * All symbols are 4-bit nibbles (0..15).
 */
static void rs1511_encode(uint8_t sym[15])
{
 uint8_t rem[4] = {0, 0, 0, 0};
 for (int i = 0; i < 11; i++) {
 uint8_t feedback = sym[i] ^ rem[3];
 rem[3] = rem[2] ^ gf16_mul(feedback, rs_gen[3]);
 rem[2] = rem[1] ^ gf16_mul(feedback, rs_gen[2]);
 rem[1] = rem[0] ^ gf16_mul(feedback, rs_gen[1]);
 rem[0] = gf16_mul(feedback, rs_gen[0]);
 }
 sym[11] = rem[3];
 sym[12] = rem[2];
 sym[13] = rem[1];
 sym[14] = rem[0];
}

/**
 * Compute 4 syndromes S_j = Σ sym[i] · α^(i·j) for j=1..4.
 */
static void rs1511_syndromes(const uint8_t sym[15], uint8_t S[4])
{
 for (int j = 0; j < 4; j++) {
 uint8_t s = 0;
 for (int i = 0; i < 15; i++)
 s ^= gf16_mul(sym[i], gf16_exp[((j + 1) * i) % 15]);
 S[j] = s;
 }
}

/**
 * RS(15,11) decoder — corrects up to 2 symbol errors via Berlekamp-
 * Massey + Chien search + Forney. Returns number of errors corrected
 * (-1 if uncorrectable).
 */
static int rs1511_decode(uint8_t sym[15])
{
 uint8_t S[4];
 rs1511_syndromes(sym, S);

 /* All-zero syndrome → no errors */
 if ((S[0] | S[1] | S[2] | S[3]) == 0) return 0;

 /* --- Berlekamp-Massey (simplified for t=2) --- */
 /* Error locator polynomial Λ(x) = 1 + σ1*x + σ2*x^2 */
 uint8_t sigma1 = 0, sigma2 = 0;

 /* Determinant of the syndrome matrix for 2-error case:
 * Δ = S[0]*S[2] ⊕ S[1]*S[1] (GF add = XOR) */
 uint8_t delta = gf16_mul(S[0], S[2]) ^ gf16_mul(S[1], S[1]);

 if (delta != 0) {
 /* 2 errors */
 uint8_t delta_inv = gf16_inv(delta);
 sigma1 = gf16_mul(gf16_mul(S[0], S[3]) ^ gf16_mul(S[1], S[2]),
 delta_inv);
 sigma2 = gf16_mul(gf16_mul(S[1], S[3]) ^ gf16_mul(S[2], S[2]),
 delta_inv);
 } else if (S[0] != 0) {
 /* 1 error: σ1 = S[1]/S[0], σ2 = 0 */
 sigma1 = gf16_mul(S[1], gf16_inv(S[0]));
 sigma2 = 0;
 } else {
 return -1; /* Uncorrectable */
 }

 /* --- Chien search: find roots of Λ(x) = 1 + σ1·x + σ2·x^2 --- */
 int err_pos[2], nerr = 0;
 for (int i = 0; i < 15 && nerr < 2; i++) {
 uint8_t ai = gf16_exp[i]; /* α^i */
 uint8_t ai2 = gf16_mul(ai, ai); /* α^(2i) */
 uint8_t val = 1 ^ gf16_mul(sigma1, ai) ^ gf16_mul(sigma2, ai2);
 if (val == 0) err_pos[nerr++] = i;
 }

 if (nerr == 0) return -1; /* No root found → uncorrectable */
 if (delta != 0 && nerr != 2) return -1;
 if (delta == 0 && nerr != 1) return -1;

 /* --- Forney: compute error magnitudes --- */
 for (int e = 0; e < nerr; e++) {
 int pos = err_pos[e];
 uint8_t Xi = gf16_exp[pos]; /* error locator root inverse */
 uint8_t Xi_inv = gf16_inv(Xi);

 /* Ω(x) = S[0] + (S[1]+σ1·S[0])·x (truncated for t=2) */
 uint8_t omega0 = S[0];
 uint8_t omega1 = S[1] ^ gf16_mul(sigma1, S[0]);
 uint8_t Omega = omega0 ^ gf16_mul(omega1, Xi_inv);

 /* Λ'(x) = σ1 + 0·x (derivative in char-2, even terms vanish) */
 /* Λ'(Xi_inv) = σ1 for 1-error; for 2-error, = σ1 (since 2σ2 x = 0 in GF(2^4)) */
 uint8_t Lambda_prime = sigma1;
 if (Lambda_prime == 0) return -1; /* degenerate */

 uint8_t e_val = gf16_mul(Omega, gf16_inv(Lambda_prime));
 sym[pos] ^= e_val;
 }

 return nerr;
}

/**
 * Encode a byte block with RS(15,11) outer code.
 * Input : data_in[data_len] bytes
 * Output: rs_out[] — 15-symbol RS blocks (nibble-packed into bytes)
 * Returns total output bytes written.
 *
 * Packing: 11 data symbols = 5.5 bytes → pad to 6 bytes input per block.
 * We pack each RS symbol pair into one byte (lo=sym[even], hi=sym[odd]).
 */
static int rs_encode_block(const uint8_t *data_in, int data_len,
 uint8_t *rs_out)
{
 int out_idx = 0;
 int in_pos = 0;
 while (in_pos < data_len) {
 uint8_t syms[15];
 memset(syms, 0, sizeof(syms));
 /* Extract up to 11 nibbles (= 5.5 bytes) of data */
 int nib = 0;
 while (nib < 11 && in_pos < data_len) {
 syms[nib++] = data_in[in_pos] & 0x0F;
 if (nib < 11)
 syms[nib++] = (data_in[in_pos] >> 4) & 0x0F;
 in_pos++;
 }
 rs1511_encode(syms);

 /* Pack 15 nibbles → 8 bytes (last byte uses only low nibble) */
 for (int i = 0; i < 15; i += 2) {
 uint8_t lo = syms[i];
 uint8_t hi = (i + 1 < 15) ? syms[i + 1] : 0;
 rs_out[out_idx++] = lo | (hi << 4);
 }
 }
 return out_idx;
}

/**
 * Decode RS(15,11)-coded block with error correction.
 * Returns total symbol errors corrected (negative on decode failure in
 * at least one block).
 */
static int rs_decode_block(const uint8_t *rs_in, int rs_len,
 uint8_t *data_out)
{
 int total_corrected = 0;
 int in_pos = 0, out_pos = 0;

 while (in_pos + 7 < rs_len) { /* 8 packed bytes per block */
 /* Unpack 15 nibbles from 8 bytes */
 uint8_t syms[15];
 memset(syms, 0, sizeof(syms));
 int nib = 0;
 for (int b = 0; b < 8 && nib < 15; b++) {
 syms[nib++] = rs_in[in_pos + b] & 0x0F;
 if (nib < 15)
 syms[nib++] = (rs_in[in_pos + b] >> 4) & 0x0F;
 }
 in_pos += 8;

 int ret = rs1511_decode(syms);
 if (ret > 0) total_corrected += ret;

 /* Extract 11 data nibbles → bytes */
 for (int i = 0; i < 11; i += 2) {
 uint8_t lo = syms[i];
 uint8_t hi = (i + 1 < 11) ? syms[i + 1] : 0;
 data_out[out_pos++] = lo | (hi << 4);
 }
 }
 return total_corrected;
}

/**
 * Run ECC verification: encode, inject random bit errors, decode, verify.
 * Uses a block of ISL payload data sized to STATE_DIM * 3 bytes.
 *
 * Inner code: Hamming(7,4) — corrects single-bit SEU per codeword.
 * Outer code: RS(15,11) — corrects burst errors (up to 2 symbol errors
 * per 15-symbol block on the Ka-band ISL channel).
 */
static void run_ecc_verification(int num_trials)
{
 const int data_len = STATE_DIM * 3; /* 54 bytes — 3× state vector size */
 uint8_t data_orig[STATE_DIM * 3];
 uint8_t encoded[STATE_DIM * 6]; /* 2× expansion from Hamming(7,4) */
 uint8_t received[STATE_DIM * 6];
 uint8_t decoded[STATE_DIM * 3];

 int total_errors_injected = 0;
 int total_errors_corrected = 0;
 int total_residual = 0;

 /* ---- Outer RS(15,11) buffers ---- */
 /* RS max expansion: ceil(data_len*2 / 11) * 8 bytes per RS block */
 uint8_t rs_encoded[512];
 uint8_t rs_received[512];
 uint8_t rs_decoded[512];
 int rs_total_injected = 0;
 int rs_total_corrected = 0;
 int rs_total_residual = 0;

 for (int trial = 0; trial < num_trials; trial++) {
 /* Generate random ISL payload data */
 for (int i = 0; i < data_len; i++)
 data_orig[i] = (uint8_t)(rand() & 0xFF);

 /* ============ Inner code: Hamming(7,4) ============ */
 int code_len = ecc_encode_block(data_orig, data_len, encoded);

 /* Copy to received buffer and inject single-bit errors per codeword */
 memcpy(received, encoded, code_len);
 int injected = 0;
 for (int i = 0; i < code_len; i++) {
 if (rand() % 3 == 0) { /* ~33% of codewords get an error */
 int bit_pos = rand() % 7;
 received[i] ^= (1 << bit_pos);
 injected++;
 }
 }
 total_errors_injected += injected;

 int corrected = ecc_decode_block(received, code_len, decoded);
 total_errors_corrected += corrected;

 for (int i = 0; i < data_len; i++)
 if (decoded[i] != data_orig[i])
 total_residual++;

 /* ============ Outer code: RS(15,11) ============ */
 int rs_code_len = rs_encode_block(data_orig, data_len, rs_encoded);
 memcpy(rs_received, rs_encoded, rs_code_len);

 /* Inject burst errors: corrupt 2 consecutive bytes per RS block
 * (simulates Ka-band burst noise) */
 int rs_inj = 0;
 for (int b = 0; b + 8 <= rs_code_len; b += 8) {
 if (rand() % 2 == 0) { /* 50% of blocks hit by burst */
 int burst_start = b + (rand() % 6);
 rs_received[burst_start] ^= (uint8_t)(rand() & 0xFF);
 rs_received[burst_start + 1] ^= (uint8_t)(rand() & 0xFF);
 rs_inj += 2;
 }
 }
 rs_total_injected += rs_inj;

 int rs_corr = rs_decode_block(rs_received, rs_code_len, rs_decoded);
 if (rs_corr > 0) rs_total_corrected += rs_corr;

 /* RS only recovers data nibbles; compare first data_len bytes */
 for (int i = 0; i < data_len; i++)
 if (rs_decoded[i] != data_orig[i])
 rs_total_residual++;
 }

 FLIGHT_LOG(" Inner Hamming(7,4): %d trials, %d bit-errors injected, "
 "%d corrected, %d residual byte errors\n",
 num_trials, total_errors_injected,
 total_errors_corrected, total_residual);
 FLIGHT_LOG(" Outer RS(15,11): %d trials, %d burst-bytes injected, "
 "%d symbols corrected, %d residual byte errors\n",
 num_trials, rs_total_injected,
 rs_total_corrected, rs_total_residual);
}

/* ================================================================== */
/* MAIN */
/* ================================================================== */

int main(void)
{
 FLIGHT_LOG("[ISL] cfg: state=%d ant=3x3(%d) neigh=%d fft=%d steps=%d\n", STATE_DIM, ANTENNA_ELEMENTS, NUM_NEIGHBOURS, FFT_LEN, NUM_TIMESTEPS);

 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(5483);
 double host_orbit[6];
 double a = R_EARTH + 550.0e3;
 double v_circ = sqrt(MU_EARTH / a);
 host_orbit[0] = a; host_orbit[1] = 0; host_orbit[2] = 0;
 host_orbit[3] = 0; host_orbit[4] = v_circ; host_orbit[5] = 0;

 /* ---------- Initialise neighbours ---------- */
 Neighbour neighbours[NUM_NEIGHBOURS];
 for (int i = 0; i < NUM_NEIGHBOURS; i++) {
 neighbours[i].id = i;
 double angle = 2.0 * M_PI * i / NUM_NEIGHBOURS;
 double incl_offset = 0.02 * (i - NUM_NEIGHBOURS / 2);
 double a_n = a + 1000.0 * (i - 3); /* Slight altitude spread */
 double v_n = sqrt(MU_EARTH / a_n);
 neighbours[i].state[0] = a_n * cos(angle);
 neighbours[i].state[1] = a_n * sin(angle);
 neighbours[i].state[2] = a_n * incl_offset;
 neighbours[i].state[3] = -v_n * sin(angle);
 neighbours[i].state[4] = v_n * cos(angle);
 neighbours[i].state[5] = v_n * incl_offset;
 neighbours[i].range = 0.0;
 neighbours[i].range_rate = 0.0;
 neighbours[i].link_margin_dB = 0.0;
 neighbours[i].doppler_Hz = 0.0;
 neighbours[i].visible = 1;
 neighbours[i].handover_score = 0.0;
 }

 /* ---------- Initialise EKF state ---------- */
 double x[STATE_DIM];
 double P[STATE_DIM * STATE_DIM];
 double Q_proc[STATE_DIM * STATE_DIM];
 memset(P, 0, sizeof(P));
 memset(Q_proc, 0, sizeof(Q_proc));

 /* Initial relative state to best neighbour */
 x[0] = 1000.0e3; x[1] = 500.0; x[2] = 200.0; /* Relative position (m) */
 x[3] = -50.0; x[4] = 10.0; x[5] = -5.0; /* Relative velocity (m/s) */
 x[6] = 0.001; x[7] = -0.0005; x[8] = 0.0002; /* Attitude error (rad) */
 x[9] = 15.0; x[10] = -6.0; x[11] = 5.4; /* Link: SNR, log(BER), margin */
 x[12] = 5000.0; x[13] = -100.0; x[14] = 2.0; /* Doppler (Hz, Hz/s, Hz/s^2) */
 x[15] = 3.33; x[16] = 0.5; x[17] = 0.001; /* Timing (ns, ns, ns/s) */

 for (int i = 0; i < STATE_DIM; i++) {
 P[i * STATE_DIM + i] = (i < 3) ? 1e6 : (i < 6) ? 100.0 :
 (i < 9) ? 1e-4 : (i < 12) ? 1.0 :
 (i < 15) ? 1e4 : 1.0;
 Q_proc[i * STATE_DIM + i] = P[i * STATE_DIM + i] * 0.001;
 }

 /* Measurement: range + range_rate + Doppler + 3 link metrics = 6 */
 int meas_dim = 6;
 double H_meas[6 * STATE_DIM];
 double R_meas[36];
 memset(H_meas, 0, sizeof(H_meas));
 memset(R_meas, 0, sizeof(R_meas));
 /* Range -> relative position magnitude proxy (state 0) */
 H_meas[0 * STATE_DIM + 0] = 1.0;
 /* Range rate -> relative velocity radial (state 3) */
 H_meas[1 * STATE_DIM + 3] = 1.0;
 /* Doppler -> state 12 */
 H_meas[2 * STATE_DIM + 12] = 1.0;
 /* SNR -> state 9 */
 H_meas[3 * STATE_DIM + 9] = 1.0;
 /* BER -> state 10 */
 H_meas[4 * STATE_DIM + 10] = 1.0;
 /* Timing -> state 15 */
 H_meas[5 * STATE_DIM + 15] = 1.0;

 R_meas[0 * meas_dim + 0] = 100.0; /* Range noise (m^2) */
 R_meas[1 * meas_dim + 1] = 0.1; /* Range rate noise */
 R_meas[2 * meas_dim + 2] = 25.0; /* Doppler noise (Hz^2) */
 R_meas[3 * meas_dim + 3] = 0.5; /* SNR noise */
 R_meas[4 * meas_dim + 4] = 0.01; /* BER noise */
 R_meas[5 * meas_dim + 5] = 0.01; /* Timing noise */

 /* Doppler sample buffer for DFT */
 double doppler_samples[FFT_LEN];
 memset(doppler_samples, 0, sizeof(doppler_samples));

 /* Wavelet Doppler analysis buffer (longer history) */
 double doppler_history[WAVELET_SIGNAL_LEN];
 memset(doppler_history, 0, sizeof(doppler_history));
 double wavelet_doppler_est = 0.0;

 /* Link parameters */
 LinkParams link;
 link.tx_power_dBW = 10.0; /* 10 W transmitter */
 link.tx_gain_dB = 20.0; /* 20 dBi */
 link.rx_gain_dB = 20.0;
 link.frequency_Hz = FREQ_KA;
 link.bandwidth_Hz = 500e6; /* 500 MHz */
 link.noise_temp_K = 290.0;
 link.data_rate_bps = 1e9; /* 1 Gbps */

 double lambda = C_LIGHT / FREQ_KA;
 double d_elem = lambda / 2.0;

 /* Track current best link */
 int current_link = 0;
 int handover_count = 0;
 double est_doppler = 0.0;

 /* ---------- Main simulation loop ---------- */
 for (int t = 0; t < NUM_TIMESTEPS; t++) {

 /* -- 1. Propagate all orbits (Dormand-Prince RK45 adaptive step) -- */
 propagate_orbit_dp45(host_orbit, DT);
 for (int i = 0; i < NUM_NEIGHBOURS; i++)
 propagate_orbit_dp45(neighbours[i].state, DT);

 /* -- 2. Compute relative states and visibility -- */
 for (int i = 0; i < NUM_NEIGHBOURS; i++) {
 double rel_pos[3], rel_vel[3];
 compute_relative_state(host_orbit, neighbours[i].state,
 rel_pos, rel_vel,
 &neighbours[i].range,
 &neighbours[i].range_rate);
 neighbours[i].visible = !is_earth_blocked(host_orbit,
 neighbours[i].state);
 /* Doppler from range rate */
 neighbours[i].doppler_Hz = -neighbours[i].range_rate /
 C_LIGHT * FREQ_KA;
 }

 /* -- 3. Beam steering to current link partner -- */
 double rel_pos_eci[3];
 for (int i = 0; i < 3; i++)
 rel_pos_eci[i] = neighbours[current_link].state[i] - host_orbit[i];

 /* Convert to body frame using attitude DCM */
 double dcm[3][3];
 double att_roll = x[6] + 0.001 * sin(0.01 * t);
 double att_pitch = x[7] + 0.001 * cos(0.01 * t);
 double att_yaw = x[8];
 euler_to_dcm(att_roll, att_pitch, att_yaw, dcm);

 double rel_body[3];
 dcm_rotate(dcm, rel_pos_eci, rel_body);

 /* Compute beam angles */
 double range_xy = sqrt(rel_body[0] * rel_body[0] +
 rel_body[1] * rel_body[1]);
 double el = atan2(range_xy, rel_body[2]);
 double az = atan2(rel_body[1], rel_body[0]);

 /* Compute beam weights (3x3 matrix-intensive) */
 double w_re[3][3], w_im[3][3];
 compute_beam_weights(az, el, lambda, d_elem, w_re, w_im);

 /* Array factor at boresight (beam gain) */
 double beam_gain = compute_array_factor(w_re, w_im, el, az,
 lambda, d_elem);

 /* -- 4. Link budget -- */
 link.range_m = neighbours[current_link].range;
 double margin = compute_link_margin(&link, beam_gain);
 neighbours[current_link].link_margin_dB = margin;

 /* -- 5. Doppler estimation via DFT -- */
 /* Shift buffer and add new sample */
 memmove(doppler_samples, doppler_samples + 1,
 (FFT_LEN - 1) * sizeof(double));
 doppler_samples[FFT_LEN - 1] = neighbours[current_link].doppler_Hz
 + 5.0 * rand_gauss();

 if (t >= FFT_LEN) {
 est_doppler = estimate_doppler_dft(doppler_samples, FFT_LEN,
 1.0 / DT);
 }

 /* -- 5b. Wavelet multi-resolution Doppler estimation -- */
 memmove(doppler_history, doppler_history + 1,
 (WAVELET_SIGNAL_LEN - 1) * sizeof(double));
 doppler_history[WAVELET_SIGNAL_LEN - 1] =
 neighbours[current_link].doppler_Hz + 5.0 * rand_gauss();

 if (t >= WAVELET_SIGNAL_LEN) {
 wavelet_doppler_est = wavelet_doppler_estimate(
 doppler_history, WAVELET_SIGNAL_LEN, 1.0 / DT);
 }

 /* -- 6. Orbit STM computation (matrix-heavy, every 6 steps) -- */
 if ((t % 6) == 0) {
 double STM[36];
 compute_orbit_stm(host_orbit, DT, STM);

 /* Use STM for uncertainty propagation:
 * P_orbit = STM * P_orbit * STM^T */
 double P_orbit[36], STM_T[36], SP[36], SPST[36];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 6; j++)
 P_orbit[i * 6 + j] = P[i * STATE_DIM + j];
 mat_mul(STM, P_orbit, SP, 6, 6, 6);
 mat_transpose(STM, STM_T, 6, 6);
 mat_mul(SP, STM_T, SPST, 6, 6, 6);
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 6; j++)
 P[i * STATE_DIM + j] = SPST[i * 6 + j];
 }

 /* -- 7. EKF prediction and update -- */
 double F[STATE_DIM * STATE_DIM];
 build_link_jacobian(x, DT, F);
 link_state_predict(x, DT);
 ekf_predict_covariance(P, F, Q_proc);

 /* Simulated measurements */
 double z_meas[6];
 z_meas[0] = x[0] + 10.0 * rand_gauss(); /* Range */
 z_meas[1] = x[3] + 0.3 * rand_gauss(); /* Range rate */
 z_meas[2] = x[12] + 5.0 * rand_gauss(); /* Doppler */
 z_meas[3] = x[9] + 0.7 * rand_gauss(); /* SNR */
 z_meas[4] = x[10] + 0.1 * rand_gauss(); /* BER */
 z_meas[5] = x[15] + 0.1 * rand_gauss(); /* Timing */
 ekf_update(x, P, z_meas, H_meas, R_meas, meas_dim);

 /* -- 8. Handover decision -- */
 for (int i = 0; i < NUM_NEIGHBOURS; i++) {
 if (!neighbours[i].visible) {
 neighbours[i].handover_score = -1e10;
 continue;
 }
 /* Score: prefer closer range, lower Doppler rate, good SNR */
 double range_score = -neighbours[i].range / 1e6;
 double doppler_penalty = -fabs(neighbours[i].doppler_Hz) / 1e4;
 double margin_bonus = (i == current_link) ?
 neighbours[i].link_margin_dB + 3.0 : /* Hysteresis */
 neighbours[i].link_margin_dB;
 neighbours[i].handover_score = range_score + doppler_penalty
 + margin_bonus;
 }

 /* Find best candidate */
 int best = current_link;
 double best_score = -1e30;
 for (int i = 0; i < NUM_NEIGHBOURS; i++) {
 if (neighbours[i].handover_score > best_score) {
 best_score = neighbours[i].handover_score;
 best = i;
 }
 }
 if (best != current_link) {
 FLIGHT_LOG(" t=%3d: Handover %d -> %d (margin %.1f dB -> est %.1f dB)\n",
 t, current_link, best, margin,
 neighbours[best].link_margin_dB);
 current_link = best;
 handover_count++;
 }

 /* -- 9. Status output -- */
 if (t % 10 == 0) {
 FLIGHT_LOG("t=%3d | Link=%d | Range=%.0f km | Doppler=%.0f Hz | "
 "Margin=%.1f dB | Beam=%.1f dB | DFT_Dop=%.1f Hz\n",
 t, current_link,
 neighbours[current_link].range / 1e3,
 neighbours[current_link].doppler_Hz,
 margin, beam_gain, est_doppler);
 }
 }

 /* ---------- Final summary ---------- */
 FLIGHT_LOG("Handovers performed : %d\n", handover_count);
 FLIGHT_LOG("Final link partner : satellite %d\n", current_link);
 FLIGHT_LOG("Final range : %.1f km\n",
 neighbours[current_link].range / 1e3);
 FLIGHT_LOG("Final Doppler : %.1f Hz\n",
 neighbours[current_link].doppler_Hz);
 FLIGHT_LOG("Final link margin : %.1f dB\n",
 neighbours[current_link].link_margin_dB);
 FLIGHT_LOG("Wavelet Doppler est : %.2f Hz\n", wavelet_doppler_est);
 FLIGHT_LOG("EKF position unc (1σ): %.1f m\n",
 sqrt(P[0] + P[STATE_DIM + 1] + P[2 * STATE_DIM + 2]));

 /* Visibility summary */
 int vis_count = 0;
 for (int i = 0; i < NUM_NEIGHBOURS; i++)
 if (neighbours[i].visible) vis_count++;
 FLIGHT_LOG("Visible neighbours : %d / %d\n", vis_count, NUM_NEIGHBOURS);

 /* ---------- ECC verification on ISL data transfer ---------- */
 FLIGHT_LOG("\n[ISL] ECC: Inner Hamming(7,4) + Outer RS(15,11) over GF(2^4)\n");
 run_ecc_verification(NUM_TIMESTEPS * NUM_NEIGHBOURS); /* Realistic trial count */

 return 0;
}
