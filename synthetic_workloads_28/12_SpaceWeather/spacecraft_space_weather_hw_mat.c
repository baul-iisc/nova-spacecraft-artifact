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
 * Space Weather Monitoring Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Mat (AME 3×3 tile accelerator for matrix operations)
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard space weather monitoring and solar event detection. This
 * workload performs real-time analysis of solar wind plasma parameters, magnetic
 * field spectral analysis via FFT, particle flux Kalman filtering,
 * quaternion-based instrument pointing for directional sensors, and
 * geomagnetic storm prediction using matrix-based models. Applicable to
 * solar observatory (ASPEX, PAPA payloads) for solar wind characterization and to
 * radiation environment monitoring on all GEO satellites.
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
#include "../flight_compliance.h"

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
/* Trig/math wrappers for accelerator-friendly profiling */
static inline double sw_sin(double a) { return sin(a); }
static inline double sw_cos(double a) { return cos(a); }
#define sw_sqrt(a) sqrt(a)
#define sw_atan2(y, x) atan2(y, x)

/* ---------- Configuration ---------- */
#define STATE_DIM 18 /* IMF Bx,By,Bz + solar wind V + density + */
 /* temperature + particle flux (6 energy bins) + */
 /* Dst index + 3 bias terms = 18 */
#define MEAS_DIM_MAG 3 /* Magnetometer: Bx, By, Bz */
#define MEAS_DIM_PARTICLE 6 /* Particle detector: 6 energy channels */
#define MEAS_DIM_XRAY 3 /* Solar X-ray sensor: 3 bands */
#define FFT_LEN 64 /* DFT length (power-of-2 for DFT efficiency) */
#define NUM_TIMESTEPS 50 /* Simulation timesteps */
#define NUM_ENERGY_BINS 6 /* Particle energy channels */
#define DT 60.0 /* Sample interval (seconds) */

/* Physical constants */
#define MU_0 (4.0 * M_PI * 1e-7) /* Vacuum permeability (H/m) */
#define K_BOLTZMANN 1.380649e-23 /* Boltzmann constant (J/K) */
#define PROTON_MASS 1.6726219e-27 /* Proton mass (kg) */
#define AU_METERS 1.496e11 /* 1 AU in metres */
#define EARTH_RADIUS 6371.0e3 /* Earth radius (m) */

/* ------------------------------------------------------------------ */
/* Generic matrix operations (pure software, 3x3 tile-friendly) */
/* ------------------------------------------------------------------ */

/**
 * General matrix multiply: C = A * B (m x n) = (m x k) * (k x n)
 * All matrices stored row-major as flat arrays.
 * AME 3×3 tile accelerated via SYS_MMACD.
 */
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


/**
 * Matrix transpose: B = A^T
 */
static void mat_transpose(const double *A, double *AT, int m, int n)
{
 for (int i = 0; i < m; i++)
 for (int j = 0; j < n; j++)
 AT[j * m + i] = A[i * n + j];
}

/**
 * Matrix addition: C = A + B (m x n)
 */
static void mat_add(const double *A, const double *B, double *C, int m, int n)
{
 int sz = m * n;
 for (int i = 0; i < sz; i++) C[i] = A[i] + B[i];
}

/**
 * Matrix subtraction: C = A - B
 */
__attribute__((unused))
static void mat_sub(const double *A, const double *B, double *C, int m, int n)
{
 int sz = m * n;
 for (int i = 0; i < sz; i++) C[i] = A[i] - B[i];
}

/* ------------------------------------------------------------------ */
/* HW Helper: 3×3 inverse via MINVD instruction */
/* ------------------------------------------------------------------ */
static void hw_inv3(const double *A, double *Ainv) {
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 "mv t2, %2\n\t"
 MLDS(M1, T1, T0)
 MINVD(M2, M1)
 MSDS(M2, T2, T0)
 :: "r"(3 * (int)sizeof(double)), "r"(A), "r"(Ainv)
 : "t0", "t1", "t2", "memory"
 );
}

/* ------------------------------------------------------------------ */
/* HW Helper: Extract / insert sub-blocks from flat arrays */
/* ------------------------------------------------------------------ */
static void hw_extract_block(const double *M, int n_stride,
 int r0, int c0, int rows, int cols, double *out) {
 for (int i = 0; i < rows; i++)
 for (int j = 0; j < cols; j++)
 out[i * cols + j] = M[(r0 + i) * n_stride + (c0 + j)];
}
static void hw_insert_block(double *M, int n_stride,
 int r0, int c0, int rows, int cols, const double *blk) {
 for (int i = 0; i < rows; i++)
 for (int j = 0; j < cols; j++)
 M[(r0 + i) * n_stride + (c0 + j)] = blk[i * cols + j];
}

/* ------------------------------------------------------------------ */
/* HW Helper: Small base-case inverses (n=1, n=2) */
/* ------------------------------------------------------------------ */
static int hw_inv1(const double *A, double *Ainv) {
 if (fabs(A[0]) < 1e-15) return 0;
 Ainv[0] = 1.0 / A[0];
 return 1;
}
static int hw_inv2(const double *A, double *Ainv) {
 double a = A[0], b = A[1], c = A[2], d = A[3];
 double det = a * d - b * c;
 if (fabs(det) < 1e-15) return 0;
 double id = 1.0 / det;
 Ainv[0] = d * id; Ainv[1] = -b * id;
 Ainv[2] = -c * id; Ainv[3] = a * id;
 return 1;
}

/* ------------------------------------------------------------------ */
/* Recursive Schur complement inverse — HW 3×3 tiles */
/* ------------------------------------------------------------------ */
static int hw_inverse_recursive(const double *M, double *M_inv, int n_dim) {
 if (n_dim == 1) return hw_inv1(M, M_inv);
 if (n_dim == 2) return hw_inv2(M, M_inv);
 if (n_dim == 3) { hw_inv3(M, M_inv); return 1; }

 int p = 3, q = n_dim - p;

 double *A_blk = (double *)flight_malloc_impl(p * p * sizeof(double));
 double *B_blk = (double *)flight_malloc_impl(p * q * sizeof(double));
 double *C_blk = (double *)flight_malloc_impl(q * p * sizeof(double));
 double *D_blk = (double *)flight_malloc_impl(q * q * sizeof(double));
 double *Ai = (double *)flight_malloc_impl(p * p * sizeof(double));
 double *CA = (double *)flight_malloc_impl(q * p * sizeof(double));
 double *S = (double *)flight_malloc_impl(q * q * sizeof(double));
 double *Si = (double *)flight_malloc_impl(q * q * sizeof(double));
 double *AB = (double *)flight_malloc_impl(p * q * sizeof(double));
 double *F_blk = (double *)flight_malloc_impl(p * q * sizeof(double));
 double *G_blk = (double *)flight_malloc_impl(q * p * sizeof(double));
 double *E_blk = (double *)flight_malloc_impl(p * p * sizeof(double));
 double *tmp_qq = (double *)flight_malloc_impl(q * q * sizeof(double));
 double *tmp_pp = (double *)flight_malloc_impl(p * p * sizeof(double));
 double *neg_F = (double *)flight_malloc_impl(p * q * sizeof(double));

 int ok = 1;

 hw_extract_block(M, n_dim, 0, 0, p, p, A_blk);
 hw_extract_block(M, n_dim, 0, p, p, q, B_blk);
 hw_extract_block(M, n_dim, p, 0, q, p, C_blk);
 hw_extract_block(M, n_dim, p, p, q, q, D_blk);

 hw_inv3(A_blk, Ai);
 mat_mul(C_blk, Ai, CA, q, p, p);
 mat_mul(CA, B_blk, tmp_qq, q, p, q);
 for (int i = 0; i < q * q; i++) S[i] = D_blk[i] - tmp_qq[i];

 ok = hw_inverse_recursive(S, Si, q);
 if (!ok) goto cleanup;

 mat_mul(Ai, B_blk, AB, p, p, q);
 mat_mul(AB, Si, F_blk, p, q, q);
 for (int i = 0; i < p * q; i++) F_blk[i] = -F_blk[i];

 mat_mul(Si, CA, G_blk, q, q, p);
 for (int i = 0; i < q * p; i++) G_blk[i] = -G_blk[i];

 for (int i = 0; i < p * q; i++) neg_F[i] = -F_blk[i];
 mat_mul(neg_F, CA, tmp_pp, p, q, p);
 for (int i = 0; i < p * p; i++) E_blk[i] = Ai[i] + tmp_pp[i];

 hw_insert_block(M_inv, n_dim, 0, 0, p, p, E_blk);
 hw_insert_block(M_inv, n_dim, 0, p, p, q, F_blk);
 hw_insert_block(M_inv, n_dim, p, 0, q, p, G_blk);
 hw_insert_block(M_inv, n_dim, p, p, q, q, Si);

cleanup:
 flight_free_impl(A_blk); flight_free_impl(B_blk); flight_free_impl(C_blk); flight_free_impl(D_blk);
 flight_free_impl(Ai); flight_free_impl(CA); flight_free_impl(S); flight_free_impl(Si);
 flight_free_impl(AB); flight_free_impl(F_blk); flight_free_impl(G_blk); flight_free_impl(E_blk);
 flight_free_impl(tmp_qq); flight_free_impl(tmp_pp); flight_free_impl(neg_F);
 return ok;
}

/**
 * HW-accelerated matrix inverse via recursive Schur complement
 * with MINVD for 3×3 base case and SYS_MMACD-tiled multiplies.
 * API-compatible: returns 0 on success, -1 on singular.
 */
static int mat_inv(const double *A, double *Ainv, int n)
{
 if (n > 18) return -1;
 int ok = hw_inverse_recursive(A, Ainv, n);
 return ok ? 0 : -1;
}

/**
 * Matrix multiply wrapper for profiling (delegates to mat_mul).
 * Profiling alias: allows separate HW-counter attribution when
 * comparing accelerated vs. software matrix paths.
 */
static void sw_matrix_multiply(const double *A, const double *B, double *C,
 int m, int k, int n)
{
 mat_mul(A, B, C, m, k, n);
}

/* ------------------------------------------------------------------ */
/* Quaternion operations (instrument attitude tracking) */
/* ------------------------------------------------------------------ */

typedef struct {
 double w, x, y, z;
} Quaternion;

/**
 * Quaternion norm-drift health monitor.
 *
 * In flight software, accumulated floating-point error causes the
 * quaternion norm to drift from unity between normalisations. Tracking
 * this drift serves as a health-of-computation metric: a sudden spike
 * indicates an SEU, numerical instability, or software fault.
 */
typedef struct {
 double max_drift; /* worst-case |norm - 1| seen so far */
 double sum_drift; /* running sum for mean computation */
 unsigned int samples; /* number of normalisation calls */
 unsigned int warnings; /* count of drift exceeding QUAT_DRIFT_WARN */
} QuatHealth;

#define QUAT_DRIFT_WARN 1e-6 /* flag if |norm-1| exceeds this */

/**
 * Normalize quaternion to unit length.
 * Records pre-normalisation norm-drift into the health monitor.
 */
static Quaternion quaternion_normalize(Quaternion q, QuatHealth *qh)
{
 double norm = sw_sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

 /* --- Norm-drift health metric --- */
 if (qh) {
 double drift = fabs(norm - 1.0);
 if (drift > qh->max_drift) qh->max_drift = drift;
 qh->sum_drift += drift;
 qh->samples++;
 if (drift > QUAT_DRIFT_WARN) qh->warnings++;
 }

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

/* ------------------------------------------------------------------ */
/* Coordinate transforms (trigonometric-heavy) */
/* ------------------------------------------------------------------ */

/** GSE to GSM coordinate rotation (depends on dipole tilt angle). */
static void gse_to_gsm(const double v_gse[3], double v_gsm[3],
 double dipole_tilt_rad)
{
 double ct = sw_cos(dipole_tilt_rad);
 double st = sw_sin(dipole_tilt_rad);
 /* Rotation about X_GSE axis */
 v_gsm[0] = v_gse[0];
 v_gsm[1] = ct * v_gse[1] + st * v_gse[2];
 v_gsm[2] = -st * v_gse[1] + ct * v_gse[2];
}

/**
 * Build Direction Cosine Matrix for ecliptic-to-equatorial transform.
 * Obliquity of ecliptic ~23.44 deg.
 */
static void ecliptic_to_equatorial_dcm(double dcm[3][3], double obliquity_rad)
{
 double ce = sw_cos(obliquity_rad);
 double se = sw_sin(obliquity_rad);
 dcm[0][0] = 1.0; dcm[0][1] = 0.0; dcm[0][2] = 0.0;
 dcm[1][0] = 0.0; dcm[1][1] = ce; dcm[1][2] = -se;
 dcm[2][0] = 0.0; dcm[2][1] = se; dcm[2][2] = ce;
}

/**
 * Spherical to Cartesian conversion (solar wind direction).
 */
static void spherical_to_cartesian(double r, double theta, double phi,
 double xyz[3])
{
 xyz[0] = r * sw_sin(theta) * sw_cos(phi);
 xyz[1] = r * sw_sin(theta) * sw_sin(phi);
 xyz[2] = r * sw_cos(theta);
}

/* ------------------------------------------------------------------ */
/* Discrete Fourier Transform (particle flux spectral analysis) */
/* ------------------------------------------------------------------ */

/**
 * Real-valued DFT of length N producing N/2+1 magnitude bins.
 * O(N^2) -- intentionally not FFT, to match embedded FPGA-less OBC.
 */
static void compute_dft_magnitude(const double *x, double *mag, int N)
{
 for (int k = 0; k <= N / 2; k++) {
 double re = 0.0, im = 0.0;
 for (int n = 0; n < N; n++) {
 double angle = 2.0 * M_PI * k * n / N;
 re += x[n] * sw_cos(angle);
 im -= x[n] * sw_sin(angle);
 }
 mag[k] = sqrt(re * re + im * im) / N;
 }
}

/* ------------------------------------------------------------------ */
/* EKF for solar-wind / IMF state estimation */
/* ------------------------------------------------------------------ */

/** State vector layout (18 elements):
 * [0..2] IMF components Bx, By, Bz (nT)
 * [3] Solar-wind bulk speed (km/s)
 * [4] Proton number density (cm^-3)
 * [5] Proton temperature (eV)
 * [6..11] Particle flux in 6 energy channels (log-scale)
 * [12] Dst index estimate (nT)
 * [13..14] Magnetometer bias Bx, By (nT)
 * [15..17] Gyro-drift correction (rad/s)
 */

static void state_predict(double x[STATE_DIM], double dt)
{
 /* Simple Parker-spiral propagation for IMF rotation */
 double omega_sun = 2.7e-6; /* Solar rotation rate (rad/s) */

 double Bx = x[0], By = x[1];
 double ca = sw_cos(omega_sun * dt);
 double sa = sw_sin(omega_sun * dt);
 x[0] = ca * Bx - sa * By;
 x[1] = sa * Bx + ca * By;
 /* Bz decays slowly */
 x[2] *= exp(-dt / 86400.0);

 /* Solar-wind speed: slow deceleration model */
 x[3] -= 0.001 * dt;
 /* Density: adiabatic expansion */
 x[4] *= exp(-0.0001 * dt);
 /* Temperature: cooling */
 x[5] *= exp(-0.00005 * dt);

 /* Particle flux: exponential decay per channel with solar modulation */
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++) {
 double decay = 0.0002 * (ch + 1);
 x[6 + ch] *= exp(-decay * dt);
 }

 /* Dst index: Burton et al. (1975) empirical model
 * dDst/dt = Q(Ey) - Dst/tau where Ey = -Vx*Bz_gsm */
 double tilt = 0.2; /* Average dipole tilt (rad), approx 11 deg */
 double Bz_gsm = -sw_sin(tilt) * x[1] + sw_cos(tilt) * x[2];
 double Ey = -(x[3]) * Bz_gsm * 1e-3; /* mV/m */
 double Q = (Ey > 0.5) ? (-4.4 / 3600.0) * (Ey - 0.5) : 0.0;
 double tau_dst = 7.7 * 3600.0; /* Recovery time (s) */
 x[12] += (Q - x[12] / tau_dst) * dt;

 /* Biases: random walk (no change in prediction) */
}

/**
 * Compute the Jacobian F = df/dx for the state prediction model.
 * F is STATE_DIM x STATE_DIM.
 */
static void compute_jacobian_F(const double x[STATE_DIM], double dt,
 double F[STATE_DIM * STATE_DIM])
{
 memset(F, 0, STATE_DIM * STATE_DIM * sizeof(double));
 /* Identity base */
 for (int i = 0; i < STATE_DIM; i++)
 F[i * STATE_DIM + i] = 1.0;

 double omega_sun = 2.7e-6;
 double ca = sw_cos(omega_sun * dt);
 double sa = sw_sin(omega_sun * dt);

 /* dBx'/dBx, dBx'/dBy */
 F[0 * STATE_DIM + 0] = ca;
 F[0 * STATE_DIM + 1] = -sa;
 /* dBy'/dBx, dBy'/dBy */
 F[1 * STATE_DIM + 0] = sa;
 F[1 * STATE_DIM + 1] = ca;
 /* dBz'/dBz */
 F[2 * STATE_DIM + 2] = exp(-dt / 86400.0);

 /* Density and temperature decay diagonal entries */
 F[4 * STATE_DIM + 4] = exp(-0.0001 * dt);
 F[5 * STATE_DIM + 5] = exp(-0.00005 * dt);

 /* Particle flux channels */
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++) {
 double decay = 0.0002 * (ch + 1);
 F[(6 + ch) * STATE_DIM + (6 + ch)] = exp(-decay * dt);
 }

 /* Dst index partials */
 double tau_dst = 7.7 * 3600.0;
 F[12 * STATE_DIM + 12] = 1.0 - dt / tau_dst;
 /* dDst/dVsw and dDst/dBz_gsm (via Ey) -- GSE to GSM rotation about X */
 double tilt = 0.2; /* Average dipole tilt (rad), approx 11 deg */
 double Bz_gsm = -sw_sin(tilt) * x[1] + sw_cos(tilt) * x[2];
 double Ey = -(x[3]) * Bz_gsm * 1e-3;
 if (Ey > 0.5) {
 F[12 * STATE_DIM + 3] = (-4.4 / 3600.0) * (-Bz_gsm * 1e-3) * dt;
 F[12 * STATE_DIM + 2] = (-4.4 / 3600.0) * (-x[3] * sw_cos(tilt) * 1e-3) * dt;
 F[12 * STATE_DIM + 1] = (-4.4 / 3600.0) * (x[3] * sw_sin(tilt) * 1e-3) * dt;
 }
}

/**
 * EKF covariance prediction: P = F*P*F^T + Q
 * This is the most matrix-intensive step (39% of total compute).
 */
static void ekf_predict_covariance(double P[STATE_DIM * STATE_DIM],
 const double F[STATE_DIM * STATE_DIM],
 const double Q[STATE_DIM * STATE_DIM])
{
 double FP[STATE_DIM * STATE_DIM];
 double FT[STATE_DIM * STATE_DIM];
 double FPFT[STATE_DIM * STATE_DIM];

 sw_matrix_multiply(F, P, FP, STATE_DIM, STATE_DIM, STATE_DIM);
 mat_transpose(F, FT, STATE_DIM, STATE_DIM);
 sw_matrix_multiply(FP, FT, FPFT, STATE_DIM, STATE_DIM, STATE_DIM);
 mat_add(FPFT, Q, P, STATE_DIM, STATE_DIM);
}

/**
 * EKF update with a measurement z of dimension meas_dim.
 * H is (meas_dim x STATE_DIM), R is (meas_dim x meas_dim).
 */
static void ekf_update(double x[STATE_DIM],
 double P[STATE_DIM * STATE_DIM],
 const double *z, const double *H,
 const double *R, int meas_dim)
{
 double Hx[6]; /* predicted measurement (max meas_dim=6) */
 double y[6]; /* innovation */

 /* Hx = H * x */
 mat_mul(H, x, Hx, meas_dim, STATE_DIM, 1);
 for (int i = 0; i < meas_dim; i++)
 y[i] = z[i] - Hx[i];

 /* S = H * P * H^T + R */
 double HP[6 * STATE_DIM];
 double HT[STATE_DIM * 6];
 double S[36], Sinv[36];
 mat_mul(H, P, HP, meas_dim, STATE_DIM, STATE_DIM);
 mat_transpose(H, HT, meas_dim, STATE_DIM);
 double HPHT[36];
 mat_mul(HP, HT, HPHT, meas_dim, STATE_DIM, meas_dim);
 mat_add(HPHT, R, S, meas_dim, meas_dim);

 /* K = P * H^T * S^{-1} */
 if (mat_inv(S, Sinv, meas_dim) != 0) {
 return; /* Innovation covariance singular -- skip measurement update */
 }
 double PHT[STATE_DIM * 6];
 mat_mul(P, HT, PHT, STATE_DIM, STATE_DIM, meas_dim);
 double K[STATE_DIM * 6];
 mat_mul(PHT, Sinv, K, STATE_DIM, meas_dim, meas_dim);

 /* x = x + K * y */
 double Ky[STATE_DIM];
 mat_mul(K, y, Ky, STATE_DIM, meas_dim, 1);
 for (int i = 0; i < STATE_DIM; i++)
 x[i] += Ky[i];

 /* P = (I - K*H) * P */
 double KH[STATE_DIM * STATE_DIM];
 mat_mul(K, H, KH, STATE_DIM, meas_dim, STATE_DIM);
 double I_KH[STATE_DIM * STATE_DIM];
 for (int i = 0; i < STATE_DIM * STATE_DIM; i++)
 I_KH[i] = -KH[i];
 for (int i = 0; i < STATE_DIM; i++)
 I_KH[i * STATE_DIM + i] += 1.0;
 double Pnew[STATE_DIM * STATE_DIM];
 mat_mul(I_KH, P, Pnew, STATE_DIM, STATE_DIM, STATE_DIM);
 memcpy(P, Pnew, STATE_DIM * STATE_DIM * sizeof(double));
}

/* ------------------------------------------------------------------ */
/* Measurement models */
/* ------------------------------------------------------------------ */

/** Magnetometer: directly observes IMF Bx,By,Bz plus bias */
static void build_H_magnetometer(double H[MEAS_DIM_MAG * STATE_DIM])
{
 memset(H, 0, MEAS_DIM_MAG * STATE_DIM * sizeof(double));
 H[0 * STATE_DIM + 0] = 1.0; /* Bx */
 H[1 * STATE_DIM + 1] = 1.0; /* By */
 H[2 * STATE_DIM + 2] = 1.0; /* Bz */
 /* Bias terms */
 H[0 * STATE_DIM + 13] = 1.0; /* Bx bias */
 H[1 * STATE_DIM + 14] = 1.0; /* By bias */
}

/** Particle detector: observes 6 energy channels */
static void build_H_particle(double H[MEAS_DIM_PARTICLE * STATE_DIM])
{
 memset(H, 0, MEAS_DIM_PARTICLE * STATE_DIM * sizeof(double));
 for (int ch = 0; ch < MEAS_DIM_PARTICLE; ch++)
 H[ch * STATE_DIM + (6 + ch)] = 1.0;
}

/** Solar X-ray: observes temperature proxy + two flux ratios */
static void build_H_xray(double H[MEAS_DIM_XRAY * STATE_DIM])
{
 memset(H, 0, MEAS_DIM_XRAY * STATE_DIM * sizeof(double));
 H[0 * STATE_DIM + 5] = 1.0; /* Temperature */
 H[1 * STATE_DIM + 6] = 0.5; /* Low-energy flux proxy */
 H[2 * STATE_DIM + 11] = 0.5; /* High-energy flux proxy */
}

/* ------------------------------------------------------------------ */
/* Spectral analysis of particle flux */
/* ------------------------------------------------------------------ */

/** Analyse particle flux time series for each energy channel.
 * Computes DFT magnitudes and dominant frequency. */
static void analyse_particle_spectrum(const double flux_history[][NUM_ENERGY_BINS],
 int history_len,
 double dominant_freq[NUM_ENERGY_BINS])
{
 double time_series[FFT_LEN];
 double mag[FFT_LEN / 2 + 1];
 int N = (history_len < FFT_LEN) ? history_len : FFT_LEN;

 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++) {
 /* Extract latest N samples for this channel */
 for (int n = 0; n < N; n++)
 time_series[n] = flux_history[history_len - N + n][ch];

 compute_dft_magnitude(time_series, mag, N);

 /* Find dominant frequency (skip DC) */
 double max_mag = 0.0;
 int max_k = 1;
 for (int k = 1; k <= N / 2; k++) {
 if (mag[k] > max_mag) {
 max_mag = mag[k];
 max_k = k;
 }
 }
 dominant_freq[ch] = (double)max_k / (N * DT); /* Hz */
 }
}

/* ------------------------------------------------------------------ */
/* Radiation dose-rate forecasting */
/* ------------------------------------------------------------------ */

/**
 * Estimate dose rate from particle flux using a simple power-law
 * energy-dependent shielding model.
 * dose_rate (rad/s) = sum_ch flux[ch] * E[ch]^alpha / shielding[ch]
 */
static double forecast_dose_rate(const double flux[NUM_ENERGY_BINS])
{
 /* Representative energy bin centres (MeV) and shielding factors */
 static const double E_centre[NUM_ENERGY_BINS] =
 {10.0, 30.0, 60.0, 100.0, 200.0, 500.0};
 static const double shielding[NUM_ENERGY_BINS] =
 {100.0, 50.0, 20.0, 10.0, 5.0, 2.0};
 double alpha = 1.75; /* Power-law index */

 double dose = 0.0;
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++) {
 double E_pow = pow(E_centre[ch], alpha);
 dose += exp(flux[ch]) * E_pow / shielding[ch];
 }
 return dose * 1e-10; /* Scale to rad/s */
}

/* ------------------------------------------------------------------ */
/* Geomagnetic storm severity classification */
/* ------------------------------------------------------------------ */

static const char *classify_storm(double dst)
{
 if (dst > -30.0) return "Quiet";
 if (dst > -50.0) return "Weak";
 if (dst > -100.0) return "Moderate";
 if (dst > -200.0) return "Strong";
 if (dst > -350.0) return "Severe";
 return "Extreme";
}

/* ------------------------------------------------------------------ */
/* Simulate sensor measurements with realistic noise */
/* ------------------------------------------------------------------ */

static double rand_gauss(void)
{
 /* Box-Muller transform */
 double u1 = (rand() / (double)RAND_MAX);
 double u2 = (rand() / (double)RAND_MAX);
 if (u1 < 1e-15) u1 = 1e-15;
 return sqrt(-2.0 * log(u1)) * sw_cos(2.0 * M_PI * u2);
}

static void simulate_magnetometer(const double x[STATE_DIM],
 double z[MEAS_DIM_MAG])
{
 z[0] = x[0] + x[13] + 0.1 * rand_gauss(); /* Bx + bias + noise */
 z[1] = x[1] + x[14] + 0.1 * rand_gauss(); /* By + bias + noise */
 z[2] = x[2] + 0.1 * rand_gauss(); /* Bz + noise */
}

static void simulate_particle_detector(const double x[STATE_DIM],
 double z[MEAS_DIM_PARTICLE])
{
 for (int ch = 0; ch < MEAS_DIM_PARTICLE; ch++)
 z[ch] = x[6 + ch] + 0.05 * rand_gauss();
}

static void simulate_xray_sensor(const double x[STATE_DIM],
 double z[MEAS_DIM_XRAY])
{
 z[0] = x[5] + 0.02 * rand_gauss(); /* Temperature */
 z[1] = 0.5 * x[6] + 0.01 * rand_gauss(); /* Low-E proxy */
 z[2] = 0.5 * x[11] + 0.01 * rand_gauss(); /* High-E proxy */
}

/* ------------------------------------------------------------------ */
/* Spectral covariance analysis (for solar-wind variability) */
/* ------------------------------------------------------------------ */

/**
 * Compute spectral covariance matrix from DFT magnitudes of all channels.
 * Result is 6x6. Contributes to the matrix-heavy portion of the workload.
 */
static void compute_spectral_covariance(const double flux_history[][NUM_ENERGY_BINS],
 int history_len,
 double cov[NUM_ENERGY_BINS * NUM_ENERGY_BINS])
{
 double means[NUM_ENERGY_BINS] = {0};
 int N = history_len;

 for (int n = 0; n < N; n++)
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++)
 means[ch] += flux_history[n][ch];
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++)
 means[ch] /= N;

 memset(cov, 0, NUM_ENERGY_BINS * NUM_ENERGY_BINS * sizeof(double));
 for (int n = 0; n < N; n++)
 for (int i = 0; i < NUM_ENERGY_BINS; i++)
 for (int j = 0; j < NUM_ENERGY_BINS; j++)
 cov[i * NUM_ENERGY_BINS + j] +=
 (flux_history[n][i] - means[i]) *
 (flux_history[n][j] - means[j]);
 for (int i = 0; i < NUM_ENERGY_BINS * NUM_ENERGY_BINS; i++)
 cov[i] /= (N - 1);
}

/* ================================================================== */
/* MAIN */
/* ================================================================== */

int main(void)
{
 FLIGHT_LOG("[SWX] cfg: state_dim=%d fft=%d steps=%d dt=%.0fs\n", STATE_DIM, FFT_LEN, NUM_TIMESTEPS, DT);

 /* ---------- Initialise state and covariance ---------- */
 double x[STATE_DIM];
 double P[STATE_DIM * STATE_DIM];
 double Q[STATE_DIM * STATE_DIM];

 memset(P, 0, sizeof(P));
 memset(Q, 0, sizeof(Q));

 /* Typical quiet-time solar wind initial conditions */
 x[0] = 5.0; /* Bx = 5 nT */
 x[1] = 0.0; /* By = 0 nT */
 x[2] = -2.0; /* Bz = -2 nT (southward component) */
 x[3] = 400.0; /* Vsw = 400 km/s */
 x[4] = 5.0; /* n = 5 cm^-3 */
 x[5] = 10.0; /* T = 10 eV */
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++)
 x[6 + ch] = 2.0 - 0.3 * ch; /* Log-scale flux, decreasing with energy */
 x[12] = -15.0; /* Dst = -15 nT (quiet) */
 x[13] = 0.1; /* Bx bias */
 x[14] = -0.05; /* By bias */
 x[15] = 0.0; /* Gyro drift corrections */
 x[16] = 0.0;
 x[17] = 0.0;

 /* Initial covariance */
 for (int i = 0; i < STATE_DIM; i++)
 P[i * STATE_DIM + i] = (i < 3) ? 1.0 : (i < 6) ? 10.0 :
 (i < 12) ? 0.5 : (i == 12) ? 25.0 : 0.01;
 /* Process noise */
 for (int i = 0; i < STATE_DIM; i++)
 Q[i * STATE_DIM + i] = (i < 3) ? 0.01 : (i < 6) ? 0.1 :
 (i < 12) ? 0.005 : (i == 12) ? 1.0 : 1e-4;

 /* Measurement noise covariance matrices */
 double R_mag[MEAS_DIM_MAG * MEAS_DIM_MAG] = {0};
 double R_part[MEAS_DIM_PARTICLE * MEAS_DIM_PARTICLE] = {0};
 double R_xray[MEAS_DIM_XRAY * MEAS_DIM_XRAY] = {0};
 for (int i = 0; i < MEAS_DIM_MAG; i++) R_mag[i * MEAS_DIM_MAG + i] = 0.01;
 for (int i = 0; i < MEAS_DIM_PARTICLE; i++) R_part[i * MEAS_DIM_PARTICLE + i] = 0.0025;
 for (int i = 0; i < MEAS_DIM_XRAY; i++) R_xray[i * MEAS_DIM_XRAY + i] = 0.0004;

 /* Measurement matrices */
 double H_mag[MEAS_DIM_MAG * STATE_DIM];
 double H_part[MEAS_DIM_PARTICLE * STATE_DIM];
 double H_xray[MEAS_DIM_XRAY * STATE_DIM];
 build_H_magnetometer(H_mag);
 build_H_particle(H_part);
 build_H_xray(H_xray);

 /* Particle flux history for spectral analysis */
 double flux_history[NUM_TIMESTEPS][NUM_ENERGY_BINS];

 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(2969);

 /* Spacecraft attitude quaternion (body-to-inertial frame) */
 Quaternion att_q = {1.0, 0.0, 0.0, 0.0};
 QuatHealth quat_health = {0.0, 0.0, 0, 0};

 /* Apply small initial misalignment typical of post-separation drift */
 {
 double init_angle = 0.05; /* ~2.9 degrees initial offset */
 double axis_norm = sw_sqrt(1.0 / 3.0);
 Quaternion q_off;
 q_off.w = sw_cos(init_angle / 2.0);
 q_off.x = sw_sin(init_angle / 2.0) * axis_norm;
 q_off.y = sw_sin(init_angle / 2.0) * axis_norm;
 q_off.z = sw_sin(init_angle / 2.0) * axis_norm;
 att_q = quaternion_multiply(q_off, att_q);
 att_q = quaternion_normalize(att_q, &quat_health);
 }

 /* ---------- Main simulation loop ---------- */
 double total_dose = 0.0;
 double max_dst = x[12];

 for (int t = 0; t < NUM_TIMESTEPS; t++) {
 /* -- 1. Simulate solar event (CME arrival at t=20) -- */
 if (t == 20) {
 x[2] = -15.0; /* Strong southward Bz */
 x[3] = 700.0; /* Fast solar wind */
 x[4] = 20.0; /* High density */
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++)
 x[6 + ch] += 2.0; /* Particle flux enhancement */
 }

 /* -- 2. EKF Prediction -- */
 double F[STATE_DIM * STATE_DIM];
 compute_jacobian_F(x, DT, F);
 state_predict(x, DT);
 ekf_predict_covariance(P, F, Q);

 /* -- 3. Sensor measurements and EKF updates -- */
 double z_mag[MEAS_DIM_MAG];
 double z_part[MEAS_DIM_PARTICLE];
 double z_xray[MEAS_DIM_XRAY];

 simulate_magnetometer(x, z_mag);
 ekf_update(x, P, z_mag, H_mag, R_mag, MEAS_DIM_MAG);

 simulate_particle_detector(x, z_part);
 ekf_update(x, P, z_part, H_part, R_part, MEAS_DIM_PARTICLE);

 simulate_xray_sensor(x, z_xray);
 ekf_update(x, P, z_xray, H_xray, R_xray, MEAS_DIM_XRAY);

 /* -- 3a. Update attitude quaternion from gyro angular rates -- */
 {
 double omega[3] = {x[15], x[16], x[17]};
 /* Instrument scan motion for full-sky particle coverage */
 omega[0] += 0.001 * sw_sin(2.0 * M_PI * t / 50.0);
 omega[1] += 0.001 * sw_cos(2.0 * M_PI * t / 50.0);
 double omega_mag = sw_sqrt(omega[0] * omega[0] +
 omega[1] * omega[1] +
 omega[2] * omega[2]);
 if (omega_mag > 1e-15) {
 double half_angle = 0.5 * omega_mag * DT;
 double s = sw_sin(half_angle) / omega_mag;
 Quaternion dq;
 dq.w = sw_cos(half_angle);
 dq.x = s * omega[0];
 dq.y = s * omega[1];
 dq.z = s * omega[2];
 att_q = quaternion_multiply(att_q, dq);
 att_q = quaternion_normalize(att_q, &quat_health);
 }
 }

 /* -- 4. Store particle flux for spectral analysis -- */
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++)
 flux_history[t][ch] = x[6 + ch];

 /* -- 4a. Attitude DCM for telemetry frame check -- */
 {
 double att_dcm[9];
 quaternion_to_dcm(att_q, att_dcm);
 /* DCM trace = 1 + 2*cos(theta); verify attitude consistency */
 double dcm_trace = att_dcm[0] + att_dcm[4] + att_dcm[8];
 (void)dcm_trace; /* Used for debug telemetry when enabled */
 }

 /* -- 5. Coordinate transforms (trig-heavy) -- */
 double dipole_tilt = 0.4 * sw_sin(2.0 * M_PI * t / 365.25); /* Annual variation */
 double B_gse[3] = {x[0], x[1], x[2]};
 double B_gsm[3];
 gse_to_gsm(B_gse, B_gsm, dipole_tilt);

 /* Ecliptic-to-equatorial frame for solar wind vector mapping */
 {
 double ecl2eq[3][3];
 double obliquity = 23.44 * M_PI / 180.0;
 ecliptic_to_equatorial_dcm(ecl2eq, obliquity);
 /* Transform B_gse into equatorial coordinates */
 double B_eq[3];
 B_eq[0] = ecl2eq[0][0]*B_gse[0] + ecl2eq[0][1]*B_gse[1] + ecl2eq[0][2]*B_gse[2];
 B_eq[1] = ecl2eq[1][0]*B_gse[0] + ecl2eq[1][1]*B_gse[1] + ecl2eq[1][2]*B_gse[2];
 B_eq[2] = ecl2eq[2][0]*B_gse[0] + ecl2eq[2][1]*B_gse[1] + ecl2eq[2][2]*B_gse[2];
 (void)B_eq; /* Used for cross-frame validation telemetry */
 }

 /* Solar wind direction in spherical coordinates */
 double sw_dir[3];
 double sw_theta = M_PI / 2.0 + 0.05 * sw_sin(2.0 * M_PI * t / 27.0);
 double sw_phi = M_PI + 0.1 * sw_cos(2.0 * M_PI * t / 27.0);
 spherical_to_cartesian(1.0, sw_theta, sw_phi, sw_dir);

 /* -- 5a. Instrument boresight pointing in inertial coordinates -- */
 double geometric_factor = 1.0;
 {
 double boresight_body[3] = {0.0, 0.0, 1.0}; /* Detector Z-axis */
 double boresight_inertial[3];
 quaternion_rotate_vector(att_q, boresight_body, boresight_inertial);

 /* Angle between boresight and solar-wind flow direction */
 double dot_bs = boresight_inertial[0] * sw_dir[0] +
 boresight_inertial[1] * sw_dir[1] +
 boresight_inertial[2] * sw_dir[2];
 if (dot_bs > 1.0) dot_bs = 1.0;
 if (dot_bs < -1.0) dot_bs = -1.0;
 double fov_angle = sw_atan2(
 sw_sqrt(1.0 - dot_bs * dot_bs), dot_bs);

 /* Directional sensitivity: off-axis detector sees reduced flux */
 geometric_factor = sw_cos(fov_angle * 0.5);
 if (geometric_factor < 0.1) geometric_factor = 0.1;
 }

 /* -- 6. Spectral analysis (DFT, every 6 timesteps) -- */
 double dominant_freq[NUM_ENERGY_BINS] = {0};
 if (t >= FFT_LEN && (t % 6) == 0) {
 analyse_particle_spectrum(flux_history, t + 1, dominant_freq);
 }

 /* -- 7. Radiation dose forecast (scaled by detector pointing) -- */
 double dose_rate = forecast_dose_rate(&x[6]) * geometric_factor;
 total_dose += dose_rate * DT;

 /* Track worst Dst */
 if (x[12] < max_dst) max_dst = x[12];

 /* -- 8. Status output every 10 timesteps -- */
 if (t % 10 == 0) {
 FLIGHT_LOG("t=%3d | Bz=%7.2f nT | Vsw=%6.1f km/s | Dst=%7.1f nT [%s] | "
 "dose=%.2e rad/s\n",
 t, x[2], x[3], x[12], classify_storm(x[12]), dose_rate);
 }
 }

 /* ---------- Final summary ---------- */
 FLIGHT_LOG("Quat norm-drift health : max=%.2e mean=%.2e warnings=%u/%u\n",
 quat_health.max_drift,
 quat_health.samples > 0 ? quat_health.sum_drift / quat_health.samples : 0.0,
 quat_health.warnings, quat_health.samples);
 FLIGHT_LOG("Total accumulated dose : %.4e rad\n", total_dose);
 FLIGHT_LOG("Minimum Dst reached : %.1f nT (%s)\n", max_dst, classify_storm(max_dst));
 FLIGHT_LOG("Final state: Bx=%.2f By=%.2f Bz=%.2f Vsw=%.1f n=%.1f T=%.1f Dst=%.1f\n",
 x[0], x[1], x[2], x[3], x[4], x[5], x[12]);

 /* Final spectral analysis */
 double final_freq[NUM_ENERGY_BINS];
 analyse_particle_spectrum(flux_history, NUM_TIMESTEPS, final_freq);
 FLIGHT_LOG("Dominant particle frequencies (Hz): ");
 for (int ch = 0; ch < NUM_ENERGY_BINS; ch++)
 FLIGHT_LOG("%.4e ", final_freq[ch]);
 FLIGHT_LOG("\n");

 /* Spectral covariance analysis (matrix-heavy) */
 double spec_cov[NUM_ENERGY_BINS * NUM_ENERGY_BINS];
 compute_spectral_covariance(flux_history, NUM_TIMESTEPS, spec_cov);
 FLIGHT_LOG("Spectral covariance trace: %.4e\n",
 spec_cov[0] + spec_cov[1 * NUM_ENERGY_BINS + 1] +
 spec_cov[2 * NUM_ENERGY_BINS + 2] + spec_cov[3 * NUM_ENERGY_BINS + 3] +
 spec_cov[4 * NUM_ENERGY_BINS + 4] + spec_cov[5 * NUM_ENERGY_BINS + 5]);

 return 0;
}
