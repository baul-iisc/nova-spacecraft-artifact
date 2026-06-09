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
 * Satellite Positioning System Algorithm
 * HW MAT VARIANT: All matrix ops use 3×3 AME accelerator
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the onboard satellite positioning and orbit determination algorithm
 * for navigation satellite constellations. This workload performs precise orbit
 * determination using Kalman-filtered pseudorange and carrier-phase observables,
 * gravitational perturbation modeling, and satellite clock correction. Directly
 * representative of the regional navigation satellite (Navigation with Indian Constellation) / regional navigation system
 * onboard processing payload for regional navigation services.
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
#include <time.h>
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
/* Constants for Earth parameters and orbital mechanics */
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)
#define EARTH_MU 3.986004418e14 /* Earth's gravitational parameter (m^3/s^2) */
#define EARTH_RADIUS 6378137.0 /* Earth's radius (m) */
#define EARTH_J2 0.00108263 /* Earth's J2 coefficient */
#define EARTH_ROT_RATE 7.2921158e-5 /* Earth's rotation rate (rad/s) */
#define MAX_ITERATIONS 20 /* Max iterations for iterative algorithms */
#define CONVERGENCE_THRESHOLD 1.0e-8 /* Convergence threshold */
#define C_LIGHT 299792458.0 /* Speed of light (m/s) */

/* Navigation filter dimensions */
#define STATE_DIM 15 /* pos(3)+vel(3)+accel(3)+clock_bias(1)+clock_drift(1)+drag(1)+SRP(1)+troposphere(1)+iono(1) */
#define MEASUREMENT_DIM 16 /* 4 satellites × 4 measurements (range, doppler, az, el) */
#define MAX_SATELLITES 24 /* Maximum number of visible satellites */

/* L5 signal parameters */
#define GNSS_L5_FREQ 1176.45e6 /* L5 carrier frequency (Hz) */
#define GNSS_L5_LAMBDA (C_LIGHT / GNSS_L5_FREQ) /* L5 wavelength (m) */

/* Structure for Date and Time */
typedef struct {
 int year;
 int month;
 int day;
 int hour;
 int minute;
 double second;
} DateTime;

/* Structure for Satellite State */
typedef struct {
 double position[3]; /* Position in ECEF (Earth-Centered Earth-Fixed) frame (m) */
 double velocity[3]; /* Velocity in ECEF frame (m/s) */
 double acceleration[3]; /* Acceleration in ECEF frame (m/s^2) */
 double clock_bias; /* Clock bias (s) */
 double clock_drift; /* Clock drift (s/s) */
 double clock_drift_rate; /* Clock drift rate (s/s^2) */
} SatelliteState;

/* Structure for Satellite Ephemeris */
typedef struct {
 double semi_major_axis; /* Semi-major axis (m) */
 double eccentricity; /* Eccentricity */
 double inclination; /* Inclination (rad) */
 double raan; /* Right Ascension of Ascending Node (rad) */
 double arg_perigee; /* Argument of perigee (rad) */
 double mean_anomaly; /* Mean anomaly (rad) */
 double time_epoch; /* Time of epoch (s) */
 double mean_motion; /* Mean motion (rad/s) */
 double af0; /* SV clock bias (s) */
 double af1; /* SV clock drift (s/s) */
 double af2; /* SV clock drift rate (s/s^2) */
} SatelliteEphemeris;

/* Structure for Spacecraft and External Satellite Information */
typedef struct {
 SatelliteState state; /* Current state */
 SatelliteEphemeris ephemeris; /* Orbital parameters */
 double covariance[STATE_DIM][STATE_DIM]; /* State covariance matrix */
 int satellite_id; /* Satellite identifier */
} Spacecraft;

/* GCRS→ITRF rotation matrix — precomputed once per navigation epoch.
 * Encapsulates IAU-2006 precession, simplified IAU-2000A nutation,
 * Earth rotation (GAST), and IERS polar motion corrections.
 * Without these, the simple GMST rotation introduces multi-meter
 * errors in navigation-grade orbit determination. */
typedef struct {
 double R[3][3];
} EciEcefRotation;

/* Function Prototypes */

/* Matrix Operations */
void matrix_multiply(double *A, double *B, double *C, int rows_a, int cols_a, int cols_b);
void matrix_add(double *A, double *B, double *C, int rows, int cols);
void matrix_transpose(double *A, double *At, int rows, int cols);
int matrix_inverse(double *A, double *Ainv, int n);

/* Vector Operations */
double vector_dot(double *a, double *b, int n);
void vector_cross(double a[3], double b[3], double c[3]);
double vector_magnitude(double *a, int n);

/* Coordinate Transformations */
void compute_eci_ecef_rotation(DateTime time, double x_p_arcsec, double y_p_arcsec,
 EciEcefRotation *rot);
void eci_to_ecef(double eci[3], double ecef[3], const EciEcefRotation *rot);
void ecef_to_geodetic(double ecef[3], double *lat, double *lon, double *alt);
void ecef_to_enu(double ecef[3], double ref_ecef[3], double enu[3]);

/* Time Systems */
double julian_date(DateTime time);
double greenwich_mean_sidereal_time(DateTime time);

/* Orbit Propagation */
void orbital_elements_to_state_vector(SatelliteEphemeris *eph, double time, double state_vector[6]);
void state_vector_to_orbital_elements(double state_vector[6], SatelliteEphemeris *eph);
void propagate_orbit_rk4(Spacecraft *sc, double dt);
void calculate_acceleration(double position[3], double velocity[3], double acceleration[3]);

/* Measurement Models */
void compute_expected_measurements(Spacecraft *sc, Spacecraft satellites[], int num_satellites,
 double expected_measurements[], int *measurement_count);
void measurement_residuals(double actual_measurements[], double expected_measurements[],
 double residuals[], int measurement_count);
void compute_measurement_jacobian(Spacecraft *sc, Spacecraft satellites[], int num_satellites,
 double *jacobian, int *measurement_count);

/* Estimation Algorithms */
void initialize_state(Spacecraft *sc);
void initialize_covariance(Spacecraft *sc);
void extended_kalman_filter(Spacecraft *sc, double measurements[], int measurement_count,
 double *jacobian, double measurement_noise[],
 double expected_meas[]);

/* Relative Navigation */
void compute_relative_state(Spacecraft *sc1, Spacecraft *sc2, double relative_state[]);
void propagate_relative_orbit(Spacecraft *sc1, Spacecraft *sc2, double dt);
void hill_clohessy_wiltshire_propagation(double state_vector[6], double reference_mean_motion, double dt, double propagated_state[6]);

/* ODP Implementation */

/* RODP Implementation */

/* Constellation Health Assessment (Decision BCE) */
int evaluate_constellation_health(Spacecraft *sc, Spacecraft satellites[], int num_satellites,
 double *jacobian, int measurement_count, double residuals[]);

/* Main function and utility functions */
void print_spacecraft_state(Spacecraft *sc);


/* ------------------------------ IMPLEMENTATION ------------------------------ */

/* ================================================================
 * HARDWARE 3×3 TILE HELPERS
 * Single-tile operations used by the block-inverse and tiled
 * multiply/add kernels below.
 * ================================================================ */

/* HW 3×3 multiply: C = A · B via SYS_MMACD (computes C += A × B (systolic),
 * so B is loaded directly (systolic, no transpose)). */
static void hw_mul3(const double A[9], const double B[9], double C[9])
{
 const long stride = 24;
 memset(C, 0, 9 * sizeof(double));
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 "mv t2, %2\n\t"
 "mv t3, %3\n\t"
 MLDS(M0, T0, T3)
 MLDS(M1, T1, T3)
 MZERO(M2)
 SYS_MMACD(M2, M0, M1)
 MSDS(M2, T2, T3)
 :
 : "r"(A), "r"(B), "r"(C), "r"(stride)
 : "t0", "t1", "t2", "t3", "memory"
 );
}

/* HW 3×3 inverse via MINVD */
static void hw_inv3(const double A[9], double Ainv[9])
{
 const long stride = 24;
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 "mv t2, %2\n\t"
 MLDS(M0, T0, T2)
 MINVD(M1, M0)
 MSDS(M1, T1, T2)
 :
 : "r"(A), "r"(Ainv), "r"(stride)
 : "t0", "t1", "t2", "memory"
 );
}

/* HW 3×3 add: C = A + B */
static void hw_add3(const double A[9], const double B[9], double C[9])
{
 const long stride = 24;
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 "mv t2, %2\n\t"
 "mv t3, %3\n\t"
 MLDS(M0, T0, T3)
 MLDS(M1, T1, T3)
 MADDFD(M2, M0, M1)
 MSDS(M2, T2, T3)
 :
 : "r"(A), "r"(B), "r"(C), "r"(stride)
 : "t0", "t1", "t2", "t3", "memory"
 );
}

/* HW 3×3 subtract: C = A - B */
static void hw_sub3(const double A[9], const double B[9], double C[9])
{
 const long stride = 24;
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 "mv t2, %2\n\t"
 "mv t3, %3\n\t"
 MLDS(M0, T0, T3)
 MLDS(M1, T1, T3)
 MSUBFD(M2, M0, M1)
 MSDS(M2, T2, T3)
 :
 : "r"(A), "r"(B), "r"(C), "r"(stride)
 : "t0", "t1", "t2", "t3", "memory"
 );
}

/* ================================================================
 * GENERIC TILED MATRIX MULTIPLY — C(M×N) = A(M×K) · B(K×N)
 *
 * Uses 3×3 SYS_MMACD tiles for the aligned portion.
 * Handles arbitrary dimensions: SW fallback for remainder rows/cols
 * when dimensions are not exact multiples of 3.
 *
 * STATE_DIM=15 (5×3) is an exact multiple of 3.
 * MEASUREMENT_DIM=16 is NOT — remainder tiles use SW.
 * ================================================================ */
static void hw_matmul(const double *A, const double *B, double *C,
 int M, int K, int N)
{
 /* ── Optimized: pre-transpose B → direct MLDS/MSDS (zero gather/scatter) ── */
 double *BT = (double *)flight_malloc_impl((size_t)N * K * sizeof(double));
 memset(C, 0, (size_t)M * N * sizeof(double));

 const long stride_k = K * (long)sizeof(double);
 const long stride_n = N * (long)sizeof(double);
 const long s24 = 24;

 int m3a = (M / 3) * 3;
 int n3a = (N / 3) * 3;
 int k3a = (K / 3) * 3;

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
 :: "r"(stride_k), "r"(&A[i * K + kk]),
 "r"(stride_n), "r"(&B[kk * N + j])
 : "t0", "t1", "memory"
 );
 }

 /* K-remainder tile (gather with zero-pad) */
 if (k3a < K) {
 double a_pad[9] = {0}, b_pad[9] = {0};
 for (int r = 0; r < 3; r++)
 for (int c = 0; c < K - k3a; c++) {
 a_pad[r * 3 + c] = A[(i + r) * K + k3a + c];
 b_pad[r * 3 + c] = B[(j + r) * K + k3a + c];
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
 :: "r"(stride_n), "r"(&C[i * N + j])
 : "t0", "t1", "memory"
 );
 }

 /* N-remainder columns — SW fallback */
 for (int j = n3a; j < N; j++)
 for (int r = 0; r < 3; r++)
 for (int kk = 0; kk < K; kk++)
 C[(i + r) * N + j] += A[(i + r) * K + kk] * B[kk * N + j];
 }

 /* M-remainder rows — SW fallback */
 for (int i = m3a; i < M; i++)
 for (int j = 0; j < N; j++)
 for (int kk = 0; kk < K; kk++)
 C[i * N + j] += A[i * K + kk] * B[kk * N + j];
}


/* HW tiled element-wise add: C = A + B */
static void hw_elem_add(const double *A, const double *B, double *C, int len)
{
 const long stride = 24;
 int i = 0;
 for (; i + 9 <= len; i += 9) {
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 "mv t2, %2\n\t"
 "mv t3, %3\n\t"
 MLDS(M0, T0, T3)
 MLDS(M1, T1, T3)
 MADDFD(M2, M0, M1)
 MSDS(M2, T2, T3)
 :
 : "r"(&A[i]), "r"(&B[i]), "r"(&C[i]), "r"(stride)
 : "t0", "t1", "t2", "t3", "memory"
 );
 }
 for (; i < len; i++) C[i] = A[i] + B[i];
}

/* HW tiled element-wise sub: C = A - B */
static void hw_elem_sub(const double *A, const double *B, double *C, int len)
{
 const long stride = 24;
 int i = 0;
 for (; i + 9 <= len; i += 9) {
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 "mv t2, %2\n\t"
 "mv t3, %3\n\t"
 MLDS(M0, T0, T3)
 MLDS(M1, T1, T3)
 MSUBFD(M2, M0, M1)
 MSDS(M2, T2, T3)
 :
 : "r"(&A[i]), "r"(&B[i]), "r"(&C[i]), "r"(stride)
 : "t0", "t1", "t2", "t3", "memory"
 );
 }
 for (; i < len; i++) C[i] = A[i] - B[i];
}

/* ================================================================
 * BLOCK INVERSE HIERARCHY
 *
 * inv_6x6: [3×3|3×3; 3×3|3×3] → 2 MINVD
 * inv_9x9: [6×6|6×3; 3×6|3×3] → 3 MINVD (2 inside inv_6x6 + 1)
 * inv_15x15: [9×9|9×6; 6×9|6×6] → 5 MINVD (3 from inv_9x9 + 2 from inv_6x6)
 *
 * For n not a multiple of 3, fall back to SW Gauss-Jordan.
 * ================================================================ */

/* 6×6 inverse via block Schur complement: [3|3] → 2 MINVD */
static void inv_6x6(const double *A, double *Ainv)
{
 const int n = 6;
 double blkA[9], blkB[9], blkC[9], blkD[9];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 blkA[i*3+j] = A[i*n + j];
 blkB[i*3+j] = A[i*n + (j+3)];
 blkC[i*3+j] = A[(i+3)*n + j];
 blkD[i*3+j] = A[(i+3)*n + (j+3)];
 }

 double Ainv_blk[9]; hw_inv3(blkA, Ainv_blk);
 double CAinv[9]; hw_mul3(blkC, Ainv_blk, CAinv);
 double CAinvB[9]; hw_mul3(CAinv, blkB, CAinvB);
 double S[9]; hw_sub3(blkD, CAinvB, S);
 double Sinv[9]; hw_inv3(S, Sinv);
 double AinvB[9]; hw_mul3(Ainv_blk, blkB, AinvB);
 double AinvBSinv[9]; hw_mul3(AinvB, Sinv, AinvBSinv);
 double SinvCAinv[9]; hw_mul3(Sinv, CAinv, SinvCAinv);
 double AinvBSinvCAinv[9]; hw_mul3(AinvBSinv, CAinv, AinvBSinvCAinv);
 double topLeft[9]; hw_add3(Ainv_blk, AinvBSinvCAinv, topLeft);

 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 Ainv[i*n + j] = topLeft[i*3+j];
 Ainv[i*n + (j+3)] = -AinvBSinv[i*3+j];
 Ainv[(i+3)*n + j] = -SinvCAinv[i*3+j];
 Ainv[(i+3)*n + (j+3)] = Sinv[i*3+j];
 }
}

/* 9×9 inverse via block Schur: [6×6|6×3; 3×6|3×3] → 3 MINVD */
static void inv_9x9(const double *M, double *Minv)
{
 const int n = 9;
 double A_blk[36], B_blk[18], C_blk[18], D_blk[9];

 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 6; j++)
 A_blk[i*6+j] = M[i*n + j];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 3; j++)
 B_blk[i*3+j] = M[i*n + (j+6)];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 6; j++)
 C_blk[i*6+j] = M[(i+6)*n + j];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 D_blk[i*3+j] = M[(i+6)*n + (j+6)];

 double A_inv[36]; inv_6x6(A_blk, A_inv);
 double CA_inv[18]; hw_matmul(C_blk, A_inv, CA_inv, 3, 6, 6);
 double CA_inv_B[9]; hw_matmul(CA_inv, B_blk, CA_inv_B, 3, 6, 3);
 double S[9]; hw_sub3(D_blk, CA_inv_B, S);
 double S_inv[9]; hw_inv3(S, S_inv);
 double A_inv_B[18]; hw_matmul(A_inv, B_blk, A_inv_B, 6, 6, 3);
 double A_inv_B_Sinv[18]; hw_matmul(A_inv_B, S_inv, A_inv_B_Sinv, 6, 3, 3);
 double Sinv_CA_inv[18]; hw_matmul(S_inv, CA_inv, Sinv_CA_inv, 3, 3, 6);
 double correction[36]; hw_matmul(A_inv_B_Sinv, CA_inv, correction, 6, 3, 6);
 double top_left[36]; hw_elem_add(A_inv, correction, top_left, 36);

 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 6; j++)
 Minv[i*n + j] = top_left[i*6+j];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 3; j++)
 Minv[i*n + (j+6)] = -A_inv_B_Sinv[i*3+j];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 6; j++)
 Minv[(i+6)*n + j] = -Sinv_CA_inv[i*6+j];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 Minv[(i+6)*n + (j+6)] = S_inv[i*3+j];
}

/* ================================================================
 * 15×15 MATRIX INVERSE via block Schur complement
 *
 * Partition as [A(9×9) B(9×6)]
 * [C(6×9) D(6×6)]
 *
 * inv_9x9(A): 3 MINVD
 * Schur S = D – C·A⁻¹·B → inv_6x6(S): 2 MINVD
 * Total: 5 MINVD calls
 * ================================================================ */
static void inv_15x15(const double *M, double *Minv)
{
 const int n = 15;

 /* Extract blocks from 15×15 matrix */
 double A_blk[81]; /* 9×9 top-left */
 double B_blk[54]; /* 9×6 top-right */
 double C_blk[54]; /* 6×9 bottom-left */
 double D_blk[36]; /* 6×6 bottom-right */

 for (int i = 0; i < 9; i++)
 for (int j = 0; j < 9; j++)
 A_blk[i*9+j] = M[i*n + j];
 for (int i = 0; i < 9; i++)
 for (int j = 0; j < 6; j++)
 B_blk[i*6+j] = M[i*n + (j+9)];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 9; j++)
 C_blk[i*9+j] = M[(i+9)*n + j];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 6; j++)
 D_blk[i*6+j] = M[(i+9)*n + (j+9)];

 /* Step 1: A_inv = inv(A_blk) — 9×9 via inv_9x9 (3 MINVD) */
 double A_inv[81];
 inv_9x9(A_blk, A_inv);

 /* Step 2: CA_inv = C(6×9) · A_inv(9×9) → (6×9) */
 double CA_inv[54];
 hw_matmul(C_blk, A_inv, CA_inv, 6, 9, 9);

 /* Step 3: CA_inv_B = CA_inv(6×9) · B(9×6) → (6×6) */
 double CA_inv_B[36];
 hw_matmul(CA_inv, B_blk, CA_inv_B, 6, 9, 6);

 /* Step 4: S = D - CA_inv_B (6×6 Schur complement) */
 double S[36];
 hw_elem_sub(D_blk, CA_inv_B, S, 36);

 /* Step 5: S_inv = inv(S) — 6×6 via inv_6x6 (2 MINVD) */
 double S_inv[36];
 inv_6x6(S, S_inv);

 /* Step 6: A_inv_B = A_inv(9×9) · B(9×6) → (9×6) */
 double A_inv_B[54];
 hw_matmul(A_inv, B_blk, A_inv_B, 9, 9, 6);

 /* Step 7: A_inv_B_Sinv = A_inv_B(9×6) · S_inv(6×6) → (9×6) */
 double A_inv_B_Sinv[54];
 hw_matmul(A_inv_B, S_inv, A_inv_B_Sinv, 9, 6, 6);

 /* Step 8: Sinv_CA_inv = S_inv(6×6) · CA_inv(6×9) → (6×9) */
 double Sinv_CA_inv[54];
 hw_matmul(S_inv, CA_inv, Sinv_CA_inv, 6, 6, 9);

 /* Step 9: correction = A_inv_B_Sinv(9×6) · CA_inv(6×9) → (9×9) */
 double correction[81];
 hw_matmul(A_inv_B_Sinv, CA_inv, correction, 9, 6, 9);

 /* Step 10: top_left = A_inv + correction → (9×9) */
 double top_left[81];
 hw_elem_add(A_inv, correction, top_left, 81);

 /* Assemble 15×15 result */
 for (int i = 0; i < 9; i++)
 for (int j = 0; j < 9; j++)
 Minv[i*n + j] = top_left[i*9+j];
 for (int i = 0; i < 9; i++)
 for (int j = 0; j < 6; j++)
 Minv[i*n + (j+9)] = -A_inv_B_Sinv[i*6+j];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 9; j++)
 Minv[(i+9)*n + j] = -Sinv_CA_inv[i*9+j];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 6; j++)
 Minv[(i+9)*n + (j+9)] = S_inv[i*6+j];
}

/* SW Gauss-Jordan fallback for non-3-aligned dimensions */
static int sw_matrix_inverse(double *A, double *Ainv, int n) {
 int i, j, k;
 int max_i;
 double max_val, abs_val;
 double temp;
 double pivot;

 double *aug = (double *)flight_malloc_impl(2 * n * n * sizeof(double));
 if (aug == NULL) return -1;

 for (i = 0; i < n; i++)
 for (j = 0; j < n; j++) {
 aug[i * (2 * n) + j] = A[i * n + j];
 aug[i * (2 * n) + (n + j)] = (i == j) ? 1.0 : 0.0;
 }

 for (i = 0; i < n; i++) {
 max_i = i;
 max_val = fabs(aug[i * (2 * n) + i]);
 for (k = i + 1; k < n; k++) {
 abs_val = fabs(aug[k * (2 * n) + i]);
 if (abs_val > max_val) { max_i = k; max_val = abs_val; }
 }
 if (max_val < 1e-10) { flight_free_impl(aug); return -1; }
 if (max_i != i)
 for (j = 0; j < 2 * n; j++) {
 temp = aug[i * (2 * n) + j];
 aug[i * (2 * n) + j] = aug[max_i * (2 * n) + j];
 aug[max_i * (2 * n) + j] = temp;
 }
 pivot = aug[i * (2 * n) + i];
 for (j = 0; j < 2 * n; j++) aug[i * (2 * n) + j] /= pivot;
 for (k = 0; k < n; k++) {
 if (k != i) {
 temp = aug[k * (2 * n) + i];
 for (j = 0; j < 2 * n; j++)
 aug[k * (2 * n) + j] -= temp * aug[i * (2 * n) + j];
 }
 }
 }

 for (i = 0; i < n; i++)
 for (j = 0; j < n; j++)
 Ainv[i * n + j] = aug[i * (2 * n) + (n + j)];

 flight_free_impl(aug);
 return 0;
}

/* ================================================================
 * MATRIX OPERATIONS — HW 3×3 tile implementations
 *
 * Generic functions matching original API signatures.
 * Uses HW tiles where dimensions allow, SW fallback otherwise.
 * ================================================================ */

/*
 * Matrix Operations
 * These functions implement common matrix operations for large matrices.
 * All matrices are stored in row-major order as 1D arrays for cache efficiency.
 */

/* Matrix multiplication: C = A * B (HW tiled) */
void matrix_multiply(double *A, double *B, double *C, int rows_a, int cols_a, int cols_b) {
 hw_matmul(A, B, C, rows_a, cols_a, cols_b);
}

/* Matrix addition: C = A + B (HW tiled element-wise) */
void matrix_add(double *A, double *B, double *C, int rows, int cols) {
 hw_elem_add(A, B, C, rows * cols);
}

/* Matrix subtraction: C = A - B */

/* Matrix transpose: At = A' (no HW transpose — pure SW) */
void matrix_transpose(double *A, double *At, int rows, int cols) {
 int i, j;

 for (i = 0; i < rows; i++) {
 for (j = 0; j < cols; j++) {
 At[j * rows + i] = A[i * cols + j];
 }
 }
}

/* Matrix inverse — dispatches to HW block Schur for n=3,6,9,15
 * or SW Gauss-Jordan fallback for other sizes (e.g. n=16).
 * Returns 0 if successful, -1 if singular.
 */
int matrix_inverse(double *A, double *Ainv, int n) {
 /* Policy: always use the SW Gauss-Jordan implementation for inversion.
  * The hardware inverse path (e.g., MINVD-based block Schur) is disabled. */
 return sw_matrix_inverse(A, Ainv, n);
}

/* Matrix scalar multiplication: B = scalar * A */

/* Matrix determinant using LU decomposition
 * Only implemented for small matrices (n <= 4)
 * O(n!) cofactor expansion; sufficient for attitude and orbit state dimensions
 */

/* Copy matrix: dst = src */

/* Create identity matrix */

/*
 * Vector Operations
 * Basic vector operations for 3D and n-dimensional vectors.
 */

/* Vector addition: c = a + b */

/* Vector subtraction: c = a - b */

/* Vector dot product: return a·b */
double vector_dot(double *a, double *b, int n) {
 int i;
 double result = 0.0;

 for (i = 0; i < n; i++) {
 result += a[i] * b[i];
 }

 return result;
}

/* Vector cross product: c = a × b (only for 3D vectors) */
void vector_cross(double a[3], double b[3], double c[3]) {
 c[0] = a[1] * b[2] - a[2] * b[1];
 c[1] = a[2] * b[0] - a[0] * b[2];
 c[2] = a[0] * b[1] - a[1] * b[0];
}

/* Vector magnitude: |a| */
double vector_magnitude(double *a, int n) {
 int i;
 double sum = 0.0;

 for (i = 0; i < n; i++) {
 sum += a[i] * a[i];
 }

 return sqrt(sum);
}

/* Vector normalization: a = a/|a| */

/*
 * Coordinate Transformations
 * These functions handle transformations between different coordinate systems.
 */

/* Compute full GCRS→ITRF rotation matrix (IAU-2006/2000A).
 *
 * Implements the IAU-recommended transformation chain:
 * ITRF = W · R₃(GAST) · N · P · GCRS
 *
 * where:
 * P = IAU-2006 precession (Capitaine et al. 2003, A&A 412)
 * N = IAU-2000A nutation (4 dominant luni-solar terms)
 * R₃ = Earth rotation at GAST = GMST + equation of equinoxes
 * W = IERS polar motion (x_p, y_p from Bulletin B)
 *
 * Omitting P and N introduces multi-metre errors for navigation-grade
 * orbit determination (CDR deficiency noted by review board).
 *
 * Parameters:
 * x_p_arcsec, y_p_arcsec — polar motion from IERS Bulletin B
 * (typical magnitude ±0.3″ → ≈10 m) */
void compute_eci_ecef_rotation(DateTime time, double x_p_arcsec, double y_p_arcsec,
 EciEcefRotation *rot)
{
 double jd = julian_date(time);
 double T = (jd - 2451545.0) / 36525.0; /* Julian centuries from J2000 */
 double Tsq = T * T;
 double Tcu = Tsq * T;
 static const double AS2RAD = PI / (180.0 * 3600.0);

 /* ── IAU-2006 Precession (Capitaine et al. 2003) ────────────── */
 double zeta_A = ( 2.5976176 + 2306.0809506*T + 0.3019015*Tsq
 + 0.0179663*Tcu) * AS2RAD;
 double z_A = (-2.5976176 + 2306.0803226*T + 1.0947790*Tsq
 + 0.0182273*Tcu) * AS2RAD;
 double theta_A = (2004.1917476*T - 0.4269353*Tsq
 - 0.0418251*Tcu) * AS2RAD;

 /* P = R₃(−z_A) · R₂(θ_A) · R₃(−ζ_A) */
 double cz = cos(z_A), sz = sin(z_A);
 double ct = cos(theta_A), st = sin(theta_A);
 double czeta = cos(zeta_A), szeta = sin(zeta_A);

 double Prec[3][3];
 Prec[0][0] = cz*ct*czeta - sz*szeta;
 Prec[0][1] = -cz*ct*szeta - sz*czeta;
 Prec[0][2] = -cz*st;
 Prec[1][0] = sz*ct*czeta + cz*szeta;
 Prec[1][1] = -sz*ct*szeta + cz*czeta;
 Prec[1][2] = -sz*st;
 Prec[2][0] = st*czeta;
 Prec[2][1] = -st*szeta;
 Prec[2][2] = ct;

 /* ── IAU-2000A Nutation (4 dominant luni-solar terms) ───────── *
 * Full series has 1365 terms; these 4 capture >98 % of the
 * total nutation amplitude, sufficient for GNSS L5 OD. */
 double eps_0 = 84381.406 * AS2RAD; /* J2000 obliquity */
 double eps_A = eps_0 - (46.836769*T - 0.0001831*Tsq
 + 0.00200340*Tcu) * AS2RAD;

 /* Delaunay fundamental arguments (IAU-2003, radians) */
 double Om = fmod(450160.398036 - 6962890.5431*T, 1296000.0) * AS2RAD;
 double F = fmod(335779.526232 + 1739527262.8478*T, 1296000.0) * AS2RAD;
 double D = fmod(1072260.70369 + 1602961601.2090*T, 1296000.0) * AS2RAD;

 double dpsi = (-17.2064161 * sin(Om)
 - 1.3170906 * sin(2.0*(F - D + Om))
 - 0.2276413 * sin(2.0*(F + Om))
 + 0.2074554 * sin(2.0*Om)) * AS2RAD;

 double deps = ( 9.2052331 * cos(Om)
 + 0.5730336 * cos(2.0*(F - D + Om))
 + 0.0978459 * cos(2.0*(F + Om))
 - 0.0897492 * cos(2.0*Om)) * AS2RAD;

 /* N = R₁(−ε_total) · R₃(−Δψ) · R₁(ε_A) */
 double eps_total = eps_A + deps;
 double ce = cos(eps_A), se = sin(eps_A);
 double cet = cos(eps_total), set = sin(eps_total);
 double cdp = cos(dpsi), sdp = sin(dpsi);

 double Nut[3][3];
 Nut[0][0] = cdp;
 Nut[0][1] = -sdp * ce;
 Nut[0][2] = -sdp * se;
 Nut[1][0] = sdp * cet;
 Nut[1][1] = cdp * cet * ce + set * se;
 Nut[1][2] = cdp * cet * se - set * ce;
 Nut[2][0] = sdp * set;
 Nut[2][1] = cdp * set * ce - cet * se;
 Nut[2][2] = cdp * set * se + cet * ce;

 /* NP = N · P */
 double NP[3][3];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 NP[i][j] = 0.0;
 for (int k = 0; k < 3; k++)
 NP[i][j] += Nut[i][k] * Prec[k][j];
 }

 /* ── Earth Rotation ────────────────────────────────────────── *
 * GAST = GMST + equation of equinoxes */
 double gmst_rad = greenwich_mean_sidereal_time(time);
 double eq_equinox = dpsi * cos(eps_A)
 + 0.00264096 * AS2RAD * sin(Om)
 + 0.00006352 * AS2RAD * sin(2.0 * Om);
 double gast = gmst_rad + eq_equinox;

 double cg = cos(gast), sg = sin(gast);

 /* R₃(GAST) · NP */
 double R3NP[3][3];
 for (int j = 0; j < 3; j++) {
 R3NP[0][j] = cg * NP[0][j] + sg * NP[1][j];
 R3NP[1][j] = -sg * NP[0][j] + cg * NP[1][j];
 R3NP[2][j] = NP[2][j];
 }

 /* ── IERS Polar Motion: W = R₂(x_p) · R₁(y_p) ──────────── *
 * x_p, y_p from IERS Bulletin B (updated ~weekly by ground). */
 double xp = x_p_arcsec * AS2RAD;
 double yp = y_p_arcsec * AS2RAD;

 double cxp = cos(xp), sxp = sin(xp);
 double cyp = cos(yp), syp = sin(yp);

 double W[3][3];
 W[0][0] = cxp; W[0][1] = sxp*syp; W[0][2] = -sxp*cyp;
 W[1][0] = 0.0; W[1][1] = cyp; W[1][2] = syp;
 W[2][0] = sxp; W[2][1] = -cxp*syp; W[2][2] = cxp*cyp;

 /* Full GCRS→ITRF: R = W · R₃(GAST) · N · P */
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 rot->R[i][j] = 0.0;
 for (int k = 0; k < 3; k++)
 rot->R[i][j] += W[i][k] * R3NP[k][j];
 }
}

/* Apply precomputed GCRS→ITRF rotation to a single 3-vector. */
void eci_to_ecef(double eci[3], double ecef[3], const EciEcefRotation *rot)
{
 ecef[0] = rot->R[0][0]*eci[0] + rot->R[0][1]*eci[1] + rot->R[0][2]*eci[2];
 ecef[1] = rot->R[1][0]*eci[0] + rot->R[1][1]*eci[1] + rot->R[1][2]*eci[2];
 ecef[2] = rot->R[2][0]*eci[0] + rot->R[2][1]*eci[1] + rot->R[2][2]*eci[2];
}

/* Transform from Earth-Centered Earth-Fixed (ECEF) to Earth-Centered Inertial (ECI) */

/* Transform from ECEF to geodetic coordinates (latitude, longitude, altitude).
 * Uses the Bowring (1985) iterative method with explicit near-pole
 * singularity guard to prevent NaN when p (distance from Z-axis) → 0.
 *
 * Near-pole case: When p < 1 mm the standard formulae become
 * ill-conditioned (division by cos(lat) → 0/0). At the geographic
 * poles, latitude = ±90°, longitude is undefined (set to 0), and
 * altitude is computed directly from |z| − b (polar semi-axis). */
void ecef_to_geodetic(double ecef[3], double *lat, double *lon, double *alt) {
 /* WGS84 parameters */
 double a = EARTH_RADIUS;
 double e2 = 0.00669437999014; /* Square of first eccentricity */
 double b = a * sqrt(1.0 - e2);

 double p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);

 /* ── Near-pole singularity guard ─────────────────────────────── */
 if (p < 1.0e-3) { /* < 1 mm from Earth's spin axis */
 *lon = 0.0; /* Longitude undefined at pole; convention = 0 */
 *lat = (ecef[2] >= 0.0) ? PI / 2.0 : -PI / 2.0;
 *alt = fabs(ecef[2]) - b;
 return;
 }

 double ep2 = (a*a - b*b) / (b*b); /* Square of second eccentricity */
 double theta = atan2(ecef[2] * a, p * b);

 *lon = atan2(ecef[1], ecef[0]);
 *lat = atan2(ecef[2] + ep2 * b * pow(sin(theta), 3),
 p - e2 * a * pow(cos(theta), 3));

 double sin_lat = sin(*lat);
 double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);

 /* Use the formula that is numerically stable at the given latitude:
 * |lat| < 45° → alt = p / cos(lat) − N (equatorial)
 * |lat| ≥ 45° → alt = z / sin(lat) − N·(1 − e²) (polar) */
 if (fabs(sin_lat) > 0.707107) {
 *alt = ecef[2] / sin_lat - N * (1.0 - e2);
 } else {
 *alt = p / cos(*lat) - N;
 }
}

/* Transform from geodetic coordinates to ECEF */

/* Transform from ECEF to local East-North-Up (ENU) coordinates */
void ecef_to_enu(double ecef[3], double ref_ecef[3], double enu[3]) {
 double lat, lon, alt;
 double delta_ecef[3];
 int i;

 /* Get geodetic coordinates of reference point */
 ecef_to_geodetic(ref_ecef, &lat, &lon, &alt);

 /* Calculate ECEF differences */
 for (i = 0; i < 3; i++) {
 delta_ecef[i] = ecef[i] - ref_ecef[i];
 }

 /* Rotation matrix from ECEF to ENU */
 double sin_lat = sin(lat);
 double cos_lat = cos(lat);
 double sin_lon = sin(lon);
 double cos_lon = cos(lon);

 /* East component */
 enu[0] = -sin_lon * delta_ecef[0] + cos_lon * delta_ecef[1];

 /* North component */
 enu[1] = -sin_lat * cos_lon * delta_ecef[0] - sin_lat * sin_lon * delta_ecef[1] + cos_lat * delta_ecef[2];

 /* Up component */
 enu[2] = cos_lat * cos_lon * delta_ecef[0] + cos_lat * sin_lon * delta_ecef[1] + sin_lat * delta_ecef[2];
}

/* Transform from ECI to orbital frame */

/*
 * Time Systems
 * Functions for handling different time systems and conversions.
 */

/* Calculate Julian Date from calendar date and time */
double julian_date(DateTime time) {
 int year = time.year;
 int month = time.month;
 int day = time.day;

 /* Adjust for January and February */
 if (month <= 2) {
 year -= 1;
 month += 12;
 }

 int a = year / 100;
 int b = 2 - a + (a / 4);

 double jd = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + b - 1524.5;

 /* Add time of day */
 jd += (time.hour + time.minute / 60.0 + time.second / 3600.0) / 24.0;

 return jd;
}

/* Calculate Greenwich Mean Sidereal Time (GMST) */
double greenwich_mean_sidereal_time(DateTime time) {
 double jd = julian_date(time);
 double t = (jd - 2451545.0) / 36525.0; /* Julian centuries since J2000.0 */

 /* GMST at 0h UT formula (in degrees) */
 double gmst = 100.46061837 + 36000.770053608 * t + 0.000387933 * t * t -
 (t * t * t) / 38710000.0;

 /* Normalize to 0-360 degrees */
 gmst = fmod(gmst, 360.0);
 if (gmst < 0) {
 gmst += 360.0;
 }

 /* Add rotation since 0h UT */
 double ut_hours = time.hour + time.minute / 60.0 + time.second / 3600.0;
 gmst += ut_hours * 15.04106728; /* Earth rotates 15.04... degrees per hour */

 /* Normalize again and convert to radians */
 gmst = fmod(gmst, 360.0);
 if (gmst < 0) {
 gmst += 360.0;
 }

 return gmst * PI / 180.0;
}

/* Convert time in seconds to DateTime structure */

/* Convert DateTime to seconds since reference epoch */

/*
 * Orbit Propagation
 * Functions for propagating satellite orbits over time.
 */

/* Convert orbital elements to Cartesian state vector */
void orbital_elements_to_state_vector(SatelliteEphemeris *eph, double time, double state_vector[6]) {
 double a = eph->semi_major_axis;
 double e = eph->eccentricity;
 double i = eph->inclination;
 double Omega = eph->raan;
 double omega = eph->arg_perigee;
 double M_0 = eph->mean_anomaly;
 double n = eph->mean_motion;
 double epoch = eph->time_epoch;

 /* Compute mean anomaly at requested time */
 double dt = time - epoch;
 double M = M_0 + n * dt;
 M = fmod(M, TWO_PI);
 if (M < 0) {
 M += TWO_PI;
 }

 /* Solve Kepler's equation for eccentric anomaly */
 double E = M; /* Initial guess */
 double E_new;
 int iter;

 for (iter = 0; iter < MAX_ITERATIONS; iter++) {
 E_new = E - (E - e * sin(E) - M) / (1.0 - e * cos(E));

 if (fabs(E_new - E) < CONVERGENCE_THRESHOLD) {
 E = E_new;
 break;
 }

 E = E_new;
 }

 /* Compute true anomaly */
 double nu = 2.0 * atan2(sqrt(1.0 + e) * sin(E / 2.0), sqrt(1.0 - e) * cos(E / 2.0));

 /* Compute position in orbital plane */
 double r = a * (1.0 - e * cos(E));
 double x_orb = r * cos(nu);
 double y_orb = r * sin(nu);

 /* Compute velocity in orbital plane */
 double p = a * (1.0 - e * e);
 double sqrt_mu_over_p = sqrt(EARTH_MU / p);
 double vx_orb = -sqrt_mu_over_p * sin(nu);
 double vy_orb = sqrt_mu_over_p * (e + cos(nu));

 /* Rotation matrices for orbital plane to ECI */
 double cos_Omega = cos(Omega);
 double sin_Omega = sin(Omega);
 double cos_i = cos(i);
 double sin_i = sin(i);
 double cos_omega = cos(omega);
 double sin_omega = sin(omega);

 /* Composite rotation from orbital plane to ECI */
 double R11 = cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i;
 double R12 = -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i;
 (void)0; /* R13 = sin_Omega * sin_i; unused — orbital plane is 2D */

 double R21 = sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i;
 double R22 = -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i;
 (void)0; /* R23 = -cos_Omega * sin_i; unused — orbital plane is 2D */

 double R31 = sin_omega * sin_i;
 double R32 = cos_omega * sin_i;
 (void)0; /* R33 = cos_i; unused — orbital plane is 2D */

 /* Rotate position to ECI */
 state_vector[0] = R11 * x_orb + R12 * y_orb;
 state_vector[1] = R21 * x_orb + R22 * y_orb;
 state_vector[2] = R31 * x_orb + R32 * y_orb;

 /* Rotate velocity to ECI */
 state_vector[3] = R11 * vx_orb + R12 * vy_orb;
 state_vector[4] = R21 * vx_orb + R22 * vy_orb;
 state_vector[5] = R31 * vx_orb + R32 * vy_orb;
}

/* Convert Cartesian state vector to orbital elements */
void state_vector_to_orbital_elements(double state_vector[6], SatelliteEphemeris *eph) {
 double r[3], v[3];
 double r_mag, v_mag;
 double h[3], h_mag;
 double n[3], n_mag;
 double e_vec[3], e_mag;
 double tmp;
 int i;

 /* Extract position and velocity */
 for (i = 0; i < 3; i++) {
 r[i] = state_vector[i];
 v[i] = state_vector[i+3];
 }

 r_mag = vector_magnitude(r, 3);
 v_mag = vector_magnitude(v, 3);

 /* Angular momentum vector */
 vector_cross(r, v, h);
 h_mag = vector_magnitude(h, 3);

 /* Line of nodes (vector pointing towards ascending node) */
 double k[3] = {0.0, 0.0, 1.0}; /* Z-axis unit vector */
 vector_cross(k, h, n);
 n_mag = vector_magnitude(n, 3);

 /* Eccentricity vector */
 double rv_dot = vector_dot(r, v, 3);
 for (i = 0; i < 3; i++) {
 e_vec[i] = ((v_mag * v_mag - EARTH_MU / r_mag) * r[i] - rv_dot * v[i]) / EARTH_MU;
 }
 e_mag = vector_magnitude(e_vec, 3);

 /* Semi-major axis */
 double energy = 0.5 * v_mag * v_mag - EARTH_MU / r_mag;
 eph->semi_major_axis = -EARTH_MU / (2.0 * energy);

 /* Eccentricity */
 eph->eccentricity = e_mag;

 /* Inclination */
 eph->inclination = acos(h[2] / h_mag);

 /* Right Ascension of Ascending Node (RAAN) */
 if (n_mag < 1e-10) {
 eph->raan = 0.0; /* For circular equatorial orbits */
 } else {
 eph->raan = acos(n[0] / n_mag);
 if (n[1] < 0) {
 eph->raan = TWO_PI - eph->raan;
 }
 }

 /* Argument of Perigee */
 if (n_mag < 1e-10) {
 /* For circular equatorial orbits, use the x-axis as reference */
 eph->arg_perigee = atan2(e_vec[1], e_vec[0]);
 } else {
 tmp = vector_dot(n, e_vec, 3) / (n_mag * e_mag);
 if (tmp > 1.0) tmp = 1.0;
 if (tmp < -1.0) tmp = -1.0;
 eph->arg_perigee = acos(tmp);
 if (e_vec[2] < 0) {
 eph->arg_perigee = TWO_PI - eph->arg_perigee;
 }
 }

 /* True Anomaly */
 if (e_mag < 1e-10) {
 /* For circular orbits, measure from the ascending node */
 if (n_mag < 1e-10) {
 /* For circular equatorial orbits, measure from the x-axis */
 tmp = r[0] / r_mag;
 if (tmp > 1.0) tmp = 1.0;
 if (tmp < -1.0) tmp = -1.0;
 double true_anomaly = acos(tmp);
 if (r[1] < 0) {
 true_anomaly = TWO_PI - true_anomaly;
 }
 eph->mean_anomaly = true_anomaly; /* For circular orbits, M = ν */
 } else {
 tmp = vector_dot(n, r, 3) / (n_mag * r_mag);
 if (tmp > 1.0) tmp = 1.0;
 if (tmp < -1.0) tmp = -1.0;
 double true_anomaly = acos(tmp);
 if (vector_dot(n, v, 3) < 0) {
 true_anomaly = TWO_PI - true_anomaly;
 }
 eph->mean_anomaly = true_anomaly; /* For circular orbits, M = ν */
 }
 } else {
 tmp = vector_dot(e_vec, r, 3) / (e_mag * r_mag);
 if (tmp > 1.0) tmp = 1.0;
 if (tmp < -1.0) tmp = -1.0;
 double true_anomaly = acos(tmp);
 if (vector_dot(r, v, 3) < 0) {
 true_anomaly = TWO_PI - true_anomaly;
 }

 /* Convert true anomaly to eccentric anomaly */
 double cos_E = (e_mag + cos(true_anomaly)) / (1.0 + e_mag * cos(true_anomaly));
 double sin_E = (sqrt(1.0 - e_mag * e_mag) * sin(true_anomaly)) / (1.0 + e_mag * cos(true_anomaly));
 double E = atan2(sin_E, cos_E);

 /* Convert eccentric anomaly to mean anomaly */
 eph->mean_anomaly = E - e_mag * sin(E);
 }

 /* Normalize mean anomaly to [0, 2π) */
 eph->mean_anomaly = fmod(eph->mean_anomaly, TWO_PI);
 if (eph->mean_anomaly < 0) {
 eph->mean_anomaly += TWO_PI;
 }

 /* Mean motion */
 eph->mean_motion = sqrt(EARTH_MU / pow(eph->semi_major_axis, 3));

 /* Current time is set as epoch */
 eph->time_epoch = 0.0; /* Initialized; updated per tracking pass */
}

/* Orbital state derivative: two-body + J2 perturbation */
static void orbit_state_derivative(double s[6], double ds_dt[6]) {
 double r_squared = s[0]*s[0] + s[1]*s[1] + s[2]*s[2];
 double r_cubed = r_squared * sqrt(r_squared);

 /* Position derivative is velocity */
 ds_dt[0] = s[3];
 ds_dt[1] = s[4];
 ds_dt[2] = s[5];

 /* Velocity derivative from gravitational acceleration */
 ds_dt[3] = -EARTH_MU * s[0] / r_cubed;
 ds_dt[4] = -EARTH_MU * s[1] / r_cubed;
 ds_dt[5] = -EARTH_MU * s[2] / r_cubed;

 /* Add J2 perturbation effects */
 double r = sqrt(r_squared);
 double J2_term = 1.5 * EARTH_J2 * EARTH_MU * pow(EARTH_RADIUS / r, 2) / r_cubed;
 double z_over_r_squared = s[2] * s[2] / r_squared;

 ds_dt[3] += J2_term * s[0] * (5.0 * z_over_r_squared - 1.0);
 ds_dt[4] += J2_term * s[1] * (5.0 * z_over_r_squared - 1.0);
 ds_dt[5] += J2_term * s[2] * (5.0 * z_over_r_squared - 3.0);
}

/* Propagate orbit using RK4 integrator */
void propagate_orbit_rk4(Spacecraft *sc, double dt) {
 int i;
 double state[6];
 double k1[6], k2[6], k3[6], k4[6];
 double temp_state[6];

 /* Extract current state vector */
 for (i = 0; i < 3; i++) {
 state[i] = sc->state.position[i];
 state[i+3] = sc->state.velocity[i];
 }

 /* RK4 integration */

 /* k1 = f(t, y) */
 orbit_state_derivative(state, k1);

 /* k2 = f(t + dt/2, y + dt*k1/2) */
 for (i = 0; i < 6; i++) {
 temp_state[i] = state[i] + 0.5 * dt * k1[i];
 }
 orbit_state_derivative(temp_state, k2);

 /* k3 = f(t + dt/2, y + dt*k2/2) */
 for (i = 0; i < 6; i++) {
 temp_state[i] = state[i] + 0.5 * dt * k2[i];
 }
 orbit_state_derivative(temp_state, k3);

 /* k4 = f(t + dt, y + dt*k3) */
 for (i = 0; i < 6; i++) {
 temp_state[i] = state[i] + dt * k3[i];
 }
 orbit_state_derivative(temp_state, k4);

 /* y(t+dt) = y(t) + dt/6 * (k1 + 2*k2 + 2*k3 + k4) */
 for (i = 0; i < 6; i++) {
 state[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
 }

 /* Update spacecraft state */
 for (i = 0; i < 3; i++) {
 sc->state.position[i] = state[i];
 sc->state.velocity[i] = state[i+3];
 }

 /* Propagate clock states */
 sc->state.clock_bias += sc->state.clock_drift * dt + 0.5 * sc->state.clock_drift_rate * dt * dt;
 sc->state.clock_drift += sc->state.clock_drift_rate * dt;

 /* Calculate accelerations based on updated state */
 calculate_acceleration(sc->state.position, sc->state.velocity, sc->state.acceleration);
}

/* SGP4 simplified propagator (Simplified General Perturbations) */

/* Calculate acceleration including various perturbations */
void calculate_acceleration(double position[3], double velocity[3], double acceleration[3]) {
 double r_squared = position[0]*position[0] + position[1]*position[1] + position[2]*position[2];
 double r = sqrt(r_squared);
 double r_cubed = r_squared * r;

 /* Two-body acceleration */
 acceleration[0] = -EARTH_MU * position[0] / r_cubed;
 acceleration[1] = -EARTH_MU * position[1] / r_cubed;
 acceleration[2] = -EARTH_MU * position[2] / r_cubed;

 /* J2 perturbation (Earth's oblateness) */
 double J2_term = 1.5 * EARTH_J2 * EARTH_MU * pow(EARTH_RADIUS / r, 2) / r_cubed;
 double z_over_r_squared = position[2] * position[2] / r_squared;

 acceleration[0] += J2_term * position[0] * (5.0 * z_over_r_squared - 1.0);
 acceleration[1] += J2_term * position[1] * (5.0 * z_over_r_squared - 1.0);
 acceleration[2] += J2_term * position[2] * (5.0 * z_over_r_squared - 3.0);

 /* ── NRLMSISE-00 approximation atmospheric drag model ──────────
 * Replaces the single exponential with a piecewise fit to the
 * NRLMSISE-00 empirical model (Picone et al., JGR 2002).
 *
 * The full model requires 80+ coefficients and thermospheric
 * temperature integration. This simplified version captures:
 * (a) altitude-dependent base density and scale height
 * (9 bands from 100 km to 800+ km)
 * (b) F10.7 solar flux correction (density ∝ UV heating)
 * (c) Ap geomagnetic activity correction (Joule heating)
 *
 * In flight, F10.7 and Ap are uplinked from ground every orbit
 * via tracking network. Default values here represent
 * moderate solar activity (F10.7 = 150 SFU, Ap = 15). */
 double vel_magnitude = vector_magnitude(velocity, 3);
 double altitude = r - EARTH_RADIUS;

 if (altitude > 0.0 && altitude < 1000000.0) { /* 0 – 1000 km */
 /* Tabulated base densities (kg/m³) and scale heights (m)
 * from NRLMSISE-00 nominal (F10.7=150, Ap=15). Each band
 * uses ρ = ρ_base · exp(−(h − h_base) / H). */
 static const double atm_alt[] = { 100e3, 150e3, 200e3, 300e3,
 400e3, 500e3, 600e3, 700e3, 800e3 };
 static const double atm_rho[] = { 5.297e-7, 2.070e-9, 2.789e-10, 7.248e-11,
 2.803e-11, 1.184e-11, 5.215e-12, 2.394e-12, 1.170e-12 };
 static const double atm_H[] = { 5900.0, 22600.0, 37800.0, 45500.0,
 53500.0, 62000.0, 72000.0, 83000.0, 95000.0 };
 int n_bands = 9;

 /* Select altitude band (highest band whose base ≤ altitude) */
 int band = 0;
 for (int b = n_bands - 1; b >= 0; b--) {
 if (altitude >= atm_alt[b]) { band = b; break; }
 }

 double density = atm_rho[band] * exp(-(altitude - atm_alt[band]) / atm_H[band]);

 /* Solar flux correction: F10.7 index (SFU, range 70–250).
 * Density roughly scales linearly with (F10.7 − 70). */
 double F107 = 150.0; /* In flight: uplinked from mission control */
 double F107_factor = 1.0 + 0.5 * (F107 - 150.0) / 80.0;
 if (F107_factor < 0.5) F107_factor = 0.5;

 /* Geomagnetic activity correction (Ap index, 0–400).
 * Storm-time heating can increase density by ~2–3× at 400 km. */
 double Ap = 15.0; /* In flight: uplinked from mission control */
 double Ap_factor = 1.0 + 0.014 * (Ap - 4.0);

 density *= F107_factor * Ap_factor;

 /* Drag: a_drag = −½ · Cd · (A/m) · ρ · |v| · v̂ */
 double drag_coef = 2.2; /* Cd typical for LEO s/c */
 double area_to_mass = 0.01; /* m²/kg (e.g. 5 m² / 500 kg) */

 double drag_factor = -0.5 * drag_coef * area_to_mass * density * vel_magnitude;

 acceleration[0] += drag_factor * velocity[0];
 acceleration[1] += drag_factor * velocity[1];
 acceleration[2] += drag_factor * velocity[2];
 }

 /* Solar radiation pressure could be added here */

 /* Third-body perturbations (Moon, Sun) could be added here */
}

/*
 * Measurement Models
 * Functions for modeling measurements from satellite and ground sensors.
 */

/* Compute expected measurements based on current state */
void compute_expected_measurements(Spacecraft *sc, Spacecraft satellites[], int num_satellites,
 double expected_measurements[], int *measurement_count) {
 int i, j = 0;
 DateTime current_time;

 /* Epoch for GCRS→ITRF rotation (IAU-2006/2000A + polar motion) */
 current_time.year = 2023;
 current_time.month = 10;
 current_time.day = 15;
 current_time.hour = 12;
 current_time.minute = 0;
 current_time.second = 0.0;

 EciEcefRotation eci2ecef;
 compute_eci_ecef_rotation(current_time, 0.0, 0.0, &eci2ecef);

 /* For each satellite */
 for (i = 0; i < num_satellites; i++) {
 /* Convert spacecraft positions to ECEF for measurements */
 double sc_ecef[3], sat_ecef[3];
 eci_to_ecef(sc->state.position, sc_ecef, &eci2ecef);
 eci_to_ecef(satellites[i].state.position, sat_ecef, &eci2ecef);

 /* Pseudorange measurement */
 double dx = sat_ecef[0] - sc_ecef[0];
 double dy = sat_ecef[1] - sc_ecef[1];
 double dz = sat_ecef[2] - sc_ecef[2];
 double geometric_range = sqrt(dx*dx + dy*dy + dz*dz);

 /* Apply relativistic correction */
 double rel_correction = 2.0 * vector_dot(satellites[i].state.position, satellites[i].state.velocity, 3) / (C_LIGHT * C_LIGHT);

 /* Apply clock bias */
 double pseudorange = geometric_range +
 C_LIGHT * (sc->state.clock_bias - satellites[i].state.clock_bias) +
 rel_correction;

 /* Line-of-sight unit vector */
 double los[3] = {dx/geometric_range, dy/geometric_range, dz/geometric_range};

 /* Doppler measurement */
 double sc_vel_ecef[3], sat_vel_ecef[3];
 eci_to_ecef(sc->state.velocity, sc_vel_ecef, &eci2ecef);
 eci_to_ecef(satellites[i].state.velocity, sat_vel_ecef, &eci2ecef);

 double rel_velocity = (sat_vel_ecef[0] - sc_vel_ecef[0]) * los[0] +
 (sat_vel_ecef[1] - sc_vel_ecef[1]) * los[1] +
 (sat_vel_ecef[2] - sc_vel_ecef[2]) * los[2];

 double doppler = -rel_velocity / GNSS_L5_LAMBDA; /* L5 Doppler shift */

 /* Azimuth and elevation in local ENU frame */
 double enu[3];
 ecef_to_enu(sat_ecef, sc_ecef, enu);

 double horizontal_distance = sqrt(enu[0]*enu[0] + enu[1]*enu[1]);
 double azimuth = atan2(enu[0], enu[1]); /* North = 0, East = pi/2 (standard nav azimuth) */
 double elevation = atan2(enu[2], horizontal_distance);

 /* Store measurements */
 expected_measurements[j++] = pseudorange;
 expected_measurements[j++] = doppler;
 expected_measurements[j++] = azimuth;
 expected_measurements[j++] = elevation;

 /* Add other measurements here as needed */
 }

 *measurement_count = j;
}

/* Compute measurement residuals (observed - predicted) */
void measurement_residuals(double actual_measurements[], double expected_measurements[],
 double residuals[], int measurement_count) {
 int i;

 for (i = 0; i < measurement_count; i++) {
 residuals[i] = actual_measurements[i] - expected_measurements[i];

 /* Special handling for angular measurements to handle wrap-around */
 if (i % 4 == 2 || i % 4 == 3) { /* Azimuth or elevation */
 /* Normalize angle difference to [-pi, pi] */
 while (residuals[i] > PI) residuals[i] -= TWO_PI;
 while (residuals[i] < -PI) residuals[i] += TWO_PI;
 }
 }
}

/* Compute geometric range between two points */

/* Compute measurement Jacobian matrix for Extended Kalman Filter */
void compute_measurement_jacobian(Spacecraft *sc, Spacecraft satellites[], int num_satellites,
 double *jacobian, int *measurement_count) {
 int i, j, k, row = 0;
 DateTime current_time;

 /* Epoch for GCRS→ITRF rotation (IAU-2006/2000A + polar motion) */
 current_time.year = 2023;
 current_time.month = 10;
 current_time.day = 15;
 current_time.hour = 12;
 current_time.minute = 0;
 current_time.second = 0.0;

 EciEcefRotation eci2ecef;
 compute_eci_ecef_rotation(current_time, 0.0, 0.0, &eci2ecef);

 /* Convert spacecraft state to ECEF */
 double sc_ecef[3], sc_vel_ecef[3];
 eci_to_ecef(sc->state.position, sc_ecef, &eci2ecef);
 eci_to_ecef(sc->state.velocity, sc_vel_ecef, &eci2ecef);

 /* For each satellite */
 for (i = 0; i < num_satellites; i++) {
 /* Convert satellite state to ECEF */
 double sat_ecef[3], sat_vel_ecef[3];
 eci_to_ecef(satellites[i].state.position, sat_ecef, &eci2ecef);
 eci_to_ecef(satellites[i].state.velocity, sat_vel_ecef, &eci2ecef);

 /* Compute range and line-of-sight unit vector */
 double dx = sat_ecef[0] - sc_ecef[0];
 double dy = sat_ecef[1] - sc_ecef[1];
 double dz = sat_ecef[2] - sc_ecef[2];
 double range = sqrt(dx*dx + dy*dy + dz*dz);

 double los[3] = {dx/range, dy/range, dz/range};

 /* Jacobian for pseudorange measurement */
 for (j = 0; j < STATE_DIM; j++) {
 jacobian[row * STATE_DIM + j] = 0.0;
 }

 /* Partial derivatives of range with respect to position */
 jacobian[row * STATE_DIM + 0] = -los[0];
 jacobian[row * STATE_DIM + 1] = -los[1];
 jacobian[row * STATE_DIM + 2] = -los[2];

 /* Partial derivative with respect to clock bias */
 jacobian[row * STATE_DIM + 9] = C_LIGHT;

 row++;

 /* Jacobian for Doppler measurement */
 for (j = 0; j < STATE_DIM; j++) {
 jacobian[row * STATE_DIM + j] = 0.0;
 }

 /* Relative velocity vector */
 double dv[3] = {
 sat_vel_ecef[0] - sc_vel_ecef[0],
 sat_vel_ecef[1] - sc_vel_ecef[1],
 sat_vel_ecef[2] - sc_vel_ecef[2]
 };

 /* Partial derivatives for Doppler with respect to position */
 double d_pos[3] = {dx, dy, dz};
 for (j = 0; j < 3; j++) {
 double sum = 0.0;
 for (k = 0; k < 3; k++) {
 if (j == k) {
 sum += dv[k] * (-1.0/range + d_pos[j]*d_pos[k]/(range*range*range));
 } else {
 sum += dv[k] * (d_pos[j]*d_pos[k]/(range*range*range));
 }
 }
 jacobian[row * STATE_DIM + j] = -sum * (GNSS_L5_FREQ / C_LIGHT);
 }

 /* Partial derivatives for Doppler with respect to velocity */
 jacobian[row * STATE_DIM + 3] = +los[0] * (GNSS_L5_FREQ / C_LIGHT);
 jacobian[row * STATE_DIM + 4] = +los[1] * (GNSS_L5_FREQ / C_LIGHT);
 jacobian[row * STATE_DIM + 5] = +los[2] * (GNSS_L5_FREQ / C_LIGHT);

 /* Partial derivative with respect to clock drift: df_D/d(dt_dot) = -f_0 */
 jacobian[row * STATE_DIM + 10] = -GNSS_L5_FREQ;

 row++;

 /* Compute ENU coordinates for azimuth and elevation */
 double enu[3];
 ecef_to_enu(sat_ecef, sc_ecef, enu);

 double horiz_dist = sqrt(enu[0]*enu[0] + enu[1]*enu[1]);
 (void)horiz_dist; /* used below via numerical diff */

 /* Jacobian for azimuth measurement */
 for (j = 0; j < STATE_DIM; j++) {
 jacobian[row * STATE_DIM + j] = 0.0;
 }

 /* Partial derivatives for azimuth with respect to position are complex
 * and would require detailed computation of the ECEF to ENU transformation Jacobian
 * Numerical approximation: */
 double delta = 1.0; /* Small position delta in meters */
 double az1, az2, el1, el2;

 for (j = 0; j < 3; j++) {
 double temp_sc_ecef[3] = {sc_ecef[0], sc_ecef[1], sc_ecef[2]};
 temp_sc_ecef[j] += delta;

 double temp_enu[3];
 ecef_to_enu(sat_ecef, temp_sc_ecef, temp_enu);

 double temp_horiz = sqrt(temp_enu[0]*temp_enu[0] + temp_enu[1]*temp_enu[1]);
 az2 = atan2(temp_enu[0], temp_enu[1]);
 el2 = atan2(temp_enu[2], temp_horiz);

 temp_sc_ecef[j] -= 2.0 * delta;
 ecef_to_enu(sat_ecef, temp_sc_ecef, temp_enu);

 temp_horiz = sqrt(temp_enu[0]*temp_enu[0] + temp_enu[1]*temp_enu[1]);
 az1 = atan2(temp_enu[0], temp_enu[1]);
 el1 = atan2(temp_enu[2], temp_horiz);

 /* Normalize angle difference to [-pi, pi] */
 double daz = az2 - az1;
 while (daz > PI) daz -= TWO_PI;
 while (daz < -PI) daz += TWO_PI;

 double del = el2 - el1;

 jacobian[row * STATE_DIM + j] = daz / (2.0 * delta);
 jacobian[(row+1) * STATE_DIM + j] = del / (2.0 * delta);
 }

 row++;

 /* Jacobian row for elevation measurement already computed in the loop above */
 row++;
 }

 *measurement_count = row;
}

/*
 * Estimation Algorithms
 * Functions for state and orbit estimation based on measurements.
 */

/* Initialize spacecraft state with default values */
void initialize_state(Spacecraft *sc) {
 int i, j;

 /* Position (in meters) - arbitrary initial values */
 sc->state.position[0] = 6678000.0; /* X */
 sc->state.position[1] = 0.0; /* Y */
 sc->state.position[2] = 0.0; /* Z */

 /* Velocity (in m/s) - corresponding to circular orbit */
 double orbit_speed = sqrt(EARTH_MU / 6678000.0);
 sc->state.velocity[0] = 0.0;
 sc->state.velocity[1] = orbit_speed;
 sc->state.velocity[2] = 0.0;

 /* Acceleration - initially zero, will be computed */
 for (i = 0; i < 3; i++) {
 sc->state.acceleration[i] = 0.0;
 }

 /* Clock states */
 sc->state.clock_bias = 0.0;
 sc->state.clock_drift = 0.0;
 sc->state.clock_drift_rate = 0.0;

 /* Initialize covariance matrix to zeros */
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 sc->covariance[i][j] = 0.0;
 }
 }

 /* Compute orbital elements from state */
 double state_vector[6];
 for (i = 0; i < 3; i++) {
 state_vector[i] = sc->state.position[i];
 state_vector[i+3] = sc->state.velocity[i];
 }

 state_vector_to_orbital_elements(state_vector, &sc->ephemeris);

 /* Set epoch time */
 sc->ephemeris.time_epoch = 0.0; /* Current time */

 /* Set clock parameters */
 sc->ephemeris.af0 = 0.0;
 sc->ephemeris.af1 = 0.0;
 sc->ephemeris.af2 = 0.0;

 /* Compute acceleration */
 calculate_acceleration(sc->state.position, sc->state.velocity, sc->state.acceleration);

 /* Assign a satellite ID */
 sc->satellite_id = 0; /* Default ID */
}

/* Initialize covariance matrix with realistic uncertainties */
void initialize_covariance(Spacecraft *sc) {
 int i;

 /* Set diagonal elements representing initial uncertainties */

 /* Position uncertainty (in meters)^2 */
 for (i = 0; i < 3; i++) {
 sc->covariance[i][i] = 1000.0 * 1000.0;
 }

 /* Velocity uncertainty (in m/s)^2 */
 for (i = 3; i < 6; i++) {
 sc->covariance[i][i] = 10.0 * 10.0;
 }

 /* Acceleration uncertainty (in m/s^2)^2 */
 for (i = 6; i < 9; i++) {
 sc->covariance[i][i] = 0.1 * 0.1;
 }

 /* Clock bias uncertainty (in seconds)^2 */
 sc->covariance[9][9] = 1.0e-6 * 1.0e-6;

 /* Clock drift uncertainty (in seconds/second)^2 */
 sc->covariance[10][10] = 1.0e-9 * 1.0e-9;

 /* Clock drift rate uncertainty (in seconds/second^2)^2 */
 sc->covariance[11][11] = 1.0e-10 * 1.0e-10;

 /* Additional states if any */
 for (i = 12; i < STATE_DIM; i++) {
 sc->covariance[i][i] = 1.0;
 }
}

/* Extended Kalman Filter implementation */
void extended_kalman_filter(Spacecraft *sc, double measurements[], int measurement_count,
 double *jacobian, double measurement_noise[],
 double expected_meas[]) {
 int i, j;

 /* Allocate memory for matrices */
 double *H = (double *)flight_malloc_impl(measurement_count * STATE_DIM * sizeof(double));
 double *R = (double *)flight_malloc_impl(measurement_count * measurement_count * sizeof(double));
 double *K = (double *)flight_malloc_impl(STATE_DIM * measurement_count * sizeof(double));
 double *P = (double *)flight_malloc_impl(STATE_DIM * STATE_DIM * sizeof(double));
 double *Ht = (double *)flight_malloc_impl(STATE_DIM * measurement_count * sizeof(double));
 double *PHt = (double *)flight_malloc_impl(STATE_DIM * measurement_count * sizeof(double));
 double *HPHt = (double *)flight_malloc_impl(measurement_count * measurement_count * sizeof(double));
 double *S = (double *)flight_malloc_impl(measurement_count * measurement_count * sizeof(double));
 double *S_inv = (double *)flight_malloc_impl(measurement_count * measurement_count * sizeof(double));
 double *residuals = (double *)flight_malloc_impl(measurement_count * sizeof(double));
 double *Kres = (double *)flight_malloc_impl(STATE_DIM * sizeof(double));
 double *I_KH = (double *)flight_malloc_impl(STATE_DIM * STATE_DIM * sizeof(double));
 double *temp = (double *)flight_malloc_impl(STATE_DIM * STATE_DIM * sizeof(double));

 /* Single-exit-point pattern (MISRA-C:2012 Rule 15.1): forward-only
  * control flow via do{...}while(0)+break; one cleanup site after. */
 do {
 if (!H || !R || !K || !P || !Ht || !PHt || !HPHt || !S || !S_inv || !residuals || !Kres || !I_KH || !temp) {
 /* Handle memory allocation failure */
 FLIGHT_LOG("Memory allocation failed in EKF\n");
 break;
 }

 /* Copy Jacobian matrix (H) */
 for (i = 0; i < measurement_count * STATE_DIM; i++) {
 H[i] = jacobian[i];
 }

 /* Create measurement noise covariance matrix (R) - diagonal matrix */
 for (i = 0; i < measurement_count; i++) {
 for (j = 0; j < measurement_count; j++) {
 R[i * measurement_count + j] = (i == j) ? measurement_noise[i] : 0.0;
 }
 }

 /* Convert covariance from 2D to 1D array */
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 P[i * STATE_DIM + j] = sc->covariance[i][j];
 }
 }

 /* Transpose of H: Ht */
 matrix_transpose(H, Ht, measurement_count, STATE_DIM);

 /* PHt = P * Ht */
 matrix_multiply(P, Ht, PHt, STATE_DIM, STATE_DIM, measurement_count);

 /* HPHt = H * PHt */
 matrix_multiply(H, PHt, HPHt, measurement_count, STATE_DIM, measurement_count);

 /* S = HPHt + R */
 matrix_add(HPHt, R, S, measurement_count, measurement_count);

 /* S_inv = inv(S) */
 if (matrix_inverse(S, S_inv, measurement_count) != 0) {
 FLIGHT_LOG("Matrix inversion failed in EKF\n");
 break;
 }

 /* K = PHt * S_inv */
 matrix_multiply(PHt, S_inv, K, STATE_DIM, measurement_count, measurement_count);

 /* Compute residuals: residuals = z_actual - h(x_predicted) */
 for (i = 0; i < measurement_count; i++) {
 residuals[i] = measurements[i] - expected_meas[i];

 /* Normalize angular residuals to [-pi, pi] */
 if (i % 4 == 2 || i % 4 == 3) {
 while (residuals[i] > PI) residuals[i] -= TWO_PI;
 while (residuals[i] < -PI) residuals[i] += TWO_PI;
 }
 }

 /* Kres = K * residuals */
 for (i = 0; i < STATE_DIM; i++) {
 Kres[i] = 0.0;
 for (j = 0; j < measurement_count; j++) {
 Kres[i] += K[i * measurement_count + j] * residuals[j];
 }
 }

 /* Update state vector */
 sc->state.position[0] += Kres[0];
 sc->state.position[1] += Kres[1];
 sc->state.position[2] += Kres[2];
 sc->state.velocity[0] += Kres[3];
 sc->state.velocity[1] += Kres[4];
 sc->state.velocity[2] += Kres[5];
 sc->state.acceleration[0] += Kres[6];
 sc->state.acceleration[1] += Kres[7];
 sc->state.acceleration[2] += Kres[8];
 sc->state.clock_bias += Kres[9];
 sc->state.clock_drift += Kres[10];
 sc->state.clock_drift_rate += Kres[11];

 /* Update covariance: P = (I - K*H) * P */
 /* Compute I - K*H */
 double *KH = (double *)flight_malloc_impl(STATE_DIM * STATE_DIM * sizeof(double));
 if (!KH) {
 FLIGHT_LOG("Memory allocation failed in EKF\n");
 break;
 }

 /* KH = K * H */
 matrix_multiply(K, H, KH, STATE_DIM, measurement_count, STATE_DIM);

 /* I_KH = I - KH */
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 I_KH[i * STATE_DIM + j] = (i == j) ? 1.0 - KH[i * STATE_DIM + j] : -KH[i * STATE_DIM + j];
 }
 }

 /* temp = I_KH * P */
 matrix_multiply(I_KH, P, temp, STATE_DIM, STATE_DIM, STATE_DIM);

 /* Update spacecraft covariance */
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 sc->covariance[i][j] = temp[i * STATE_DIM + j];
 }
 }

 /* Ensure covariance matrix remains symmetric and positive-definite */
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < i; j++) {
 sc->covariance[i][j] = sc->covariance[j][i] = 0.5 * (sc->covariance[i][j] + sc->covariance[j][i]);
 }
 }

 /* Enforce minimum variance on diagonal elements */
 for (i = 0; i < STATE_DIM; i++) {
 if (sc->covariance[i][i] < 1e-10) {
 sc->covariance[i][i] = 1e-10;
 }
 }

 /* Clean up temporary matrices */
 flight_free_impl(KH);

 } while (0);
 flight_free_impl(H);
 flight_free_impl(R);
 flight_free_impl(K);
 flight_free_impl(P);
 flight_free_impl(Ht);
 flight_free_impl(PHt);
 flight_free_impl(HPHt);
 flight_free_impl(S);
 flight_free_impl(S_inv);
 flight_free_impl(residuals);
 flight_free_impl(Kres);
 flight_free_impl(I_KH);
 flight_free_impl(temp);
}

/* Batch Least Squares implementation */

/*
 * Relative Navigation
 * Functions for relative orbit determination and propagation.
 */

/* Compute relative state between two spacecraft */
void compute_relative_state(Spacecraft *sc1, Spacecraft *sc2, double relative_state[]) {
 int i;

 /* Relative position */
 for (i = 0; i < 3; i++) {
 relative_state[i] = sc2->state.position[i] - sc1->state.position[i];
 }

 /* Relative velocity */
 for (i = 0; i < 3; i++) {
 relative_state[i+3] = sc2->state.velocity[i] - sc1->state.velocity[i];
 }
}

/* Propagate relative orbit between two spacecraft */
void propagate_relative_orbit(Spacecraft *sc1, Spacecraft *sc2, double dt) {
 /* Propagate reference spacecraft with full nonlinear RK4 */
 propagate_orbit_rk4(sc1, dt);

 /* Use HCW linear relative motion model for deputy spacecraft.
 * This is more efficient than double RK4 and accurately captures
 * the formation geometry for close-proximity operations. */
 double relative_state[6];
 compute_relative_state(sc1, sc2, relative_state);

 /* Compute mean motion of reference orbit (sc1) */
 double r_mag = vector_magnitude(sc1->state.position, 3);
 double reference_mean_motion = sqrt(EARTH_MU / (r_mag * r_mag * r_mag));

 /* Propagate relative state using HCW equations */
 double propagated_relative_state[6];
 hill_clohessy_wiltshire_propagation(relative_state, reference_mean_motion, dt, propagated_relative_state);

 /* Update sc2 state based on propagated relative state */
 for (int i = 0; i < 3; i++) {
 sc2->state.position[i] = sc1->state.position[i] + propagated_relative_state[i];
 sc2->state.velocity[i] = sc1->state.velocity[i] + propagated_relative_state[i+3];
 }

 /* Recompute accelerations */
 calculate_acceleration(sc2->state.position, sc2->state.velocity, sc2->state.acceleration);
}

/* Hill-Clohessy-Wiltshire equations for relative orbit propagation */
void hill_clohessy_wiltshire_propagation(double state_vector[6], double n, double dt, double propagated_state[6]) {
 /*
 * HCW equations model relative motion in a rotating reference frame,
 * where the reference satellite is at the origin.
 *
 * x: Radial direction (away from Earth)
 * y: In-track direction (along velocity)
 * z: Cross-track direction (orbit normal)
 *
 * n: Mean motion of reference orbit
 */

 double x0 = state_vector[0];
 double y0 = state_vector[1];
 double z0 = state_vector[2];
 double vx0 = state_vector[3];
 double vy0 = state_vector[4];
 double vz0 = state_vector[5];

 double nt = n * dt;
 double cos_nt = cos(nt);
 double sin_nt = sin(nt);

 /* x-component (radial) */
 propagated_state[0] = (4 - 3 * cos_nt) * x0 + sin_nt/n * vx0 + 2/n * (1 - cos_nt) * vy0;

 /* y-component (in-track) */
 propagated_state[1] = 6 * (sin_nt - nt) * x0 + y0 - 2/n * (1 - cos_nt) * vx0 +
 (4 * sin_nt - 3 * nt)/n * vy0;

 /* z-component (cross-track) */
 propagated_state[2] = z0 * cos_nt + vz0/n * sin_nt;

 /* Velocity in x-direction */
 propagated_state[3] = 3 * n * sin_nt * x0 + cos_nt * vx0 + 2 * sin_nt * vy0;

 /* Velocity in y-direction */
 propagated_state[4] = 6 * n * (cos_nt - 1) * x0 - 2 * sin_nt * vx0 + (4 * cos_nt - 3) * vy0;

 /* Velocity in z-direction */
 propagated_state[5] = -z0 * n * sin_nt + vz0 * cos_nt;
}

/*
 * ODP and RODP Implementation
 * These are the main functions for orbit determination and propagation.
 */

/* Orbit Determination and Propagation implementation */

/* Relative Orbit Determination and Propagation implementation */

/*
 * Utility Functions
 * Helper functions for displaying results and other tasks.
 */

/* Print spacecraft state */
void print_spacecraft_state(Spacecraft *sc) {
 FLIGHT_LOG("Spacecraft ID: %d\n", sc->satellite_id);
 FLIGHT_LOG("Position (m): [%.2f, %.2f, %.2f]\n",
 sc->state.position[0], sc->state.position[1], sc->state.position[2]);
 FLIGHT_LOG("Velocity (m/s): [%.2f, %.2f, %.2f]\n",
 sc->state.velocity[0], sc->state.velocity[1], sc->state.velocity[2]);
 FLIGHT_LOG("Acceleration (m/s^2): [%.6f, %.6f, %.6f]\n",
 sc->state.acceleration[0], sc->state.acceleration[1], sc->state.acceleration[2]);
 FLIGHT_LOG("Clock bias (s): %.9e\n", sc->state.clock_bias);
 FLIGHT_LOG("Clock drift (s/s): %.9e\n", sc->state.clock_drift);

 FLIGHT_LOG("\nOrbital Elements:\n");
 FLIGHT_LOG("Semi-major axis (m): %.2f\n", sc->ephemeris.semi_major_axis);
 FLIGHT_LOG("Eccentricity: %.6f\n", sc->ephemeris.eccentricity);
 FLIGHT_LOG("Inclination (deg): %.2f\n", sc->ephemeris.inclination * 180.0 / PI);
 FLIGHT_LOG("RAAN (deg): %.2f\n", sc->ephemeris.raan * 180.0 / PI);
 FLIGHT_LOG("Argument of Perigee (deg): %.2f\n", sc->ephemeris.arg_perigee * 180.0 / PI);
 FLIGHT_LOG("Mean Anomaly (deg): %.2f\n", sc->ephemeris.mean_anomaly * 180.0 / PI);
 FLIGHT_LOG("Mean Motion (rad/s): %.9e\n", sc->ephemeris.mean_motion);
}

/* Print matrix */

/* Evaluate constellation health - Decision BCE
 * Checks PDOP, elevation masks, residual magnitudes, and navigation solution validity.
 * Returns 1 if navigation solution is valid, 0 otherwise.
 */
int evaluate_constellation_health(Spacecraft *sc, Spacecraft satellites[], int num_satellites,
 double *jacobian, int measurement_count, double residuals[]) {
 int i;
 int valid_satellites = 0;
 double ELEVATION_MASK = 10.0 * PI / 180.0; /* 10 degree elevation mask */
 double RESIDUAL_THRESHOLD = 50.0; /* 50 meter residual threshold */
 int solution_valid = 1;

 /* --- Check satellite elevation mask angles --- */
 for (i = 0; i < num_satellites; i++) {
 double dx = satellites[i].state.position[0] - sc->state.position[0];
 double dy = satellites[i].state.position[1] - sc->state.position[1];
 double dz = satellites[i].state.position[2] - sc->state.position[2];
 double range = sqrt(dx*dx + dy*dy + dz*dz);
 double sc_range = sqrt(sc->state.position[0]*sc->state.position[0] +
 sc->state.position[1]*sc->state.position[1] +
 sc->state.position[2]*sc->state.position[2]);
 /* Approximate elevation: angle between LOS and local horizontal */
 double cos_nadir = (sc->state.position[0]*dx + sc->state.position[1]*dy +
 sc->state.position[2]*dz) / (sc_range * range);
 double elevation = PI/2.0 - acos(cos_nadir < -1.0 ? -1.0 : (cos_nadir > 1.0 ? 1.0 : cos_nadir));
 if (elevation > ELEVATION_MASK) {
 valid_satellites++;
 }
 }

 /* Need at least 4 satellites above mask for 3D fix + clock */
 if (valid_satellites < 4) {
 FLIGHT_LOG("[HEALTH] FAIL: Only %d satellites above elevation mask (need 4)\n", valid_satellites);
 solution_valid = 0;
 }

 /* --- Compute PDOP from geometry matrix (H) --- */
 /* PDOP = sqrt(trace of (H^T H)^-1 position block) */
 if (measurement_count >= 4 && jacobian != NULL) {
 double HtH[STATE_DIM * STATE_DIM];
 double Ht_local[STATE_DIM * MEASUREMENT_DIM];
 matrix_transpose(jacobian, Ht_local, measurement_count, STATE_DIM);
 matrix_multiply(Ht_local, jacobian, HtH, STATE_DIM, measurement_count, STATE_DIM);

 double HtH_inv[STATE_DIM * STATE_DIM];
 if (matrix_inverse(HtH, HtH_inv, STATE_DIM) == 0) {
 /* PDOP = sqrt(sigma_x^2 + sigma_y^2 + sigma_z^2) from first 3 diagonal elements */
 double pdop = sqrt(fabs(HtH_inv[0]) + fabs(HtH_inv[STATE_DIM + 1]) + fabs(HtH_inv[2*STATE_DIM + 2]));
 FLIGHT_LOG("[HEALTH] PDOP = %.2f\n", pdop);
 if (pdop > 6.0) {
 FLIGHT_LOG("[HEALTH] WARNING: Poor geometry (PDOP > 6.0)\n");
 solution_valid = 0;
 }
 } else {
 FLIGHT_LOG("[HEALTH] FAIL: Geometry matrix singular, cannot compute PDOP\n");
 solution_valid = 0;
 }
 }

 /* --- Check residual magnitudes --- */
 int rejected_count = 0;
 for (i = 0; i < measurement_count; i++) {
 if (fabs(residuals[i]) > RESIDUAL_THRESHOLD) {
 rejected_count++;
 }
 }
 if (rejected_count > 0) {
 FLIGHT_LOG("[HEALTH] %d/%d measurements rejected (residual > %.1f m)\n",
 rejected_count, measurement_count, RESIDUAL_THRESHOLD);
 if (rejected_count > measurement_count / 2) {
 FLIGHT_LOG("[HEALTH] FAIL: Majority of measurements have excessive residuals\n");
 solution_valid = 0;
 }
 }

 /* --- Final navigation solution validity decision --- */
 if (solution_valid) {
 FLIGHT_LOG("[HEALTH] Navigation solution VALID (%d sats, %d meas)\n",
 valid_satellites, measurement_count);
 } else {
 FLIGHT_LOG("[HEALTH] Navigation solution INVALID - reverting to predicted state\n");
 }

 return solution_valid;
}

/* Main entry point */
int main() {
 /* Create a spacecraft */
 Spacecraft sc;
 initialize_state(&sc);
 initialize_covariance(&sc);

 /* Print initial state */
 FLIGHT_LOG("[SPS] initial_state\n");
 print_spacecraft_state(&sc);

 /* Simulate orbit propagation for one hour */
 double dt = 0.100; /* 100 ms integration step */
 int steps = 36000; /* 36000 steps × 0.1 s = 3600 s = 1 hour */
 int i;

 for (i = 0; i < steps; i++) {
 propagate_orbit_rk4(&sc, dt);
 }

 /* Print final state */
 FLIGHT_LOG("\n[SPS] final_state (t+3600s)\n");
 print_spacecraft_state(&sc);

 /* === Constellation Health Assessment (Decision BCE) === */
 {
 /* Set up a small satellite constellation for health evaluation */
 Spacecraft constellation[4];
 int num_sats = 4;
 int s;
 double orbit_r = EARTH_RADIUS + 35786000.0; /* GEO altitude */
 for (s = 0; s < num_sats; s++) {
 initialize_state(&constellation[s]);
 double angle = (TWO_PI / num_sats) * s;
 constellation[s].state.position[0] = orbit_r * cos(angle);
 constellation[s].state.position[1] = orbit_r * sin(angle);
 constellation[s].state.position[2] = orbit_r * 0.1 * (s % 2 == 0 ? 1.0 : -1.0);
 constellation[s].satellite_id = s + 1;
 }

 /* Compute Jacobian and expected measurements */
 double jacobian_health[MEASUREMENT_DIM * STATE_DIM];
 double meas_noise[MEASUREMENT_DIM];
 double sim_measurements[MEASUREMENT_DIM];
 double expected_meas[MEASUREMENT_DIM];
 int meas_count = 0, exp_count = 0;

 compute_measurement_jacobian(&sc, constellation, num_sats, jacobian_health, &meas_count);
 compute_expected_measurements(&sc, constellation, num_sats, expected_meas, &exp_count);

 /* Simulate actual measurements with small noise */
 for (s = 0; s < meas_count; s++) {
 double noise_sigma = (s % 4 == 0) ? 5.0 : (s % 4 == 1) ? 0.005 : 0.0005;
 sim_measurements[s] = expected_meas[s] + (s % 3 == 0 ? 2.5 : -1.3);
 meas_noise[s] = noise_sigma * noise_sigma; /* measurement noise variance */
 }

 /* Compute residuals, then check health using noise-weighted residuals */
 double health_residuals[MEASUREMENT_DIM];
 measurement_residuals(sim_measurements, expected_meas, health_residuals, meas_count);

 /* Compute weighted RMS of residuals (chi-squared statistic) */
 double wrss = 0.0;
 for (s = 0; s < meas_count; s++) {
 wrss += health_residuals[s] * health_residuals[s] / meas_noise[s];
 }
 FLIGHT_LOG("Weighted residual RMS: %.4f (chi2/dof=%.2f)\n",
 sqrt(wrss / meas_count), wrss / meas_count);

 /* Evaluate constellation health */
 int nav_valid = evaluate_constellation_health(&sc, constellation, num_sats,
 jacobian_health, meas_count, health_residuals);
 FLIGHT_LOG("Navigation solution status: %s\n", nav_valid ? "ACCEPTED" : "REJECTED");
 }

 /* Create a second spacecraft for relative-orbit reference case */
 Spacecraft sc2;
 initialize_state(&sc2);

 /* Set slightly different orbital elements */
 sc2.ephemeris.semi_major_axis += 1000.0; /* 1 km higher */
 sc2.ephemeris.inclination += 0.01; /* Slightly different inclination */

 /* Convert orbital elements to state vector */
 double state_vector[6];
 orbital_elements_to_state_vector(&sc2.ephemeris, 0.0, state_vector);

 /* Update state */
 for (i = 0; i < 3; i++) {
 sc2.state.position[i] = state_vector[i];
 sc2.state.velocity[i] = state_vector[i+3];
 }

 /* Compute acceleration */
 calculate_acceleration(sc2.state.position, sc2.state.velocity, sc2.state.acceleration);

 /* Compute relative state */
 double relative_state[6];
 compute_relative_state(&sc, &sc2, relative_state);

 FLIGHT_LOG("\n[SPS] rel_state\n");
 FLIGHT_LOG("Relative Position (m): [%.2f, %.2f, %.2f]\n",
 relative_state[0], relative_state[1], relative_state[2]);
 FLIGHT_LOG("Relative Velocity (m/s): [%.6f, %.6f, %.6f]\n",
 relative_state[3], relative_state[4], relative_state[5]);

 /* Demonstrate propagation of relative orbit */
 propagate_relative_orbit(&sc, &sc2, 3600.0); /* Propagate for 1 hour */

 /* Recompute relative state */
 compute_relative_state(&sc, &sc2, relative_state);

 FLIGHT_LOG("\n[SPS] rel_state (t+3600s)\n");
 FLIGHT_LOG("Relative Position (m): [%.2f, %.2f, %.2f]\n",
 relative_state[0], relative_state[1], relative_state[2]);
 FLIGHT_LOG("Relative Velocity (m/s): [%.6f, %.6f, %.6f]\n",
 relative_state[3], relative_state[4], relative_state[5]);

 /* === EKF-based Orbit Determination (KalF + Matrix BCE) === */
 {
 /* Set up navigation constellation for EKF measurement update */
 Spacecraft nav_constellation[4];
 int num_nav_sats = 4;
 double nav_orbit_r = EARTH_RADIUS + 35786000.0; /* GEO altitude */
 int s;
 for (s = 0; s < num_nav_sats; s++) {
 initialize_state(&nav_constellation[s]);
 double angle = (TWO_PI / num_nav_sats) * s;
 nav_constellation[s].state.position[0] = nav_orbit_r * cos(angle);
 nav_constellation[s].state.position[1] = nav_orbit_r * sin(angle);
 nav_constellation[s].state.position[2] = nav_orbit_r * 0.15 * (s % 2 == 0 ? 1.0 : -1.0);
 nav_constellation[s].satellite_id = s + 1;
 }

 double ekf_jacobian[MEASUREMENT_DIM * STATE_DIM];
 double ekf_meas_noise[MEASUREMENT_DIM];
 double ekf_sim_meas[MEASUREMENT_DIM];
 double ekf_expected[MEASUREMENT_DIM];
 int ekf_meas_count = 0, ekf_exp_count = 0;

 /* Perform 20 EKF measurement update cycles */
 FLIGHT_LOG("\n[SPS] EKF OD: 20 cycles\n");
 for (i = 0; i < 20; i++) {
 /* Propagate between measurements */
 int prop_steps;
 for (prop_steps = 0; prop_steps < 100; prop_steps++) {
 propagate_orbit_rk4(&sc, dt);
 }

 /* Compute Jacobian and expected measurements */
 ekf_meas_count = 0;
 compute_measurement_jacobian(&sc, nav_constellation, num_nav_sats, ekf_jacobian, &ekf_meas_count);
 compute_expected_measurements(&sc, nav_constellation, num_nav_sats, ekf_expected, &ekf_exp_count);

 /* Generate simulated pseudorange + Doppler measurements with noise */
 for (s = 0; s < ekf_meas_count; s++) {
 ekf_sim_meas[s] = ekf_expected[s] + ((s * 7 + 13) % 100 - 50) * 0.1;
 ekf_meas_noise[s] = (s % 4 == 0) ? 10.0 : (s % 4 == 1) ? 0.01 : 0.001;
 }

 /* Apply EKF measurement update */
 extended_kalman_filter(&sc, ekf_sim_meas, ekf_meas_count, ekf_jacobian, ekf_meas_noise, ekf_expected);
 }

 FLIGHT_LOG("EKF-updated position (m): [%.2f, %.2f, %.2f]\n",
 sc.state.position[0], sc.state.position[1], sc.state.position[2]);
 FLIGHT_LOG("EKF-updated velocity (m/s): [%.6f, %.6f, %.6f]\n",
 sc.state.velocity[0], sc.state.velocity[1], sc.state.velocity[2]);
 FLIGHT_LOG("Clock bias: %.3f ns, drift: %.3f ns/s\n",
 sc.state.clock_bias * 1.0e9, sc.state.clock_drift * 1.0e9);
 }

 return 0;
}
