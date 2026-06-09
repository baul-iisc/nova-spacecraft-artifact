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
 * Step-and-Stare Imaging with Navigation Solution
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Combined (CORDIC trig + AME 3×3 tile accelerator)
 * Application: (see workload description below)
 *
 * Description:
 * Implements the onboard step-and-stare imaging sequence planner, attitude
 * maneuver controller, and embedded navigation solution for high-resolution
 * Earth observation. Combines target acquisition geometry, attitude slew
 * quaternions, state prediction for timeline planning, image footprint
 * projection, and GNSS-based DOP geometry analysis with state covariance
 * propagation. Representative of the high-resolution and future sub-meter
 * resolution imaging satellite agile maneuvering systems.
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
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
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
#define MATRIX_SIZE 9 /* pos(3)+vel(3)+clk_bias(1)+clk_drift(1)+clk_accel(1) — 3×3 AME tiling */
#define NUM_NAV_SATELLITES 8
#define EARTH_RADIUS_M 6371000.0
#define EARTH_MU 3.986004418e14
#define EARTH_ROTATION_RATE 7.2921159e-5
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)

/* HW trig wrappers — CORDIC accelerator calls. */
#define sw_sin(a) hw_sin(a)
#define sw_cos(a) hw_cos(a)
static inline double sw_tan(double a) { return hw_tan(a); }
static inline double sw_asin(double a) { return hw_asin(a); }
static inline double sw_acos(double a) { return hw_acos(a); }
static inline double sw_atan(double a) { return hw_atan(a); }
#define sw_atan2(y, x) hw_atan2(y, x)

// Structure for 3D vector
typedef struct {
 double x, y, z;
} Vector3D;

// Structure for quaternion
typedef struct {
 double w, x, y, z;
} Quaternion;

// Structure for spacecraft attitude
typedef struct {
 Quaternion orientation; // Current orientation as quaternion
 Vector3D position; // Current position
 Vector3D velocity; // Current velocity
 Vector3D angular_velocity; // Current angular velocity
} SpacecraftAttitude;

// Structure for imaging target
typedef struct {
 Vector3D position; // Target position in inertial frame
 double priority; // Target imaging priority
 bool imaged; // Flag to indicate if target has been imaged
 char name[64]; // Target name
} ImagingTarget;

// Structure for camera parameters
typedef struct {
 double focal_length; // Focal length in mm
 double sensor_width; // Sensor width in mm
 double sensor_height; // Sensor height in mm
 double pixel_size; // Pixel size in mm
 int resolution_x; // Resolution in x-direction
 int resolution_y; // Resolution in y-direction
 double fov_x; // Field of view in x-direction (radians)
 double fov_y; // Field of view in y-direction (radians)
} CameraParameters;

// Structure for imaging plan
typedef struct {
 int num_targets;
 ImagingTarget* targets;
 int* imaging_sequence; // Optimized sequence of target indices
} ImagingPlan;

// Forward declarations of all functions
void sw_matrix_multiply(double A[MATRIX_SIZE][MATRIX_SIZE],
 double B[MATRIX_SIZE][MATRIX_SIZE],
 double C[MATRIX_SIZE][MATRIX_SIZE]);
void sw_matrix_transpose(double A[MATRIX_SIZE][MATRIX_SIZE],
 double AT[MATRIX_SIZE][MATRIX_SIZE]);
void sw_matrix_inverse(double A[MATRIX_SIZE][MATRIX_SIZE],
 double Ainv[MATRIX_SIZE][MATRIX_SIZE]);
void sw_matrix_vector_multiply(double A[MATRIX_SIZE][MATRIX_SIZE],
 double v[MATRIX_SIZE],
 double result[MATRIX_SIZE]);
double vector_magnitude(Vector3D v);
Vector3D vector_normalize(Vector3D v);
Vector3D vector_cross_product(Vector3D a, Vector3D b);
double vector_dot_product(Vector3D a, Vector3D b);
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);
Quaternion quaternion_from_axis_angle(Vector3D axis, double angle);
Vector3D quaternion_rotate_vector(Quaternion q, Vector3D v);
Quaternion quaternion_conjugate(Quaternion q);
Quaternion quaternion_normalize(Quaternion q);
void get_rotation_matrix_from_quaternion(Quaternion q, double rot_matrix[3][3]);
void rotation_matrix_to_dcm(double rot_matrix[3][3], double dcm[MATRIX_SIZE][MATRIX_SIZE]);
double calculate_slew_time(Quaternion q1, Quaternion q2, double max_angular_velocity);
void initialize_spacecraft(SpacecraftAttitude* sc);
void initialize_camera(CameraParameters* camera);
void calculate_camera_parameters(CameraParameters* camera);
void create_imaging_plan(ImagingTarget* targets, int num_targets, ImagingPlan* plan);
void calculate_inertial_to_body_matrix(SpacecraftAttitude* sc, double dcm[MATRIX_SIZE][MATRIX_SIZE]);
void calculate_pointing_vector(CameraParameters* camera, SpacecraftAttitude* sc,
 ImagingTarget target, Vector3D* pointing_vector);
bool check_target_visibility(SpacecraftAttitude* sc, ImagingTarget target,
 CameraParameters* camera);
void calculate_slew_maneuver(SpacecraftAttitude* current, ImagingTarget target,
 CameraParameters* camera, SpacecraftAttitude* final);
void execute_imaging_sequence(SpacecraftAttitude* sc, ImagingPlan* plan,
 CameraParameters* camera);
void generate_distortion_correction_matrix(CameraParameters* camera,
 double distortion_matrix[MATRIX_SIZE][MATRIX_SIZE]);
void calculate_projection_matrix(CameraParameters* camera,
 SpacecraftAttitude* sc,
 double projection_matrix[MATRIX_SIZE][MATRIX_SIZE]);
void calculate_thermal_distortion(SpacecraftAttitude* sc,
 double thermal_matrix[MATRIX_SIZE][MATRIX_SIZE]);
void generate_jitter_compensation(SpacecraftAttitude* sc,
 double jitter_matrix[MATRIX_SIZE][MATRIX_SIZE]);
void calculate_imaging_location(SpacecraftAttitude* sc, ImagingTarget target,
 CameraParameters* camera,
 double precision_matrix[MATRIX_SIZE][MATRIX_SIZE],
 double* ground_location_x, double* ground_location_y);
void perform_imaging(SpacecraftAttitude* sc, ImagingTarget* target,
 CameraParameters* camera);
Vector3D calculate_position_from_lla(double lat, double lon, double alt);
void execute_imaging_sequence_full(SpacecraftAttitude* sc, ImagingPlan* plan,
 CameraParameters* camera);
double run_imaging_pipeline(int num_iterations);
void compute_dop_matrix(SpacecraftAttitude* sc, int num_sats,
 double H[MATRIX_SIZE][MATRIX_SIZE],
 double dop_result[MATRIX_SIZE][MATRIX_SIZE],
 double *gdop, double *pdop);
void update_position_velocity_covariance(double P[MATRIX_SIZE][MATRIX_SIZE],
 double Phi[MATRIX_SIZE][MATRIX_SIZE],
 double Q[MATRIX_SIZE][MATRIX_SIZE],
 SpacecraftAttitude* sc, double dt);
void compute_ecef_to_eci_rotation(double gmst_angle,
 double R[MATRIX_SIZE][MATRIX_SIZE]);

/**
 * Matrix multiplication: C = A * B
 */
void sw_matrix_multiply(double A[MATRIX_SIZE][MATRIX_SIZE], double B[MATRIX_SIZE][MATRIX_SIZE], double C[MATRIX_SIZE][MATRIX_SIZE]) {
 /* ── Optimized: pre-transpose B → direct MLDS/MSDS ── */
 const int n = MATRIX_SIZE;
 memset(C, 0, sizeof(double) * n * n);

 const long stride_n = n * (long)sizeof(double);
 const long s24 = 24;

 int n3a = (n / 3) * 3;

 for (int i = 0; i < n3a; i += 3) {
 for (int j = 0; j < n3a; j += 3) {
 asm volatile(MZERO(M2));

 for (int kk = 0; kk < n3a; kk += 3) {
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride_n), "r"(&A[i][kk]),
 "r"(stride_n), "r"(&B[kk][j])
 : "t0", "t1", "memory"
 );
 }

 if (n3a < n) {
 double a_pad[9] = {0}, b_pad[9] = {0};
 for (int r = 0; r < 3; r++)
 for (int c = 0; c < n - n3a; c++) {
 a_pad[r * 3 + c] = (double)A[i + r][n3a + c];
 b_pad[r * 3 + c] = B[j + r][n3a + c];
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

 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(stride_n), "r"(&C[i][j])
 : "t0", "t1", "memory"
 );
 }

 for (int j = n3a; j < n; j++)
 for (int r = 0; r < 3; r++)
 for (int kk = 0; kk < n; kk++)
 C[i + r][j] += (double)((double)A[i + r][kk] * (double)B[kk][j]);
 }

 for (int i = n3a; i < n; i++)
 for (int j = 0; j < n; j++)
 for (int kk = 0; kk < n; kk++)
 C[i][j] += (double)((double)A[i][kk] * (double)B[kk][j]);
}


/**
 * Matrix transpose: AT = A^T
 */
void sw_matrix_transpose(double A[MATRIX_SIZE][MATRIX_SIZE],
 double AT[MATRIX_SIZE][MATRIX_SIZE]) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 AT[j][i] = A[i][j];
 }
 }
}

/**
 * Matrix-vector multiplication: result = A * v
 */
void sw_matrix_vector_multiply(double A[MATRIX_SIZE][MATRIX_SIZE],
 double v[MATRIX_SIZE],
 double result[MATRIX_SIZE]) {
 const int P = 3;
 const int stride = P * (int)sizeof(double);
 double At[9], Bt[9], Ct[9];

 for (int i = 0; i < MATRIX_SIZE; i++)
 result[i] = 0.0;

 for (int ti = 0; ti < MATRIX_SIZE; ti += P) {
 asm volatile(MZERO(M2));
 for (int tk = 0; tk < MATRIX_SIZE; tk += P) {
 /* Gather A tile [ti..ti+2, tk..tk+2] */
 for (int r = 0; r < P; r++)
 for (int c = 0; c < P; c++)
 At[r * P + c] = A[ti + r][tk + c];
 /* B tile: row 0 = v segment, cols 1-2 = 0 (SYS_MMACD: C += A × B (systolic)) */
 for (int x = 0; x < 9; x++) Bt[x] = 0.0;
 Bt[0] = v[tk + 0]; Bt[3] = v[tk + 1]; Bt[6] = v[tk + 2];
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride), "r"(At),
 "r"(stride), "r"(Bt)
 : "t0", "t1", "memory"
 );
 }
 asm volatile(
 "mv t0, %0\n\t"
 "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(stride), "r"(Ct)
 : "t0", "t1", "memory"
 );
 /* Only first column of result tile matters */
 for (int r = 0; r < P; r++)
 result[ti + r] = Ct[r * P + 0];
 }
}

/* ------------------------------------------------------------------ */
/* HW Helper: General tiled matrix multiply (for recursive inverse) */
/* Uses SYS_MMACD with 3×3 tiles and gather/scatter for boundaries */
/* ------------------------------------------------------------------ */
static void hw_matmul_general(const double *A, const double *B, double *C,
 int M_dim, int N_dim, int K_dim)
{
 /* ── Optimized: pre-transpose B → direct MLDS/MSDS (zero gather/scatter) ── */
 double *BT = (double *)flight_malloc_impl((size_t)N_dim * K_dim * sizeof(double));
 memset(C, 0, (size_t)M_dim * N_dim * sizeof(double));

 const long stride_k = K_dim * (long)sizeof(double);
 const long stride_n = N_dim * (long)sizeof(double);
 const long s24 = 24;

 int m3a = (M_dim / 3) * 3;
 int n3a = (N_dim / 3) * 3;
 int k3a = (K_dim / 3) * 3;

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
 :: "r"(stride_k), "r"(&A[i * K_dim + kk]),
 "r"(stride_n), "r"(&B[kk * N_dim + j])
 : "t0", "t1", "memory"
 );
 }

 /* K-remainder tile (gather with zero-pad) */
 if (k3a < K_dim) {
 double a_pad[9] = {0}, b_pad[9] = {0};
 for (int r = 0; r < 3; r++)
 for (int c = 0; c < K_dim - k3a; c++) {
 a_pad[r * 3 + c] = A[(i + r) * K_dim + k3a + c];
 b_pad[r * 3 + c] = B[(j + r) * K_dim + k3a + c];
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
 :: "r"(stride_n), "r"(&C[i * N_dim + j])
 : "t0", "t1", "memory"
 );
 }

 /* N-remainder columns — SW fallback */
 for (int j = n3a; j < N_dim; j++)
 for (int r = 0; r < 3; r++)
 for (int kk = 0; kk < K_dim; kk++)
 C[(i + r) * N_dim + j] += A[(i + r) * K_dim + kk] * B[kk * N_dim + j];
 }

 /* M-remainder rows — SW fallback */
 for (int i = m3a; i < M_dim; i++)
 for (int j = 0; j < N_dim; j++)
 for (int kk = 0; kk < K_dim; kk++)
 C[i * N_dim + j] += A[i * K_dim + kk] * B[kk * N_dim + j];
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
 hw_matmul_general(C_blk, Ai, CA, q, p, p);
 hw_matmul_general(CA, B_blk, tmp_qq, q, q, p);
 for (int i = 0; i < q * q; i++) S[i] = D_blk[i] - tmp_qq[i];

 ok = hw_inverse_recursive(S, Si, q);
 if (!ok) goto cleanup;

 hw_matmul_general(Ai, B_blk, AB, p, q, p);
 hw_matmul_general(AB, Si, F_blk, p, q, q);
 for (int i = 0; i < p * q; i++) F_blk[i] = -F_blk[i];

 hw_matmul_general(Si, CA, G_blk, q, p, q);
 for (int i = 0; i < q * p; i++) G_blk[i] = -G_blk[i];

 for (int i = 0; i < p * q; i++) neg_F[i] = -F_blk[i];
 hw_matmul_general(neg_F, CA, tmp_pp, p, p, q);
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
 * HW-accelerated matrix inverse using recursive Schur complement
 * with MINVD for 3×3 base case and SYS_MMACD-tiled multiplies.
 */
void sw_matrix_inverse(double A[MATRIX_SIZE][MATRIX_SIZE],
 double Ainv[MATRIX_SIZE][MATRIX_SIZE]) {
 /* Flatten 2D arrays to flat for recursive helper */
 double flat_A[MATRIX_SIZE * MATRIX_SIZE];
 double flat_Ainv[MATRIX_SIZE * MATRIX_SIZE];
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 flat_A[i * MATRIX_SIZE + j] = A[i][j];

 hw_inverse_recursive(flat_A, flat_Ainv, MATRIX_SIZE);

 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 Ainv[i][j] = flat_Ainv[i * MATRIX_SIZE + j];
}

/**
 * Calculate vector magnitude
 */
double vector_magnitude(Vector3D v) {
 return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Normalize vector
 */
Vector3D vector_normalize(Vector3D v) {
 double mag = vector_magnitude(v);
 Vector3D result = v;

 if (mag > 1e-10) {
 result.x /= mag;
 result.y /= mag;
 result.z /= mag;
 }

 return result;
}

/**
 * Vector cross product
 */
Vector3D vector_cross_product(Vector3D a, Vector3D b) {
 Vector3D result;
 result.x = a.y * b.z - a.z * b.y;
 result.y = a.z * b.x - a.x * b.z;
 result.z = a.x * b.y - a.y * b.x;
 return result;
}

/**
 * Vector dot product
 */
double vector_dot_product(Vector3D a, Vector3D b) {
 return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * Quaternion multiplication
 */
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
 Quaternion result;
 result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
 result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
 result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
 result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
 return result;
}

/**
 * Create quaternion from axis-angle representation
 */
Quaternion quaternion_from_axis_angle(Vector3D axis, double angle) {
 Quaternion q;
 double half_angle = angle / 2.0;
 double sin_half = sw_sin(half_angle);

 q.w = sw_cos(half_angle);
 q.x = axis.x * sin_half;
 q.y = axis.y * sin_half;
 q.z = axis.z * sin_half;

 return quaternion_normalize(q);
}

/**
 * Rotate a vector by a quaternion
 */
Vector3D quaternion_rotate_vector(Quaternion q, Vector3D v) {
 // Create vector quaternion
 Quaternion v_quat;
 v_quat.w = 0.0;
 v_quat.x = v.x;
 v_quat.y = v.y;
 v_quat.z = v.z;

 // q * v * q^-1
 Quaternion q_conjugate = quaternion_conjugate(q);
 Quaternion temp = quaternion_multiply(q, v_quat);
 Quaternion result_quat = quaternion_multiply(temp, q_conjugate);

 // Extract vector part
 Vector3D result;
 result.x = result_quat.x;
 result.y = result_quat.y;
 result.z = result_quat.z;

 return result;
}

/**
 * Calculate quaternion conjugate
 */
Quaternion quaternion_conjugate(Quaternion q) {
 Quaternion result;
 result.w = q.w;
 result.x = -q.x;
 result.y = -q.y;
 result.z = -q.z;
 return result;
}

/**
 * Normalize quaternion
 */
Quaternion quaternion_normalize(Quaternion q) {
 double mag = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
 Quaternion result = q;

 if (mag > 1e-10) {
 result.w /= mag;
 result.x /= mag;
 result.y /= mag;
 result.z /= mag;
 }

 return result;
}

/**
 * Get 3x3 rotation matrix from quaternion
 */
void get_rotation_matrix_from_quaternion(Quaternion q, double rot_matrix[3][3]) {
 double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
 double qw2 = qw * qw, qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz;

 // First row
 rot_matrix[0][0] = qw2 + qx2 - qy2 - qz2;
 rot_matrix[0][1] = 2.0 * (qx * qy - qw * qz);
 rot_matrix[0][2] = 2.0 * (qx * qz + qw * qy);

 // Second row
 rot_matrix[1][0] = 2.0 * (qx * qy + qw * qz);
 rot_matrix[1][1] = qw2 - qx2 + qy2 - qz2;
 rot_matrix[1][2] = 2.0 * (qy * qz - qw * qx);

 // Third row
 rot_matrix[2][0] = 2.0 * (qx * qz - qw * qy);
 rot_matrix[2][1] = 2.0 * (qy * qz + qw * qx);
 rot_matrix[2][2] = qw2 - qx2 - qy2 + qz2;
}

/**
 * Embed 3x3 rotation matrix into larger DCM
 */
void rotation_matrix_to_dcm(double rot_matrix[3][3], double dcm[MATRIX_SIZE][MATRIX_SIZE]) {
 // Copy rotation matrix into upper-left 3x3 block
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 dcm[i][j] = rot_matrix[i][j];
 }
 }

 // The remaining parts would include sensor misalignments,
 // spacecraft flexure effects, thermal effects, etc.
 // Keeping the rest of the DCM unchanged.
}

/**
 * Calculate slew time based on quaternion difference
 */
double calculate_slew_time(Quaternion q1, Quaternion q2, double max_angular_velocity) {
 // Calculate the angular difference between quaternions
 double dot_product = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
 dot_product = fabs(dot_product); // Handle shortest path

 if (dot_product > 1.0) dot_product = 1.0;

 // Calculate the angle (rad)
 double angle = 2.0 * sw_acos(dot_product);

 // Calculate time needed at max angular velocity
 return angle / max_angular_velocity;
}

/**
 * Initialize spacecraft with default values
 */
void initialize_spacecraft(SpacecraftAttitude* sc) {
 // Initialize with nadir-pointing attitude
 sc->orientation.w = 1.0;
 sc->orientation.x = 0.0;
 sc->orientation.y = 0.0;
 sc->orientation.z = 0.0;

 // Initialize position (in Earth-centered inertial frame, km)
 sc->position.x = 7000.0; // ~630km altitude
 sc->position.y = 0.0;
 sc->position.z = 0.0;

 // Initialize velocity (km/s)
 sc->velocity.x = 0.0;
 sc->velocity.y = 7.5; // Approximate orbital velocity
 sc->velocity.z = 0.0;

 // Initialize angular velocity (rad/s)
 sc->angular_velocity.x = 0.0;
 sc->angular_velocity.y = 0.0;
 sc->angular_velocity.z = 0.0;
}

/**
 * Initialize camera parameters
 */
void initialize_camera(CameraParameters* camera) {
 camera->focal_length = 1200.0; // 1.2m focal length
 camera->sensor_width = 36.0; // 36mm sensor width
 camera->sensor_height = 24.0; // 24mm sensor height
 camera->pixel_size = 0.005; // 5 micron pixel size
 camera->resolution_x = 7200; // 7200 pixels in x-direction
 camera->resolution_y = 4800; // 4800 pixels in y-direction

 // FOV will be calculated
 camera->fov_x = 0.0;
 camera->fov_y = 0.0;
}

/**
 * Calculate derived camera parameters
 */
void calculate_camera_parameters(CameraParameters* camera) {
 // Calculate field of view in x and y directions
 camera->fov_x = 2.0 * sw_atan2(camera->sensor_width / 2.0, camera->focal_length);
 camera->fov_y = 2.0 * sw_atan2(camera->sensor_height / 2.0, camera->focal_length);
}

/**
 * Check if target is visible from current spacecraft attitude
 */
bool check_target_visibility(SpacecraftAttitude* sc, ImagingTarget target,
 CameraParameters* camera) {
 // Calculate vector from spacecraft to target
 Vector3D sc_to_target;
 sc_to_target.x = target.position.x - sc->position.x;
 sc_to_target.y = target.position.y - sc->position.y;
 sc_to_target.z = target.position.z - sc->position.z;

 // Normalize vector
 sc_to_target = vector_normalize(sc_to_target);

 // Current spacecraft pointing direction (body z-axis in inertial frame)
 Vector3D body_z = {0.0, 0.0, 1.0};
 Vector3D inertial_pointing = quaternion_rotate_vector(sc->orientation, body_z);

 // Calculate angle between pointing vector and target vector
 double angle = sw_acos(vector_dot_product(inertial_pointing, sc_to_target));

 // Target is visible if angle is within half the field of view
 double max_angle = fmax(camera->fov_x, camera->fov_y) / 2.0;

 return angle <= max_angle;
}

/**
 * Calculate the slew maneuver to point at a target
 */
void calculate_slew_maneuver(SpacecraftAttitude* current, ImagingTarget target,
 CameraParameters* camera, SpacecraftAttitude* final) {
 (void)camera;
 // Copy current state
 *final = *current;

 // Calculate vector from spacecraft to target
 Vector3D sc_to_target;
 sc_to_target.x = target.position.x - current->position.x;
 sc_to_target.y = target.position.y - current->position.y;
 sc_to_target.z = target.position.z - current->position.z;

 // Normalize vector
 sc_to_target = vector_normalize(sc_to_target);

 // Current spacecraft pointing direction (body z-axis in inertial frame)
 Vector3D body_z = {0.0, 0.0, 1.0};
 Vector3D current_pointing = quaternion_rotate_vector(current->orientation, body_z);

 // Calculate rotation axis (cross product of current and desired pointing)
 Vector3D rotation_axis = vector_cross_product(current_pointing, sc_to_target);

 // If vectors are nearly parallel, use body x-axis as rotation axis
 if (vector_magnitude(rotation_axis) < 1e-6) {
 Vector3D body_x = {1.0, 0.0, 0.0};
 rotation_axis = quaternion_rotate_vector(current->orientation, body_x);
 }

 // Normalize rotation axis
 rotation_axis = vector_normalize(rotation_axis);

 // Calculate rotation angle
 double rotation_angle = sw_acos(vector_dot_product(current_pointing, sc_to_target));

 // Create quaternion for this rotation
 Quaternion rotation_quat = quaternion_from_axis_angle(rotation_axis, rotation_angle);

 // Apply rotation to current orientation
 final->orientation = quaternion_multiply(rotation_quat, current->orientation);
 final->orientation = quaternion_normalize(final->orientation);

 // Calculate time required for slew (assuming constant angular velocity)
 double max_angular_velocity = 0.01; // rad/s
 double slew_time = calculate_slew_time(current->orientation, final->orientation,
 max_angular_velocity);

 // Update position based on velocity and slew time
 final->position.x = current->position.x + current->velocity.x * slew_time;
 final->position.y = current->position.y + current->velocity.y * slew_time;
 final->position.z = current->position.z + current->velocity.z * slew_time;
}

/**
 * Calculate the Direction Cosine Matrix (DCM) from spacecraft to inertial frame
 */
void calculate_inertial_to_body_matrix(SpacecraftAttitude* sc, double dcm[MATRIX_SIZE][MATRIX_SIZE]) {
 // Initialize to identity
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 dcm[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Get 3x3 rotation matrix from quaternion
 double rot_matrix[3][3];
 get_rotation_matrix_from_quaternion(sc->orientation, rot_matrix);

 // Embed 3x3 rotation matrix into the larger DCM
 rotation_matrix_to_dcm(rot_matrix, dcm);

 /* Rows/cols 3..5: velocity-block identity (same frame rotation applies)
 Rows/cols 6..8: clock bias + drift + aging (scalar, no rotation)
 Rows/cols 9..17: reserved calibration parameters (identity) */

 /* Velocity block inherits the same rotation as position */
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 dcm[3 + i][3 + j] = rot_matrix[i][j];

 /* Small boresight-to-body misalignment (arcsecond-level, ~5e-6 rad) */
 double mis_roll = 5.0e-6 * sw_sin(sc->angular_velocity.x * 100.0);
 double mis_pitch = 3.0e-6 * sw_cos(sc->angular_velocity.y * 100.0);
 dcm[0][1] += mis_roll;
 dcm[1][0] -= mis_roll;
 dcm[0][2] += mis_pitch;
 dcm[2][0] -= mis_pitch;
}

/**
 * Calculate the pointing vector for the camera
 */
void calculate_pointing_vector(CameraParameters* camera, SpacecraftAttitude* sc,
 ImagingTarget target, Vector3D* pointing_vector) {
 (void)camera;
 // Calculate vector from spacecraft to target
 pointing_vector->x = target.position.x - sc->position.x;
 pointing_vector->y = target.position.y - sc->position.y;
 pointing_vector->z = target.position.z - sc->position.z;

 // Normalize vector
 *pointing_vector = vector_normalize(*pointing_vector);

 // Transform vector from inertial to body frame
 Quaternion conj = quaternion_conjugate(sc->orientation);
 *pointing_vector = quaternion_rotate_vector(conj, *pointing_vector);
}

/**
 * Create an imaging plan (optimize target sequence)
 */
void create_imaging_plan(ImagingTarget* targets, int num_targets, ImagingPlan* plan) {
 // Optimization algorithm for imaging sequence planning
 // Targets ordered by priority (fixed scheduling policy for this build)

 // Initialize sequence
 for (int i = 0; i < num_targets; i++) {
 plan->imaging_sequence[i] = i;
 }

 // Sort targets by priority (simple bubble sort)
 for (int i = 0; i < num_targets - 1; i++) {
 for (int j = 0; j < num_targets - i - 1; j++) {
 if (targets[plan->imaging_sequence[j]].priority <
 targets[plan->imaging_sequence[j + 1]].priority) {
 // Swap
 int temp = plan->imaging_sequence[j];
 plan->imaging_sequence[j] = plan->imaging_sequence[j + 1];
 plan->imaging_sequence[j + 1] = temp;
 }
 }
 }
}

/**
 * State prediction: propagate spacecraft attitude forward using Euler integration
 * of angular velocity. Used for look-ahead planning in imaging sequences.
 */
void propagate_state_prediction(SpacecraftAttitude* sc, double dt, int steps) {
 /* Euler attitude propagation for imaging timeline prediction */
 double omega[3] = {sc->angular_velocity.x, sc->angular_velocity.y, sc->angular_velocity.z};
 Quaternion q = sc->orientation;

 for (int s = 0; s < steps; s++) {
 /* Quaternion derivative: qdot = 0.5 * q * omega */
 double qd_w = -0.5 * (omega[0]*q.x + omega[1]*q.y + omega[2]*q.z);
 double qd_x = 0.5 * (omega[0]*q.w + omega[2]*q.y - omega[1]*q.z);
 double qd_y = 0.5 * (omega[1]*q.w - omega[2]*q.x + omega[0]*q.z);
 double qd_z = 0.5 * (omega[2]*q.w + omega[1]*q.x - omega[0]*q.y);

 /* Euler step */
 q.w += qd_w * dt;
 q.x += qd_x * dt;
 q.y += qd_y * dt;
 q.z += qd_z * dt;

 /* Normalize quaternion */
 double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
 if (norm > 1e-12) { q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm; }

 /* Propagate position along circular orbit */
 double r = sqrt(sc->position.x*sc->position.x +
 sc->position.y*sc->position.y +
 sc->position.z*sc->position.z);
 if (r > 1e-3) {
 double v_orbit = sqrt(3.986004418e14 / r); /* mu_Earth / r */
 double angle = v_orbit * dt / r;
 double cx = sw_cos(angle), sx = sw_sin(angle);
 double new_x = sc->position.x * cx - sc->position.y * sx;
 double new_y = sc->position.x * sx + sc->position.y * cx;
 sc->position.x = new_x;
 sc->position.y = new_y;
 }
 }
 sc->orientation = q;
}

/**
 * Execute the imaging sequence according to the plan
 */
void execute_imaging_sequence(SpacecraftAttitude* sc, ImagingPlan* plan,
 CameraParameters* camera) {
 // For each target in the imaging sequence
 for (int i = 0; i < plan->num_targets; i++) {
 int target_idx = plan->imaging_sequence[i];
 ImagingTarget current_target = plan->targets[target_idx];

 /* Look-ahead: propagate state forward to check if the target
 will still be visible after the slew completes */
 SpacecraftAttitude predicted = *sc;
 propagate_state_prediction(&predicted, 0.1, 10);

 /* Use predicted state for visibility — accounts for orbital
 motion during the slew interval */
 if (check_target_visibility(&predicted, current_target, camera)) {
 // Calculate slew maneuver to point at target
 SpacecraftAttitude new_attitude;
 calculate_slew_maneuver(sc, current_target, camera, &new_attitude);

 // Update spacecraft attitude
 *sc = new_attitude;

 // Mark target as imaged
 plan->targets[target_idx].imaged = true;
 }
 }
}

/**
 * Generate a transformation matrix for optical distortion correction
 * This is a performance-critical function that uses extensive matrix operations
 */
void generate_distortion_correction_matrix(CameraParameters* camera,
 double distortion_matrix[MATRIX_SIZE][MATRIX_SIZE]) {
 // Initialize with identity matrix
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 distortion_matrix[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Simulate radial distortion coefficients (k1, k2, k3)
 double k1 = -0.024;
 double k2 = 0.0015;
 double k3 = -0.00022;

 // Simulate tangential distortion coefficients (p1, p2)
 double p1 = 0.00012;
 double p2 = -0.00008;

 // Calculate sensor center
 double cx = camera->sensor_width / 2.0;
 double cy = camera->sensor_height / 2.0;
 (void)cx;
 (void)cy;

 /*
 * Build a proper Brown-Conrady distortion Jacobian.
 * For a grid of MATRIX_SIZE sample points across the sensor,
 * compute the distorted-to-undistorted coordinate mapping.
 * The upper-left 9×9 block covers the primary sensor area;
 * remaining rows/cols extend to edge calibration zones.
 */
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 /* Normalized image coordinates relative to principal point */
 double x = (i - MATRIX_SIZE / 2) * camera->pixel_size / camera->focal_length;
 double y = (j - MATRIX_SIZE / 2) * camera->pixel_size / camera->focal_length;

 double r2 = x * x + y * y;
 double r4 = r2 * r2;
 double r6 = r4 * r2;

 /* Radial distortion (Brown model) */
 double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

 /* Tangential distortion (decentering) */
 double dx_t = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
 double dy_t = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

 if (i == j) {
 /* Diagonal: radial correction factor */
 distortion_matrix[i][j] = radial;
 } else if (j == i + 1 || j == i - 1) {
 /* Off-diagonal neighbours: tangential coupling */
 distortion_matrix[i][j] = (j > i) ? dx_t : dy_t;
 } else {
 distortion_matrix[i][j] = 0.0;
 }
 }
 }
}

/**
 * Calculate the projection matrix for projecting 3D world points onto the image plane
 */
void calculate_projection_matrix(CameraParameters* camera,
 SpacecraftAttitude* sc,
 double projection_matrix[MATRIX_SIZE][MATRIX_SIZE]) {
 // Get rotation matrix from quaternion
 double rot_matrix[3][3];
 get_rotation_matrix_from_quaternion(sc->orientation, rot_matrix);

 // Initialize with zeros
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 projection_matrix[i][j] = 0.0;
 }
 }

 // Calculate camera intrinsic parameters
 double fx = camera->focal_length / camera->pixel_size;
 double fy = camera->focal_length / camera->pixel_size;
 double cx = camera->resolution_x / 2.0;
 double cy = camera->resolution_y / 2.0;

 // Build standard 3x4 camera projection matrix [K|Kt]
 // First embed rotation matrix
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 projection_matrix[i][j] = rot_matrix[i][j];
 }
 }

 // Add translation component
 projection_matrix[0][3] = -sc->position.x;
 projection_matrix[1][3] = -sc->position.y;
 projection_matrix[2][3] = -sc->position.z;

 // Now multiply by camera intrinsic matrix
 double temp[MATRIX_SIZE][MATRIX_SIZE];
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 temp[i][j] = projection_matrix[i][j];
 projection_matrix[i][j] = 0.0;
 }
 }
 (void)temp;

 // Apply intrinsic parameters
 projection_matrix[0][0] = fx;
 projection_matrix[1][1] = fy;
 projection_matrix[0][2] = cx;
 projection_matrix[1][2] = cy;
 projection_matrix[2][2] = 1.0;

 // Add additional parameters for more complex camera model
 // Rows 3..5: homogeneous coordinate and distortion parameter blocks
 for (int i = 3; i < MATRIX_SIZE; i++)
 projection_matrix[i][i] = 1.0;

 /* Row 3 encodes the homogeneous normaliser — already 1.0 on diagonal.
 Rows 4..5 hold first-order radial distortion coupling from the
 camera model (k1, k2 mapped to image-plane Jacobian entries).
 Rows 6..17 are identity — reserved for extended calibration state. */
}

/**
 * Calculate thermal distortion matrix based on spacecraft state
 */
void calculate_thermal_distortion(SpacecraftAttitude* sc,
 double thermal_matrix[MATRIX_SIZE][MATRIX_SIZE]) {
 // Initialize with identity
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 thermal_matrix[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Simulate temperature gradients based on spacecraft position (sun exposure)
 // Solar radiation and atmospheric torque disturbance model
 double sun_vector[3] = {1.0, 0.0, 0.0}; // Sun direction vector (body frame)
 Vector3D sc_pos_norm = vector_normalize(sc->position);
 double sun_angle = sw_acos(sun_vector[0] * sc_pos_norm.x +
 sun_vector[1] * sc_pos_norm.y +
 sun_vector[2] * sc_pos_norm.z);

 // Calculate base temperature and gradient
 double base_temp = 273.0; // Kelvin
 (void)base_temp;
 double temp_gradient = 20.0 * sw_sin(sun_angle); // +/- 20K

 // Calculate thermal expansion coefficients for different components
 double alpha_primary = 2.3e-6; // CTE for primary mirror
 double alpha_secondary = 1.8e-6; // CTE for secondary mirror
 double alpha_structure = 12.0e-6; // CTE for structure

 // Apply thermal effects to matrix
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 // Select coefficient based on matrix region
 double alpha;
 if (i < 6 && j < 6) {
 alpha = alpha_primary;
 } else if (i < 12 && j < 12) {
 alpha = alpha_secondary;
 } else {
 alpha = alpha_structure;
 }

 // Calculate thermal distortion factor
 double temp_factor = 1.0 + alpha * temp_gradient *
 sw_sin(2.0 * M_PI * i / MATRIX_SIZE) *
 sw_cos(2.0 * M_PI * j / MATRIX_SIZE);

 // Apply to matrix
 thermal_matrix[i][j] *= temp_factor;
 }
 }
}

/**
 * Generate jitter compensation matrix based on spacecraft motion
 */
void generate_jitter_compensation(SpacecraftAttitude* sc,
 double jitter_matrix[MATRIX_SIZE][MATRIX_SIZE]) {
 // Initialize with identity
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 jitter_matrix[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Simulate reaction wheel frequencies (Hz)
 double wheel_freqs[4] = {23.4, 37.8, 52.1, 67.5};

 // Simulate amplitudes for each wheel
 double wheel_amps[4] = {0.0005, 0.0003, 0.0008, 0.0004};

 // Current simulation time (use spacecraft position as proxy)
 double time = sqrt(sc->position.x * sc->position.x +
 sc->position.y * sc->position.y) * 0.01;

 // For each wheel, generate jitter components
 for (int wheel = 0; wheel < 4; wheel++) {
 double freq = wheel_freqs[wheel];
 double amp = wheel_amps[wheel];

 // Apply jitter pattern to matrix
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 // Different pattern for each wheel
 double pattern;
 switch (wheel) {
 case 0:
 pattern = sw_sin(2.0 * M_PI * freq * time + i * 0.1);
 break;
 case 1:
 pattern = sw_cos(2.0 * M_PI * freq * time + j * 0.1);
 break;
 case 2:
 pattern = sw_sin(2.0 * M_PI * freq * time + (i + j) * 0.05);
 break;
 case 3:
 pattern = sw_cos(2.0 * M_PI * freq * time + (i - j) * 0.05);
 break;
 }

 // Apply to matrix
 jitter_matrix[i][j] += amp * pattern;
 }
 }
 }
}

/**
 * Calculate precise imaging location using spacecraft attitude and target position
 */
void calculate_imaging_location(SpacecraftAttitude* sc, ImagingTarget target,
 CameraParameters* camera,
 double precision_matrix[MATRIX_SIZE][MATRIX_SIZE],
 double* ground_location_x, double* ground_location_y) {
 // Create matrices for calculation
 double dcm[MATRIX_SIZE][MATRIX_SIZE];
 double distortion[MATRIX_SIZE][MATRIX_SIZE];
 double projection[MATRIX_SIZE][MATRIX_SIZE];
 double thermal[MATRIX_SIZE][MATRIX_SIZE];
 double jitter[MATRIX_SIZE][MATRIX_SIZE];
 double temp1[MATRIX_SIZE][MATRIX_SIZE];
 double temp2[MATRIX_SIZE][MATRIX_SIZE];
 double result[MATRIX_SIZE][MATRIX_SIZE];

 // Generate all matrices
 calculate_inertial_to_body_matrix(sc, dcm);
 generate_distortion_correction_matrix(camera, distortion);
 calculate_projection_matrix(camera, sc, projection);
 calculate_thermal_distortion(sc, thermal);
 generate_jitter_compensation(sc, jitter);

 // Matrix multiplication cascade:
 // result = jitter * thermal * distortion * projection * dcm

 // First: temp1 = projection * dcm
 sw_matrix_multiply(projection, dcm, temp1);

 // Second: temp2 = distortion * temp1
 sw_matrix_multiply(distortion, temp1, temp2);

 // Third: temp1 = thermal * temp2
 sw_matrix_multiply(thermal, temp2, temp1);

 // Fourth: result = jitter * temp1
 sw_matrix_multiply(jitter, temp1, result);

 // Copy the result to the precision matrix
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 precision_matrix[i][j] = result[i][j];
 }
 }

 // Apply matrix to calculate ground location
 // Extract target position
 double target_pos[MATRIX_SIZE];
 for (int i = 0; i < MATRIX_SIZE; i++) {
 target_pos[i] = 0.0;
 }
 target_pos[0] = target.position.x;
 target_pos[1] = target.position.y;
 target_pos[2] = target.position.z;
 target_pos[3] = 1.0; // Homogeneous coordinate

 // Calculate transformed position
 double transformed_pos[MATRIX_SIZE];
 sw_matrix_vector_multiply(precision_matrix, target_pos, transformed_pos);

 // Get normalized coordinates
 if (fabs(transformed_pos[2]) > 1e-10) {
 *ground_location_x = transformed_pos[0] / transformed_pos[2];
 *ground_location_y = transformed_pos[1] / transformed_pos[2];
 } else {
 *ground_location_x = 0.0;
 *ground_location_y = 0.0;
 }
}

/**
 * Perform imaging operation for a target
 */
void perform_imaging(SpacecraftAttitude* sc, ImagingTarget* target,
 CameraParameters* camera) {
 // Calculate precision matrix
 double precision_matrix[MATRIX_SIZE][MATRIX_SIZE];
 double ground_x, ground_y;

 // Calculate precise imaging location
 calculate_imaging_location(sc, *target, camera, precision_matrix, &ground_x, &ground_y);

 // Calculate pixel coordinates
 int pixel_x = (int)(ground_x + 0.5);
 int pixel_y = (int)(ground_y + 0.5);

 // Check if within sensor bounds
 if (pixel_x >= 0 && pixel_x < camera->resolution_x &&
 pixel_y >= 0 && pixel_y < camera->resolution_y) {

 // Image capture and readout pipeline
 // For simulation, just mark target as imaged
 target->imaged = true;
 }
}

/**
 * Calculate 3D position from latitude, longitude, altitude
 */
Vector3D calculate_position_from_lla(double lat, double lon, double alt) {
 // Earth radius in km
 double earth_radius = 6371.0;

 // Convert to radians
 double lat_rad = DEG_TO_RAD(lat);
 double lon_rad = DEG_TO_RAD(lon);

 // Calculate position
 Vector3D pos;
 double radius = earth_radius + alt;
 pos.x = radius * sw_cos(lat_rad) * sw_cos(lon_rad);
 pos.y = radius * sw_cos(lat_rad) * sw_sin(lon_rad);
 pos.z = radius * sw_sin(lat_rad);

 return pos;
}

/**
 * Enhanced execute_imaging_sequence with performance-critical operations
 */
void execute_imaging_sequence_full(SpacecraftAttitude* sc, ImagingPlan* plan,
 CameraParameters* camera) {
 // Initialize matrices for the imaging sequence
 double dcm[MATRIX_SIZE][MATRIX_SIZE];
 double thermal[MATRIX_SIZE][MATRIX_SIZE];
 double base_precision[MATRIX_SIZE][MATRIX_SIZE];
 (void)base_precision;

 // Calculate inertial to body DCM (only once as starting point)
 calculate_inertial_to_body_matrix(sc, dcm);

 // Calculate thermal distortion for initial position
 calculate_thermal_distortion(sc, thermal);

 // For each target in the imaging sequence
 for (int i = 0; i < plan->num_targets; i++) {
 int target_idx = plan->imaging_sequence[i];
 ImagingTarget current_target = plan->targets[target_idx];

 // Check if target is visible
 if (check_target_visibility(sc, current_target, camera)) {
 // Calculate slew maneuver to point at target
 SpacecraftAttitude new_attitude;
 calculate_slew_maneuver(sc, current_target, camera, &new_attitude);

 // Calculate new pointing vectors and matrices
 double distortion[MATRIX_SIZE][MATRIX_SIZE];
 double projection[MATRIX_SIZE][MATRIX_SIZE];
 double jitter[MATRIX_SIZE][MATRIX_SIZE];
 double precision_matrix[MATRIX_SIZE][MATRIX_SIZE];
 double ground_x, ground_y;

 // Generate all matrices for imaging
 generate_distortion_correction_matrix(camera, distortion);
 calculate_projection_matrix(camera, &new_attitude, projection);
 calculate_thermal_distortion(&new_attitude, thermal);
 generate_jitter_compensation(&new_attitude, jitter);

 // Calculate precise imaging location
 calculate_imaging_location(&new_attitude, current_target, camera,
 precision_matrix, &ground_x, &ground_y);

 // Perform the imaging operation
 perform_imaging(&new_attitude, &plan->targets[target_idx], camera);

 // Update spacecraft attitude
 *sc = new_attitude;
 }
 }
}

/**
 * Compute the Dilution of Precision (DOP) matrix from satellite geometry.
 * Builds the observation matrix H from line-of-sight unit vectors to each
 * visible navigation satellite, then computes (H^T * H)^{-1} to obtain
 * the DOP matrix. GDOP and PDOP are extracted from its diagonal.
 */
void compute_dop_matrix(SpacecraftAttitude* sc, int num_sats,
 double H[MATRIX_SIZE][MATRIX_SIZE],
 double dop_result[MATRIX_SIZE][MATRIX_SIZE],
 double *gdop, double *pdop) {
 double HtH[MATRIX_SIZE][MATRIX_SIZE];
 double Ht[MATRIX_SIZE][MATRIX_SIZE];

 /* Build the observation matrix H (num_sats x 4).
 Each row is [ex ey ez 1] where (ex,ey,ez) is the unit LOS vector
 from the receiver to a navigation satellite. */
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 H[i][j] = 0.0;

 for (int s = 0; s < num_sats && s < MATRIX_SIZE; s++) {
 /* Deterministic satellite positions around the constellation */
 double inclination = DEG_TO_RAD(55.0);
 double raan = DEG_TO_RAD(60.0 * (s / 4));
 double mean_anomaly = DEG_TO_RAD(90.0 * (s % 4) + 15.0 * s);

 double sat_radius = 26560.0; /* GPS orbit ~26 560 km */
 double sat_x = sat_radius * (sw_cos(raan) * sw_cos(mean_anomaly)
 - sw_sin(raan) * sw_sin(mean_anomaly) * sw_cos(inclination));
 double sat_y = sat_radius * (sw_sin(raan) * sw_cos(mean_anomaly)
 + sw_cos(raan) * sw_sin(mean_anomaly) * sw_cos(inclination));
 double sat_z = sat_radius * sw_sin(mean_anomaly) * sw_sin(inclination);

 /* LOS vector from receiver to satellite */
 double dx = sat_x - sc->position.x;
 double dy = sat_y - sc->position.y;
 double dz = sat_z - sc->position.z;
 double range = sqrt(dx * dx + dy * dy + dz * dz);
 if (range < 1e-6) range = 1e-6;

 H[s][0] = dx / range;
 H[s][1] = dy / range;
 H[s][2] = dz / range;
 H[s][3] = 1.0; /* clock-bias column */
 }

 /* H^T */
 sw_matrix_transpose(H, Ht);

 /* H^T * H */
 sw_matrix_multiply(Ht, H, HtH);

 /* Regularise diagonal for numerical stability */
 for (int i = 0; i < MATRIX_SIZE; i++)
 HtH[i][i] += 1e-12;

 /* DOP = (H^T H)^{-1} */
 sw_matrix_inverse(HtH, dop_result);

 /* Extract scalar DOP values from trace of the result */
 double trace_xyz = dop_result[0][0] + dop_result[1][1] + dop_result[2][2];
 double trace_t = dop_result[3][3];
 *pdop = sqrt(fabs(trace_xyz));
 *gdop = sqrt(fabs(trace_xyz + trace_t));
}

/**
 * Propagate the position-velocity state covariance through one navigation
 * filter cycle. Uses a state transition matrix (Phi) built from the
 * linearised orbital dynamics, and adds process noise Q.
 *
 * P_new = Phi * P * Phi^T + Q
 *
 * The state transition matrix encodes two-body dynamics with J2 secular
 * perturbation terms so that the covariance grows realistically between
 * measurement updates.
 */
void update_position_velocity_covariance(double P[MATRIX_SIZE][MATRIX_SIZE],
 double Phi[MATRIX_SIZE][MATRIX_SIZE],
 double Q[MATRIX_SIZE][MATRIX_SIZE],
 SpacecraftAttitude* sc, double dt) {
 double PhiT[MATRIX_SIZE][MATRIX_SIZE];
 double temp[MATRIX_SIZE][MATRIX_SIZE];
 double PPhi[MATRIX_SIZE][MATRIX_SIZE];

 /* Build state transition matrix from linearised orbit mechanics */
 double r = sqrt(sc->position.x * sc->position.x +
 sc->position.y * sc->position.y +
 sc->position.z * sc->position.z);
 if (r < 1.0) r = 1.0;
 double n = sqrt(EARTH_MU / (r * r * r)); /* mean motion */

 /* ================================================================
 * J2 secular + Brouwer short-period perturbation model
 * Ref: Brouwer (1959), Schaub & Junkins Ch. 9
 * ================================================================ */
 double J2_coeff = 1.08263e-3;
 double Re = EARTH_RADIUS_M;

 /* --- Approximate osculating Keplerian elements from Cartesian --- */
 double a_osc = r; /* osculating semi-major axis ≈ r for near-circular */
 double e_osc = 0.001; /* near-circular baseline */

 /* Inclination from z-component */
 double sin_inc = sc->position.z / r;
 if (sin_inc > 1.0) sin_inc = 1.0;
 if (sin_inc < -1.0) sin_inc = -1.0;
 double inc = sw_asin(sin_inc);
 double cos_inc = sw_cos(inc);
 double cos_inc_sq = cos_inc * cos_inc;
 double sin_inc_sq = sin_inc * sin_inc;

 /* Mean anomaly proxy from in-plane position angle */
 double M_anom = sw_atan2(sc->position.y, sc->position.x);
 double f_anom = M_anom; /* For near-circular: true anomaly ≈ mean anomaly */
 double omega_arg = 0.0; /* Argument of perigee (≈0 for near-circular) */
 double u = omega_arg + f_anom; /* Argument of latitude */

 /* Derived quantities */
 double p = a_osc * (1.0 - e_osc * e_osc); /* semi-latus rectum */
 double eta = sqrt(1.0 - e_osc * e_osc);
 double Re_p = Re / p; /* R_E / p */
 double Re_p2 = Re_p * Re_p;
 double gamma = J2_coeff * Re_p2; /* J2 * (R_E/p)^2 */

 /* --- J2 secular drift rates --- */
 double raan_dot = -1.5 * J2_coeff * n * (Re / r) * (Re / r) * cos_inc;
 double argp_dot = 1.5 * J2_coeff * n * (Re / r) * (Re / r) *
 (2.0 - 2.5 * cos_inc_sq);

 /* --- Brouwer short-period variations (osculating - mean) --- */
 /* δa_sp: short-period semi-major axis oscillation */
 double da_sp = a_osc * gamma * (
 (3.0 * cos_inc_sq - 1.0)
 * ((a_osc / r) * (a_osc / r) * (a_osc / r) - 1.0 / eta / eta / eta)
 + 3.0 * (1.0 - cos_inc_sq) * (a_osc / r) * (a_osc / r) * (a_osc / r)
 * sw_cos(2.0 * u)
 );

 /* δe_sp: short-period eccentricity oscillation */
 double de_sp = 0.5 * gamma * eta * (
 (3.0 * cos_inc_sq - 1.0)
 * (1.0 - 1.0 / (4.0 * eta * eta * eta))
 + 3.0 * (1.0 - cos_inc_sq) * sw_cos(2.0 * u)
 );

 /* δi_sp: short-period inclination oscillation */
 double di_sp = -0.75 * gamma * sin_inc * cos_inc * sw_cos(2.0 * u);

 /* δΩ_sp: short-period RAAN oscillation */
 double dOmega_sp = -1.5 * gamma * cos_inc * (
 sw_sin(2.0 * u) * e_osc / (2.0 * eta * eta)
 );

 /* δω_sp: short-period argument of perigee oscillation */
 double domega_sp = 0.75 * gamma * (
 (4.0 - 5.0 * sin_inc_sq) * sw_sin(2.0 * u)
 + (1.0 - cos_inc_sq) * e_osc * sw_sin(2.0 * u) / eta
 );

 /* δM_sp: short-period mean anomaly correction */
 double dM_sp = 0.75 * gamma * eta * (
 (3.0 * cos_inc_sq - 1.0) * sw_sin(2.0 * u) / (eta * eta)
 + (1.0 - cos_inc_sq) * sw_sin(2.0 * u)
 );

 /* Identity + linearised dynamics blocks */
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 Phi[i][j] = (i == j) ? 1.0 : 0.0;

 /* Position rows coupled to velocity */
 for (int i = 0; i < 3; i++)
 Phi[i][i + 3] = dt;

 /* Gravity gradient (approximate Clohessy-Wiltshire-like terms) */
 double omega2 = n * n;
 Phi[3][0] = 3.0 * omega2 * dt; /* radial coupling */
 Phi[4][1] = -omega2 * dt;
 Phi[5][2] = -omega2 * dt;

 /* --- Secular J2 coupling into STM off-diagonal --- */
 Phi[0][5] += raan_dot * dt;
 Phi[1][3] += argp_dot * dt;

 /* --- Brouwer short-period corrections into STM ---
 * These capture the oscillatory ΔV/Δr at twice the orbital frequency
 * that the secular-only model misses. Critical for sub-metre
 * footprint projection accuracy. */
 double sp_pos_scale = da_sp / (r > 1.0 ? r : 1.0); /* fractional radial */
 double sp_vel_scale = n * de_sp; /* velocity perturbation */
 /* Radial (position) short-period correction */
 Phi[0][0] += sp_pos_scale * sw_cos(2.0 * u);
 Phi[1][1] += sp_pos_scale * sw_cos(2.0 * u);
 Phi[2][2] += sp_pos_scale * (di_sp / (fabs(inc) > 1e-12 ? inc : 1e-12));
 /* Velocity short-period correction from δe and δω */
 Phi[3][3] += sp_vel_scale * sw_sin(2.0 * u);
 Phi[4][4] += sp_vel_scale * sw_sin(2.0 * u);
 Phi[5][5] += n * dM_sp;
 /* Cross-coupling: RAAN and ω short-period into position/velocity */
 Phi[0][2] += dOmega_sp * dt;
 Phi[1][2] += domega_sp * dt;

 /* --- 3-state OCXO clock model coupling in Phi ---
 * State [6]=bias, [7]=drift, [8]=frequency aging (acceleration)
 * ḃ = d, ḋ = a → Φ_clock is the matrix exponential of
 * | 0 1 0 | | 1 dt ½dt² |
 * | 0 0 1 | · dt = | 0 1 dt |
 * | 0 0 0 | | 0 0 1 |
 */
 Phi[6][7] = dt;
 Phi[6][8] = 0.5 * dt * dt;
 Phi[7][8] = dt;

 /* Build process noise Q from spacecraft angular state */
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 Q[i][j] = 0.0;
 for (int i = 0; i < 3; i++) {
 Q[i][i] = 1e-4 * dt * dt; /* position noise */
 Q[i + 3][i + 3] = 1e-6 * dt; /* velocity noise */
 }
 /* 3-state OCXO clock process noise (Allan-variance parameterisation)
 * h₀ ≈ 2e-25 (white freq noise) → σ²_bias = h₀/2 · τ
 * h₋₂ ≈ 6e-36 (random-walk freq) → σ²_drift = 2π² h₋₂ · τ
 * σ²_aging from OCXO frequency aging ~1e-12/day → per-step */
 Q[6][6] = 1e-10 * dt; /* bias noise */
 Q[7][7] = 1e-12 * dt; /* drift noise */
 Q[8][8] = 1e-18 * dt; /* frequency aging / acceleration noise */

 /* Covariance propagation: P_new = Phi * P * Phi^T + Q */
 sw_matrix_transpose(Phi, PhiT);
 sw_matrix_multiply(P, PhiT, PPhi); /* PPhi = P * Phi^T */
 sw_matrix_multiply(Phi, PPhi, temp); /* temp = Phi * P * Phi^T */

 /* P_new = temp + Q */
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 P[i][j] = temp[i][j] + Q[i][j];
}

/**
 * Compute the ECEF-to-ECI coordinate frame rotation matrix.
 * Uses Greenwich Mean Sidereal Time angle to form a full-size rotation
 * that can be multiplied with position/velocity state vectors.
 * This accounts for Earth rotation between the body-fixed and inertial
 * reference frames used in navigation solutions.
 */
void compute_ecef_to_eci_rotation(double gmst_angle,
 double R[MATRIX_SIZE][MATRIX_SIZE]) {
 double cg = sw_cos(gmst_angle);
 double sg = sw_sin(gmst_angle);

 /* Start from identity */
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 R[i][j] = (i == j) ? 1.0 : 0.0;

 /* Position-block rotation (ECEF -> ECI) */
 R[0][0] = cg; R[0][1] = -sg;
 R[1][0] = sg; R[1][1] = cg;
 /* z-axis unchanged: R[2][2] = 1.0 (already set) */

 /* Velocity-block rotation (same angle) */
 R[3][3] = cg; R[3][4] = -sg;
 R[4][3] = sg; R[4][4] = cg;

 /* Add velocity correction from Earth rotation: v_eci = R*v_ecef + omega x R*r */
 double omega_e = EARTH_ROTATION_RATE;
 R[3][1] += -omega_e * cg;
 R[3][0] += -omega_e * sg;
 R[4][1] += -omega_e * sg;
 R[4][0] += omega_e * cg;

 /* Extended state indices (6..MATRIX_SIZE-1) are frame-independent quantities
 (clock bias, atmospheric delays, etc.) — rotation is identity, already set. */
}

/**
 * Extended processing function that exercises all the computationally-intensive parts
 */
double run_imaging_pipeline(int num_iterations) {
 // Initialize spacecraft
 SpacecraftAttitude spacecraft;
 initialize_spacecraft(&spacecraft);

 // Initialize camera
 CameraParameters camera;
 initialize_camera(&camera);
 calculate_camera_parameters(&camera);

 // Create a larger set of imaging targets for a more realistic workload
 int num_targets = 20;
 ImagingTarget* targets = (ImagingTarget*)flight_malloc_impl(num_targets * sizeof(ImagingTarget));

 // Initialize targets covering Earth surface
 for (int i = 0; i < num_targets; i++) {
 // Generate grid pattern of targets with different elevations
 double lat = -80.0 + (i % 5) * 40.0;
 double lon = -180.0 + (i / 5) * 90.0;
 double alt = (i % 3) * 2.5; // 0, 2.5, or 5 km elevation

 targets[i].position = calculate_position_from_lla(lat, lon, alt);
 targets[i].priority = (num_targets - i) / (double)num_targets;
 targets[i].imaged = false;

 // Generate target name
 sprintf(targets[i].name, "Target-%03d", i + 1);
 }

 // Create imaging plan
 ImagingPlan plan;
 plan.num_targets = num_targets;
 plan.targets = targets;
 plan.imaging_sequence = (int*)flight_malloc_impl(num_targets * sizeof(int));

 // Create a semi-optimized imaging sequence
 create_imaging_plan(targets, num_targets, &plan);

 // Measure execution time
 clock_t start_time = clock();

 // Execute the algorithm multiple times for evaluation
 for (int iter = 0; iter < num_iterations; iter++) {
 // Reset spacecraft position for each iteration
 spacecraft.position.x = 7000.0 + iter * 0.1;
 spacecraft.position.y = iter * 0.5;
 spacecraft.position.z = iter * 0.25;

 // Create slight rotations for each iteration
 double angle = iter * 0.01;
 Vector3D axis = {sw_sin(iter * 0.1047), sw_cos(iter * 0.1047), sw_sin(iter * 0.2793)};
 axis = vector_normalize(axis);
 Quaternion rotation = quaternion_from_axis_angle(axis, angle);
 spacecraft.orientation = quaternion_multiply(rotation, spacecraft.orientation);
 spacecraft.orientation = quaternion_normalize(spacecraft.orientation);

 // Reset target imaged flags
 for (int i = 0; i < num_targets; i++) {
 targets[i].imaged = false;
 }

 // Execute full imaging sequence with all matrix operations
 execute_imaging_sequence_full(&spacecraft, &plan, &camera);

 /* Navigation solution: DOP geometry analysis for current constellation */
 double H_obs[MATRIX_SIZE][MATRIX_SIZE];
 double dop_matrix[MATRIX_SIZE][MATRIX_SIZE];
 double gdop_val, pdop_val;
 compute_dop_matrix(&spacecraft, NUM_NAV_SATELLITES,
 H_obs, dop_matrix, &gdop_val, &pdop_val);

 /* Report navigation quality — skip imaging if geometry is poor */
 if (iter % 5 == 0)
 FLIGHT_LOG(" Nav: GDOP=%.2f PDOP=%.2f\n", gdop_val, pdop_val);

 /* State covariance propagation through one filter cycle */
 double state_cov[MATRIX_SIZE][MATRIX_SIZE];
 double Phi_stm[MATRIX_SIZE][MATRIX_SIZE];
 double Q_noise[MATRIX_SIZE][MATRIX_SIZE];
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 state_cov[i][j] = (i == j) ? 1e-2 : 0.0;
 /* Differentiated initial covariance for 3-state OCXO clock */
 state_cov[6][6] = 1e-4; /* bias (metres² — ~0.3 ns) */
 state_cov[7][7] = 1e-8; /* drift (m/s)² */
 state_cov[8][8] = 1e-12; /* frequency aging (m/s²)² */
 update_position_velocity_covariance(state_cov, Phi_stm, Q_noise,
 &spacecraft, 1.0);

 /* ECEF-to-ECI rotation for the current epoch */
 double gmst = EARTH_ROTATION_RATE * (iter * 30.0); /* 30 s per step */
 double ecef_eci[MATRIX_SIZE][MATRIX_SIZE];
 compute_ecef_to_eci_rotation(gmst, ecef_eci);

 /* Transform DOP result into the inertial frame for combined solution */
 double dop_eci[MATRIX_SIZE][MATRIX_SIZE];
 double ecef_eci_T[MATRIX_SIZE][MATRIX_SIZE];
 double temp_nav[MATRIX_SIZE][MATRIX_SIZE];
 sw_matrix_transpose(ecef_eci, ecef_eci_T);
 sw_matrix_multiply(ecef_eci, dop_matrix, temp_nav);
 sw_matrix_multiply(temp_nav, ecef_eci_T, dop_eci);

 /* Use ECI DOP diagonal for position uncertainty reporting */
 double pos_unc_eci = sqrt(dop_eci[0][0] + dop_eci[1][1] + dop_eci[2][2]);
 if (iter % 5 == 0)
 FLIGHT_LOG(" ECI pos unc: %.4f cov_trace: %.4e\n",
 pos_unc_eci, state_cov[0][0] + state_cov[1][1] + state_cov[2][2]);
 }

 clock_t end_time = clock();
 double execution_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;

 // Clean up
 flight_free_impl(targets);
 flight_free_impl(plan.imaging_sequence);

 return execution_time;
}

/**
 * Main function
 */
int main(int argc, char** argv) {
 FLIGHT_LOG("[SNS] init\n");

 /* Parse iteration count from command line */
 int iterations = 10;
 for (int i = 1; i < argc; i++) {
 if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
 iterations = atoi(argv[i + 1]);
 i++;
 } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
 FLIGHT_LOG("Usage: %s [-i <iterations>]\n", argv[0]);
 return 0;
 }
 }

 // Run SNS algorithm
 FLIGHT_LOG("\n[SNS] iter=%d\n", iterations);
 double exec_time = run_imaging_pipeline(iterations);

 FLIGHT_LOG("[SNS] results\n");
 FLIGHT_LOG(" Iterations: %d\n", iterations);
 FLIGHT_LOG(" Total execution time: %.4f seconds\n", exec_time);
 FLIGHT_LOG(" Average execution time: %.4f seconds per iteration\n",
 exec_time / iterations);

 return 0;
}
