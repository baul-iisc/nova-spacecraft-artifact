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
 * Planetary Rover SLAM Algorithm — HW MAT VARIANT
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: AME 3×3 matrix tile accelerator (SYS_MMACD/MINVD/MADDFD/MSUBFD)
 * Application: (see workload description below)
 *
 * Description:
 * Implements simultaneous localization and mapping (SLAM) for planetary rover
 * autonomous navigation. This workload performs EKF-SLAM with landmark tracking,
 * image processing (Sobel edge detection, Gaussian filtering) for visual odometry,
 * and map update using dynamically-sized (3+2L) state covariance matrices. Derived from the
 * lunar rover navigation system and applicable to future
 * planetary surface exploration rovers.
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
// Configuration parameters
#define MAX_LANDMARKS 1000 // Maximum number of landmarks in the map
#define MAX_POSES 500 // Maximum number of rover poses to track
#define STATE_DIM 3 // Rover state dimension (x, y, theta)
#define MEAS_DIM 2 // Measurement dimension (range, bearing)
#define LANDMARK_DIM 2 // Landmark dimension (x, y)
#define MATRIX_BLOCK_SIZE 4 // Block size for matrix operations
#define LOOP_DETECT_WINDOW 10 // Window size for loop closure detection
#define MAX_LOOP_CLOSURES 500 // Maximum number of loop closures (sparse storage)
#define IMG_SIZE 64 /* downsampled NavCam patch (flight: 256×256) */

/* ------------------------------------------------------------------ */
/* Flight-profile static memory (link-time BSS placement) */
/* Strict Flight Profile (crewed spacecraft mission / DO-178C): all memory for the */
/* maximum landmark state (MAX_LANDMARKS × LANDMARK_DIM doubles) is */
/* pre-allocated at link-time to eliminate heap fragmentation. */
/* ------------------------------------------------------------------ */

/** Maximum EKF-SLAM state dimension: rover pose (3) + all landmarks (1000×2) */
#define MAX_SLAM_DIM (STATE_DIM + MAX_LANDMARKS * LANDMARK_DIM) /* 2003 */

/* Persistent SLAM state vector backing store (≈16 KB in BSS) */
static double g_state_pool[MAX_SLAM_DIM];

/* Persistent SLAM covariance backing store (2003² doubles ≈ 32 MB BSS).
 * Full-covariance EKF-SLAM with 1000 landmarks requires this. Flight
 * implementations would use an information-filter or √-form; we keep
 * the dense matrix for hardware matrix-accelerator profiling. */
static double g_cov_pool[MAX_SLAM_DIM * MAX_SLAM_DIM];

/* Scratch arena for ALL temporary Matrix / Vector allocations.
 * Stack discipline: each function calls scratch_save()/scratch_restore()
 * so memory is reclaimed on scope exit — zero fragmentation, zero malloc.
 * Worst-case concurrent: Cholesky inside pose-graph optimisation
 * requires J[1500×1500] + L[1500×1500] ≈ 4.5 M doubles. */
#define SCRATCH_POOL_DOUBLES (5u * 1024u * 1024u) /* 5M doubles = 40 MB */
static double g_scratch_pool[SCRATCH_POOL_DOUBLES];
static size_t g_scratch_wm;

static double *scratch_alloc(size_t n_doubles)
{
 size_t aligned = (n_doubles + 7u) & ~7u; /* 64-byte cache-line */
 if (g_scratch_wm + aligned > SCRATCH_POOL_DOUBLES) {
 FLIGHT_LOG("FATAL: scratch pool exhausted (%zu + %zu > %zu)\n",
 g_scratch_wm, aligned, (size_t)SCRATCH_POOL_DOUBLES);
 FLIGHT_SAFE_HALT(1);
 }
 double *p = &g_scratch_pool[g_scratch_wm];
 g_scratch_wm += aligned;
 return p;
}

/** Save current arena watermark (call at function entry). */
static inline size_t scratch_save(void) { return g_scratch_wm; }

/** Restore arena watermark — frees all scratch allocated since save. */
static inline void scratch_restore(size_t wm) { g_scratch_wm = wm; }

// Data structures

// Matrix structure for linear algebra operations
typedef struct {
 int rows;
 int cols;
 double* data; // Row-major order (i*cols + j) for better cache locality
} Matrix;

// Vector structure
typedef struct {
 int size;
 double* data;
} Vector;

// Rover pose (position and orientation)
typedef struct {
 double x;
 double y;
 double theta;
 double timestamp;
} Pose;

// Landmark observation
typedef struct {
 double range;
 double bearing;
 int id; // Landmark ID
 double quality; // Observation quality/certainty
} Observation;

// Landmark in the map
typedef struct {
 double x;
 double y;
 int id;
 double certainty;
 int observations; // Number of times observed
} Landmark;

// Loop closure constraint (sparse representation)
typedef struct {
 int from_pose;
 int to_pose;
 double constraints[3]; // dx, dy, dtheta
} LoopClosure;

// SLAM system state
typedef struct {
 // Rover trajectory
 Pose poses[MAX_POSES];
 int pose_count;

 // Map of landmarks
 Landmark landmarks[MAX_LANDMARKS];
 int landmark_count;

 // Covariance matrix for the entire SLAM state
 Matrix covariance;

 // Current state estimate
 Vector state;

 // Odometry noise parameters
 double alpha1, alpha2, alpha3, alpha4;

 // Observation noise parameters
 double sigma_range;
 double sigma_bearing;

 // Loop closure parameters (sparse list — avoids O(n^2) memory)
 LoopClosure loop_closures[MAX_LOOP_CLOSURES];
 int loop_closure_count;
} SLAMSystem;

// Function prototypes
Matrix createMatrix(int rows, int cols);
void freeMatrix(Matrix* m);
void matrixMatrixBlockMultiply(Matrix* A, Matrix* B, Matrix* C);
void matrixTranspose(Matrix* A, Matrix* At);
void matrixInverse(Matrix* A, Matrix* Ainv);
void choleskySolve(Matrix* A, Vector* b, Vector* x);

Vector createVector(int size);
void freeVector(Vector* v);
double vectorNorm(Vector* v);
void matrixVectorMultiply(Matrix* A, Vector* x, Vector* b);

SLAMSystem* initializeSLAM();
void freeSLAM(SLAMSystem* slam);
void predictMotion(SLAMSystem* slam, double dx, double dy, double dtheta);
void processObservations(SLAMSystem* slam, Observation* observations, int count);
void detectLoopClosures(SLAMSystem* slam);
void optimizePoseGraph(SLAMSystem* slam);
void computeJacobians(SLAMSystem* slam, Matrix* H_x, Matrix* H_l, Vector* z_pred, Observation* obs);
void updateStateCovariance(SLAMSystem* slam, Matrix* K, Matrix* H, Matrix* R);
double normalizeAngle(double angle);
void saveSLAMState(SLAMSystem* slam, const char* filename);

// Implementation of matrix operations

/* ================================================================== */
/* HARDWARE 3×3 TILE HELPERS */
/* Single-tile operations used by the block-inverse and tiled */
/* multiply/add kernels below. */
/* ================================================================== */

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

/* ================================================================== */
/* GENERIC TILED MATRIX MULTIPLY — C(M×N) = A(M×K) · B(K×N) */
/* */
/* Uses 3×3 SYS_MMACD tiles for the aligned portion. */
/* SW remainder tails handle non-3-aligned dimensions. */
/* ================================================================== */
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

/* ================================================================== */
/* BLOCK INVERSE HIERARCHY */
/* */
/* inv_6x6: [3×3|3×3; 3×3|3×3] → 2 MINVD */
/* inv_9x9: [6×6|6×3; 3×6|3×3] → 3 MINVD */
/* */
/* For n not a multiple of 3 (e.g. n=2), fall back to SW. */
/* ================================================================== */

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
static void inv_9x9(const double *Mat, double *Minv)
{
 const int n = 9;
 double A_blk[36], B_blk[18], C_blk[18], D_blk[9];

 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 6; j++)
 A_blk[i*6+j] = Mat[i*n + j];
 for (int i = 0; i < 6; i++)
 for (int j = 0; j < 3; j++)
 B_blk[i*3+j] = Mat[i*n + (j+6)];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 6; j++)
 C_blk[i*6+j] = Mat[(i+6)*n + j];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 D_blk[i*3+j] = Mat[(i+6)*n + (j+6)];

 double A_inv[36]; inv_6x6(A_blk, A_inv);
 double CA_inv[18]; hw_matmul(C_blk, A_inv, CA_inv, 3, 6, 6);
 double CA_inv_B[9]; hw_matmul(CA_inv, B_blk, CA_inv_B, 3, 6, 3);
 double S_blk[9]; hw_sub3(D_blk, CA_inv_B, S_blk);
 double S_inv[9]; hw_inv3(S_blk, S_inv);
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

/* ================================================================== */

/* Allocate matrix from scratch arena (zero-initialised).
 * No malloc — Strict Flight profile (crewed spacecraft mission / DO-178C). */
Matrix createMatrix(int rows, int cols) {
 Matrix m;
 m.rows = rows;
 m.cols = cols;
 m.data = scratch_alloc((size_t)rows * cols);
 memset(m.data, 0, (size_t)rows * cols * sizeof(double));
 return m;
}

/* No-op: scratch arena reclaims memory via scratch_restore().
 * Persistent pools (g_state_pool, g_cov_pool) are never freed. */
void freeMatrix(Matrix* m) {
 if (m) { m->data = NULL; m->rows = 0; m->cols = 0; }
}

// Multiply two matrices: C = A * B

// Block matrix multiplication — HW 3×3 tiled via AME accelerator
void matrixMatrixBlockMultiply(Matrix* A, Matrix* B, Matrix* C) {
 if (A->cols != B->rows || C->rows != A->rows || C->cols != B->cols) {
 FLIGHT_LOG("Matrix dimension mismatch in block multiplication\n");
 return;
 }

 hw_matmul(A->data, B->data, C->data, A->rows, A->cols, B->cols);
}

// Compute the transpose of a matrix
void matrixTranspose(Matrix* A, Matrix* At) {
 if (At->rows != A->cols || At->cols != A->rows) {
 FLIGHT_LOG("Matrix dimension mismatch in transpose\n");
 return;
 }

 for (int i = 0; i < A->rows; i++) {
 for (int j = 0; j < A->cols; j++) {
 At->data[j * At->cols + i] = A->data[i * A->cols + j];
 }
 }
}

// Matrix inversion — HW dispatch for n=3,6,9; SW Gauss-Jordan fallback
void matrixInverse(Matrix* A, Matrix* Ainv) {
 if (A->rows != A->cols || Ainv->rows != A->rows || Ainv->cols != A->cols) {
 FLIGHT_LOG("Matrix dimension mismatch in inverse\n");
 return;
 }

 int n = A->rows;

 /* HW accelerated paths for multiples of 3 */
 if (n == 3) {
 hw_inv3(A->data, Ainv->data);
 return;
 }
 if (n == 6) {
 inv_6x6(A->data, Ainv->data);
 return;
 }
 if (n == 9) {
 inv_9x9(A->data, Ainv->data);
 return;
 }

 /* SW Gauss-Jordan fallback (n=2 for innovation covariance, or any other size) */
 size_t _mark = scratch_save();

 // Create augmented matrix [A|I]
 Matrix augmented = createMatrix(n, 2 * n);

 // Fill augmented matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 augmented.data[i * augmented.cols + j] = A->data[i * A->cols + j];
 }
 augmented.data[i * augmented.cols + n + i] = 1.0;
 }

 // Perform Gauss-Jordan elimination
 for (int i = 0; i < n; i++) {
 // Find pivot
 double max = fabs(augmented.data[i * augmented.cols + i]);
 int maxRow = i;
 for (int k = i + 1; k < n; k++) {
 if (fabs(augmented.data[k * augmented.cols + i]) > max) {
 max = fabs(augmented.data[k * augmented.cols + i]);
 maxRow = k;
 }
 }

 // Swap rows if necessary
 if (maxRow != i) {
 for (int j = 0; j < 2 * n; j++) {
 double temp = augmented.data[i * augmented.cols + j];
 augmented.data[i * augmented.cols + j] = augmented.data[maxRow * augmented.cols + j];
 augmented.data[maxRow * augmented.cols + j] = temp;
 }
 }

 // Scale row so pivot becomes 1
 double scale = augmented.data[i * augmented.cols + i];
 if (fabs(scale) < 1e-10) {
 // Handle near-singular matrix
 scale = 1e-10;
 }

 for (int j = 0; j < 2 * n; j++) {
 augmented.data[i * augmented.cols + j] /= scale;
 }

 // Eliminate other rows
 for (int k = 0; k < n; k++) {
 if (k != i) {
 double factor = augmented.data[k * augmented.cols + i];
 for (int j = 0; j < 2 * n; j++) {
 augmented.data[k * augmented.cols + j] -= factor * augmented.data[i * augmented.cols + j];
 }
 }
 }
 }

 // Extract inverse from augmented matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 Ainv->data[i * Ainv->cols + j] = augmented.data[i * augmented.cols + n + j];
 }
 }

 scratch_restore(_mark);
}

// Solve Ax = b using Cholesky decomposition (for positive definite matrices)
void choleskySolve(Matrix* A, Vector* b, Vector* x) {
 int n = A->rows;
 if (A->cols != n || b->size != n || x->size != n) {
 FLIGHT_LOG("Dimension mismatch in Cholesky solve\n");
 return;
 }
 size_t _mark = scratch_save();

 // Create Lower triangular matrix for Cholesky decomposition
 Matrix L = createMatrix(n, n);

 // Compute Cholesky decomposition A = L * L^T
 for (int i = 0; i < n; i++) {
 for (int j = 0; j <= i; j++) {
 double sum = 0;

 if (j == i) { // Diagonal elements
 for (int k = 0; k < j; k++) {
 sum += L.data[j * n + k] * L.data[j * n + k];
 }
 double val = A->data[j * n + j] - sum;
 if (val <= 0) {
 // Handle non-positive definite matrix
 val = 1e-10;
 }
 L.data[j * n + j] = sqrt(val);
 } else { // Non-diagonal elements
 for (int k = 0; k < j; k++) {
 sum += L.data[i * n + k] * L.data[j * n + k];
 }
 if (fabs(L.data[j * n + j]) < 1e-10) {
 // Avoid division by near-zero
 L.data[i * n + j] = 0;
 } else {
 L.data[i * n + j] = (A->data[i * n + j] - sum) / L.data[j * n + j];
 }
 }
 }
 }

 // Forward substitution to solve L * y = b
 Vector y = createVector(n);
 for (int i = 0; i < n; i++) {
 double sum = 0;
 for (int j = 0; j < i; j++) {
 sum += L.data[i * n + j] * y.data[j];
 }
 if (fabs(L.data[i * n + i]) < 1e-10) {
 // Avoid division by near-zero
 y.data[i] = 0;
 } else {
 y.data[i] = (b->data[i] - sum) / L.data[i * n + i];
 }
 }

 // Backward substitution to solve L^T * x = y
 for (int i = n - 1; i >= 0; i--) {
 double sum = 0;
 for (int j = i + 1; j < n; j++) {
 sum += L.data[j * n + i] * x->data[j];
 }
 if (fabs(L.data[i * n + i]) < 1e-10) {
 // Avoid division by near-zero
 x->data[i] = 0;
 } else {
 x->data[i] = (y.data[i] - sum) / L.data[i * n + i];
 }
 }

 scratch_restore(_mark);
}

// Print a matrix

// Vector operations

/* Allocate vector from scratch arena (zero-initialised).
 * No malloc — Strict Flight profile. */
Vector createVector(int size) {
 Vector v;
 v.size = size;
 v.data = scratch_alloc((size_t)size);
 memset(v.data, 0, (size_t)size * sizeof(double));
 return v;
}

/* No-op: scratch arena reclaims memory via scratch_restore(). */
void freeVector(Vector* v) {
 if (v) { v->data = NULL; v->size = 0; }
}

// Compute the Euclidean norm of a vector
double vectorNorm(Vector* v) {
 double sum = 0.0;
 for (int i = 0; i < v->size; i++) {
 sum += v->data[i] * v->data[i];
 }
 return sqrt(sum);
}

// Add two vectors: result = a + b

// Subtract two vectors: result = a - b

// Multiply a matrix and a vector: b = A * x
void matrixVectorMultiply(Matrix* A, Vector* x, Vector* b) {
 if (A->cols != x->size || A->rows != b->size) {
 FLIGHT_LOG("Dimension mismatch in matrix-vector multiplication\n");
 return;
 }

 for (int i = 0; i < A->rows; i++) {
 double sum = 0.0;
 for (int j = 0; j < A->cols; j++) {
 sum += A->data[i * A->cols + j] * x->data[j];
 }
 b->data[i] = sum;
 }
}

// Compute dot product of two vectors

// Print a vector

// SLAM algorithm implementation

/* Static global SLAMSystem in BSS (~60 KB with sparse loop closures). */
static SLAMSystem g_slam_instance;

// Initialize the SLAM system
SLAMSystem* initializeSLAM() {
 SLAMSystem* slam = &g_slam_instance;
 memset(slam, 0, sizeof(SLAMSystem));

 // Initialize rover trajectory with a single pose at origin
 slam->poses[0].x = 0.0;
 slam->poses[0].y = 0.0;
 slam->poses[0].theta = 0.0;
 slam->poses[0].timestamp = 0.0;
 slam->pose_count = 1;

 // Initialize empty map
 slam->landmark_count = 0;

 /* State vector and covariance use pre-allocated static pools
 * (link-time BSS). Active size starts at STATE_DIM and grows
 * in-place as landmarks are observed — zero heap allocation. */
 memset(g_state_pool, 0, sizeof(g_state_pool));
 slam->state.size = STATE_DIM;
 slam->state.data = g_state_pool;

 memset(g_cov_pool, 0, sizeof(g_cov_pool));
 slam->covariance.rows = STATE_DIM;
 slam->covariance.cols = STATE_DIM;
 slam->covariance.data = g_cov_pool;
 for (int i = 0; i < STATE_DIM; i++) {
 slam->covariance.data[i * STATE_DIM + i] = 0.01; // Small initial uncertainty
 }

 // Set motion model noise parameters
 slam->alpha1 = 0.1; // Rotation noise proportional to rotation
 slam->alpha2 = 0.01; // Rotation noise proportional to translation
 slam->alpha3 = 0.01; // Translation noise proportional to translation
 slam->alpha4 = 0.1; // Translation noise proportional to rotation

 // Set observation model noise parameters
 slam->sigma_range = 0.1; // Range measurement noise (meters)
 slam->sigma_bearing = 0.05; // Bearing measurement noise (radians)

 // Initialize loop closure data structures
 slam->loop_closure_count = 0;

 return slam;
}

// Free memory allocated for the SLAM system
void freeSLAM(SLAMSystem* slam) {
 if (slam) {
 /* State / covariance live in link-time static pools — no deallocation.
 * Scratch arena is implicitly freed by program termination. */
 slam->state.data = NULL; slam->state.size = 0;
 slam->covariance.data = NULL;
 slam->covariance.rows = 0; slam->covariance.cols = 0;
 }
}

// Normalize angle to [-pi, pi]
double normalizeAngle(double angle) {
 while (angle > M_PI) angle -= 2.0 * M_PI;
 while (angle < -M_PI) angle += 2.0 * M_PI;
 return angle;
}

// Predict motion based on odometry
void predictMotion(SLAMSystem* slam, double dx, double dy, double dtheta) {
 // Get current state
 double x = slam->state.data[0];
 double y = slam->state.data[1];
 double theta = slam->state.data[2];

 // Add noise to motion (odometry error model)
 double noise_dx = dx * (1.0 + slam->alpha1 * fabs(dtheta) + slam->alpha3 * fabs(dx));
 double noise_dy = dy * (1.0 + slam->alpha1 * fabs(dtheta) + slam->alpha3 * fabs(dy));
 double noise_dtheta = dtheta * (1.0 + slam->alpha2 * fabs(dx) + slam->alpha4 * fabs(dtheta));

 // Trigonometric calculations for motion update
 double cos_theta = cos(theta);
 double sin_theta = sin(theta);

 // Update state with motion prediction
 double new_x = x + noise_dx * cos_theta - noise_dy * sin_theta;
 double new_y = y + noise_dx * sin_theta + noise_dy * cos_theta;
 double new_theta = normalizeAngle(theta + noise_dtheta);

 // Update state vector
 slam->state.data[0] = new_x;
 slam->state.data[1] = new_y;
 slam->state.data[2] = new_theta;

 size_t _mark = scratch_save();

 // Compute Jacobian of motion model
 Matrix G = createMatrix(STATE_DIM, STATE_DIM);
 G.data[0 * STATE_DIM + 0] = 1.0;
 G.data[0 * STATE_DIM + 1] = 0.0;
 G.data[0 * STATE_DIM + 2] = -noise_dx * sin_theta - noise_dy * cos_theta;
 G.data[1 * STATE_DIM + 0] = 0.0;
 G.data[1 * STATE_DIM + 1] = 1.0;
 G.data[1 * STATE_DIM + 2] = noise_dx * cos_theta - noise_dy * sin_theta;
 G.data[2 * STATE_DIM + 0] = 0.0;
 G.data[2 * STATE_DIM + 1] = 0.0;
 G.data[2 * STATE_DIM + 2] = 1.0;

 // Compute motion noise covariance
 Matrix R = createMatrix(STATE_DIM, STATE_DIM);
 R.data[0 * STATE_DIM + 0] = slam->alpha3 * fabs(dx);
 R.data[1 * STATE_DIM + 1] = slam->alpha3 * fabs(dy);
 R.data[2 * STATE_DIM + 2] = slam->alpha4 * fabs(dtheta) + slam->alpha2 * sqrt(dx*dx + dy*dy);

 // Update covariance using proper EKF-SLAM prediction.
 // Full Jacobian F = [G, 0; 0, I] → P_new = F*P*F^T + Q
 // Block updates:
 // P_rr = G * P_rr * G^T + R (pose-pose 3×3 block)
 // P_rl = G * P_rl, P_lr = P_rl^T (cross-covariance)
 // P_ll unchanged (landmark-landmark block)
 int n = slam->state.size;

 // Extract 3×3 pose-pose covariance block
 Matrix P_pose = createMatrix(STATE_DIM, STATE_DIM);
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 P_pose.data[i * STATE_DIM + j] = slam->covariance.data[i * n + j];

 Matrix temp1 = createMatrix(STATE_DIM, STATE_DIM);
 Matrix temp2 = createMatrix(STATE_DIM, STATE_DIM);
 Matrix G_transpose = createMatrix(STATE_DIM, STATE_DIM);

 matrixTranspose(&G, &G_transpose);
 matrixMatrixBlockMultiply(&G, &P_pose, &temp1);
 matrixMatrixBlockMultiply(&temp1, &G_transpose, &temp2);

 // Write back pose-pose block with correct stride
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 slam->covariance.data[i * n + j] =
 temp2.data[i * STATE_DIM + j] + R.data[i * STATE_DIM + j];

 // Update cross-covariance: P_rl = G * P_rl, P_lr = P_rl^T
 for (int j = STATE_DIM; j < n; j++) {
 double col[STATE_DIM];
 for (int ci = 0; ci < STATE_DIM; ci++)
 col[ci] = slam->covariance.data[ci * n + j];
 for (int ci = 0; ci < STATE_DIM; ci++) {
 double sum = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 sum += G.data[ci * STATE_DIM + k] * col[k];
 slam->covariance.data[ci * n + j] = sum;
 slam->covariance.data[j * n + ci] = sum; // symmetric
 }
 }

 // Add new pose to trajectory
 if (slam->pose_count < MAX_POSES) {
 Pose new_pose;
 new_pose.x = new_x;
 new_pose.y = new_y;
 new_pose.theta = new_theta;
 new_pose.timestamp = slam->poses[slam->pose_count - 1].timestamp + 1.0; // Assume unit time step
 slam->poses[slam->pose_count++] = new_pose;
 }

 scratch_restore(_mark);
}

// Compute Jacobians for EKF update
void computeJacobians(SLAMSystem* slam, Matrix* H_x, Matrix* H_l, Vector* z_pred, Observation* obs) {
 // Current robot state
 double x = slam->state.data[0];
 double y = slam->state.data[1];
 double theta = slam->state.data[2];

 // Find landmark index
 int landmark_idx = -1;
 for (int i = 0; i < slam->landmark_count; i++) {
 if (slam->landmarks[i].id == obs->id) {
 landmark_idx = i;
 break;
 }
 }

 if (landmark_idx == -1) {
 FLIGHT_LOG("Error: Landmark not found in computeJacobians\n");
 return;
 }

 // Landmark position
 double lm_x = slam->state.data[STATE_DIM + landmark_idx * LANDMARK_DIM];
 double lm_y = slam->state.data[STATE_DIM + landmark_idx * LANDMARK_DIM + 1];

 // Delta between landmark and robot
 double dx = lm_x - x;
 double dy = lm_y - y;

 // Compute range and bearing (prediction)
 double q = dx*dx + dy*dy;
 if (q < 1e-10) q = 1e-10; // Avoid division by zero or very small numbers
 double sqrt_q = sqrt(q);

 z_pred->data[0] = sqrt_q; // Range
 z_pred->data[1] = normalizeAngle(atan2(dy, dx) - theta); // Bearing

 // Compute Jacobians

 // Jacobian of measurement w.r.t. robot pose [x, y, theta]
 // H_x = [d_range/d_rx, d_range/d_ry, d_range/d_rtheta;
 // d_bearing/d_rx, d_bearing/d_ry, d_bearing/d_rtheta]

 // Range derivatives
 double d_range_d_rx = -dx / sqrt_q;
 double d_range_d_ry = -dy / sqrt_q;
 double d_range_d_rtheta = 0;

 // Bearing derivatives
 double d_bearing_d_rx = dy / q;
 double d_bearing_d_ry = -dx / q;
 double d_bearing_d_rtheta = -1.0;

 // Fill in H_x
 H_x->data[0 * H_x->cols + 0] = d_range_d_rx;
 H_x->data[0 * H_x->cols + 1] = d_range_d_ry;
 H_x->data[0 * H_x->cols + 2] = d_range_d_rtheta;

 H_x->data[1 * H_x->cols + 0] = d_bearing_d_rx;
 H_x->data[1 * H_x->cols + 1] = d_bearing_d_ry;
 H_x->data[1 * H_x->cols + 2] = d_bearing_d_rtheta;

 // Jacobian of measurement w.r.t. landmark [lm_x, lm_y]
 // H_l = [d_range/d_lx, d_range/d_ly;
 // d_bearing/d_lx, d_bearing/d_ly]

 // Landmark position derivatives for range
 double d_range_d_lx = dx / sqrt_q;
 double d_range_d_ly = dy / sqrt_q;

 // Landmark position derivatives for bearing
 double d_bearing_d_lx = -dy / q;
 double d_bearing_d_ly = dx / q;

 // Fill in H_l matrix
 int lm_offset = STATE_DIM + landmark_idx * LANDMARK_DIM;

 // These are only non-zero for the current landmark columns
 for (int i = 0; i < H_l->rows; i++) {
 for (int j = 0; j < H_l->cols; j++) {
 H_l->data[i * H_l->cols + j] = 0.0;
 }
 }

 H_l->data[0 * H_l->cols + lm_offset] = d_range_d_lx;
 H_l->data[0 * H_l->cols + lm_offset + 1] = d_range_d_ly;

 H_l->data[1 * H_l->cols + lm_offset] = d_bearing_d_lx;
 H_l->data[1 * H_l->cols + lm_offset + 1] = d_bearing_d_ly;
}

// Update state covariance
void updateStateCovariance(SLAMSystem* slam, Matrix* K, Matrix* H, Matrix* R) {
 // P = (I - K*H)*P*(I - K*H)^T + K*R*K^T

 int n = slam->state.size;
 size_t _mark = scratch_save();

 // Compute I - K*H
 Matrix KH = createMatrix(n, n);
 Matrix I_KH = createMatrix(n, n);

 matrixMatrixBlockMultiply(K, H, &KH);

 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 if (i == j) {
 I_KH.data[i * n + j] = 1.0 - KH.data[i * n + j];
 } else {
 I_KH.data[i * n + j] = -KH.data[i * n + j];
 }
 }
 }

 // Compute (I - K*H)*P
 Matrix temp1 = createMatrix(n, n);
 matrixMatrixBlockMultiply(&I_KH, &slam->covariance, &temp1);

 // Compute (I - K*H)^T
 Matrix I_KH_T = createMatrix(n, n);
 matrixTranspose(&I_KH, &I_KH_T);

 // Compute (I - K*H)*P*(I - K*H)^T
 Matrix temp2 = createMatrix(n, n);
 matrixMatrixBlockMultiply(&temp1, &I_KH_T, &temp2);

 // Compute K*R
 Matrix KR = createMatrix(n, MEAS_DIM);
 matrixMatrixBlockMultiply(K, R, &KR);

 // Compute K*R*K^T
 Matrix K_T = createMatrix(MEAS_DIM, n);
 matrixTranspose(K, &K_T);

 Matrix KRK_T = createMatrix(n, n);
 matrixMatrixBlockMultiply(&KR, &K_T, &KRK_T);

 // Update covariance: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 slam->covariance.data[i * n + j] = temp2.data[i * n + j] + KRK_T.data[i * n + j];
 }
 }

 scratch_restore(_mark);
}

// Process landmark observations and update state
void processObservations(SLAMSystem* slam, Observation* observations, int count) {
 // Current robot state
 double x = slam->state.data[0];
 double y = slam->state.data[1];
 double theta = slam->state.data[2];

 // For each observation
 for (int i = 0; i < count; i++) {
 Observation* obs = &observations[i];

 // Check if this is a new landmark
 bool is_new = true;
 int landmark_idx = -1;

 for (int j = 0; j < slam->landmark_count; j++) {
 if (slam->landmarks[j].id == obs->id) {
 is_new = false;
 landmark_idx = j;
 break;
 }
 }

 if (is_new) {
 // Add new landmark to the map
 if (slam->landmark_count < MAX_LANDMARKS) {
 // Convert range and bearing to global coordinates
 double lm_x = x + obs->range * cos(theta + obs->bearing);
 double lm_y = y + obs->range * sin(theta + obs->bearing);

 slam->landmarks[slam->landmark_count].x = lm_x;
 slam->landmarks[slam->landmark_count].y = lm_y;
 slam->landmarks[slam->landmark_count].id = obs->id;
 slam->landmarks[slam->landmark_count].certainty = obs->quality;
 slam->landmarks[slam->landmark_count].observations = 1;

 landmark_idx = slam->landmark_count;
 slam->landmark_count++;

 /* Expand state and covariance in pre-allocated pools.
 * No heap allocation — Strict Flight profile. */
 int old_size = slam->state.size;
 int new_size = old_size + LANDMARK_DIM;

 /* State: extend active region in g_state_pool */
 slam->state.data[old_size] = lm_x;
 slam->state.data[old_size + 1] = lm_y;
 slam->state.size = new_size;

 /* Covariance: re-layout rows from stride old_size →
 * new_size. Bottom-up memmove is safe because
 * new_size > old_size ⇒ dest offset > source offset. */
 for (int r = old_size - 1; r >= 0; r--) {
 memmove(&slam->covariance.data[(size_t)r * new_size],
 &slam->covariance.data[(size_t)r * old_size],
 (size_t)old_size * sizeof(double));
 memset(&slam->covariance.data[(size_t)r * new_size + old_size],
 0, (size_t)LANDMARK_DIM * sizeof(double));
 }
 for (int r = old_size; r < new_size; r++)
 memset(&slam->covariance.data[(size_t)r * new_size], 0,
 (size_t)new_size * sizeof(double));
 for (int r = old_size; r < new_size; r++)
 slam->covariance.data[(size_t)r * new_size + r] = 100.0;

 slam->covariance.rows = new_size;
 slam->covariance.cols = new_size;
 }
 } else {
 // Update landmark observation count
 slam->landmarks[landmark_idx].observations++;
 size_t _obs_mark = scratch_save();

 // Update existing landmark using EKF

 // Compute predicted observation
 double lm_x = slam->state.data[STATE_DIM + landmark_idx * LANDMARK_DIM];
 double lm_y = slam->state.data[STATE_DIM + landmark_idx * LANDMARK_DIM + 1];

 // Delta between landmark and robot in robot's frame
 double dx = lm_x - x;
 double dy = lm_y - y;

 // Predicted range and bearing (using trig functions)
 double pred_range = sqrt(dx*dx + dy*dy);
 double pred_bearing = normalizeAngle(atan2(dy, dx) - theta);

 // Innovation (difference between observation and prediction)
 double inn_range = obs->range - pred_range;
 double inn_bearing = normalizeAngle(obs->bearing - pred_bearing);

 // Compute Jacobians for EKF update
 Matrix H_x = createMatrix(MEAS_DIM, slam->state.size);
 Matrix H_l = createMatrix(MEAS_DIM, slam->state.size);
 Vector z_pred = createVector(MEAS_DIM);

 computeJacobians(slam, &H_x, &H_l, &z_pred, obs);

 // Observation noise covariance
 Matrix R = createMatrix(MEAS_DIM, MEAS_DIM);
 R.data[0 * MEAS_DIM + 0] = slam->sigma_range * slam->sigma_range;
 R.data[1 * MEAS_DIM + 1] = slam->sigma_bearing * slam->sigma_bearing;

 // Innovation covariance: S = H*P*H^T + R
 Matrix H = createMatrix(MEAS_DIM, slam->state.size);
 for (int r = 0; r < MEAS_DIM; r++) {
 for (int c = 0; c < slam->state.size; c++) {
 if (c < STATE_DIM) {
 H.data[r * slam->state.size + c] = H_x.data[r * slam->state.size + c];
 } else if (c >= STATE_DIM + landmark_idx * LANDMARK_DIM &&
 c < STATE_DIM + (landmark_idx + 1) * LANDMARK_DIM) {
 H.data[r * slam->state.size + c] = H_l.data[r * slam->state.size + c];
 } else {
 H.data[r * slam->state.size + c] = 0.0;
 }
 }
 }

 // Compute innovation covariance S = H*P*H^T + R
 Matrix H_transpose = createMatrix(slam->state.size, MEAS_DIM);
 matrixTranspose(&H, &H_transpose);

 Matrix temp1 = createMatrix(MEAS_DIM, slam->state.size);
 Matrix S = createMatrix(MEAS_DIM, MEAS_DIM);

 matrixMatrixBlockMultiply(&H, &slam->covariance, &temp1);
 matrixMatrixBlockMultiply(&temp1, &H_transpose, &S);

 // Add observation noise
 for (int r = 0; r < MEAS_DIM; r++) {
 for (int c = 0; c < MEAS_DIM; c++) {
 S.data[r * MEAS_DIM + c] += R.data[r * MEAS_DIM + c];
 }
 }

 // Compute Kalman gain: K = P*H^T*S^-1
 Matrix S_inv = createMatrix(MEAS_DIM, MEAS_DIM);
 matrixInverse(&S, &S_inv);

 Matrix PH_transpose = createMatrix(slam->state.size, MEAS_DIM);
 Matrix K = createMatrix(slam->state.size, MEAS_DIM);

 matrixMatrixBlockMultiply(&slam->covariance, &H_transpose, &PH_transpose);
 matrixMatrixBlockMultiply(&PH_transpose, &S_inv, &K);

 // Update state: x = x + K * (z - h(x))
 Vector innovation = createVector(MEAS_DIM);
 innovation.data[0] = inn_range;
 innovation.data[1] = inn_bearing;

 Vector state_update = createVector(slam->state.size);
 matrixVectorMultiply(&K, &innovation, &state_update);

 for (int i = 0; i < slam->state.size; i++) {
 slam->state.data[i] += state_update.data[i];
 }

 // Normalize orientation
 slam->state.data[2] = normalizeAngle(slam->state.data[2]);

 // Update covariance: P = (I - K*H)*P
 updateStateCovariance(slam, &K, &H, &R);

 scratch_restore(_obs_mark);
 }
 }
}

// Update landmarks using observations

// Image processing for visual feature extraction (Gaussian + Sobel edge detection)
void processImageFeatures(double terrain[IMG_SIZE][IMG_SIZE],
 double edges[IMG_SIZE][IMG_SIZE],
 int *feature_count) {
 // Step 1: Gaussian blur for noise reduction (BEFORE edge detection)
 static const double gauss[3][3] = {{1,2,1},{2,4,2},{1,2,1}};
 double smoothed[IMG_SIZE][IMG_SIZE];
 memset(smoothed, 0, sizeof(smoothed));
 for (int y = 1; y < IMG_SIZE - 1; y++) {
 for (int x = 1; x < IMG_SIZE - 1; x++) {
 double sum = 0.0;
 for (int ky = -1; ky <= 1; ky++)
 for (int kx = -1; kx <= 1; kx++)
 sum += terrain[y+ky][x+kx] * gauss[ky+1][kx+1];
 smoothed[y][x] = sum / 16.0;
 }
 }

 // Step 2: Sobel edge detection on smoothed image
 static const double Gx[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};
 static const double Gy[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};
 *feature_count = 0;
 memset(edges, 0, IMG_SIZE * IMG_SIZE * sizeof(double));
 for (int y = 1; y < IMG_SIZE - 1; y++) {
 for (int x = 1; x < IMG_SIZE - 1; x++) {
 double sx = 0.0, sy = 0.0;
 for (int ky = -1; ky <= 1; ky++) {
 for (int kx = -1; kx <= 1; kx++) {
 sx += smoothed[y+ky][x+kx] * Gx[ky+1][kx+1];
 sy += smoothed[y+ky][x+kx] * Gy[ky+1][kx+1];
 }
 }
 edges[y][x] = sqrt(sx*sx + sy*sy);
 if (edges[y][x] > 0.5) (*feature_count)++;
 }
 }
}

// Detect loop closures
void detectLoopClosures(SLAMSystem* slam) {
 // We look for poses that are spatially close but temporally distant

 // Only detect loops if we have enough poses
 if (slam->pose_count < LOOP_DETECT_WINDOW * 2) {
 return;
 }

 // Loop through all poses (skip the most recent ones)
 for (int i = 0; i < slam->pose_count - LOOP_DETECT_WINDOW; i++) {
 Pose* pose_i = &slam->poses[i];

 // Compare with recent poses that are temporally distant
 for (int j = slam->pose_count - LOOP_DETECT_WINDOW; j < slam->pose_count; j++) {
 // Skip if already connected (search sparse list)
 bool already_detected = false;
 for (int k = 0; k < slam->loop_closure_count; k++) {
 if ((slam->loop_closures[k].from_pose == i && slam->loop_closures[k].to_pose == j) ||
 (slam->loop_closures[k].from_pose == j && slam->loop_closures[k].to_pose == i)) {
 already_detected = true;
 break;
 }
 }
 if (already_detected) {
 continue;
 }

 Pose* pose_j = &slam->poses[j];

 // Compute Euclidean distance between poses
 double dx = pose_i->x - pose_j->x;
 double dy = pose_i->y - pose_j->y;
 double dist = sqrt(dx*dx + dy*dy);

 // Compute orientation difference
 double dtheta = normalizeAngle(pose_i->theta - pose_j->theta);

 // Threshold for loop closure detection
 // (Close in space and similar orientation)
 if (dist < 5.0 && fabs(dtheta) < 0.5) {
 // Found a loop closure — store in sparse list
 if (slam->loop_closure_count < MAX_LOOP_CLOSURES) {
 int idx = slam->loop_closure_count++;
 slam->loop_closures[idx].from_pose = i;
 slam->loop_closures[idx].to_pose = j;
 slam->loop_closures[idx].constraints[0] = pose_j->x - pose_i->x;
 slam->loop_closures[idx].constraints[1] = pose_j->y - pose_i->y;
 slam->loop_closures[idx].constraints[2] = normalizeAngle(pose_j->theta - pose_i->theta);
 }
 }
 }
 }
}

// Optimize pose graph
void optimizePoseGraph(SLAMSystem* slam) {
 // Skip optimization if no loop closures
 if (slam->loop_closure_count == 0) return;

 // Pose graph optimization using Gauss-Newton algorithm

 // Number of parameters: 3 per pose (x, y, theta)
 int n_params = slam->pose_count * 3;
 size_t _mark = scratch_save();

 // Create error vector and Jacobian matrix
 Vector error = createVector(n_params);
 Matrix J = createMatrix(n_params, n_params);
 Vector delta = createVector(n_params);

 // Initialize parameters with current poses
 Vector params = createVector(n_params);
 for (int i = 0; i < slam->pose_count; i++) {
 params.data[i*3 + 0] = slam->poses[i].x;
 params.data[i*3 + 1] = slam->poses[i].y;
 params.data[i*3 + 2] = slam->poses[i].theta;
 }

 // Optimization iterations
 int max_iterations = 10;
 double lambda = 0.1; // Damping factor for Levenberg-Marquardt
 double prev_err_norm = 0.0;

 for (int iter = 0; iter < max_iterations; iter++) {
 // Reset error and Jacobian
 for (int i = 0; i < n_params; i++) {
 error.data[i] = 0.0;
 for (int j = 0; j < n_params; j++) {
 J.data[i * n_params + j] = 0.0;
 }
 }

 // Odometry constraints (sequential poses)
 for (int i = 0; i < slam->pose_count - 1; i++) {
 // Original odometry measurement (difference between poses)
 double odom_dx = slam->poses[i+1].x - slam->poses[i].x;
 double odom_dy = slam->poses[i+1].y - slam->poses[i].y;
 double odom_dtheta = normalizeAngle(slam->poses[i+1].theta - slam->poses[i].theta);

 // Current estimate
 double p1_x = params.data[i*3 + 0];
 double p1_y = params.data[i*3 + 1];
 double p1_theta = params.data[i*3 + 2];

 double p2_x = params.data[(i+1)*3 + 0];
 double p2_y = params.data[(i+1)*3 + 1];
 double p2_theta = params.data[(i+1)*3 + 2];

 // Compute current delta in global frame
 double dx = p2_x - p1_x;
 double dy = p2_y - p1_y;

 // Error in global frame (no rotation — keeps Jacobian simple and correct)
 double err_x = dx - odom_dx;
 double err_y = dy - odom_dy;
 double err_theta = normalizeAngle((p2_theta - p1_theta) - odom_dtheta);

 // Weight for odometry constraints
 double w_odom = 1.0;

 // Add weighted error
 error.data[i*3 + 0] += w_odom * err_x;
 error.data[i*3 + 1] += w_odom * err_y;
 error.data[i*3 + 2] += w_odom * err_theta;

 error.data[(i+1)*3 + 0] -= w_odom * err_x;
 error.data[(i+1)*3 + 1] -= w_odom * err_y;
 error.data[(i+1)*3 + 2] -= w_odom * err_theta;

 // Jacobian for odometry constraints
 // This is a sparse matrix with non-zero blocks only for the connected poses

 // For pose i
 J.data[(i*3 + 0) * n_params + (i*3 + 0)] += w_odom;
 J.data[(i*3 + 1) * n_params + (i*3 + 1)] += w_odom;
 J.data[(i*3 + 2) * n_params + (i*3 + 2)] += w_odom;

 // For pose i+1
 J.data[((i+1)*3 + 0) * n_params + ((i+1)*3 + 0)] += w_odom;
 J.data[((i+1)*3 + 1) * n_params + ((i+1)*3 + 1)] += w_odom;
 J.data[((i+1)*3 + 2) * n_params + ((i+1)*3 + 2)] += w_odom;

 // Cross terms
 J.data[(i*3 + 0) * n_params + ((i+1)*3 + 0)] -= w_odom;
 J.data[(i*3 + 1) * n_params + ((i+1)*3 + 1)] -= w_odom;
 J.data[(i*3 + 2) * n_params + ((i+1)*3 + 2)] -= w_odom;

 J.data[((i+1)*3 + 0) * n_params + (i*3 + 0)] -= w_odom;
 J.data[((i+1)*3 + 1) * n_params + (i*3 + 1)] -= w_odom;
 J.data[((i+1)*3 + 2) * n_params + (i*3 + 2)] -= w_odom;
 }

 // Loop closure constraints (iterate sparse list)
 for (int lc = 0; lc < slam->loop_closure_count; lc++) {
 int ci = slam->loop_closures[lc].from_pose;
 int cj = slam->loop_closures[lc].to_pose;

 // Loop closure measurement
 double loop_dx = slam->loop_closures[lc].constraints[0];
 double loop_dy = slam->loop_closures[lc].constraints[1];
 double loop_dtheta = slam->loop_closures[lc].constraints[2];

 // Current estimate
 double p1_x = params.data[ci*3 + 0];
 double p1_y = params.data[ci*3 + 1];
 double p1_theta = params.data[ci*3 + 2];

 double p2_x = params.data[cj*3 + 0];
 double p2_y = params.data[cj*3 + 1];
 double p2_theta = params.data[cj*3 + 2];

 // Compute current delta in global frame
 double dx = p2_x - p1_x;
 double dy = p2_y - p1_y;
 double dtheta = normalizeAngle(p2_theta - p1_theta);

 // Error in global frame (consistent with odometry constraints)
 double err_x = dx - loop_dx;
 double err_y = dy - loop_dy;
 double err_theta = normalizeAngle(dtheta - loop_dtheta);

 // Weight for loop closure constraints (higher than odometry)
 double w_loop = 10.0;

 // Add weighted error
 error.data[ci*3 + 0] += w_loop * err_x;
 error.data[ci*3 + 1] += w_loop * err_y;
 error.data[ci*3 + 2] += w_loop * err_theta;

 error.data[cj*3 + 0] -= w_loop * err_x;
 error.data[cj*3 + 1] -= w_loop * err_y;
 error.data[cj*3 + 2] -= w_loop * err_theta;

 // Jacobian for loop closure constraints
 J.data[(ci*3 + 0) * n_params + (ci*3 + 0)] += w_loop;
 J.data[(ci*3 + 1) * n_params + (ci*3 + 1)] += w_loop;
 J.data[(ci*3 + 2) * n_params + (ci*3 + 2)] += w_loop;

 J.data[(cj*3 + 0) * n_params + (cj*3 + 0)] += w_loop;
 J.data[(cj*3 + 1) * n_params + (cj*3 + 1)] += w_loop;
 J.data[(cj*3 + 2) * n_params + (cj*3 + 2)] += w_loop;

 J.data[(ci*3 + 0) * n_params + (cj*3 + 0)] -= w_loop;
 J.data[(ci*3 + 1) * n_params + (cj*3 + 1)] -= w_loop;
 J.data[(ci*3 + 2) * n_params + (cj*3 + 2)] -= w_loop;

 J.data[(cj*3 + 0) * n_params + (ci*3 + 0)] -= w_loop;
 J.data[(cj*3 + 1) * n_params + (ci*3 + 1)] -= w_loop;
 J.data[(cj*3 + 2) * n_params + (ci*3 + 2)] -= w_loop;
 }

 // Add damping to diagonal (Levenberg-Marquardt)
 for (int i = 0; i < n_params; i++) {
 J.data[i * n_params + i] += lambda;
 }

 // Fix the first pose to avoid gauge freedom
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < n_params; j++) {
 J.data[i * n_params + j] = 0.0;
 }
 J.data[i * n_params + i] = 1.0;
 error.data[i] = 0.0;
 }

 // Solve directly: J is already the information matrix H (symmetric, SPD).
 // Using H directly avoids computing H^T*H (which squares the condition number).
 choleskySolve(&J, &error, &delta);

 // Update parameters
 for (int i = 0; i < n_params; i++) {
 params.data[i] -= delta.data[i];
 }

 // Normalize angle parameters
 for (int i = 0; i < slam->pose_count; i++) {
 params.data[i*3 + 2] = normalizeAngle(params.data[i*3 + 2]);
 }

 // Compute error norm
 double err_norm = vectorNorm(&error);
 double delta_norm = vectorNorm(&delta);

 // Early stopping if converged
 if (delta_norm < 1e-6) {
 break;
 }

 // Update lambda based on error progress (decrease if improving, increase if not)
 if (iter > 0) {
 if (err_norm < prev_err_norm) {
 lambda *= 0.5; // Decrease damping
 } else {
 lambda *= 2.0; // Increase damping
 }
 }
 prev_err_norm = err_norm;

 // Clean up (no temporary matrices needed — solve was direct)
 }

 // Update SLAM poses with optimized values
 for (int i = 0; i < slam->pose_count; i++) {
 slam->poses[i].x = params.data[i*3 + 0];
 slam->poses[i].y = params.data[i*3 + 1];
 slam->poses[i].theta = params.data[i*3 + 2];
 }

 // Update current state (latest pose)
 if (slam->pose_count > 0) {
 slam->state.data[0] = slam->poses[slam->pose_count-1].x;
 slam->state.data[1] = slam->poses[slam->pose_count-1].y;
 slam->state.data[2] = slam->poses[slam->pose_count-1].theta;
 }

 scratch_restore(_mark);
}

// Save SLAM state to a file
void saveSLAMState(SLAMSystem* slam, const char* filename) {
 FILE* file = FLIGHT_FOPEN(filename, "w");
 if (!file) {
 FLIGHT_LOG("Error opening file for writing: %s\n", filename);
 return;
 }

 // Save basic SLAM parameters
 FLIGHT_LOG("SLAM System State\n\n");

 // Save current pose
 FLIGHT_LOG("Current Pose:\n");
 FLIGHT_LOG("x: %f\n", slam->state.data[0]);
 FLIGHT_LOG("y: %f\n", slam->state.data[1]);
 FLIGHT_LOG("theta: %f\n\n", slam->state.data[2]);

 // Save trajectory
 FLIGHT_LOG("Trajectory (%d poses):\n", slam->pose_count);
 for (int i = 0; i < slam->pose_count; i++) {
 FLIGHT_LOG("Pose %d: x=%f, y=%f, theta=%f, t=%f\n",
 i, slam->poses[i].x, slam->poses[i].y,
 slam->poses[i].theta, slam->poses[i].timestamp);
 }
 FLIGHT_LOG("\n");

 // Save landmarks
 FLIGHT_LOG("Landmarks (%d):\n", slam->landmark_count);
 for (int i = 0; i < slam->landmark_count; i++) {
 FLIGHT_LOG("Landmark %d: x=%f, y=%f, id=%d, certainty=%f, obs=%d\n",
 i, slam->landmarks[i].x, slam->landmarks[i].y,
 slam->landmarks[i].id, slam->landmarks[i].certainty,
 slam->landmarks[i].observations);
 }

 FLIGHT_FCLOSE(file);
}

/**
 * Main function to demonstrate SLAM algorithm execution and evaluation
 */
int main(int argc, char* argv[]) {
 (void)argc; (void)argv;
 FLIGHT_LOG("[SLAM] init\n");

 // Initialize SLAM system
 SLAMSystem* slam = initializeSLAM();

 // Parameters for simulation
 int num_steps = 500;
 int num_landmarks = 50;
 double map_size = 100.0;
 int landmark_observations_per_step = 5;
 double sensor_range = 20.0;

 // Generate ground truth landmark positions (separate from SLAM map).
 // These represent the real-world positions; landmarks are only added
 // to the SLAM state when they are first observed via processObservations.
 static Landmark true_landmarks[MAX_LANDMARKS];
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(2311);

 for (int i = 0; i < num_landmarks; i++) {
 true_landmarks[i].id = i;
 true_landmarks[i].x = ((double)rand() / RAND_MAX) * map_size - map_size/2;
 true_landmarks[i].y = ((double)rand() / RAND_MAX) * map_size - map_size/2;
 true_landmarks[i].certainty = 0.0;
 true_landmarks[i].observations = 0;
 }

 // Timing variables
 clock_t start_time, end_time;
 double total_motion_time = 0.0;
 double total_observation_time = 0.0;
 double total_loop_closure_time = 0.0;
 double total_optimization_time = 0.0;

 FLIGHT_LOG("[SLAM] steps=%d landmarks=%d\n", num_steps, num_landmarks);

 // Main SLAM loop
 for (int step = 0; step < num_steps; step++) {
 // Get current pose for reference
 double x = slam->state.data[0];
 double y = slam->state.data[1];
 double theta = slam->state.data[2];

 // Simulate rover motion (circular path with noise)
 double true_dx = 0.5 * cos(step * 0.05);
 double true_dy = 0.5 * sin(step * 0.05);
 double true_dtheta = 0.05;

 // Add noise to motion
 double noisy_dx = true_dx + ((double)rand() / RAND_MAX - 0.5) * 0.1;
 double noisy_dy = true_dy + ((double)rand() / RAND_MAX - 0.5) * 0.1;
 double noisy_dtheta = true_dtheta + ((double)rand() / RAND_MAX - 0.5) * 0.02;

 // Predict motion (heavy on trig operations)
 start_time = clock();
 predictMotion(slam, noisy_dx, noisy_dy, noisy_dtheta);
 end_time = clock();
 total_motion_time += ((double)(end_time - start_time)) / CLOCKS_PER_SEC;

 // Simulate landmark observations from ground truth positions
 static Observation observations[MAX_LANDMARKS];
 int observation_count = 0;

 for (int i = 0; i < num_landmarks && observation_count < landmark_observations_per_step; i++) {
 // Compute true distance to landmark (from ground truth)
 double dx = true_landmarks[i].x - x;
 double dy = true_landmarks[i].y - y;
 double true_range = sqrt(dx*dx + dy*dy);

 // Only observe landmarks within sensor range
 if (true_range < sensor_range) {
 double true_bearing = normalizeAngle(atan2(dy, dx) - theta);

 // Add noise to measurements
 double noisy_range = true_range + ((double)rand() / RAND_MAX - 0.5) * slam->sigma_range * 2.0;
 double noisy_bearing = true_bearing + ((double)rand() / RAND_MAX - 0.5) * slam->sigma_bearing * 2.0;

 // Create observation
 observations[observation_count].id = true_landmarks[i].id;
 observations[observation_count].range = noisy_range;
 observations[observation_count].bearing = noisy_bearing;
 observations[observation_count].quality = 1.0 / (1.0 + true_range); // Quality decreases with distance

 observation_count++;
 }
 }

 // Process observations (heavy on matrix operations)
 start_time = clock();
 processObservations(slam, observations, observation_count);
 end_time = clock();
 total_observation_time += ((double)(end_time - start_time)) / CLOCKS_PER_SEC;

 // Visual feature extraction via image processing (Gaussian + Sobel)
 int feat_count = 0;
 {
 static double terrain_img[IMG_SIZE][IMG_SIZE];
 static double edge_img[IMG_SIZE][IMG_SIZE];
 // Generate terrain image around rover position
 for (int iy = 0; iy < IMG_SIZE; iy++)
 for (int ix = 0; ix < IMG_SIZE; ix++)
 terrain_img[iy][ix] = sin(0.3*(x+ix)) * cos(0.3*(y+iy)) + 0.1*((rand()%100)/100.0);
 processImageFeatures(terrain_img, edge_img, &feat_count);
 }

 // Detect loop closures every 20 steps
 if (step % 20 == 0 && step > 0) {
 start_time = clock();
 detectLoopClosures(slam);
 end_time = clock();
 total_loop_closure_time += ((double)(end_time - start_time)) / CLOCKS_PER_SEC;

 // Perform pose graph optimization
 start_time = clock();
 optimizePoseGraph(slam);
 end_time = clock();
 total_optimization_time += ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
 }

 // Print progress
 if (step % 50 == 0 || step == num_steps - 1) {
 FLIGHT_LOG("Step %d/%d - Position: (%.2f, %.2f, %.2f) - Observed %d landmarks, %d edge features\n",
 step+1, num_steps, slam->state.data[0], slam->state.data[1],
 slam->state.data[2], observation_count, feat_count);
 }
 }

 // Print performance statistics
 FLIGHT_LOG("\n[SLAM] perf\n");
 FLIGHT_LOG("Total motion prediction time: %.4f seconds\n", total_motion_time);
 FLIGHT_LOG("Total observation processing time: %.4f seconds\n", total_observation_time);
 FLIGHT_LOG("Total loop closure detection time: %.4f seconds\n", total_loop_closure_time);
 FLIGHT_LOG("Total pose graph optimization time: %.4f seconds\n", total_optimization_time);
 FLIGHT_LOG("Total execution time: %.4f seconds\n",
 total_motion_time + total_observation_time + total_loop_closure_time + total_optimization_time);

 // Save final state
 saveSLAMState(slam, "slam_result.txt");

 // Free memory
 freeSLAM(slam);

 return 0;
}
