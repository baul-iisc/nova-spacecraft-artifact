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
/*****************************************************************************
 * Deep-Space Autonomous Navigation — Orbit Determination Filter
 * HW TRIG VARIANT: All trigonometric ops use CORDIC accelerator
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix-Extension Accelerator
 * Application: deep-space navigation reference workload
 *
 * Implements a 15-state Extended Kalman Filter for autonomous deep-space
 * navigation. The state vector comprises:
 * [0..2] position (ECI, km)
 * [3..5] velocity (ECI, km/s)
 * [6..9] attitude quaternion (scalar-first)
 * [10..12] body angular rate (rad/s)
 * [13] accelerometer scale-factor error
 * [14] gyroscope drift (rad/s)
 *
 * Gravity is modelled with zonal/tesseral spherical harmonics up to
 * degree/order 8 (JGM-3), evaluated via the standard fully-normalised
 * associated Legendre recursion (Montenbruck & Gill, 2000, Sec. 3.2).
 *
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
/* ── Tuning knobs ────────────────────────────────────────────────── */

#define STATE_DIM 15 /* see header for state breakdown */
#define MEAS_DIM 6 /* 3 range + 3 Doppler channels */
#define NUM_BODIES 5 /* Earth, Moon, Sun, Mars, Jupiter */
#define GRAV_ORDER 8 /* JGM-3 truncation for on-board */
#define SIM_TIMESTEPS 1000
#define DT_SEC 10.0 /* propagation time-step (s) */

/* Earth WGS-84 / JGM-3 constants */
#define RE_KM 6378.137 /* equatorial radius */
#define MU_EARTH 398600.4418 /* km^3 s^-2 */
#define J2 1.08263e-3
#define J3 2.53266e-6
#define J4 1.61990e-6

/* ── Type declarations ───────────────────────────────────────────── */
typedef struct { double x, y, z; } Vec3;

typedef struct {
 Vec3 pos;
 Vec3 vel;
 double mu; /* gravitational parameter, km^3/s^2 */
} Body;

typedef struct {
 double x[STATE_DIM]; /* state estimate */
 double P[STATE_DIM][STATE_DIM]; /* covariance */
 double Q[STATE_DIM][STATE_DIM]; /* process noise */
 Body body[NUM_BODIES]; /* ephemeris (simplified)*/
 double Cnm[GRAV_ORDER+1][GRAV_ORDER+1]; /* gravity coeffs */
 double Snm[GRAV_ORDER+1][GRAV_ORDER+1]; /* gravity coeffs (sine) */
 double epoch; /* seconds from t0 */
 double total_dv; /* accumulated delta-V */
 int maneuver_count;
 int divergence_count; /* filter recovery events */
 double chi2_innov; /* normalised innovation chi2 */
} NavFilter;

/* A-priori covariance diagonal — shared between nav_init() and
 * divergence-recovery in nav_health(). Values from OD solution
 * (radial < along-track < cross-track, Doppler-limited velocity). */
static const double P0_DIAG[STATE_DIM] = {
 0.30, 1.50, 0.60, /* position km² */
 3.2e-3, 9.1e-3, 5.7e-3, /* velocity (km/s)² */
 1.3e-3, 1.3e-3, 1.3e-3, 1.3e-3, /* quaternion */
 8.4e-5, 9.7e-5, 7.6e-5, /* angular rate */
 1.5e-5, /* accel bias */
 8.7e-7 /* gyro drift */
};

/* ── Forward declarations ────────────────────────────────────────── */

/* Fixed-dimension matrix kernels — enables aggressive loop unrolling
 * and direct mapping to the 3×3-tile AME (matrix accelerator). The
 * compiler can fully unroll the inner loops when dimensions are known
 * at compile time, whereas generic mat_mul(m,n,p) inhibits this.
 *
 * Dimension key (derived from EKF algebra):
 * NxN = STATE_DIM × STATE_DIM (15×15) — covariance propagation
 * MxN = MEAS_DIM × STATE_DIM (6×15) — observation / gain
 * MxM = MEAS_DIM × MEAS_DIM (6×6) — innovation covariance
 */
static void mat_mul_NxN (const double *A, const double *B, double *C);
static void mat_mul_MxN (const double *A, const double *B, double *C);
static void mat_mul_MxNxM(const double *A, const double *B, double *C);
static void mat_mul_NxMxM(const double *A, const double *B, double *C);
static void mat_mul_NxM_MxM(const double *A, const double *B, double *C);
static void mat_mul_NxMxN(const double *A, const double *B, double *C);
static void mat_trans_NxN(const double *A, double *AT);
static void mat_trans_MxN(const double *A, double *AT);
static void mat_trans_NxM(const double *A, double *AT);
static void mat_inv_M (const double *A, double *Ainv);
static void mat_add (const double *A, const double *B, double *C, int len);
static void mat_sub (const double *A, const double *B, double *C, int len);

static void nav_init (NavFilter *f);
static void nav_propagate (NavFilter *f, double dt);
static void nav_update (NavFilter *f, const double z[MEAS_DIM]);
static void gravity_accel (const NavFilter *f, const Vec3 *pos, Vec3 *acc);
static void legendre_Pnm (double costh, double sinth, int nmax,
 double P[GRAV_ORDER+1][GRAV_ORDER+1]);
static void quat_to_dcm (const double q[4], double C[3][3]);
static void ecliptic_to_body(const double eci[6], double body[6],
 double obliq, double raan,
 double inc, double argp);
static int nav_health (NavFilter *f, int step);
static void guidance_dv (NavFilter *f, const double tgt[3],
 double tof, double dv_out[3]);

/* ================================================================
 * FIXED-DIMENSION MATRIX KERNELS — software reference
 * (replaced by AME tile-dispatch in the HW build)
 *
 * Using compile-time-constant loop bounds allows the compiler to
 * fully unroll inner loops, eliminating branch overhead and enabling
 * register tiling that maps directly onto the 3×3 AME systolic array.
 * This is standard practice in flight-grade embedded code (cf. ECSS-Q-
 * ST-80-04C) and satisfies MISRA-C:2012 Rule 17.3 (no dynamic memory
 * in safety-critical functions).
 * ================================================================ */

/* C = A(N×N) · B(N×N) — covariance propagation kernel */
static void mat_mul_NxN(const double *A, const double *B, double *C)
{
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 double s = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 s += A[i*STATE_DIM + k] * B[k*STATE_DIM + j];
 C[i*STATE_DIM + j] = s;
 }
}

/* C = A(M×N) · B(N×N) → C(M×N) — H·P */
static void mat_mul_MxN(const double *A, const double *B, double *C)
{
 for (int i = 0; i < MEAS_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 double s = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 s += A[i*STATE_DIM + k] * B[k*STATE_DIM + j];
 C[i*STATE_DIM + j] = s;
 }
}

/* C = A(M×N) · B(N×M) → C(M×M) — HP·H^T */
static void mat_mul_MxNxM(const double *A, const double *B, double *C)
{
 for (int i = 0; i < MEAS_DIM; i++)
 for (int j = 0; j < MEAS_DIM; j++) {
 double s = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 s += A[i*STATE_DIM + k] * B[k*MEAS_DIM + j];
 C[i*MEAS_DIM + j] = s;
 }
}

/* C = A(N×N) · B(N×M) → C(N×M) — P·H^T */
static void mat_mul_NxMxM(const double *A, const double *B, double *C)
{
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < MEAS_DIM; j++) {
 double s = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 s += A[i*STATE_DIM + k] * B[k*MEAS_DIM + j];
 C[i*MEAS_DIM + j] = s;
 }
}

/* C = A(N×M) · B(M×M) → C(N×M) — PH^T·S^{-1}, K·R */
static void mat_mul_NxM_MxM(const double *A, const double *B, double *C)
{
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < MEAS_DIM; j++) {
 double s = 0.0;
 for (int k = 0; k < MEAS_DIM; k++)
 s += A[i*MEAS_DIM + k] * B[k*MEAS_DIM + j];
 C[i*MEAS_DIM + j] = s;
 }
}

/* C = A(N×M) · B(M×N) → C(N×N) — K·H, KR·K^T */
static void mat_mul_NxMxN(const double *A, const double *B, double *C)
{
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 double s = 0.0;
 for (int k = 0; k < MEAS_DIM; k++)
 s += A[i*MEAS_DIM + k] * B[k*STATE_DIM + j];
 C[i*STATE_DIM + j] = s;
 }
}

/* AT = A^T for N×N */
static void mat_trans_NxN(const double *A, double *AT)
{
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 AT[j*STATE_DIM + i] = A[i*STATE_DIM + j];
}

/* AT(N×M) = A(M×N)^T */
static void mat_trans_MxN(const double *A, double *AT)
{
 for (int i = 0; i < MEAS_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 AT[j*MEAS_DIM + i] = A[i*STATE_DIM + j];
}

/* AT(M×N) = A(N×M)^T */
static void mat_trans_NxM(const double *A, double *AT)
{
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < MEAS_DIM; j++)
 AT[j*STATE_DIM + i] = A[i*MEAS_DIM + j];
}

/* Gauss-Jordan partial-pivot inverse, MEAS_DIM × MEAS_DIM only.
 *
 * Uses a static augmented-matrix scratchpad (MISRA-C:2012 compliant —
 * no heap allocation). For the 6×6 innovation matrix S this requires
 * 6 × 12 × 8 = 576 bytes of BSS, well within any flight SRAM budget.
 *
 * NOTE for flight: consider Square-Root Information Filter (SRIF) or
 * U-D covariance factorisation (Bierman, 1977) to guarantee P remains
 * positive-definite over long-duration missions. The Joseph-form
 * covariance update in nav_update() partially mitigates round-off
 * loss of symmetry; full U-D factorisation would eliminate explicit
 * matrix inversion for covariance operations entirely. */
static void mat_inv_M(const double *A, double *Ainv)
{
 /* Static scratchpad — no malloc/flight_free_impl(MISRA-C Rule 21.3). Sized
 * for MEAS_DIM × (2·MEAS_DIM) augmented matrix [A | I]. */
 static double aug[MEAS_DIM * 2 * MEAS_DIM];
 const int n = MEAS_DIM;

 memset(aug, 0, sizeof(aug));
 for (int i = 0; i < n; i++) {
 memcpy(&aug[i*2*n], &A[i*n], n * sizeof(double));
 aug[i*2*n + n + i] = 1.0;
 }
 for (int col = 0; col < n; col++) {
 /* partial pivot */
 int best = col;
 double bv = fabs(aug[col*2*n + col]);
 for (int r = col + 1; r < n; r++) {
 double av = fabs(aug[r*2*n + col]);
 if (av > bv) { bv = av; best = r; }
 }
 if (best != col)
 for (int j = 0; j < 2*n; j++) {
 double t = aug[col*2*n+j];
 aug[col*2*n+j] = aug[best*2*n+j];
 aug[best*2*n+j] = t;
 }
 double piv = aug[col*2*n+col];
 if (fabs(piv) < 1e-30) piv = 1e-30;
 for (int j = 0; j < 2*n; j++) aug[col*2*n+j] /= piv;
 for (int r = 0; r < n; r++) {
 if (r == col) continue;
 double f = aug[r*2*n+col];
 for (int j = 0; j < 2*n; j++)
 aug[r*2*n+j] -= f * aug[col*2*n+j];
 }
 }
 for (int i = 0; i < n; i++)
 memcpy(&Ainv[i*n], &aug[i*2*n + n], n * sizeof(double));
}

static void mat_add(const double *A, const double *B, double *C, int len)
{
 for (int i = 0; i < len; i++) C[i] = A[i] + B[i];
}

static void mat_sub(const double *A, const double *B, double *C, int len)
{
 for (int i = 0; i < len; i++) C[i] = A[i] - B[i];
}

/* ================================================================
 * QUATERNION → DCM (Hamilton, scalar-first)
 * ================================================================ */
static void quat_to_dcm(const double q[4], double C[3][3])
{
 double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
 double n2 = q0*q0 + q1*q1 + q2*q2 + q3*q3;
 if (n2 < 1e-30) n2 = 1e-30;
 double s = 2.0 / n2;

 C[0][0] = 1.0 - s*(q2*q2 + q3*q3);
 C[0][1] = s*(q1*q2 - q0*q3);
 C[0][2] = s*(q1*q3 + q0*q2);
 C[1][0] = s*(q1*q2 + q0*q3);
 C[1][1] = 1.0 - s*(q1*q1 + q3*q3);
 C[1][2] = s*(q2*q3 - q0*q1);
 C[2][0] = s*(q1*q3 - q0*q2);
 C[2][1] = s*(q2*q3 + q0*q1);
 C[2][2] = 1.0 - s*(q1*q1 + q2*q2);
}

/* ================================================================
 * ECLIPTIC → BODY-FIXED coordinate rotation
 * R = Rz(-argp) Rx(-inc) Rz(-raan) Rx(obliq)
 * ================================================================ */
static void ecliptic_to_body(const double eci[6], double body[6],
 double obliq, double raan,
 double inc, double argp)
{
 double ce = hw_cos(obliq), se = hw_sin(obliq);
 double cO = hw_cos(raan), sO = hw_sin(raan);
 double ci = hw_cos(inc), si = hw_sin(inc);
 double cw = hw_cos(argp), sw = hw_sin(argp);

 /* composite rotation matrix */
 double R[3][3];
 R[0][0] = cO*cw - sO*sw*ci;
 R[0][1] = -cO*sw - sO*cw*ci;
 R[0][2] = sO*si;
 R[1][0] = sO*cw + cO*sw*ci;
 R[1][1] = -sO*sw + cO*cw*ci;
 R[1][2] = -cO*si;
 R[2][0] = sw*si;
 R[2][1] = cw*si;
 R[2][2] = ci;

 /* apply obliquity rotation */
 double M[3][3];
 for (int i = 0; i < 3; i++) {
 M[i][0] = R[i][0];
 M[i][1] = R[i][1]*ce + R[i][2]*se;
 M[i][2] =-R[i][1]*se + R[i][2]*ce;
 }

 /* rotate position & velocity */
 for (int i = 0; i < 3; i++) {
 body[i] = M[i][0]*eci[0] + M[i][1]*eci[1] + M[i][2]*eci[2];
 body[i + 3] = M[i][0]*eci[3] + M[i][1]*eci[4] + M[i][2]*eci[5];
 }
}

/* ================================================================
 * ASSOCIATED LEGENDRE FUNCTIONS
 * Ref: Montenbruck & Gill, "Satellite Orbits", Sec 3.2, Eq. 3.26
 *
 * Standard recursion:
 * P[0][0] = 1
 * P[n][n] = (2n-1) sin(theta) P[n-1][n-1] (sectoral)
 * P[n][n-1] = (2n-1) cos(theta) P[n-1][n-1] (sub-diagonal)
 * P[n][m] = ((2n-1) cos(theta) P[n-1][m]
 * - (n+m-1) P[n-2][m]) / (n-m) (general)
 * ================================================================ */
static void legendre_Pnm(double costh, double sinth, int nmax,
 double P[GRAV_ORDER+1][GRAV_ORDER+1])
{
 memset(P, 0, (GRAV_ORDER+1)*(GRAV_ORDER+1)*sizeof(double));
 P[0][0] = 1.0;

 if (nmax < 1) return;
 P[1][0] = costh; /* P_1^0 = cos(theta) */
 P[1][1] = sinth; /* P_1^1 = sin(theta) */

 for (int n = 2; n <= nmax; n++) {
 /* sectoral term P[n][n] */
 P[n][n] = (2*n - 1) * sinth * P[n-1][n-1];

 /* Overflow guard — unnormalised sectoral terms grow as (2n-1)!!
 * and overflow IEEE-754 double around degree ≈ 170. Current
 * GRAV_ORDER = 8 is safe, but protect against accidental increase.
 * For high-fidelity models (EGM2008 360×360) switch to fully-
 * normalised ALFs with column-scaling (Holmes & Featherstone 2002). */
 if (!isfinite(P[n][n]) || fabs(P[n][n]) > 1e290) {
 P[n][n] = 0.0; /* truncate: higher harmonics lost */
 }

 /* sub-diagonal P[n][n-1] */
 P[n][n-1] = (2*n - 1) * costh * P[n-1][n-1];

 /* general recursion for m = 0 … n-2 */
 for (int m = 0; m <= n - 2; m++) {
 P[n][m] = ((2*n - 1) * costh * P[n-1][m]
 - (double)(n + m - 1) * P[n-2][m])
 / (double)(n - m);
 }
 }
}

/* ================================================================
 * GRAVITY ACCELERATION (JGM-3 up to degree/order GRAV_ORDER)
 * Plus point-mass third-body perturbations from Sun, Moon, etc.
 * ================================================================ */
static void gravity_accel(const NavFilter *f, const Vec3 *pos, Vec3 *acc)
{
 acc->x = acc->y = acc->z = 0.0;

 /* ── third-body point-mass contributions (skip Earth i=0) ──── */
 for (int b = 1; b < NUM_BODIES; b++) {
 Vec3 dr;
 dr.x = f->body[b].pos.x - pos->x;
 dr.y = f->body[b].pos.y - pos->y;
 dr.z = f->body[b].pos.z - pos->z;
 double d = sqrt(dr.x*dr.x + dr.y*dr.y + dr.z*dr.z);
 double d3 = d * d * d;
 if (d3 < 1e-30) d3 = 1e-30;
 acc->x += f->body[b].mu * dr.x / d3;
 acc->y += f->body[b].mu * dr.y / d3;
 acc->z += f->body[b].mu * dr.z / d3;
 }

 /* ── Earth high-order field (spherical harmonics) ──────────── */
 double r = sqrt(pos->x*pos->x + pos->y*pos->y + pos->z*pos->z);
 if (r < 100.0) r = 100.0; /* safety clamp */
 double rxy = sqrt(pos->x*pos->x + pos->y*pos->y);

 double costh = pos->z / r;
 double sinth = rxy / r;
 double coslam = (rxy > 1e-12) ? pos->x / rxy : 1.0;
 double sinlam = (rxy > 1e-12) ? pos->y / rxy : 0.0;
 double lam = hw_atan2(pos->y, pos->x);

 double P[GRAV_ORDER+1][GRAV_ORDER+1];
 legendre_Pnm(costh, sinth, GRAV_ORDER, P);

 double dU_dr = 0.0, dU_dth = 0.0, dU_dlam = 0.0;
 double mu = MU_EARTH;

 for (int n = 2; n <= GRAV_ORDER; n++) {
 double rn = (mu / (r*r)) * pow(RE_KM / r, (double)n);

 for (int m = 0; m <= n; m++) {
 double Cnm = f->Cnm[n][m];
 double Snm = f->Snm[n][m];
 double cosml = hw_cos(m * lam);
 double sinml = hw_sin(m * lam);
 double Vm = Cnm * cosml + Snm * sinml;

 /* ∂U/∂r */
 dU_dr += -(n + 1) * rn * P[n][m] * Vm;

 /* ∂P/∂θ (recursion-based, stable form) */
 double dP;
 if (m < n)
 dP = (fabs(sinth) > 1e-14)
 ? (m * costh / sinth * P[n][m] - P[n][m+1])
 : 0.0;
 else
 dP = (fabs(sinth) > 1e-14)
 ? m * costh / sinth * P[n][m]
 : 0.0;

 dU_dth += rn * dP * Vm;

 /* ∂U/∂λ */
 dU_dlam += rn * P[n][m] * m * (-Cnm * sinml + Snm * cosml);
 }
 }

 /* central term −μ/r² in radial direction */
 double a_central = -mu / (r * r);

 /* spherical → Cartesian conversion */
 double ar = a_central + dU_dr;
 double inv_sinth = (fabs(sinth) > 1e-14) ? 1.0 / sinth : 0.0;

 acc->x += ar * sinth * coslam
 + dU_dth / r * costh * coslam
 - dU_dlam / r * sinlam * inv_sinth;
 acc->y += ar * sinth * sinlam
 + dU_dth / r * costh * sinlam
 + dU_dlam / r * coslam * inv_sinth;
 acc->z += ar * costh
 - dU_dth / r * sinth;
}

/* ================================================================
 * NAVIGATION FILTER — initialisation
 * ================================================================ */
static void nav_init(NavFilter *f)
{
 memset(f, 0, sizeof(*f));

 /* initial state: ~400 km LEO, 51.6° inclination (ISS-like staging orbit) */
 f->x[0] = 6778.0; /* rx km */
 f->x[4] = 7.669; /* vy km/s (circular velocity) */
 f->x[6] = 1.0; /* quaternion scalar */
 f->x[13] = 1.2e-4; /* accel scale factor error (ppm-level)*/
 f->x[14] = 3.5e-6; /* gyro drift (rad/s, MEMS-grade) */

 /* diagonal covariance from a-priori OD (values in P0_DIAG) */
 for (int i = 0; i < STATE_DIM; i++) f->P[i][i] = P0_DIAG[i];

 /* process noise (SNC formulation, Montenbruck & Gill Sec 5.6) */
 double qdiag[STATE_DIM] = {
 8.1e-3, 8.1e-3, 8.1e-3, /* pos: drag + SRP */
 1.2e-3, 1.2e-3, 1.2e-3, /* vel */
 8.5e-5, 8.5e-5, 8.5e-5, 8.5e-5,/* quat */
 1.2e-5, 1.2e-5, 1.2e-5, /* ang rate (ARW) */
 7.5e-7, /* accel bias RW */
 4.2e-8 /* gyro in-run stab */
 };
 for (int i = 0; i < STATE_DIM; i++) f->Q[i][i] = qdiag[i];

 /* ── simplified body ephemeris (ECI J2000, km) ───────────────── */
 /* Earth (central body, index 0) */
 f->body[0].pos = (Vec3){0.0, 0.0, 0.0};
 f->body[0].mu = MU_EARTH;
 /* Moon — realistic 3-D position (incl. ~5° orbital inclination) */
 f->body[1].pos = (Vec3){350000.0, 120000.0, 18000.0};
 f->body[1].vel = (Vec3){0.0, 1.022, 0.0};
 f->body[1].mu = 4902.8;
 /* Sun — ecliptic plane offset in ECI */
 f->body[2].pos = (Vec3){-1.2e8, 8.4e7, 3.6e7};
 f->body[2].mu = 1.327e11;
 /* Mars */
 f->body[3].pos = (Vec3){2.1e8, 1.0e8, 4.0e7};
 f->body[3].mu = 42828.0;
 /* Jupiter */
 f->body[4].pos = (Vec3){5.9e8, -3.2e8, -1.2e7};
 f->body[4].mu = 1.267e8;

 /* ── JGM-3 gravity coefficients (unnormalised zonals + key tesserals) */
 f->Cnm[0][0] = 1.0;
 f->Cnm[2][0] = -J2;
 f->Cnm[3][0] = J3;
 f->Cnm[4][0] = J4;
 f->Cnm[2][2] = 1.57457e-6;
 f->Cnm[3][1] = 2.19264e-6;
 f->Cnm[3][2] = 3.08905e-7;
 f->Cnm[3][3] = 1.00559e-7;
 f->Cnm[4][1] = -5.08726e-7;
 f->Cnm[4][4] = 3.48326e-9;
 /* Snm: only S22 is large enough to matter for 8×8 */
 f->Snm[2][2] = -9.03803e-7;
 f->Snm[3][1] = 2.68424e-7;

 f->total_dv = 0.0;
 f->maneuver_count = 0;
}

/* ================================================================
 * STATE PROPAGATION (Euler with gravity + attitude dynamics)
 * ================================================================ */
static void nav_propagate(NavFilter *f, double dt)
{
 Vec3 pos = {f->x[0], f->x[1], f->x[2]};
 Vec3 vel = {f->x[3], f->x[4], f->x[5]};
 double q[4] = {f->x[6], f->x[7], f->x[8], f->x[9]};
 Vec3 w = {f->x[10], f->x[11], f->x[12]};
 double sa = f->x[13]; /* accel scale factor */
 double gd = f->x[14]; /* gyro drift */

 /* ── 1–2. RK4 orbit integration (pos/vel) ────────────────── */
 Vec3 acc;
 gravity_accel(f, &pos, &acc);
 Vec3 k1r = vel, k1v = acc;

 Vec3 rk_p2 = {pos.x+0.5*dt*k1r.x, pos.y+0.5*dt*k1r.y, pos.z+0.5*dt*k1r.z};
 Vec3 k2v; gravity_accel(f, &rk_p2, &k2v);
 Vec3 k2r = {vel.x+0.5*dt*k1v.x, vel.y+0.5*dt*k1v.y, vel.z+0.5*dt*k1v.z};

 Vec3 rk_p3 = {pos.x+0.5*dt*k2r.x, pos.y+0.5*dt*k2r.y, pos.z+0.5*dt*k2r.z};
 Vec3 k3v; gravity_accel(f, &rk_p3, &k3v);
 Vec3 k3r = {vel.x+0.5*dt*k2v.x, vel.y+0.5*dt*k2v.y, vel.z+0.5*dt*k2v.z};

 Vec3 rk_p4 = {pos.x+dt*k3r.x, pos.y+dt*k3r.y, pos.z+dt*k3r.z};
 Vec3 k4v; gravity_accel(f, &rk_p4, &k4v);
 Vec3 k4r = {vel.x+dt*k3v.x, vel.y+dt*k3v.y, vel.z+dt*k3v.z};

 pos.x += dt/6.0*(k1r.x + 2.0*k2r.x + 2.0*k3r.x + k4r.x);
 pos.y += dt/6.0*(k1r.y + 2.0*k2r.y + 2.0*k3r.y + k4r.y);
 pos.z += dt/6.0*(k1r.z + 2.0*k2r.z + 2.0*k3r.z + k4r.z);
 vel.x += dt/6.0*(k1v.x + 2.0*k2v.x + 2.0*k3v.x + k4v.x);
 vel.y += dt/6.0*(k1v.y + 2.0*k2v.y + 2.0*k3v.y + k4v.y);
 vel.z += dt/6.0*(k1v.z + 2.0*k2v.z + 2.0*k3v.z + k4v.z);

 /* ── 3. quaternion kinematic integration ──────────────────── */
 double wx = w.x, wy = w.y, wz = w.z;
 double qdot[4];
 qdot[0] = 0.5*(-q[1]*wx - q[2]*wy - q[3]*wz);
 qdot[1] = 0.5*( q[0]*wx + q[2]*wz - q[3]*wy);
 qdot[2] = 0.5*( q[0]*wy - q[1]*wz + q[3]*wx);
 qdot[3] = 0.5*( q[0]*wz + q[1]*wy - q[2]*wx);
 for (int i = 0; i < 4; i++) q[i] += qdot[i] * dt;
 double qn = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
 if (qn > 1e-30) for (int i = 0; i < 4; i++) q[i] /= qn;

 /* ── 4. Euler rigid-body dynamics (torque-free) ──────────── */
 /* Ix=450, Iy=380, Iz=520 kg·m² (spacecraft bus) */
 double Ix = 450.0, Iy = 380.0, Iz = 520.0;
 double dwx = (Iy - Iz) * wy * wz / Ix;
 double dwy = (Iz - Ix) * wz * wx / Iy;
 double dwz = (Ix - Iy) * wx * wy / Iz;
 w.x += dwx * dt;
 w.y += dwy * dt;
 w.z += dwz * dt;

 /* ── 5. bias random walk ─────────────────────────────────── */
 /* Deterministic seed: ensures reproducible gem5 simulation */
 double sdt = sqrt(dt);
 sa += 1e-5 * sdt * (2.0 * ((double)rand() / RAND_MAX) - 1.0);
 gd += 1e-6 * sdt * (2.0 * ((double)rand() / RAND_MAX) - 1.0);

 /* ── 6. write back state ─────────────────────────────────── */
 f->x[0] = pos.x; f->x[1] = pos.y; f->x[2] = pos.z;
 f->x[3] = vel.x; f->x[4] = vel.y; f->x[5] = vel.z;
 f->x[6] = q[0]; f->x[7] = q[1]; f->x[8] = q[2]; f->x[9] = q[3];
 f->x[10] = w.x; f->x[11] = w.y; f->x[12] = w.z;
 f->x[13] = sa;
 f->x[14] = gd;
 f->epoch += dt;

 /* ── 7. build linearised state-transition Φ ──────────────── */
 double F[STATE_DIM * STATE_DIM];
 memset(F, 0, sizeof(F));
 for (int i = 0; i < STATE_DIM; i++) F[i*STATE_DIM + i] = 1.0;

 /* ∂pos/∂vel = I·dt */
 for (int i = 0; i < 3; i++) F[i*STATE_DIM + (i+3)] = dt;

 /* gravity gradient ∂acc/∂pos */
 double r = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
 double r3 = r*r*r, r5 = r3*r*r;
 double mu = MU_EARTH;
 double p3[3] = {pos.x, pos.y, pos.z};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 double gg = 3.0*mu*p3[i]*p3[j] / r5;
 if (i == j) gg -= mu / r3;
 F[(i+3)*STATE_DIM + j] = gg * dt;
 }

 /* quaternion kinematic coupling ∂qdot/∂w */
 double dcm[3][3];
 quat_to_dcm(q, dcm);
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 F[(6+i)*STATE_DIM + (10+j)] = 0.5 * dcm[i][j] * dt;

 /* gyro drift couples into quaternion propagation */
 F[6*STATE_DIM + 14] = -0.5 * w.x * dt;
 F[7*STATE_DIM + 14] = -0.5 * w.y * dt;
 F[8*STATE_DIM + 14] = -0.5 * w.z * dt;

 /* ── 8. covariance propagation P = F P F^T + Q ────────── */
 double Pflat[STATE_DIM*STATE_DIM];
 double FP[STATE_DIM*STATE_DIM];
 double FT[STATE_DIM*STATE_DIM];
 double FPFT[STATE_DIM*STATE_DIM];
 double Qflat[STATE_DIM*STATE_DIM];
 double Pnew[STATE_DIM*STATE_DIM];

 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 Pflat[i*STATE_DIM+j] = f->P[i][j];
 Qflat[i*STATE_DIM+j] = f->Q[i][j];
 }

 mat_mul_NxN(F, Pflat, FP);
 mat_trans_NxN(F, FT);
 mat_mul_NxN(FP, FT, FPFT);
 mat_add(FPFT, Qflat, Pnew, STATE_DIM*STATE_DIM);

 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 f->P[i][j] = Pnew[i*STATE_DIM+j];
}

/* ================================================================
 * EKF MEASUREMENT UPDATE
 *
 * Measurements: 3 range channels + 3 Doppler velocity channels
 * observed through body-frame DCM (star-tracker / transponder)
 * ================================================================ */
static void nav_update(NavFilter *f, const double z[MEAS_DIM])
{
 /* build H: position & velocity observed through attitude DCM */
 double H[MEAS_DIM * STATE_DIM];
 memset(H, 0, sizeof(H));

 double q4[4] = {f->x[6], f->x[7], f->x[8], f->x[9]};
 double Cbn[3][3];
 quat_to_dcm(q4, Cbn);

 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 H[i * STATE_DIM + j] = Cbn[i][j]; /* pos block */
 H[(i+3)*STATE_DIM + (j+3)] = Cbn[i][j]; /* vel block */
 }
 /* accel-bias coupling into velocity measurements */
 for (int i = 3; i < 6; i++) H[i*STATE_DIM + 13] = 0.018;

 /* predicted measurement zpred = H · x (proper matrix-vector product) */
 double zpred[MEAS_DIM];
 memset(zpred, 0, sizeof(zpred));
 for (int i = 0; i < MEAS_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 zpred[i] += H[i * STATE_DIM + j] * f->x[j];

 /* innovation y = z − zpred */
 double y[MEAS_DIM];
 for (int i = 0; i < MEAS_DIM; i++) y[i] = z[i] - zpred[i];

 /* flatten P, H etc. for matrix kernels */
 double Pflat[STATE_DIM*STATE_DIM];
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 Pflat[i*STATE_DIM+j] = f->P[i][j];

 /* measurement noise R (diagonal): range 15 m → 2.25e-4 km², Doppler 0.8 m/s → 6.4e-7 */
 double R[MEAS_DIM * MEAS_DIM];
 memset(R, 0, sizeof(R));
 for (int i = 0; i < 3; i++) R[i*MEAS_DIM+i] = 2.25e-4;
 for (int i = 3; i < 6; i++) R[i*MEAS_DIM+i] = 6.4e-7;

 /* S = H P H^T + R */
 double HT[STATE_DIM * MEAS_DIM];
 double HP[MEAS_DIM * STATE_DIM];
 double HPHT[MEAS_DIM * MEAS_DIM];
 double S[MEAS_DIM * MEAS_DIM];
 mat_trans_MxN(H, HT);
 mat_mul_MxN(H, Pflat, HP);
 mat_mul_MxNxM(HP, HT, HPHT);
 mat_add(HPHT, R, S, MEAS_DIM*MEAS_DIM);

 /* K = P H^T S^{-1} */
 double Sinv[MEAS_DIM * MEAS_DIM];
 double PHT[STATE_DIM * MEAS_DIM];
 double K[STATE_DIM * MEAS_DIM];
 mat_inv_M(S, Sinv);
 mat_mul_NxMxM(Pflat, HT, PHT);
 mat_mul_NxM_MxM(PHT, Sinv, K);

 /* state update x += K y */
 for (int i = 0; i < STATE_DIM; i++) {
 double corr = 0.0;
 for (int j = 0; j < MEAS_DIM; j++)
 corr += K[i*MEAS_DIM + j] * y[j];
 f->x[i] += corr;
 }

 /* re-normalise quaternion */
 double qn = 0.0;
 for (int i = 6; i <= 9; i++) qn += f->x[i]*f->x[i];
 qn = sqrt(qn);
 if (qn > 1e-30) for (int i = 6; i <= 9; i++) f->x[i] /= qn;

 /* innovation chi2 for health monitoring: chi2 = y^T S^{-1} y
 * Normalised by degrees of freedom so expected value ≈ 1.0 */
 {
 double chi2_sum = 0.0;
 for (int i = 0; i < MEAS_DIM; i++) {
 double si = 0.0;
 for (int j = 0; j < MEAS_DIM; j++)
 si += Sinv[i*MEAS_DIM + j] * y[j];
 chi2_sum += y[i] * si;
 }
 f->chi2_innov = chi2_sum / (double)MEAS_DIM;
 }

 /* Covariance update — Joseph form for numerical stability:
 * P = (I - KH) P (I - KH)^T + K R K^T
 * Guarantees P remains symmetric positive semi-definite, unlike
 * the simple (I-KH)P which accumulates round-off and can lose
 * positive-definiteness over long missions. For full flight
 * robustness, U-D factorisation (Bierman 1977) or SRIF is
 * preferred — see note on mat_inv(). */
 double KH[STATE_DIM*STATE_DIM];
 double IKH[STATE_DIM*STATE_DIM];
 mat_mul_NxMxN(K, H, KH);

 double eye[STATE_DIM*STATE_DIM];
 memset(eye, 0, sizeof(eye));
 for (int i = 0; i < STATE_DIM; i++) eye[i*STATE_DIM+i] = 1.0;

 mat_sub(eye, KH, IKH, STATE_DIM*STATE_DIM);

 /* term 1: (I-KH) P (I-KH)^T */
 double IKHP[STATE_DIM*STATE_DIM];
 double IKHT[STATE_DIM*STATE_DIM];
 double term1[STATE_DIM*STATE_DIM];
 mat_mul_NxN(IKH, Pflat, IKHP);
 mat_trans_NxN(IKH, IKHT);
 mat_mul_NxN(IKHP, IKHT, term1);

 /* term 2: K R K^T */
 double KR[STATE_DIM*MEAS_DIM];
 double KT[MEAS_DIM*STATE_DIM];
 double term2[STATE_DIM*STATE_DIM];
 mat_mul_NxM_MxM(K, R, KR);
 mat_trans_NxM(K, KT);
 mat_mul_NxMxN(KR, KT, term2);

 /* P = term1 + term2, then enforce exact symmetry */
 double Pnew[STATE_DIM*STATE_DIM];
 mat_add(term1, term2, Pnew, STATE_DIM*STATE_DIM);

 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j <= i; j++) {
 double sym = 0.5 * (Pnew[i*STATE_DIM+j] + Pnew[j*STATE_DIM+i]);
 f->P[i][j] = sym;
 f->P[j][i] = sym;
 }
}

/* ================================================================
 * NAVIGATION HEALTH & MODE SELECTION
 *
 * Returns the active nav mode:
 * 0 = optical-only (pos σ < 1 km)
 * 1 = radio-ranging (pos σ < 100 km)
 * 2 = inertial hold-over (high uncertainty)
 * 3 = combined optical+radio (nominal)
 * ================================================================ */
static int nav_health(NavFilter *f, int step)
{
 double pos_tr = 0.0, vel_tr = 0.0, full_tr = 0.0;
 for (int i = 0; i < 3; i++) pos_tr += f->P[i][i];
 for (int i = 3; i < 6; i++) vel_tr += f->P[i][i];
 for (int i = 0; i < STATE_DIM; i++) full_tr += f->P[i][i];
 (void)full_tr;

 /* Use proper innovation χ² computed in nav_update (y^T S^{-1} y)
 * rather than state-based proxy. Expected value ≈ 1.0 for a
 * healthy filter; values >> 1 indicate innovation inconsistency. */
 double chi2 = f->chi2_innov;

 double pos_sig = sqrt(fabs(pos_tr));
 double vel_sig = sqrt(fabs(vel_tr));

 int mode;
 if (pos_sig < 1.0 && vel_sig < 0.001 && chi2 < 2.0)
 mode = 0;
 else if (pos_sig < 10.0 && vel_sig < 0.01 && chi2 < 4.0)
 mode = 3;
 else if (pos_sig < 100.0 && chi2 < 8.0)
 mode = 1;
 else
 mode = 2;

 int go = (pos_sig < 50.0 && vel_sig < 0.05);

 /* Divergence recovery — detect 'smug' filter (over-confident in a
 * wrong estimate) and re-inflate covariance so the filter can
 * accept fresh measurements. Two triggers:
 * (a) normalised chi2 >> expected → innovations wildly inconsistent
 * (b) P very small but chi2 elevated → smug condition
 * Skip first 10 steps (filter settling / burn-in period). */
 if (step >= 10) {
 int smug = (pos_sig < 0.1 && chi2 > 4.0);
 int diverge = (chi2 > 12.0);
 if (smug || diverge) {
 for (int i = 0; i < STATE_DIM; i++) {
 f->P[i][i] = P0_DIAG[i]; /* re-inflate diagonal */
 for (int j = 0; j < STATE_DIM; j++)
 if (i != j) f->P[i][j] *= 0.5; /* damp cross terms */
 }
 f->divergence_count++;
 if (step % 100 == 0 || f->divergence_count <= 3)
 FLIGHT_LOG(" [Recovery] step %4d chi2=%.3f pos_sig=%.3f "
 "reset #%d (%s)\n",
 step, chi2, pos_sig, f->divergence_count,
 smug ? "smug" : "diverge");
 }
 }

 if (step % 100 == 0)
 FLIGHT_LOG(" [Health] step %4d mode=%d pos_sig=%.3f km "
 "vel_sig=%.6f km/s chi2=%.3f go=%d\n",
 step, mode, pos_sig, vel_sig, chi2, go);

 return mode;
}

/* ================================================================
 * GUIDANCE ΔV CORRECTION (Lambert two-impulse)
 *
 * Computes required ΔV from current position to target via a
 * Lambert arc, plus 15×15 sensitivity for covariance dispersion
 * analysis (Battin, 1999, Sec 6.3).
 * ================================================================ */
static void guidance_dv(NavFilter *f, const double tgt[3],
 double tof, double dv_out[3])
{
 (void)tof;
 double r1[3], v1[3];
 for (int i = 0; i < 3; i++) { r1[i] = f->x[i]; v1[i] = f->x[i+3]; }

 double r1m = sqrt(r1[0]*r1[0] + r1[1]*r1[1] + r1[2]*r1[2]);
 double r2m = sqrt(tgt[0]*tgt[0] + tgt[1]*tgt[1] + tgt[2]*tgt[2]);
 if (r1m < 1.0) r1m = 1.0;
 if (r2m < 1.0) r2m = 1.0;

 double cdth = (r1[0]*tgt[0]+r1[1]*tgt[1]+r1[2]*tgt[2]) / (r1m*r2m);
 if (cdth > 1.0) cdth = 1.0;
 if (cdth < -1.0) cdth = -1.0;
 double sdth = sqrt(1.0 - cdth*cdth);

 /* universal-variable Lambert (simplified Battin) */
 double mu = f->body[0].mu;
 double k = r1m * r2m * (1.0 - cdth);
 double l = r1m + r2m;
 double m = r1m * r2m * (1.0 + cdth);
 double pi = k / (l + sqrt(fabs(2.0 * m)));
 if (fabs(pi) < 1e-12) pi = 1e-12;

 double ff = 1.0 - r2m / pi * (1.0 - cdth);
 double g = r1m * r2m * sdth / sqrt(mu * fabs(pi));
 if (fabs(g) < 1e-12) g = 1e-12;
 double gd_coef = 1.0 - r1m / pi * (1.0 - cdth);
 (void)gd_coef;

 double vt[3];
 for (int i = 0; i < 3; i++)
 vt[i] = (tgt[i] - ff * r1[i]) / g;

 for (int i = 0; i < 3; i++)
 dv_out[i] = vt[i] - v1[i];

 /* ── numerical Jacobian ∂dv/∂state (finite differences) ──── */
 double J[3 * STATE_DIM];
 memset(J, 0, sizeof(J));
 double eps = 1e-6;
 for (int c = 0; c < 6; c++) { /* perturb pos & vel only */
 double saved = f->x[c];
 f->x[c] = saved + eps;

 double rp[3], vp[3];
 for (int i = 0; i < 3; i++) { rp[i] = f->x[i]; vp[i] = f->x[i+3]; }
 double rpm = sqrt(rp[0]*rp[0]+rp[1]*rp[1]+rp[2]*rp[2]);
 if (rpm < 1.0) rpm = 1.0;
 double cp = (rp[0]*tgt[0]+rp[1]*tgt[1]+rp[2]*tgt[2])/(rpm*r2m);
 if (cp > 1.0) cp = 1.0;
 if (cp < -1.0) cp = -1.0;
 double sp = sqrt(1.0 - cp*cp);
 double pp = rpm*r2m*(1.0-cp) / (rpm+r2m + sqrt(fabs(2.0*rpm*r2m*(1.0+cp))));
 if (fabs(pp) < 1e-12) pp = 1e-12;
 double fp2 = 1.0 - r2m/pp*(1.0-cp);
 double gp = rpm*r2m*sp / sqrt(mu*fabs(pp));
 if (fabs(gp) < 1e-12) gp = 1e-12;
 for (int i = 0; i < 3; i++)
 J[i*STATE_DIM + c] = ((tgt[i]-fp2*rp[i])/gp - vp[i] - dv_out[i]) / eps;

 f->x[c] = saved;
 }

 /* dispersion covariance C_dv = J P J^T */
 double Pflat[STATE_DIM*STATE_DIM];
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 Pflat[i*STATE_DIM+j] = f->P[i][j];

 /* we only need top-left 3×3 of J P J^T → do it row-by-row */
 double dv_3sig_sq = 0.0;
 for (int i = 0; i < 3; i++) {
 double row[STATE_DIM];
 for (int k = 0; k < STATE_DIM; k++) {
 double s = 0.0;
 for (int j = 0; j < STATE_DIM; j++)
 s += J[i*STATE_DIM+j] * Pflat[j*STATE_DIM+k];
 row[k] = s;
 }
 double diag = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 diag += row[k] * J[i*STATE_DIM+k];
 dv_3sig_sq += 9.0 * diag;
 }
 double dv_3sig = sqrt(fabs(dv_3sig_sq));

 FLIGHT_LOG(" Guidance: dV=[%.6f, %.6f, %.6f] km/s 3sig=%.6f km/s\n",
 dv_out[0], dv_out[1], dv_out[2], dv_3sig);
}

/* ================================================================
 * MAIN — simulation driver
 * ================================================================ */
int main(void)
{
 /* Deterministic seed: ensures reproducible gem5 simulation */
 srand(3079);

 NavFilter nf;
 nav_init(&nf);

 FLIGHT_LOG("[NAV] init\n");
 FLIGHT_LOG(" State dim : %d x %d\n", STATE_DIM, STATE_DIM);
 FLIGHT_LOG(" Gravity order: %d x %d (JGM-3)\n", GRAV_ORDER, GRAV_ORDER);
 FLIGHT_LOG(" Steps : %d dt = %.1f s\n\n", SIM_TIMESTEPS, DT_SEC);

 clock_t t0 = clock();

 /* target for guidance: Trans-Lunar Injection point */
 double tgt[3] = {200000.0, 150000.0, 50000.0};
 double tof_total = 86400.0 * 3.0; /* 3-day transfer */

 int mode_hist[4] = {0, 0, 0, 0}; /* track nav-mode usage */

 for (int step = 0; step < SIM_TIMESTEPS; step++) {

 /* 1. propagate orbit + attitude */
 nav_propagate(&nf, DT_SEC);

 /* 2. ecliptic → body frame for sensor modelling */
 double eci6[6], body6[6];
 for (int i = 0; i < 6; i++) eci6[i] = nf.x[i];
 double obliq = 23.4393 * M_PI / 180.0;
 double raan = 0.001 * step;
 double inc = 17.9 * M_PI / 180.0; /* lunar transfer */
 double argp = 0.0005 * step;
 ecliptic_to_body(eci6, body6, obliq, raan, inc, argp);

 /* 3. synthesise measurements consistent with EKF H matrix */
 /* Observe pos/vel through quaternion DCM (body frame) */
 double Cbn_m[3][3];
 {
 double qm[4] = {nf.x[6], nf.x[7], nf.x[8], nf.x[9]};
 quat_to_dcm(qm, Cbn_m);
 }
 double z[MEAS_DIM];
 for (int i = 0; i < 3; i++) {
 z[i] = Cbn_m[i][0]*eci6[0] + Cbn_m[i][1]*eci6[1]
 + Cbn_m[i][2]*eci6[2]
 + 0.015 * hw_sin(step*0.73 + i*1.1)
 + 0.003 * hw_cos(step*2.91 + i);
 z[i+3] = Cbn_m[i][0]*eci6[3] + Cbn_m[i][1]*eci6[4]
 + Cbn_m[i][2]*eci6[5]
 + 0.0008 * hw_sin(step*1.37 + (i+3)*0.8);
 }
 (void)body6; /* keep ecliptic_to_body call for computational load */

 /* 4. EKF measurement update */
 nav_update(&nf, z);

 /* 5. nav health & mode selection (feeds guidance logic) */
 int mode = nav_health(&nf, step);
 mode_hist[mode]++;

 /* 6. guidance ΔV (every 50 steps, only if nav mode allows) */
 if (step % 50 == 0 && mode != 2) { /* skip if inertial holdover */
 double dv[3];
 double tof_rem = tof_total - step * DT_SEC;
 if (tof_rem < 100.0) tof_rem = 100.0;
 guidance_dv(&nf, tgt, tof_rem, dv);

 /* accumulate ΔV budget */
 double dv_mag = sqrt(dv[0]*dv[0] + dv[1]*dv[1] + dv[2]*dv[2]);
 nf.total_dv += dv_mag;
 nf.maneuver_count++;

 /* ΔV budget tracking only — no velocity correction for LEO hold */
 }

 if (step % 100 == 0) {
 FLIGHT_LOG("Step %d/%d\n", step, SIM_TIMESTEPS);
 FLIGHT_LOG(" Pos: [%.3f, %.3f, %.3f] km\n",
 nf.x[0], nf.x[1], nf.x[2]);
 FLIGHT_LOG(" Vel: [%.5f, %.5f, %.5f] km/s\n",
 nf.x[3], nf.x[4], nf.x[5]);
 }
 }

 clock_t t1 = clock();
 double elapsed = (double)(t1 - t0) / CLOCKS_PER_SEC;

 /* ── final summary ───────────────────────────────────────── */
 double pos_rss = sqrt(nf.x[0]*nf.x[0]+nf.x[1]*nf.x[1]+nf.x[2]*nf.x[2]);
 double vel_rss = sqrt(nf.x[3]*nf.x[3]+nf.x[4]*nf.x[4]+nf.x[5]*nf.x[5]);
 double cov_tr = 0.0;
 for (int i = 0; i < STATE_DIM; i++) cov_tr += nf.P[i][i];

 FLIGHT_LOG("\n[NAV] results\n");
 FLIGHT_LOG(" Execution time : %.3f s\n", elapsed);
 FLIGHT_LOG(" Final |r| : %.3f km\n", pos_rss);
 FLIGHT_LOG(" Final |v| : %.5f km/s\n", vel_rss);
 FLIGHT_LOG(" Cov trace : %.6e\n", cov_tr);
 FLIGHT_LOG(" Total dV budget: %.6f km/s (%d maneuvers)\n",
 nf.total_dv, nf.maneuver_count);
 FLIGHT_LOG(" Nav mode usage : opt=%d radio=%d inertial=%d combined=%d\n",
 mode_hist[0], mode_hist[1], mode_hist[2], mode_hist[3]);
 FLIGHT_LOG(" Div recoveries : %d\n", nf.divergence_count);

 return 0;
}
