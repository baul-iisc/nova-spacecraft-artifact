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
 * Star Sensor Processing Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the onboard star sensor image processing and attitude determination
 * algorithm. This workload performs star centroid extraction, pattern matching
 * against onboard star catalogs, and least-squares attitude estimation using
 * Kalman-filtered quaternion updates. Star trackers are standard attitude sensors
 * on all GEO communication satellites (GEO communication satellite series), Earth observation
 * missions (Earth observation satellite, SAR satellite), and interplanetary probes.
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
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../flight_compliance.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------- Configuration ---------- */
#define STATE_DIM 18 /* EKF: MRP(3)+bias(3)+rate(3)+cal(3)+align(3)+thermal(3) */
#define MEAS_DIM 12 /* MRP(3)+rate(3)+centroid_resid(3)+thermal_obs(3) */
#define STAR_CATALOG_SIZE 100 /* Number of stars in onboard catalog */
#define IMAGE_SIZE 64 /* focal-plane sub-window (flight: 256×256 CCD ROI) */
#define NUM_DETECTED_STARS 15 /* Typical bright-star limit per FOV */
#define FEATURE_DIM 15 /* Angular distance feature vector */
#define NUM_TIMESTEPS 100 /* Pipeline iterations */

/* Sensor parameters */
#define FOV_DEG 15.0 /* Field of view (degrees) */
#define FOCAL_LENGTH 50.0 /* Focal length (mm) */
#define PIXEL_SIZE 0.015 /* Pixel size (mm) */

/* ---------- Orbital mechanics & gravity constants ---------- */
#define MU_EARTH 3.986004418e14 /* Earth gravitational parameter (m^3/s^2) */
#define J2_EARTH 1.08263e-3 /* J2 zonal harmonic coefficient */
#define R_EARTH 6371000.0 /* Mean Earth radius (m) */
#define EARTH_EXCLUSION_MARGIN 10.0 /* Extra margin for Earth exclusion (deg) */
#define SUN_EXCLUSION_DEG 35.0 /* Sun exclusion half-angle (deg) */
#define MOON_EXCLUSION_DEG 15.0 /* Moon exclusion half-angle (deg) */

/* Spacecraft inertia tensor for gravity gradient (kg*m^2, typical small sat) */
#define SC_IXX 25.0
#define SC_IYY 28.0
#define SC_IZZ 22.0
#define SC_IXY 0.5
#define SC_IXZ -0.3
#define SC_IYZ 0.2

/* ------------------------------------------------------------------ */
/* Generic matrix / vector operations (3x3 tile-friendly) */
/* ------------------------------------------------------------------ */

static void mat_mul(const double *A, const double *B, double *C,
 int m, int k, int n)
{
 memset(C, 0, (size_t)m * n * sizeof(double));
 for (int i = 0; i < m; i++)
 for (int j = 0; j < n; j++)
 for (int p = 0; p < k; p++)
 C[i * n + j] += A[i * k + p] * B[p * n + j];
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

static int mat_inv(const double *A, double *Ainv, int n)
{
 double aug[12 * 24]; /* max 12x24 augmented */
 if (n > 12) return -1;
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 aug[i * 2 * n + j] = A[i * n + j];
 aug[i * 2 * n + n + j] = (i == j) ? 1.0 : 0.0;
 }
 }
 for (int col = 0; col < n; col++) {
 int pivot = col;
 for (int row = col + 1; row < n; row++)
 if (fabs(aug[row * 2 * n + col]) > fabs(aug[pivot * 2 * n + col]))
 pivot = row;
 if (pivot != col)
 for (int j = 0; j < 2 * n; j++) {
 double tmp = aug[col * 2 * n + j];
 aug[col * 2 * n + j] = aug[pivot * 2 * n + j];
 aug[pivot * 2 * n + j] = tmp;
 }
 double d = aug[col * 2 * n + col];
 if (fabs(d) < 1e-15) return -1;
 for (int j = 0; j < 2 * n; j++)
 aug[col * 2 * n + j] /= d;
 for (int row = 0; row < n; row++) {
 if (row == col) continue;
 double f = aug[row * 2 * n + col];
 for (int j = 0; j < 2 * n; j++)
 aug[row * 2 * n + j] -= f * aug[col * 2 * n + j];
 }
 }
 for (int i = 0; i < n; i++)
 for (int j = 0; j < n; j++)
 Ainv[i * n + j] = aug[i * 2 * n + n + j];
 return 0;
}

static double vec_norm(const double *v, int n)
{
 double s = 0.0;
 for (int i = 0; i < n; i++) s += v[i] * v[i];
 return sqrt(s);
}

static void vec_normalize(double *v, int n)
{
 double mag = vec_norm(v, n);
 if (mag > 1e-15)
 for (int i = 0; i < n; i++) v[i] /= mag;
}

static void vec_cross(const double a[3], const double b[3], double c[3])
{
 c[0] = a[1] * b[2] - a[2] * b[1];
 c[1] = a[2] * b[0] - a[0] * b[2];
 c[2] = a[0] * b[1] - a[1] * b[0];
}

/* ------------------------------------------------------------------ */
/* Quaternion utilities */
/* ------------------------------------------------------------------ */

static void quat_normalize(double q[4])
{
 double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
 if (n > 1e-15)
 for (int i = 0; i < 4; i++) q[i] /= n;
}

/** Quaternion product: r = p * q (Hamilton convention, scalar-first) */
static void quat_mul(const double p[4], const double q_in[4], double r[4])
{
 r[0] = p[0]*q_in[0] - p[1]*q_in[1] - p[2]*q_in[2] - p[3]*q_in[3];
 r[1] = p[0]*q_in[1] + p[1]*q_in[0] + p[2]*q_in[3] - p[3]*q_in[2];
 r[2] = p[0]*q_in[2] - p[1]*q_in[3] + p[2]*q_in[0] + p[3]*q_in[1];
 r[3] = p[0]*q_in[3] + p[1]*q_in[2] - p[2]*q_in[1] + p[3]*q_in[0];
}

/** Quaternion to 3x3 DCM (flat 9-element array, row-major) */
static void quat_to_dcm(const double q[4], double dcm[9])
{
 double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
 dcm[0] = 1 - 2*(q2*q2 + q3*q3);
 dcm[1] = 2*(q1*q2 - q0*q3);
 dcm[2] = 2*(q1*q3 + q0*q2);
 dcm[3] = 2*(q1*q2 + q0*q3);
 dcm[4] = 1 - 2*(q1*q1 + q3*q3);
 dcm[5] = 2*(q2*q3 - q0*q1);
 dcm[6] = 2*(q1*q3 - q0*q2);
 dcm[7] = 2*(q2*q3 + q0*q1);
 dcm[8] = 1 - 2*(q1*q1 + q2*q2);
}

/** Modified Rodrigues Parameters (MRP) to quaternion */

/* ------------------------------------------------------------------ */
/* Star catalog */
/* ------------------------------------------------------------------ */

static void initialize_star_catalog(double catalog[][3], int size)
{
 unsigned int seed = 42;
 for (int i = 0; i < size; i++) {
 /* Deterministic pseudo-random unit vectors */
 seed = seed * 1103515245 + 12345;
 catalog[i][0] = ((seed >> 16) & 0x7FFF) / 16383.5 - 1.0;
 seed = seed * 1103515245 + 12345;
 catalog[i][1] = ((seed >> 16) & 0x7FFF) / 16383.5 - 1.0;
 seed = seed * 1103515245 + 12345;
 catalog[i][2] = ((seed >> 16) & 0x7FFF) / 16383.5 - 1.0;
 vec_normalize(catalog[i], 3);
 }
}

/* ------------------------------------------------------------------ */
/* Star image simulation */
/* ------------------------------------------------------------------ */

static void generate_star_image(double image[IMAGE_SIZE][IMAGE_SIZE],
 const double catalog[][3],
 const double dcm[9], int num_stars)
{
 double fov_rad = FOV_DEG * M_PI / 180.0;
 memset(image, 0, IMAGE_SIZE * IMAGE_SIZE * sizeof(double));

 for (int s = 0; s < num_stars; s++) {
 int idx = (s * 7 + 13) % STAR_CATALOG_SIZE;

 /* Rotate catalog star to body frame: b = DCM * r */
 double b[3] = {0};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 b[i] += dcm[i * 3 + j] * catalog[idx][j];

 if (b[2] <= cos(fov_rad / 2.0)) continue;

 /* Gnomonic projection */
 double px = FOCAL_LENGTH * b[0] / b[2] / PIXEL_SIZE + IMAGE_SIZE / 2;
 double py = FOCAL_LENGTH * b[1] / b[2] / PIXEL_SIZE + IMAGE_SIZE / 2;
 int ix = (int)(px + 0.5);
 int iy = (int)(py + 0.5);

 if (ix < 1 || ix >= IMAGE_SIZE - 1 || iy < 1 || iy >= IMAGE_SIZE - 1)
 continue;

 /* Gaussian PSF (3x3 kernel) */
 for (int di = -1; di <= 1; di++)
 for (int dj = -1; dj <= 1; dj++) {
 double r2 = di * di + dj * dj;
 image[iy + di][ix + dj] += exp(-r2 / 0.72);
 }
 }

 /* Add read noise */
 for (int i = 0; i < IMAGE_SIZE; i++)
 for (int j = 0; j < IMAGE_SIZE; j++)
 image[i][j] += 0.01 * ((i * 31 + j * 17) % 100) / 100.0;
}

/* ------------------------------------------------------------------ */
/* Star detection (centroiding) */
/* ------------------------------------------------------------------ */

static int detect_stars(const double image[IMAGE_SIZE][IMAGE_SIZE],
 double centroids[][2])
{
 double threshold = 0.5;
 int count = 0;

 for (int i = 1; i < IMAGE_SIZE - 1 && count < NUM_DETECTED_STARS; i++) {
 for (int j = 1; j < IMAGE_SIZE - 1 && count < NUM_DETECTED_STARS; j++) {
 if (image[i][j] < threshold) continue;

 /* Check local maximum in 3x3 neighbourhood */
 int is_max = 1;
 for (int di = -1; di <= 1 && is_max; di++)
 for (int dj = -1; dj <= 1 && is_max; dj++)
 if ((di || dj) && image[i + di][j + dj] >= image[i][j])
 is_max = 0;
 if (!is_max) continue;

 /* Weighted centroid over 3x3 window */
 double sx = 0, sy = 0, sw = 0;
 for (int di = -1; di <= 1; di++)
 for (int dj = -1; dj <= 1; dj++) {
 double w = image[i + di][j + dj];
 sx += (j + dj) * w;
 sy += (i + di) * w;
 sw += w;
 }
 centroids[count][0] = sx / sw;
 centroids[count][1] = sy / sw;
 count++;
 }
 }
 return count;
}

/* ------------------------------------------------------------------ */
/* Feature extraction (inter-star angular distances) */
/* ------------------------------------------------------------------ */

static void extract_features(const double centroids[][2], int n,
 double features[][FEATURE_DIM])
{
 for (int i = 0; i < n; i++) {
 /* Direction vector from pixel coords */
 double xi = (centroids[i][0] - IMAGE_SIZE / 2) * PIXEL_SIZE;
 double yi = (centroids[i][1] - IMAGE_SIZE / 2) * PIXEL_SIZE;
 double zi = FOCAL_LENGTH;
 double mi = sqrt(xi * xi + yi * yi + zi * zi);
 xi /= mi; yi /= mi; zi /= mi;

 int fidx = 0;
 for (int j = 0; j < n && fidx < FEATURE_DIM; j++) {
 if (i == j) continue;
 double xj = (centroids[j][0] - IMAGE_SIZE / 2) * PIXEL_SIZE;
 double yj = (centroids[j][1] - IMAGE_SIZE / 2) * PIXEL_SIZE;
 double zj = FOCAL_LENGTH;
 double mj = sqrt(xj * xj + yj * yj + zj * zj);
 xj /= mj; yj /= mj; zj /= mj;

 double cos_ang = xi * xj + yi * yj + zi * zj;
 if (cos_ang > 1.0) cos_ang = 1.0;
 if (cos_ang < -1.0) cos_ang = -1.0;
 features[i][fidx++] = acos(cos_ang) * 180.0 / M_PI;
 }
 /* Fill remaining with trig-derived features */
 while (fidx < FEATURE_DIM) {
 double a = features[i][fidx % 3] * M_PI / 180.0;
 features[i][fidx] = sin(a) * cos(a) + tan(a * 0.5);
 fidx++;
 }
 }
}

/* ------------------------------------------------------------------ */
/* Star matching (pattern matching against catalog) */
/* ------------------------------------------------------------------ */

static void match_stars(const double features[][FEATURE_DIM], int n,
 const double catalog[][3], int cat_size,
 int matches[])
{
 (void)catalog;
 /* Generate catalog features (deterministic) */
 double cat_feat[STAR_CATALOG_SIZE][FEATURE_DIM];
 for (int i = 0; i < cat_size; i++)
 for (int j = 0; j < FEATURE_DIM; j++) {
 unsigned int s = (unsigned int)(i * 1000 + j);
 s = s * 1103515245 + 12345;
 cat_feat[i][j] = ((s >> 16) & 0x7FFF) / 32768.0 * 30.0;
 }

 /* Find nearest match for each detected star */
 for (int i = 0; i < n; i++) {
 double best_dist = 1e30;
 int best_idx = 0;
 for (int c = 0; c < cat_size; c++) {
 double dist = 0;
 for (int f = 0; f < FEATURE_DIM; f++) {
 double d = features[i][f] - cat_feat[c][f];
 dist += d * d;
 }
 if (dist < best_dist) {
 best_dist = dist;
 best_idx = c;
 }
 }
 matches[i] = best_idx;
 }
}

/* ------------------------------------------------------------------ */
/* QUEST attitude determination */
/* ------------------------------------------------------------------ */

static void quest_attitude(const double centroids[][2], int n,
 const double catalog[][3],
 const int matches[],
 double q_out[4])
{
 /* Body-frame direction vectors */
 double body[NUM_DETECTED_STARS][3];
 for (int i = 0; i < n; i++) {
 body[i][0] = (centroids[i][0] - IMAGE_SIZE / 2) * PIXEL_SIZE;
 body[i][1] = (centroids[i][1] - IMAGE_SIZE / 2) * PIXEL_SIZE;
 body[i][2] = FOCAL_LENGTH;
 vec_normalize(body[i], 3);
 }

 /* Compute attitude profile matrix B (3x3) */
 double B[9] = {0};
 double w = 1.0 / n; /* Equal weights */
 for (int s = 0; s < n; s++)
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 B[i * 3 + j] += w * body[s][i] * catalog[matches[s]][j];

 /* Construct Davenport K-matrix (4x4) from B */
 double sigma = B[0] + B[4] + B[8]; /* trace(B) */

 /* S = B + B^T */
 double S[9];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 S[i * 3 + j] = B[i * 3 + j] + B[j * 3 + i];

 /* Z vector = [B23-B32, B31-B13, B12-B21] */
 double Z[3] = {B[5] - B[7], B[6] - B[2], B[1] - B[3]};

 /* Davenport matrix K (4x4) */
 double K[16];
 K[0] = sigma; K[1] = Z[0]; K[2] = Z[1]; K[3] = Z[2];
 K[4] = Z[0]; K[5] = S[0]-sigma; K[6] = S[1]; K[7] = S[2];
 K[8] = Z[1]; K[9] = S[3]; K[10] = S[4]-sigma; K[11] = S[5];
 K[12] = Z[2]; K[13] = S[6]; K[14] = S[7]; K[15] = S[8]-sigma;

 /* Power iteration to find dominant eigenvector */
 double v[4] = {1.0, 0.0, 0.0, 0.0};
 for (int iter = 0; iter < 15; iter++) {
 double nv[4] = {0};
 for (int i = 0; i < 4; i++)
 for (int j = 0; j < 4; j++)
 nv[i] += K[i * 4 + j] * v[j];
 double mag = sqrt(nv[0]*nv[0]+nv[1]*nv[1]+nv[2]*nv[2]+nv[3]*nv[3]);
 for (int i = 0; i < 4; i++) v[i] = nv[i] / mag;
 }

 for (int i = 0; i < 4; i++) q_out[i] = v[i];
 quat_normalize(q_out);
}

/* ------------------------------------------------------------------ */
/* Centroid residual computation for calibration/misalignment obs. */
/* ------------------------------------------------------------------ */

/**
 * Compute averaged centroid prediction residuals.
 *
 * For each detected star matched to the catalog, predict the focal-plane
 * centroid from the current attitude (DCM), then compute the difference
 * from the measured centroid. Averaged residuals are sensitive to:
 * - Calibration offsets x[9..11]: pixel shift dx, dy and focal-length error df
 * - Misalignment angles x[12..14]: systematic boresight rotation (φ,θ,ψ)
 *
 * This provides the additional measurement diversity required for full
 * observability of the 18-state EKF (resolves CDR observability concern).
 *
 * Jacobian coupling (linearised sensitivity):
 * ∂(resid_x)/∂(cal_x) = 1 (direct pixel offset)
 * ∂(resid_y)/∂(cal_y) = 1 (direct pixel offset)
 * ∂(resid_r)/∂(cal_f) ≈ tan(θ_star)/σ_p (radial focal-length distortion)
 * ∂(resid_x)/∂(align_θ) = −f/σ_p (pitch → x-shift on focal plane)
 * ∂(resid_y)/∂(align_φ) = +f/σ_p (roll → y-shift on focal plane)
 * ∂(resid_r)/∂(align_ψ) ≈ f·r̄/σ_p (yaw → tangential rotation)
 *
 * @param centroids Measured centroid positions (pixels)
 * @param n_det Number of detected/matched stars
 * @param catalog Star catalog (unit vectors in ECI)
 * @param matches Index mapping: detected star → catalog star
 * @param dcm Current body-from-inertial DCM (3×3 row-major)
 * @param x EKF state vector (18 elements)
 * @param residual Output: averaged [dx, dy, dr] centroid residuals (pixels)
 */
static void compute_centroid_residuals(const double centroids[][2], int n_det,
 const double catalog[][3],
 const int matches[],
 const double dcm[9],
 const double x[STATE_DIM],
 double residual[3])
{
 residual[0] = residual[1] = residual[2] = 0.0;
 if (n_det < 2) return;

 /* Build misalignment rotation (small-angle approximation):
 * M = I + [align×] where align = [φ, θ, ψ] = x[12..14] */
 double dcm_corr[9];
 {
 double M_align[9] = {
 1.0, -x[14], x[13],
 x[14], 1.0, -x[12],
 -x[13], x[12], 1.0
 };
 /* DCM_corrected = M_align · DCM (misaligned body frame) */
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 dcm_corr[i * 3 + j] = 0.0;
 for (int k = 0; k < 3; k++)
 dcm_corr[i * 3 + j] += M_align[i * 3 + k] * dcm[k * 3 + j];
 }
 }

 double fl = FOCAL_LENGTH + x[11]; /* corrected focal length (mm) */
 int count = 0;

 for (int s = 0; s < n_det; s++) {
 int idx = matches[s];
 if (idx < 0 || idx >= STAR_CATALOG_SIZE) continue;

 /* Predicted body-frame direction: b = DCM_corr · catalog[idx] */
 double b[3] = {0};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 b[i] += dcm_corr[i * 3 + j] * catalog[idx][j];

 if (b[2] <= 0.01) continue; /* star behind focal plane */

 /* Predicted pixel coordinates with calibration corrections */
 double pred_x = fl * b[0] / (b[2] * PIXEL_SIZE)
 + IMAGE_SIZE / 2.0 + x[9];
 double pred_y = fl * b[1] / (b[2] * PIXEL_SIZE)
 + IMAGE_SIZE / 2.0 + x[10];

 double dx = centroids[s][0] - pred_x;
 double dy = centroids[s][1] - pred_y;
 double dr = sqrt(dx * dx + dy * dy);

 residual[0] += dx;
 residual[1] += dy;
 residual[2] += dr;
 count++;
 }

 if (count > 0) {
 double inv_count = 1.0 / count;
 residual[0] *= inv_count;
 residual[1] *= inv_count;
 residual[2] *= inv_count;
 }
}

/* ------------------------------------------------------------------ */
/* Temporal differencing for thermal-drift observability */
/* ------------------------------------------------------------------ */

/**
 * Compute thermal-drift observation via temporal attitude differencing.
 *
 * The thermal drift states x[15..17] induce a slowly-varying bias in the
 * attitude measurement that cannot be separated from the gyro bias in a
 * single-epoch observation. By differencing the MRP measurement across
 * successive epochs and removing the predicted kinematic change, the
 * residual reveals the thermal drift contribution:
 *
 * z_thermal[i] = (mrp_now[i] − mrp_prev[i]) − 0.25 · ω_est[i] · dt
 *
 * This is the attitude "innovation rate" — the part of the attitude change
 * unexplained by the gyroscope/dynamics model, attributable to unmodeled
 * thermal deformation of the star tracker optical assembly.
 *
 * Over a longer observation window (> thermal time constant, typically
 * 10–30 min for a typical star tracker), these observations accumulate
 * sufficient Fisher information to constrain the thermal drift states,
 * resolving the observability deficiency for slowly-varying states.
 *
 * @param mrp_now Current MRP measurement (from QUEST, 3 elements)
 * @param mrp_prev Previous-epoch MRP measurement (3 elements)
 * @param omega_est Estimated angular velocity from EKF x[6..8] (rad/s)
 * @param dt Time step (s)
 * @param z_therm Output: thermal drift observation (3 elements)
 */
static void compute_thermal_drift_obs(const double mrp_now[3],
 const double mrp_prev[3],
 const double omega_est[3],
 double dt,
 double z_therm[3])
{
 for (int i = 0; i < 3; i++) {
 double predicted_change = 0.25 * omega_est[i] * dt;
 z_therm[i] = (mrp_now[i] - mrp_prev[i]) - predicted_change;
 }
}

/* ------------------------------------------------------------------ */
/* Observability Gramian condition monitoring */
/* ------------------------------------------------------------------ */

/**
 * Compute local observability metric for the 18-state EKF.
 *
 * Evaluates the single-epoch Fisher information matrix:
 *
 * M_info = H^T · R^{-1} · H (18×18)
 *
 * Then checks each of the 6 state blocks (MRP, bias, rate, cal, align,
 * thermal). Blocks with negligible information (trace < threshold) are
 * flagged as weakly observable and their covariance is bounded from below
 * to prevent filter divergence.
 *
 * With the expanded 12-dim measurement model (MRP + rate + centroid
 * residuals + temporal differencing), all 6 blocks should receive
 * non-zero information, confirming full observability.
 *
 * @param H Measurement Jacobian (MEAS_DIM × STATE_DIM)
 * @param R Measurement noise covariance (MEAS_DIM × MEAS_DIM)
 * @param P State covariance (may be bounded for weak states)
 * @param obs_metric Output: per-block information trace (6 values)
 * @return Number of fully observable state blocks (0–6)
 */
static int compute_observability_metric(const double H[MEAS_DIM * STATE_DIM],
 const double R[MEAS_DIM * MEAS_DIM],
 double P[STATE_DIM * STATE_DIM],
 double obs_metric[6])
{
 /* Diagonal-R assumption: R_inv[k] = 1/R[k][k] */
 double Rinv_diag[MEAS_DIM];
 for (int i = 0; i < MEAS_DIM; i++) {
 double r_ii = R[i * MEAS_DIM + i];
 Rinv_diag[i] = (fabs(r_ii) > 1e-20) ? (1.0 / r_ii) : 0.0;
 }

 /* M_info[i][j] = Σ_k H[k][i] · (1/R[k][k]) · H[k][j] */
 double M_info[STATE_DIM * STATE_DIM];
 memset(M_info, 0, sizeof(M_info));
 for (int k = 0; k < MEAS_DIM; k++) {
 double w = Rinv_diag[k];
 for (int i = 0; i < STATE_DIM; i++) {
 double h_ki_w = H[k * STATE_DIM + i] * w;
 if (fabs(h_ki_w) < 1e-20) continue;
 for (int j = i; j < STATE_DIM; j++) {
 double val = h_ki_w * H[k * STATE_DIM + j];
 M_info[i * STATE_DIM + j] += val;
 if (i != j) M_info[j * STATE_DIM + i] += val;
 }
 }
 }

 /* Per-block trace: sum of diagonal elements in each 3-state block */
 int n_observable = 0;
 double threshold = 1e-10;

 for (int b = 0; b < 6; b++) {
 double trace = 0.0;
 int base = b * 3;
 for (int i = 0; i < 3; i++)
 trace += M_info[(base + i) * STATE_DIM + (base + i)];
 obs_metric[b] = trace;

 if (trace > threshold) {
 n_observable++;
 } else {
 /* Bound covariance for weakly observable states to prevent
 * filter divergence while still allowing slow convergence. */
 for (int i = 0; i < 3; i++) {
 double p_ii = P[(base + i) * STATE_DIM + (base + i)];
 double p_max = (b < 3) ? 0.1 : 1.0;
 if (p_ii > p_max)
 P[(base + i) * STATE_DIM + (base + i)] = p_max;
 }
 }
 }

 return n_observable;
}

/* ------------------------------------------------------------------ */
/* EKF for attitude estimation (18-state) */
/* ------------------------------------------------------------------ */

/**
 * State vector (18 elements):
 * [0..2] Modified Rodrigues Parameters (MRP) attitude error
 * [3..5] Gyroscope bias (rad/s)
 * [6..8] Angular velocity (rad/s)
 * [9..11] Star tracker calibration (pixel offset x, y + focal length err)
 * [12..14] Misalignment Euler angles (rad)
 * [15..17] Thermal drift coefficients
 */

static void ekf_state_predict(double x[STATE_DIM], const double gyro[3],
 double dt)
{
 /* Corrected angular rate = gyro - bias */
 double omega[3];
 for (int i = 0; i < 3; i++)
 omega[i] = gyro[i] - x[3 + i];

 /* MRP kinematic equation: dp/dt = 0.25*[(1-p^Tp)*I + 2*[px] + 2*p*p^T]*w */
 double p2 = x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
 double G[9]; /* 3x3 MRP kinematics matrix */
 G[0] = 1-p2+2*x[0]*x[0]; G[1] = 2*(x[0]*x[1]-x[2]); G[2] = 2*(x[0]*x[2]+x[1]);
 G[3] = 2*(x[1]*x[0]+x[2]); G[4] = 1-p2+2*x[1]*x[1]; G[5] = 2*(x[1]*x[2]-x[0]);
 G[6] = 2*(x[2]*x[0]-x[1]); G[7] = 2*(x[2]*x[1]+x[0]); G[8] = 1-p2+2*x[2]*x[2];

 double dp[3] = {0};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 dp[i] += 0.25 * G[i * 3 + j] * omega[j];

 for (int i = 0; i < 3; i++)
 x[i] += dp[i] * dt;

 /* Angular velocity update (gyro model with noise) */
 for (int i = 0; i < 3; i++)
 x[6 + i] = omega[i];

 /* Bias: random walk (no change in prediction) */
 /* Calibration: slow drift */
 x[9] *= 0.9999;
 x[10] *= 0.9999;
 x[11] *= 0.9999;

 /* Misalignment: essentially constant */
 /* Thermal drift: slow exponential decay */
 for (int i = 0; i < 3; i++)
 x[15 + i] *= exp(-1e-4 * dt);
}

static void build_jacobian_F(const double x[STATE_DIM],
 const double gyro[3], double dt,
 double F[STATE_DIM * STATE_DIM])
{
 memset(F, 0, STATE_DIM * STATE_DIM * sizeof(double));
 for (int i = 0; i < STATE_DIM; i++)
 F[i * STATE_DIM + i] = 1.0;

 double omega[3];
 for (int i = 0; i < 3; i++)
 omega[i] = gyro[i] - x[3 + i];

 /* Skew-symmetric of omega */
 F[0 * STATE_DIM + 1] = omega[2] * dt * 0.5;
 F[0 * STATE_DIM + 2] = -omega[1] * dt * 0.5;
 F[1 * STATE_DIM + 0] = -omega[2] * dt * 0.5;
 F[1 * STATE_DIM + 2] = omega[0] * dt * 0.5;
 F[2 * STATE_DIM + 0] = omega[1] * dt * 0.5;
 F[2 * STATE_DIM + 1] = -omega[0] * dt * 0.5;

 /* MRP sensitivity to bias */
 for (int i = 0; i < 3; i++)
 F[i * STATE_DIM + (3 + i)] = -0.25 * dt;

 /* Angular velocity from gyro */
 for (int i = 0; i < 3; i++)
 F[(6 + i) * STATE_DIM + (3 + i)] = -1.0;

 /* Calibration decay */
 for (int i = 9; i < 12; i++)
 F[i * STATE_DIM + i] = 0.9999;

 /* Thermal drift */
 for (int i = 15; i < 18; i++)
 F[i * STATE_DIM + i] = exp(-1e-4 * dt);
}

/**
 * EKF covariance prediction: P = F*P*F^T + Q (18x18 matrix operations)
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
 * EKF measurement update with 12-dimensional measurement vector:
 * z[0..2] : MRP attitude from QUEST star-ID
 * z[3..5] : Angular rate from bias-compensated gyro
 * z[6..8] : Centroid prediction residuals (→ calibration + misalignment)
 * z[9..11]: Temporal attitude differencing (→ thermal drift)
 *
 * The expanded measurement model ensures all 18 EKF states are observable
 * through diverse sensor modalities and multi-epoch information accumulation.
 */
static void ekf_update(double x[STATE_DIM],
 double P[STATE_DIM * STATE_DIM],
 const double z[MEAS_DIM],
 const double H[MEAS_DIM * STATE_DIM],
 const double R[MEAS_DIM * MEAS_DIM])
{
 double Hx[MEAS_DIM], y[MEAS_DIM];
 mat_mul(H, x, Hx, MEAS_DIM, STATE_DIM, 1);
 for (int i = 0; i < MEAS_DIM; i++)
 y[i] = z[i] - Hx[i];

 double HP[MEAS_DIM * STATE_DIM], HT[STATE_DIM * MEAS_DIM];
 double S[MEAS_DIM * MEAS_DIM], Sinv[MEAS_DIM * MEAS_DIM];
 double HPHT[MEAS_DIM * MEAS_DIM];
 mat_mul(H, P, HP, MEAS_DIM, STATE_DIM, STATE_DIM);
 mat_transpose(H, HT, MEAS_DIM, STATE_DIM);
 mat_mul(HP, HT, HPHT, MEAS_DIM, STATE_DIM, MEAS_DIM);
 mat_add(HPHT, R, S, MEAS_DIM, MEAS_DIM);
 mat_inv(S, Sinv, MEAS_DIM);

 double PHT[STATE_DIM * MEAS_DIM], K[STATE_DIM * MEAS_DIM];
 mat_mul(P, HT, PHT, STATE_DIM, STATE_DIM, MEAS_DIM);
 mat_mul(PHT, Sinv, K, STATE_DIM, MEAS_DIM, MEAS_DIM);

 double Ky[STATE_DIM];
 mat_mul(K, y, Ky, STATE_DIM, MEAS_DIM, 1);
 for (int i = 0; i < STATE_DIM; i++)
 x[i] += Ky[i];

 double KH[STATE_DIM * STATE_DIM], I_KH[STATE_DIM * STATE_DIM];
 mat_mul(K, H, KH, STATE_DIM, MEAS_DIM, STATE_DIM);
 for (int i = 0; i < STATE_DIM * STATE_DIM; i++)
 I_KH[i] = -KH[i];
 for (int i = 0; i < STATE_DIM; i++)
 I_KH[i * STATE_DIM + i] += 1.0;
 double Pnew[STATE_DIM * STATE_DIM];
 mat_mul(I_KH, P, Pnew, STATE_DIM, STATE_DIM, STATE_DIM);
 memcpy(P, Pnew, STATE_DIM * STATE_DIM * sizeof(double));
}

/* ------------------------------------------------------------------ */
/* Simulated sensors */
/* ------------------------------------------------------------------ */

static double rand_gauss_star(unsigned int *seed)
{
 *seed = *seed * 1103515245 + 12345;
 double u1 = ((*seed >> 16) & 0x7FFF) / 32768.0;
 *seed = *seed * 1103515245 + 12345;
 double u2 = ((*seed >> 16) & 0x7FFF) / 32768.0;
 if (u1 < 1e-15) u1 = 1e-15;
 return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ------------------------------------------------------------------ */
/* OrbitalMecha: Orbit propagation with J2 secular drift */
/* ------------------------------------------------------------------ */

/**
 * Orbital elements structure (classical Keplerian).
 * a : semi-major axis (m)
 * e : eccentricity
 * inc : inclination (rad)
 * Omega : right ascension of ascending node – RAAN (rad)
 * omega : argument of perigee (rad)
 * M : mean anomaly (rad)
 */
typedef struct {
 double a;
 double e;
 double inc;
 double Omega;
 double omega;
 double M;
} OrbitalElements;

/**
 * Propagate orbital elements by dt seconds with J2 secular perturbations.
 *
 * J2 secular rates (Brouwer first-order):
 * dOmega/dt = -1.5 n J2 (R_e/a)^2 cos(i) / (1-e^2)^2
 * domega/dt = 1.5 n J2 (R_e/a)^2 (2 - 2.5 sin^2 i) / (1-e^2)^2
 * dM/dt = n [1 + 1.5 J2 (R_e/a)^2 (1 - 1.5 sin^2 i) sqrt(1-e^2)/(1-e^2)^2]
 */
static void propagate_orbit_j2(OrbitalElements *oe, double dt)
{
 double n = sqrt(MU_EARTH / (oe->a * oe->a * oe->a)); /* mean motion */
 double Re_a = R_EARTH / oe->a;
 double Re_a2 = Re_a * Re_a;
 double sin_i = sin(oe->inc);
 double cos_i = cos(oe->inc);
 double e2 = oe->e * oe->e;
 double eta2 = 1.0 - e2;
 double eta = sqrt(eta2);
 double eta2_inv2 = 1.0 / (eta2 * eta2);

 double dOmega_dt = -1.5 * n * J2_EARTH * Re_a2 * cos_i * eta2_inv2;
 double domega_dt = 1.5 * n * J2_EARTH * Re_a2 *
 (2.0 - 2.5 * sin_i * sin_i) * eta2_inv2;
 double dM_dt = n * (1.0 + 1.5 * J2_EARTH * Re_a2 *
 (1.0 - 1.5 * sin_i * sin_i) * eta * eta2_inv2);

 oe->Omega += dOmega_dt * dt;
 oe->omega += domega_dt * dt;
 oe->M += dM_dt * dt;

 /* Wrap to [0, 2*pi) */
 oe->Omega = fmod(oe->Omega, 2.0 * M_PI);
 if (oe->Omega < 0) oe->Omega += 2.0 * M_PI;
 oe->omega = fmod(oe->omega, 2.0 * M_PI);
 if (oe->omega < 0) oe->omega += 2.0 * M_PI;
 oe->M = fmod(oe->M, 2.0 * M_PI);
 if (oe->M < 0) oe->M += 2.0 * M_PI;
}

/**
 * Solve Kepler's equation M = E - e sin(E) for eccentric anomaly E
 * via Newton-Raphson iteration.
 */
static double solve_kepler(double M, double e)
{
 double E = M;
 for (int i = 0; i < 10; i++) {
 double f = E - e * sin(E) - M;
 double fp = 1.0 - e * cos(E);
 if (fabs(fp) < 1e-15) break;
 E -= f / fp;
 }
 return E;
}

/**
 * Convert classical orbital elements to Earth-Centered Inertial (ECI)
 * position and velocity vectors.
 *
 * 1. Solve Kepler's equation for eccentric anomaly E
 * 2. True anomaly nu from E
 * 3. Perifocal position: r_pqw = [r cos(nu), r sin(nu), 0]
 * 4. Rotate perifocal -> ECI : Rz(-Omega) Rx(-i) Rz(-omega)
 */
static void orbital_elements_to_eci(const OrbitalElements *oe,
 double pos_eci[3], double vel_eci[3])
{
 double E = solve_kepler(oe->M, oe->e);
 double cos_E = cos(E);
 double sin_E = sin(E);
 double sqrt1me2 = sqrt(1.0 - oe->e * oe->e);
 double nu = atan2(sqrt1me2 * sin_E, cos_E - oe->e);
 double r = oe->a * (1.0 - oe->e * cos_E);

 /* Position in perifocal (PQW) frame */
 double r_pqw[3] = { r * cos(nu), r * sin(nu), 0.0 };

 /* Velocity in PQW frame */
 double coeff = sqrt(MU_EARTH * oe->a) / r;
 double v_pqw[3] = { coeff * (-sin_E), coeff * sqrt1me2 * cos_E, 0.0 };

 /* Rotation matrix PQW -> ECI */
 double cO = cos(oe->Omega), sO = sin(oe->Omega);
 double ci = cos(oe->inc), si = sin(oe->inc);
 double cw = cos(oe->omega), sw = sin(oe->omega);

 double R11 = cO*cw - sO*sw*ci, R12 = -cO*sw - sO*cw*ci, R13 = sO*si;
 double R21 = sO*cw + cO*sw*ci, R22 = -sO*sw + cO*cw*ci, R23 = -cO*si;
 double R31 = sw*si, R32 = cw*si, R33 = ci;

 pos_eci[0] = R11*r_pqw[0] + R12*r_pqw[1] + R13*r_pqw[2];
 pos_eci[1] = R21*r_pqw[0] + R22*r_pqw[1] + R23*r_pqw[2];
 pos_eci[2] = R31*r_pqw[0] + R32*r_pqw[1] + R33*r_pqw[2];

 vel_eci[0] = R11*v_pqw[0] + R12*v_pqw[1] + R13*v_pqw[2];
 vel_eci[1] = R21*v_pqw[0] + R22*v_pqw[1] + R23*v_pqw[2];
 vel_eci[2] = R31*v_pqw[0] + R32*v_pqw[1] + R33*v_pqw[2];
}

/**
 * Compute Earth exclusion half-angle from the spacecraft position.
 * Stars within this cone around nadir are obscured by Earth.
 *
 * Earth angular radius: alpha = asin(R_earth / |r|)
 * Total exclusion = alpha + configurable margin
 */
static double compute_earth_exclusion_angle(const double pos_eci[3])
{
 double r_mag = vec_norm(pos_eci, 3);
 if (r_mag < R_EARTH) r_mag = R_EARTH + 1.0;
 double alpha_rad = asin(R_EARTH / r_mag);
 return alpha_rad + EARTH_EXCLUSION_MARGIN * M_PI / 180.0;
}

/**
 * Sun direction in ECI from elapsed mission time.
 * Circular ecliptic model, obliquity = 23.44 deg.
 */
static void compute_sun_direction_eci(double t_sec, double sun_dir[3])
{
 double obliquity = 23.44 * M_PI / 180.0;
 double sun_period = 365.25 * 86400.0;
 double lambda = 2.0 * M_PI * t_sec / sun_period;

 sun_dir[0] = cos(lambda);
 sun_dir[1] = sin(lambda) * cos(obliquity);
 sun_dir[2] = sin(lambda) * sin(obliquity);
}

/**
 * Moon direction in ECI.
 * Circular model, period ~27.32 days, 5.15 deg inclination to ecliptic.
 */
static void compute_moon_direction_eci(double t_sec, double moon_dir[3])
{
 double obliquity = 23.44 * M_PI / 180.0;
 double moon_incl = 5.15 * M_PI / 180.0;
 double moon_period = 27.32 * 86400.0;
 double theta = 2.0 * M_PI * t_sec / moon_period;

 /* Moon in its orbital plane */
 double m_ecl[3];
 m_ecl[0] = cos(theta);
 m_ecl[1] = sin(theta) * cos(moon_incl);
 m_ecl[2] = sin(theta) * sin(moon_incl);

 /* Rotate ecliptic -> ECI (obliquity rotation about X) */
 moon_dir[0] = m_ecl[0];
 moon_dir[1] = m_ecl[1] * cos(obliquity) - m_ecl[2] * sin(obliquity);
 moon_dir[2] = m_ecl[1] * sin(obliquity) + m_ecl[2] * cos(obliquity);
}

/**
 * Determine which catalog stars are visible given Earth, Sun, and Moon
 * exclusion zones. For each star (unit vector in ECI):
 * - Earth: angle to nadir < earth_exclusion_angle -> blocked
 * - Sun: angle to Sun < SUN_EXCLUSION_DEG -> glare
 * - Moon: angle to Moon < MOON_EXCLUSION_DEG -> glare
 *
 * Returns the number of visible stars; visible[] flags are set.
 */
static int check_star_visibility(const double catalog[][3], int cat_size,
 const double pos_eci[3], double t_sec,
 int visible[])
{
 double r_mag = vec_norm(pos_eci, 3);
 double nadir[3];
 for (int i = 0; i < 3; i++) nadir[i] = -pos_eci[i] / r_mag;

 double earth_excl = compute_earth_exclusion_angle(pos_eci);
 double cos_earth_excl = cos(earth_excl);
 double cos_sun_excl = cos(SUN_EXCLUSION_DEG * M_PI / 180.0);
 double cos_moon_excl = cos(MOON_EXCLUSION_DEG * M_PI / 180.0);

 double sun_dir[3], moon_dir[3];
 compute_sun_direction_eci(t_sec, sun_dir);
 compute_moon_direction_eci(t_sec, moon_dir);

 int n_vis = 0;
 for (int s = 0; s < cat_size; s++) {
 double dot_earth = 0, dot_sun = 0, dot_moon = 0;
 for (int k = 0; k < 3; k++) {
 dot_earth += catalog[s][k] * nadir[k];
 dot_sun += catalog[s][k] * sun_dir[k];
 dot_moon += catalog[s][k] * moon_dir[k];
 }
 if (dot_earth > cos_earth_excl) {
 visible[s] = 0; /* blocked by Earth disc */
 } else if (dot_sun > cos_sun_excl) {
 visible[s] = 0; /* Sun glare zone */
 } else if (dot_moon > cos_moon_excl) {
 visible[s] = 0; /* Moon glare zone */
 } else {
 visible[s] = 1;
 n_vis++;
 }
 }
 return n_vis;
}

/* ------------------------------------------------------------------ */
/* GravityPoten: J2 gravity gradient torque computation */
/* ------------------------------------------------------------------ */

/**
 * Compute gravity gradient disturbance torque on the spacecraft.
 *
 * Physics:
 * A gravity gradient torque arises because different mass elements of the
 * spacecraft are at slightly different distances from the Earth centre,
 * creating a tidal differential that tends to align the minimum-inertia
 * axis with nadir.
 *
 * T_gg = (3 mu / r^3) * (z_nadir x (J * z_nadir))
 *
 * where z_nadir is the nadir unit vector in body frame and J is the
 * spacecraft inertia tensor (3x3). A first-order J2 correction is
 * applied to account for the oblate Earth gravity field:
 *
 * factor = 1 + 1.5 J2 (R_e/r)^2 (1 - 5 sin^2(lat))
 *
 * Inputs:
 * pos_eci[3] – spacecraft ECI position (m)
 * dcm_bi[9] – body-from-inertial DCM (3x3 row-major)
 *
 * Output:
 * torque_body[3] – torque in body frame (Nm)
 * return value – torque magnitude (Nm)
 */
static double compute_gravity_gradient_torque(const double pos_eci[3],
 const double dcm_bi[9],
 double torque_body[3])
{
 double r_mag = vec_norm(pos_eci, 3);
 if (r_mag < R_EARTH) r_mag = R_EARTH + 1.0;

 /* Nadir direction in ECI */
 double nadir_eci[3];
 for (int i = 0; i < 3; i++) nadir_eci[i] = -pos_eci[i] / r_mag;

 /* Transform to body frame: z_b = DCM^T * nadir_eci (DCM is body->inertial) */
 double z_b[3] = {0};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 z_b[i] += dcm_bi[j * 3 + i] * nadir_eci[j];

 /* Spacecraft inertia tensor (3x3, symmetric) */
 double J_sc[9] = {
 SC_IXX, SC_IXY, SC_IXZ,
 SC_IXY, SC_IYY, SC_IYZ,
 SC_IXZ, SC_IYZ, SC_IZZ
 };

 /* J * z_b (3x3 times 3x1) */
 double Jz[3] = {0};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 Jz[i] += J_sc[i * 3 + j] * z_b[j];

 /* T = (3 mu / r^3) * (z_b x Jz) */
 double coeff = 3.0 * MU_EARTH / (r_mag * r_mag * r_mag);
 double cross_zJz[3];
 vec_cross(z_b, Jz, cross_zJz);
 for (int i = 0; i < 3; i++)
 torque_body[i] = coeff * cross_zJz[i];

 /* J2 correction factor on the gravity gradient coefficient */
 /* Geocentric latitude: sin(lat) = z_eci / r */
 double sin_lat = pos_eci[2] / r_mag;
 double Re_r = R_EARTH / r_mag;
 double j2_fac = 1.0 + 1.5 * J2_EARTH * Re_r * Re_r *
 (1.0 - 5.0 * sin_lat * sin_lat);
 for (int i = 0; i < 3; i++)
 torque_body[i] *= j2_fac;

 return vec_norm(torque_body, 3);
}

/**
 * Compute the full J2-perturbed gravitational acceleration in ECI.
 *
 * ax = -(mu/r^3) x [1 + 1.5 J2 (R_e/r)^2 (1 - 5(z/r)^2)]
 * ay = -(mu/r^3) y [1 + 1.5 J2 (R_e/r)^2 (1 - 5(z/r)^2)]
 * az = -(mu/r^3) z [1 + 1.5 J2 (R_e/r)^2 (3 - 5(z/r)^2)]
 */
static void compute_j2_gravity_accel(const double pos_eci[3],
 double accel_eci[3])
{
 double r = vec_norm(pos_eci, 3);
 if (r < 1.0) r = 1.0;
 double r2 = r * r;
 double mu_r3 = MU_EARTH / (r2 * r);
 double Re_r2 = (R_EARTH * R_EARTH) / r2;
 double z_r2 = (pos_eci[2] * pos_eci[2]) / r2;

 double j2_xy = 1.5 * J2_EARTH * Re_r2 * (1.0 - 5.0 * z_r2);
 double j2_z = 1.5 * J2_EARTH * Re_r2 * (3.0 - 5.0 * z_r2);

 accel_eci[0] = -mu_r3 * pos_eci[0] * (1.0 + j2_xy);
 accel_eci[1] = -mu_r3 * pos_eci[1] * (1.0 + j2_xy);
 accel_eci[2] = -mu_r3 * pos_eci[2] * (1.0 + j2_z);
}

/**
 * Feed gravity-gradient disturbance torque into the EKF angular-velocity
 * state. delta_omega = J^{-1} T_gg dt (Euler's equation linearised).
 */
static void apply_gravity_gradient_to_ekf(double x[STATE_DIM],
 const double torque_body[3],
 double dt)
{
 double J_sc[9] = {
 SC_IXX, SC_IXY, SC_IXZ,
 SC_IXY, SC_IYY, SC_IYZ,
 SC_IXZ, SC_IYZ, SC_IZZ
 };
 double J_inv[9];
 if (mat_inv(J_sc, J_inv, 3) != 0) return;

 /* delta_omega = J^{-1} * T_gg * dt */
 double dw[3] = {0};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 dw[i] += J_inv[i * 3 + j] * torque_body[j] * dt;

 /* Accumulate into angular-velocity state [6..8] */
 for (int i = 0; i < 3; i++)
 x[6 + i] += dw[i];
}

/* ================================================================== */
/* MAIN */
/* ================================================================== */

int main(void)
{
 FLIGHT_LOG("[STR] init (3x3 tile)\n");
 FLIGHT_LOG("State dimension : %d\n", STATE_DIM);
 FLIGHT_LOG("Meas dimension : %d\n", MEAS_DIM);
 FLIGHT_LOG("Image size : %dx%d\n", IMAGE_SIZE, IMAGE_SIZE);
 FLIGHT_LOG("Feature dim : %d\n", FEATURE_DIM);
 FLIGHT_LOG("Iterations : %d\n", NUM_TIMESTEPS);

 unsigned int rng_seed = 42;

 /* ---------- Star catalog ---------- */
 double catalog[STAR_CATALOG_SIZE][3];
 initialize_star_catalog(catalog, STAR_CATALOG_SIZE);

 /* ---------- Initial attitude (quaternion) ---------- */
 double q_true[4] = {0.7071, 0.0, 0.0, 0.7071}; /* 90 deg about X */
 quat_normalize(q_true);

 /* ---------- Gyroscope measurement model ---------- */
 double gyro_bias_true[3] = {0.0002, -0.0001, 0.00015};
 double gyro_data[3];
 double angular_vel[3] = {0.001, 0.002, -0.001}; /* rad/s */

 /* ---------- EKF initialisation ---------- */
 double x[STATE_DIM];
 double P[STATE_DIM * STATE_DIM];
 double Q_proc[STATE_DIM * STATE_DIM];
 memset(x, 0, sizeof(x));
 memset(P, 0, sizeof(P));
 memset(Q_proc, 0, sizeof(Q_proc));

 /* Initial state: small attitude error, zero biases */
 x[0] = 0.01; x[1] = -0.005; x[2] = 0.008; /* MRP error */
 x[9] = 0.5; x[10] = -0.3; x[11] = 0.1; /* Calibration offsets */
 x[15] = 0.001; x[16] = -0.0005; x[17] = 0.0002; /* Thermal drift */

 for (int i = 0; i < STATE_DIM; i++) {
 P[i * STATE_DIM + i] = (i < 3) ? 0.01 : (i < 6) ? 1e-6 :
 (i < 9) ? 1e-4 : (i < 12) ? 1.0 :
 (i < 15) ? 1e-6 : 1e-4;
 Q_proc[i * STATE_DIM + i] = P[i * STATE_DIM + i] * 0.01;
 }

 /* Measurement matrix H (12×18) — observes all 6 state blocks:
 * Rows 0-2 : MRP attitude from QUEST → states 0-2 (MRP)
 * Rows 3-5 : Angular rate from gyro−bias → states 6-8 (rate)
 * Rows 6-8 : Centroid prediction residuals → states 9-14 (cal+align)
 * Rows 9-11 : Temporal attitude differencing → states 15-17 (thermal)
 *
 * This expanded measurement model addresses the CDR observability concern
 * by providing diverse observation types for all 18 EKF states. */
 double dt = 0.1;
 double H[MEAS_DIM * STATE_DIM];
 double R[MEAS_DIM * MEAS_DIM];
 memset(H, 0, sizeof(H));
 memset(R, 0, sizeof(R));

 /* Block 1: MRP direct observation (rows 0-2 → states 0-2) */
 for (int i = 0; i < 3; i++)
 H[i * STATE_DIM + i] = 1.0;

 /* Block 2: Angular rate observation (rows 3-5 → bias 3-5 + rate 6-8)
 * Measurement model: z_rate = gyro_raw = ω_true + bias_true + noise
 * Predicted: h(x) = x[6..8] + x[3..5] (rate + bias)
 * Jacobian: ∂h/∂(bias) = +I₃, ∂h/∂(rate) = +I₃ */
 for (int i = 0; i < 3; i++) {
 H[(3 + i) * STATE_DIM + (3 + i)] = 1.0; /* rate → bias coupling */
 H[(3 + i) * STATE_DIM + (6 + i)] = 1.0; /* rate → rate */
 }

 /* Block 3: Centroid residuals (rows 6-8 → cal 9-11, align 12-14)
 * Pixel sensitivity: f/σ_p ≈ 3333.3 px/rad for misalignment,
 * 1.0 px/px for calibration offsets, tan(FOV/4)/σ_p for focal length. */
 double f_over_p = FOCAL_LENGTH / PIXEL_SIZE; /* ≈3333.3 px/rad */
 double fov_rad = FOV_DEG * M_PI / 180.0;
 double cal_f_sens = tan(fov_rad / 4.0) / PIXEL_SIZE; /* ≈4.37 px/mm */

 H[6 * STATE_DIM + 9] = 1.0; /* centroid x ← cal_x offset */
 H[7 * STATE_DIM + 10] = 1.0; /* centroid y ← cal_y offset */
 H[8 * STATE_DIM + 11] = cal_f_sens; /* centroid r ← focal len err */
 H[6 * STATE_DIM + 13] = -f_over_p; /* centroid x ← align_θ pitch */
 H[7 * STATE_DIM + 12] = f_over_p; /* centroid y ← align_φ roll */
 H[8 * STATE_DIM + 14] = f_over_p * 0.1; /* centroid r ← align_ψ yaw */

 /* Block 4: Temporal differencing (rows 9-11 → thermal 15-17)
 * Sensitivity proportional to dt (attitude rate residual). */
 for (int i = 0; i < 3; i++)
 H[(9 + i) * STATE_DIM + (15 + i)] = dt;

 /* Measurement noise covariance R (12×12 diagonal) */
 R[0] = 1e-4; /* MRP x: arc-second class tracker (rad²) */
 R[1 * MEAS_DIM + 1] = 1e-4; /* MRP y */
 R[2 * MEAS_DIM + 2] = 1e-4; /* MRP z */
 R[3 * MEAS_DIM + 3] = 1e-6; /* rate x: (rad/s)² */
 R[4 * MEAS_DIM + 4] = 1e-6; /* rate y */
 R[5 * MEAS_DIM + 5] = 1e-6; /* rate z */
 R[6 * MEAS_DIM + 6] = 0.25; /* centroid resid x: sub-pixel (pixel²) */
 R[7 * MEAS_DIM + 7] = 0.25; /* centroid resid y */
 R[8 * MEAS_DIM + 8] = 0.25; /* centroid resid r */
 R[9 * MEAS_DIM + 9] = 1e-6; /* temporal diff x: numerical diff noise */
 R[10 * MEAS_DIM + 10] = 1e-6; /* temporal diff y */
 R[11 * MEAS_DIM + 11] = 1e-6; /* temporal diff z */

 /* ---------- Orbital elements (LEO sun-synchronous orbit) ---------- */
 OrbitalElements orbit;
 orbit.a = R_EARTH + 700000.0; /* 700 km altitude LEO */
 orbit.e = 0.001; /* Near-circular */
 orbit.inc = 98.2 * M_PI / 180.0; /* Sun-synchronous inclination */
 orbit.Omega = 45.0 * M_PI / 180.0; /* Initial RAAN */
 orbit.omega = 0.0; /* Argument of perigee */
 orbit.M = 0.0; /* Start at ascending node */
 double elapsed_time = 0.0; /* Mission elapsed time (s) */
 int star_visibility[STAR_CATALOG_SIZE];
 double pos_eci[3] = {0}, vel_eci[3] = {0};
 double gg_torque[3] = {0};
 double gg_mag = 0.0;
 int n_visible = STAR_CATALOG_SIZE;

 /* Temporal differencing state for thermal-drift observability */
 double z_mrp_prev[3] = {x[0], x[1], x[2]}; /* init to MRP estimate */

 /* Observability monitoring */
 double obs_metric[6] = {0};
 int n_obs_blocks = 0;

 /* ---------- Main processing loop ---------- */
 for (int iter = 0; iter < NUM_TIMESTEPS; iter++) {

 /* -- 1. Simulate gyro reading -- */
 for (int i = 0; i < 3; i++)
 gyro_data[i] = angular_vel[i] + gyro_bias_true[i]
 + 1e-4 * rand_gauss_star(&rng_seed);

 /* -- 2. Update true attitude -- */
 double omega_norm = vec_norm(angular_vel, 3);
 if (omega_norm > 1e-12) {
 double half_angle = omega_norm * dt / 2.0;
 double dq[4] = {
 cos(half_angle),
 sin(half_angle) * angular_vel[0] / omega_norm,
 sin(half_angle) * angular_vel[1] / omega_norm,
 sin(half_angle) * angular_vel[2] / omega_norm
 };
 double q_new[4];
 quat_mul(q_true, dq, q_new);
 memcpy(q_true, q_new, 4 * sizeof(double));
 quat_normalize(q_true);
 }

 /* -- 2a. Propagate orbit with J2 secular drift (OrbitalMecha) -- */
 propagate_orbit_j2(&orbit, dt);
 elapsed_time += dt;
 orbital_elements_to_eci(&orbit, pos_eci, vel_eci);

 /* -- 2b. Check star visibility (Earth/Sun/Moon exclusion) -- */
 n_visible = check_star_visibility(catalog, STAR_CATALOG_SIZE,
 pos_eci, elapsed_time,
 star_visibility);

 /* -- 3. Generate star image from true attitude -- */
 double dcm[9];
 quat_to_dcm(q_true, dcm);
 double image[IMAGE_SIZE][IMAGE_SIZE];
 generate_star_image(image, catalog, dcm, NUM_DETECTED_STARS);

 /* -- 4. Detect stars -- */
 double centroids[NUM_DETECTED_STARS][2];
 int n_det = detect_stars(image, centroids);

 /* -- 5. Extract features (trig-heavy) -- */
 double features[NUM_DETECTED_STARS][FEATURE_DIM];
 if (n_det > 0)
 extract_features(centroids, n_det, features);

 /* -- 6. Match stars -- */
 int matches[NUM_DETECTED_STARS];
 if (n_det > 0)
 match_stars(features, n_det, catalog, STAR_CATALOG_SIZE, matches);

 /* -- 7. QUEST attitude determination -- */
 double q_meas[4] = {1, 0, 0, 0};
 if (n_det >= 3)
 quest_attitude(centroids, n_det, catalog, matches, q_meas);

 /* -- 8. Convert measured quaternion to MRP for EKF measurement -- */
 /* Compute error quaternion: q_err = q_meas * q_est^{-1} */
 /* Approximate as MRP for small error */
 double z_meas[MEAS_DIM];
 z_meas[0] = q_meas[1] / (1.0 + q_meas[0]) + 0.001 * rand_gauss_star(&rng_seed);
 z_meas[1] = q_meas[2] / (1.0 + q_meas[0]) + 0.001 * rand_gauss_star(&rng_seed);
 z_meas[2] = q_meas[3] / (1.0 + q_meas[0]) + 0.001 * rand_gauss_star(&rng_seed);
 /* Angular rate measurement: raw gyro (bias modelled in H matrix) */
 z_meas[3] = gyro_data[0] + 1e-4 * rand_gauss_star(&rng_seed);
 z_meas[4] = gyro_data[1] + 1e-4 * rand_gauss_star(&rng_seed);
 z_meas[5] = gyro_data[2] + 1e-4 * rand_gauss_star(&rng_seed);

 /* -- 8b. Centroid prediction residuals (calibration/alignment) -- */
 double centroid_resid[3] = {0};
 if (n_det >= 2)
 compute_centroid_residuals(centroids, n_det, catalog, matches,
 dcm, x, centroid_resid);
 z_meas[6] = centroid_resid[0];
 z_meas[7] = centroid_resid[1];
 z_meas[8] = centroid_resid[2];

 /* -- 8c. Temporal attitude differencing (thermal-drift obs.) -- */
 double z_thermal[3] = {0};
 if (iter > 0)
 compute_thermal_drift_obs(z_meas, z_mrp_prev, &x[6], dt,
 z_thermal);
 z_meas[9] = z_thermal[0];
 z_meas[10] = z_thermal[1];
 z_meas[11] = z_thermal[2];

 /* Save current MRP for next epoch's temporal differencing */
 z_mrp_prev[0] = z_meas[0];
 z_mrp_prev[1] = z_meas[1];
 z_mrp_prev[2] = z_meas[2];

 /* -- 9. EKF prediction -- */
 double F[STATE_DIM * STATE_DIM];
 build_jacobian_F(x, gyro_data, dt, F);
 ekf_state_predict(x, gyro_data, dt);
 ekf_predict_covariance(P, F, Q_proc);

 /* -- 9a. Gravity gradient torque (GravityPoten) -- */
 gg_mag = compute_gravity_gradient_torque(pos_eci, dcm, gg_torque);

 /* -- 9b. J2 gravity acceleration — feeds position propagation -- */
 double grav_accel[3];
 compute_j2_gravity_accel(pos_eci, grav_accel);

 /* Removed: Keplerian propagation already accounts for gravity */
 // pos_eci[0] += 0.5 * grav_accel[0] * dt * dt;
 // pos_eci[1] += 0.5 * grav_accel[1] * dt * dt;
 // pos_eci[2] += 0.5 * grav_accel[2] * dt * dt;

 /* -- 9c. Feed gravity gradient disturbance into EKF state -- */
 apply_gravity_gradient_to_ekf(x, gg_torque, dt);

 /* -- 10. EKF update -- */
 ekf_update(x, P, z_meas, H, R);

 /* -- 10b. Observability monitoring (Fisher information per block) -- */
 n_obs_blocks = compute_observability_metric(H, R, P, obs_metric);

 /* -- 11. Status output -- */
 if (iter % 10 == 0) {
 double att_err = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]) * 180.0 / M_PI;
 double bias_err = sqrt(x[3]*x[3] + x[4]*x[4] + x[5]*x[5]);
 FLIGHT_LOG("Iter %3d | Stars=%2d | Att err=%.4f deg | Bias err=%.2e rad/s | "
 "P_trace=%.2e\n",
 iter, n_det, att_err, bias_err,
 P[0] + P[STATE_DIM + 1] + P[2 * STATE_DIM + 2]);
 FLIGHT_LOG(" | Visible=%3d/%d | GG torque=%.2e Nm | "
 "Alt=%.1f km\n",
 n_visible, STAR_CATALOG_SIZE, gg_mag,
 (vec_norm(pos_eci, 3) - R_EARTH) / 1000.0);
 FLIGHT_LOG(" | Obs=%d/6 | Info: MRP=%.1e Rate=%.1e "
 "Cal=%.1e Align=%.1e Therm=%.1e\n",
 n_obs_blocks, obs_metric[0], obs_metric[2],
 obs_metric[3], obs_metric[4], obs_metric[5]);
 }
 }

 /* ---------- Final summary ---------- */
 double final_att = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]) * 180.0 / M_PI;
 double final_bias = sqrt(x[3]*x[3] + x[4]*x[4] + x[5]*x[5]);
 FLIGHT_LOG("Final attitude error : %.6f deg\n", final_att);
 FLIGHT_LOG("Final gyro bias est : [%.2e, %.2e, %.2e] rad/s\n", x[3], x[4], x[5]);
 FLIGHT_LOG("Bias estimation err : %.2e rad/s\n", final_bias);
 FLIGHT_LOG("Calibration state : [%.4f, %.4f, %.4f]\n", x[9], x[10], x[11]);
 FLIGHT_LOG("Misalignment state : [%.2e, %.2e, %.2e] rad\n", x[12], x[13], x[14]);
 FLIGHT_LOG("Thermal drift : [%.2e, %.2e, %.2e]\n", x[15], x[16], x[17]);
 FLIGHT_LOG("Meas types : 4 (MRP, rate, centroid_residual, thermal_diff)\n");
 FLIGHT_LOG("Observability blocks : %d / 6 (MRP,Bias,Rate,Cal,Align,Therm)\n",
 n_obs_blocks);
 FLIGHT_LOG("Info metric : MRP=%.2e Bias=%.2e Rate=%.2e "
 "Cal=%.2e Align=%.2e Therm=%.2e\n",
 obs_metric[0], obs_metric[1], obs_metric[2],
 obs_metric[3], obs_metric[4], obs_metric[5]);
 FLIGHT_LOG("Orbital altitude : %.1f km\n",
 (vec_norm(pos_eci, 3) - R_EARTH) / 1000.0);
 FLIGHT_LOG("RAAN (J2 drifted) : %.4f deg\n", orbit.Omega * 180.0 / M_PI);
 FLIGHT_LOG("Arg perigee (J2) : %.4f deg\n", orbit.omega * 180.0 / M_PI);
 FLIGHT_LOG("Visible stars : %d / %d\n", n_visible, STAR_CATALOG_SIZE);
 FLIGHT_LOG("GG torque magnitude : %.4e Nm\n", gg_mag);
 return 0;
}
