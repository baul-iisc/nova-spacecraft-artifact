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
 * Onboard Cloud Detection Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard cloud detection for Earth observation image triage. This
 * workload processes multispectral imagery using spatial and spectral feature
 * extraction, image classification with threshold-based decision logic, and
 * histogram analysis. Applicable to Earth observation resource satellite (AWiFS, LISS-III),
 * Earth observation satellite series, and EOS (Earth Observation Satellite) missions where onboard
 * cloud screening reduces data downlink volume by discarding cloudy scenes.
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

/* ---------- Configuration ---------- */
#define STATE_DIM 9 /* EKF state: cloud_frac(3) + cloud_top_T(3) + optical_thick(3) */
#define NUM_BANDS 6 /* Spectral bands: B,G,R,NIR,SWIR,TIR */
#define IMG_ROWS 64 /* multispectral imaging tile (flight: 512×512) */
#define IMG_COLS 64 /* multispectral imaging tile (flight: 512×512) */
#define BLOCK_SIZE 3 /* Block size for spatial statistics */
#define FFT_LEN 64 /* DFT length for temporal analysis (power of 2) */
#define NUM_TIMESTEPS 25 /* Number of image acquisitions per orbit pass */
#define NUM_CLASSES 3 /* Cloud / clear / shadow */
#define ANOMALY_THRESH_NOM 3.0 /* Nominal spectral anomaly threshold (sigma) */

/* Solar / atmospheric constants used in generate_simulated_image */
/* (SOLAR_CONSTANT and R_EARTH not needed for Level-1B reflectance input) */

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
 double aug[9 * 18]; /* max 9x18 augmented */
 if (n > 9) return -1;
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

/* ------------------------------------------------------------------ */
/* Solar geometry (trigonometric-heavy) */
/* ------------------------------------------------------------------ */

/**
 * Compute solar zenith and azimuth angles for a given pixel.
 * Uses latitude, longitude, and fractional day of year.
 */
static void solar_geometry(double lat_rad, double lon_rad, double day_frac,
 double *zenith_rad, double *azimuth_rad)
{
 /* Solar declination (Spencer, 1971) */
 double gamma = 2.0 * M_PI * day_frac;
 double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma)
 - 0.006758 * cos(2.0 * gamma) + 0.000907 * sin(2.0 * gamma)
 - 0.002697 * cos(3.0 * gamma) + 0.00148 * sin(3.0 * gamma);

 /* Hour angle */
 double eot = 229.18 * (0.000075 + 0.001868 * cos(gamma)
 - 0.032077 * sin(gamma) - 0.014615 * cos(2.0 * gamma)
 - 0.04089 * sin(2.0 * gamma)); /* Equation of time (minutes) */
 double solar_noon = 12.0 - eot / 60.0 - lon_rad * 12.0 / M_PI;
 double hour_frac = fmod(day_frac * 24.0, 24.0);
 double ha = (hour_frac - solar_noon) * M_PI / 12.0;

 /* Zenith angle */
 double cos_zen = sin(lat_rad) * sin(decl) + cos(lat_rad) * cos(decl) * cos(ha);
 if (cos_zen > 1.0) cos_zen = 1.0;
 if (cos_zen < -1.0) cos_zen = -1.0;
 *zenith_rad = acos(cos_zen);

 /* Azimuth angle */
 double sin_zen = sin(*zenith_rad);
 if (sin_zen < 1e-10) {
 *azimuth_rad = 0.0;
 } else {
 double cos_az = (sin(decl) - cos_zen * sin(lat_rad)) / (sin_zen * cos(lat_rad));
 if (cos_az > 1.0) cos_az = 1.0;
 if (cos_az < -1.0) cos_az = -1.0;
 *azimuth_rad = acos(cos_az);
 if (ha > 0) *azimuth_rad = 2.0 * M_PI - *azimuth_rad;
 }
}

/* ------------------------------------------------------------------ */
/* Atmospheric correction (radiative transfer, trig-heavy) */
/* ------------------------------------------------------------------ */

/**
 * Simple 6S-like atmospheric correction for top-of-atmosphere (TOA)
 * reflectance to surface reflectance.
 *
 * Uses Beer-Lambert law with Rayleigh scattering:
 * rho_s = (rho_TOA - rho_path) / (T_down * T_up + S * rho_path)
 *
 * Band-dependent optical depths and Rayleigh phase function (trig).
 */
static void atmospheric_correction(const double toa_refl[NUM_BANDS],
 double sza_rad, double vza_rad,
 double raa_rad,
 double surface_refl[NUM_BANDS])
{
 /* Band-centre wavelengths (um) */
 static const double lambda[NUM_BANDS] = {0.49, 0.56, 0.665, 0.842, 1.61, 10.5};

 /* Rayleigh optical depth (approximate, decreases with lambda^4) */
 double tau_r[NUM_BANDS];
 for (int b = 0; b < NUM_BANDS; b++)
 tau_r[b] = 0.008569 / pow(lambda[b], 4) *
 (1.0 + 0.0113 / (lambda[b] * lambda[b]) +
 0.00013 / pow(lambda[b], 4));

 double mu_s = cos(sza_rad);
 double mu_v = cos(vza_rad);
 if (mu_s < 0.01) mu_s = 0.01;
 if (mu_v < 0.01) mu_v = 0.01;

 /* Rayleigh scattering phase function */
 double cos_scat = -cos(sza_rad) * cos(vza_rad)
 + sin(sza_rad) * sin(vza_rad) * cos(raa_rad);
 double P_ray = 0.75 * (1.0 + cos_scat * cos_scat);

 for (int b = 0; b < NUM_BANDS; b++) {
 double T_down = exp(-tau_r[b] / mu_s);
 double T_up = exp(-tau_r[b] / mu_v);

 /* Path radiance (single-scattering approximation) */
 double rho_path = tau_r[b] * P_ray / (4.0 * mu_s * mu_v);

 /* Spherical albedo */
 double S_alb = 0.0;
 for (int k = 1; k <= 3; k++)
 S_alb += pow(tau_r[b], k) / (2.0 * k + 1.0);

 /* Surface reflectance */
 double num = toa_refl[b] - rho_path;
 double den = T_down * T_up + S_alb * (toa_refl[b] - rho_path);
 surface_refl[b] = (den > 1e-10) ? num / den : toa_refl[b];
 if (surface_refl[b] < 0.0) surface_refl[b] = 0.0;
 if (surface_refl[b] > 1.0) surface_refl[b] = 1.0;
 }
}

/* ------------------------------------------------------------------ */
/* Spectral index computation */
/* ------------------------------------------------------------------ */

typedef struct {
 double ndvi; /* NDVI (Normalized Difference Vegetation Index) (NIR-Red)/(NIR+Red) */
 double ndsi; /* Normalized Difference Snow Index (Green-SWIR)/(Green+SWIR) */
 double bt; /* Brightness temperature (from TIR band) */
 double whiteness; /* Spectral flatness of visible bands */
} SpectralIndices;

static SpectralIndices compute_spectral_indices(const double refl[NUM_BANDS])
{
 SpectralIndices si;
 /* NDVI (Normalized Difference Vegetation Index) */
 double d1 = refl[3] + refl[2]; /* NIR + Red */
 si.ndvi = (d1 > 1e-10) ? (refl[3] - refl[2]) / d1 : 0.0;

 /* NDSI */
 double d2 = refl[1] + refl[4]; /* Green + SWIR */
 si.ndsi = (d2 > 1e-10) ? (refl[1] - refl[4]) / d2 : 0.0;

 /* Brightness temperature from TIR (Planck inversion) */
 double L = refl[5] * 10.0; /* Radiance proxy from TIR reflectance */
 double c1 = 3.7418e-16; /* 2*h*c^2 */
 double c2 = 1.4388e-2; /* h*c/k */
 double lam_tir = 10.5e-6; /* TIR wavelength (m) */
 if (L > 1e-10)
 si.bt = c2 / (lam_tir * log(c1 / (pow(lam_tir, 5) * L * 1e6) + 1.0));
 else
 si.bt = 200.0; /* Default cold */

 /* Whiteness: coefficient of variation of VNIR bands */
 double mean_vnir = (refl[0] + refl[1] + refl[2]) / 3.0;
 double var_vnir = 0.0;
 for (int b = 0; b < 3; b++) {
 double d = refl[b] - mean_vnir;
 var_vnir += d * d;
 }
 var_vnir /= 3.0;
 si.whiteness = (mean_vnir > 1e-10) ? sqrt(var_vnir) / mean_vnir : 1.0;

 return si;
}

/* ------------------------------------------------------------------ */
/* Spectral transform (6x6 band correlation matrix) */
/* ------------------------------------------------------------------ */

/**
 * Compute 6x6 inter-band correlation matrix from a 3x3 pixel neighbourhood.
 * Result is symmetric positive semi-definite.
 */
static void compute_spectral_covariance(const double pixels[9][NUM_BANDS],
 double cov[NUM_BANDS * NUM_BANDS])
{
 double means[NUM_BANDS] = {0};
 for (int p = 0; p < 9; p++)
 for (int b = 0; b < NUM_BANDS; b++)
 means[b] += pixels[p][b];
 for (int b = 0; b < NUM_BANDS; b++)
 means[b] /= 9.0;

 memset(cov, 0, NUM_BANDS * NUM_BANDS * sizeof(double));
 for (int p = 0; p < 9; p++)
 for (int i = 0; i < NUM_BANDS; i++)
 for (int j = 0; j < NUM_BANDS; j++)
 cov[i * NUM_BANDS + j] += (pixels[p][i] - means[i]) *
 (pixels[p][j] - means[j]);
 for (int i = 0; i < NUM_BANDS * NUM_BANDS; i++)
 cov[i] /= 8.0; /* Unbiased */
}

/**
 * Spectral whitening transform: multiply pixel vector by C^{-1}.
 * For Mahalanobis distance: d² = pᵀ C⁻¹ p = pᵀ · (C⁻¹ p).
 * Returns C⁻¹ p in transformed[], caller computes dot product with p.
 */
static void spectral_transform(const double cov[NUM_BANDS * NUM_BANDS],
 const double pixel[NUM_BANDS],
 double transformed[NUM_BANDS])
{
 double Cinv[NUM_BANDS * NUM_BANDS];
 if (mat_inv(cov, Cinv, NUM_BANDS) != 0) {
 /* Singular covariance — return identity transform */
 memcpy(transformed, pixel, NUM_BANDS * sizeof(double));
 return;
 }
 mat_mul(Cinv, pixel, transformed, NUM_BANDS, NUM_BANDS, 1);
}

/* ------------------------------------------------------------------ */
/* Spatial texture analysis (3x3 kernels) */
/* ------------------------------------------------------------------ */

static void apply_3x3_kernel(const double window[3][3],
 const double kernel[3][3],
 double *result)
{
 /*
 * 2-D convolution kernel response = Frobenius inner product of window
 * and kernel = trace(W^T K). We flatten both to 1×9 row vectors and
 * use mat_mul to form the 1×1 dot-product; this maps to a single
 * AME tile operation on the HW variant.
 */
 double W[9], K[9], dot[1];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 W[i * 3 + j] = window[i][j];
 K[i * 3 + j] = kernel[i][j];
 }
 /* dot = W(1x9) · K^T(9x1) = Frobenius inner product */
 mat_mul(W, K, dot, 1, 9, 1); /* K is already column-major layout */
 *result = dot[0];
}

typedef struct {
 double sobel_mag;
 double laplacian;
 double local_std;
} TextureFeatures;

static TextureFeatures compute_texture(const double ndvi_image[IMG_ROWS][IMG_COLS],
 int r, int c)
{
 TextureFeatures tf = {0, 0, 0};
 double window[3][3];

 /* Extract 3x3 window with boundary handling */
 for (int di = -1; di <= 1; di++)
 for (int dj = -1; dj <= 1; dj++) {
 int ri = r + di, cj = c + dj;
 if (ri < 0) ri = 0;
 if (ri >= IMG_ROWS) ri = IMG_ROWS - 1;
 if (cj < 0) cj = 0;
 if (cj >= IMG_COLS) cj = IMG_COLS - 1;
 window[di + 1][dj + 1] = ndvi_image[ri][cj];
 }

 /* Sobel operators */
 double sobel_x[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};
 double sobel_y[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};
 double gx, gy;
 apply_3x3_kernel(window, sobel_x, &gx);
 apply_3x3_kernel(window, sobel_y, &gy);
 tf.sobel_mag = sqrt(gx * gx + gy * gy);

 /* Laplacian */
 double laplacian_k[3][3] = {{0,1,0},{1,-4,1},{0,1,0}};
 apply_3x3_kernel(window, laplacian_k, &tf.laplacian);
 tf.laplacian = fabs(tf.laplacian);

 /* Local standard deviation */
 double mean = 0, var = 0;
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 mean += window[i][j];
 mean /= 9.0;
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 double d = window[i][j] - mean;
 var += d * d;
 }
 tf.local_std = sqrt(var / 9.0);

 return tf;
}

/* ------------------------------------------------------------------ */
/* DFT for cloud-cover temporal analysis */
/* ------------------------------------------------------------------ */

static void compute_dft_magnitude(const double *x, double *mag, int N)
{
 for (int k = 0; k <= N / 2; k++) {
 double re = 0.0, im = 0.0;
 for (int n = 0; n < N; n++) {
 double angle = 2.0 * M_PI * k * n / N;
 re += x[n] * cos(angle);
 im -= x[n] * sin(angle);
 }
 mag[k] = sqrt(re * re + im * im) / N;
 }
}

/* ------------------------------------------------------------------ */
/* EKF for cloud state tracking */
/* ------------------------------------------------------------------ */

/**
 * State vector (9 elements):
 * [0..2] Cloud fraction (3 zones: nadir, port, starboard)
 * [3..5] Cloud-top temperature (K) per zone
 * [6..8] Cloud optical thickness per zone
 */

static void cloud_state_predict(double x[STATE_DIM], double dt)
{
 /* Cloud fraction: slow advective change */
 for (int z = 0; z < 3; z++) {
 x[z] += 0.001 * sin(0.01 * dt * (z + 1));
 if (x[z] > 1.0) x[z] = 1.0;
 if (x[z] < 0.0) x[z] = 0.0;
 }

 /* Cloud-top temperature: radiative cooling */
 for (int z = 0; z < 3; z++)
 x[3 + z] -= 0.01 * dt;

 /* Optical thickness: slow evolution */
 for (int z = 0; z < 3; z++)
 x[6 + z] *= exp(-0.001 * dt);
}

static void build_cloud_jacobian(const double x[STATE_DIM], double dt,
 double F[STATE_DIM * STATE_DIM])
{
 (void)x;
 memset(F, 0, STATE_DIM * STATE_DIM * sizeof(double));
 for (int i = 0; i < STATE_DIM; i++)
 F[i * STATE_DIM + i] = 1.0;

 /* Cloud fraction partials */
 for (int z = 0; z < 3; z++)
 F[z * STATE_DIM + z] = 1.0;

 /* Optical thickness */
 for (int z = 0; z < 3; z++)
 F[(6 + z) * STATE_DIM + (6 + z)] = exp(-0.001 * dt);
}

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
/* Cloud classification */
/* ------------------------------------------------------------------ */

typedef enum { CLEAR = 0, CLOUD = 1, SHADOW = 2 } PixelClass;

static PixelClass classify_pixel(const SpectralIndices *si,
 const TextureFeatures *tf)
{
 /* Multi-threshold classifier */
 /* Cloud: NDVI > 0.2 selects vegetated pixels, high whiteness (<0.3), warm BT (>250K) */
 if (si->ndvi > 0.2 && si->whiteness < 0.3 && si->bt > 250.0)
 return CLOUD;

 /* Snow: high NDSI (>0.4) and cold (<270K) */
 if (si->ndsi > 0.4 && si->bt < 270.0)
 return CLEAR; /* Snow classified as clear for cloud detection */

 /* Shadow: low brightness, strong edges */
 if (si->ndvi < -0.1 && tf->sobel_mag > 0.5)
 return SHADOW;

 return CLEAR;
}

/* ------------------------------------------------------------------ */
/* Confusion matrix (3x3) */
/* ------------------------------------------------------------------ */

static void update_confusion_matrix(int cm[3][3], PixelClass predicted,
 PixelClass actual)
{
 cm[actual][predicted]++;
}

static void print_confusion_matrix(const int cm[NUM_CLASSES][NUM_CLASSES])
{
 const char *labels[NUM_CLASSES] = {"Clear", "Cloud", "Shadow"};
 FLIGHT_LOG("[CLD] confusion_matrix\n");
 FLIGHT_LOG(" Pred:Clear Pred:Cloud Pred:Shadow\n");
 for (int i = 0; i < NUM_CLASSES; i++) {
 FLIGHT_LOG("Act:%-7s", labels[i]);
 for (int j = 0; j < NUM_CLASSES; j++)
 FLIGHT_LOG(" %5d ", cm[i][j]);
 FLIGHT_LOG("\n");
 }
 int correct = cm[0][0] + cm[1][1] + cm[2][2];
 int total = 0;
 for (int i = 0; i < NUM_CLASSES; i++)
 for (int j = 0; j < NUM_CLASSES; j++)
 total += cm[i][j];
 FLIGHT_LOG("Accuracy: %.2f%%\n", 100.0 * correct / (total > 0 ? total : 1));
}

/* ------------------------------------------------------------------ */
/* Simulated image generation */
/* ------------------------------------------------------------------ */

static void generate_simulated_image(double image[IMG_ROWS][IMG_COLS][NUM_BANDS],
 PixelClass truth[IMG_ROWS][IMG_COLS],
 int timestep, unsigned int *seed)
{
 double cloud_center_r = IMG_ROWS / 2.0 + 3.0 * sin(0.2 * timestep);
 double cloud_center_c = IMG_COLS / 2.0 + 3.0 * cos(0.15 * timestep);
 double cloud_radius = 6.0 + 2.0 * sin(0.1 * timestep);

 for (int r = 0; r < IMG_ROWS; r++) {
 for (int c = 0; c < IMG_COLS; c++) {
 double dr = r - cloud_center_r;
 double dc = c - cloud_center_c;
 double dist = sqrt(dr * dr + dc * dc);

 *seed = (*seed) * 1103515245 + 12345;
 double noise = ((*seed >> 16) & 0x7FFF) / 32768.0 * 0.05;

 if (dist < cloud_radius) {
 /* Cloud pixel */
 truth[r][c] = CLOUD;
 image[r][c][0] = 0.75 + noise; /* Blue */
 image[r][c][1] = 0.73 + noise; /* Green */
 image[r][c][2] = 0.72 + noise; /* Red */
 image[r][c][3] = 0.70 + noise; /* NIR */
 image[r][c][4] = 0.30 + noise; /* SWIR */
 image[r][c][5] = 0.40 + noise; /* TIR (warm cloud) */
 } else if (dist < cloud_radius + 3.0 && dc > 0) {
 /* Shadow pixel */
 truth[r][c] = SHADOW;
 image[r][c][0] = 0.05 + noise;
 image[r][c][1] = 0.08 + noise;
 image[r][c][2] = 0.06 + noise;
 image[r][c][3] = 0.15 + noise;
 image[r][c][4] = 0.10 + noise;
 image[r][c][5] = 0.20 + noise;
 } else {
 /* Clear land pixel */
 truth[r][c] = CLEAR;
 image[r][c][0] = 0.10 + noise;
 image[r][c][1] = 0.20 + noise;
 image[r][c][2] = 0.30 + noise;
 image[r][c][3] = 0.45 + noise;
 image[r][c][4] = 0.25 + noise;
 image[r][c][5] = 0.35 + noise;
 }
 }
 }
}

/* ================================================================== */
/* MAIN */
/* ================================================================== */

int main(void)
{
 FLIGHT_LOG("[CLD] init\n");
 FLIGHT_LOG("State dimension : %d\n", STATE_DIM);
 FLIGHT_LOG("Spectral bands : %d\n", NUM_BANDS);
 FLIGHT_LOG("Image size : %dx%d\n", IMG_ROWS, IMG_COLS);
 FLIGHT_LOG("Timesteps : %d\n", NUM_TIMESTEPS);

 unsigned int rng_seed = 2718;

 /* ---------- EKF initialisation ---------- */
 double x[STATE_DIM];
 double P[STATE_DIM * STATE_DIM];
 double Q_proc[STATE_DIM * STATE_DIM];
 memset(P, 0, sizeof(P));
 memset(Q_proc, 0, sizeof(Q_proc));

 /* Initial state: cloud fraction, cloud-top temp, optical thickness */
 x[0] = 0.3; x[1] = 0.25; x[2] = 0.35; /* Cloud fraction per zone */
 x[3] = 260.0; x[4] = 257.5; x[5] = 263.0; /* Cloud-top temp (K) */
 x[6] = 5.2; x[7] = 4.3; x[8] = 5.8; /* Optical thickness */

 for (int i = 0; i < STATE_DIM; i++) {
 P[i * STATE_DIM + i] = (i < 3) ? 0.01 : (i < 6) ? 10.0 : 1.0;
 Q_proc[i * STATE_DIM + i] = P[i * STATE_DIM + i] * 0.01;
 }

 /* Measurement: observe cloud fraction(3) + cloud-top temp(3) = 6 */
 int meas_dim = 6;
 double H_meas[6 * STATE_DIM];
 double R_meas[36];
 memset(H_meas, 0, sizeof(H_meas));
 memset(R_meas, 0, sizeof(R_meas));
 for (int i = 0; i < 6; i++) {
 H_meas[i * STATE_DIM + i] = 1.0;
 R_meas[i * meas_dim + i] = (i < 3) ? 0.001 : 5.0;
 }

 /* Cloud-cover history for DFT */
 double coverage_history[NUM_TIMESTEPS];
 int total_cm[NUM_CLASSES][NUM_CLASSES] = {{0}};

 /* Adaptive anomaly threshold — DFT-derived */
 double anomaly_thresh = ANOMALY_THRESH_NOM;

 /* ---------- Main loop ---------- */
 for (int t = 0; t < NUM_TIMESTEPS; t++) {
 /* -- 1. Generate simulated multi-spectral image -- */
 double image[IMG_ROWS][IMG_COLS][NUM_BANDS];
 PixelClass truth[IMG_ROWS][IMG_COLS];
 generate_simulated_image(image, truth, t, &rng_seed);

 /* -- 2. Solar geometry -- */
 double lat = 30.0 * M_PI / 180.0;
 double lon = 10.0 * M_PI / 180.0;
 double day_frac = 0.5 + t * 0.01; /* Step through days */
 double sza, saa;
 solar_geometry(lat, lon, day_frac, &sza, &saa);
 double vza = 0.1; /* Near-nadir viewing */
 double raa = saa;

 /* -- 3. Per-pixel processing -- */
 double ndvi_img[IMG_ROWS][IMG_COLS];
 PixelClass predicted[IMG_ROWS][IMG_COLS];
 int cloud_count = 0;

 /* Pass 1: compute spectral indices for all pixels (needed by texture) */
 double all_surface[IMG_ROWS][IMG_COLS][NUM_BANDS];
 SpectralIndices all_si[IMG_ROWS][IMG_COLS];
 for (int r = 0; r < IMG_ROWS; r++) {
 for (int c = 0; c < IMG_COLS; c++) {
 atmospheric_correction(image[r][c], sza, vza, raa, all_surface[r][c]);
 all_si[r][c] = compute_spectral_indices(all_surface[r][c]);
 ndvi_img[r][c] = all_si[r][c].ndvi;
 }
 }

 /* Pass 2: classification with texture + spectral anomaly */
 for (int r = 0; r < IMG_ROWS; r++) {
 for (int c = 0; c < IMG_COLS; c++) {
 /* 3a. Texture features (needs fully populated ndvi_img) */
 TextureFeatures tf = {0, 0, 0};
 if (r >= 1 && r < IMG_ROWS - 1 && c >= 1 && c < IMG_COLS - 1)
 tf = compute_texture(ndvi_img, r, c);

 /* 3b. Threshold-based classification */
 predicted[r][c] = classify_pixel(&all_si[r][c], &tf);

 /* 3c. Spectral covariance anomaly (6x6 matrix) — thin cirrus override */
 if (r >= 1 && r < IMG_ROWS - 1 && c >= 1 && c < IMG_COLS - 1 &&
 (r % BLOCK_SIZE == 0) && (c % BLOCK_SIZE == 0)) {
 double neighbourhood[9][NUM_BANDS];
 int pidx = 0;
 for (int di = -1; di <= 1; di++)
 for (int dj = -1; dj <= 1; dj++)
 memcpy(neighbourhood[pidx++], all_surface[r+di][c+dj],
 NUM_BANDS * sizeof(double));

 double cov[NUM_BANDS * NUM_BANDS];
 compute_spectral_covariance(neighbourhood, cov);

 double transformed[NUM_BANDS];
 spectral_transform(cov, all_surface[r][c], transformed);

 /* Correct Mahalanobis distance: d² = pᵀ C⁻¹ p */
 double anom_score = 0.0;
 for (int b = 0; b < NUM_BANDS; b++)
 anom_score += all_surface[r][c][b] * transformed[b];
 anom_score = sqrt(fabs(anom_score));

 /* Thin cirrus detection: anomalous + cold → override to CLOUD */
 if (anom_score > anomaly_thresh && all_si[r][c].bt < 260.0)
 predicted[r][c] = CLOUD;
 }

 if (predicted[r][c] == CLOUD) cloud_count++;
 update_confusion_matrix(total_cm, predicted[r][c], truth[r][c]);
 }
 }

 double coverage = (double)cloud_count / (IMG_ROWS * IMG_COLS);
 coverage_history[t] = coverage;

 /* -- 4. EKF with measured cloud fraction -- */
 double z_meas[6];
 int zone_rows = IMG_ROWS / 3;
 /* Zone fractions (split image into 3 horizontal strips) */
 for (int z = 0; z < 3; z++) {
 int zone_cloud = 0;
 int zone_start = z * zone_rows;
 int zone_end = (z < 2) ? zone_start + zone_rows : IMG_ROWS;
 for (int r = zone_start; r < zone_end; r++)
 for (int c = 0; c < IMG_COLS; c++)
 if (predicted[r][c] == CLOUD) zone_cloud++;
 z_meas[z] = (double)zone_cloud / ((zone_end - zone_start) * IMG_COLS);
 }
 /* Zone cloud-top temperatures (from mean TIR in cloudy pixels) */
 for (int z = 0; z < 3; z++) {
 double sum_bt = 0;
 int count = 0;
 int zone_start = z * zone_rows;
 int zone_end = (z < 2) ? zone_start + zone_rows : IMG_ROWS;
 for (int r = zone_start; r < zone_end; r++)
 for (int c = 0; c < IMG_COLS; c++)
 if (predicted[r][c] == CLOUD) {
 sum_bt += all_si[r][c].bt;
 count++;
 }
 z_meas[3 + z] = (count > 0) ? sum_bt / count : 220.0;
 }

 /* EKF predict — dt matches inter-frame interval (864 s ≈ 14.4 min) */
 double dt_frame = 864.0; /* 0.01 day_frac × 86400 s */
 double F[STATE_DIM * STATE_DIM];
 build_cloud_jacobian(x, dt_frame, F);
 cloud_state_predict(x, dt_frame);
 ekf_predict_covariance(P, F, Q_proc);

 /* EKF update */
 ekf_update(x, P, z_meas, H_meas, R_meas, meas_dim);

 /* -- 5. DFT analysis — feed into adaptive anomaly threshold -- */
 if (t >= FFT_LEN && (t % 4 == 0)) {
 double mag[FFT_LEN / 2 + 1];
 compute_dft_magnitude(coverage_history + (t + 1 - FFT_LEN), mag, FFT_LEN);

 /* Find dominant non-DC temporal frequency */
 int dom_bin = 1;
 for (int k = 2; k <= FFT_LEN / 2; k++)
 if (mag[k] > mag[dom_bin]) dom_bin = k;

 /* Adapt anomaly threshold: high temporal variability → tighter threshold */
 double spectral_energy = 0.0;
 for (int k = 1; k <= FFT_LEN / 2; k++)
 spectral_energy += mag[k] * mag[k];
 anomaly_thresh = ANOMALY_THRESH_NOM - 0.5 * spectral_energy;
 if (anomaly_thresh < 1.5) anomaly_thresh = 1.5;
 if (anomaly_thresh > 5.0) anomaly_thresh = 5.0;

 if (t % 8 == 0)
 FLIGHT_LOG(" DFT: DC=%.4f dom_bin=%d (mag=%.4f) anom_thresh=%.2f\n",
 mag[0], dom_bin, mag[dom_bin], anomaly_thresh);
 }

 /* -- 6. Status output -- */
 if (t % 5 == 0) {
 FLIGHT_LOG("t=%2d | SZA=%.1f° | Coverage=%.1f%% | CF=[%.2f,%.2f,%.2f] | "
 "CTT=[%.0f,%.0f,%.0f]K\n",
 t, sza * 180.0 / M_PI, coverage * 100.0,
 x[0], x[1], x[2], x[3], x[4], x[5]);
 }
 }

 /* ---------- Final summary ---------- */
 FLIGHT_LOG("Final cloud fraction: [%.3f, %.3f, %.3f]\n", x[0], x[1], x[2]);
 FLIGHT_LOG("Final cloud-top temp: [%.1f, %.1f, %.1f] K\n", x[3], x[4], x[5]);
 FLIGHT_LOG("Final optical depth : [%.2f, %.2f, %.2f]\n", x[6], x[7], x[8]);
 FLIGHT_LOG("Final anomaly thresh: %.2f\n", anomaly_thresh);

 print_confusion_matrix(total_cm);

 /* Final DFT */
 if (NUM_TIMESTEPS >= FFT_LEN) {
 double mag[FFT_LEN / 2 + 1];
 compute_dft_magnitude(coverage_history + (NUM_TIMESTEPS - FFT_LEN),
 mag, FFT_LEN);
 FLIGHT_LOG("Coverage DFT (DC + first 3): %.4f %.4f %.4f %.4f\n",
 mag[0], mag[1], mag[2], mag[3]);
 }

 return 0;
}
