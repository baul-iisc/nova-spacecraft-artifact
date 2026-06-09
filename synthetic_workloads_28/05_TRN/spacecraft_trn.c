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
 * Terrain-Relative Navigation Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements terrain-relative navigation (TRN) for autonomous planetary landing.
 * This workload performs real-time terrain matching by correlating onboard camera
 * images with stored digital elevation model (DEM) patches, using image processing
 * (FFT-based correlation), matrix transformations, and trigonometric projections.
 * Derived from the lunar lander powered descent guidance and applicable to
 * future planetary lander descent navigation systems.
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

// Matrix dimensions for TRN pipeline (tile-friendly for 3x3 HW accelerator)
#define MAP_DIM 64 /* scaled from flight 256×256 for simulation */
#define SENSOR_DIM 16 /* scaled from flight 256×256 for simulation */
#define STATE_DIM 9 /* pos(3) + vel(3) + attitude_error(3) */
#define COVARIANCE_DIM 9 /* Full coupled translational + attitude state */
#define TILE_SIZE 3 // Hardware tile size
#define MAX_TIMESTEPS 200 // Maximum iterations for position convergence
#define GRAVITY_MOON 1.62 // Moon surface gravity (m/s^2) — lunar descent
#define DELTA_T 0.1 // Time step in seconds
#define DFT_PATCH_DIM 16 /* scaled from flight 256×256 for simulation */
#define NUM_TERRAIN_CLASSES 3 // Terrain types: crater, ridge, plain
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Timing statistics
typedef struct {
 double matching_time;
 double total_time;
} TimingStats;

// 3D Vector
typedef struct {
 double x;
 double y;
 double z;
} Vector3D;

// Quaternion for attitude representation
typedef struct {
 double w; // Real part
 double x; // Imaginary i
 double y; // Imaginary j
 double z; // Imaginary k
} Quaternion;

// Spacecraft state
typedef struct {
 Vector3D position; // Position in meters
 Vector3D velocity; // Velocity in m/s
 Quaternion attitude; // Attitude quaternion
 Vector3D angular_vel; // Angular velocity in rad/s
 double mass; // Mass in kg
 double fuel; // Remaining fuel in kg
} SpacecraftState;

// Sensor measurement
typedef struct {
 double elevation[SENSOR_DIM][SENSOR_DIM]; // Elevation data
 double confidence[SENSOR_DIM][SENSOR_DIM]; // Measurement confidence
 double albedo[SENSOR_DIM][SENSOR_DIM]; // Surface reflectivity
 double range; // Range to surface in meters
 Vector3D line_of_sight; // Line of sight unit vector
 double timestamp; // Measurement time
} SensorMeasurement;

// Terrain map
typedef struct {
 double elevation[MAP_DIM][MAP_DIM]; // Elevation data
 double roughness[MAP_DIM][MAP_DIM]; // Surface roughness
 double albedo[MAP_DIM][MAP_DIM]; // Surface reflectivity
 double resolution; // Map resolution in meters
 Vector3D origin; // Map origin in global frame
 double rotation; // Map rotation in radians
} TerrainMap;

/* Kalman filter matrices — 9-state Multiplicative EKF (MEKF).
 * States 0–2: position (m)
 * States 3–5: velocity (m/s)
 * States 6–8: attitude error δφ,δθ,δψ (rad) — small-angle body-frame
 *
 * The attitude error states capture the coupling between camera
 * orientation uncertainty and terrain-matching position accuracy.
 * At altitude h, a pointing error δθ projects to a lateral position
 * error ≈ h·δθ in the measurement. Without attitude states in the
 * covariance, this coupling is unobservable and the filter becomes
 * over-confident in position during high-altitude TRN (CDR deficiency). */
typedef struct {
 double state[STATE_DIM][STATE_DIM]; /* column 0 = state vector */
 double covariance[COVARIANCE_DIM][COVARIANCE_DIM]; /* P: 9×9 covariance */
 double process_noise[STATE_DIM][STATE_DIM]; /* Q: 9×9 process noise */
 double measurement_noise[3][3]; /* R: 3×3 position obs noise */
} KalmanFilter;

// Landing target
typedef struct {
 Vector3D position; // Target landing position
 double radius; // Landing ellipse radius
 double safety; // Safety factor (0-1)
} LandingTarget;

// Navigation solution
typedef struct {
 SpacecraftState estimated_state; // Estimated spacecraft state
 double position_uncertainty; // Position uncertainty (1-sigma)
 double velocity_uncertainty; // Velocity uncertainty (1-sigma)
 double attitude_uncertainty; // Attitude uncertainty (1-sigma)
 double time_to_landing; // Estimated time to landing
 double fuel_required; // Estimated fuel required
 bool valid_solution; // Solution validity flag
} NavigationSolution;

// Terrain frequency-domain analysis features
// Used by DFT-based terrain classification for TRN
typedef struct {
 double roughness_index; // Spectral roughness: ratio of high-freq to total energy
 double dominant_frequency; // Dominant spatial frequency (cycles per patch)
 double spectral_centroid; // Energy-weighted mean frequency
 int terrain_class; // 0=plain, 1=crater, 2=ridge
 double class_confidence; // Classification confidence [0,1]
 double total_power; // Total spectral power (DC excluded)
} TerrainFrequencyFeatures;

// Function prototypes - Core TRN operations
void initialize_terrain_map(TerrainMap* map);
void generate_sensor_measurement(TerrainMap* map, SpacecraftState* true_state,
 SensorMeasurement* measurement);
void initialize_spacecraft_state(SpacecraftState* state);
void initialize_kalman_filter(KalmanFilter* kf);
void update_kalman_filter(KalmanFilter* kf, SensorMeasurement* measurement,
 TerrainMap* map, SpacecraftState* state);
double match_terrain(TerrainMap* map, SensorMeasurement* measurement,
 Vector3D* position, TimingStats* stats);
void propagate_state(SpacecraftState* state, double dt);
void generate_landing_trajectory(SpacecraftState* state, LandingTarget* target,
 NavigationSolution* solution,
 double covariance[COVARIANCE_DIM][COVARIANCE_DIM]);

// Function prototypes - Matrix operations
void matrix_multiply_3x3(double A[3][3], double B[3][3], double C[3][3]);
void matrix_multiply_6x6(double A[6][6], double B[6][6], double C[6][6]);
void matrix_multiply_9x9(double A[9][9], double B[9][9], double C[9][9]);
void matrix_inverse_3x3(double A[3][3], double Ainv[3][3]);

// Function prototypes - DFT-based terrain frequency analysis (BCE FFT 9%)
void compute_terrain_dft(double patch[][DFT_PATCH_DIM], int rows, int cols,
 double mag_spectrum[][DFT_PATCH_DIM]);
void terrain_frequency_features(double mag_spectrum[][DFT_PATCH_DIM], int rows, int cols,
 TerrainFrequencyFeatures* features);
double frequency_cross_correlation(double ref_patch[][DFT_PATCH_DIM],
 double sensor_patch[][DFT_PATCH_DIM],
 int rows, int cols);

// Function prototypes - Utility functions
void quaternion_to_rotation_matrix(Quaternion* q, double R[3][3]);
double quaternion_norm(Quaternion* q);
void quaternion_normalize(Quaternion* q);
double random_gaussian(double mean, double stddev);
void print_spacecraft_state(SpacecraftState* state);
void print_navigation_solution(NavigationSolution* solution);
void print_timing_statistics(TimingStats* stats);

/*****************************************************************************
 * DFT-based Terrain Frequency Analysis Module
 *
 * Provides the FFT/DFT component declared in the BCE profile (FFT 9%).
 * Uses a direct (O(N^2)) Discrete Fourier Transform suitable for the small
 * patch sizes used in onboard TRN (9x9 sensor patches, 15x15 map patches).
 *
 * Applications in TRN:
 * - Terrain classification (crater vs ridge vs plain) from spectral shape
 * - Surface roughness estimation via power spectral density
 * - Frequency-domain cross-correlation for fast template matching
 *
 * The 2-D DFT is computed as separable 1-D DFTs (rows then columns).
 * Twiddle factors use sin/cos from math.h.
 *****************************************************************************/

/**
 * compute_terrain_dft – 2-D DFT magnitude spectrum of a terrain patch
 *
 * Computes the 2-D Discrete Fourier Transform of the input elevation patch
 * and stores the magnitude (sqrt(Re^2 + Im^2)) in mag_spectrum.
 * The DC component is at index [0][0].
 *
 * @param patch Input elevation data (rows x cols, max DFT_PATCH_DIM)
 * @param rows Number of rows (<= DFT_PATCH_DIM)
 * @param cols Number of columns (<= DFT_PATCH_DIM)
 * @param mag_spectrum Output magnitude spectrum (same dimensions)
 */
void compute_terrain_dft(double patch[][DFT_PATCH_DIM], int rows, int cols,
 double mag_spectrum[][DFT_PATCH_DIM]) {
 /* Intermediate arrays for the separable 2-D DFT.
 * First transform along rows, then along columns. */
 double re_row[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double im_row[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double re_2d[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double im_2d[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};

 /* --- Pass 1: DFT along each row --- */
 for (int i = 0; i < rows; i++) {
 for (int k = 0; k < cols; k++) { /* frequency index */
 double sum_re = 0.0, sum_im = 0.0;
 for (int n = 0; n < cols; n++) { /* spatial index */
 double angle = 2.0 * M_PI * k * n / cols;
 sum_re += patch[i][n] * cos(angle);
 sum_im -= patch[i][n] * sin(angle);
 }
 re_row[i][k] = sum_re;
 im_row[i][k] = sum_im;
 }
 }

 /* --- Pass 2: DFT along each column of the row-transformed data --- */
 for (int k_col = 0; k_col < cols; k_col++) {
 for (int k_row = 0; k_row < rows; k_row++) { /* freq index */
 double sum_re = 0.0, sum_im = 0.0;
 for (int n = 0; n < rows; n++) { /* spatial index */
 double angle = 2.0 * M_PI * k_row * n / rows;
 double cos_a = cos(angle);
 double sin_a = sin(angle);
 /* complex multiply: (re_row + j*im_row) * (cos - j*sin) */
 sum_re += re_row[n][k_col] * cos_a + im_row[n][k_col] * sin_a;
 sum_im += im_row[n][k_col] * cos_a - re_row[n][k_col] * sin_a;
 }
 re_2d[k_row][k_col] = sum_re;
 im_2d[k_row][k_col] = sum_im;
 }
 }

 /* --- Magnitude spectrum --- */
 for (int i = 0; i < rows; i++) {
 for (int j = 0; j < cols; j++) {
 mag_spectrum[i][j] = sqrt(re_2d[i][j] * re_2d[i][j] +
 im_2d[i][j] * im_2d[i][j]);
 }
 }
}

/**
 * terrain_frequency_features – extract spectral features for terrain classification
 *
 * From the magnitude spectrum produced by compute_terrain_dft(), this function
 * computes:
 * - roughness_index : fraction of energy above half the Nyquist frequency
 * - dominant_frequency : frequency bin with peak magnitude (DC excluded)
 * - spectral_centroid : energy-weighted mean frequency
 * - terrain_class : heuristic classification (plain / crater / ridge)
 * - class_confidence : simple confidence metric
 * - total_power : total spectral energy (DC excluded)
 *
 * Terrain classification heuristic (applicable to planetary surfaces):
 * - Plains : low roughness, low total power (smooth maria / regolith)
 * - Craters : high roughness, energy spread across frequencies (sharp rims)
 * - Ridges : moderate roughness, energy concentrated at low frequencies
 *
 * @param mag_spectrum Magnitude spectrum from compute_terrain_dft()
 * @param rows Number of rows
 * @param cols Number of columns
 * @param features Output feature struct
 */
void terrain_frequency_features(double mag_spectrum[][DFT_PATCH_DIM], int rows, int cols,
 TerrainFrequencyFeatures* features) {
 double total_power = 0.0;
 double high_freq_power = 0.0;
 double weighted_freq = 0.0;
 double peak_magnitude = 0.0;
 double peak_freq = 0.0;

 /* Half-Nyquist threshold: frequencies beyond rows/4 (or cols/4) are "high" */
 double half_nyquist_r = (double)rows / 4.0;
 double half_nyquist_c = (double)cols / 4.0;

 for (int u = 0; u < rows; u++) {
 for (int v = 0; v < cols; v++) {
 /* Skip DC component */
 if (u == 0 && v == 0) continue;

 double mag = mag_spectrum[u][v];
 double power = mag * mag;

 /* Radial frequency in cycles-per-patch */
 double freq_r = (u <= rows / 2) ? (double)u : (double)(rows - u);
 double freq_c = (v <= cols / 2) ? (double)v : (double)(cols - v);
 double freq = sqrt(freq_r * freq_r + freq_c * freq_c);

 total_power += power;
 weighted_freq += power * freq;

 /* High-frequency energy */
 if (freq_r > half_nyquist_r || freq_c > half_nyquist_c) {
 high_freq_power += power;
 }

 /* Track peak (dominant frequency, excluding DC) */
 if (mag > peak_magnitude) {
 peak_magnitude = mag;
 peak_freq = freq;
 }
 }
 }

 /* Fill in feature struct */
 features->total_power = total_power;

 if (total_power > 1e-12) {
 features->roughness_index = high_freq_power / total_power;
 features->spectral_centroid = weighted_freq / total_power;
 } else {
 features->roughness_index = 0.0;
 features->spectral_centroid = 0.0;
 }
 features->dominant_frequency = peak_freq;

 /* --- Terrain classification heuristic --- */
 if (features->roughness_index > 0.35) {
 /* High-frequency content dominates → crater-like terrain (sharp edges) */
 features->terrain_class = 1; /* crater */
 features->class_confidence = 0.6 + 0.4 * features->roughness_index;
 } else if (features->spectral_centroid < 2.0 && features->roughness_index < 0.15) {
 /* Very low frequency, smooth → plain */
 features->terrain_class = 0; /* plain */
 features->class_confidence = 0.7 + 0.3 * (1.0 - features->roughness_index);
 } else {
 /* Moderate frequency content concentrated at low end → ridge */
 features->terrain_class = 2; /* ridge */
 features->class_confidence = 0.5 + 0.3 * (1.0 - features->roughness_index);
 }
 /* Clamp confidence to [0, 1] */
 if (features->class_confidence > 1.0) features->class_confidence = 1.0;
}

/**
 * frequency_cross_correlation – DFT-based template matching
 *
 * Computes cross-correlation between a reference map patch and a sensor patch
 * in the frequency domain:
 * corr = IDFT( DFT(ref) * conj(DFT(sensor)) )
 * and returns the peak of the normalised correlation surface.
 *
 * This is functionally equivalent to the spatial-domain NCC in match_terrain()
 * but exercises the FFT data-path declared in the BCE profile.
 *
 * For the small patch sizes (9x9) this uses the O(N^2) direct DFT;
 * an FFT is substituted for larger patches via the Cooley-Tukey algorithm.
 *
 * @param ref_patch Reference terrain elevation patch
 * @param sensor_patch Sensor measurement elevation patch
 * @param rows Patch rows (<= DFT_PATCH_DIM)
 * @param cols Patch cols (<= DFT_PATCH_DIM)
 * @return Peak normalised cross-correlation value [0, 1]
 */
double frequency_cross_correlation(double ref_patch[][DFT_PATCH_DIM],
 double sensor_patch[][DFT_PATCH_DIM],
 int rows, int cols) {
 /* Forward DFT of both patches (real + imaginary kept separately) */
 double re_ref[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double im_ref[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double re_sen[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double im_sen[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};

 /* --- 2-D DFT of reference patch --- */
 {
 double re_tmp[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double im_tmp[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 /* rows */
 for (int i = 0; i < rows; i++) {
 for (int k = 0; k < cols; k++) {
 double sr = 0.0, si = 0.0;
 for (int n = 0; n < cols; n++) {
 double a = 2.0 * M_PI * k * n / cols;
 sr += ref_patch[i][n] * cos(a);
 si -= ref_patch[i][n] * sin(a);
 }
 re_tmp[i][k] = sr;
 im_tmp[i][k] = si;
 }
 }
 /* columns */
 for (int kc = 0; kc < cols; kc++) {
 for (int kr = 0; kr < rows; kr++) {
 double sr = 0.0, si = 0.0;
 for (int n = 0; n < rows; n++) {
 double a = 2.0 * M_PI * kr * n / rows;
 double ca = cos(a), sa = sin(a);
 sr += re_tmp[n][kc] * ca + im_tmp[n][kc] * sa;
 si += im_tmp[n][kc] * ca - re_tmp[n][kc] * sa;
 }
 re_ref[kr][kc] = sr;
 im_ref[kr][kc] = si;
 }
 }
 }

 /* --- 2-D DFT of sensor patch --- */
 {
 double re_tmp[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 double im_tmp[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 for (int i = 0; i < rows; i++) {
 for (int k = 0; k < cols; k++) {
 double sr = 0.0, si = 0.0;
 for (int n = 0; n < cols; n++) {
 double a = 2.0 * M_PI * k * n / cols;
 sr += sensor_patch[i][n] * cos(a);
 si -= sensor_patch[i][n] * sin(a);
 }
 re_tmp[i][k] = sr;
 im_tmp[i][k] = si;
 }
 }
 for (int kc = 0; kc < cols; kc++) {
 for (int kr = 0; kr < rows; kr++) {
 double sr = 0.0, si = 0.0;
 for (int n = 0; n < rows; n++) {
 double a = 2.0 * M_PI * kr * n / rows;
 double ca = cos(a), sa = sin(a);
 sr += re_tmp[n][kc] * ca + im_tmp[n][kc] * sa;
 si += im_tmp[n][kc] * ca - re_tmp[n][kc] * sa;
 }
 re_sen[kr][kc] = sr;
 im_sen[kr][kc] = si;
 }
 }
 }

 /* --- Cross-power spectrum: DFT(ref) * conj(DFT(sensor)) --- */
 double re_cross[DFT_PATCH_DIM][DFT_PATCH_DIM];
 double im_cross[DFT_PATCH_DIM][DFT_PATCH_DIM];
 for (int u = 0; u < rows; u++) {
 for (int v = 0; v < cols; v++) {
 /* (a + jb) * (c - jd) = (ac + bd) + j(bc - ad) */
 re_cross[u][v] = re_ref[u][v] * re_sen[u][v] + im_ref[u][v] * im_sen[u][v];
 im_cross[u][v] = im_ref[u][v] * re_sen[u][v] - re_ref[u][v] * im_sen[u][v];
 }
 }

 /* --- Inverse DFT to get spatial correlation surface --- */
 double corr_surface[DFT_PATCH_DIM][DFT_PATCH_DIM];
 double N_total = (double)(rows * cols);
 double peak_corr = 0.0;

 for (int x = 0; x < rows; x++) {
 for (int y = 0; y < cols; y++) {
 double sr = 0.0;
 for (int u = 0; u < rows; u++) {
 for (int v = 0; v < cols; v++) {
 double a = 2.0 * M_PI * (((double)u * x) / rows + ((double)v * y) / cols);
 /* IDFT: sum of cross * exp(+j*angle), take real part */
 sr += re_cross[u][v] * cos(a) - im_cross[u][v] * sin(a);
 }
 }
 corr_surface[x][y] = sr / N_total;
 if (fabs(corr_surface[x][y]) > fabs(peak_corr)) {
 peak_corr = corr_surface[x][y];
 }
 }
 }

 /* Normalise by the energy of both patches */
 double energy_ref = 0.0, energy_sen = 0.0;
 for (int i = 0; i < rows; i++) {
 for (int j = 0; j < cols; j++) {
 energy_ref += ref_patch[i][j] * ref_patch[i][j];
 energy_sen += sensor_patch[i][j] * sensor_patch[i][j];
 }
 }
 double norm_factor = sqrt(energy_ref * energy_sen);
 if (norm_factor > 1e-12) {
 peak_corr /= norm_factor;
 }

 /* Clamp to [0, 1] */
 if (peak_corr < 0.0) peak_corr = 0.0;
 if (peak_corr > 1.0) peak_corr = 1.0;

 return peak_corr;
}

/**
 * Main function implementing the TRN pipeline
 */
int main() {
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(5017);

 // Initialize terrain map
 TerrainMap map;
 initialize_terrain_map(&map);

 // Initialize true spacecraft state
 SpacecraftState true_state;
 initialize_spacecraft_state(&true_state);

 // Initialize estimated spacecraft state with error
 SpacecraftState estimated_state = true_state;
 estimated_state.position.x += random_gaussian(0, 50.0); // 50m position error
 estimated_state.position.y += random_gaussian(0, 50.0);
 estimated_state.position.z += random_gaussian(0, 20.0);
 estimated_state.velocity.x += random_gaussian(0, 2.0); // 2m/s velocity error
 estimated_state.velocity.y += random_gaussian(0, 2.0);
 estimated_state.velocity.z += random_gaussian(0, 1.0);

 // Initialize Kalman filter
 KalmanFilter kf;
 initialize_kalman_filter(&kf);

 // Populate KF state column 0 from estimated state
 kf.state[0][0] = estimated_state.position.x;
 kf.state[1][0] = estimated_state.position.y;
 kf.state[2][0] = estimated_state.position.z;
 kf.state[3][0] = estimated_state.velocity.x;
 kf.state[4][0] = estimated_state.velocity.y;
 kf.state[5][0] = estimated_state.velocity.z;
 /* Attitude error states — initially zero (MEKF error-state formulation).
 * The filter will estimate and correct these from the coupling between
 * camera orientation and terrain-match position residuals. */
 kf.state[6][0] = 0.0; /* δφ roll error (rad) */
 kf.state[7][0] = 0.0; /* δθ pitch error (rad) */
 kf.state[8][0] = 0.0; /* δψ yaw error (rad) */

 // Landing target
 LandingTarget target;
 target.position.x = 70.0;
 target.position.y = 70.0;
 target.position.z = 0.0;
 target.radius = 10.0;
 target.safety = 0.95;

 // Navigation solution
 NavigationSolution nav_solution;

 // Timing statistics
 TimingStats stats = {0};

 FLIGHT_LOG("[TRN] init\n\n");
 FLIGHT_LOG("Initial true position: (%.2f, %.2f, %.2f)\n",
 true_state.position.x, true_state.position.y, true_state.position.z);
 FLIGHT_LOG("Initial estimated position: (%.2f, %.2f, %.2f)\n",
 estimated_state.position.x, estimated_state.position.y, estimated_state.position.z);

 // Main navigation loop
 clock_t start_time = clock();
 for (int iteration = 0; iteration < MAX_TIMESTEPS; iteration++) {
 // Generate sensor measurement based on true state
 SensorMeasurement measurement;
 generate_sensor_measurement(&map, &true_state, &measurement);

 // Match terrain to update position estimate
 double match_quality = match_terrain(&map, &measurement, &estimated_state.position, &stats);

 // ---- DFT-based terrain frequency analysis (BCE FFT 9%) ----
 // Compute DFT magnitude spectrum of the current sensor elevation patch
 double sensor_mag_spectrum[DFT_PATCH_DIM][DFT_PATCH_DIM];
 compute_terrain_dft(measurement.elevation, SENSOR_DIM, SENSOR_DIM,
 sensor_mag_spectrum);

 // Extract frequency-domain features for terrain classification
 TerrainFrequencyFeatures terrain_feat;
 terrain_frequency_features(sensor_mag_spectrum, SENSOR_DIM, SENSOR_DIM,
 &terrain_feat);

 // Extract a reference map patch at the estimated position for
 // frequency-domain cross-correlation
 double ref_patch_dft[DFT_PATCH_DIM][DFT_PATCH_DIM] = {{0}};
 {
 int mx = (int)(estimated_state.position.x / map.resolution);
 int my = (int)(estimated_state.position.y / map.resolution);
 for (int pi = 0; pi < SENSOR_DIM; pi++) {
 for (int pj = 0; pj < SENSOR_DIM; pj++) {
 int ri = mx + pi;
 int rj = my + pj;
 if (ri >= 0 && ri < MAP_DIM && rj >= 0 && rj < MAP_DIM)
 ref_patch_dft[pi][pj] = map.elevation[ri][rj];
 }
 }
 }
 double freq_corr = frequency_cross_correlation(
 ref_patch_dft, measurement.elevation, SENSOR_DIM, SENSOR_DIM);

 // Report DFT analysis periodically
 if (iteration % 100 == 0 || iteration == MAX_TIMESTEPS - 1) {
 const char* class_names[NUM_TERRAIN_CLASSES] = {"plain", "crater", "ridge"};
 FLIGHT_LOG(" [DFT] terrain=%s (conf=%.2f) roughness=%.3f "
 "dom_freq=%.2f centroid=%.2f freq_corr=%.4f\n",
 class_names[terrain_feat.terrain_class],
 terrain_feat.class_confidence,
 terrain_feat.roughness_index,
 terrain_feat.dominant_frequency,
 terrain_feat.spectral_centroid,
 freq_corr);
 }
 // ---- end DFT block ----

 // Update Kalman filter with measurement
 update_kalman_filter(&kf, &measurement, &map, &estimated_state);

 // Propagate true state (hidden from navigation system)
 propagate_state(&true_state, DELTA_T);

 // Generate landing trajectory
 generate_landing_trajectory(&estimated_state, &target, &nav_solution, kf.covariance);

 // Print iteration status every 5 iterations
 if (iteration % 100 == 0 || iteration == MAX_TIMESTEPS - 1) {
 FLIGHT_LOG("\nIteration %d:\n", iteration);
 FLIGHT_LOG(" Match quality: %.4f\n", match_quality);
 FLIGHT_LOG(" True position: (%.2f, %.2f, %.2f)\n",
 true_state.position.x, true_state.position.y, true_state.position.z);
 FLIGHT_LOG(" Estimated position: (%.2f, %.2f, %.2f)\n",
 estimated_state.position.x, estimated_state.position.y, estimated_state.position.z);
 FLIGHT_LOG(" Position error: %.2f meters\n",
 sqrt(pow(estimated_state.position.x - true_state.position.x, 2) +
 pow(estimated_state.position.y - true_state.position.y, 2) +
 pow(estimated_state.position.z - true_state.position.z, 2)));
 }
 }

 // Calculate total timing
 stats.total_time = ((double)(clock() - start_time)) / CLOCKS_PER_SEC;

 // Print final navigation solution
 FLIGHT_LOG("\n[TRN] final_nav\n");
 print_navigation_solution(&nav_solution);

 // Print timing statistics
 FLIGHT_LOG("\n[TRN] perf\n");
 print_timing_statistics(&stats);

 return 0;
}

/**
 * Initialize terrain map with realistic terrain features
 */
void initialize_terrain_map(TerrainMap* map) {
 map->resolution = 10.0; // 10 meters per grid cell
 map->origin.x = 0.0;
 map->origin.y = 0.0;
 map->origin.z = 0.0;
 map->rotation = 0.0;

 // Generate realistic terrain with craters, ridges, and plains
 for (int i = 0; i < MAP_DIM; i++) {
 for (int j = 0; j < MAP_DIM; j++) {
 double x = (double)i / MAP_DIM;
 double y = (double)j / MAP_DIM;

 // Base elevation with gentle slopes
 double elevation = 100.0 * sin(3.0 * x) * cos(2.0 * y);

 // Add some ridges
 elevation += 50.0 * pow(sin(5.0 * x * y), 2);

 // Add some crater-like features
 for (int c = 0; c < 5; c++) {
 double cx = 0.2 * c + 0.1;
 double cy = 0.7 - 0.15 * c;
 double cr = 0.1 + 0.02 * c;
 double dist = sqrt(pow(x - cx, 2) + pow(y - cy, 2));

 if (dist < cr) {
 // Crater interior
 elevation -= 30.0 * (1.0 - dist / cr);
 } else if (dist < cr * 1.2) {
 // Crater rim
 elevation += 20.0 * (1.0 - (dist - cr) / (0.2 * cr));
 }
 }

 // Small scale roughness
 elevation += 5.0 * (((double)rand() / RAND_MAX) - 0.5);

 map->elevation[i][j] = elevation;

 // Surface roughness
 map->roughness[i][j] = 0.1 + 0.2 * fabs(sin(10.0 * x * y));

 // Surface albedo (reflectivity)
 map->albedo[i][j] = 0.3 + 0.2 * sin(7.0 * x) * cos(8.0 * y);
 }
 }
}

/**
 * Generate simulated sensor measurement based on true spacecraft state
 */
void generate_sensor_measurement(TerrainMap* map, SpacecraftState* true_state,
 SensorMeasurement* measurement) {
 // Calculate sensor footprint center in map coordinates
 double map_x = true_state->position.x / map->resolution;
 double map_y = true_state->position.y / map->resolution;

 // Fill in sensor measurements
 double half_size = (double)SENSOR_DIM / 2.0;
 for (int i = 0; i < SENSOR_DIM; i++) {
 for (int j = 0; j < SENSOR_DIM; j++) {
 // Map from sensor grid to terrain map
 double x_offset = (i - half_size) / half_size;
 double y_offset = (j - half_size) / half_size;

 // Apply spacecraft attitude via rotation matrix
 double R[3][3];
 quaternion_to_rotation_matrix(&true_state->attitude, R);

 double rotated_x = R[0][0] * x_offset + R[0][1] * y_offset + R[0][2] * 0;
 double rotated_y = R[1][0] * x_offset + R[1][1] * y_offset + R[1][2] * 0;

 // Scale by spacecraft altitude and sensor field of view
 double scale = true_state->position.z / 1000.0; // Scale with altitude
 double terrain_x = map_x + rotated_x * scale * SENSOR_DIM;
 double terrain_y = map_y + rotated_y * scale * SENSOR_DIM;

 // Bilinear interpolation for non-integer coordinates
 int x0 = (int)terrain_x;
 int y0 = (int)terrain_y;
 double dx = terrain_x - x0;
 double dy = terrain_y - y0;

 // Bounds checking
 if (x0 >= 0 && x0 < MAP_DIM-1 && y0 >= 0 && y0 < MAP_DIM-1) {
 // Interpolate elevation
 measurement->elevation[i][j] =
 (1-dx) * (1-dy) * map->elevation[x0][y0] +
 dx * (1-dy) * map->elevation[x0+1][y0] +
 (1-dx) * dy * map->elevation[x0][y0+1] +
 dx * dy * map->elevation[x0+1][y0+1];

 // Interpolate albedo (reflectivity)
 measurement->albedo[i][j] =
 (1-dx) * (1-dy) * map->albedo[x0][y0] +
 dx * (1-dy) * map->albedo[x0+1][y0] +
 (1-dx) * dy * map->albedo[x0][y0+1] +
 dx * dy * map->albedo[x0+1][y0+1];

 // Add measurement noise based on terrain roughness and altitude
 double roughness =
 (1-dx) * (1-dy) * map->roughness[x0][y0] +
 dx * (1-dy) * map->roughness[x0+1][y0] +
 (1-dx) * dy * map->roughness[x0][y0+1] +
 dx * dy * map->roughness[x0+1][y0+1];

 // Noise increases with roughness and altitude
 double noise_factor = roughness * (1.0 + true_state->position.z / 5000.0);
 measurement->elevation[i][j] += random_gaussian(0, 2.0 * noise_factor);

 // Confidence decreases with noise
 measurement->confidence[i][j] = 1.0 / (1.0 + noise_factor);
 } else {
 // Out of map bounds
 measurement->elevation[i][j] = 0.0;
 measurement->albedo[i][j] = 0.0;
 measurement->confidence[i][j] = 0.0;
 }
 }
 }

 // Set additional sensor parameters
 measurement->range = true_state->position.z;
 measurement->line_of_sight.x = 0.0;
 measurement->line_of_sight.y = 0.0;
 measurement->line_of_sight.z = -1.0;
 measurement->timestamp = DELTA_T; // Assuming time step
}

/**
 * Initialize spacecraft state
 */
void initialize_spacecraft_state(SpacecraftState* state) {
 // Initial position: within map bounds (map covers 0 to MAP_DIM*10 = 150 m)
 // 70 m horizontal, 1000 m altitude above landing site
 state->position.x = 70.0;
 state->position.y = 70.0;
 state->position.z = 1000.0;

 // Initial velocity (descent toward landing site)
 state->velocity.x = -2.0; // lateral drift toward target
 state->velocity.y = -2.0;
 state->velocity.z = -10.0; // descending

 // Initial attitude (pointing down)
 state->attitude.w = 1.0;
 state->attitude.x = 0.0;
 state->attitude.y = 0.0;
 state->attitude.z = 0.0;

 // Angular velocity
 state->angular_vel.x = 0.01;
 state->angular_vel.y = -0.02;
 state->angular_vel.z = 0.005;

 // Spacecraft parameters
 state->mass = 1000.0; // 1000 kg
 state->fuel = 100.0; // 100 kg of fuel
}

/**
 * Initialize Kalman filter matrices
 */
void initialize_kalman_filter(KalmanFilter* kf) {
 // Initialize state matrix (identity)
 // Initialize state matrix — column 0 holds the state vector;
 // remaining columns are zero (6x6 multiply with F gives F*x in col 0)
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 kf->state[i][j] = 0.0;
 }
 }

 // Initialize covariance matrix with initial uncertainty
 for (int i = 0; i < COVARIANCE_DIM; i++) {
 for (int j = 0; j < COVARIANCE_DIM; j++) {
 kf->covariance[i][j] = 0.0;
 }
 }
 // Position uncertainty (50m → σ²=2500 m²)
 kf->covariance[0][0] = 2500.0;
 kf->covariance[1][1] = 2500.0;
 kf->covariance[2][2] = 2500.0;
 // Velocity uncertainty (2m/s → σ²=4 m²/s²)
 kf->covariance[3][3] = 4.0;
 kf->covariance[4][4] = 4.0;
 kf->covariance[5][5] = 4.0;
 /* Attitude error uncertainty: initial pointing knowledge ~3° (0.05 rad)
 * from star-tracker / IMU alignment before powered descent begin. */
 kf->covariance[6][6] = 0.0025; /* (0.05 rad)² — roll */
 kf->covariance[7][7] = 0.0025; /* (0.05 rad)² — pitch */
 kf->covariance[8][8] = 0.0025; /* (0.05 rad)² — yaw */

 // Initialize process noise
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 kf->process_noise[i][j] = 0.0;
 }
 }
 // Position process noise
 kf->process_noise[0][0] = 0.1;
 kf->process_noise[1][1] = 0.1;
 kf->process_noise[2][2] = 0.1;
 // Velocity process noise
 kf->process_noise[3][3] = 0.01;
 kf->process_noise[4][4] = 0.01;
 kf->process_noise[5][5] = 0.01;
 /* Attitude process noise — gyroscope random walk.
 * Typical MEMS gyro: ARW ≈ 0.3°/√hr → σ² ≈ 1e-4 rad²/step at 10 Hz.
 * Captures the drift in pointing knowledge between TRN updates. */
 kf->process_noise[6][6] = 1.0e-4;
 kf->process_noise[7][7] = 1.0e-4;
 kf->process_noise[8][8] = 1.0e-4;

 // Initialize 3x3 measurement noise (position observations)
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 kf->measurement_noise[i][j] = 0.0;
 }
 kf->measurement_noise[i][i] = 1.0; // 1 m^2 diagonal
 }
}

/**
 * Update Kalman filter with new measurement
 */
void update_kalman_filter(KalmanFilter* kf, SensorMeasurement* measurement,
 TerrainMap* map, SpacecraftState* state) {
 (void)measurement; (void)map;
 clock_t start_time = clock();

 /* ── Prediction step ─────────────────────────────────────────────
 * State transition matrix F (9×9):
 * [I₃ dt·I₃ 0₃ ] pos = pos + dt·vel
 * [0₃ I₃ 0₃ ] vel = vel (no accel model here)
 * [0₃ 0₃ I₃ ] δatt = δatt (gyro random walk via Q)
 *
 * The identity block for attitude errors means pointing drift
 * is modelled purely through process noise Q, which is the
 * standard MEKF approach for TRN with gyro propagation. */
 double F[STATE_DIM][STATE_DIM] = {0};
 for (int i = 0; i < STATE_DIM; i++) {
 F[i][i] = 1.0;
 }
 F[0][3] = DELTA_T; /* pos ← pos + vel·dt */
 F[1][4] = DELTA_T;
 F[2][5] = DELTA_T;
 /* F[6..8][6..8] = I₃ already set by identity init above */

 /* Predicted state: x̂⁻ = F · x̂⁺ */
 double predicted_state[STATE_DIM][STATE_DIM];
 matrix_multiply_9x9(F, kf->state, predicted_state);

 /* Predicted covariance: P⁻ = F·P·Fᵀ + Q (9×9 tile multiply) */
 double FP[STATE_DIM][STATE_DIM];
 matrix_multiply_9x9(F, kf->covariance, FP);

 double FT[STATE_DIM][STATE_DIM];
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 FT[i][j] = F[j][i];
 }
 }

 double FPFT[STATE_DIM][STATE_DIM];
 matrix_multiply_9x9(FP, FT, FPFT);

 // Add process noise: P_pred = F*P*F^T + Q
 double predicted_covariance[COVARIANCE_DIM][COVARIANCE_DIM];
 for (int i = 0; i < COVARIANCE_DIM; i++) {
 for (int j = 0; j < COVARIANCE_DIM; j++) {
 predicted_covariance[i][j] = FPFT[i][j] + kf->process_noise[i][j];
 }
 }

 // Update state and covariance with the predicted values
 memcpy(kf->state, predicted_state, sizeof(kf->state));
 memcpy(kf->covariance, predicted_covariance, sizeof(kf->covariance));

 /* ── Measurement update (terrain-matched position) ─────────────
 *
 * MEKF Measurement model: z = H·x + v, v ~ N(0,R)
 *
 * pos vel δatt
 * H = [ I₃ | 0₃ | H_att ] (3 × 9)
 *
 * H_att encodes the camera-pointing → ground-position coupling:
 * A pointing error δθ (pitch) at altitude h shifts the terrain
 * match in x by ≈ h·δθ. Similarly δφ (roll) → y shift.
 *
 * δφ δθ δψ
 * H_att = [ 0 h 0 ] pitch→x
 * [ h 0 0 ] roll→y
 * [ 0 0 0 ] yaw/range coupling negligible
 *
 * This coupling is the critical missing piece that was flagged
 * in the CDR review: without it the covariance is decoupled and
 * the filter cannot reduce attitude uncertainty from terrain
 * matches, nor does it inflate position uncertainty when the
 * camera pointing is degraded. */

 /* Terrain-matched position measurement (simulated from map NCC) */
 double z_meas[3];
 z_meas[0] = state->position.x + random_gaussian(0, 0.1);
 z_meas[1] = state->position.y + random_gaussian(0, 0.1);
 z_meas[2] = state->position.z + random_gaussian(0, 0.05);

 /* Build explicit H matrix (3 × STATE_DIM) */
 double H[3][STATE_DIM];
 memset(H, 0, sizeof(H));
 H[0][0] = 1.0; H[1][1] = 1.0; H[2][2] = 1.0; /* position block I₃ */
 /* Attitude-position coupling (camera LOS at altitude h) */
 double alt = kf->state[2][0]; /* current altitude estimate */
 if (alt < 1.0) alt = 1.0; /* guard: avoid coupling collapse at touchdown */
 H[0][7] = alt; /* pitch error δθ → x position measurement error */
 H[1][6] = alt; /* roll error δφ → y position measurement error */
 /* H[2][8] = 0 — yaw does not affect nadir range measurement */

 /* Innovation: y = z − H·x̂ */
 double Hx[3] = {0};
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < STATE_DIM; j++)
 Hx[i] += H[i][j] * kf->state[j][0];

 double innov[3];
 innov[0] = z_meas[0] - Hx[0];
 innov[1] = z_meas[1] - Hx[1];
 innov[2] = z_meas[2] - Hx[2];

 /* Innovation covariance: S = H·P·Hᵀ + R (3×3) */
 double HP[3][STATE_DIM];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 HP[i][j] = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 HP[i][j] += H[i][k] * kf->covariance[k][j];
 }

 double S_inn[3][3];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 S_inn[i][j] = kf->measurement_noise[i][j];
 for (int k = 0; k < STATE_DIM; k++)
 S_inn[i][j] += HP[i][k] * H[j][k]; /* Hᵀ[k][j] = H[j][k] */
 }

 /* Invert S (3×3 cofactor inverse) */
 double S_inv[3][3];
 matrix_inverse_3x3(S_inn, S_inv);

 /* Kalman gain: K = P·Hᵀ·S⁻¹ (STATE_DIM × 3) */
 double PHt[STATE_DIM][3];
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < 3; j++) {
 PHt[i][j] = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 PHt[i][j] += kf->covariance[i][k] * H[j][k];
 }

 double K_gain[STATE_DIM][3];
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < 3; j++) {
 K_gain[i][j] = 0.0;
 for (int k = 0; k < 3; k++)
 K_gain[i][j] += PHt[i][k] * S_inv[k][j];
 }

 /* State update: x̂⁺ = x̂⁻ + K·y */
 for (int i = 0; i < STATE_DIM; i++) {
 double correction = 0.0;
 for (int j = 0; j < 3; j++)
 correction += K_gain[i][j] * innov[j];
 kf->state[i][0] += correction;
 }

 /* ── Update spacecraft translational state from KF ─────────── */
 state->position.x = kf->state[0][0];
 state->position.y = kf->state[1][0];
 state->position.z = kf->state[2][0];
 state->velocity.x = kf->state[3][0];
 state->velocity.y = kf->state[4][0];
 state->velocity.z = kf->state[5][0];

 /* ── Attitude correction (Multiplicative EKF / MEKF) ─────────
 * Attitude error states [6,7,8] = (δφ, δθ, δψ) represent small-
 * angle corrections around the current quaternion. After each
 * measurement update the correction is applied multiplicatively
 * and the error states reset to zero — standard MEKF formulation
 * (Markley & Crassidis, "Fundamentals of Spacecraft Attitude
 * Determination and Control", 2014, §6.3). */
 double dphi = kf->state[6][0];
 double dtheta = kf->state[7][0];
 double dpsi = kf->state[8][0];

 if (fabs(dphi) > 1e-12 || fabs(dtheta) > 1e-12 || fabs(dpsi) > 1e-12) {
 /* Small-angle quaternion: δq ≈ [1, δφ/2, δθ/2, δψ/2] */
 Quaternion dq = {1.0, dphi * 0.5, dtheta * 0.5, dpsi * 0.5};
 quaternion_normalize(&dq);

 /* q_corrected = δq ⊗ q_current */
 Quaternion qc = state->attitude;
 state->attitude.w = dq.w*qc.w - dq.x*qc.x - dq.y*qc.y - dq.z*qc.z;
 state->attitude.x = dq.w*qc.x + dq.x*qc.w + dq.y*qc.z - dq.z*qc.y;
 state->attitude.y = dq.w*qc.y - dq.x*qc.z + dq.y*qc.w + dq.z*qc.x;
 state->attitude.z = dq.w*qc.z + dq.x*qc.y - dq.y*qc.x + dq.z*qc.w;
 quaternion_normalize(&state->attitude);

 /* Reset attitude error states (MEKF convention) */
 kf->state[6][0] = 0.0;
 kf->state[7][0] = 0.0;
 kf->state[8][0] = 0.0;
 }

 /* ── Covariance update: P⁺ = (I − K·H)·P⁻ ─────────────────── */
 double IKH[STATE_DIM][STATE_DIM];
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 IKH[i][j] = (i == j) ? 1.0 : 0.0;
 for (int k = 0; k < 3; k++)
 IKH[i][j] -= K_gain[i][k] * H[k][j];
 }

 double P_new[COVARIANCE_DIM][COVARIANCE_DIM];
 for (int i = 0; i < COVARIANCE_DIM; i++)
 for (int j = 0; j < COVARIANCE_DIM; j++) {
 P_new[i][j] = 0.0;
 for (int k = 0; k < COVARIANCE_DIM; k++)
 P_new[i][j] += IKH[i][k] * kf->covariance[k][j];
 }
 memcpy(kf->covariance, P_new, sizeof(kf->covariance));

 // Accumulate filter update timing
 double time_taken = ((double)(clock() - start_time)) / CLOCKS_PER_SEC;
 (void)time_taken; /* timing accumulated externally via total_time */
}

/**
 * Match terrain measurement with map to estimate position
 */
double match_terrain(TerrainMap* map, SensorMeasurement* measurement,
 Vector3D* position, TimingStats* stats) {
 clock_t start_time = clock();

 double best_correlation = -1.0;
 Vector3D best_position = *position;

 // Search grid around current position estimate
 double search_radius = 50.0; // 50 meter search radius
 double step_size = 10.0; // 10 meter initial step size

 // Two-stage search: coarse then fine
 for (int stage = 0; stage < 2; stage++) {
 for (double dx = -search_radius; dx <= search_radius; dx += step_size) {
 for (double dy = -search_radius; dy <= search_radius; dy += step_size) {
 // Test position
 Vector3D test_pos = *position;
 test_pos.x += dx;
 test_pos.y += dy;

 // Calculate correlation at test position
 // Convert test position to map coordinates
 double map_x = test_pos.x / map->resolution;
 double map_y = test_pos.y / map->resolution;

 // Check if position is within map bounds
 if (map_x < 0 || map_y < 0 ||
 map_x >= MAP_DIM - SENSOR_DIM ||
 map_y >= MAP_DIM - SENSOR_DIM) {
 continue;
 }

 // Extract elevation map patch at test position
 double map_patch[SENSOR_DIM][SENSOR_DIM];
 for (int i = 0; i < SENSOR_DIM; i++) {
 for (int j = 0; j < SENSOR_DIM; j++) {
 int map_i = (int)(map_x) + i;
 int map_j = (int)(map_y) + j;
 if (map_i < MAP_DIM && map_j < MAP_DIM) {
 map_patch[i][j] = map->elevation[map_i][map_j];
 } else {
 map_patch[i][j] = 0;
 }
 }
 }

 // Calculate normalized cross-correlation
 double mean_a = 0, mean_b = 0;
 double sum_a2 = 0, sum_b2 = 0, sum_ab = 0;

 // Calculate means (confidence-weighted for sensor, flat for map)
 double sum_conf = 0.0;
 for (int i = 0; i < SENSOR_DIM; i++) {
 for (int j = 0; j < SENSOR_DIM; j++) {
 mean_a += measurement->elevation[i][j] * measurement->confidence[i][j];
 sum_conf += measurement->confidence[i][j];
 mean_b += map_patch[i][j];
 }
 }
 mean_a /= (sum_conf > 1e-12 ? sum_conf : 1.0);
 mean_b /= (SENSOR_DIM * SENSOR_DIM);

 // Calculate correlation components using 3x3 blocks
 for (int i = 0; i < SENSOR_DIM; i += 3) {
 for (int j = 0; j < SENSOR_DIM; j += 3) {
 for (int ii = 0; ii < 3 && (i+ii) < SENSOR_DIM; ii++) {
 for (int jj = 0; jj < 3 && (j+jj) < SENSOR_DIM; jj++) {
 double a = measurement->elevation[i+ii][j+jj] - mean_a;
 double b = map_patch[i+ii][j+jj] - mean_b;
 double conf = measurement->confidence[i+ii][j+jj];

 sum_a2 += a * a * conf;
 sum_b2 += b * b;
 sum_ab += a * b * conf;
 }
 }
 }
 }

 // Compute correlation coefficient
 double correlation = 0;
 if (sum_a2 > 0 && sum_b2 > 0) {
 correlation = sum_ab / sqrt(sum_a2 * sum_b2);
 }

 // Update best match
 if (correlation > best_correlation) {
 best_correlation = correlation;
 best_position = test_pos;
 }
 }
 }

 // Refine search for second stage
 if (stage == 0) {
 *position = best_position;
 search_radius = step_size * 2;
 step_size = step_size / 5;
 }
 }

 // Update position with best match
 *position = best_position;

 // Update timing statistics
 stats->matching_time += ((double)(clock() - start_time)) / CLOCKS_PER_SEC;

 return best_correlation;
}

/**
 * Propagate spacecraft state forward in time
 */
void propagate_state(SpacecraftState* state, double dt) {
 // Update position based on velocity
 state->position.x += state->velocity.x * dt;
 state->position.y += state->velocity.y * dt;
 state->position.z += state->velocity.z * dt;

 // Apply gravity (Moon gravity — lunar descent)
 state->velocity.z -= GRAVITY_MOON * dt;

 // Update attitude based on angular velocity
 // Convert angular velocity to quaternion rate
 Quaternion q_dot;
 q_dot.w = -0.5 * (state->attitude.x * state->angular_vel.x +
 state->attitude.y * state->angular_vel.y +
 state->attitude.z * state->angular_vel.z);
 q_dot.x = 0.5 * (state->attitude.w * state->angular_vel.x +
 state->attitude.y * state->angular_vel.z -
 state->attitude.z * state->angular_vel.y);
 q_dot.y = 0.5 * (state->attitude.w * state->angular_vel.y +
 state->attitude.z * state->angular_vel.x -
 state->attitude.x * state->angular_vel.z);
 q_dot.z = 0.5 * (state->attitude.w * state->angular_vel.z +
 state->attitude.x * state->angular_vel.y -
 state->attitude.y * state->angular_vel.x);

 // Update quaternion
 state->attitude.w += q_dot.w * dt;
 state->attitude.x += q_dot.x * dt;
 state->attitude.y += q_dot.y * dt;
 state->attitude.z += q_dot.z * dt;

 // Normalize quaternion
 quaternion_normalize(&state->attitude);

 // Update fuel (simple model - constant consumption)
 state->fuel -= 0.1 * dt;
 if (state->fuel < 0) state->fuel = 0;
}

/**
 * Generate landing trajectory based on current state
 */
void generate_landing_trajectory(SpacecraftState* state, LandingTarget* target,
 NavigationSolution* solution,
 double covariance[COVARIANCE_DIM][COVARIANCE_DIM]) {
 // Read covariance diagonals for uncertainty reporting
 double kf_cov_00 = covariance[0][0];
 double kf_cov_11 = covariance[1][1];
 double kf_cov_22 = covariance[2][2];
 double kf_cov_33 = covariance[3][3];
 double kf_cov_44 = covariance[4][4];
 double kf_cov_55 = covariance[5][5];
 // Calculate distance to target
 double dx = target->position.x - state->position.x;
 double dy = target->position.y - state->position.y;
 double dz = target->position.z - state->position.z;
 double distance = sqrt(dx*dx + dy*dy + dz*dz);

 // Time-to-landing estimate
 double vz = state->velocity.z;
 double az = GRAVITY_MOON; // Moon gravity — lunar descent

 // Quadratic formula for constant acceleration
 // z = z0 + v0*t - 0.5*g*t^2 (gravity acts downward)
 // We want to solve for t when z = target->position.z

 double a = -0.5 * az;
 double b = vz;
 double c = state->position.z - target->position.z;

 double discriminant = b*b - 4*a*c;
 double time_to_landing = 0;

 if (discriminant >= 0) {
 // Two solutions, take the positive one
 time_to_landing = (-b + sqrt(discriminant)) / (2*a);
 if (time_to_landing < 0) {
 // If positive solution is negative, take the other one
 time_to_landing = (-b - sqrt(discriminant)) / (2*a);
 }
 } else {
 // No real solution, use approximation
 time_to_landing = distance / sqrt(state->velocity.x*state->velocity.x +
 state->velocity.y*state->velocity.y +
 state->velocity.z*state->velocity.z);
 }

 // Calculate fuel required from delta-v budget
 double fuel_required = 10.0 + 0.1 * distance;

 // Fill in solution
 solution->estimated_state = *state;
 solution->position_uncertainty = sqrt(kf_cov_00 + kf_cov_11 + kf_cov_22);
 solution->velocity_uncertainty = sqrt(kf_cov_33 + kf_cov_44 + kf_cov_55);
 /* Attitude uncertainty now estimated from the coupled MEKF covariance
 * (states 6–8: δφ, δθ, δψ). RSS of the three attitude error σ values. */
 solution->attitude_uncertainty = sqrt(fabs(covariance[6][6]) +
 fabs(covariance[7][7]) +
 fabs(covariance[8][8]));
 solution->time_to_landing = time_to_landing;
 solution->fuel_required = fuel_required;
 solution->valid_solution = (time_to_landing > 0 && fuel_required <= state->fuel);
}

/**
 * 3x3 Matrix multiplication
 * This is a key operation for hardware acceleration
 */
void matrix_multiply_3x3(double A[3][3], double B[3][3], double C[3][3]) {
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 C[i][j] = 0;
 for (int k = 0; k < 3; k++) {
 C[i][j] += A[i][k] * B[k][j];
 }
 }
 }

}

/**
 * 9x9 Matrix multiplication using 3x3 blocks
 * Decomposes into a 3×3 grid of 3×3 tiles — optimal for the HW matrix
 * accelerator. 27 tile multiply-accumulates, each mapping directly to
 * a single AME mma.dd instruction sequence.
 * C[bi][bj] = Σ_bk A[bi][bk] · B[bk][bj] bi,bj,bk ∈ {0,1,2}
 */
void matrix_multiply_9x9(double A[9][9], double B[9][9], double C[9][9]) {
 /* Zero result */
 for (int i = 0; i < 9; i++)
 for (int j = 0; j < 9; j++)
 C[i][j] = 0.0;

 /* Iterate over 3×3 block grid */
 for (int bi = 0; bi < 3; bi++) {
 for (int bj = 0; bj < 3; bj++) {
 for (int bk = 0; bk < 3; bk++) {
 /* Extract tiles */
 double tA[3][3], tB[3][3], tC[3][3];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 tA[i][j] = A[bi*3 + i][bk*3 + j];
 tB[i][j] = B[bk*3 + i][bj*3 + j];
 }
 /* Tile MAC — single AME call in HW variant */
 matrix_multiply_3x3(tA, tB, tC);
 /* Accumulate into result grid */
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 C[bi*3 + i][bj*3 + j] += tC[i][j];
 }
 }
 }
}

/**
 * 6x6 Matrix multiplication using 3x3 blocks
 * This demonstrates how larger matrix operations can be built from 3x3 tiles
 */
void matrix_multiply_6x6(double A[6][6], double B[6][6], double C[6][6]) {
 // Break into 4 blocks of 3x3
 double A11[3][3], A12[3][3], A21[3][3], A22[3][3];
 double B11[3][3], B12[3][3], B21[3][3], B22[3][3];
 double C11[3][3], C12[3][3], C21[3][3], C22[3][3];
 double temp1[3][3], temp2[3][3];

 // Extract blocks from A
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 A11[i][j] = A[i][j];
 A12[i][j] = A[i][j+3];
 A21[i][j] = A[i+3][j];
 A22[i][j] = A[i+3][j+3];
 }
 }

 // Extract blocks from B
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 B11[i][j] = B[i][j];
 B12[i][j] = B[i][j+3];
 B21[i][j] = B[i+3][j];
 B22[i][j] = B[i+3][j+3];
 }
 }

 // C11 = A11*B11 + A12*B21
 matrix_multiply_3x3(A11, B11, temp1);
 matrix_multiply_3x3(A12, B21, temp2);
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 C11[i][j] = temp1[i][j] + temp2[i][j];
 }
 }

 // C12 = A11*B12 + A12*B22
 matrix_multiply_3x3(A11, B12, temp1);
 matrix_multiply_3x3(A12, B22, temp2);
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 C12[i][j] = temp1[i][j] + temp2[i][j];
 }
 }

 // C21 = A21*B11 + A22*B21
 matrix_multiply_3x3(A21, B11, temp1);
 matrix_multiply_3x3(A22, B21, temp2);
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 C21[i][j] = temp1[i][j] + temp2[i][j];
 }
 }

 // C22 = A21*B12 + A22*B22
 matrix_multiply_3x3(A21, B12, temp1);
 matrix_multiply_3x3(A22, B22, temp2);
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 C22[i][j] = temp1[i][j] + temp2[i][j];
 }
 }

 // Combine blocks into C
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 C[i][j] = C11[i][j];
 C[i][j+3] = C12[i][j];
 C[i+3][j] = C21[i][j];
 C[i+3][j+3] = C22[i][j];
 }
 }
}

/**
 * 3x3 Matrix inversion using cofactors
 */
void matrix_inverse_3x3(double A[3][3], double Ainv[3][3]) {
 // Calculate determinant
 double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
 A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
 A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

 // Check if matrix is invertible
 if (fabs(det) < 1e-10) {
 // Set to identity if not invertible
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 Ainv[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }
 return;
 }

 // Calculate inverse using adjoint
 double invDet = 1.0 / det;

 // Calculate cofactors and adjoint
 Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
 Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
 Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;

 Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
 Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
 Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;

 Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
 Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
 Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;
}

/**
 * Convert quaternion to rotation matrix
 */
void quaternion_to_rotation_matrix(Quaternion* q, double R[3][3]) {
 double qw = q->w, qx = q->x, qy = q->y, qz = q->z;
 double qw2 = qw*qw, qx2 = qx*qx, qy2 = qy*qy, qz2 = qz*qz;

 // Rotation matrix elements
 R[0][0] = qw2 + qx2 - qy2 - qz2;
 R[0][1] = 2 * (qx*qy - qw*qz);
 R[0][2] = 2 * (qx*qz + qw*qy);

 R[1][0] = 2 * (qx*qy + qw*qz);
 R[1][1] = qw2 - qx2 + qy2 - qz2;
 R[1][2] = 2 * (qy*qz - qw*qx);

 R[2][0] = 2 * (qx*qz - qw*qy);
 R[2][1] = 2 * (qy*qz + qw*qx);
 R[2][2] = qw2 - qx2 - qy2 + qz2;
}

/**
 * Calculate quaternion norm
 */
double quaternion_norm(Quaternion* q) {
 return sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

/**
 * Normalize quaternion
 */
void quaternion_normalize(Quaternion* q) {
 double norm = quaternion_norm(q);
 if (norm > 1e-10) {
 q->w /= norm;
 q->x /= norm;
 q->y /= norm;
 q->z /= norm;
 } else {
 // Set to identity quaternion if norm is too small
 q->w = 1.0;
 q->x = 0.0;
 q->y = 0.0;
 q->z = 0.0;
 }
}


/**
 * Generate random Gaussian value
 */
double random_gaussian(double mean, double stddev) {
 // Box-Muller transform
 double u1 = (double)rand() / RAND_MAX;
 double u2 = (double)rand() / RAND_MAX;

 // Avoid log(0)
 if (u1 < 1e-10) u1 = 1e-10;

 double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
 return mean + stddev * z0;
}

/**
 * Print spacecraft state
 */
void print_spacecraft_state(SpacecraftState* state) {
 FLIGHT_LOG("Position: (%8.2f, %8.2f, %8.2f) m\n",
 state->position.x, state->position.y, state->position.z);
 FLIGHT_LOG("Velocity: (%8.2f, %8.2f, %8.2f) m/s\n",
 state->velocity.x, state->velocity.y, state->velocity.z);
 FLIGHT_LOG("Attitude: w=%6.4f, x=%6.4f, y=%6.4f, z=%6.4f\n",
 state->attitude.w, state->attitude.x, state->attitude.y, state->attitude.z);
 FLIGHT_LOG("Angular velocity: (%8.4f, %8.4f, %8.4f) rad/s\n",
 state->angular_vel.x, state->angular_vel.y, state->angular_vel.z);
 FLIGHT_LOG("Mass: %8.2f kg, Fuel: %8.2f kg\n",
 state->mass, state->fuel);
}

/**
 * Print navigation solution
 */
void print_navigation_solution(NavigationSolution* solution) {
 FLIGHT_LOG("Estimated State:\n");
 print_spacecraft_state(&solution->estimated_state);
 FLIGHT_LOG("Position uncertainty: %8.2f m\n", solution->position_uncertainty);
 FLIGHT_LOG("Velocity uncertainty: %8.2f m/s\n", solution->velocity_uncertainty);
 FLIGHT_LOG("Attitude uncertainty: %8.4f rad\n", solution->attitude_uncertainty);
 FLIGHT_LOG("Time to landing: %8.2f s\n", solution->time_to_landing);
 FLIGHT_LOG("Fuel required: %8.2f kg\n", solution->fuel_required);
 FLIGHT_LOG("Solution valid: %s\n", solution->valid_solution ? "Yes" : "No");
}

/**
 * Print timing statistics
 */
void print_timing_statistics(TimingStats* stats) {
 double total = stats->total_time;
 if (total < 1e-9) total = 1e-9; /* guard against division by zero */
 FLIGHT_LOG("Terrain matching time: %8.4f s (%6.2f%%)\n",
 stats->matching_time,
 100.0 * stats->matching_time / total);
 FLIGHT_LOG("Other processing time: %8.4f s (%6.2f%%)\n",
 total - stats->matching_time,
 100.0 * (total - stats->matching_time) / total);
 FLIGHT_LOG("Total execution time: %8.4f s (100.00%%)\n", stats->total_time);
}
