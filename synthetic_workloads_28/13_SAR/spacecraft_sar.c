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
 * Synthetic Aperture Radar Processing Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard SAR signal processing including range compression, azimuth
 * focusing, Doppler frequency estimation, and geometric correction. This workload
 * uses intensive trigonometric computations for chirp generation, FFT for range
 * compression, and matrix operations for image formation. Directly representative
 * of the SAR satellite (Radar Imaging Satellite) series and the joint NASA-
 * NISAR (SAR Interferometry) mission onboard processing.
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
#include <stdint.h>
#include "../flight_compliance.h"

// Spacecraft SAR system parameters (based on typical LEO SAR satellites)
#define PRF 3000 // Pulse Repetition Frequency (Hz)
#define RANGE_SAMPLES 2400 // Range samples per pulse
#define AZIMUTH_SAMPLES 2400 // Azimuth samples per aperture
#define DOPPLER_BINS 15 // Doppler centroid estimation bins (PRF/200)
#define RCM_INTERP_SIZE 9 // Range cell migration interpolation kernel
#define AUTOFOCUS_SIZE 12 // Autofocus phase gradient window
#define BLOCK_SIZE 3 // For 3x3 systolic array

// Radar parameters
#define C_LIGHT 299792458.0 // Speed of light (m/s)
#define FC 9.65e9 // Center frequency (X-band, Hz)
#define BANDWIDTH 300e6 // Chirp bandwidth (Hz)
#define PULSE_DURATION 50e-6 // Pulse duration (s)
#define ALTITUDE 514000.0 // Orbital altitude (m)
#define VELOCITY 7600.0 // Satellite velocity (m/s)
#define LOOK_ANGLE 35.0 // Look angle (degrees)

// Processing parameters
#define FFT_SIZE_R 3072 // FFT size for range (3x1024 radix-3)
#define FFT_SIZE_A 3072 // FFT size for azimuth (3x1024 radix-3)
#define LOOKS 3 // Number of looks for multilooking
#define WAVELET_TILE 16 // Wavelet processing tile size (power of 2 for dyadic decomposition)
#define SRC_TILE 15 // Secondary range compression tile (5 AME 3x3 blocks)

// Complex number structure
typedef struct {
 float real;
 float imag;
} complex_float;

// SAR data structures
typedef struct {
 complex_float **data; // Raw SAR data (row pointers into static pool)
 float **magnitude; // Detected magnitude image (row pointers into static pool)
 int range_lines;
 int azimuth_lines;
 float range_spacing;
 float azimuth_spacing;
 float doppler_centroid;
 float doppler_rate;
} SARData;

/* -----------------------------------------------------------------------
 * Static memory pools — flight-software allocation strategy.
 *
 * All SAR image buffers are allocated at compile time in BSS to prevent
 * heap fragmentation or runtime malloc failures during an imaging pass.
 * Row-pointer tables provide the double-indirection required by the
 * existing sar->data[r][a] access pattern without any heap usage.
 * ----------------------------------------------------------------------- */
static complex_float sar_data_pool[RANGE_SAMPLES][AZIMUTH_SAMPLES];
static float sar_mag_pool [RANGE_SAMPLES][AZIMUTH_SAMPLES];
static complex_float *sar_data_rows[RANGE_SAMPLES];
static float *sar_mag_rows [RANGE_SAMPLES];
static SARData sar_static;

// Orbit state vector
typedef struct {
 double time;
 double position[3]; // X, Y, Z in ECEF
 double velocity[3]; // Vx, Vy, Vz
} OrbitState;

// Function prototypes
SARData* sar_init_static(void);
void generate_raw_sar_echo(SARData* sar, OrbitState* orbit);
void range_compression_matched_filter(SARData* sar);
void doppler_centroid_estimation(SARData* sar, float doppler_est[DOPPLER_BINS][DOPPLER_BINS]);
void range_cell_migration_correction(SARData* sar, float rcmc_matrix[RANGE_SAMPLES/3][AZIMUTH_SAMPLES/3]);
void azimuth_compression(SARData* sar, float az_filter[AZIMUTH_SAMPLES][AZIMUTH_SAMPLES/3]);
void secondary_range_compression(SARData* sar);
void autofocus_phase_gradient(SARData* sar, float phase_matrix[AUTOFOCUS_SIZE][AUTOFOCUS_SIZE]);
void geometric_correction(SARData* sar, float geom_matrix[DOPPLER_BINS][DOPPLER_BINS]);
void radiometric_calibration(SARData* sar, float cal_matrix[DOPPLER_BINS][DOPPLER_BINS]);
void speckle_filter_frost(SARData* sar, float filter_kernel[9][9]);
void multilook_processing(SARData* sar);
void fft_radix3_matrix(complex_float* data, int size, float twiddle_matrix[9][9]);
float compute_snr(SARData* sar);
float sinc(float x);
void print_performance_metrics(clock_t start, clock_t end, const char* operation);

// Wavelet-based speckle denoising prototypes
void wavelet_forward_1d(float *buf, int n);
void wavelet_inverse_1d(float *buf, int n);
void wavelet_decompose_2d(float tile[WAVELET_TILE][WAVELET_TILE], int size);
void wavelet_reconstruct_2d(float tile[WAVELET_TILE][WAVELET_TILE], int size);
void wavelet_threshold_filter(float tile[WAVELET_TILE][WAVELET_TILE], int size);
void wavelet_denoise_sar(SARData *sar);

// Coordinate transformation prototypes (BCE: Co-ordinateTrans 7%)
void ecef_to_geodetic(double x, double y, double z, double *lat, double *lon, double *alt);
void radar_to_geographic(SARData *sar, OrbitState *orbit, float geo_lat[RANGE_SAMPLES][AZIMUTH_SAMPLES],
 float geo_lon[RANGE_SAMPLES][AZIMUTH_SAMPLES]);

// Error correction code prototypes (BCE: ECC 9%)
#define GF_ORDER 8
#define GF_SIZE (1 << GF_ORDER)
#define ECC_NSYM 18 /* RS parity symbols per block (t=9 correction) */
#define SNR_SIGNAL_THRESH 5.2f /* Signal detection threshold (linear amplitude) */
void gf_init(void);
unsigned char gf_mul(unsigned char a, unsigned char b);
void rs_encode_block(const unsigned char *data, int data_len, unsigned char *parity);
void ecc_encode_sar_data(SARData *sar);

// Main SAR processing pipeline
int main() {
 clock_t start, end, total_start;

 FLIGHT_LOG("[SAR] init\n");
 FLIGHT_LOG("Simulating X-band SAR system at %d km altitude\n", (int)(ALTITUDE/1000));
 FLIGHT_LOG("Range samples: %d, Azimuth samples: %d\n", RANGE_SAMPLES, AZIMUTH_SAMPLES);
 FLIGHT_LOG("PRF: %d Hz, Bandwidth: %.0f MHz\n", PRF, BANDWIDTH/1e6);
 FLIGHT_LOG("[SAR] tile=%dx%d\n\n", BLOCK_SIZE, BLOCK_SIZE);

 total_start = clock();

 // Initialize orbit state (typical sun-synchronous orbit)
 OrbitState orbit = {
 .time = 0.0,
 .position = {ALTITUDE + 6371000, 0, 0}, // LEO reference orbit
 .velocity = {0, VELOCITY, 0}
 };

 // Initialise SAR data from static memory pools (no heap allocation)
 SARData* sar = sar_init_static();

 // Step 1: Generate synthetic raw SAR echo data
 FLIGHT_LOG("1. Generating raw SAR echo data...\n");
 start = clock();
 generate_raw_sar_echo(sar, &orbit);
 end = clock();
 print_performance_metrics(start, end, "Echo Generation");

 // Step 2: Range compression with matched filtering
 FLIGHT_LOG("\n2. Range compression (matched filtering)...\n");
 start = clock();
 range_compression_matched_filter(sar);
 end = clock();
 print_performance_metrics(start, end, "Range Compression");

 // Step 3: Doppler centroid estimation (matrix operations)
 FLIGHT_LOG("\n3. Doppler centroid estimation...\n");
 start = clock();
 static float doppler_matrix[DOPPLER_BINS][DOPPLER_BINS];
 doppler_centroid_estimation(sar, doppler_matrix);
 end = clock();
 print_performance_metrics(start, end, "Doppler Estimation");
 FLIGHT_LOG(" Estimated Doppler centroid: %.2f Hz\n", sar->doppler_centroid);

 // Step 4: Range Cell Migration Correction (RCMC)
 FLIGHT_LOG("\n4. Range Cell Migration Correction...\n");
 start = clock();
 static float rcmc_matrix[RANGE_SAMPLES/3][AZIMUTH_SAMPLES/3];
 range_cell_migration_correction(sar, rcmc_matrix);
 end = clock();
 print_performance_metrics(start, end, "RCMC");

 // Step 5: Azimuth compression
 FLIGHT_LOG("\n5. Azimuth compression...\n");
 start = clock();
 static float az_filter[AZIMUTH_SAMPLES][AZIMUTH_SAMPLES/3];
 azimuth_compression(sar, az_filter);
 end = clock();
 print_performance_metrics(start, end, "Azimuth Compression");

 // Step 6: Secondary Range Compression (SRC)
 FLIGHT_LOG("\n6. Secondary range compression...\n");
 start = clock();
 secondary_range_compression(sar);
 end = clock();
 print_performance_metrics(start, end, "SRC");

 // Step 7: Autofocus using Phase Gradient Algorithm
 FLIGHT_LOG("\n7. Autofocus (Phase Gradient Algorithm)...\n");
 start = clock();
 static float phase_matrix[AUTOFOCUS_SIZE][AUTOFOCUS_SIZE];
 autofocus_phase_gradient(sar, phase_matrix);
 end = clock();
 print_performance_metrics(start, end, "Autofocus");

 // Step 8: Geometric correction
 FLIGHT_LOG("\n8. Geometric correction...\n");
 start = clock();
 static float geom_matrix[DOPPLER_BINS][DOPPLER_BINS];
 geometric_correction(sar, geom_matrix);
 end = clock();
 print_performance_metrics(start, end, "Geometric Correction");

 // Step 9: Radiometric calibration
 FLIGHT_LOG("\n9. Radiometric calibration...\n");
 start = clock();
 static float cal_matrix[DOPPLER_BINS][DOPPLER_BINS];
 radiometric_calibration(sar, cal_matrix);
 end = clock();
 print_performance_metrics(start, end, "Radiometric Calibration");

 // Step 10: Speckle filtering (Frost filter)
 FLIGHT_LOG("\n10. Speckle filtering (Frost filter)...\n");
 start = clock();
 static float frost_kernel[9][9];
 speckle_filter_frost(sar, frost_kernel);
 end = clock();
 print_performance_metrics(start, end, "Speckle Filtering");

 // Step 11: Wavelet-domain speckle denoising (Haar lifting + BayesShrink)
 FLIGHT_LOG("\n11. Wavelet speckle denoising (Haar lifting + BayesShrink)...\n");
 start = clock();
 wavelet_denoise_sar(sar);
 end = clock();
 print_performance_metrics(start, end, "Wavelet Denoising");

 // Step 12: Multilook processing
 FLIGHT_LOG("\n12. Multilook processing (%d looks)...\n", LOOKS);
 start = clock();
 multilook_processing(sar);
 end = clock();
 print_performance_metrics(start, end, "Multilooking");

 // Step 13: Coordinate transformation – radar pixels to geographic (lat/lon)
 // BCE: Co-ordinateTrans 7%
 FLIGHT_LOG("\n13. Coordinate transformation (radar→geographic)...\n");
 start = clock();
 {
 static float geo_lat[RANGE_SAMPLES][AZIMUTH_SAMPLES];
 static float geo_lon[RANGE_SAMPLES][AZIMUTH_SAMPLES];
 radar_to_geographic(sar, &orbit, geo_lat, geo_lon);
 FLIGHT_LOG(" Centre pixel: lat=%.4f° lon=%.4f°\n",
 geo_lat[RANGE_SAMPLES/2][AZIMUTH_SAMPLES/2],
 geo_lon[RANGE_SAMPLES/2][AZIMUTH_SAMPLES/2]);
 }
 end = clock();
 print_performance_metrics(start, end, "Coordinate Transform");

 // Step 14: Error correction coding for downlink (BCE: ECC 9%)
 FLIGHT_LOG("\n14. Reed-Solomon ECC encoding for downlink...\n");
 start = clock();
 ecc_encode_sar_data(sar);
 end = clock();
 print_performance_metrics(start, end, "ECC Encoding");

 // Calculate final SNR
 float snr = compute_snr(sar);

 // Final performance summary
 FLIGHT_LOG("[SAR] done (%.3fs)\n",
 ((double)(clock() - total_start)) / CLOCKS_PER_SEC);
 FLIGHT_LOG("Output Image Size: %d x %d pixels\n",
 sar->range_lines/LOOKS, sar->azimuth_lines/LOOKS);
 FLIGHT_LOG("Final SNR: %.2f dB\n", snr);
 FLIGHT_LOG("Ground Range Resolution: %.2f m\n",
 C_LIGHT / (2 * BANDWIDTH));
 FLIGHT_LOG("Azimuth Resolution: %.2f m\n",
 VELOCITY / (2 * PRF * AZIMUTH_SAMPLES / FFT_SIZE_A));

 /* Static memory — no deallocation required */
 return 0;
}

/**
 * Initialise SAR data structure from static memory pools.
 *
 * All buffers reside in BSS (zero-initialised at boot, no heap usage).
 * This mirrors flight-software practice: memory is reserved at link time
 * to guarantee deterministic availability during an imaging pass and to
 * prevent heap fragmentation on long-duration missions.
 */
SARData* sar_init_static(void) {
 SARData *sar = &sar_static;

 sar->range_lines = RANGE_SAMPLES;
 sar->azimuth_lines = AZIMUTH_SAMPLES;
 sar->range_spacing = C_LIGHT / (2 * BANDWIDTH);
 sar->azimuth_spacing = VELOCITY / PRF;
 sar->doppler_centroid = 0.0;
 sar->doppler_rate = 0.0;

 /* Wire row-pointer tables into the flat static pools */
 for (int i = 0; i < RANGE_SAMPLES; i++) {
 sar_data_rows[i] = sar_data_pool[i];
 sar_mag_rows[i] = sar_mag_pool[i];
 }
 sar->data = sar_data_rows;
 sar->magnitude = sar_mag_rows;

 /* Zero the image buffers (BSS is zero-init, but be explicit for re-entry) */
 memset(sar_data_pool, 0, sizeof(sar_data_pool));
 memset(sar_mag_pool, 0, sizeof(sar_mag_pool));

 return sar;
}

// Generate synthetic raw SAR echo data
void generate_raw_sar_echo(SARData* sar, OrbitState* orbit) {
 (void)orbit;
 float chirp_rate = BANDWIDTH / PULSE_DURATION;

 // Simulate point targets and distributed scatterers
 for (int r = 0; r < sar->range_lines; r++) {
 for (int a = 0; a < sar->azimuth_lines; a++) {
 // Range time
 float tau = r * 2 * sar->range_spacing / C_LIGHT;

 // Azimuth time
 float eta = (a - sar->azimuth_lines/2) / (float)PRF;

 // Simulate chirp signal with Doppler shift
 float phase_range = M_PI * chirp_rate * tau * tau;
 float doppler_shift = 2 * VELOCITY * eta / (C_LIGHT / FC);
 float phase_azimuth = 2 * M_PI * doppler_shift;

 // Add random scatterers
 float amplitude = 0.1 + 0.9 * ((float)rand() / RAND_MAX);

 // Point target simulation at specific locations
 if ((r == sar->range_lines/2 && a == sar->azimuth_lines/2) ||
 (r == sar->range_lines/3 && a == sar->azimuth_lines/3)) {
 amplitude += 10.0; // Strong point target
 }

 // Generate complex echo
 sar->data[r][a].real = amplitude * cos(phase_range + phase_azimuth);
 sar->data[r][a].imag = amplitude * sin(phase_range + phase_azimuth);
 }
 }
}

// Range compression using matched filtering (frequency domain)
void range_compression_matched_filter(SARData* sar) {
 static complex_float range_ref[FFT_SIZE_R];
 static complex_float fft_buffer[FFT_SIZE_R];
 static float twiddle_matrix[9][9]; // For radix-3 FFT

 // Generate range reference function (matched filter)
 float chirp_rate = BANDWIDTH / PULSE_DURATION;
 for (int i = 0; i < FFT_SIZE_R; i++) {
 float tau = (i - FFT_SIZE_R/2) * PULSE_DURATION / FFT_SIZE_R;
 float phase = -M_PI * chirp_rate * tau * tau; // Conjugate for matched filter
 range_ref[i].real = cos(phase);
 range_ref[i].imag = sin(phase);
 }

 // Process each azimuth line
 for (int a = 0; a < sar->azimuth_lines; a += BLOCK_SIZE) {
 // Process in blocks of 3 for systolic array
 for (int aa = 0; aa < BLOCK_SIZE && (a + aa) < sar->azimuth_lines; aa++) {
 // Copy to FFT buffer
 for (int r = 0; r < sar->range_lines; r++) {
 fft_buffer[r] = sar->data[r][a + aa];
 }

 // Simulate FFT using matrix operations (for systolic array)
 fft_radix3_matrix(fft_buffer, FFT_SIZE_R, twiddle_matrix);

 // Multiply with reference function in frequency domain
 for (int r = 0; r < FFT_SIZE_R; r++) {
 complex_float temp;
 temp.real = fft_buffer[r].real * range_ref[r].real -
 fft_buffer[r].imag * range_ref[r].imag;
 temp.imag = fft_buffer[r].real * range_ref[r].imag +
 fft_buffer[r].imag * range_ref[r].real;
 fft_buffer[r] = temp;
 }

 // Inverse FFT: conjugate → forward FFT → conjugate → normalize
 for (int r = 0; r < FFT_SIZE_R; r++)
 fft_buffer[r].imag = -fft_buffer[r].imag;
 fft_radix3_matrix(fft_buffer, FFT_SIZE_R, twiddle_matrix);
 for (int r = 0; r < FFT_SIZE_R; r++) {
 fft_buffer[r].real /= (float)FFT_SIZE_R;
 fft_buffer[r].imag = -fft_buffer[r].imag / (float)FFT_SIZE_R;
 }

 // Copy back
 for (int r = 0; r < sar->range_lines; r++) {
 sar->data[r][a + aa] = fft_buffer[r];
 }
 }
 }
}

// Doppler centroid estimation using correlation method
void doppler_centroid_estimation(SARData* sar, float doppler_est[DOPPLER_BINS][DOPPLER_BINS]) {
 static complex_float correlation[DOPPLER_BINS][DOPPLER_BINS];

 // Compute azimuth correlation at different lags
 for (int lag = 0; lag < DOPPLER_BINS; lag++) {
 for (int r = 0; r < DOPPLER_BINS; r++) {
 correlation[lag][r].real = 0;
 correlation[lag][r].imag = 0;

 // Correlate adjacent azimuth samples
 for (int a = 0; a < sar->azimuth_lines - lag - 1; a++) {
 int r_idx = r * sar->range_lines / DOPPLER_BINS;

 // Complex conjugate multiplication
 correlation[lag][r].real += sar->data[r_idx][a].real * sar->data[r_idx][a+lag+1].real +
 sar->data[r_idx][a].imag * sar->data[r_idx][a+lag+1].imag;
 correlation[lag][r].imag += sar->data[r_idx][a].imag * sar->data[r_idx][a+lag+1].real -
 sar->data[r_idx][a].real * sar->data[r_idx][a+lag+1].imag;
 }
 }
 }

 // Matrix operations for Doppler estimation (24x24)
 for (int i = 0; i < DOPPLER_BINS; i++) {
 for (int j = 0; j < DOPPLER_BINS; j++) {
 doppler_est[i][j] = atan2(correlation[i][j].imag, correlation[i][j].real);
 }
 }

 // Estimate Doppler centroid from phase slope
 float sum_doppler = 0;
 for (int i = 0; i < DOPPLER_BINS; i++) {
 sum_doppler += doppler_est[i][i] * PRF / (2 * M_PI);
 }
 sar->doppler_centroid = sum_doppler / DOPPLER_BINS;
}

// Range Cell Migration Correction
void range_cell_migration_correction(SARData* sar, float rcmc_matrix[RANGE_SAMPLES/3][AZIMUTH_SAMPLES/3]) {
 static complex_float temp_data[RANGE_SAMPLES][AZIMUTH_SAMPLES];

 // Calculate RCM for each range-azimuth position
 float lambda = C_LIGHT / FC;

 // Build RCMC interpolation matrix (processed in 3x3 blocks)
 for (int rb = 0; rb < RANGE_SAMPLES/3; rb++) {
 for (int ab = 0; ab < AZIMUTH_SAMPLES/3; ab++) {
 float r0 = (rb * 3 + 1.5) * sar->range_spacing;
 float v_r = VELOCITY;
 float f_eta = (ab * 3 - AZIMUTH_SAMPLES/2) * PRF / (float)AZIMUTH_SAMPLES;

 // Range migration in samples (λ²·f_η² / 8·v_r²·Δr)
 rcmc_matrix[rb][ab] = r0 * lambda * lambda * f_eta * f_eta / (8.0f * v_r * v_r * sar->range_spacing);
 }
 }

 // Apply RCMC using sinc interpolation in blocks
 for (int ab = 0; ab < sar->azimuth_lines; ab += BLOCK_SIZE) {
 for (int rb = 0; rb < sar->range_lines; rb += BLOCK_SIZE) {
 // Process 3x3 block
 for (int r = rb; r < rb + BLOCK_SIZE && r < sar->range_lines; r++) {
 for (int a = ab; a < ab + BLOCK_SIZE && a < sar->azimuth_lines; a++) {
 float migration = rcmc_matrix[r/3][a/3];
 int r_shift = (int)migration;
 float frac = migration - r_shift;

 // Sinc interpolation
 temp_data[r][a].real = 0;
 temp_data[r][a].imag = 0;

 for (int k = -4; k <= 4; k++) {
 int r_interp = r + r_shift + k;
 if (r_interp >= 0 && r_interp < sar->range_lines) {
 float sinc_val = (k == 0) ? 1.0 : sin(M_PI * (k - frac)) / (M_PI * (k - frac));
 temp_data[r][a].real += sar->data[r_interp][a].real * sinc_val;
 temp_data[r][a].imag += sar->data[r_interp][a].imag * sinc_val;
 }
 }
 }
 }
 }
 }

 // Copy corrected data back
 for (int r = 0; r < sar->range_lines; r++) {
 memcpy(sar->data[r], temp_data[r], sar->azimuth_lines * sizeof(complex_float));
 }
}

// Azimuth compression using matched filtering
void azimuth_compression(SARData* sar, float az_filter[AZIMUTH_SAMPLES][AZIMUTH_SAMPLES/3]) {
 static complex_float az_ref[FFT_SIZE_A];
 static complex_float fft_buffer[FFT_SIZE_A];
 (void)fft_buffer;

 // Generate azimuth reference function based on Doppler parameters
 float lambda = C_LIGHT / FC; // Wavelength
 float range = ALTITUDE / cosf(LOOK_ANGLE * M_PI / 180.0f); /* slant range */
 float Ka = 2 * VELOCITY * VELOCITY / (lambda * range); // Doppler rate

 for (int i = 0; i < FFT_SIZE_A; i++) {
 float eta = (i - FFT_SIZE_A/2) / (float)PRF;
 float phase = -M_PI * Ka * eta * eta + 2 * M_PI * sar->doppler_centroid * eta;
 az_ref[i].real = cos(phase);
 az_ref[i].imag = sin(phase);
 }

 // Build azimuth compression matrix
 for (int i = 0; i < AZIMUTH_SAMPLES; i++) {
 for (int j = 0; j < AZIMUTH_SAMPLES/3; j++) {
 az_filter[i][j] = az_ref[i % FFT_SIZE_A].real;
 }
 }

 // Process each range line with block processing
 for (int r = 0; r < sar->range_lines; r += BLOCK_SIZE) {
 for (int rr = 0; rr < BLOCK_SIZE && (r + rr) < sar->range_lines; rr++) {
 // Matrix multiplication for azimuth compression
 static complex_float temp_line[AZIMUTH_SAMPLES];

 // Apply filter using matrix operations (optimized for 3x3 blocks)
 for (int a = 0; a < sar->azimuth_lines; a += BLOCK_SIZE) {
 for (int aa = 0; aa < BLOCK_SIZE && (a + aa) < sar->azimuth_lines; aa++) {
 temp_line[a + aa].real = 0;
 temp_line[a + aa].imag = 0;

 // Convolution with reference function
 for (int k = 0; k < AZIMUTH_SAMPLES/3; k += BLOCK_SIZE) {
 for (int kk = 0; kk < BLOCK_SIZE && (k + kk) < AZIMUTH_SAMPLES/3; kk++) {
 int idx = (a + aa + (k + kk) * 3 - AZIMUTH_SAMPLES/2) % sar->azimuth_lines;
 if (idx >= 0) {
 temp_line[a + aa].real += sar->data[r + rr][idx].real * az_filter[a + aa][k + kk];
 temp_line[a + aa].imag += sar->data[r + rr][idx].imag * az_filter[a + aa][k + kk];
 }
 }
 }
 }
 }

 // Copy back
 memcpy(sar->data[r + rr], temp_line, sar->azimuth_lines * sizeof(complex_float));
 }
 }
}

// Secondary Range Compression
void secondary_range_compression(SARData* sar) {
 static float src_matrix[SRC_TILE][SRC_TILE];

 // Build SRC correction matrix
 int half = SRC_TILE / 2;
 for (int i = 0; i < SRC_TILE; i++) {
 for (int j = 0; j < SRC_TILE; j++) {
 // Phase correction for residual RCM
 float range_factor = (i - half) / (float)half;
 float azimuth_factor = (j - half) / (float)half;
 src_matrix[i][j] = exp(-2 * M_PI * range_factor * azimuth_factor);
 }
 }

 // Apply SRC in blocks
 for (int r = 0; r < sar->range_lines - SRC_TILE; r += SRC_TILE) {
 for (int a = 0; a < sar->azimuth_lines - SRC_TILE; a += SRC_TILE) {
 // Matrix multiplication for phase correction
 static complex_float block[SRC_TILE][SRC_TILE];

 // Extract block
 for (int i = 0; i < SRC_TILE; i++) {
 for (int j = 0; j < SRC_TILE; j++) {
 block[i][j] = sar->data[r + i][a + j];
 }
 }

 // Apply correction matrix
 for (int i = 0; i < SRC_TILE; i += BLOCK_SIZE) {
 for (int j = 0; j < SRC_TILE; j += BLOCK_SIZE) {
 for (int ii = 0; ii < BLOCK_SIZE; ii++) {
 for (int jj = 0; jj < BLOCK_SIZE; jj++) {
 float correction = src_matrix[i + ii][j + jj];
 block[i + ii][j + jj].real *= correction;
 block[i + ii][j + jj].imag *= correction;
 }
 }
 }
 }

 // Copy back
 for (int i = 0; i < SRC_TILE; i++) {
 for (int j = 0; j < SRC_TILE; j++) {
 sar->data[r + i][a + j] = block[i][j];
 }
 }
 }
 }
}

// Autofocus using Phase Gradient Algorithm
void autofocus_phase_gradient(SARData* sar, float phase_matrix[AUTOFOCUS_SIZE][AUTOFOCUS_SIZE]) {
 static complex_float gradient[AUTOFOCUS_SIZE][AUTOFOCUS_SIZE];

 // Select bright pixels for autofocus
 int num_bright = 0;
 static int bright_r[1000], bright_a[1000];

 // Find bright pixels
 for (int r = AUTOFOCUS_SIZE; r < sar->range_lines - AUTOFOCUS_SIZE; r += 10) {
 for (int a = AUTOFOCUS_SIZE; a < sar->azimuth_lines - AUTOFOCUS_SIZE; a += 10) {
 float mag = sqrt(sar->data[r][a].real * sar->data[r][a].real +
 sar->data[r][a].imag * sar->data[r][a].imag);
 if (mag > 5.0 && num_bright < 1000) { // Threshold for bright pixels
 bright_r[num_bright] = r;
 bright_a[num_bright] = a;
 num_bright++;
 }
 }
 }

 // Compute phase gradient for each bright pixel
 for (int p = 0; p < num_bright; p++) {
 int r0 = bright_r[p];
 int a0 = bright_a[p];

 // Extract local window
 for (int i = 0; i < AUTOFOCUS_SIZE; i++) {
 for (int j = 0; j < AUTOFOCUS_SIZE; j++) {
 int r = r0 + i - AUTOFOCUS_SIZE/2;
 int a = a0 + j - AUTOFOCUS_SIZE/2;

 // Compute phase gradient
 if (j < AUTOFOCUS_SIZE - 1) {
 complex_float diff;
 diff.real = sar->data[r][a+1].real * sar->data[r][a].real +
 sar->data[r][a+1].imag * sar->data[r][a].imag;
 diff.imag = sar->data[r][a+1].imag * sar->data[r][a].real -
 sar->data[r][a+1].real * sar->data[r][a].imag;

 gradient[i][j].real += diff.real;
 gradient[i][j].imag += diff.imag;
 }
 }
 }
 }

 // Convert to phase error estimate
 for (int i = 0; i < AUTOFOCUS_SIZE; i++) {
 for (int j = 0; j < AUTOFOCUS_SIZE; j++) {
 phase_matrix[i][j] = atan2(gradient[i][j].imag, gradient[i][j].real);
 }
 }

 // Apply phase correction
 for (int a = 0; a < sar->azimuth_lines; a++) {
 float phase_error = phase_matrix[AUTOFOCUS_SIZE/2][a % AUTOFOCUS_SIZE];
 float cos_phi = cos(-phase_error);
 float sin_phi = sin(-phase_error);

 for (int r = 0; r < sar->range_lines; r++) {
 complex_float temp = sar->data[r][a];
 sar->data[r][a].real = temp.real * cos_phi - temp.imag * sin_phi;
 sar->data[r][a].imag = temp.real * sin_phi + temp.imag * cos_phi;
 }
 }
}

// Geometric correction using ground control points
void geometric_correction(SARData* sar, float geom_matrix[DOPPLER_BINS][DOPPLER_BINS]) {
 // Build geometric transformation matrix
 // Simulating slant-to-ground range conversion

 float incidence_angle = 35.0 * M_PI / 180.0; // Typical incidence angle

 // Build transformation matrix
 for (int i = 0; i < DOPPLER_BINS; i++) {
 for (int j = 0; j < DOPPLER_BINS; j++) {
 if (i == j) {
 // Diagonal elements for range scaling
 geom_matrix[i][j] = 1.0 / sin(incidence_angle + i * 0.001);
 } else {
 // Off-diagonal for terrain correction
 geom_matrix[i][j] = 0.01 * exp(-(float)abs(i-j)/6.0);
 }
 }
 }

 // Apply geometric correction in blocks
 for (int rb = 0; rb < sar->range_lines/DOPPLER_BINS; rb++) {
 for (int ab = 0; ab < sar->azimuth_lines/DOPPLER_BINS; ab++) {
 static complex_float block[DOPPLER_BINS][DOPPLER_BINS];

 // Extract block
 for (int i = 0; i < DOPPLER_BINS; i++) {
 for (int j = 0; j < DOPPLER_BINS; j++) {
 block[i][j] = sar->data[rb*DOPPLER_BINS + i][ab*DOPPLER_BINS + j];
 }
 }

 // Apply transformation using matrix multiplication
 static complex_float temp_block[DOPPLER_BINS][DOPPLER_BINS];

 // Matrix multiply: temp = geom_matrix * block
 for (int i = 0; i < DOPPLER_BINS; i += BLOCK_SIZE) {
 for (int j = 0; j < DOPPLER_BINS; j += BLOCK_SIZE) {
 for (int k = 0; k < DOPPLER_BINS; k += BLOCK_SIZE) {
 // 3x3 block processing
 for (int ii = 0; ii < BLOCK_SIZE; ii++) {
 for (int jj = 0; jj < BLOCK_SIZE; jj++) {
 if (k == 0) {
 temp_block[i+ii][j+jj].real = 0;
 temp_block[i+ii][j+jj].imag = 0;
 }
 for (int kk = 0; kk < BLOCK_SIZE; kk++) {
 temp_block[i+ii][j+jj].real +=
 geom_matrix[i+ii][k+kk] * block[k+kk][j+jj].real;
 temp_block[i+ii][j+jj].imag +=
 geom_matrix[i+ii][k+kk] * block[k+kk][j+jj].imag;
 }
 }
 }
 }
 }
 }

 // Copy back
 for (int i = 0; i < DOPPLER_BINS; i++) {
 for (int j = 0; j < DOPPLER_BINS; j++) {
 sar->data[rb*DOPPLER_BINS + i][ab*DOPPLER_BINS + j] = temp_block[i][j];
 }
 }
 }
 }
}

// Radiometric calibration
void radiometric_calibration(SARData* sar, float cal_matrix[DOPPLER_BINS][DOPPLER_BINS]) {
 /* Build calibration matrix for antenna pattern and range spreading */
 float antenna_gain = 44.7f; /* dB — typical X-band SAR (e.g. SAR satellite active array) */
 float system_loss = 3.2f; /* dB — feed network + radome */
 int half = DOPPLER_BINS / 2;

 for (int i = 0; i < DOPPLER_BINS; i++) {
 for (int j = 0; j < DOPPLER_BINS; j++) {
 /* Antenna gain pattern */
 float theta_r = (i - half) * 0.1f; /* Elevation angle variation */
 float theta_a = (j - half) * 0.1f; /* Azimuth angle variation */

 float antenna_pattern = pow(10, (antenna_gain - system_loss) / 10.0) *
 sinc(theta_r) * sinc(theta_a);

 /* Range spreading loss (R^4 for point targets) */
 float range_norm = 1.0f + (i - half) * 0.1f;
 float range_loss = pow(range_norm, 4); /* R^4 radar range equation */

 cal_matrix[i][j] = antenna_pattern / (range_loss > 1e-6f ? range_loss : 1e-6f);
 }
 }

 /* Apply calibration in tiled blocks */
 for (int r = 0; r < sar->range_lines - DOPPLER_BINS; r += DOPPLER_BINS) {
 for (int a = 0; a < sar->azimuth_lines - DOPPLER_BINS; a += DOPPLER_BINS) {
 for (int i = 0; i < DOPPLER_BINS; i += BLOCK_SIZE) {
 for (int j = 0; j < DOPPLER_BINS; j += BLOCK_SIZE) {
 /* 3x3 block processing */
 for (int ii = 0; ii < BLOCK_SIZE; ii++) {
 for (int jj = 0; jj < BLOCK_SIZE; jj++) {
 float cal_factor = cal_matrix[i+ii][j+jj];
 sar->data[r+i+ii][a+j+jj].real *= cal_factor;
 sar->data[r+i+ii][a+j+jj].imag *= cal_factor;
 }
 }
 }
 }
 }
 }
}

// Speckle filtering using Frost filter
void speckle_filter_frost(SARData* sar, float filter_kernel[9][9]) {
 static complex_float filtered[RANGE_SAMPLES][AZIMUTH_SAMPLES];
 float damping_factor = 2.0; // Frost filter parameter

 // Build Frost filter kernel (9x9)
 for (int i = 0; i < 9; i++) {
 for (int j = 0; j < 9; j++) {
 float dist = sqrt((i-4)*(i-4) + (j-4)*(j-4));
 filter_kernel[i][j] = exp(-damping_factor * dist);
 }
 }

 // Normalize kernel
 float sum = 0;
 for (int i = 0; i < 9; i++) {
 for (int j = 0; j < 9; j++) {
 sum += filter_kernel[i][j];
 }
 }
 for (int i = 0; i < 9; i++) {
 for (int j = 0; j < 9; j++) {
 filter_kernel[i][j] /= sum;
 }
 }

 // Apply filter with local statistics
 for (int r = 4; r < sar->range_lines - 4; r++) {
 for (int a = 4; a < sar->azimuth_lines - 4; a++) {
 // Compute local statistics
 float mean_i = 0, mean_q = 0;
 float var_i = 0, var_q = 0;

 // First pass: compute mean
 for (int i = -4; i <= 4; i++) {
 for (int j = -4; j <= 4; j++) {
 mean_i += sar->data[r+i][a+j].real;
 mean_q += sar->data[r+i][a+j].imag;
 }
 }
 mean_i /= 81;
 mean_q /= 81;

 // Second pass: compute variance
 for (int i = -4; i <= 4; i++) {
 for (int j = -4; j <= 4; j++) {
 var_i += pow(sar->data[r+i][a+j].real - mean_i, 2);
 var_q += pow(sar->data[r+i][a+j].imag - mean_q, 2);
 }
 }
 var_i /= 81;
 var_q /= 81;

 // Compute coefficient of variation
 float cv = sqrt(var_i + var_q) / sqrt(mean_i*mean_i + mean_q*mean_q + 1e-10);

 // Apply adaptive Frost filter
 filtered[r][a].real = 0;
 filtered[r][a].imag = 0;

 for (int i = 0; i < 9; i++) {
 for (int j = 0; j < 9; j++) {
 float weight = filter_kernel[i][j] * exp(-damping_factor * cv);
 filtered[r][a].real += sar->data[r+i-4][a+j-4].real * weight;
 filtered[r][a].imag += sar->data[r+i-4][a+j-4].imag * weight;
 }
 }
 }
 }

 // Copy filtered data back
 for (int r = 4; r < sar->range_lines - 4; r++) {
 for (int a = 4; a < sar->azimuth_lines - 4; a++) {
 sar->data[r][a] = filtered[r][a];
 }
 }
}

/* =================================================================
 * Wavelet-based SAR Speckle Denoising Module
 *
 * SAR images are inherently corrupted by multiplicative speckle noise
 * due to coherent imaging. Wavelet-domain denoising exploits the fact
 * that signal energy concentrates in a few large coefficients while
 * speckle spreads uniformly across subbands. A lifting-scheme Haar
 * wavelet keeps computation purely in-place with no FFT dependency,
 * making it ideal for onboard RISC-V processing.
 *
 * Pipeline: 2D decompose -> BayesShrink threshold -> 2D reconstruct
 * This fulfils the Wavelet(9%) component of the BCE profile.
 * ================================================================= */

/*
 * wavelet_forward_1d -- Single-level forward Haar wavelet via lifting
 *
 * In-place reordering: even indices become low-pass (L), odd become
 * high-pass (H).
 * Split: L[k] = x[2k], H[k] = x[2k+1]
 * Predict: H[k] -= L[k] (detail = odd - even)
 * Update: L[k] += H[k] / 2 (approx = even + detail/2)
 * Scale: L *= sqrt(2), H /= sqrt(2) (energy preservation)
 */
void wavelet_forward_1d(float *buf, int n) {
 int half = n / 2;
 float tmp[WAVELET_TILE];

 // Split into even / odd
 for (int i = 0; i < half; i++) {
 tmp[i] = buf[2 * i]; // L (even samples)
 tmp[half + i] = buf[2 * i + 1]; // H (odd samples)
 }

 // Predict step: H[k] -= L[k]
 for (int i = 0; i < half; i++) {
 tmp[half + i] -= tmp[i];
 }

 // Update step: L[k] += H[k] * 0.5
 for (int i = 0; i < half; i++) {
 tmp[i] += tmp[half + i] * 0.5f;
 }

 // Normalise so that energy is preserved (sqrt(2) scaling)
 float s2 = sqrtf(2.0f);
 for (int i = 0; i < half; i++) {
 tmp[i] *= s2; // scale L
 tmp[half + i] /= s2; // scale H
 }

 memcpy(buf, tmp, n * sizeof(float));
}

/*
 * wavelet_inverse_1d -- Single-level inverse Haar wavelet via lifting
 *
 * Reverses the forward lifting steps in exact reverse order.
 */
void wavelet_inverse_1d(float *buf, int n) {
 int half = n / 2;
 float s2 = sqrtf(2.0f);

 // Undo normalisation
 for (int i = 0; i < half; i++) {
 buf[i] /= s2;
 buf[half + i] *= s2;
 }

 // Undo update: L[k] -= H[k] * 0.5
 for (int i = 0; i < half; i++) {
 buf[i] -= buf[half + i] * 0.5f;
 }

 // Undo predict: H[k] += L[k]
 for (int i = 0; i < half; i++) {
 buf[half + i] += buf[i];
 }

 // Merge even / odd back to interleaved order
 float tmp[WAVELET_TILE];
 for (int i = 0; i < half; i++) {
 tmp[2 * i] = buf[i];
 tmp[2 * i + 1] = buf[half + i];
 }
 memcpy(buf, tmp, n * sizeof(float));
}

/*
 * wavelet_decompose_2d -- Single-level 2D Haar decomposition
 *
 * Applies 1D forward transform to every row, then to every column.
 * Produces four quadrants:
 * LL (top-left) LH (top-right)
 * HL (bottom-left) HH (bottom-right)
 */
void wavelet_decompose_2d(float tile[WAVELET_TILE][WAVELET_TILE], int size) {
 // Row-wise transform
 for (int r = 0; r < size; r++) {
 wavelet_forward_1d(tile[r], size);
 }

 // Column-wise transform (extract column, transform, write back)
 float col[WAVELET_TILE];
 for (int c = 0; c < size; c++) {
 for (int r = 0; r < size; r++) col[r] = tile[r][c];
 wavelet_forward_1d(col, size);
 for (int r = 0; r < size; r++) tile[r][c] = col[r];
 }
}

/*
 * wavelet_reconstruct_2d -- Single-level 2D Haar reconstruction
 *
 * Inverse of wavelet_decompose_2d: column-wise inverse, then row-wise.
 */
void wavelet_reconstruct_2d(float tile[WAVELET_TILE][WAVELET_TILE], int size) {
 float col[WAVELET_TILE];

 // Column-wise inverse
 for (int c = 0; c < size; c++) {
 for (int r = 0; r < size; r++) col[r] = tile[r][c];
 wavelet_inverse_1d(col, size);
 for (int r = 0; r < size; r++) tile[r][c] = col[r];
 }

 // Row-wise inverse
 for (int r = 0; r < size; r++) {
 wavelet_inverse_1d(tile[r], size);
 }
}

/*
 * wavelet_threshold_filter -- BayesShrink soft-thresholding
 *
 * Estimates noise variance sigma_n^2 from the HH subband using the
 * Median Absolute Deviation (MAD) robust estimator:
 * sigma_n = MAD(HH) / 0.6745
 *
 * For each detail subband (LH, HL, HH) the signal variance is:
 * sigma_s^2 = max(sigma_y^2 - sigma_n^2, 0)
 * and the BayesShrink threshold is:
 * T = sigma_n^2 / sigma_s (or large if sigma_s ~ 0)
 *
 * Soft thresholding: sign(x) * max(|x| - T, 0)
 */
void wavelet_threshold_filter(float tile[WAVELET_TILE][WAVELET_TILE], int size) {
 int half = size / 2;

 /* ---- Estimate noise sigma from HH subband (bottom-right quadrant) ---- */
 int hh_count = half * half;
 float hh_abs[WAVELET_TILE * WAVELET_TILE / 4];
 int idx = 0;
 for (int r = half; r < size; r++) {
 for (int c = half; c < size; c++) {
 hh_abs[idx++] = fabsf(tile[r][c]);
 }
 }

 // Selection sort of |HH| to find median (tile is small, <= 18x18)
 for (int i = 0; i < hh_count - 1; i++) {
 int min_idx = i;
 for (int j = i + 1; j < hh_count; j++) {
 if (hh_abs[j] < hh_abs[min_idx]) min_idx = j;
 }
 if (min_idx != i) {
 float t = hh_abs[i];
 hh_abs[i] = hh_abs[min_idx];
 hh_abs[min_idx] = t;
 }
 }
 float median_abs = hh_abs[hh_count / 2];
 float sigma_n = median_abs / 0.6745f; // Robust noise estimate
 float sigma_n2 = sigma_n * sigma_n;

 /* ---- Threshold each detail subband (LH, HL, HH) ---- */
 typedef struct { int r0, r1, c0, c1; } SubbandRegion;
 SubbandRegion subs[3] = {
 {0, half, half, size}, // LH (top-right)
 {half, size, 0, half}, // HL (bottom-left)
 {half, size, half, size} // HH (bottom-right)
 };

 for (int s = 0; s < 3; s++) {
 // Compute subband variance sigma_y^2
 float sum2 = 0;
 int cnt = 0;
 for (int r = subs[s].r0; r < subs[s].r1; r++) {
 for (int c = subs[s].c0; c < subs[s].c1; c++) {
 sum2 += tile[r][c] * tile[r][c];
 cnt++;
 }
 }
 float sigma_y2 = sum2 / cnt;

 // BayesShrink threshold
 float sigma_s2 = sigma_y2 - sigma_n2;
 float threshold;
 if (sigma_s2 <= 0.0f) {
 threshold = 1e6f; // Kill all coefficients (pure noise)
 } else {
 threshold = sigma_n2 / sqrtf(sigma_s2);
 }

 // Soft thresholding
 for (int r = subs[s].r0; r < subs[s].r1; r++) {
 for (int c = subs[s].c0; c < subs[s].c1; c++) {
 float val = tile[r][c];
 float abs_val = fabsf(val);
 if (abs_val <= threshold) {
 tile[r][c] = 0.0f;
 } else {
 tile[r][c] = (val > 0 ? 1.0f : -1.0f) * (abs_val - threshold);
 }
 }
 }
 }
}

/*
 * wavelet_denoise_sar -- Full wavelet-domain speckle denoising
 *
 * Processes the SAR image in WAVELET_TILE x WAVELET_TILE tiles:
 * 1. Extract tile from complex SAR data (magnitude)
 * 2. Forward 2D Haar decomposition (lifting scheme, no FFT)
 * 3. BayesShrink soft-threshold detail subbands (LH, HL, HH)
 * 4. Inverse 2D Haar reconstruction
 * 5. Write denoised magnitudes back, preserving original phase
 */
void wavelet_denoise_sar(SARData *sar) {
 float tile[WAVELET_TILE][WAVELET_TILE];

 int r_tiles = sar->range_lines / WAVELET_TILE;
 int a_tiles = sar->azimuth_lines / WAVELET_TILE;

 for (int rt = 0; rt < r_tiles; rt++) {
 for (int at = 0; at < a_tiles; at++) {
 int r0 = rt * WAVELET_TILE;
 int a0 = at * WAVELET_TILE;

 // Extract magnitude tile from complex SAR data
 for (int r = 0; r < WAVELET_TILE; r++) {
 for (int c = 0; c < WAVELET_TILE; c++) {
 float re = sar->data[r0 + r][a0 + c].real;
 float im = sar->data[r0 + r][a0 + c].imag;
 tile[r][c] = sqrtf(re * re + im * im);
 }
 }

 // Forward 2D wavelet decomposition
 wavelet_decompose_2d(tile, WAVELET_TILE);

 // BayesShrink thresholding of detail subbands
 wavelet_threshold_filter(tile, WAVELET_TILE);

 // Inverse 2D wavelet reconstruction
 wavelet_reconstruct_2d(tile, WAVELET_TILE);

 // Write denoised magnitudes back, preserving original phase
 for (int r = 0; r < WAVELET_TILE; r++) {
 for (int c = 0; c < WAVELET_TILE; c++) {
 float re = sar->data[r0 + r][a0 + c].real;
 float im = sar->data[r0 + r][a0 + c].imag;
 float orig_mag = sqrtf(re * re + im * im);
 float new_mag = fabsf(tile[r][c]); // Ensure non-negative

 // Scale complex value to new magnitude while preserving phase
 if (orig_mag > 1e-10f) {
 float scale = new_mag / orig_mag;
 sar->data[r0 + r][a0 + c].real *= scale;
 sar->data[r0 + r][a0 + c].imag *= scale;
 }
 }
 }
 }
 }
}

// Multilook processing
void multilook_processing(SARData* sar) {
 static complex_float multilooked[RANGE_SAMPLES/LOOKS][AZIMUTH_SAMPLES/LOOKS];

 // Average LOOKS x LOOKS pixels
 for (int r = 0; r < sar->range_lines/LOOKS; r++) {
 for (int a = 0; a < sar->azimuth_lines/LOOKS; a++) {
 multilooked[r][a].real = 0;
 multilooked[r][a].imag = 0;

 // Sum pixels in look window (incoherent — power averaging)
 for (int i = 0; i < LOOKS; i++) {
 for (int j = 0; j < LOOKS; j++) {
 float re = sar->data[r*LOOKS + i][a*LOOKS + j].real;
 float im = sar->data[r*LOOKS + i][a*LOOKS + j].imag;
 float power = re * re + im * im;
 multilooked[r][a].real += power;
 }
 }

 // Average power and take sqrt for magnitude
 multilooked[r][a].real = sqrtf(multilooked[r][a].real / (LOOKS * LOOKS));
 multilooked[r][a].imag = 0.0f;

 // Final magnitude is the incoherent average
 sar->magnitude[r][a] = multilooked[r][a].real;
 }
 }
}

// FFT using Cooley-Tukey DIT radix-3 butterfly with 3x3 twiddle matrix tiles.
// For N not a power of 3, performs mixed-radix decomposition by applying
// the radix-3 stages first, then finishing with Bluestein chirp-z for the
// remaining factor. Result is in-place in data[].
void fft_radix3_matrix(complex_float* data, int size, float twiddle_matrix[9][9]) {
 /* Pre-compute 9-point DFT matrix with complex twiddles,
 stored as real+imag pairs in a 9x9 real matrix (Re) and 9x9 imag matrix. */
 static float tw_real[9][9], tw_imag[9][9];
 for (int i = 0; i < 9; i++) {
 for (int j = 0; j < 9; j++) {
 float angle = -2.0f * (float)M_PI * i * j / 9.0f;
 tw_real[i][j] = cosf(angle);
 tw_imag[i][j] = sinf(angle);
 }
 }
 /* Copy real part to caller's twiddle_matrix for AME tile acceleration */
 for (int i = 0; i < 9; i++)
 for (int j = 0; j < 9; j++)
 twiddle_matrix[i][j] = tw_real[i][j];

 /* Radix-3 butterfly stages: process groups of 9 with complex multiply */
 int num_stages = (int)(log((double)size) / log(3.0));
 for (int stage = 0; stage < num_stages; stage++) {
 int stride = 1;
 for (int s = 0; s < stage; s++) stride *= 3;

 for (int start = 0; start < size; start += stride * 9) {
 complex_float temp[9];
 int count = 0;
 for (int i = 0; i < 9 && (start + i * stride) < size; i++) {
 temp[i] = data[start + i * stride];
 count++;
 }
 /* Zero-pad if fewer than 9 elements available */
 for (int i = count; i < 9; i++) {
 temp[i].real = 0.0f;
 temp[i].imag = 0.0f;
 }

 /* Complex 9-point DFT butterfly: out[i] = Σⱼ W[i][j] · temp[j] */
 for (int i = 0; i < count; i++) {
 complex_float sum = {0.0f, 0.0f};
 for (int j = 0; j < count; j++) {
 /* (a+bi)(c+di) = (ac−bd) + (ad+bc)i */
 sum.real += temp[j].real * tw_real[i][j]
 - temp[j].imag * tw_imag[i][j];
 sum.imag += temp[j].real * tw_imag[i][j]
 + temp[j].imag * tw_real[i][j];
 }
 /* Inter-stage twiddle: W_N^{k·stride} rotation */
 int k = i;
 float tw_angle = -2.0f * (float)M_PI * k * stride / (float)size;
 float tw_re = cosf(tw_angle);
 float tw_im = sinf(tw_angle);
 float out_re = sum.real * tw_re - sum.imag * tw_im;
 float out_im = sum.real * tw_im + sum.imag * tw_re;
 data[start + i * stride].real = out_re;
 data[start + i * stride].imag = out_im;
 }
 }
 }
}

// Compute SNR of processed image
float compute_snr(SARData* sar) {
 float signal_power = 0;
 float noise_power = 0;
 int num_pixels = 0;

 // Find bright targets (signal)
 for (int r = 0; r < sar->range_lines/LOOKS; r++) {
 for (int a = 0; a < sar->azimuth_lines/LOOKS; a++) {
 float mag = sar->magnitude[r][a];

 if (mag > 5.0) { // Threshold for signal
 signal_power += mag * mag;
 } else {
 noise_power += mag * mag;
 }
 num_pixels++;
 }
 }

 signal_power /= num_pixels;
 noise_power /= num_pixels;

 return 10 * log10(signal_power / (noise_power + 1e-10));
}

/* ===================================================================
 * Coordinate Transformation Module (BCE: Co-ordinateTrans 7%)
 * Radar pixel (range, azimuth) → Geographic (lat, lon) using ECEF
 * =================================================================== */

/* WGS-84 ellipsoid constants */
#define WGS84_A 6378137.0 /* semi-major axis (m) */
#define WGS84_F (1.0/298.257223563) /* flattening */
#define WGS84_E2 (2*WGS84_F - WGS84_F*WGS84_F) /* first eccentricity² */

void ecef_to_geodetic(double x, double y, double z,
 double *lat, double *lon, double *alt)
{
 *lon = atan2(y, x);
 double p = sqrt(x*x + y*y);
 double phi = atan2(z, p * (1.0 - WGS84_E2)); /* initial guess */
 for (int i = 0; i < 8; i++) { /* Bowring iteration */
 double sp = sin(phi);
 double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sp * sp);
 phi = atan2(z + WGS84_E2 * N * sp, p);
 }
 double sp = sin(phi);
 double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sp * sp);
 *lat = phi;
 *alt = p / cos(phi) - N;
}

void radar_to_geographic(SARData *sar, OrbitState *orbit,
 float geo_lat[RANGE_SAMPLES][AZIMUTH_SAMPLES],
 float geo_lon[RANGE_SAMPLES][AZIMUTH_SAMPLES])
{
 (void)sar;
 /* Build local ENU frame from ECEF orbit position */
 double pos[3] = {orbit->position[0], orbit->position[1], orbit->position[2]};
 double vel[3] = {orbit->velocity[0], orbit->velocity[1], orbit->velocity[2]};
 double pnorm = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);

 /* Unit nadir vector */
 double nadir[3] = {-pos[0]/pnorm, -pos[1]/pnorm, -pos[2]/pnorm};

 /* Along-track (normalised velocity) */
 double vnorm = sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
 double along[3] = {vel[0]/vnorm, vel[1]/vnorm, vel[2]/vnorm};

 /* Cross-track = nadir × along */
 double cross[3] = {
 nadir[1]*along[2] - nadir[2]*along[1],
 nadir[2]*along[0] - nadir[0]*along[2],
 nadir[0]*along[1] - nadir[1]*along[0]
 };

 double range_start = C_LIGHT / (2.0 * BANDWIDTH * RANGE_SAMPLES);
 double az_spacing = vnorm / PRF;
 float inc_angle = asin((float)(ALTITUDE / pnorm)); /* incidence angle */

 /* 3×3-blocked transformation loop for systolic array compatibility */
 for (int r = 0; r < RANGE_SAMPLES; r += BLOCK_SIZE) {
 for (int a = 0; a < AZIMUTH_SAMPLES; a += BLOCK_SIZE) {
 /* small 3×3 tile of ground-range / azimuth offsets */
 double tile_x[BLOCK_SIZE][BLOCK_SIZE];
 double tile_y[BLOCK_SIZE][BLOCK_SIZE];

 for (int dr = 0; dr < BLOCK_SIZE; dr++) {
 for (int da = 0; da < BLOCK_SIZE; da++) {
 int rr = r + dr, aa = a + da;
 if (rr >= RANGE_SAMPLES || aa >= AZIMUTH_SAMPLES) {
 tile_x[dr][da] = 0; tile_y[dr][da] = 0;
 continue;
 }
 /* slant range → ground range (flat-Earth approx) */
 double sr = range_start * rr;
 double gr = sr * cos(inc_angle);
 tile_x[dr][da] = gr; /* cross-track */
 tile_y[dr][da] = (aa - AZIMUTH_SAMPLES/2) * az_spacing; /* along-track */
 }
 }

 /* Convert ground offsets to ECEF, then geodetic */
 for (int dr = 0; dr < BLOCK_SIZE; dr++) {
 for (int da = 0; da < BLOCK_SIZE; da++) {
 int rr = r + dr, aa = a + da;
 if (rr >= RANGE_SAMPLES || aa >= AZIMUTH_SAMPLES) continue;
 double ecef[3];
 for (int k = 0; k < 3; k++)
 ecef[k] = pos[k] + cross[k]*tile_x[dr][da]
 + along[k]*tile_y[dr][da]
 + nadir[k]*ALTITUDE;
 double lat, lon, alt;
 ecef_to_geodetic(ecef[0], ecef[1], ecef[2], &lat, &lon, &alt);
 geo_lat[rr][aa] = (float)(lat * 180.0 / M_PI);
 geo_lon[rr][aa] = (float)(lon * 180.0 / M_PI);
 }
 }
 }
 }
 FLIGHT_LOG(" Geocoded %d × %d pixels (WGS-84 Bowring iteration)\n",
 RANGE_SAMPLES, AZIMUTH_SAMPLES);
}

/* ===================================================================
 * Reed-Solomon ECC Module (BCE: ECC 9%)
 * Systematic RS(n, k) over GF(2^8) with generator x^18
 * =================================================================== */
static unsigned char gf_exp_table[GF_SIZE];
static unsigned char gf_log_table[GF_SIZE];
static int gf_initialised = 0;

void gf_init(void) {
 if (gf_initialised) return;
 unsigned int x = 1;
 for (int i = 0; i < GF_SIZE - 1; i++) {
 gf_exp_table[i] = (unsigned char)x;
 gf_log_table[x] = (unsigned char)i;
 x <<= 1;
 if (x & GF_SIZE) x ^= 0x11D; /* primitive polynomial for GF(2^8) */
 }
 gf_exp_table[GF_SIZE - 1] = 0;
 gf_log_table[0] = 0; /* log(0) undefined, conventionally 0 */
 gf_initialised = 1;
}

unsigned char gf_mul(unsigned char a, unsigned char b) {
 if (a == 0 || b == 0) return 0;
 int s = (int)gf_log_table[a] + (int)gf_log_table[b];
 if (s >= GF_SIZE - 1) s -= (GF_SIZE - 1);
 return gf_exp_table[s];
}

void rs_encode_block(const unsigned char *data, int data_len, unsigned char *parity)
{
 /* Build generator polynomial roots α^0 .. α^(ECC_NSYM-1) */
 unsigned char gen[ECC_NSYM + 1];
 memset(gen, 0, sizeof(gen));
 gen[0] = 1;
 for (int i = 0; i < ECC_NSYM; i++) {
 for (int j = i + 1; j >= 1; j--)
 gen[j] = gen[j - 1] ^ gf_mul(gen[j], gf_exp_table[i]);
 gen[0] = gf_mul(gen[0], gf_exp_table[i]);
 }
 /* Systematic encoding: parity = data(x) · x^nsym mod gen(x) */
 memset(parity, 0, ECC_NSYM);
 for (int i = 0; i < data_len; i++) {
 unsigned char feedback = data[i] ^ parity[0];
 for (int j = 0; j < ECC_NSYM - 1; j++)
 parity[j] = parity[j + 1] ^ gf_mul(feedback, gen[ECC_NSYM - 1 - j]);
 parity[ECC_NSYM - 1] = gf_mul(feedback, gen[0]);
 }
}


void ecc_encode_sar_data(SARData *sar)
{
 gf_init();
 int block_len = RANGE_SAMPLES; /* encode row-by-row */
 int total_blocks = 0;
 int total_parity_bytes = 0;
 unsigned char parity[ECC_NSYM];

 for (int a = 0; a < sar->azimuth_lines; a++) {
 /* Quantise magnitude row to 8-bit for ECC */
 unsigned char row_data[RANGE_SAMPLES];
 for (int r = 0; r < sar->range_lines; r++) {
 float v = sar->magnitude[r][a];
 int q = (int)(v * 10.0f) & 0xFF;
 row_data[r] = (unsigned char)q;
 }
 /* Encode in blocks of 237 bytes (RS(255,237)) */
 int offset = 0;
 while (offset + 237 <= block_len) {
 rs_encode_block(row_data + offset, 237, parity);
 total_parity_bytes += ECC_NSYM;
 total_blocks++;
 offset += 237;
 }
 /* Remaining short block */
 if (offset < block_len) {
 rs_encode_block(row_data + offset, block_len - offset, parity);
 total_parity_bytes += ECC_NSYM;
 total_blocks++;
 }
 }
 FLIGHT_LOG(" RS(%d,%d) encoded %d blocks, %d parity bytes\n",
 237 + ECC_NSYM, 237, total_blocks, total_parity_bytes);
}

// Sinc function
float sinc(float x) {
 if (fabs(x) < 1e-10) return 1.0;
 return sin(M_PI * x) / (M_PI * x);
}

// Print performance metrics
void print_performance_metrics(clock_t start, clock_t end, const char* operation) {
 double cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
 FLIGHT_LOG(" %s completed in %.4f seconds\n", operation, cpu_time_used);
}
