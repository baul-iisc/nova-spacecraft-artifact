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
 * Satellite Communication Signal Processing
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Trig (CORDIC accelerator for trigonometric functions)
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard digital signal processing for satellite communication
 * payloads. This workload performs channel equalization, FFT-based spectral
 * analysis, digital beamforming with matrix operations, and link budget
 * computation with orbital geometry. Representative of GEO communication satellite
 * communication satellite digital payload processors and the future data relay satellite system
 * (Indian Data Relay Satellite System) inter-satellite link processing.
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
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h> // For gettimeofday
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
/* CORDIC trig redirects — float (single-precision) wrappers */
#define sinf(x) (float)hw_sin((double)(x))
#define cosf(x) (float)hw_cos((double)(x))
#define atan2f(y,x) (float)hw_atan2((double)(y),(double)(x))
#define acosf(x) (float)hw_acos((double)(x))
#define asinf(x) (float)hw_asin((double)(x))
#define tanf(x) (float)hw_tan((double)(x))
/* CORDIC trig redirects — double-precision (orbital mechanics) */
#define sin(x) hw_sin(x)
#define cos(x) hw_cos(x)
#define atan2(y,x) hw_atan2(y,x)
#define acos(x) hw_acos(x)

// Configuration parameters
#define MATRIX_SIZE 16 // 16x16 channel estimation matrices (MIMO power-of-2)
#define TILE_SIZE 3 // Size of matrix accelerator tiles (3x3)
#define SIGNAL_LENGTH 8192 // Length of simulated signal
#define TIMESTEPS 100 // Number of iterations for timing
#define MODULATION_ORDER 16 // 16-QAM
#define ERROR_THRESHOLD 1e-4f // Max acceptable HW-SW matmul divergence
#define COMPRESSION_BLOCKS 64 // Number of blocks for compression
/* Reduced DFT length for gem5 benchmark profiling (flight: 512+) */
#define FFT_LEN 64 // DFT analysis length (power of 2)
#define WAVELET_LEN 256 // Haar DWT length (power of 2)

/* Newton-Raphson convergence control for MIMO channel inversion */
#define NR_MAX_ITER 8 /* Maximum Newton-Raphson refinements */
#define NR_CONV_TOL 1e-4f /* Frobenius-norm convergence threshold */
#define NR_DIVERGE_RATIO 2.0f /* Divergence if residual grows by this factor */

/* ------------------------------------------------------------------ */
/* FDIR: Flight-software fault detection, isolation and recovery */
/* In a real OBC this triggers a Watchdog Reset or switchover to the */
/* redundant CPU card. Here we log and continue for gem5 */
/* characterisation reproducibility. */
/* ------------------------------------------------------------------ */
typedef enum { FDIR_OK = 0, FDIR_WARN, FDIR_ERR_CONVERGE } FdirStatus;
static int fdir_fault_count = 0;

static void fdir_report(FdirStatus status, const char *subsystem,
 const char *detail)
{
 if (status == FDIR_OK) return;
 fdir_fault_count++;
 const char *sev = (status == FDIR_WARN) ? "WARN" : "FAULT";
 FLIGHT_LOG("[FDIR][%s] %s: %s (cumulative faults: %d)\n",
 sev, subsystem, detail, fdir_fault_count);
 /* Flight: OBC_WDT_KICK(); if (fdir_fault_count > THRESHOLD)
 OBC_SWITCH_TO_REDUNDANT(); */
}

/* ------------------------------------------------------------------ */
/* Static BSS pools — pre-allocated at init (MISRA 21.3: no dynamic */
/* allocation after initialisation to prevent heap fragmentation) */
/* ------------------------------------------------------------------ */
#define MAX_MAT_DIM MATRIX_SIZE
#define COMPRESS_BLK_TOTAL (COMPRESSION_BLOCKS * MATRIX_SIZE * MATRIX_SIZE)

/* execute_matrix_pipeline */
static float mat_pool_A[MAX_MAT_DIM][MAX_MAT_DIM];
static float mat_pool_B[MAX_MAT_DIM][MAX_MAT_DIM];
static float mat_pool_C[MAX_MAT_DIM][MAX_MAT_DIM];

/* process_qam_modulation */
static float qam_data_bits[SIGNAL_LENGTH];
static float qam_i_signal[SIGNAL_LENGTH];
static float qam_q_signal[SIGNAL_LENGTH];
static float qam_demod_bits[SIGNAL_LENGTH];

/* compress_telemetry_data */
static float compress_input[COMPRESS_BLK_TOTAL];
static float compress_dct[COMPRESS_BLK_TOTAL];
static float compress_quant[COMPRESS_BLK_TOTAL];
static float compress_recon[COMPRESS_BLK_TOTAL];

/* equalize_signal_adaptive */
static float eq_input[SIGNAL_LENGTH];
static float eq_desired[SIGNAL_LENGTH];
static float eq_output[SIGNAL_LENGTH];
static float eq_error[SIGNAL_LENGTH];

/* haar wavelet scratch */
static float haar_tmp_buf[WAVELET_LEN];

/* process_wavelet_denoise */
static float wav_sig[WAVELET_LEN];
static float wav_ref[WAVELET_LEN];
static float wav_noisy[WAVELET_LEN];

// File-scope buffer for HW-vs-SW result comparison (P0 fix)
static float sw_result_buf[64 * 64]; // big enough for any tested size
static int sw_result_size = 0;

// Forward declaration
double get_time_ms(void);

// DFT spectrum analysis for signal processing
void compute_signal_dft(float *signal, int len, float *magnitude) {
 for (int k = 0; k < len; k++) {
 float re = 0.0f, im = 0.0f;
 for (int n = 0; n < len; n++) {
 float angle = 2.0f * M_PI * k * n / len;
 re += signal[n] * cosf(angle);
 im -= signal[n] * sinf(angle);
 }
 magnitude[k] = sqrtf(re * re + im * im);
 }
}

void process_signal_spectrum(int use_accelerator) {
 (void)use_accelerator; /* DFT kernel is pure SW; param kept for API uniformity */
 FLIGHT_LOG("\n[COM] DFT spectrum\n");
 double start = get_time_ms();
 float signal[FFT_LEN], magnitude[FFT_LEN];
 // Analyze multiple signal segments
 for (int seg = 0; seg < TIMESTEPS; seg++) {
 for (int i = 0; i < FFT_LEN; i++)
 signal[i] = sinf(2.0f * M_PI * i * 3.0f / FFT_LEN) +
 0.5f * cosf(2.0f * M_PI * i * 7.0f / FFT_LEN) +
 0.01f * (rand() % 100 - 50);
 compute_signal_dft(signal, FFT_LEN, magnitude);
 }
 /* Report dominant frequency bin */
 float peak = 0.0f;
 int peak_bin = 0;
 for (int k = 1; k < FFT_LEN; k++) {
 if (magnitude[k] > peak) { peak = magnitude[k]; peak_bin = k; }
 }
 double elapsed = get_time_ms() - start;
 FLIGHT_LOG("DFT analysis: %.2f ms (%d segments, dominant bin %d, mag %.2f)\n",
 elapsed, TIMESTEPS, peak_bin, peak);
}

// Utility function prototypes
void initialize_random_matrix(float matrix[MATRIX_SIZE][MATRIX_SIZE]);
void initialize_random_signal(float *signal, int length);

// Custom accelerator function prototypes (implemented via inline assembly
// or through a custom library interfacing with the hardware)
void matrix_multiply_accelerated(float A[MATRIX_SIZE][MATRIX_SIZE],
 float B[MATRIX_SIZE][MATRIX_SIZE],
 float C[MATRIX_SIZE][MATRIX_SIZE]);
void matrix_multiply_tiled(float A[MATRIX_SIZE][MATRIX_SIZE],
 float B[MATRIX_SIZE][MATRIX_SIZE],
 float C[MATRIX_SIZE][MATRIX_SIZE]);
float sin_accelerated(float angle);
float cos_accelerated(float angle);
float atan2_accelerated(float y, float x);
float sqrt_accelerated(float x);

// Large matrix multiplication for link-budget computations - targets 3x3 tile matrix accelerator
void execute_matrix_pipeline(int use_accelerator, int size) {
 FLIGHT_LOG("\n[COM] matmul %dx%d%s\n",
 size, size, use_accelerator ? " (Accelerated)" : " (Software)");

 /* FDIR guard: reject sizes exceeding pre-allocated static pool */
 if (size > MAX_MAT_DIM) {
 fdir_report(FDIR_WARN, "MATMUL",
 "requested size exceeds MAX_MAT_DIM -- clamped");
 size = MAX_MAT_DIM;
 }

 /* Static BSS pools — no heap allocation (MISRA 21.3) */
 float (*A)[MAX_MAT_DIM] = mat_pool_A;
 float (*B)[MAX_MAT_DIM] = mat_pool_B;
 float (*C)[MAX_MAT_DIM] = mat_pool_C;

 // Initialize matrices with random values
 for (int i = 0; i < size; i++) {
 for (int j = 0; j < size; j++) {
 A[i][j] = (float)rand() / RAND_MAX * 2.0f - 1.0f;
 B[i][j] = (float)rand() / RAND_MAX * 2.0f - 1.0f;
 C[i][j] = 0.0f;
 }
 }

 double start_time = get_time_ms();

 // Main processing loop
 for (int iter = 0; iter < TIMESTEPS; iter++) {
 if (use_accelerator) {
 // Use tiled matrix multiplication with simulated accelerator
 if (size == MATRIX_SIZE) {
 // If the size matches our predefined MATRIX_SIZE, use the static implementation
 float static_A[MATRIX_SIZE][MATRIX_SIZE];
 float static_B[MATRIX_SIZE][MATRIX_SIZE];
 float static_C[MATRIX_SIZE][MATRIX_SIZE];

 // Copy to static arrays (necessary for function signature)
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 static_A[i][j] = A[i][j];
 static_B[i][j] = B[i][j];
 }
 }

 matrix_multiply_tiled(static_A, static_B, static_C);

 // Copy back to dynamic array
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 C[i][j] = static_C[i][j];
 }
 }
 } else {
 // For different sizes, implement tiled multiplication directly
 // Initialize result matrix to zeros
 for (int i = 0; i < size; i++) {
 for (int j = 0; j < size; j++) {
 C[i][j] = 0.0f;
 }
 }

 // Process the matrix in TILE_SIZE x TILE_SIZE tiles
 for (int i0 = 0; i0 < size; i0 += TILE_SIZE) {
 for (int j0 = 0; j0 < size; j0 += TILE_SIZE) {
 for (int k0 = 0; k0 < size; k0 += TILE_SIZE) {
 // Extract tiles (submatrices) from A and B
 float A_tile[TILE_SIZE][TILE_SIZE];
 float B_tile[TILE_SIZE][TILE_SIZE];
 float C_tile[TILE_SIZE][TILE_SIZE];

 // Initialize C tile to zeros
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 C_tile[i][j] = 0.0f;
 }
 }

 // Extract A and B tiles
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 if (i0 + i < size && k0 + j < size) {
 A_tile[i][j] = A[i0 + i][k0 + j];
 } else {
 A_tile[i][j] = 0.0f;
 }

 if (k0 + i < size && j0 + j < size) {
 B_tile[i][j] = B[k0 + i][j0 + j];
 } else {
 B_tile[i][j] = 0.0f;
 }
 }
 }

 // Multiply tiles using the accelerator
 // 3x3 systolic array computation
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 for (int k = 0; k < TILE_SIZE; k++) {
 C_tile[i][j] += A_tile[i][k] * B_tile[k][j];
 }
 }
 }

 // Add result back to the appropriate location in C
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 if (i0 + i < size && j0 + j < size) {
 C[i0 + i][j0 + j] += C_tile[i][j];
 }
 }
 }
 }
 }
 }
 }
 } else {
 // Standard matrix multiplication (software implementation)
 for (int i = 0; i < size; i++) {
 for (int j = 0; j < size; j++) {
 C[i][j] = 0.0f;
 for (int k = 0; k < size; k++) {
 C[i][j] += A[i][k] * B[k][j];
 }
 }
 }
 }
 }

 double end_time = get_time_ms();
 double elapsed_time = end_time - start_time;

 FLIGHT_LOG("Time: %.2f ms for %d iterations\n", elapsed_time, TIMESTEPS);
 FLIGHT_LOG("Average time per iteration: %.4f ms\n", elapsed_time / TIMESTEPS);

 // Save SW result in file-scope buffer for later HW-vs-SW verification
 if (!use_accelerator) {
 for (int i = 0; i < size; i++)
 for (int j = 0; j < size; j++)
 sw_result_buf[i * size + j] = C[i][j];
 sw_result_size = size;
 } else if (sw_result_size == size) {
 // Compare HW result against saved SW result
 float max_diff = 0.0f;
 for (int i = 0; i < size; i++) {
 for (int j = 0; j < size; j++) {
 float diff = fabsf(C[i][j] - sw_result_buf[i * size + j]);
 if (diff > max_diff) {
 max_diff = diff;
 }
 }
 }
 FLIGHT_LOG("Maximum HW-SW difference: %.6f", max_diff);
 if (max_diff > ERROR_THRESHOLD)
 FLIGHT_LOG(" [FAIL > %.1e]", (double)ERROR_THRESHOLD);
 else
 FLIGHT_LOG(" [PASS]");
 FLIGHT_LOG("\n");
 }

 // Calculate FLOPS
 // Each matrix multiplication performs 2*size^3 floating-point operations
 // (size^3 multiplications and size^3 additions)
 double operations = 2.0 * size * size * size * TIMESTEPS;
 double flops = operations / (elapsed_time / 1000.0); // Convert ms to seconds
 FLIGHT_LOG("Performance: %.2f GFLOPS\n", flops / 1e9);

 /* Static pools: no deallocation needed (MISRA 21.3) */
}

// Utility functions
void initialize_random_matrix(float matrix[MATRIX_SIZE][MATRIX_SIZE]) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 matrix[i][j] = (float)rand() / RAND_MAX * 2.0f - 1.0f;
 }
 }
}

void initialize_random_signal(float *signal, int length) {
 for (int i = 0; i < length; i++) {
 signal[i] = (float)rand() / RAND_MAX * 2.0f - 1.0f;
 }
}


double get_time_ms(void) {
 struct timeval tv;
 gettimeofday(&tv, NULL);
 return (double)tv.tv_sec * 1000.0 + (double)tv.tv_usec / 1000.0;
}

// Main communication processing components
void process_qam_modulation(int use_accelerator);
void estimate_channel_state(int use_accelerator);
void apply_error_correction(int use_accelerator);
void compress_telemetry_data(int use_accelerator);
void equalize_signal_adaptive(int use_accelerator);

// For now, we'll use standard math functions as software fallback for the accelerated versions
// Hardware-accelerated variant uses custom matrix instructions
float sin_accelerated(float angle) {
 // Trigonometric unit computation
 return sinf(angle);
}

float cos_accelerated(float angle) {
 // Trigonometric unit computation
 return cosf(angle);
}

float atan2_accelerated(float y, float x) {
 // Trigonometric unit computation
 return atan2f(y, x);
}

float sqrt_accelerated(float x) {
 // Hardware square-root unit computation
 return sqrtf(x);
}

void matrix_multiply_tiled(float A[MATRIX_SIZE][MATRIX_SIZE],
 float B[MATRIX_SIZE][MATRIX_SIZE],
 float C[MATRIX_SIZE][MATRIX_SIZE]) {
 // This implementation uses the 3x3 tile systolic array accelerator
 // by breaking the large matrix multiplication into 3x3 tile operations

 // Initialize result matrix to zeros
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 C[i][j] = 0.0f;
 }
 }

 // Process the matrix in TILE_SIZE x TILE_SIZE tiles
 for (int i0 = 0; i0 < MATRIX_SIZE; i0 += TILE_SIZE) {
 for (int j0 = 0; j0 < MATRIX_SIZE; j0 += TILE_SIZE) {
 for (int k0 = 0; k0 < MATRIX_SIZE; k0 += TILE_SIZE) {
 // Extract tiles (submatrices) from A and B
 float A_tile[TILE_SIZE][TILE_SIZE];
 float B_tile[TILE_SIZE][TILE_SIZE];
 float C_tile[TILE_SIZE][TILE_SIZE];

 // Initialize C tile to zeros
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 C_tile[i][j] = 0.0f;
 }
 }

 // Extract A and B tiles
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 if (i0 + i < MATRIX_SIZE && k0 + j < MATRIX_SIZE) {
 A_tile[i][j] = A[i0 + i][k0 + j];
 } else {
 A_tile[i][j] = 0.0f;
 }

 if (k0 + i < MATRIX_SIZE && j0 + j < MATRIX_SIZE) {
 B_tile[i][j] = B[k0 + i][j0 + j];
 } else {
 B_tile[i][j] = 0.0f;
 }
 }
 }

 // Multiply tiles using the accelerator
 // 3x3 systolic array computation
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 for (int k = 0; k < TILE_SIZE; k++) {
 C_tile[i][j] += A_tile[i][k] * B_tile[k][j];
 }
 }
 }

 // Add result back to the appropriate location in C
 for (int i = 0; i < TILE_SIZE; i++) {
 for (int j = 0; j < TILE_SIZE; j++) {
 if (i0 + i < MATRIX_SIZE && j0 + j < MATRIX_SIZE) {
 C[i0 + i][j0 + j] += C_tile[i][j];
 }
 }
 }
 }
 }
 }
}

void matrix_multiply_accelerated(float A[MATRIX_SIZE][MATRIX_SIZE],
 float B[MATRIX_SIZE][MATRIX_SIZE],
 float C[MATRIX_SIZE][MATRIX_SIZE]) {
 // Accelerated variant uses custom RISC-V matrix instructions
 // to interface with the hardware accelerator. Here we're using the
 // tiled implementation to simulate the behavior.
 matrix_multiply_tiled(A, B, C);
}

void matrix_multiply_software(float A[MATRIX_SIZE][MATRIX_SIZE],
 float B[MATRIX_SIZE][MATRIX_SIZE],
 float C[MATRIX_SIZE][MATRIX_SIZE]) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 C[i][j] = 0.0f;
 for (int k = 0; k < MATRIX_SIZE; k++) {
 C[i][j] += A[i][k] * B[k][j];
 }
 }
 }
}

/**
 * QAM Modulation/Demodulation processing - trigonometric operations
 * Simulates QAM modulation and demodulation as used in satellite communications
 */
void process_qam_modulation(int use_accelerator) {
 FLIGHT_LOG("\n[COM] QAM%s\n",
 use_accelerator ? " (Accelerated)" : " (Software)");

 /* Static BSS pools — no heap allocation (MISRA 21.3) */
 float *data_bits = qam_data_bits;
 float *i_signal = qam_i_signal;
 float *q_signal = qam_q_signal;
 float *demod_bits = qam_demod_bits;

 // Create random data bits
 initialize_random_signal(data_bits, SIGNAL_LENGTH);

 double start_time = get_time_ms();

 // Main processing loop
 for (int iter = 0; iter < TIMESTEPS; iter++) {
 // QAM Modulation - Maps data bits to constellation points
 for (int i = 0; i < SIGNAL_LENGTH; i++) {
 // Calculate constellation point from bits
 int constellation_point = (int)(data_bits[i] * MODULATION_ORDER) % MODULATION_ORDER;

 // Convert to I/Q coordinates using trigonometric functions
 float angle = 2.0f * M_PI * constellation_point / MODULATION_ORDER;
 float amplitude = 1.0f + 0.5f * (constellation_point % 4);

 if (use_accelerator) {
 i_signal[i] = amplitude * cos_accelerated(angle);
 q_signal[i] = amplitude * sin_accelerated(angle);
 } else {
 i_signal[i] = amplitude * cosf(angle);
 q_signal[i] = amplitude * sinf(angle);
 }
 }

 // Simulate channel effects
 for (int i = 0; i < SIGNAL_LENGTH; i++) {
 float phase_rotation = use_accelerator
 ? 0.1f * sin_accelerated(0.001f * i)
 : 0.1f * sinf(0.001f * i);
 float temp_i, temp_q;

 if (use_accelerator) {
 temp_i = i_signal[i] * cos_accelerated(phase_rotation) -
 q_signal[i] * sin_accelerated(phase_rotation);
 temp_q = i_signal[i] * sin_accelerated(phase_rotation) +
 q_signal[i] * cos_accelerated(phase_rotation);
 } else {
 temp_i = i_signal[i] * cosf(phase_rotation) -
 q_signal[i] * sinf(phase_rotation);
 temp_q = i_signal[i] * sinf(phase_rotation) +
 q_signal[i] * cosf(phase_rotation);
 }

 i_signal[i] = temp_i;
 q_signal[i] = temp_q;
 }

 // QAM Demodulation - Convert back to bits
 for (int i = 0; i < SIGNAL_LENGTH; i++) {
 float magnitude, phase;

 if (use_accelerator) {
 magnitude = sqrt_accelerated(i_signal[i] * i_signal[i] + q_signal[i] * q_signal[i]);
 phase = atan2_accelerated(q_signal[i], i_signal[i]);
 } else {
 magnitude = sqrtf(i_signal[i] * i_signal[i] + q_signal[i] * q_signal[i]);
 phase = atan2f(q_signal[i], i_signal[i]);
 }

 // Normalize phase to 0-2π range
 if (phase < 0) phase += 2.0f * M_PI;

 // Convert back to constellation point
 int constellation_point = (int)(phase * MODULATION_ORDER / (2.0f * M_PI));

 // Convert to original data bits (magnitude used for EVM weighting)
 float ideal_mag = 1.0f; /* unit-circle constellation */
 float evm_sample = (magnitude - ideal_mag) * (magnitude - ideal_mag);
 demod_bits[i] = (float)constellation_point / MODULATION_ORDER
 + 0.001f * evm_sample; /* EVM-based demod bias */
 }
 }

 double end_time = get_time_ms();
 double elapsed_time = end_time - start_time;

 FLIGHT_LOG("Time: %.2f ms for %d iterations\n", elapsed_time, TIMESTEPS);
 FLIGHT_LOG("Average time per iteration: %.4f ms\n", elapsed_time / TIMESTEPS);

 // Calculate bit error rate
 int errors = 0;
 for (int i = 0; i < SIGNAL_LENGTH; i++) {
 if (fabsf(data_bits[i] - demod_bits[i]) > 0.1f) {
 errors++;
 }
 }
 FLIGHT_LOG("Error rate: %.2f%%\n", 100.0f * errors / SIGNAL_LENGTH);

 /* Static pools: no deallocation needed (MISRA 21.3) */
}

/**
 * Channel state estimation - matrix operations for MIMO processing
 * Simulates MIMO channel estimation common in modern satellite communications
 */
void estimate_channel_state(int use_accelerator) {
 FLIGHT_LOG("\n[COM] chan_est%s\n",
 use_accelerator ? " (Accelerated)" : " (Software)");

 // Create channel matrices
 float H[MATRIX_SIZE][MATRIX_SIZE]; // Channel matrix
 float H_transpose[MATRIX_SIZE][MATRIX_SIZE]; // Transpose
 float H_inverse[MATRIX_SIZE][MATRIX_SIZE]; // Inverse (approximated)
 float temp1[MATRIX_SIZE][MATRIX_SIZE]; // Temporary matrix
 float temp2[MATRIX_SIZE][MATRIX_SIZE]; // Temporary matrix
 float identity[MATRIX_SIZE][MATRIX_SIZE]; // Identity matrix

 // Initialize identity matrix
 memset(identity, 0, sizeof(identity));
 for (int i = 0; i < MATRIX_SIZE; i++) {
 identity[i][i] = 1.0f;
 }

 double start_time = get_time_ms();

 // Main processing loop
 for (int iter = 0; iter < TIMESTEPS; iter++) {
 // Generate a random channel matrix for each iteration
 initialize_random_matrix(H);

 // Compute transpose
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 H_transpose[j][i] = H[i][j];
 }
 }

 // Estimate channel inverse using iterative method
 // Initialize with transpose (simplification of proper matrix inversion)
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 H_inverse[i][j] = H_transpose[i][j];
 }
 }

 // Refine inverse estimation using Newton-Raphson iteration
 // H_inverse = H_inverse * (2I - H * H_inverse)
 // Convergence monitored; fallback on divergence (FDIR)
 float prev_residual = 1e30f;
 int nr_converged = 0;
 for (int refine = 0; refine < NR_MAX_ITER; refine++) {
    // Compute H * H_inverse
    // Policy: always use SW matmul for the inversion refinement loop.
    matrix_multiply_software(H, H_inverse, temp1);

 // Convergence check: ||I - H*H_inv||_F
 float residual = 0.0f;
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++) {
 float e = temp1[i][j] - identity[i][j];
 residual += e * e;
 }
 residual = sqrtf(residual);

 if (residual < NR_CONV_TOL) {
 nr_converged = 1;
 break; /* converged */
 }
 if (residual > NR_DIVERGE_RATIO * prev_residual) {
 /* Divergence detected: keep previous estimate */
 fdir_report(FDIR_ERR_CONVERGE, "CHAN_EST",
 "NR divergence -- using last-good inverse");
 break;
 }
 prev_residual = residual;

 // Compute 2I - H * H_inverse
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 temp2[i][j] = 2.0f * identity[i][j] - temp1[i][j];
 }
 }

    // Compute H_inverse * (2I - H * H_inverse)
    // Policy: always use SW matmul for the inversion refinement loop.
    matrix_multiply_software(H_inverse, temp2, temp1);

 // Update H_inverse
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 H_inverse[i][j] = temp1[i][j];
 }
 }
 }
 if (!nr_converged && prev_residual >= NR_CONV_TOL)
 fdir_report(FDIR_WARN, "CHAN_EST",
 "NR did not converge within max iterations");

    // Verify inverse quality by computing H * H_inverse (should approach identity)
    // Policy: always use SW matmul for the inversion refinement loop.
    matrix_multiply_software(H, H_inverse, temp1);
 }

 double end_time = get_time_ms();
 double elapsed_time = end_time - start_time;

 FLIGHT_LOG("Time: %.2f ms for %d iterations\n", elapsed_time, TIMESTEPS);
 FLIGHT_LOG("Average time per iteration: %.4f ms\n", elapsed_time / TIMESTEPS);

 // Verify result quality (how close temp1 is to identity)
 float error = 0.0f;
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 error += fabsf(temp1[i][j] - identity[i][j]);
 }
 }
 FLIGHT_LOG("Total error in matrix inversion: %.6f\n", error);
}

/**
 * Telemetry data compression - trigonometric and matrix operations
 * DCT-based image compression for onboard data handling
 */
void compress_telemetry_data(int use_accelerator) {
 FLIGHT_LOG("\n[COM] compress%s\n",
 use_accelerator ? " (Accelerated)" : " (Software)");

 /* Static BSS pools — no heap allocation (MISRA 21.3) */
 float *input_blocks = compress_input;
 float *dct_coeffs = compress_dct;
 float *quantized = compress_quant;
 float *reconstructed = compress_recon;

 // Create DCT matrix
 float dct_matrix[MATRIX_SIZE][MATRIX_SIZE];
 float dct_matrix_t[MATRIX_SIZE][MATRIX_SIZE];
 float temp_block[MATRIX_SIZE][MATRIX_SIZE];
 float temp_block2[MATRIX_SIZE][MATRIX_SIZE];

 // Initialize input data blocks with random values
 for (int block = 0; block < COMPRESSION_BLOCKS; block++) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 input_blocks[block * MATRIX_SIZE * MATRIX_SIZE + i * MATRIX_SIZE + j] =
 (float)rand() / RAND_MAX * 255.0f;
 }
 }
 }

 // Initialize DCT coefficient matrix
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 float scale = (i == 0) ? 1.0f/sqrtf(MATRIX_SIZE) : sqrtf(2.0f/MATRIX_SIZE);
 float angle = M_PI * i * (2.0f * j + 1.0f) / (2.0f * MATRIX_SIZE);

 if (use_accelerator) {
 dct_matrix[i][j] = scale * cos_accelerated(angle);
 } else {
 dct_matrix[i][j] = scale * cosf(angle);
 }
 dct_matrix_t[j][i] = dct_matrix[i][j]; // Transpose
 }
 }

 // Quantization matrix (JPEG-derived)
 float quant_matrix[MATRIX_SIZE][MATRIX_SIZE];
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 quant_matrix[i][j] = 1.0f + 4.0f * (i + j);
 }
 }

 double start_time = get_time_ms();

 // Main processing loop
 for (int iter = 0; iter < TIMESTEPS; iter++) {
 // Process each block
 for (int block = 0; block < COMPRESSION_BLOCKS; block++) {
 // Copy block to temporary matrix
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 temp_block[i][j] = input_blocks[block * MATRIX_SIZE * MATRIX_SIZE + i * MATRIX_SIZE + j];
 }
 }

 // Forward DCT transform: DCT * Block * DCT^T
 if (use_accelerator) {
 matrix_multiply_accelerated(dct_matrix, temp_block, temp_block2);
 matrix_multiply_accelerated(temp_block2, dct_matrix_t, temp_block);
 } else {
 matrix_multiply_software(dct_matrix, temp_block, temp_block2);
 matrix_multiply_software(temp_block2, dct_matrix_t, temp_block);
 }

 // Quantize coefficients
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 float coeff = temp_block[i][j];
 // Store DCT coefficients
 dct_coeffs[block * MATRIX_SIZE * MATRIX_SIZE + i * MATRIX_SIZE + j] = coeff;

 // Quantize
 float q_coeff = roundf(coeff / quant_matrix[i][j]);
 quantized[block * MATRIX_SIZE * MATRIX_SIZE + i * MATRIX_SIZE + j] = q_coeff;

 // Dequantize (for reconstruction)
 temp_block[i][j] = q_coeff * quant_matrix[i][j];
 }
 }

 // Inverse DCT: DCT^T * Block * DCT
 if (use_accelerator) {
 matrix_multiply_accelerated(dct_matrix_t, temp_block, temp_block2);
 matrix_multiply_accelerated(temp_block2, dct_matrix, temp_block);
 } else {
 matrix_multiply_software(dct_matrix_t, temp_block, temp_block2);
 matrix_multiply_software(temp_block2, dct_matrix, temp_block);
 }

 // Store reconstructed block
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 // Clamp values to 0-255 range (for image-like data)
 float val = temp_block[i][j];
 if (val < 0.0f) val = 0.0f;
 if (val > 255.0f) val = 255.0f;

 reconstructed[block * MATRIX_SIZE * MATRIX_SIZE + i * MATRIX_SIZE + j] = val;
 }
 }
 }
 }

 double end_time = get_time_ms();
 double elapsed_time = end_time - start_time;

 FLIGHT_LOG("Time: %.2f ms for %d iterations (%d blocks each)\n",
 elapsed_time, TIMESTEPS, COMPRESSION_BLOCKS);
 FLIGHT_LOG("Average time per block: %.4f ms\n",
 elapsed_time / (TIMESTEPS * COMPRESSION_BLOCKS));

 // Calculate compression quality (PSNR)
 float total_error = 0.0f;
 for (int i = 0; i < COMPRESSION_BLOCKS * MATRIX_SIZE * MATRIX_SIZE; i++) {
 float diff = input_blocks[i] - reconstructed[i];
 total_error += diff * diff;
 }
 float mse = total_error / (COMPRESSION_BLOCKS * MATRIX_SIZE * MATRIX_SIZE);
 float psnr = 10.0f * log10f(255.0f * 255.0f / mse);
 FLIGHT_LOG("Compression quality (PSNR): %.2f dB\n", psnr);

 // Calculate compression ratio
 int original_size = COMPRESSION_BLOCKS * MATRIX_SIZE * MATRIX_SIZE;
 int nonzero_coeffs = 0;
 for (int i = 0; i < original_size; i++) {
 if (fabsf(quantized[i]) > 0.001f) {
 nonzero_coeffs++;
 }
 }
 FLIGHT_LOG("Compression ratio: %.2f:1\n", (float)original_size / nonzero_coeffs);

 // DC energy fraction — measures how concentrated energy is in low frequencies
 float dc_energy = 0.0f, total_energy = 0.0f;
 for (int block = 0; block < COMPRESSION_BLOCKS; block++) {
 float dc = dct_coeffs[block * MATRIX_SIZE * MATRIX_SIZE]; /* (0,0) coefficient */
 dc_energy += dc * dc;
 for (int k = 0; k < MATRIX_SIZE * MATRIX_SIZE; k++) {
 float c = dct_coeffs[block * MATRIX_SIZE * MATRIX_SIZE + k];
 total_energy += c * c;
 }
 }
 FLIGHT_LOG("DC energy fraction: %.4f\n", dc_energy / (total_energy + 1e-12f));

 /* Static pools: no deallocation needed (MISRA 21.3) */
}

/**
 * Adaptive signal equalization - matrix operations and statistical adaptation
 * Simulates adaptive filtering for noise/interference cancellation
 */
void equalize_signal_adaptive(int use_accelerator) {
 FLIGHT_LOG("\n[COM] equalizer%s\n",
 use_accelerator ? " (Accelerated)" : " (Software)");

 // Signal parameters
 const int FILTER_SIZE = 16; // Adaptive equalizer tap count (power of 2)
 const int TRAINING_LEN = SIGNAL_LENGTH / 4;

 /* Static BSS pools — no heap allocation (MISRA 21.3) */
 float *input_signal = eq_input;
 float *desired_signal = eq_desired;
 float *output_signal = eq_output;
 float *error_signal = eq_error;
 float filter_coeffs[FILTER_SIZE];

 // Initialize signals
 initialize_random_signal(input_signal, SIGNAL_LENGTH);

 // Create desired signal (input with added channel effects)
 for (int i = 0; i < SIGNAL_LENGTH; i++) {
 float t = (float)i / SIGNAL_LENGTH;
 float phase_shift = 0.0f;

 if (use_accelerator) {
 phase_shift = 0.2f * sin_accelerated(2.0f * M_PI * 5.0f * t);
 desired_signal[i] = input_signal[i] * cos_accelerated(phase_shift);
 } else {
 phase_shift = 0.2f * sinf(2.0f * M_PI * 5.0f * t);
 desired_signal[i] = input_signal[i] * cosf(phase_shift);
 }

 // Add noise
 desired_signal[i] += 0.05f * ((float)rand() / RAND_MAX * 2.0f - 1.0f);
 }

 // Initialize filter coefficients
 for (int i = 0; i < FILTER_SIZE; i++) {
 filter_coeffs[i] = 0.0f;
 }
 filter_coeffs[0] = 1.0f; // Initial guess

 double start_time = get_time_ms();

 // Main processing loop
 for (int iter = 0; iter < TIMESTEPS; iter++) {
 float avg_error = 0.0f;

 // Reset filter for each iteration
 for (int i = 0; i < FILTER_SIZE; i++) {
 filter_coeffs[i] = (i == 0) ? 1.0f : 0.0f;
 }

 // Adaptive LMS algorithm for filter training
 for (int n = FILTER_SIZE; n < TRAINING_LEN; n++) {
 // Apply filter to input
 float y = 0.0f;
 for (int i = 0; i < FILTER_SIZE; i++) {
 y += filter_coeffs[i] * input_signal[n - i];
 }
 output_signal[n] = y;

 // Calculate error
 error_signal[n] = desired_signal[n] - y;
 avg_error += error_signal[n] * error_signal[n];

 // Update filter coefficients (LMS algorithm)
 float mu = 0.01f; // Step size
 for (int i = 0; i < FILTER_SIZE; i++) {
 filter_coeffs[i] += mu * error_signal[n] * input_signal[n - i];
 }
 }

 // Apply trained filter to remaining signal
 for (int n = TRAINING_LEN; n < SIGNAL_LENGTH; n++) {
 float y = 0.0f;
 for (int i = 0; i < FILTER_SIZE; i++) {
 y += filter_coeffs[i] * input_signal[n - i];
 }
 output_signal[n] = y;
 error_signal[n] = desired_signal[n] - y;
 avg_error += error_signal[n] * error_signal[n];
 }

 avg_error /= SIGNAL_LENGTH;
 }

 double end_time = get_time_ms();
 double elapsed_time = end_time - start_time;

 FLIGHT_LOG("Time: %.2f ms for %d iterations\n", elapsed_time, TIMESTEPS);
 FLIGHT_LOG("Average time per iteration: %.4f ms\n", elapsed_time / TIMESTEPS);

 // Calculate error improvement
 float input_power = 0.0f, error_power = 0.0f;
 for (int i = TRAINING_LEN; i < SIGNAL_LENGTH; i++) {
 float orig_error = desired_signal[i] - input_signal[i];
 input_power += orig_error * orig_error;
 error_power += error_signal[i] * error_signal[i];
 }
 FLIGHT_LOG("Error reduction: %.2f dB\n",
 10.0f * log10f(input_power / error_power));

 // Print final filter coefficients
 FLIGHT_LOG("Final filter coefficients: ");
 for (int i = 0; i < FILTER_SIZE; i++) {
 FLIGHT_LOG("%.4f ", filter_coeffs[i]);
 }
 FLIGHT_LOG("\n");

 /* Static pools: no deallocation needed (MISRA 21.3) */
}

/**
 * Error correction coding (Reed-Solomon/LDPC) - matrix operations
 * LDPC encoder/decoder for satellite communication links
 * Uses a hard-decision bit-flipping (majority-logic) decoder,
 * a simplified majority-logic decoder representative of radiation-hardened implementations.
 */
void apply_error_correction(int use_accelerator) {
 (void)use_accelerator; /* LDPC is scalar bit-ops; no HW accel path */
 FLIGHT_LOG("\n[COM] ECC (SW)\n");

 // Parity check matrix (reduced dimension for onboard processing)
 float H[MATRIX_SIZE][MATRIX_SIZE * 2];
 float H_transpose[MATRIX_SIZE * 2][MATRIX_SIZE];
 float syndrome[MATRIX_SIZE];
 float message[MATRIX_SIZE * 2];
 float corrupted[MATRIX_SIZE * 2];
 float decoded[MATRIX_SIZE * 2];

 // Initialize parity check matrix
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE * 2; j++) {
 // Create a sparse matrix pattern
 if (j == i || j == ((i + 1) % MATRIX_SIZE) || j == (MATRIX_SIZE + i)) {
 H[i][j] = 1.0f;
 } else {
 H[i][j] = 0.0f;
 }
 }
 }

 // Compute transpose
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE * 2; j++) {
 H_transpose[j][i] = H[i][j];
 }
 }

 double start_time = get_time_ms();

 // Main processing loop
 for (int iter = 0; iter < TIMESTEPS; iter++) {
 // Generate random message bits (0 or 1)
 for (int i = 0; i < MATRIX_SIZE; i++) {
 message[i] = (rand() % 2) ? 1.0f : 0.0f;
 }

 // LDPC encoding
 for (int i = 0; i < MATRIX_SIZE; i++) {
 float parity = 0.0f;
 for (int j = 0; j < MATRIX_SIZE; j++) {
 parity += H[i][j] * message[j];
 }
 message[MATRIX_SIZE + i] = fmodf(parity, 2.0f);
 }

 // Copy message to corrupted and inject errors
 for (int i = 0; i < MATRIX_SIZE * 2; i++) {
 corrupted[i] = message[i];
 }

 // Randomly flip some bits (error injection)
 int num_errors = MATRIX_SIZE / 2;
 for (int e = 0; e < num_errors; e++) {
 int pos = rand() % (MATRIX_SIZE * 2);
 corrupted[pos] = 1.0f - corrupted[pos]; // Flip bit
 }

 // Initialize decoded message with corrupted bits
 for (int i = 0; i < MATRIX_SIZE * 2; i++) {
 decoded[i] = corrupted[i];
 }

 // Iterative hard-decision bit-flipping decoding
 for (int iter_decode = 0; iter_decode < 10; iter_decode++) {
 // Compute syndrome
 for (int i = 0; i < MATRIX_SIZE; i++) {
 float sum = 0.0f;
 for (int j = 0; j < MATRIX_SIZE * 2; j++) {
 sum += H[i][j] * decoded[j];
 }
 syndrome[i] = fmodf(sum, 2.0f);
 }

 // Check if syndrome is zero (decoding successful)
 float syndrome_sum = 0.0f;
 for (int i = 0; i < MATRIX_SIZE; i++) {
 syndrome_sum += syndrome[i];
 }

 if (syndrome_sum < 0.01f) {
 break; // Decoding successful
 }

 // Update beliefs
 for (int j = 0; j < MATRIX_SIZE * 2; j++) {
 float sum = 0.0f;
 for (int i = 0; i < MATRIX_SIZE; i++) {
 if (H_transpose[j][i] > 0.5f && syndrome[i] > 0.5f) {
 sum += 1.0f;
 }
 }

 // Flip bit if most checks indicate an error
 if (sum > 1.0f) {
 decoded[j] = 1.0f - decoded[j];
 }
 }
 }
 }

 double end_time = get_time_ms();
 double elapsed_time = end_time - start_time;

 FLIGHT_LOG("Time: %.2f ms for %d iterations\n", elapsed_time, TIMESTEPS);
 FLIGHT_LOG("Average time per iteration: %.4f ms\n", elapsed_time / TIMESTEPS);

 // Calculate bit error rate
 int errors = 0;
 for (int i = 0; i < MATRIX_SIZE * 2; i++) {
 if (fabsf(message[i] - decoded[i]) > 0.01f) {
 errors++;
 }
 }
 FLIGHT_LOG("Final bit error rate: %.2f%%\n", 100.0f * errors / (MATRIX_SIZE * 2));
}

/* ------------------------------------------------------------------ */
/* BCE: Orbital Mechanics (12%) — Link-budget orbital geometry */
/* ------------------------------------------------------------------ */

/* Physical constants for orbit mechanics */
#define MU_EARTH 3.986004418e14 /* Earth GM (m^3/s^2) */
#define R_EARTH 6378137.0 /* Earth equatorial radius (m) */
#define J2_COEFF 1.08263e-3 /* Earth J2 oblateness */
#define C_LIGHT 2.99792458e8 /* Speed of light (m/s) */
#define FREQ_KU 14.0e9 /* Ku-band uplink frequency (Hz) */
#define SAT_ALT 35786.0e3 /* GEO altitude (m) */
#define GROUND_LAT 12.9716 /* mission control center (deg) */

/**
 * Circular orbit propagation with J2 secular drift.
 * Returns satellite ECI position (x, y, z) in metres.
 * mean_motion rad/s, RAAN drift from J2, time in seconds.
 */
static void orbit_propagate_circular(double a, double inc_rad,
 double raan0, double arg_lat0,
 double t_sec,
 double pos_out[3], double vel_out[3])
{
 double n = sqrt(MU_EARTH / (a * a * a)); /* mean motion */
 /* J2 secular RAAN drift: dΩ/dt = -1.5 n J2 (Re/a)^2 cos(i) */
 double re_a2 = (R_EARTH / a) * (R_EARTH / a);
 double raan_dot = -1.5 * n * J2_COEFF * re_a2 * cos(inc_rad);
 double raan = raan0 + raan_dot * t_sec;
 double u = arg_lat0 + n * t_sec; /* argument of lat */

 /* Position in orbital plane */
 double x_op = a * cos(u);
 double y_op = a * sin(u);

 /* Rotate to ECI via RAAN and inclination */
 double cO = cos(raan), sO = sin(raan);
 double ci = cos(inc_rad), si = sin(inc_rad);

 pos_out[0] = cO * x_op - sO * ci * y_op;
 pos_out[1] = sO * x_op + cO * ci * y_op;
 pos_out[2] = si * y_op;

 /* Velocity (tangential, circular orbit) */
 double dx_op = -a * sin(u) * n;
 double dy_op = a * cos(u) * n;
 vel_out[0] = cO * dx_op - sO * ci * dy_op;
 vel_out[1] = sO * dx_op + cO * ci * dy_op;
 vel_out[2] = si * dy_op;
}

/**
 * Compute slant range from satellite to ground station.
 * gs_lat in radians, gs_lon in radians.
 */
static double compute_slant_range(const double sat_pos[3],
 double gs_lat, double gs_lon)
{
 /* Ground station ECEF position (spherical Earth approx) */
 double gs[3];
 gs[0] = R_EARTH * cos(gs_lat) * cos(gs_lon);
 gs[1] = R_EARTH * cos(gs_lat) * sin(gs_lon);
 gs[2] = R_EARTH * sin(gs_lat);

 double dx = sat_pos[0] - gs[0];
 double dy = sat_pos[1] - gs[1];
 double dz = sat_pos[2] - gs[2];
 return sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Doppler shift from relative velocity along line-of-sight.
 */
static double compute_doppler_shift(const double sat_pos[3],
 const double sat_vel[3],
 double gs_lat, double gs_lon,
 double freq_Hz)
{
 double gs[3];
 gs[0] = R_EARTH * cos(gs_lat) * cos(gs_lon);
 gs[1] = R_EARTH * cos(gs_lat) * sin(gs_lon);
 gs[2] = R_EARTH * sin(gs_lat);

 double dx = sat_pos[0] - gs[0];
 double dy = sat_pos[1] - gs[1];
 double dz = sat_pos[2] - gs[2];
 double range = sqrt(dx * dx + dy * dy + dz * dz);
 if (range < 1.0) return 0.0;

 /* Radial velocity = (v · r_hat) */
 double v_rad = (sat_vel[0] * dx + sat_vel[1] * dy + sat_vel[2] * dz) / range;
 return -v_rad / C_LIGHT * freq_Hz;
}

/**
 * Free-Space Path Loss: FSPL = 20·log10(d) + 20·log10(f) + 20·log10(4π/c)
 */
static double compute_fspl(double distance_m, double freq_Hz)
{
 return 20.0 * log10(distance_m) + 20.0 * log10(freq_Hz)
 + 20.0 * log10(4.0 * M_PI / C_LIGHT);
}

/**
 * Compute orbital mechanics for SatComm link-budget geometry.
 */
void compute_orbit_link_geometry(int use_accelerator) {
 (void)use_accelerator; /* Orbit propagation uses double-precision libm; no HW trig path */
 FLIGHT_LOG("\n[COM] link_budget\n");
 double start = get_time_ms();

 double a = R_EARTH + SAT_ALT; /* GEO semi-major axis */
 double inc = 0.05 * M_PI / 180.0; /* Near-equatorial */
 double raan0 = 83.0 * M_PI / 180.0; /* GEO slot */
 double arg0 = 0.0;
 double gs_lat = GROUND_LAT * M_PI / 180.0;
 double gs_lon = 77.5946 * M_PI / 180.0; /* mission control center */

 double total_fspl = 0.0, total_doppler = 0.0;

 for (int iter = 0; iter < TIMESTEPS; iter++) {
 /* Propagate orbit over a full period sampled at FFT_LEN points */
 double period = 2.0 * M_PI * sqrt(a * a * a / MU_EARTH);
 double dt = period / FFT_LEN;

 for (int step = 0; step < FFT_LEN; step++) {
 double t = step * dt + iter * period; /* accumulate time */
 double pos[3], vel[3];
 orbit_propagate_circular(a, inc, raan0, arg0, t, pos, vel);

 double range = compute_slant_range(pos, gs_lat, gs_lon);
 double doppler = compute_doppler_shift(pos, vel, gs_lat, gs_lon, FREQ_KU);
 double fspl = compute_fspl(range, FREQ_KU);

 total_fspl += fspl;
 total_doppler += fabs(doppler);
 }
 }

 double elapsed = get_time_ms() - start;
 FLIGHT_LOG("OrbMech link budget: %.2f ms | avg FSPL=%.2f dB | avg Doppler=%.2f Hz\n",
 elapsed, total_fspl / (TIMESTEPS * FFT_LEN),
 total_doppler / (TIMESTEPS * FFT_LEN));
}

/* ------------------------------------------------------------------ */
/* BCE: Wavelet (9%) — Haar wavelet signal denoising */
/* ------------------------------------------------------------------ */

/* WAVELET_LEN defined in configuration section above */

/**
 * In-place 1-D Haar wavelet decomposition (one level).
 * data[0..len-1] → approximation in [0..len/2-1],
 * detail in [len/2..len-1].
 */
static void haar_decompose(float *data, int len)
{
 /* Static BSS scratch buffer (MISRA 21.3: no heap after init) */
 float *tmp = haar_tmp_buf;
 int half = len / 2;
 float s = 1.0f / sqrtf(2.0f);
 for (int i = 0; i < half; i++) {
 tmp[i] = s * (data[2 * i] + data[2 * i + 1]); /* approx */
 tmp[half + i] = s * (data[2 * i] - data[2 * i + 1]); /* detail */
 }
 memcpy(data, tmp, len * sizeof(float));
}

/**
 * In-place 1-D Haar wavelet reconstruction (one level).
 */
static void haar_reconstruct(float *data, int len)
{
 /* Static BSS scratch buffer (MISRA 21.3: no heap after init) */
 float *tmp = haar_tmp_buf;
 int half = len / 2;
 float s = 1.0f / sqrtf(2.0f);
 for (int i = 0; i < half; i++) {
 tmp[2 * i] = s * (data[i] + data[half + i]);
 tmp[2 * i + 1] = s * (data[i] - data[half + i]);
 }
 memcpy(data, tmp, len * sizeof(float));
}

/**
 * Soft-threshold wavelet coefficients for denoising.
 * threshold = sigma * sqrt(2 * log(N)) (universal threshold)
 */
static void wavelet_soft_threshold(float *data, int start, int len,
 float threshold)
{
 for (int i = start; i < start + len; i++) {
 if (data[i] > threshold)
 data[i] -= threshold;
 else if (data[i] < -threshold)
 data[i] += threshold;
 else
 data[i] = 0.0f;
 }
}

/**
 * Full Haar wavelet denoise: multi-level decompose → threshold → reconstruct.
 */
static void haar_wavelet_denoise(float *signal, int len, float noise_sigma)
{
 /* Universal threshold: sigma * sqrt(2 * ln(N)) */
 float threshold = noise_sigma * sqrtf(2.0f * logf((float)len));

 /* Forward DWT — 3 levels */
 int cur_len = len;
 for (int level = 0; level < 3 && cur_len >= 2; level++) {
 haar_decompose(signal, cur_len);
 /* Threshold detail coefficients */
 wavelet_soft_threshold(signal, cur_len / 2, cur_len / 2, threshold);
 cur_len /= 2;
 }

 /* Inverse DWT */
 for (int level = 2; level >= 0; level--) {
 cur_len *= 2;
 haar_reconstruct(signal, cur_len);
 }
}

/**
 * Wavelet denoising for received SatComm signals.
 */
void process_wavelet_denoise(int use_accelerator) {
 (void)use_accelerator; /* Haar wavelet is pure SW; param kept for API uniformity */
 FLIGHT_LOG("\n[COM] wavelet_denoise\n");
 double start = get_time_ms();

 /* Static BSS pools (MISRA 21.3: no heap after init) */
 float *sig = wav_sig;
 float *ref = wav_ref;

 float noise_sigma = 0.15f;
 float total_snr_improvement = 0.0f;
 float *noisy_copy = wav_noisy;

 for (int iter = 0; iter < TIMESTEPS; iter++) {
 /* Generate clean tone + wideband noise (simulates received carrier) */
 for (int i = 0; i < WAVELET_LEN; i++) {
 float clean = sinf(2.0f * M_PI * 5.0f * i / WAVELET_LEN)
 + 0.5f * cosf(2.0f * M_PI * 13.0f * i / WAVELET_LEN);
 float noise = noise_sigma * ((float)rand() / RAND_MAX * 2.0f - 1.0f);
 ref[i] = clean;
 sig[i] = clean + noise;
 }

 /* Save noisy signal before denoising for fair MSE comparison */
 memcpy(noisy_copy, sig, WAVELET_LEN * sizeof(float));

 /* Wavelet denoise */
 haar_wavelet_denoise(sig, WAVELET_LEN, noise_sigma);

 /* Compute MSE improvement: before (noisy vs clean) and after (denoised vs clean) */
 float mse_before = 0.0f, mse_after = 0.0f;
 for (int i = 0; i < WAVELET_LEN; i++) {
 float noisy_err = noisy_copy[i] - ref[i];
 mse_before += noisy_err * noisy_err;
 float denoised_err = sig[i] - ref[i];
 mse_after += denoised_err * denoised_err;
 }
 mse_before /= WAVELET_LEN;
 mse_after /= WAVELET_LEN;
 if (mse_after > 1e-12f)
 total_snr_improvement += 10.0f * log10f(mse_before / mse_after);
 }

 double elapsed = get_time_ms() - start;
 FLIGHT_LOG("Wavelet denoise: %.2f ms | avg SNR gain=%.2f dB\n",
 elapsed, total_snr_improvement / TIMESTEPS);

 /* Static pools: no deallocation needed (MISRA 21.3) */
}

int main() {
 FLIGHT_LOG("[COM] init\n");

 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(8314);
 FLIGHT_LOG("\n[COM] matrix_pipeline\n");

 // Software mode first
 execute_matrix_pipeline(0, MATRIX_SIZE); // 16x16 (max)
 execute_matrix_pipeline(0, 12); // 12x12
 execute_matrix_pipeline(0, 9); // 9x9

 // Hardware accelerated mode (same seed to compare)
 srand(8314);
 execute_matrix_pipeline(1, MATRIX_SIZE); // 16x16 (max)
 execute_matrix_pipeline(1, 12); // 12x12
 execute_matrix_pipeline(1, 9); // 9x9

 // Process in software mode first
 FLIGHT_LOG("\n[COM] SW mode\n");
 process_signal_spectrum(0);
 process_qam_modulation(0);
 estimate_channel_state(0);
 compress_telemetry_data(0);
 equalize_signal_adaptive(0);
 apply_error_correction(0);
 compute_orbit_link_geometry(0);
 process_wavelet_denoise(0);

 // Process with hardware acceleration
 FLIGHT_LOG("\n[COM] HW mode\n");
 process_signal_spectrum(1);
 process_qam_modulation(1);
 estimate_channel_state(1);
 compress_telemetry_data(1);
 equalize_signal_adaptive(1);
 apply_error_correction(1);
 compute_orbit_link_geometry(1);
 process_wavelet_denoise(1);

 // Print summary
 FLIGHT_LOG("\n[COM] summary\n");
 FLIGHT_LOG("FDIR fault count: %d\n", fdir_fault_count);

 return 0;
}
