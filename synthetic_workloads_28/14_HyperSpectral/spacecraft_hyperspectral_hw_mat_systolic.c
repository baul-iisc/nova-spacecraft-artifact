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
 * Hyperspectral Image Compression Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Mat (AME 3×3 tile accelerator for matrix operations)
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard hyperspectral image compression using principal component
 * analysis (PCA) for spectral decorrelation, block DCT for spatial transform,
 * DFT-based spectral complexity analysis (driving adaptive quantization), and
 * zigzag + run-length entropy coding. The pipeline mirrors a simplified CCSDS-123
 * compression chain. Configured with 24 spectral bands (a compact subset of the
 * full hyperspectral payloads) to keep gem5 simulation tractable.
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
/* ------------------------------------------------------------------ */
/* HW Helper: Multiply two float** matrices via SYS_MMACD 3×3 tiles */
/* C[m][n] = A[m][k] * B[k][n] */
/* If transB != 0, uses B^T instead (B[j][k] accessed as B^T[k][j]) */
/* ------------------------------------------------------------------ */
static void hw_matmul_2d(float **A, float **B, float **C,
 int m, int k, int n, int transB)
{
 /* ── Optimized: pre-transpose B as doubles → direct MLDS for BT ── */
 const int P = 3;
 const long s24 = 24;

 /* Pre-transpose B into flat double B[n][k] (or B^T if transB) */
 double *B_d = (double *)flight_malloc_impl((size_t)n * k * sizeof(double));
 if (transB) {
 /* B is n×k → transpose to k×n for SYS_MMACD */
 for (int ii = 0; ii < k; ii++)
 for (int jj = 0; jj < n; jj++)
 B_d[ii * n + jj] = (double)B[jj][ii];
 } else {
 /* B is k×n → copy as-is for SYS_MMACD */
 for (int ii = 0; ii < k; ii++)
 for (int jj = 0; jj < n; jj++)
 B_d[ii * n + jj] = (double)B[ii][jj];
 }

 for (int i = 0; i < m; i++)
 for (int j = 0; j < n; j++)
 C[i][j] = 0.0f;

 const long stride_bt = k * (long)sizeof(double);
 const long stride_n = n * (long)sizeof(double);

 for (int i = 0; i < m; i += P) {
 int mr = (i + P <= m) ? P : m - i;
 for (int j = 0; j < n; j += P) {
 int nr = (j + P <= n) ? P : n - j;
 asm volatile(MZERO(M2));
 for (int kk = 0; kk < k; kk += P) {
 int kr = (kk + P <= k) ? P : k - kk;
 /* Gather A tile with float→double (required) */
 double At[9] = {0};
 for (int r = 0; r < mr; r++)
 for (int c = 0; c < kr; c++)
 At[r * P + c] = (double)A[i + r][kk + c];

 if (nr == P && kr == P) {
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride_bt), "r"(At),
 "r"(stride_n), "r"(&B_d[kk * n + j])
 : "t0", "t1", "memory"
 );
 } else {
 double b_pad[9] = {0};
 for (int r = 0; r < nr; r++)
 for (int c = 0; c < kr; c++)
 b_pad[r * P + c] = B_d[(j + r) * k + kk + c];
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(s24), "r"(At),
 "r"(s24), "r"(b_pad)
 : "t0", "t1", "memory"
 );
 }
 }
 double Ct[9];
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(s24), "r"(Ct)
 : "t0", "t1", "memory"
 );
 for (int r = 0; r < mr; r++)
 for (int c = 0; c < nr; c++)
 C[i + r][j + c] = (float)Ct[r * P + c];
 }
 }
}


/* -----------------------------------------------------------------------
 * FDIR (Fault Detection, Isolation, and Recovery) Event Reporting
 *
 * In flight software, printf/exit are replaced by structured event packets
 * routed to the spacecraft telemetry stream and monitored by the FDIR
 * subsystem. We emulate that pattern here: every anomaly is logged via
 * fdir_event() with a severity level and event ID, allowing autonomous
 * recovery decisions rather than hard aborts.
 *
 * Severity levels follow ECSS-E-ST-70-41C Table 5-3:
 * INFO — nominal housekeeping (no action required)
 * WARNING — off-nominal but recoverable
 * ERROR — function-level failure, isolation initiated
 * CRITICAL— subsystem-level failure, safe-mode candidate
 * ----------------------------------------------------------------------- */
typedef enum {
 FDIR_INFO = 0,
 FDIR_WARNING = 1,
 FDIR_ERROR = 2,
 FDIR_CRITICAL = 3
} FdirSeverity;

typedef struct {
 unsigned int evt_count[4]; /* per-severity counters (INFO..CRITICAL) */
 unsigned int total;
} FdirLog;

static FdirLog fdir_log = {{0, 0, 0, 0}, 0};

static const char *fdir_sev_str[] = {"INFO", "WARN", "ERROR", "CRIT"};

/**
 * Emit an FDIR event packet to the telemetry stream.
 * In a real OBC this would enqueue a CCSDS event packet; here we use
 * printf as a gem5-compatible stand-in.
 */
static void fdir_event(FdirSeverity sev, unsigned int evt_id,
 const char *msg)
{
 fdir_log.evt_count[sev]++;
 fdir_log.total++;
 FLIGHT_LOG("[HSI] EVT %s #%04u: %s\n", fdir_sev_str[sev], evt_id, msg);
}

// Define constants for the hyperspectral image dimensions
#define BANDS 24 // Spectral bands (VNIR subset of hyperspectral 55-band payload)
#define HEIGHT 64 // Image height
#define WIDTH 64 // Image width
#define BLOCK_SIZE 8 // Processing block size for transforms
#define FFT_LEN 24 // DFT length = BANDS (spectral complexity analysis)
#define PI 3.14159265358979323846

// Structure for hyperspectral cube
typedef struct {
 float ***data; // 3D array [bands][height][width]
 int bands;
 int height;
 int width;
} HyperspectralCube;

// Structure for compressed data
typedef struct {
 unsigned char *data; // Compressed byte stream
 size_t size; // Size in bytes
} CompressedData;

// Function prototypes
HyperspectralCube* allocateHyperspectralCube(int bands, int height, int width);
void freeHyperspectralCube(HyperspectralCube *cube);
HyperspectralCube* generateSimulatedData(int bands, int height, int width);
void performSpatialTransform(float **band_data, int height, int width);
void performDiscreteCosineTransform(float **block, int size);
void performQuantization(float **block, int size, float quality_factor);
CompressedData* performEntropyCoding(float ***reduced_transformed_data, int bands, int height, int width);
void writeCompressedData(CompressedData *compressed, const char *filename);
float** allocate2DArray(int rows, int cols);
void free2DArray(float **array, int rows);
float*** allocate3DArray(int depth, int rows, int cols);
void free3DArray(float ***array, int depth, int rows);
void applyWaveletTransform(float **data, int height, int width);
void runLengthEncoding(float *input, int length, float *output, int *output_length);
void performPrincipalComponentAnalysis(HyperspectralCube *cube, float ***reduced_data, int reduced_bands);
void transpose(float **src, int src_rows, int src_cols, float **dst);
void covariance(float **data, int rows, int cols, float **cov);
void eigenDecomposition(float **cov, int n, float **eigenvectors, float *eigenvalues);
void sortEigenPairs(float **eigenvectors, float *eigenvalues, int n);
void projectData(float ***data, int bands, int height, int width, float **projection_matrix, int reduced_bands, float ***reduced_data);
void compute_spectral_dft(float *spectral_signal, int len, float *magnitude);

// Main function
int main(int argc, char *argv[]) {
 /* Deterministic seed for reproducible gem5 execution traces. */
 srand(6397);

 // Handle command line arguments
 int num_bands = BANDS;
 int img_height = HEIGHT;
 int img_width = WIDTH;
 int reduced_bands = 11; /* Retain ~95% variance (hyperspectral typical) */
 float quality_factor = 0.78f;
 const char *output_file = "compressed.bin";

 // Parse arguments if provided
 if (argc > 1) {
 for (int i = 1; i < argc; i++) {
 if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
 num_bands = atoi(argv[i + 1]);
 i++;
 } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
 img_height = atoi(argv[i + 1]);
 i++;
 } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
 img_width = atoi(argv[i + 1]);
 i++;
 } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
 reduced_bands = atoi(argv[i + 1]);
 i++;
 } else if (strcmp(argv[i], "-q") == 0 && i + 1 < argc) {
 quality_factor = atof(argv[i + 1]);
 i++;
 } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
 output_file = argv[i + 1];
 i++;
 } else if (strcmp(argv[i], "--help") == 0) {
 FLIGHT_LOG("Usage: %s [-b bands] [-h height] [-w width] [-r reduced_bands] [-q quality] [-o output]\n", argv[0]);
 return 0;
 }
 }
 }

 FLIGHT_LOG("[HSI] init\n");
 FLIGHT_LOG("Dimensions: %d bands x %d height x %d width\n", num_bands, img_height, img_width);
 FLIGHT_LOG("Reduced bands: %d\n", reduced_bands);

 // Start timing
 clock_t start = clock();

 // Generate or load hyperspectral data
 HyperspectralCube *cube = generateSimulatedData(num_bands, img_height, img_width);
 if (!cube) {
 fdir_event(FDIR_CRITICAL, 0x0100, "hyperspectral cube alloc failed");
 FLIGHT_LOG("[HSI] FDIR summary: INFO=%u WARN=%u ERR=%u CRIT=%u\n",
 fdir_log.evt_count[0], fdir_log.evt_count[1],
 fdir_log.evt_count[2], fdir_log.evt_count[3]);
 return 1;
 }

 FLIGHT_LOG("Data generation complete\n");

 // Allocate space for reduced data
 float ***reduced_data = allocate3DArray(reduced_bands, img_height, img_width);
 if (!reduced_data) {
 fdir_event(FDIR_CRITICAL, 0x0101, "reduced_data alloc failed — isolating PCA stage");
 freeHyperspectralCube(cube);
 FLIGHT_LOG("[HSI] FDIR summary: INFO=%u WARN=%u ERR=%u CRIT=%u\n",
 fdir_log.evt_count[0], fdir_log.evt_count[1],
 fdir_log.evt_count[2], fdir_log.evt_count[3]);
 return 1;
 }

 // Step 1: Spectral dimension reduction (PCA)
 FLIGHT_LOG("Performing spectral dimension reduction...\n");
 performPrincipalComponentAnalysis(cube, reduced_data, reduced_bands);
 FLIGHT_LOG("Spectral dimension reduction complete\n");

 // Step 1b: DFT spectral complexity — drives adaptive quantization quality
 FLIGHT_LOG("Performing spectral DFT analysis...\n");
 double mean_complexity = 0.0;
 int n_samples = 0;
 {
 float spectral_signal[FFT_LEN], magnitude[FFT_LEN];
 for (int y = 0; y < img_height; y += 4) {
 for (int x = 0; x < img_width; x += 4) {
 for (int b = 0; b < num_bands && b < FFT_LEN; b++)
 spectral_signal[b] = cube->data[b][y][x];
 for (int b = num_bands; b < FFT_LEN; b++)
 spectral_signal[b] = 0.0f;
 compute_spectral_dft(spectral_signal, FFT_LEN, magnitude);

 /* Spectral complexity: ratio of high-freq to low-freq energy */
 float lo = 0.0f, hi = 0.0f;
 for (int k = 0; k < FFT_LEN; k++) {
 if (k < FFT_LEN / 3) lo += magnitude[k];
 else hi += magnitude[k];
 }
 mean_complexity += (double)(hi / (lo + 1e-6f));
 n_samples++;
 }
 }
 }
 mean_complexity /= n_samples;
 /* Adapt quantization: high spectral complexity → finer quantisation */
 if (mean_complexity > 1.35)
 quality_factor = fmin(quality_factor + 0.08f, 0.98f);
 else if (mean_complexity < 0.45)
 quality_factor = fmax(quality_factor - 0.12f, 0.18f);
 FLIGHT_LOG("Spectral DFT analysis complete (mean complexity %.3f, quality %.2f)\n",
 mean_complexity, quality_factor);

 // Step 2: Apply spatial transforms (DCT or wavelet) to each reduced band
 FLIGHT_LOG("Performing spatial transforms...\n");
 for (int band = 0; band < reduced_bands; band++) {
 performSpatialTransform(reduced_data[band], img_height, img_width);
 }
 FLIGHT_LOG("Spatial transforms complete\n");

 // Step 3: Quantize the transformed coefficients
 FLIGHT_LOG("Performing quantization...\n");
 for (int band = 0; band < reduced_bands; band++) {
 for (int y = 0; y < img_height; y += BLOCK_SIZE) {
 for (int x = 0; x < img_width; x += BLOCK_SIZE) {
 // Extract block
 float **block = allocate2DArray(BLOCK_SIZE, BLOCK_SIZE);
 for (int i = 0; i < BLOCK_SIZE; i++) {
 for (int j = 0; j < BLOCK_SIZE; j++) {
 if (y + i < img_height && x + j < img_width) {
 block[i][j] = reduced_data[band][y + i][x + j];
 } else {
 block[i][j] = 0.0;
 }
 }
 }

 // Quantize block
 performQuantization(block, BLOCK_SIZE, quality_factor);

 // Put quantized data back
 for (int i = 0; i < BLOCK_SIZE; i++) {
 for (int j = 0; j < BLOCK_SIZE; j++) {
 if (y + i < img_height && x + j < img_width) {
 reduced_data[band][y + i][x + j] = block[i][j];
 }
 }
 }

 free2DArray(block, BLOCK_SIZE);
 }
 }
 }
 FLIGHT_LOG("Quantization complete\n");

 // Step 4: Run-length entropy coding (zigzag scan + RLE)
 FLIGHT_LOG("Performing entropy coding...\n");
 CompressedData *compressed = performEntropyCoding(reduced_data, reduced_bands, img_height, img_width);
 if (!compressed) {
 fdir_event(FDIR_ERROR, 0x0200, "entropy coding failed — skipping downlink");
 free3DArray(reduced_data, reduced_bands, img_height);
 freeHyperspectralCube(cube);
 FLIGHT_LOG("[HSI] FDIR summary: INFO=%u WARN=%u ERR=%u CRIT=%u\n",
 fdir_log.evt_count[0], fdir_log.evt_count[1],
 fdir_log.evt_count[2], fdir_log.evt_count[3]);
 return 1;
 }
 FLIGHT_LOG("Entropy coding complete\n");

 // Write compressed data to file
 writeCompressedData(compressed, output_file);

 // Calculate compression ratio
 size_t original_size = (size_t)num_bands * img_height * img_width * sizeof(float);
 double compression_ratio = (double)original_size / compressed->size;
 FLIGHT_LOG("Original size: %zu bytes\n", original_size);
 FLIGHT_LOG("Compressed size: %zu bytes\n", compressed->size);
 FLIGHT_LOG("Compression ratio: %.2f:1\n", compression_ratio);

 // Clean up
 flight_free_impl(compressed->data);
 flight_free_impl(compressed);
 free3DArray(reduced_data, reduced_bands, img_height);
 freeHyperspectralCube(cube);

 // End timing
 clock_t end = clock();
 double time_spent = (double)(end - start) / CLOCKS_PER_SEC;
 FLIGHT_LOG("Total execution time: %.2f seconds\n", time_spent);

 /* FDIR health summary — in flight, this goes into the housekeeping TM frame */
 FLIGHT_LOG("[HSI] FDIR summary: INFO=%u WARN=%u ERR=%u CRIT=%u\n",
 fdir_log.evt_count[0], fdir_log.evt_count[1],
 fdir_log.evt_count[2], fdir_log.evt_count[3]);

 return 0;
}

// Allocate memory for a hyperspectral cube
HyperspectralCube* allocateHyperspectralCube(int bands, int height, int width) {
 HyperspectralCube *cube = (HyperspectralCube*)flight_malloc_impl(sizeof(HyperspectralCube));
 if (!cube) return NULL;

 cube->bands = bands;
 cube->height = height;
 cube->width = width;
 cube->data = allocate3DArray(bands, height, width);

 if (!cube->data) {
 flight_free_impl(cube);
 return NULL;
 }

 return cube;
}

// Free memory for a hyperspectral cube
void freeHyperspectralCube(HyperspectralCube *cube) {
 if (cube) {
 if (cube->data) {
 free3DArray(cube->data, cube->bands, cube->height);
 }
 flight_free_impl(cube);
 }
}

// Generate simulated hyperspectral data for processing
HyperspectralCube* generateSimulatedData(int bands, int height, int width) {
 HyperspectralCube *cube = allocateHyperspectralCube(bands, height, width);
 if (!cube) return NULL;

 // Create simulated data with spectral and spatial correlations
 for (int band = 0; band < bands; band++) {
 float band_factor = sin(band * PI / bands) * 87.5f;

 for (int y = 0; y < height; y++) {
 for (int x = 0; x < width; x++) {
 // Create spatial patterns
 float spatial_pattern = sin(x * PI / 64.0f) * cos(y * PI / 64.0f) * 50.0f;

 // Add some spectral signature variations
 float spectral_signature = band_factor * (1.0f + 0.2f * sin(band * x * y * 0.0001f));

 // Combine spatial and spectral patterns
 cube->data[band][y][x] = spatial_pattern + spectral_signature +
 (rand() % 20) - 10; // Add some noise
 }
 }
 }

 return cube;
}

// Allocate a 2D array
float** allocate2DArray(int rows, int cols) {
 float **array = (float**)flight_malloc_impl(rows * sizeof(float*));
 if (!array) return NULL;

 for (int i = 0; i < rows; i++) {
 array[i] = (float*)flight_malloc_impl(cols * sizeof(float));
 if (!array[i]) {
 // Clean up on failure
 for (int j = 0; j < i; j++) {
 flight_free_impl(array[j]);
 }
 flight_free_impl(array);
 return NULL;
 }
 }

 return array;
}

// Free a 2D array
void free2DArray(float **array, int rows) {
 if (array) {
 for (int i = 0; i < rows; i++) {
 if (array[i]) {
 flight_free_impl(array[i]);
 }
 }
 flight_free_impl(array);
 }
}

// Allocate a 3D array
float*** allocate3DArray(int depth, int rows, int cols) {
 float ***array = (float***)flight_malloc_impl(depth * sizeof(float**));
 if (!array) return NULL;

 for (int i = 0; i < depth; i++) {
 array[i] = allocate2DArray(rows, cols);
 if (!array[i]) {
 // Clean up on failure
 for (int j = 0; j < i; j++) {
 free2DArray(array[j], rows);
 }
 flight_free_impl(array);
 return NULL;
 }
 }

 return array;
}

// Free a 3D array
void free3DArray(float ***array, int depth, int rows) {
 if (array) {
 for (int i = 0; i < depth; i++) {
 free2DArray(array[i], rows);
 }
 flight_free_impl(array);
 }
}

// Perform Discrete Cosine Transform on a block
void performDiscreteCosineTransform(float **block, int size) {
 float **temp = allocate2DArray(size, size);
 float **result = allocate2DArray(size, size);

 // Initialize DCT coefficient matrices
 float **dct_matrix = allocate2DArray(size, size);

 // Compute DCT transformation matrix
 for (int i = 0; i < size; i++) {
 for (int j = 0; j < size; j++) {
 if (i == 0) {
 dct_matrix[i][j] = 1.0f / sqrt(size);
 } else {
 dct_matrix[i][j] = sqrt(2.0f / size) * cos((2.0f * j + 1.0f) * i * PI / (2.0f * size));
 }
 }
 }

 // Perform 2D DCT: result = dct_matrix * block * dct_matrix^T
 // First step: temp = dct_matrix * block (AME 3×3 tiled)
 hw_matmul_2d(dct_matrix, block, temp, size, size, size, 0);

 // Second step: result = temp * dct_matrix^T (AME 3×3 tiled, transB=1)
 hw_matmul_2d(temp, dct_matrix, result, size, size, size, 1);

 // Copy result back to input block
 for (int i = 0; i < size; i++) {
 for (int j = 0; j < size; j++) {
 block[i][j] = result[i][j];
 }
 }

 // Clean up
 free2DArray(temp, size);
 free2DArray(result, size);
 free2DArray(dct_matrix, size);
}

// Perform Inverse Discrete Cosine Transform on a block (reference decompressor path)

// Perform Haar wavelet transform on an image band
void applyWaveletTransform(float **data, int height, int width) {
 // Apply 1D Haar wavelet transform horizontally
 float *temp = (float*)flight_malloc_impl(width * sizeof(float));

 // Process each row
 for (int y = 0; y < height; y++) {
 // Copy row to temp buffer
 memcpy(temp, data[y], width * sizeof(float));

 // Apply wavelet transform
 for (int length = width; length >= 2; length /= 2) {
 for (int i = 0; i < length/2; i++) {
 float avg = (temp[2*i] + temp[2*i+1]) / 2.0f;
 float diff = (temp[2*i] - temp[2*i+1]) / 2.0f;
 temp[i] = avg;
 temp[i + length/2] = diff;
 }
 }

 // Copy transformed data back to row
 memcpy(data[y], temp, width * sizeof(float));
 }

 // Process each column
 float *col_temp = (float*)flight_malloc_impl(height * sizeof(float));

 for (int x = 0; x < width; x++) {
 // Copy column to temp buffer
 for (int y = 0; y < height; y++) {
 col_temp[y] = data[y][x];
 }

 // Apply wavelet transform
 for (int length = height; length >= 2; length /= 2) {
 for (int i = 0; i < length/2; i++) {
 float avg = (col_temp[2*i] + col_temp[2*i+1]) / 2.0f;
 float diff = (col_temp[2*i] - col_temp[2*i+1]) / 2.0f;
 col_temp[i] = avg;
 col_temp[i + length/2] = diff;
 }
 }

 // Copy transformed data back to column
 for (int y = 0; y < height; y++) {
 data[y][x] = col_temp[y];
 }
 }

 flight_free_impl(temp);
 flight_free_impl(col_temp);
}

// Apply spatial transform to a band (block-wise DCT)
void performSpatialTransform(float **band_data, int height, int width) {
 // Block-wise Type-II DCT
 for (int y = 0; y < height; y += BLOCK_SIZE) {
 for (int x = 0; x < width; x += BLOCK_SIZE) {
 // Extract block
 float **block = allocate2DArray(BLOCK_SIZE, BLOCK_SIZE);
 for (int i = 0; i < BLOCK_SIZE; i++) {
 for (int j = 0; j < BLOCK_SIZE; j++) {
 if (y + i < height && x + j < width) {
 block[i][j] = band_data[y + i][x + j];
 } else {
 block[i][j] = 0.0; // Zero padding for blocks at the edges
 }
 }
 }

 // Apply DCT to block
 performDiscreteCosineTransform(block, BLOCK_SIZE);

 // Put transformed data back
 for (int i = 0; i < BLOCK_SIZE; i++) {
 for (int j = 0; j < BLOCK_SIZE; j++) {
 if (y + i < height && x + j < width) {
 band_data[y + i][x + j] = block[i][j];
 }
 }
 }

 free2DArray(block, BLOCK_SIZE);
 }
 }
}

// Quantize transformed coefficients
void performQuantization(float **block, int size, float quality_factor) {
 // Standard JPEG quantization matrix (scaled with quality factor)
 int standard_matrix[8][8] = {
 {16, 11, 10, 16, 24, 40, 51, 61},
 {12, 12, 14, 19, 26, 58, 60, 55},
 {14, 13, 16, 24, 40, 57, 69, 56},
 {14, 17, 22, 29, 51, 87, 80, 62},
 {18, 22, 37, 56, 68, 109, 103, 77},
 {24, 35, 55, 64, 81, 104, 113, 92},
 {49, 64, 78, 87, 103, 121, 120, 101},
 {72, 92, 95, 98, 112, 100, 103, 99}
 };

 // Scale matrix with quality factor (1.0 = highest quality, 0.1 = lowest)
 float scale_factor = quality_factor < 0.5 ?
 50.0f / quality_factor :
 2.0f - 2.0f * quality_factor;

 // Quantize coefficients
 for (int i = 0; i < size && i < 8; i++) {
 for (int j = 0; j < size && j < 8; j++) {
 float q_value = standard_matrix[i][j] * scale_factor;
 block[i][j] = round(block[i][j] / q_value) * q_value;
 }
 }

 // For blocks larger than 8x8, use a simple scalar quantization for the rest
 if (size > 8) {
 float base_q = 100.0f * (1.0f - quality_factor);
 for (int i = 0; i < size; i++) {
 for (int j = 0; j < size; j++) {
 if (i >= 8 || j >= 8) {
 float q_value = base_q * (1.0f + (i + j) / 16.0f);
 block[i][j] = round(block[i][j] / q_value) * q_value;
 }
 }
 }
 }
}

// Perform run-length encoding
void runLengthEncoding(float *input, int length, float *output, int *output_length) {
 int count = 0;
 int out_idx = 0;

 for (int i = 0; i < length; i++) {
 count = 1;
 float value = input[i];

 while (i + 1 < length && input[i] == input[i + 1]) {
 count++;
 i++;
 }

 // Store count and value
 output[out_idx++] = (float)count;
 output[out_idx++] = value;
 }

 *output_length = out_idx;
}

// Matrix transpose
void transpose(float **src, int src_rows, int src_cols, float **dst) {
 for (int i = 0; i < src_rows; i++) {
 for (int j = 0; j < src_cols; j++) {
 dst[j][i] = src[i][j];
 }
 }
}

// Covariance matrix calculation
void covariance(float **data, int rows, int cols, float **cov) {
 // Calculate mean of each column
 float *means = (float*)flight_calloc_impl(cols, sizeof(float));
 for (int j = 0; j < cols; j++) {
 means[j] = 0.0f;
 for (int i = 0; i < rows; i++) {
 means[j] += data[i][j];
 }
 means[j] /= rows;
 }

 // Calculate covariance matrix
 for (int i = 0; i < cols; i++) {
 for (int j = 0; j <= i; j++) {
 cov[i][j] = 0.0f;
 for (int k = 0; k < rows; k++) {
 cov[i][j] += (data[k][i] - means[i]) * (data[k][j] - means[j]);
 }
 cov[i][j] /= (rows - 1);
 // Symmetry
 if (i != j) {
 cov[j][i] = cov[i][j];
 }
 }
 }

 flight_free_impl(means);
}

// Simple power iteration method for eigendecomposition
// Power iteration with Rayleigh quotient and deflation
void eigenDecomposition(float **cov, int n, float **eigenvectors, float *eigenvalues) {
 // Allocate temporary vectors
 float *vec = (float*)flight_malloc_impl(n * sizeof(float));
 float *next_vec = (float*)flight_malloc_impl(n * sizeof(float));

 // Find largest eigenvalues and eigenvectors using power iteration
 for (int k = 0; k < n; k++) {
 // Initialize random vector
 for (int i = 0; i < n; i++) {
 vec[i] = (float)rand() / RAND_MAX;
 }

 // Normalize
 float norm = 0.0f;
 for (int i = 0; i < n; i++) {
 norm += vec[i] * vec[i];
 }
 norm = sqrt(norm);
 for (int i = 0; i < n; i++) {
 vec[i] /= norm;
 }

 // Power iteration
 for (int iter = 0; iter < 100; iter++) {
 // Matrix-vector multiplication
 for (int i = 0; i < n; i++) {
 next_vec[i] = 0.0f;
 for (int j = 0; j < n; j++) {
 next_vec[i] += cov[i][j] * vec[j];
 }
 }

 // Normalize
 norm = 0.0f;
 for (int i = 0; i < n; i++) {
 norm += next_vec[i] * next_vec[i];
 }
 norm = sqrt(norm);
 for (int i = 0; i < n; i++) {
 next_vec[i] /= norm;
 }

 // Copy back
 for (int i = 0; i < n; i++) {
 vec[i] = next_vec[i];
 }
 }

 // Calculate eigenvalue (Rayleigh quotient)
 float eigenvalue = 0.0f;
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 eigenvalue += vec[i] * cov[i][j] * vec[j];
 }
 }

 // Store eigenvalue and eigenvector
 eigenvalues[k] = eigenvalue;
 for (int i = 0; i < n; i++) {
 eigenvectors[k][i] = vec[i];
 }

 // Deflate the matrix (subtract this component)
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 cov[i][j] -= eigenvalue * vec[i] * vec[j];
 }
 }
 }

 flight_free_impl(vec);
 flight_free_impl(next_vec);
}

// Sort eigenvalues and eigenvectors in descending order
void sortEigenPairs(float **eigenvectors, float *eigenvalues, int n) {
 // Use bubble sort for sorting
 for (int i = 0; i < n - 1; i++) {
 for (int j = 0; j < n - i - 1; j++) {
 if (eigenvalues[j] < eigenvalues[j + 1]) {
 // Swap eigenvalues
 float temp_val = eigenvalues[j];
 eigenvalues[j] = eigenvalues[j + 1];
 eigenvalues[j + 1] = temp_val;

 // Swap eigenvectors (rows of eigenvectors matrix)
 for (int k = 0; k < n; k++) {
 float temp_vec = eigenvectors[j][k];
 eigenvectors[j][k] = eigenvectors[j + 1][k];
 eigenvectors[j + 1][k] = temp_vec;
 }
 }
 }
 }
}

// Project data onto principal components
void projectData(float ***data, int bands, int height, int width, float **projection_matrix, int reduced_bands, float ***reduced_data) {
 // For each pixel location
 for (int y = 0; y < height; y++) {
 for (int x = 0; x < width; x++) {
 // Extract spectral vector for this pixel
 float *spectral_vector = (float*)flight_malloc_impl(bands * sizeof(float));
 for (int b = 0; b < bands; b++) {
 spectral_vector[b] = data[b][y][x];
 }

 // Project onto each of the reduced bands
 for (int rb = 0; rb < reduced_bands; rb++) {
 reduced_data[rb][y][x] = 0.0f;
 for (int b = 0; b < bands; b++) {
 reduced_data[rb][y][x] += projection_matrix[rb][b] * spectral_vector[b];
 }
 }

 flight_free_impl(spectral_vector);
 }
 }
}

// Principal Component Analysis for spectral dimension reduction
void performPrincipalComponentAnalysis(HyperspectralCube *cube, float ***reduced_data, int reduced_bands) {
 int bands = cube->bands;
 int height = cube->height;
 int width = cube->width;

 // Step 1: Reshape the cube into a 2D matrix (bands x pixels)
 int n_pixels = height * width;
 float **data_matrix = allocate2DArray(bands, n_pixels);

 for (int b = 0; b < bands; b++) {
 int pixel_idx = 0;
 for (int y = 0; y < height; y++) {
 for (int x = 0; x < width; x++) {
 data_matrix[b][pixel_idx++] = cube->data[b][y][x];
 }
 }
 }

 // Step 2: Calculate the covariance matrix
 float **cov_matrix = allocate2DArray(bands, bands);
 float **data_matrix_t = allocate2DArray(n_pixels, bands);

 // Transpose data matrix for covariance calculation
 transpose(data_matrix, bands, n_pixels, data_matrix_t);

 // Calculate covariance matrix
 covariance(data_matrix_t, n_pixels, bands, cov_matrix);

 // Step 3: Perform eigendecomposition
 float **eigenvectors = allocate2DArray(bands, bands);
 float *eigenvalues = (float*)flight_malloc_impl(bands * sizeof(float));

 eigenDecomposition(cov_matrix, bands, eigenvectors, eigenvalues);

 // Step 4: Sort eigenvalues and eigenvectors
 sortEigenPairs(eigenvectors, eigenvalues, bands);

 // Step 5: Extract the top principal components
 float **projection_matrix = allocate2DArray(reduced_bands, bands);
 for (int i = 0; i < reduced_bands; i++) {
 for (int j = 0; j < bands; j++) {
 projection_matrix[i][j] = eigenvectors[i][j];
 }
 }

 // Step 6: Project the data onto the reduced space
 projectData(cube->data, bands, height, width, projection_matrix, reduced_bands, reduced_data);

 // Clean up
 free2DArray(data_matrix, bands);
 free2DArray(data_matrix_t, n_pixels);
 free2DArray(cov_matrix, bands);
 free2DArray(eigenvectors, bands);
 flight_free_impl(eigenvalues);
 free2DArray(projection_matrix, reduced_bands);
}

// Compact entropy encoder
CompressedData* performEntropyCoding(float ***reduced_transformed_data, int bands, int height, int width) {
 // Allocate compressed data structure
 CompressedData *compressed = (CompressedData*)flight_malloc_impl(sizeof(CompressedData));
 if (!compressed) return NULL;

 // For entropy coding, we do zigzag scanning and run-length encoding
 // Following stage: Huffman or arithmetic entropy coding

 // Allocate buffer for compressed data (worst case: no compression)
 size_t max_size = 2 * bands * height * width * sizeof(float); // Factor of 2 for RLE overhead
 compressed->data = (unsigned char*)flight_malloc_impl(max_size);
 if (!compressed->data) {
 flight_free_impl(compressed);
 return NULL;
 }

 // Current position in output buffer
 size_t output_pos = 0;

 // Store dimensions in the compressed data
 ((int*)compressed->data)[0] = bands;
 ((int*)compressed->data)[1] = height;
 ((int*)compressed->data)[2] = width;
 output_pos += 3 * sizeof(int);

 // Process each band
 for (int band = 0; band < bands; band++) {
 // Process blocks
 for (int y = 0; y < height; y += BLOCK_SIZE) {
 for (int x = 0; x < width; x += BLOCK_SIZE) {
 // Extract block
 float block[BLOCK_SIZE * BLOCK_SIZE];
 int block_idx = 0;

 // Zigzag scanning of the block
 int zigzag_x, zigzag_y;
 for (int sum = 0; sum < 2 * BLOCK_SIZE - 1; sum++) {
 for (int i = 0; i <= sum && i < BLOCK_SIZE; i++) {
 if (sum % 2 == 0) { // Even sum: down-right diagonal
 zigzag_y = (sum < BLOCK_SIZE) ? i : (sum - BLOCK_SIZE + 1 + i);
 zigzag_x = (sum < BLOCK_SIZE) ? (sum - i) : (BLOCK_SIZE - 1 - i);
 } else { // Odd sum: up-left diagonal
 zigzag_y = (sum < BLOCK_SIZE) ? (sum - i) : (BLOCK_SIZE - 1 - i);
 zigzag_x = (sum < BLOCK_SIZE) ? i : (sum - BLOCK_SIZE + 1 + i);
 }

 // Check bounds
 if (zigzag_y < BLOCK_SIZE && zigzag_x < BLOCK_SIZE &&
 y + zigzag_y < height && x + zigzag_x < width) {
 block[block_idx++] = reduced_transformed_data[band][y + zigzag_y][x + zigzag_x];
 } else {
 block[block_idx++] = 0.0f;
 }
 }
 }

 // Run-length encode the block
 float rle_output[2 * BLOCK_SIZE * BLOCK_SIZE]; // Worst case: no runs
 int rle_length;
 runLengthEncoding(block, BLOCK_SIZE * BLOCK_SIZE, rle_output, &rle_length);

 // Store RLE length
 ((int*)(compressed->data + output_pos))[0] = rle_length;
 output_pos += sizeof(int);

 // Store RLE data
 memcpy(compressed->data + output_pos, rle_output, rle_length * sizeof(float));
 output_pos += rle_length * sizeof(float);
 }
 }
 }

 compressed->size = output_pos;
 return compressed;
}

// Write compressed data to a file
void writeCompressedData(CompressedData *compressed, const char *filename) {
 FILE *file = FLIGHT_FOPEN(filename, "wb");
 if (!file) {
 fdir_event(FDIR_WARNING, 0x0300, "output file open failed — data retained in memory");
 return;
 }

 FLIGHT_FWRITE(&compressed->size, sizeof(size_t), 1, file);
 FLIGHT_FWRITE(compressed->data, 1, compressed->size, file);

 FLIGHT_FCLOSE(file);
 FLIGHT_LOG("Compressed data written to %s\n", filename);
}

// Compute DFT magnitude spectrum for spectral frequency analysis
void compute_spectral_dft(float *spectral_signal, int len, float *magnitude) {
 for (int k = 0; k < len; k++) {
 float real_sum = 0.0f, imag_sum = 0.0f;
 for (int n = 0; n < len; n++) {
 float angle = 2.0f * PI * k * n / len;
 real_sum += spectral_signal[n] * cos(angle);
 imag_sum -= spectral_signal[n] * sin(angle);
 }
 magnitude[k] = sqrt(real_sum * real_sum + imag_sum * imag_sum);
 }
}
