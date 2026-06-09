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
 * CCSDS 122.0-B Image Data Compression
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the CCSDS 122.0-B-2 image data compression standard for onboard
 * image compression prior to downlink. This workload performs discrete wavelet
 * transform (DWT), bit-plane encoding (BPE), entropy coding, and DFT-based
 * quality assessment. The CCSDS 122.0 standard is adopted by for all
 * Earth observation payload data compression on Earth observation satellite, Earth resource satellite , and
 * EOS-series satellites to maximize downlink data throughput.
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
#include <stdint.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include "../flight_compliance.h"

/* AME 3×3 tile matrix accelerator */

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
/* CCSDS 122.0-B Parameters */
#define BLOCK_SIZE 8 // Standard block size for DCT/DWT
#define MAX_SEGMENT_ROWS 64 // Maximum rows per segment
#define MAX_BLOCKS_PER_SEGMENT (MAX_SEGMENT_ROWS / BLOCK_SIZE)
#define MAX_Z_SIZE 15 // Maximum size of Z parameter
#define NUM_SUBBANDS 10 // 3*3+1 subbands (3 levels of DWT + DC)
#define MAX_DWT_LEVELS 3 // Maximum wavelet decomposition levels
#define FFT_LEN 64 // DFT length for subband quality analysis (power-of-2)

/* Use clock() for timing instead of clock_gettime */
double measure_time_diff(clock_t start, clock_t end) {
 return (double)(end - start) / CLOCKS_PER_SEC;
}

/* Memory allocation safety */
#define SAFE_MALLOC(ptr, size) do { \
 ptr = flight_malloc_impl(size); \
 if (!(ptr)) { \
 FLIGHT_LOG("Memory allocation failed at %s:%d\n", __FILE__, __LINE__); \
 return NULL; \
 } \
} while(0)

#define SAFE_CALLOC(ptr, nmemb, size) do { \
 ptr = flight_calloc_impl(nmemb, size); \
 if (!(ptr)) { \
 FLIGHT_LOG("Memory allocation failed at %s:%d\n", __FILE__, __LINE__); \
 return NULL; \
 } \
} while(0)

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

/*
 * Spacecraft-specific error handling macros
 */
#define SPACECRAFT_ASSERT(condition, message) do { \
 if (!(condition)) { \
 FLIGHT_LOG("Assertion failed: %s at %s:%d\n", message, __FILE__, __LINE__); \
 return -1; \
 } \
} while(0)

#define SPACECRAFT_CHECK_PTR(ptr) do { \
 if (!(ptr)) { \
 FLIGHT_LOG("NULL pointer detected at %s:%d\n", __FILE__, __LINE__); \
 return -1; \
 } \
} while(0)

/* Counters for memory operations - Added to match hardware implementation */
typedef struct {
 long long reads;
 long long writes;
} mem_counter_t;

/* Data Structures */

/* Image Data Structure */
typedef struct {
 int width; // Width of image in pixels
 int height; // Height of image in pixels
 int bit_depth; // Bit depth of image (8 to 16 bits)
 int signed_pixels; // 0 for unsigned, 1 for signed pixels
 uint16_t *data; // Raw image data
} Image;

/* Matrix Structure for high-dimensional operations */
typedef struct {
 int rows; // Number of rows
 int cols; // Number of columns
 int depth; // Depth (for 3D matrices)
 float *data; // Matrix data in row-major format
} Matrix;

/* Segment Data Structure */
typedef struct {
 int segment_width; // Width of segment in blocks
 int segment_height; // Height of segment in blocks
 Matrix **blocks; // Array of matrices for transformed blocks
 int *dc_coeffs; // DC coefficients
 float *ac_coeffs; // AC coefficients
 int max_bit_plane; // Maximum bit plane for this segment
} Segment;

/* Context and State Variables for BPE */
typedef struct {
 int bit_plane; // Current bit plane being processed
 int max_bit_plane; // Maximum bit plane value
 int *priorities; // Priorities for each coefficient
 uint8_t *coded_stream; // Compressed output stream
 size_t stream_size; // Size of compressed stream in bytes
 size_t stream_capacity; // Capacity of coded_stream buffer
 uint32_t buffer; // Bit buffer for output
 int bits_in_buffer; // Number of bits in buffer
 int z_param; // CCSDS Z parameter for quantization
} BPEContext;

/* Function prototypes */

/* Matrix operations */
Matrix* matrix_create(int rows, int cols, int depth);
void matrix_free(Matrix *mat);
Matrix* matrix_clone(Matrix *src);
float matrix_get(Matrix *mat, int row, int col, int depth);
void matrix_set(Matrix *mat, int row, int col, int depth, float value);
float matrix_get_max_abs(Matrix *mat);

/* Trigonometric operations */
float compute_cosine(float angle);
float compute_sine(float angle);

/* Discrete Wavelet Transform */
int dwt_forward_97_sw_systolic(float *data, int length, mem_counter_t *counter);
int dwt_2d_forward_sw_systolic(Matrix *data, int levels, mem_counter_t *counter);

/* Bit Plane Encoding */
BPEContext* init_bpe_context(int max_bit_plane, int num_coeffs, int z_param);
void free_bpe_context(BPEContext *ctx);
int write_bit(BPEContext *ctx, int bit);
int flush_bits(BPEContext *ctx);
int is_significant(float coeff, int bit_plane);
int encode_block(BPEContext *ctx, Matrix *block, int *dc_coeff);
int encode_segment(BPEContext *ctx, Segment *segment);

/* Segment Processing */
Segment* init_segment(int segment_width, int segment_height);
void free_segment(Segment *segment);
int process_segment(Segment *segment, Image *image, int segment_x, int segment_y, int *output_size, uint8_t **output_data, mem_counter_t *counter);

/* Image Processing */
int compress_ccsds122(Image *image, uint8_t **output_data, int *output_size, int z_param, mem_counter_t *counter);
Image* load_raw_image(const char *filename, int width, int height, int bit_depth);
int save_compressed(const char *filename, uint8_t *data, int size);
void free_image(Image *img);

/* Create a new matrix with given dimensions */
Matrix* matrix_create(int rows, int cols, int depth) {
 Matrix *mat;

 if (rows <= 0 || cols <= 0 || depth <= 0) {
 FLIGHT_LOG("Invalid matrix dimensions: %d x %d x %d\n", rows, cols, depth);
 return NULL;
 }

 SAFE_MALLOC(mat, sizeof(Matrix));

 mat->rows = rows;
 mat->cols = cols;
 mat->depth = depth;

 SAFE_CALLOC(mat->data, rows * cols * depth, sizeof(float));

 return mat;
}

/* Free matrix resources */
void matrix_free(Matrix *mat) {
 if (mat) {
 flight_free_impl(mat->data);
 flight_free_impl(mat);
 }
}

/* Clone a matrix */
Matrix* matrix_clone(Matrix *src) {
 if (!src) return NULL;

 Matrix *dst = matrix_create(src->rows, src->cols, src->depth);
 if (!dst) return NULL;

 memcpy(dst->data, src->data, src->rows * src->cols * src->depth * sizeof(float));

 return dst;
}

/* Get matrix element */
float matrix_get(Matrix *mat, int row, int col, int depth) {
 if (!mat || row < 0 || col < 0 || depth < 0 ||
 row >= mat->rows || col >= mat->cols || depth >= mat->depth) {
 return 0.0f;
 }

 return mat->data[(row * mat->cols + col) * mat->depth + depth];
}

/* Set matrix element */
void matrix_set(Matrix *mat, int row, int col, int depth, float value) {
 if (!mat || row < 0 || col < 0 || depth < 0 ||
 row >= mat->rows || col >= mat->cols || depth >= mat->depth) {
 return;
 }

 mat->data[(row * mat->cols + col) * mat->depth + depth] = value;
}

/* Find maximum absolute value in matrix - important for bit plane encoding */
float matrix_get_max_abs(Matrix *mat) {
 if (!mat) return 0.0f;

 float max_val = 0.0f;
 int total_elements = mat->rows * mat->cols * mat->depth;

 for (int i = 0; i < total_elements; i++) {
 float abs_val = fabsf(mat->data[i]);
 if (abs_val > max_val) {
 max_val = abs_val;
 }
 }

 return max_val;
}

/*
 * Spacecraft-optimized trigonometric operations
 */

/* Optimized cosine function */
float compute_cosine(float angle) {
 // Normalize angle to [0, 2π)
 while (angle < 0) angle += 2.0f * (float)M_PI;
 while (angle >= 2.0f * (float)M_PI) angle -= 2.0f * (float)M_PI;

 // Simple case optimization
 if (fabsf(angle) < FLT_EPSILON) return 1.0f;
 if (fabsf(angle - (float)(M_PI/2.0)) < FLT_EPSILON) return 0.0f;
 if (fabsf(angle - (float)M_PI) < FLT_EPSILON) return -1.0f;
 if (fabsf(angle - (float)(3.0*M_PI/2.0)) < FLT_EPSILON) return 0.0f;

 // Using the standard library function
 return cosf(angle);
}

/* Optimized sine function */
float compute_sine(float angle) {
 // Normalize angle to [0, 2π)
 while (angle < 0) angle += 2.0f * (float)M_PI;
 while (angle >= 2.0f * (float)M_PI) angle -= 2.0f * (float)M_PI;

 // Simple case optimization
 if (fabsf(angle) < FLT_EPSILON) return 0.0f;
 if (fabsf(angle - (float)(M_PI/2.0)) < FLT_EPSILON) return 1.0f;
 if (fabsf(angle - (float)M_PI) < FLT_EPSILON) return 0.0f;
 if (fabsf(angle - (float)(3.0*M_PI/2.0)) < FLT_EPSILON) return -1.0f;

 // Using the standard library function
 return sinf(angle);
}

/* Forward 9/7 Discrete Wavelet Transform - modified to match hardware implementation */
int dwt_forward_97_sw_systolic(float *data, int length, mem_counter_t *counter) {
 if (!data || length <= 1) return -1;

 float *temp = (float *)flight_malloc_impl(length * sizeof(float));
 if (!temp) return -1;

 // Copy data to temp
 for (int i = 0; i < length; i++) {
 counter->reads++;
 counter->writes++;
 temp[i] = data[i];
 }

 // Lifting scheme coefficients for 9/7 wavelet
 const float alpha = -1.586134342f;
 const float beta = -0.05298011854f;
 const float gamma = 0.8829110762f;
 const float delta = 0.4435068522f;
 const float K = 1.149604398f;

 // Predict 1
 for (int i = 1; i < length - 1; i += 2) {
 if (i + 5 < length && (i % 6) == 1) {
 // Handle a 3x3 block (3 odd indices and their neighboring even indices)
 counter->reads += 7; // Reading 7 unique elements

 // Process in 3x3 blocks - simulating hardware matrix operations
 temp[i] += alpha * (temp[i-1] + temp[i+1]);
 temp[i+2] += alpha * (temp[i+1] + temp[i+3]);
 temp[i+4] += alpha * (temp[i+3] + temp[i+5]);

 counter->writes += 3;

 i += 4; // Skip to next block
 } else {
 // Handle individual elements for the remainder
 counter->reads += 2;
 counter->writes++;
 temp[i] += alpha * (temp[i-1] + temp[i+1]);
 }
 }

 // Handle last element if length is even
 if (length % 2 == 0) {
 counter->reads++;
 counter->writes++;
 temp[length-1] += 2 * alpha * temp[length-2];
 }

 // Update 1
 for (int i = 0; i < length - 1; i += 2) {
 if (i + 5 < length && (i % 6) == 0) {
 // Handle a 3x3 block
 if (i + 6 < length) {
 counter->reads += 7; // Reading 7 unique elements
 } else {
 counter->reads += 6; // Reading 6 unique elements
 }

 // Process in 3x3 blocks - simulating hardware matrix operations
 float add_val = temp[i+1];
 if (i > 0) add_val += temp[i-1];
 else add_val += temp[i+1];
 temp[i] += beta * add_val;

 add_val = temp[i+3];
 add_val += temp[i+1];
 temp[i+2] += beta * add_val;

 add_val = temp[i+5];
 add_val += temp[i+3];
 temp[i+4] += beta * add_val;

 counter->writes += 3;

 i += 4; // Skip to next block
 } else {
 // Handle individual elements for the remainder
 float add_val = temp[i+1];
 if (i > 0) {
 counter->reads++;
 add_val += temp[i-1];
 } else {
 counter->reads++;
 add_val += temp[i+1];
 }
 counter->reads++;
 counter->writes++;
 temp[i] += beta * add_val;
 }
 }

 // Handle last even element when length is odd (mirror boundary)
 if (length % 2 == 1) {
 counter->reads++;
 counter->writes++;
 temp[length-1] += 2 * beta * temp[length-2];
 }

 // Predict 2 - implement with similar blocking approach as in HW version
 for (int i = 1; i < length - 1; i += 2) {
 if (i + 5 < length && (i % 6) == 1) {
 // Handle a 3x3 block
 counter->reads += 7; // Reading 7 unique elements

 // Process in 3x3 blocks - simulating hardware matrix operations
 temp[i] += gamma * (temp[i-1] + temp[i+1]);
 temp[i+2] += gamma * (temp[i+1] + temp[i+3]);
 temp[i+4] += gamma * (temp[i+3] + temp[i+5]);

 counter->writes += 3;

 i += 4; // Skip to next block
 } else {
 // Handle individual elements for the remainder
 counter->reads += 2;
 counter->writes++;
 temp[i] += gamma * (temp[i-1] + temp[i+1]);
 }
 }

 if (length % 2 == 0) {
 counter->reads++;
 counter->writes++;
 temp[length-1] += 2 * gamma * temp[length-2];
 }

 // Update 2 - implement with similar blocking approach as in HW version
 for (int i = 0; i < length - 1; i += 2) {
 if (i + 5 < length && (i % 6) == 0) {
 // Handle a 3x3 block
 if (i + 6 < length) {
 counter->reads += 7; // Reading 7 unique elements
 } else {
 counter->reads += 6; // Reading 6 unique elements
 }

 // Process in 3x3 blocks - simulating hardware matrix operations
 float add_val = temp[i+1];
 if (i > 0) add_val += temp[i-1];
 else add_val += temp[i+1];
 temp[i] += delta * add_val;

 add_val = temp[i+3];
 add_val += temp[i+1];
 temp[i+2] += delta * add_val;

 add_val = temp[i+5];
 add_val += temp[i+3];
 temp[i+4] += delta * add_val;

 counter->writes += 3;

 i += 4; // Skip to next block
 } else {
 // Handle individual elements for the remainder
 float add_val = temp[i+1];
 if (i > 0) {
 counter->reads++;
 add_val += temp[i-1];
 } else {
 counter->reads++;
 add_val += temp[i+1];
 }
 counter->reads++;
 counter->writes++;
 temp[i] += delta * add_val;
 }
 }

 // Handle last even element when length is odd (mirror boundary)
 if (length % 2 == 1) {
 counter->reads++;
 counter->writes++;
 temp[length-1] += 2 * delta * temp[length-2];
 }

 // Scale - implement with similar blocking approach
 for (int i = 0; i < length; i += 6) {
 int block_len = MIN(6, length - i);

 counter->reads += block_len;
 counter->writes += block_len;

 for (int j = 0; j < block_len; j++) {
 if ((i + j) % 2 == 0) {
 temp[i + j] *= K;
 } else {
 temp[i + j] /= K;
 }
 }
 }

 // Split into low-pass and high-pass bands
 for (int i = 0; i < length/2; i++) {
 counter->reads += 2;
 counter->writes += 2;
 data[i] = temp[2*i]; // Low-pass
 data[i + length/2] = temp[2*i + 1]; // High-pass
 }

 flight_free_impl(temp);
 return 0;
}

/* 2D Forward Discrete Wavelet Transform - modified to match hardware implementation */
int dwt_2d_forward_sw_systolic(Matrix *data, int levels, mem_counter_t *counter) {
 if (!data || levels <= 0 || data->depth != 1) return -1;

 int width = data->cols;
 int height = data->rows;
 float *temp = (float *)flight_malloc_impl(MAX(width, height) * sizeof(float));
 if (!temp) return -1;

 for (int level = 0; level < levels; level++) {
 // Process rows
 for (int y = 0; y < height; y++) {
 // Copy row to temp
 for (int x = 0; x < width; x++) {
 counter->reads++;
 counter->writes++;
 temp[x] = data->data[y * data->cols + x];
 }

 // Apply 1D DWT to row with systolic hardware
 dwt_forward_97_sw_systolic(temp, width, counter);

 // Copy back to data
 for (int x = 0; x < width; x++) {
 counter->reads++;
 counter->writes++;
 data->data[y * data->cols + x] = temp[x];
 }
 }

 // Process columns
 for (int x = 0; x < width; x++) {
 // Copy column to temp
 for (int y = 0; y < height; y++) {
 counter->reads++;
 counter->writes++;
 temp[y] = data->data[y * data->cols + x];
 }

 // Apply 1D DWT to column with systolic hardware
 dwt_forward_97_sw_systolic(temp, height, counter);

 // Copy back to data
 for (int y = 0; y < height; y++) {
 counter->reads++;
 counter->writes++;
 data->data[y * data->cols + x] = temp[y];
 }
 }

 // Update dimensions for next level
 width /= 2;
 height /= 2;
 }

 flight_free_impl(temp);
 return 0;
}

/*
 * Bit Plane Encoder (BPE) for CCSDS 122.0-B
 */

/* Initialize BPE context */
BPEContext* init_bpe_context(int max_bit_plane, int num_coeffs, int z_param) {
 BPEContext *ctx;

 SAFE_MALLOC(ctx, sizeof(BPEContext));

 ctx->bit_plane = max_bit_plane;
 ctx->max_bit_plane = max_bit_plane;
 ctx->z_param = z_param;

 SAFE_MALLOC(ctx->priorities, num_coeffs * sizeof(int));

 // Initialize priorities (subband-based prioritization)
 for (int i = 0; i < num_coeffs; i++) {
 ctx->priorities[i] = i;
 }

 // Allocate stream buffer with conservative size estimate
 ctx->stream_capacity = num_coeffs * (max_bit_plane + 1) / 4 + 1024;
 SAFE_CALLOC(ctx->coded_stream, ctx->stream_capacity, sizeof(uint8_t));

 ctx->stream_size = 0;
 ctx->buffer = 0;
 ctx->bits_in_buffer = 0;

 return ctx;
}

/* Clean up BPE context */
void free_bpe_context(BPEContext *ctx) {
 if (ctx) {
 flight_free_impl(ctx->priorities);
 flight_free_impl(ctx->coded_stream);
 flight_free_impl(ctx);
 }
}

/* Write a bit to the output stream */
int write_bit(BPEContext *ctx, int bit) {
 if (!ctx) return -1;

 ctx->buffer = (ctx->buffer << 1) | (bit & 1);
 ctx->bits_in_buffer++;

 if (ctx->bits_in_buffer == 8) {
 if (ctx->stream_size >= ctx->stream_capacity) {
 // Increase buffer size if needed
 size_t new_capacity = ctx->stream_capacity * 2;
 uint8_t *new_stream = (uint8_t *)flight_realloc_impl(ctx->coded_stream, new_capacity);
 if (!new_stream) return -1;

 ctx->coded_stream = new_stream;
 ctx->stream_capacity = new_capacity;
 }

 ctx->coded_stream[ctx->stream_size++] = (uint8_t)ctx->buffer;
 ctx->buffer = 0;
 ctx->bits_in_buffer = 0;
 }

 return 0;
}

/* Flush any remaining bits in the buffer */
int flush_bits(BPEContext *ctx) {
 if (!ctx) return -1;

 if (ctx->bits_in_buffer > 0) {
 ctx->buffer <<= (8 - ctx->bits_in_buffer);

 if (ctx->stream_size >= ctx->stream_capacity) {
 // Increase buffer size if needed
 size_t new_capacity = ctx->stream_capacity * 2;
 uint8_t *new_stream = (uint8_t *)flight_realloc_impl(ctx->coded_stream, new_capacity);
 if (!new_stream) return -1;

 ctx->coded_stream = new_stream;
 ctx->stream_capacity = new_capacity;
 }

 ctx->coded_stream[ctx->stream_size++] = (uint8_t)ctx->buffer;
 ctx->buffer = 0;
 ctx->bits_in_buffer = 0;
 }

 return 0;
}

/* Calculate coefficient significance */
int is_significant(float coeff, int bit_plane) {
 float threshold = powf(2.0f, (float)bit_plane);
 return fabsf(coeff) >= threshold;
}

/* Encode a block using the Bit Plane Encoder */
int encode_block(BPEContext *ctx, Matrix *block, int *dc_coeff) {
 if (!ctx || !block) return -1;

 int block_area = block->rows * block->cols;

 // First encode DC coefficient separately
 if (dc_coeff) {
 // DC coefficient coding
 for (int bp = ctx->max_bit_plane; bp >= 0; bp--) {
 int bit = (*dc_coeff >> bp) & 1;
 if (write_bit(ctx, bit) != 0) return -1;
 }
 }

 // Initialize priorities for block coefficients
 // This follows the coefficient scanning pattern in CCSDS 122.0-B

 // Ordering is important for progressive transmission
 for (int i = 0; i < block_area; i++) {
 ctx->priorities[i] = i;
 }

 // Encode AC coefficients bit plane by bit plane
 for (int bp = ctx->max_bit_plane; bp >= 0; bp--) {
 // First pass: significance pass
 for (int i = 0; i < block_area; i++) {
 int idx = ctx->priorities[i];
 float coeff = block->data[idx];

 if (!is_significant(coeff, bp + 1) && is_significant(coeff, bp)) {
 if (write_bit(ctx, 1) != 0) return -1; // Significant
 if (write_bit(ctx, coeff < 0 ? 1 : 0) != 0) return -1; // Sign bit
 } else if (!is_significant(coeff, bp + 1)) {
 if (write_bit(ctx, 0) != 0) return -1; // Not significant yet
 }
 }

 // Second pass: refinement pass
 for (int i = 0; i < block_area; i++) {
 int idx = ctx->priorities[i];
 float coeff = block->data[idx];

 if (is_significant(coeff, bp + 1)) {
 int bit = (int)(fabsf(coeff) / powf(2.0f, (float)bp)) & 1;
 if (write_bit(ctx, bit) != 0) return -1;
 }
 }
 }

 return 0;
}

/* Encode an entire segment */
int encode_segment(BPEContext *ctx, Segment *segment) {
 if (!ctx || !segment) return -1;

 int num_blocks = segment->segment_width * segment->segment_height;

 // Add segment header

 // Write max_bit_plane (5 bits)
 for (int i = 4; i >= 0; i--) {
 if (write_bit(ctx, (segment->max_bit_plane >> i) & 1) != 0) return -1;
 }

 // Write segment dimensions
 for (int i = 5; i >= 0; i--) {
 if (write_bit(ctx, (segment->segment_width >> i) & 1) != 0) return -1;
 }

 for (int i = 5; i >= 0; i--) {
 if (write_bit(ctx, (segment->segment_height >> i) & 1) != 0) return -1;
 }

 // Encode each block
 for (int i = 0; i < num_blocks; i++) {
 if (encode_block(ctx, segment->blocks[i], &segment->dc_coeffs[i]) != 0)
 return -1;
 }

 return 0;
}

/*
 * Segment Processing Functions
 */

/* Allocate and initialize a segment */
Segment* init_segment(int segment_width, int segment_height) {
 Segment *segment;

 if (segment_width <= 0 || segment_height <= 0) return NULL;

 SAFE_MALLOC(segment, sizeof(Segment));

 segment->segment_width = segment_width;
 segment->segment_height = segment_height;

 int num_blocks = segment_width * segment_height;
 SAFE_MALLOC(segment->blocks, num_blocks * sizeof(Matrix *));

 for (int i = 0; i < num_blocks; i++) {
 segment->blocks[i] = matrix_create(BLOCK_SIZE, BLOCK_SIZE, 1);
 if (!segment->blocks[i]) {
 // Clean up on failure
 for (int j = 0; j < i; j++) {
 matrix_free(segment->blocks[j]);
 }
 flight_free_impl(segment->blocks);
 flight_free_impl(segment);
 return NULL;
 }
 }

 SAFE_MALLOC(segment->dc_coeffs, num_blocks * sizeof(int));
 memset(segment->dc_coeffs, 0, num_blocks * sizeof(int));

 int ac_coeffs_count = num_blocks * (BLOCK_SIZE * BLOCK_SIZE - 1);
 SAFE_MALLOC(segment->ac_coeffs, ac_coeffs_count * sizeof(float));
 memset(segment->ac_coeffs, 0, ac_coeffs_count * sizeof(float));

 segment->max_bit_plane = 0;

 return segment;
}

/* Free segment resources */
void free_segment(Segment *segment) {
 if (segment) {
 int num_blocks = segment->segment_width * segment->segment_height;
 for (int i = 0; i < num_blocks; i++) {
 matrix_free(segment->blocks[i]);
 }
 flight_free_impl(segment->blocks);
 flight_free_impl(segment->dc_coeffs);
 flight_free_impl(segment->ac_coeffs);
 flight_free_impl(segment);
 }
}

/* Process a segment of the image - modified to match hardware implementation */
int process_segment(Segment *segment, Image *image, int segment_x, int segment_y,
 int *output_size, uint8_t **output_data, mem_counter_t *counter) {
 if (!segment || !image || !output_size || !output_data) return -1;

 // Copy image data to segment blocks
 for (int by = 0; by < segment->segment_height; by++) {
 for (int bx = 0; bx < segment->segment_width; bx++) {
 Matrix *block = segment->blocks[by * segment->segment_width + bx];

 // Copy block data from image
 for (int y = 0; y < BLOCK_SIZE; y++) {
 for (int x = 0; x < BLOCK_SIZE; x++) {
 int img_x = segment_x + bx * BLOCK_SIZE + x;
 int img_y = segment_y + by * BLOCK_SIZE + y;

 if (img_x < image->width && img_y < image->height) {
 counter->reads++; // Read from image
 counter->writes++; // Write to block
 matrix_set(block, y, x, 0, (float)image->data[img_y * image->width + img_x]);
 } else {
 // Handle border cases with mirroring
 int mirror_x = MIN(image->width - 1, MAX(0, img_x));
 int mirror_y = MIN(image->height - 1, MAX(0, img_y));
 counter->reads++; // Read from image
 counter->writes++; // Write to block
 matrix_set(block, y, x, 0, (float)image->data[mirror_y * image->width + mirror_x]);
 }
 }
 }

 // Apply 2D DWT to block using systolic hardware
 dwt_2d_forward_sw_systolic(block, 3, counter); // 3 levels of DWT

 // Extract DC coefficient
 segment->dc_coeffs[by * segment->segment_width + bx] = (int)matrix_get(block, 0, 0, 0);
 counter->reads++; // Read from block
 counter->writes++; // Write to dc_coeffs

 // Extract AC coefficients
 int ac_idx = (by * segment->segment_width + bx) * (BLOCK_SIZE * BLOCK_SIZE - 1);
 int coeff_idx = 1;
 for (int y = 0; y < BLOCK_SIZE; y++) {
 for (int x = 0; x < BLOCK_SIZE; x++) {
 if (y != 0 || x != 0) { // Skip DC coefficient
 counter->reads++; // Read from block
 counter->writes++; // Write to ac_coeffs
 segment->ac_coeffs[ac_idx + coeff_idx - 1] = matrix_get(block, y, x, 0);
 coeff_idx++;
 }
 }
 }
 }
 }

 // Determine maximum bit plane
 float max_val = 0.0f;
 int num_blocks = segment->segment_width * segment->segment_height;

 for (int i = 0; i < num_blocks; i++) {
 float block_max = matrix_get_max_abs(segment->blocks[i]);
 counter->reads += segment->blocks[i]->rows * segment->blocks[i]->cols; // Read all elements
 if (block_max > max_val) max_val = block_max;
 }

 if (max_val > 0) {
 segment->max_bit_plane = (int)floorf(log2f(max_val));
 } else {
 segment->max_bit_plane = 0;
 }

 // Initialize BPE context with default Z parameter
 BPEContext *ctx = init_bpe_context(segment->max_bit_plane,
 num_blocks * BLOCK_SIZE * BLOCK_SIZE,
 MAX_Z_SIZE / 2);
 if (!ctx) return -1;

 // Encode segment
 if (encode_segment(ctx, segment) != 0) {
 free_bpe_context(ctx);
 return -1;
 }

 // Flush remaining bits
 if (flush_bits(ctx) != 0) {
 free_bpe_context(ctx);
 return -1;
 }

 // Set output size and data
 *output_size = ctx->stream_size;
 *output_data = (uint8_t *)flight_malloc_impl(ctx->stream_size);
 if (!*output_data) {
 FLIGHT_LOG("Memory allocation failed\n");
 free_bpe_context(ctx);
 return -1;
 }

 // Copy data from BPE context stream to output
 memcpy(*output_data, ctx->coded_stream, ctx->stream_size);

 // Clean up
 free_bpe_context(ctx);

 return 0;
}

/*
 * Main CCSDS 122.0-B compression function - modified to match hardware implementation
 */
int compress_ccsds122(Image *image, uint8_t **output_data, int *output_size, int z_param, mem_counter_t *counter) {
 if (!image || !output_data || !output_size) return -1;

 // Calculate segment dimensions
 int segment_width = (image->width + BLOCK_SIZE - 1) / BLOCK_SIZE;
 int segment_height = (image->height + BLOCK_SIZE - 1) / BLOCK_SIZE;

 // Limit segment size to MAX_SEGMENT_ROWS
 segment_width = MIN(segment_width, MAX_BLOCKS_PER_SEGMENT);
 segment_height = MIN(segment_height, MAX_BLOCKS_PER_SEGMENT);

 // Allocate temporary buffer for compressed data
 size_t max_compressed_size = image->width * image->height * sizeof(uint16_t) * 2;
 uint8_t *compressed_data = (uint8_t *)flight_malloc_impl(max_compressed_size);
 if (!compressed_data) {
 FLIGHT_LOG("Memory allocation failed\n");
 return -1;
 }

 // Write compressed file header
 int header_size = 24; // Bytes

 // Write image dimensions
 compressed_data[0] = (image->width >> 8) & 0xFF;
 compressed_data[1] = image->width & 0xFF;
 compressed_data[2] = (image->height >> 8) & 0xFF;
 compressed_data[3] = image->height & 0xFF;

 // Write bit depth
 compressed_data[4] = image->bit_depth & 0xFF;

 // Write signed flag
 compressed_data[5] = image->signed_pixels & 0xFF;

 // Write segment dimensions
 compressed_data[6] = segment_width & 0xFF;
 compressed_data[7] = segment_height & 0xFF;

 // Write Z parameter
 compressed_data[8] = z_param & 0xFF;

 // Reserved bytes
 for (int i = 9; i < header_size; i++) {
 compressed_data[i] = 0;
 }

 size_t compressed_offset = header_size;

 // Process each segment
 for (int seg_y = 0; seg_y < image->height; seg_y += segment_height * BLOCK_SIZE) {
 for (int seg_x = 0; seg_x < image->width; seg_x += segment_width * BLOCK_SIZE) {
 // Initialize segment
 Segment *segment = init_segment(
 MIN(segment_width, (image->width - seg_x + BLOCK_SIZE - 1) / BLOCK_SIZE),
 MIN(segment_height, (image->height - seg_y + BLOCK_SIZE - 1) / BLOCK_SIZE));

 if (!segment) {
 flight_free_impl(compressed_data);
 return -1;
 }

 // Process segment with hardware acceleration simulation
 uint8_t *segment_data;
 int segment_size;
 if (process_segment(segment, image, seg_x, seg_y, &segment_size, &segment_data, counter) != 0) {
 free_segment(segment);
 flight_free_impl(compressed_data);
 return -1;
 }

 // Write segment header
 compressed_data[compressed_offset++] = (segment_size >> 8) & 0xFF;
 compressed_data[compressed_offset++] = segment_size & 0xFF;

 // Copy segment data to compressed buffer
 if (compressed_offset + segment_size <= max_compressed_size) {
 memcpy(compressed_data + compressed_offset, segment_data, segment_size);
 compressed_offset += segment_size;
 } else {
 flight_free_impl(segment_data);
 free_segment(segment);
 flight_free_impl(compressed_data);
 return -1;
 }

 // Clean up
 flight_free_impl(segment_data);
 free_segment(segment);
 }
 }

 // Set output size and data
 *output_size = compressed_offset;
 *output_data = (uint8_t *)flight_malloc_impl(compressed_offset);
 if (!*output_data) {
 FLIGHT_LOG("Memory allocation failed\n");
 flight_free_impl(compressed_data);
 return -1;
 }

 // Copy data from compressed_data to output
 memcpy(*output_data, compressed_data, compressed_offset);

 // Clean up
 flight_free_impl(compressed_data);

 return 0;
}

/*
 * Utility Functions
 */

/* Load image from raw file */
Image* load_raw_image(const char *filename, int width, int height, int bit_depth) {
 FILE *fp = FLIGHT_FOPEN(filename, "rb");
 if (!fp) {
 FLIGHT_LOG("Could not open file: %s\n", filename);
 return NULL;
 }

 Image *img;
 SAFE_MALLOC(img, sizeof(Image));

 img->width = width;
 img->height = height;
 img->bit_depth = bit_depth;
 img->signed_pixels = 0; // Default to unsigned

 size_t num_pixels = width * height;
 SAFE_MALLOC(img->data, num_pixels * sizeof(uint16_t));

 size_t bytes_per_pixel = (bit_depth + 7) / 8;
 uint8_t *buffer;
 SAFE_MALLOC(buffer, num_pixels * bytes_per_pixel);

 size_t bytes_read = FLIGHT_FREAD(buffer, 1, num_pixels * bytes_per_pixel, fp);
 FLIGHT_FCLOSE(fp);

 if (bytes_read != num_pixels * bytes_per_pixel) {
 FLIGHT_LOG("Error reading file: %s\n", filename);
 flight_free_impl(buffer);
 flight_free_impl(img->data);
 flight_free_impl(img);
 return NULL;
 }

 // Convert bytes to uint16_t
 if (bytes_per_pixel == 1) {
 for (size_t i = 0; i < num_pixels; i++) {
 img->data[i] = buffer[i];
 }
 } else { // bytes_per_pixel == 2
 for (size_t i = 0; i < num_pixels; i++) {
 // Adjust byte order based on endianness requirements
 img->data[i] = (buffer[i*2] << 8) | buffer[i*2+1]; // Big-endian
 }
 }

 flight_free_impl(buffer);
 return img;
}

/* Save compressed data to file */
int save_compressed(const char *filename, uint8_t *data, int size) {
 FILE *fp = FLIGHT_FOPEN(filename, "wb");
 if (!fp) {
 FLIGHT_LOG("Could not open file for writing: %s\n", filename);
 return -1;
 }

 size_t bytes_written = FLIGHT_FWRITE(data, 1, size, fp);
 FLIGHT_FCLOSE(fp);

 if (bytes_written != (size_t)size) {
 FLIGHT_LOG("Error writing to file: %s\n", filename);
 return -1;
 }

 return 0;
}

/* Free image resources */
void free_image(Image *img) {
 if (img) {
 flight_free_impl(img->data);
 flight_free_impl(img);
 }
}

/* Compression pipeline with timing instrumentation */
void compute_subband_dft(float *coeffs, int len, float *magnitude);
void execute_compression(const char *input_file, int width, int height, int bit_depth,
 const char *output_file, int z_param) {
 FLIGHT_LOG("[CCSDS] init\n");
 FLIGHT_LOG("Input: %s (%dx%d, %d bits)\n", input_file, width, height, bit_depth);

 // Load image
 Image *img = load_raw_image(input_file, width, height, bit_depth);
 if (!img) {
 FLIGHT_LOG("Failed to load image\n");
 return;
 }

 // Memory operation counters
 mem_counter_t counter_sw = {0};

 // Variables for compressed output
 uint8_t *compressed_data = NULL;
 int compressed_size = 0;

 // Measure execution time using clock()
 clock_t start_time, end_time;

 // Compress image with simulated hardware acceleration
 FLIGHT_LOG("\n[CCSDS] compress (systolic)\n");
 start_time = clock();

 int ret = compress_ccsds122(img, &compressed_data, &compressed_size, z_param, &counter_sw);

 end_time = clock();
 double sw_time = measure_time_diff(start_time, end_time);

 if (ret != 0 || !compressed_data) {
 FLIGHT_LOG("Compression failed\n");
 free_image(img);
 return;
 }

 // Perform DFT quality analysis on compressed coefficients
 {
 float dft_in[FFT_LEN], dft_mag[FFT_LEN];
 int seg_len = (compressed_size < FFT_LEN) ? compressed_size : FFT_LEN;
 for (int i = 0; i < seg_len; i++)
 dft_in[i] = (float)compressed_data[i];
 for (int i = seg_len; i < FFT_LEN; i++)
 dft_in[i] = 0.0f;
 compute_subband_dft(dft_in, FFT_LEN, dft_mag);
 FLIGHT_LOG("DFT quality analysis: peak magnitude = %.2f\n", dft_mag[0]);
 }

 // Save compressed data
 save_compressed(output_file, compressed_data, compressed_size);

 // Calculate compression ratio
 float ratio = (float)compressed_size / (width * height * ((bit_depth + 7) / 8));

 // Print compression results
 FLIGHT_LOG("\n[CCSDS] results\n");
 FLIGHT_LOG(" Input: %d x %d pixels, %d-bit = %d bytes\n",
 width, height, bit_depth, width * height * ((bit_depth + 7) / 8));
 FLIGHT_LOG(" Output: %d bytes\n", compressed_size);
 FLIGHT_LOG(" Ratio: %.4f (%.2f%%)\n", ratio, ratio * 100.0f);
 FLIGHT_LOG(" Time: %.4f seconds\n", sw_time);

 // Print memory operation statistics
 FLIGHT_LOG("\n[CCSDS] mem_ops\n");
 FLIGHT_LOG(" Software Implementation (systolic simulation):\n");
 FLIGHT_LOG(" Reads: %lld\n", counter_sw.reads);
 FLIGHT_LOG(" Writes: %lld\n", counter_sw.writes);
 FLIGHT_LOG(" Total: %lld\n", counter_sw.reads + counter_sw.writes);

 // Clean up
 flight_free_impl(compressed_data);
 free_image(img);

 FLIGHT_LOG("\n[CCSDS] done: %s\n", output_file);
}

/* AME-accelerated matrix-vector multiply: y[m] = A[m×n] × x[n]
 * Uses 3×3 tile operations via SYS_MMACD (md += ms1 × ms2^T). */
static void ame_matvec(const double *A, const double *x, double *y, int m, int n) {
 const int P = 3;
 long stride = P * (long)sizeof(double); /* 24 bytes */

 for (int i = 0; i < m; i += P) {
 double c_buf[P * P];

 /* Zero accumulator register M2 */
 asm volatile(MZERO(M2));

 for (int kk = 0; kk < n; kk += P) {
 double a_buf[P * P], b_buf[P * P];

 /* Load A tile [i..i+2][kk..kk+2] with boundary padding */
 for (int r = 0; r < P; r++)
 for (int c = 0; c < P; c++)
 a_buf[r * P + c] = (i + r < m && kk + c < n) ?
 A[(i + r) * n + kk + c] : 0.0;

 /* Load x column tile: x[kk..kk+2] in col 0, rest zero.
 * SYS_MMACD does md += ms1 × ms2^T, so ms2^T row 0 = [x0,x1,x2]
 * giving column 0 of result = A_tile × x_chunk. */
 for (int r = 0; r < P; r++)
 for (int c = 0; c < P; c++)
 b_buf[r * P + c] = (c == 0 && kk + r < n) ? x[kk + r] : 0.0;

 /* Load A tile into M0 */
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 :: "r"(stride), "r"(a_buf) : "t0", "t1", "memory");

 /* Load B tile into M1 */
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M1, T1, T0)
 :: "r"(stride), "r"(b_buf) : "t0", "t1", "memory");

 /* M2 += M0 × M1^T */
 asm volatile(SYS_MMACD(M2, M0, M1));
 }

 /* Store result tile from M2 */
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(stride), "r"(c_buf) : "t0", "t1", "memory");

 /* Extract column 0 of result tile → y[i..i+2] */
 for (int r = 0; r < P; r++)
 if (i + r < m) y[i + r] = c_buf[r * P];
 }
}

/* DFT frequency analysis on DWT subbands — AME-accelerated matrix-vector DFT */
void compute_subband_dft(float *coeffs, int len, float *magnitude) {
 int n = (len < FFT_LEN) ? len : FFT_LEN;

 /* Build DFT cosine and sine matrices (double precision for AME) */
 static double DFT_cos[FFT_LEN * FFT_LEN];
 static double DFT_sin[FFT_LEN * FFT_LEN];
 for (int k = 0; k < n; k++) {
 for (int j = 0; j < n; j++) {
 double angle = 2.0 * M_PI * k * j / (double)n;
 DFT_cos[k * n + j] = cos(angle);
 DFT_sin[k * n + j] = -sin(angle);
 }
 }

 /* Convert input to double */
 double xd[FFT_LEN], re_d[FFT_LEN], im_d[FFT_LEN];
 for (int j = 0; j < n; j++) xd[j] = (double)coeffs[j];
 for (int j = n; j < FFT_LEN; j++) xd[j] = 0.0;

 /* AME-accelerated matrix-vector products */
 ame_matvec(DFT_cos, xd, re_d, n, n);
 ame_matvec(DFT_sin, xd, im_d, n, n);

 /* Compute magnitude (back to float) */
 for (int k = 0; k < n; k++)
 magnitude[k] = (float)sqrt(re_d[k] * re_d[k] + im_d[k] * im_d[k]);
}

/* ------------------------------------------------------------------ */
/* BCE: Orbital Mechanics (12%) — Imaging geometry & downlink window */
/* ------------------------------------------------------------------ */

/* Physical constants */
#define MU_EARTH_OM 3.986004418e14 /* Earth GM (m^3/s^2) */
#define R_EARTH_OM 6378137.0 /* Earth equatorial radius (m) */
#define J2_COEFF_OM 1.08263e-3 /* J2 oblateness */
#define OMEGA_EARTH 7.2921159e-5 /* Earth rotation rate (rad/s) */
#define C_LIGHT_OM 2.99792458e8 /* Speed of light (m/s) */
#define K_BOLTZMANN_OM 1.380649e-23 /* Boltzmann constant (J/K) */

/* Orbit parameters for a typical sun-synchronous EO satellite (e.g., high-resolution imaging satellite) */
#define EO_ALTITUDE 509.0e3 /* Orbit altitude (m) */
#define EO_INCLINATION 97.5 /* Sun-sync inclination (deg) */
#define DOWNLINK_FREQ 8.125e9 /* X-band downlink frequency (Hz) */
#define DOWNLINK_BW 320.0e6 /* Downlink bandwidth (Hz) */
#define TX_POWER_DBW 7.0 /* Transmitter power (dBW) */
#define TX_GAIN_DBI 6.0 /* Antenna gain (dBi) */

typedef struct {
 double acq_start_s; /* Time when target enters FoV (s from epoch) */
 double acq_end_s; /* Time when target exits FoV */
 double downlink_start; /* Start of ground station visibility window */
 double downlink_end; /* End of ground station visibility window */
 double gsd_m; /* Ground sampling distance at nadir (m) */
 double incidence_deg; /* Off-nadir incidence angle (deg) */
 double data_rate_Mbps; /* Achievable downlink data rate (Mbps) */
 double fspl_dB; /* Free-space path loss (dB) */
} ImagingGeometry;

/**
 * Circular orbit propagation with J2 secular RAAN drift.
 * Returns ECI (x,y,z) in metres at time t_sec from epoch.
 */
static void orbmech_propagate(double a, double inc_rad,
 double raan0, double arg_lat0,
 double t_sec,
 double pos[3], double vel[3])
{
 double n = sqrt(MU_EARTH_OM / (a * a * a));
 double re_a2 = (R_EARTH_OM / a) * (R_EARTH_OM / a);
 double raan_dot = -1.5 * n * J2_COEFF_OM * re_a2 * cos(inc_rad);
 double raan = raan0 + raan_dot * t_sec;
 double u = arg_lat0 + n * t_sec;

 double x_op = a * cos(u);
 double y_op = a * sin(u);
 double cO = cos(raan), sO = sin(raan);
 double ci = cos(inc_rad), si = sin(inc_rad);

 pos[0] = cO * x_op - sO * ci * y_op;
 pos[1] = sO * x_op + cO * ci * y_op;
 pos[2] = si * y_op;

 double dx = -a * sin(u) * n;
 double dy = a * cos(u) * n;
 vel[0] = cO * dx - sO * ci * dy;
 vel[1] = sO * dx + cO * ci * dy;
 vel[2] = si * dy;
}

/**
 * Compute slant range from satellite position to a ground point.
 */
static double orbmech_slant_range(const double sat_pos[3],
 double lat_rad, double lon_rad)
{
 double gs[3];
 gs[0] = R_EARTH_OM * cos(lat_rad) * cos(lon_rad);
 gs[1] = R_EARTH_OM * cos(lat_rad) * sin(lon_rad);
 gs[2] = R_EARTH_OM * sin(lat_rad);
 double dx = sat_pos[0] - gs[0];
 double dy = sat_pos[1] - gs[1];
 double dz = sat_pos[2] - gs[2];
 return sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Free-space path loss (dB).
 */
static double orbmech_fspl(double range_m, double freq_Hz)
{
 return 20.0 * log10(range_m) + 20.0 * log10(freq_Hz)
 + 20.0 * log10(4.0 * M_PI / C_LIGHT_OM);
}

/**
 * Compute imaging geometry for a given satellite sub-point and target.
 * - GSD = altitude * pixel_ifov (simplified: altitude / focal_length * pixel_pitch)
 * - Incidence angle from nadir
 * - Acquisition window based on half-cone FoV
 */
static void compute_imaging_geometry(double alt_m, double nadir_angle_deg,
 double focal_m, double pixel_um,
 ImagingGeometry *geom)
{
 /* Ground Sampling Distance at nadir */
 double pixel_m = pixel_um * 1.0e-6;
 geom->gsd_m = alt_m * pixel_m / focal_m;

 /* Off-nadir: GSD degrades by 1/cos^3(theta) across-track */
 double theta_rad = nadir_angle_deg * M_PI / 180.0;
 double gsd_off = geom->gsd_m / (cos(theta_rad) * cos(theta_rad) * cos(theta_rad));
 geom->incidence_deg = nadir_angle_deg;

 /* Swath half-angle ≈ atan(swath/2 / altitude) */
 double fov_half_rad = atan(12000.0 / (2.0 * alt_m)); /* ~12 km swath */

 /* Acquisition window duration ≈ swath / ground-track velocity */
 double v_ground = sqrt(MU_EARTH_OM / (R_EARTH_OM + alt_m)) *
 R_EARTH_OM / (R_EARTH_OM + alt_m); /* ground-track speed */
 double swath_along = 2.0 * alt_m * tan(fov_half_rad);
 double acq_duration = swath_along / v_ground;
 geom->acq_start_s = 0.0;
 geom->acq_end_s = acq_duration;

 /* Overwrite GSD with off-nadir value for reporting */
 geom->gsd_m = gsd_off;
}

/**
 * Compute downlink window and achievable data rate for a ground station pass.
 * Uses link-budget to determine SNR and Shannon capacity.
 */
static void compute_downlink_window(const double sat_pos[3],
 double gs_lat_rad, double gs_lon_rad,
 double noise_temp_K,
 ImagingGeometry *geom)
{
 double range = orbmech_slant_range(sat_pos, gs_lat_rad, gs_lon_rad);
 double fspl = orbmech_fspl(range, DOWNLINK_FREQ);
 geom->fspl_dB = fspl;

 /* Elevation angle to satellite */
 double gs[3];
 gs[0] = R_EARTH_OM * cos(gs_lat_rad) * cos(gs_lon_rad);
 gs[1] = R_EARTH_OM * cos(gs_lat_rad) * sin(gs_lon_rad);
 gs[2] = R_EARTH_OM * sin(gs_lat_rad);
 double dx = sat_pos[0] - gs[0], dy = sat_pos[1] - gs[1], dz = sat_pos[2] - gs[2];
 /* Unit vector to satellite in local horizontal */
 double up[3] = { cos(gs_lat_rad) * cos(gs_lon_rad),
 cos(gs_lat_rad) * sin(gs_lon_rad),
 sin(gs_lat_rad) };
 double cos_elev = (dx * up[0] + dy * up[1] + dz * up[2]) / range;
 double elev = asin(cos_elev > 1.0 ? 1.0 : (cos_elev < -1.0 ? -1.0 : cos_elev));

 /* Visibility: elevation > 5 deg */
 int visible = (elev > 5.0 * M_PI / 180.0) ? 1 : 0;

 if (visible) {
 /* EIRP */
 double eirp = TX_POWER_DBW + TX_GAIN_DBI;
 /* Received power (dBW) */
 double rx_power = eirp - fspl + 34.0; /* 34 dBi ground antenna */
 /* Noise floor */
 double N0 = 10.0 * log10(K_BOLTZMANN_OM * noise_temp_K);
 double noise = N0 + 10.0 * log10(DOWNLINK_BW);
 double snr_dB = rx_power - noise;
 /* Shannon capacity: C = BW * log2(1 + SNR_linear) */
 double snr_lin = pow(10.0, snr_dB / 10.0);
 geom->data_rate_Mbps = DOWNLINK_BW * log2(1.0 + snr_lin) / 1.0e6;
 } else {
 geom->data_rate_Mbps = 0.0;
 }

 /* Typical LEO pass window ≈ 10 min for near-overhead pass */
 geom->downlink_start = 0.0;
 geom->downlink_end = visible ? 600.0 : 0.0;
}

/**
 * Compute orbital mechanics: propagate orbit, compute imaging &
 * downlink geometry over multiple time steps.
 */
void run_imaging_geometry_analysis(void) {
 FLIGHT_LOG("\n[CCSDS] orbit_geometry\n");
 clock_t t0 = clock();

 double a = R_EARTH_OM + EO_ALTITUDE;
 double inc_rad = EO_INCLINATION * M_PI / 180.0;
 double raan0 = 22.5 * M_PI / 180.0; /* Ascending node */
 double arg0 = 0.0;

 /* Shadnagar ground station (Hyderabad, NRSC) */
 double gs_lat = 17.04 * M_PI / 180.0;
 double gs_lon = 78.2 * M_PI / 180.0;

 /* high-resolution optics approximation */
 double focal_len = 5.2; /* metres */
 double pixel_pitch = 7.0; /* micrometres */

 double period = 2.0 * M_PI * sqrt(a * a * a / MU_EARTH_OM);
 int num_steps = FFT_LEN; /* Sample one orbit in FFT_LEN steps */
 double dt = period / num_steps;

 double total_gsd = 0.0, total_rate = 0.0;
 int windows_found = 0;

 for (int step = 0; step < num_steps; step++) {
 double t_sec = step * dt;
 double pos[3], vel[3];
 orbmech_propagate(a, inc_rad, raan0, arg0, t_sec, pos, vel);

 /* Altitude above spherical Earth */
 double r = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
 double alt = r - R_EARTH_OM;

 /* Imaging geometry for a +15° off-nadir look angle */
 ImagingGeometry geom;
 compute_imaging_geometry(alt, 15.0, focal_len, pixel_pitch, &geom);
 total_gsd += geom.gsd_m;

 /* Downlink window & data rate */
 compute_downlink_window(pos, gs_lat, gs_lon, 150.0, &geom);
 if (geom.data_rate_Mbps > 0.0) {
 total_rate += geom.data_rate_Mbps;
 windows_found++;
 }
 }

 clock_t t1 = clock();
 double elapsed = measure_time_diff(t0, t1);

 FLIGHT_LOG(" Orbit period : %.1f min\n", period / 60.0);
 FLIGHT_LOG(" Avg GSD (@15° off) : %.3f m\n", total_gsd / num_steps);
 FLIGHT_LOG(" Downlink windows : %d / %d steps\n", windows_found, num_steps);
 if (windows_found > 0)
 FLIGHT_LOG(" Avg achievable rate : %.1f Mbps\n", total_rate / windows_found);
 FLIGHT_LOG(" OrbMech time : %.4f s\n", elapsed);
}

/* ===================================================================
 * Reed-Solomon ECC Module (BCE: ECC 9%)
 * Protects compressed CCSDS 122.0 packets for downlink.
 * RS(255,237) over GF(2^8), 18 parity symbols per block.
 * =================================================================== */
#define GF_ORDER_C 8
#define GF_SIZE_C (1 << GF_ORDER_C)
#define ECC_NSYM_C 18 /* RS(255,237): 18 parity symbols, t=9 correction */

static unsigned char gf_exp_c[GF_SIZE_C];
static unsigned char gf_log_c[GF_SIZE_C];
static int gf_init_done_c = 0;

static void gf_init_c(void) {
 if (gf_init_done_c) return;
 unsigned int x = 1;
 for (int i = 0; i < GF_SIZE_C - 1; i++) {
 gf_exp_c[i] = (unsigned char)x;
 gf_log_c[x] = (unsigned char)i;
 x <<= 1;
 if (x & GF_SIZE_C) x ^= 0x11D; /* primitive polynomial */
 }
 gf_exp_c[GF_SIZE_C - 1] = 0;
 gf_log_c[0] = 0;
 gf_init_done_c = 1;
}

static unsigned char gf_mul_c(unsigned char a, unsigned char b) {
 if (a == 0 || b == 0) return 0;
 int s = (int)gf_log_c[a] + (int)gf_log_c[b];
 if (s >= GF_SIZE_C - 1) s -= (GF_SIZE_C - 1);
 return gf_exp_c[s];
}

static void rs_encode_c(const unsigned char *data, int data_len, unsigned char *parity)
{
 unsigned char gen[ECC_NSYM_C + 1];
 memset(gen, 0, sizeof(gen));
 gen[0] = 1;
 for (int i = 0; i < ECC_NSYM_C; i++) {
 for (int j = i + 1; j >= 1; j--)
 gen[j] = gen[j - 1] ^ gf_mul_c(gen[j], gf_exp_c[i]);
 gen[0] = gf_mul_c(gen[0], gf_exp_c[i]);
 }
 memset(parity, 0, ECC_NSYM_C);
 for (int i = 0; i < data_len; i++) {
 unsigned char fb = data[i] ^ parity[0];
 for (int j = 0; j < ECC_NSYM_C - 1; j++)
 parity[j] = parity[j + 1] ^ gf_mul_c(fb, gen[ECC_NSYM_C - 1 - j]);
 parity[ECC_NSYM_C - 1] = gf_mul_c(fb, gen[0]);
 }
}

static int rs_syndrome_c(const unsigned char *data, int data_len,
 const unsigned char *parity)
{
 int has_error = 0;
 for (int i = 0; i < ECC_NSYM_C; i++) {
 unsigned char s = 0;
 unsigned char ai = gf_exp_c[i], pw = 1;
 for (int j = 0; j < data_len; j++) {
 s ^= gf_mul_c(data[j], pw);
 pw = gf_mul_c(pw, ai);
 }
 for (int j = 0; j < ECC_NSYM_C; j++) {
 s ^= gf_mul_c(parity[j], pw);
 pw = gf_mul_c(pw, ai);
 }
 if (s) has_error = 1;
 }
 return has_error;
}

void apply_ecc_integrity_check(void) {
 FLIGHT_LOG("\n[CCSDS] RS-ECC\n");
 clock_t t0 = clock();
 gf_init_c();

 /* Simulate compressed packet stream (multiple segments) */
 int num_packets = 48; /* typical strip of 48 segments */
 int pkt_len = 237; /* RS(255,237) data portion */
 int total_blocks = 0;
 unsigned char pkt_data[237];
 unsigned char parity[ECC_NSYM_C];

 for (int p = 0; p < num_packets; p++) {
 /* Fill pseudo-random packet data */
 for (int i = 0; i < pkt_len; i++)
 pkt_data[i] = (unsigned char)((p * 131 + i * 37) & 0xFF);
 rs_encode_c(pkt_data, pkt_len, parity);
 /* Verify syndrome == 0 (no errors injected) */
 int err = rs_syndrome_c(pkt_data, pkt_len, parity);
 if (err) {
 FLIGHT_LOG("RS syndrome check FAILED for packet %d\n", p);
 }
 total_blocks++;
 }

 clock_t t1 = clock();
 double elapsed = measure_time_diff(t0, t1);
 FLIGHT_LOG(" RS(255,237) encoded %d packets, %d parity bytes/pkt\n",
 total_blocks, ECC_NSYM_C);
 FLIGHT_LOG(" ECC time : %.4f s\n", elapsed);
}

/**
 * Generate a synthetic 8-bit pushbroom Earth-observation image for self-test.
 * Simulates a nadir-looking imager with along-track gradient and cross-track
 * features typical of IRS/multispectral imagery.
 */
static int generate_synthetic_image(const char *path, int width, int height,
 int bit_depth) {
 FILE *fp = FLIGHT_FOPEN(path, "wb");
 if (!fp) return -1;
 unsigned int seed = 4093; /* deterministic PRNG for reproducibility */
 int bytes_per_pixel = (bit_depth <= 8) ? 1 : 2;
 int max_val = (1 << bit_depth) - 1;
 for (int r = 0; r < height; r++) {
 for (int c = 0; c < width; c++) {
 /* Along-track gradient + cross-track sinusoidal feature */
 double base = (double)r / height * max_val * 0.6;
 double feature = sin(c * 3.14159 / width * 4.0) * max_val * 0.15;
 /* Pseudo-random sensor noise */
 seed = seed * 1103515245u + 12345u;
 double noise = ((int)(seed >> 16) % 64 - 32) * (max_val / 255.0);
 int val = (int)(base + feature + noise);
 if (val < 0) val = 0;
 if (val > max_val) val = max_val;
 if (bytes_per_pixel == 1) {
 uint8_t b = (uint8_t)val;
 FLIGHT_FWRITE(&b, 1, 1, fp);
 } else {
 uint8_t buf[2] = {(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
 FLIGHT_FWRITE(buf, 1, 2, fp);
 }
 }
 }
 FLIGHT_FCLOSE(fp);
 return 0;
}

/* Main function */
int main(int argc, char **argv) {
 const char *input_file;
 int width, height, bit_depth;
 const char *output_file = "output.ccsds";
 int z_param = 8;
 int self_test = 0;

 if (argc < 5) {
 /* Self-test mode: generate synthetic 64x64 8-bit image */
 FLIGHT_LOG("[CCSDS] self-test\n");
 width = 64;
 height = 64;
 bit_depth = 8;
 input_file = "/tmp/ccsds122_selftest.raw";
 output_file = "/tmp/ccsds122_selftest.ccsds";
 if (generate_synthetic_image(input_file, width, height, bit_depth) != 0) {
 FLIGHT_LOG("Failed to generate synthetic test image\n");
 return 1;
 }
 FLIGHT_LOG("Generated synthetic %dx%d %d-bit image\n", width, height, bit_depth);
 self_test = 1;
 } else {
 input_file = argv[1];
 width = atoi(argv[2]);
 height = atoi(argv[3]);
 bit_depth = atoi(argv[4]);

 if (width <= 0 || height <= 0 || bit_depth < 8 || bit_depth > 16) {
 FLIGHT_LOG("Invalid parameters\n");
 return 1;
 }

 if (argc > 5) output_file = argv[5];
 if (argc > 6) {
 z_param = atoi(argv[6]);
 if (z_param < 0 || z_param > MAX_Z_SIZE) {
 FLIGHT_LOG("Invalid Z parameter, using default: 8\n");
 z_param = 8;
 }
 }
 }

 // Execute compression
 execute_compression(input_file, width, height, bit_depth, output_file, z_param);

 // Compute orbital mechanics imaging geometry
 run_imaging_geometry_analysis();

 // Apply ECC integrity verification (BCE: ECC 9%)
 apply_ecc_integrity_check();

 /* Clean up self-test temp files */
 if (self_test) {
 (void)remove("/tmp/ccsds122_selftest.raw");
 (void)remove("/tmp/ccsds122_selftest.ccsds");
 }

 return 0;
}
