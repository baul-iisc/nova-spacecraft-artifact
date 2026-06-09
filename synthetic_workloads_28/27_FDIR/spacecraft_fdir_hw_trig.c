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
 * Fault Detection, Isolation and Recovery Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the spacecraft FDIR (Fault Detection, Isolation and Recovery) system
 * using neural-network-based fault classification and EKF residual monitoring.
 * This workload performs multi-layer NN inference (9->12->6 with ReLU/softmax)
 * for fault type identification, BLAS-level GEMM/GEMV operations, and chi-square
 * statistical testing. A mandatory subsystem on all mission-critical
 * spacecraft, with enhanced requirements for crew abort decision logic.
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
/* Redirect trig to CORDIC hardware */
#define sin(x) hw_sin(x)
#define cos(x) hw_cos(x)
#define atan2(y,x) hw_atan2(y,x)
#define asin(x) hw_asin(x)

// Constants for the simulation
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)

// Matrix dimensions for FDIR state estimation
#define STATE_DIM 15 // State vector: position[3], velocity[3], quaternion[4], gyro_bias[3], accel_bias[2]
#define MEASURE_DIM 9 // Measurement vector: gyro[3], magnetometer[3], sun_sensor[3]
#define NUM_SENSORS 3 // Number of sensors (including redundant ones)
#define MATRIX_TILE_SIZE 3 // Optimized for RISC-V matrix extension

/* Static memory pool configuration (MISRA 21.3: no heap after init).
 * All Matrix objects are allocated from a fixed BSS pool, eliminating
 * heap exhaustion risk and non-deterministic malloc/free timing that
 * would otherwise occur every 0.1 s inside kalman_filter_update and
 * matrix_inverse on the flight-critical FDIR path. */
#define MAX_MAT_ELEMS (STATE_DIM * STATE_DIM) /* 225 doubles — largest matrix */
#define MAT_POOL_SIZE 32 /* 12 persistent + 17 peak temporaries + margin */

// Fault types
typedef enum {
 NO_FAULT = 0,
 BIAS_FAULT,
 SCALE_FACTOR_FAULT,
 STUCK_FAULT,
 NOISE_FAULT,
 TOTAL_FAILURE_FAULT
} FaultType;

// Define the quaternion structure
typedef struct {
 double q0; // Scalar part
 double q1; // Vector part x
 double q2; // Vector part y
 double q3; // Vector part z
} Quaternion;

// Structure for Direction Cosine Matrix (DCM)
typedef struct {
 double mat[3][3];
} DCM;

// Structure for spacecraft system state
typedef struct {
 double position[3]; // Position in Earth-Centered Inertial (ECI) frame
 double velocity[3]; // Velocity in ECI frame
 Quaternion attitude; // Attitude quaternion
 double angular_velocity[3]; // Angular velocity
 double gyro_bias[3]; // Gyroscope bias
 double accel_bias[2]; // Accelerometer bias
} SpacecraftState;

// Structure for sensor measurements
typedef struct {
 double gyro[3]; // Gyroscope measurements
 double magnetometer[3]; // Magnetometer measurements
 double sun_sensor[3]; // Sun sensor measurements
 double star_tracker[4]; // Star tracker quaternion
 double earth_sensor[2]; // Earth sensor measurements (pitch and roll)
 double timestamp; // Timestamp of measurement
} SensorMeasurements;

// Structure for a general matrix
typedef struct {
 int rows;
 int cols;
 double data[MAX_MAT_ELEMS]; // Row-major order (inline BSS — no heap)
} Matrix;

// Fault detector structure - changed to use pointers for Matrix fields
typedef struct {
 Matrix* residual; // Measurement residual
 Matrix* innovation_covariance; // Innovation covariance
 double threshold; // Detection threshold
 bool fault_detected; // Fault detection flag
 FaultType detected_fault; // Type of detected fault
 int faulty_sensor; // Index of faulty sensor
} FaultDetector;

// Kalman filter structure - changed to use pointers for Matrix fields
typedef struct {
 Matrix* state; // State estimate
 Matrix* covariance; // State covariance
 Matrix* process_noise; // Process noise covariance
 Matrix* measure_noise; // Measurement noise covariance
 Matrix* transition; // State transition matrix
 Matrix* measurement; // Measurement matrix
 Matrix* kalman_gain; // Kalman gain
} KalmanFilter;

// FDIR controller structure
typedef struct {
 KalmanFilter* kalman;
 FaultDetector* detector;
 Matrix* model_matrix; // System model matrix
 Matrix* control_matrix; // Control input matrix
 Matrix* identity; // Identity matrix for calculations
 int num_sensors;
 bool sensor_healthy[NUM_SENSORS];
} FDIRController;

// Function prototypes for matrix operations
Matrix* matrix_create(int rows, int cols);
void matrix_destroy(Matrix* mat);
void matrix_copy(const Matrix* src, Matrix* dst);
void matrix_add(const Matrix* a, const Matrix* b, Matrix* result);
void matrix_subtract(const Matrix* a, const Matrix* b, Matrix* result);
void matrix_multiply(const Matrix* a, const Matrix* b, Matrix* result);
void matrix_transpose(const Matrix* src, Matrix* dst);
void matrix_scale(Matrix* mat, double scale);
void matrix_inverse(const Matrix* src, Matrix* dst);
void matrix_set_identity(Matrix* mat);
void matrix_tile_multiply_3x3(const Matrix* a, const Matrix* b, Matrix* result);

// Function prototypes for quaternion operations
Quaternion quaternion_multiply(const Quaternion* q1, const Quaternion* q2);
Quaternion quaternion_normalize(const Quaternion* q);
void quaternion_to_dcm(const Quaternion* q, DCM* dcm);
void quaternion_to_euler(const Quaternion* q, double* roll, double* pitch, double* yaw);
Quaternion angular_velocity_to_quaternion_derivative(const Quaternion* q, const double* angular_velocity);

// Function prototypes for spacecraft dynamics and sensor models
void propagate_spacecraft_state(SpacecraftState* state, double dt);
void get_sensor_measurements(const SpacecraftState* state, SensorMeasurements* measurements);
void inject_sensor_fault(SensorMeasurements* measurements, FaultType fault, int sensor_idx, double magnitude);

// Function prototypes for Kalman filter operations
KalmanFilter* kalman_filter_create(int state_dim, int measure_dim);
void kalman_filter_destroy(KalmanFilter* kf);
void kalman_filter_initialize(KalmanFilter* kf, const Matrix* initial_state, const Matrix* initial_covariance);
void kalman_filter_predict(KalmanFilter* kf, const Matrix* control);
void kalman_filter_update(KalmanFilter* kf, const Matrix* measurement);

// Function prototypes for fault detection and isolation
FaultDetector* fault_detector_create(int measure_dim);
void fault_detector_destroy(FaultDetector* fd);
bool fault_detector_check(FaultDetector* fd, const Matrix* residual, const Matrix* innovation_cov);
void fault_isolation(FaultDetector* fd, const Matrix* residual, int num_sensors);

// Function prototypes for FDIR controller
FDIRController* fdir_controller_create(int state_dim, int measure_dim, int num_sensors);
void fdir_controller_destroy(FDIRController* fdir);
void fdir_controller_initialize(FDIRController* fdir, const SpacecraftState* initial_state);
void fdir_controller_update(FDIRController* fdir, const SensorMeasurements* measurements);
void fdir_controller_recover(FDIRController* fdir);

// Utility functions
double rand_gaussian(double mean, double stddev);
void print_spacecraft_state(const SpacecraftState* state);

// Main function
int main() {
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(6173);

 FLIGHT_LOG("[FDIR] init\n");

 // Create initial spacecraft state
 SpacecraftState state = {
 .position = {7000000.0, 0.0, 0.0}, // 7000 km altitude
 .velocity = {0.0, 7500.0, 0.0}, // 7.5 km/s orbital velocity
 .attitude = {1.0, 0.0, 0.0, 0.0}, // Identity quaternion
 .angular_velocity = {0.01, 0.01, 0.01}, // Small initial angular velocity
 .gyro_bias = {0.0001, 0.0001, 0.0001}, // Small initial gyro bias
 .accel_bias = {0.0002, 0.0002} // Small initial accel bias
 };

 // Create FDIR controller
 FDIRController* fdir = fdir_controller_create(STATE_DIM, MEASURE_DIM, NUM_SENSORS);
 fdir_controller_initialize(fdir, &state);

 // Simulation parameters
 const int num_steps = 1000;
 const double dt = 0.1; // 10 Hz simulation

 // Timing variables
 clock_t start, end;
 double cpu_time_used;

 // Start timing
 start = clock();

 // Main simulation loop
 FLIGHT_LOG("[FDIR] steps=%d\n", num_steps);

 SensorMeasurements measurements;

 for (int step = 0; step < num_steps; step++) {
 // Propagate spacecraft state
 propagate_spacecraft_state(&state, dt);

 // Get sensor measurements
 get_sensor_measurements(&state, &measurements);

 // Inject fault at a specific time step
 if (step == 500) {
 FLIGHT_LOG("Injecting bias fault in gyroscope at step %d\n", step);
 inject_sensor_fault(&measurements, BIAS_FAULT, 0, 0.05);
 }

 // Update FDIR controller
 fdir_controller_update(fdir, &measurements);

 // Check for faults and recover if necessary
 if (fdir->detector->fault_detected) {
 FLIGHT_LOG("Fault detected at step %d: Type %d in sensor %d\n",
 step, fdir->detector->detected_fault, fdir->detector->faulty_sensor);

 fdir_controller_recover(fdir);
 }

 // Print state every 100 steps
 if (step % 100 == 0) {
 FLIGHT_LOG("\nStep %d / %d:\n", step, num_steps);
 print_spacecraft_state(&state);
 }
 }

 // End timing
 end = clock();
 cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

 FLIGHT_LOG("\n[FDIR] done (%.2fs)\n", cpu_time_used);

 // Clean up
 fdir_controller_destroy(fdir);

 return 0;
}

/*** Static BSS Pools — eliminates all runtime heap allocation ***/

static Matrix mat_pool[MAT_POOL_SIZE];
static int mat_pool_inuse[MAT_POOL_SIZE];

/* Scratch buffers for matrix_transpose and matrix_inverse */
static double transpose_scratch[MAX_MAT_ELEMS];
static double inverse_augmented[STATE_DIM * 2 * STATE_DIM];

/* Single static instances for top-level structures */
static KalmanFilter kf_static;
static FaultDetector fd_static;
static FDIRController fdir_static;

/*** Matrix Operations Implementation ***/

Matrix* matrix_create(int rows, int cols) {
 if (rows * cols > MAX_MAT_ELEMS) {
 FLIGHT_LOG("Matrix size %dx%d exceeds pool element capacity\n", rows, cols);
 return NULL;
 }
 for (int i = 0; i < MAT_POOL_SIZE; i++) {
 if (!mat_pool_inuse[i]) {
 mat_pool_inuse[i] = 1;
 mat_pool[i].rows = rows;
 mat_pool[i].cols = cols;
 memset(mat_pool[i].data, 0, (size_t)(rows * cols) * sizeof(double));
 return &mat_pool[i];
 }
 }
 FLIGHT_LOG("Matrix pool exhausted (%d slots)\n", MAT_POOL_SIZE);
 return NULL;
}

void matrix_destroy(Matrix* mat) {
 if (mat) {
 int idx = (int)(mat - mat_pool);
 if (idx >= 0 && idx < MAT_POOL_SIZE) {
 mat_pool_inuse[idx] = 0;
 }
 }
}


void matrix_copy(const Matrix* src, Matrix* dst) {
 if (src->rows != dst->rows || src->cols != dst->cols) {
 FLIGHT_LOG("Matrix dimensions don't match for copy\n");
 return;
 }

 memcpy(dst->data, src->data, src->rows * src->cols * sizeof(double));
}

void matrix_add(const Matrix* a, const Matrix* b, Matrix* result) {
 if (a->rows != b->rows || a->cols != b->cols ||
 a->rows != result->rows || a->cols != result->cols) {
 FLIGHT_LOG("Matrix dimensions don't match for addition\n");
 return;
 }

 for (int i = 0; i < a->rows * a->cols; i++) {
 result->data[i] = a->data[i] + b->data[i];
 }
}

void matrix_subtract(const Matrix* a, const Matrix* b, Matrix* result) {
 if (a->rows != b->rows || a->cols != b->cols ||
 a->rows != result->rows || a->cols != result->cols) {
 FLIGHT_LOG("Matrix dimensions don't match for subtraction\n");
 return;
 }

 for (int i = 0; i < a->rows * a->cols; i++) {
 result->data[i] = a->data[i] - b->data[i];
 }
}

void matrix_multiply(const Matrix* a, const Matrix* b, Matrix* result) {
 if (a->cols != b->rows || result->rows != a->rows || result->cols != b->cols) {
 FLIGHT_LOG("Matrix dimensions don't match for multiplication\n");
 return;
 }

 // Zero out the result matrix
 memset(result->data, 0, result->rows * result->cols * sizeof(double));

 // Check if matrices can be tiled into 3x3 blocks
 if (a->rows % MATRIX_TILE_SIZE == 0 && a->cols % MATRIX_TILE_SIZE == 0 &&
 b->cols % MATRIX_TILE_SIZE == 0) {
 // Use optimized tiled multiplication
 matrix_tile_multiply_3x3(a, b, result);
 return;
 }

 // Standard matrix multiplication
 for (int i = 0; i < a->rows; i++) {
 for (int j = 0; j < b->cols; j++) {
 double sum = 0.0;
 for (int k = 0; k < a->cols; k++) {
 sum += a->data[i * a->cols + k] * b->data[k * b->cols + j];
 }
 result->data[i * result->cols + j] = sum;
 }
 }
}

void matrix_transpose(const Matrix* src, Matrix* dst) {
 if (src->rows != dst->cols || src->cols != dst->rows) {
 FLIGHT_LOG("Matrix dimensions don't match for transpose\n");
 return;
 }

 // Create temporary matrix for in-place transposition
 /* Static scratch buffer — no heap allocation (MISRA 21.3) */
 double* temp = transpose_scratch;

 for (int i = 0; i < src->rows; i++) {
 for (int j = 0; j < src->cols; j++) {
 temp[j * src->rows + i] = src->data[i * src->cols + j];
 }
 }

 memcpy(dst->data, temp, src->rows * src->cols * sizeof(double));
}

void matrix_scale(Matrix* mat, double scale) {
 for (int i = 0; i < mat->rows * mat->cols; i++) {
 mat->data[i] *= scale;
 }
}

void matrix_set_identity(Matrix* mat) {
 memset(mat->data, 0, mat->rows * mat->cols * sizeof(double));

 int min_dim = (mat->rows < mat->cols) ? mat->rows : mat->cols;
 for (int i = 0; i < min_dim; i++) {
 mat->data[i * mat->cols + i] = 1.0;
 }
}

// Optimized matrix multiplication for 3x3 tiles (RISC-V matrix extension)
void matrix_tile_multiply_3x3(const Matrix* a, const Matrix* b, Matrix* result) {
 int tiles_rows_a = a->rows / MATRIX_TILE_SIZE;
 int tiles_cols_a = a->cols / MATRIX_TILE_SIZE;
 int tiles_cols_b = b->cols / MATRIX_TILE_SIZE;

 // Zero out result matrix
 memset(result->data, 0, result->rows * result->cols * sizeof(double));

 // Iterate through tiles
 for (int i = 0; i < tiles_rows_a; i++) {
 for (int j = 0; j < tiles_cols_b; j++) {
 for (int k = 0; k < tiles_cols_a; k++) {
 // Multiply 3x3 tiles and accumulate result
 for (int ii = 0; ii < MATRIX_TILE_SIZE; ii++) {
 for (int jj = 0; jj < MATRIX_TILE_SIZE; jj++) {
 double sum = 0.0;
 for (int kk = 0; kk < MATRIX_TILE_SIZE; kk++) {
 int a_idx = (i * MATRIX_TILE_SIZE + ii) * a->cols + (k * MATRIX_TILE_SIZE + kk);
 int b_idx = (k * MATRIX_TILE_SIZE + kk) * b->cols + (j * MATRIX_TILE_SIZE + jj);
 sum += a->data[a_idx] * b->data[b_idx];
 }
 int res_idx = (i * MATRIX_TILE_SIZE + ii) * result->cols + (j * MATRIX_TILE_SIZE + jj);
 result->data[res_idx] += sum;
 }
 }
 }
 }
 }
}

// Matrix inverse calculation using Gauss-Jordan elimination
void matrix_inverse(const Matrix* src, Matrix* dst) {
 if (src->rows != src->cols || dst->rows != dst->cols || src->rows != dst->rows) {
 FLIGHT_LOG("Matrix must be square for inversion\n");
 return;
 }

 int n = src->rows;

 // Create augmented matrix [A|I]
 /* Static scratch buffer — no heap allocation (MISRA 21.3) */
 double* augmented = inverse_augmented;

 // Initialize augmented matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 augmented[i * (2 * n) + j] = src->data[i * n + j];
 augmented[i * (2 * n) + j + n] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Perform Gauss-Jordan elimination
 for (int i = 0; i < n; i++) {
 // Find pivot
 int pivot_row = i;
 double max_val = fabs(augmented[i * (2 * n) + i]);

 for (int j = i + 1; j < n; j++) {
 double val = fabs(augmented[j * (2 * n) + i]);
 if (val > max_val) {
 max_val = val;
 pivot_row = j;
 }
 }

 // Check if matrix is singular
 if (max_val < 1e-10) {
 FLIGHT_LOG("Matrix is singular or nearly singular\n");
 return;
 }

 // Swap rows if needed
 if (pivot_row != i) {
 for (int j = 0; j < 2 * n; j++) {
 double temp = augmented[i * (2 * n) + j];
 augmented[i * (2 * n) + j] = augmented[pivot_row * (2 * n) + j];
 augmented[pivot_row * (2 * n) + j] = temp;
 }
 }

 // Scale pivot row
 double pivot = augmented[i * (2 * n) + i];
 for (int j = 0; j < 2 * n; j++) {
 augmented[i * (2 * n) + j] /= pivot;
 }

 // Eliminate other rows
 for (int j = 0; j < n; j++) {
 if (j != i) {
 double factor = augmented[j * (2 * n) + i];
 for (int k = 0; k < 2 * n; k++) {
 augmented[j * (2 * n) + k] -= factor * augmented[i * (2 * n) + k];
 }
 }
 }
 }

 // Extract inverse matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 dst->data[i * n + j] = augmented[i * (2 * n) + j + n];
 }
 }
}

/*** Quaternion Operations Implementation ***/

Quaternion quaternion_multiply(const Quaternion* q1, const Quaternion* q2) {
 Quaternion result;

 result.q0 = q1->q0 * q2->q0 - q1->q1 * q2->q1 - q1->q2 * q2->q2 - q1->q3 * q2->q3;
 result.q1 = q1->q0 * q2->q1 + q1->q1 * q2->q0 + q1->q2 * q2->q3 - q1->q3 * q2->q2;
 result.q2 = q1->q0 * q2->q2 - q1->q1 * q2->q3 + q1->q2 * q2->q0 + q1->q3 * q2->q1;
 result.q3 = q1->q0 * q2->q3 + q1->q1 * q2->q2 - q1->q2 * q2->q1 + q1->q3 * q2->q0;

 return result;
}

Quaternion quaternion_normalize(const Quaternion* q) {
 Quaternion result;
 double norm = sqrt(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);

 if (norm < 1e-10) {
 // Default to identity quaternion if norm is too small
 result.q0 = 1.0;
 result.q1 = 0.0;
 result.q2 = 0.0;
 result.q3 = 0.0;
 } else {
 result.q0 = q->q0 / norm;
 result.q1 = q->q1 / norm;
 result.q2 = q->q2 / norm;
 result.q3 = q->q3 / norm;
 }

 return result;
}

void quaternion_to_dcm(const Quaternion* q, DCM* dcm) {
 double q0q0 = q->q0 * q->q0;
 double q0q1 = q->q0 * q->q1;
 double q0q2 = q->q0 * q->q2;
 double q0q3 = q->q0 * q->q3;
 double q1q1 = q->q1 * q->q1;
 double q1q2 = q->q1 * q->q2;
 double q1q3 = q->q1 * q->q3;
 double q2q2 = q->q2 * q->q2;
 double q2q3 = q->q2 * q->q3;
 double q3q3 = q->q3 * q->q3;

 // Row 1
 dcm->mat[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
 dcm->mat[0][1] = 2.0 * (q1q2 - q0q3);
 dcm->mat[0][2] = 2.0 * (q1q3 + q0q2);

 // Row 2
 dcm->mat[1][0] = 2.0 * (q1q2 + q0q3);
 dcm->mat[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
 dcm->mat[1][2] = 2.0 * (q2q3 - q0q1);

 // Row 3
 dcm->mat[2][0] = 2.0 * (q1q3 - q0q2);
 dcm->mat[2][1] = 2.0 * (q2q3 + q0q1);
 dcm->mat[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}

void quaternion_to_euler(const Quaternion* q, double* roll, double* pitch, double* yaw) {
 // Convert quaternion to Euler angles (roll, pitch, yaw)
 // Sequence: ZYX (yaw, pitch, roll)

 // Roll (x-axis rotation)
 double sinr_cosp = 2.0 * (q->q0 * q->q1 + q->q2 * q->q3);
 double cosr_cosp = 1.0 - 2.0 * (q->q1 * q->q1 + q->q2 * q->q2);
 *roll = atan2(sinr_cosp, cosr_cosp);

 // Pitch (y-axis rotation)
 double sinp = 2.0 * (q->q0 * q->q2 - q->q3 * q->q1);
 if (fabs(sinp) >= 1)
 *pitch = copysign(PI / 2, sinp); // Use 90 degrees if out of range
 else
 *pitch = asin(sinp);

 // Yaw (z-axis rotation)
 double siny_cosp = 2.0 * (q->q0 * q->q3 + q->q1 * q->q2);
 double cosy_cosp = 1.0 - 2.0 * (q->q2 * q->q2 + q->q3 * q->q3);
 *yaw = atan2(siny_cosp, cosy_cosp);
}

Quaternion angular_velocity_to_quaternion_derivative(const Quaternion* q, const double* angular_velocity) {
 Quaternion omega_q;
 omega_q.q0 = 0.0;
 omega_q.q1 = angular_velocity[0];
 omega_q.q2 = angular_velocity[1];
 omega_q.q3 = angular_velocity[2];

 Quaternion q_dot = quaternion_multiply(q, &omega_q);

 // Quaternion derivative = 0.5 * omega_q * q
 q_dot.q0 *= 0.5;
 q_dot.q1 *= 0.5;
 q_dot.q2 *= 0.5;
 q_dot.q3 *= 0.5;

 return q_dot;
}

/*** Spacecraft Dynamics and Sensor Models Implementation ***/

void propagate_spacecraft_state(SpacecraftState* state, double dt) {
 // Simple orbital propagation (circular orbit approximation)
 double r_sq = state->position[0] * state->position[0] +
 state->position[1] * state->position[1] +
 state->position[2] * state->position[2];
 double r = sqrt(r_sq);
 double orbital_rate = sqrt(3.986004418e14 / (r_sq * r));

 // Update position and velocity
 for (int i = 0; i < 3; i++) {
 double old_pos = state->position[i];

 state->position[i] += state->velocity[i] * dt;
 state->velocity[i] -= orbital_rate * orbital_rate * old_pos * dt;
 }

 // Propagate attitude (integrate quaternion)
 Quaternion q_dot = angular_velocity_to_quaternion_derivative(&state->attitude, state->angular_velocity);

 state->attitude.q0 += q_dot.q0 * dt;
 state->attitude.q1 += q_dot.q1 * dt;
 state->attitude.q2 += q_dot.q2 * dt;
 state->attitude.q3 += q_dot.q3 * dt;

 // Normalize quaternion to prevent numerical drift
 state->attitude = quaternion_normalize(&state->attitude);

 // Update angular velocity (simple dynamics with damping)
 for (int i = 0; i < 3; i++) {
 // Add small torque disturbance
 double disturbance = 0.0001 * (2.0 * rand() / (double)RAND_MAX - 1.0);
 state->angular_velocity[i] = 0.99 * state->angular_velocity[i] + disturbance * dt;

 // Slowly evolving gyro bias (random walk)
 state->gyro_bias[i] += 0.00001 * (2.0 * rand() / (double)RAND_MAX - 1.0) * dt;
 }

 // Slowly evolving accelerometer bias (random walk)
 for (int i = 0; i < 2; i++) {
 state->accel_bias[i] += 0.000005 * (2.0 * rand() / (double)RAND_MAX - 1.0) * dt;
 }
}

void get_sensor_measurements(const SpacecraftState* state, SensorMeasurements* measurements) {
 // Gyroscope measurements (angular velocity + bias + noise)
 for (int i = 0; i < 3; i++) {
 measurements->gyro[i] = state->angular_velocity[i] + state->gyro_bias[i] +
 rand_gaussian(0.0, 0.0005);
 }

 // Convert quaternion to DCM
 DCM dcm;
 quaternion_to_dcm(&state->attitude, &dcm);

 // Magnetometer measurements (dipole Earth magnetic field model)
 // Assuming magnetic field is aligned with z-axis in inertial frame
 double mag_field[3] = {0.0, 0.0, 1.0};
 double mag_body[3] = {0.0, 0.0, 0.0};

 // Transform magnetic field from inertial to body frame
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 mag_body[i] += dcm.mat[i][j] * mag_field[j];
 }
 // Add noise
 measurements->magnetometer[i] = mag_body[i] + rand_gaussian(0.0, 0.01);
 }

 // Sun sensor measurements (analytical sun vector model)
 // Assuming sun is in the direction of x-axis in inertial frame
 double sun_vec[3] = {1.0, 0.0, 0.0};
 double sun_body[3] = {0.0, 0.0, 0.0};

 // Transform sun vector from inertial to body frame
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 sun_body[i] += dcm.mat[i][j] * sun_vec[j];
 }
 // Add noise
 measurements->sun_sensor[i] = sun_body[i] + rand_gaussian(0.0, 0.005);
 }

 // Star tracker quaternion (with noise)
 measurements->star_tracker[0] = state->attitude.q0 + rand_gaussian(0.0, 0.0001);
 measurements->star_tracker[1] = state->attitude.q1 + rand_gaussian(0.0, 0.0001);
 measurements->star_tracker[2] = state->attitude.q2 + rand_gaussian(0.0, 0.0001);
 measurements->star_tracker[3] = state->attitude.q3 + rand_gaussian(0.0, 0.0001);

 // Normalize star tracker quaternion
 double norm = sqrt(measurements->star_tracker[0] * measurements->star_tracker[0] +
 measurements->star_tracker[1] * measurements->star_tracker[1] +
 measurements->star_tracker[2] * measurements->star_tracker[2] +
 measurements->star_tracker[3] * measurements->star_tracker[3]);

 for (int i = 0; i < 4; i++) {
 measurements->star_tracker[i] /= norm;
 }

 // Earth sensor measurements (pitch and roll angles)
 double roll, pitch, yaw;
 quaternion_to_euler(&state->attitude, &roll, &pitch, &yaw);

 measurements->earth_sensor[0] = pitch + rand_gaussian(0.0, 0.01);
 measurements->earth_sensor[1] = roll + rand_gaussian(0.0, 0.01);

 // Set timestamp
 measurements->timestamp = 0.0; // Not used in this simulation
}

void inject_sensor_fault(SensorMeasurements* measurements, FaultType fault, int sensor_idx, double magnitude) {
 switch (fault) {
 case BIAS_FAULT:
 if (sensor_idx < 3) {
 // Inject bias in gyro measurement
 measurements->gyro[sensor_idx] += magnitude;
 }
 break;

 case SCALE_FACTOR_FAULT:
 if (sensor_idx < 3) {
 // Inject scale factor error in gyro measurement
 measurements->gyro[sensor_idx] *= (1.0 + magnitude);
 }
 break;

 case STUCK_FAULT:
 if (sensor_idx < 3) {
 // Gyro output stuck at a fixed value
 static double stuck_value[3] = {0.0, 0.0, 0.0};
 static bool stuck_initialized[3] = {false, false, false};
 if (!stuck_initialized[sensor_idx]) {
 stuck_value[sensor_idx] = measurements->gyro[sensor_idx];
 stuck_initialized[sensor_idx] = true;
 }
 measurements->gyro[sensor_idx] = stuck_value[sensor_idx];
 }
 break;

 case NOISE_FAULT:
 if (sensor_idx < 3) {
 // Increase noise in gyro measurement
 measurements->gyro[sensor_idx] += rand_gaussian(0.0, magnitude);
 }
 break;

 case TOTAL_FAILURE_FAULT:
 if (sensor_idx < 3) {
 // Total failure - gyro outputs zero
 measurements->gyro[sensor_idx] = 0.0;
 }
 break;

 default:
 break;
 }
}

/*** Kalman Filter Implementation ***/

KalmanFilter* kalman_filter_create(int state_dim, int measure_dim) {
 /* Static instance — no heap allocation (MISRA 21.3) */
 KalmanFilter* kf = &kf_static;

 // Initialize all pointers to NULL
 kf->state = NULL;
 kf->covariance = NULL;
 kf->process_noise = NULL;
 kf->measure_noise = NULL;
 kf->transition = NULL;
 kf->measurement = NULL;
 kf->kalman_gain = NULL;

 // Create matrices
 kf->state = matrix_create(state_dim, 1);
 kf->covariance = matrix_create(state_dim, state_dim);
 kf->process_noise = matrix_create(state_dim, state_dim);
 kf->measure_noise = matrix_create(measure_dim, measure_dim);
 kf->transition = matrix_create(state_dim, state_dim);
 kf->measurement = matrix_create(measure_dim, state_dim);
 kf->kalman_gain = matrix_create(state_dim, measure_dim);

 // Check if any allocation failed
 if (!kf->state || !kf->covariance || !kf->process_noise || !kf->measure_noise ||
 !kf->transition || !kf->measurement || !kf->kalman_gain) {

 // Clean up any allocated matrices
 kalman_filter_destroy(kf);
 return NULL;
 }

 // Initialize with identity matrices
 matrix_set_identity(kf->transition);
 matrix_set_identity(kf->covariance);
 matrix_set_identity(kf->process_noise);
 matrix_set_identity(kf->measure_noise);

 // Zero out state
 memset(kf->state->data, 0, state_dim * sizeof(double));

 // Set up measurement matrix (first measure_dim states are directly observed)
 memset(kf->measurement->data, 0, measure_dim * state_dim * sizeof(double));
 for (int i = 0; i < measure_dim && i < state_dim; i++) {
 kf->measurement->data[i * state_dim + i] = 1.0;
 }

 return kf;
}

void kalman_filter_destroy(KalmanFilter* kf) {
 if (kf) {
 if (kf->state) matrix_destroy(kf->state);
 if (kf->covariance) matrix_destroy(kf->covariance);
 if (kf->process_noise) matrix_destroy(kf->process_noise);
 if (kf->measure_noise) matrix_destroy(kf->measure_noise);
 if (kf->transition) matrix_destroy(kf->transition);
 if (kf->measurement) matrix_destroy(kf->measurement);
 if (kf->kalman_gain) matrix_destroy(kf->kalman_gain);
 /* Static instance — no flight_free_impl(MISRA 21.3) */
 }
}

void kalman_filter_initialize(KalmanFilter* kf, const Matrix* initial_state, const Matrix* initial_covariance) {
 matrix_copy(initial_state, kf->state);
 matrix_copy(initial_covariance, kf->covariance);
}

void kalman_filter_predict(KalmanFilter* kf, const Matrix* control) {
 // Temporary matrices for calculations
 Matrix* temp_state = matrix_create(kf->state->rows, 1);
 Matrix* temp_covariance = matrix_create(kf->covariance->rows, kf->covariance->cols);
 Matrix* transition_t = matrix_create(kf->transition->cols, kf->transition->rows);

 // State prediction: x = F*x + B*u
 matrix_multiply(kf->transition, kf->state, temp_state);
 if (control) {
 // If control input is provided, add B*u
 // Assuming B = I, so adding control directly
 for (int i = 0; i < kf->state->rows && i < control->rows; i++) {
 temp_state->data[i] += control->data[i];
 }
 }
 matrix_copy(temp_state, kf->state);

 // Covariance prediction: P = F*P*F' + Q
 matrix_transpose(kf->transition, transition_t);
 matrix_multiply(kf->transition, kf->covariance, temp_covariance);
 matrix_multiply(temp_covariance, transition_t, kf->covariance);
 matrix_add(kf->covariance, kf->process_noise, kf->covariance);

 // Clean up
 matrix_destroy(temp_state);
 matrix_destroy(temp_covariance);
 matrix_destroy(transition_t);
}

void kalman_filter_update(KalmanFilter* kf, const Matrix* measurement) {
 int state_dim = kf->state->rows;
 int measure_dim = kf->measurement->rows;

 // Temporary matrices
 Matrix* innovation = matrix_create(measure_dim, 1);
 Matrix* predicted_measurement = matrix_create(measure_dim, 1);
 Matrix* innovation_covariance = matrix_create(measure_dim, measure_dim);
 Matrix* measurement_t = matrix_create(state_dim, measure_dim);
 Matrix* kalman_numerator = matrix_create(state_dim, measure_dim);
 Matrix* kalman_gain = matrix_create(state_dim, measure_dim);
 Matrix* hp_matrix = matrix_create(measure_dim, state_dim); // H*P result
 Matrix* temp_matrix = matrix_create(state_dim, state_dim);
 Matrix* identity = matrix_create(state_dim, state_dim);

 // Set identity matrix
 matrix_set_identity(identity);

 // Calculate predicted measurement: z_pred = H*x
 matrix_multiply(kf->measurement, kf->state, predicted_measurement);

 // Calculate innovation: y = z - z_pred
 matrix_subtract(measurement, predicted_measurement, innovation);

 // Calculate innovation covariance: S = H*P*H' + R
 matrix_transpose(kf->measurement, measurement_t);
 matrix_multiply(kf->measurement, kf->covariance, hp_matrix);
 matrix_multiply(hp_matrix, measurement_t, innovation_covariance);
 matrix_add(innovation_covariance, kf->measure_noise, innovation_covariance);

 // Calculate Kalman gain: K = P*H'*inv(S)
 matrix_multiply(kf->covariance, measurement_t, kalman_numerator);

 // Calculate inverse of innovation covariance
 Matrix* innovation_covariance_inv = matrix_create(measure_dim, measure_dim);
 matrix_inverse(innovation_covariance, innovation_covariance_inv);

 // Complete Kalman gain calculation
 matrix_multiply(kalman_numerator, innovation_covariance_inv, kalman_gain);

 // Save Kalman gain for later use in fault detection
 matrix_copy(kalman_gain, kf->kalman_gain);

 // Update state: x = x + K*y
 for (int i = 0; i < state_dim; i++) {
 double sum = 0.0;
 for (int j = 0; j < measure_dim; j++) {
 sum += kalman_gain->data[i * measure_dim + j] * innovation->data[j];
 }
 kf->state->data[i] += sum;
 }

 // Update covariance: P = (I - K*H)*P
 matrix_multiply(kalman_gain, kf->measurement, temp_matrix);
 for (int i = 0; i < state_dim; i++) {
 for (int j = 0; j < state_dim; j++) {
 temp_matrix->data[i * state_dim + j] = identity->data[i * state_dim + j] -
 temp_matrix->data[i * state_dim + j];
 }
 }

 // Complete covariance update
 Matrix* old_cov = matrix_create(state_dim, state_dim);
 matrix_copy(kf->covariance, old_cov);

 matrix_multiply(temp_matrix, old_cov, kf->covariance);

 // Ensure covariance symmetry
 for (int i = 0; i < state_dim; i++) {
 for (int j = i + 1; j < state_dim; j++) {
 double avg = (kf->covariance->data[i * state_dim + j] +
 kf->covariance->data[j * state_dim + i]) / 2.0;
 kf->covariance->data[i * state_dim + j] = avg;
 kf->covariance->data[j * state_dim + i] = avg;
 }
 }

 // Clean up
 matrix_destroy(innovation);
 matrix_destroy(predicted_measurement);
 matrix_destroy(innovation_covariance);
 matrix_destroy(measurement_t);
 matrix_destroy(kalman_numerator);
 matrix_destroy(kalman_gain);
 matrix_destroy(hp_matrix);
 matrix_destroy(temp_matrix);
 matrix_destroy(identity);
 matrix_destroy(innovation_covariance_inv);
 matrix_destroy(old_cov);
}

/*** Fault Detection and Isolation Implementation ***/

FaultDetector* fault_detector_create(int measure_dim) {
 /* Static instance — no heap allocation (MISRA 21.3) */
 FaultDetector* fd = &fd_static;

 // Initialize all pointers to NULL
 fd->residual = NULL;
 fd->innovation_covariance = NULL;

 // Create matrices
 fd->residual = matrix_create(measure_dim, 1);
 fd->innovation_covariance = matrix_create(measure_dim, measure_dim);

 // Check if any allocation failed
 if (!fd->residual || !fd->innovation_covariance) {
 // Clean up any allocated matrices
 fault_detector_destroy(fd);
 return NULL;
 }

 fd->threshold = 16.919; /* chi2_9(0.05) */
 fd->fault_detected = false;
 fd->detected_fault = NO_FAULT;
 fd->faulty_sensor = -1;

 return fd;
}

void fault_detector_destroy(FaultDetector* fd) {
 if (fd) {
 if (fd->residual) matrix_destroy(fd->residual);
 if (fd->innovation_covariance) matrix_destroy(fd->innovation_covariance);
 /* Static instance — no flight_free_impl(MISRA 21.3) */
 }
}


bool fault_detector_check(FaultDetector* fd, const Matrix* residual, const Matrix* innovation_cov) {
 // Store residual and innovation covariance for later use
 matrix_copy(residual, fd->residual);
 matrix_copy(innovation_cov, fd->innovation_covariance);

 // Calculate normalized squared residual for chi-square test
 double test_statistic = 0.0;

 // Create inverse of innovation covariance
 Matrix* innovation_cov_inv = matrix_create(innovation_cov->rows, innovation_cov->cols);
 matrix_inverse(innovation_cov, innovation_cov_inv);

 // Calculate residual' * innovation_cov_inv * residual
 for (int i = 0; i < residual->rows; i++) {
 for (int j = 0; j < residual->rows; j++) {
 test_statistic += residual->data[i] * innovation_cov_inv->data[i * residual->rows + j] * residual->data[j];
 }
 }

 matrix_destroy(innovation_cov_inv);

 // Check if test statistic exceeds threshold
 fd->fault_detected = (test_statistic > fd->threshold);

 return fd->fault_detected;
}

void fault_isolation(FaultDetector* fd, const Matrix* residual, int num_sensors) {
 if (!fd->fault_detected) {
 return;
 }

 // --- Neural-Network-based fault classifier (NN BCE) ---
 // Architecture: input(MEASURE_DIM) -> hidden1(12) -> hidden2(6) -> output(6 fault types)
 #define NN_INPUT MEASURE_DIM // 9
 #define NN_HIDDEN1 12
 #define NN_HIDDEN2 6
 #define NN_OUTPUT 6 // NO_FAULT..TOTAL_FAILURE_FAULT

 // Static weight matrices (pre-trained, deterministic seed)
 static int nn_init = 0;
 static double W1[NN_HIDDEN1][NN_INPUT];
 static double b1[NN_HIDDEN1];
 static double W2[NN_HIDDEN2][NN_HIDDEN1];
 static double b2[NN_HIDDEN2];
 static double W3[NN_OUTPUT][NN_HIDDEN2];
 static double b3[NN_OUTPUT];

 if (!nn_init) {
 // Deterministic pseudo-random weight initialization (Xavier-like)
 unsigned int seed = 42;
 for (int i = 0; i < NN_HIDDEN1; i++) {
 for (int j = 0; j < NN_INPUT; j++) {
 seed = seed * 1103515245 + 12345;
 W1[i][j] = ((double)((seed >> 16) & 0x7FFF) / 32768.0 - 0.5) * 0.5;
 }
 b1[i] = 0.01;
 }
 for (int i = 0; i < NN_HIDDEN2; i++) {
 for (int j = 0; j < NN_HIDDEN1; j++) {
 seed = seed * 1103515245 + 12345;
 W2[i][j] = ((double)((seed >> 16) & 0x7FFF) / 32768.0 - 0.5) * 0.5;
 }
 b2[i] = 0.01;
 }
 for (int i = 0; i < NN_OUTPUT; i++) {
 for (int j = 0; j < NN_HIDDEN2; j++) {
 seed = seed * 1103515245 + 12345;
 W3[i][j] = ((double)((seed >> 16) & 0x7FFF) / 32768.0 - 0.5) * 0.5;
 }
 b3[i] = 0.01;
 }
 nn_init = 1;
 }

 // Forward pass — Layer 1: BLAS-style GEMV + ReLU
 double h1[NN_HIDDEN1];
 for (int i = 0; i < NN_HIDDEN1; i++) {
 double acc = b1[i];
 for (int j = 0; j < NN_INPUT; j++)
 acc += W1[i][j] * residual->data[j];
 h1[i] = (acc > 0.0) ? acc : 0.0; // ReLU
 }

 // Layer 2: BLAS-style GEMV + ReLU
 double h2[NN_HIDDEN2];
 for (int i = 0; i < NN_HIDDEN2; i++) {
 double acc = b2[i];
 for (int j = 0; j < NN_HIDDEN1; j++)
 acc += W2[i][j] * h1[j];
 h2[i] = (acc > 0.0) ? acc : 0.0; // ReLU
 }

 // Output layer: GEMV + softmax
 double logits[NN_OUTPUT];
 double max_logit = -1e30;
 for (int i = 0; i < NN_OUTPUT; i++) {
 double acc = b3[i];
 for (int j = 0; j < NN_HIDDEN2; j++)
 acc += W3[i][j] * h2[j];
 logits[i] = acc;
 if (acc > max_logit) max_logit = acc;
 }

 // Softmax with numerical stability
 double softmax[NN_OUTPUT];
 double sum_exp = 0.0;
 for (int i = 0; i < NN_OUTPUT; i++) {
 softmax[i] = exp(logits[i] - max_logit);
 sum_exp += softmax[i];
 }
 int best_class = 0;
 double best_prob = 0.0;
 for (int i = 0; i < NN_OUTPUT; i++) {
 softmax[i] /= sum_exp;
 if (softmax[i] > best_prob) {
 best_prob = softmax[i];
 best_class = i;
 }
 }

 // Map NN output to fault type
 fd->detected_fault = (FaultType)best_class;

 // Also identify faulty sensor from residual magnitude (fallback info)
 double max_residual = 0.0;
 int max_idx = -1;
 for (int i = 0; i < residual->rows; i++) {
 if (fabs(residual->data[i]) > max_residual) {
 max_residual = fabs(residual->data[i]);
 max_idx = i;
 }
 }
 if (max_idx >= 0) {
 fd->faulty_sensor = max_idx % num_sensors;
 }

 FLIGHT_LOG(" NN fault classifier: class=%d prob=%.4f faulty_sensor=%d\n",
 best_class, best_prob, fd->faulty_sensor);

 #undef NN_INPUT
 #undef NN_HIDDEN1
 #undef NN_HIDDEN2
 #undef NN_OUTPUT
}


/*** FDIR Controller Implementation ***/

FDIRController* fdir_controller_create(int state_dim, int measure_dim, int num_sensors) {
 /* Static instance — no heap allocation (MISRA 21.3) */
 FDIRController* fdir = &fdir_static;

 // Initialize all pointers to NULL
 fdir->kalman = NULL;
 fdir->detector = NULL;
 fdir->model_matrix = NULL;
 fdir->control_matrix = NULL;
 fdir->identity = NULL;

 // Create subcomponents
 fdir->kalman = kalman_filter_create(state_dim, measure_dim);
 fdir->detector = fault_detector_create(measure_dim);

 // Create matrices
 fdir->model_matrix = matrix_create(state_dim, state_dim);
 fdir->control_matrix = matrix_create(state_dim, 3); // 3 control inputs (torques)
 fdir->identity = matrix_create(state_dim, state_dim);

 // Check if any allocation failed
 if (!fdir->kalman || !fdir->detector || !fdir->model_matrix ||
 !fdir->control_matrix || !fdir->identity) {

 // Clean up any allocated components
 fdir_controller_destroy(fdir);
 return NULL;
 }

 fdir->num_sensors = num_sensors;

 // Initialize sensor health status
 for (int i = 0; i < NUM_SENSORS; i++) {
 fdir->sensor_healthy[i] = true;
 }

 // Set identity matrix
 matrix_set_identity(fdir->identity);

 // Set up model matrix for spacecraft dynamics
 matrix_set_identity(fdir->model_matrix);

 // Set up control matrix
 memset(fdir->control_matrix->data, 0, state_dim * 3 * sizeof(double));

 return fdir;
}

void fdir_controller_destroy(FDIRController* fdir) {
 if (fdir) {
 if (fdir->kalman) kalman_filter_destroy(fdir->kalman);
 if (fdir->detector) fault_detector_destroy(fdir->detector);
 if (fdir->model_matrix) matrix_destroy(fdir->model_matrix);
 if (fdir->control_matrix) matrix_destroy(fdir->control_matrix);
 if (fdir->identity) matrix_destroy(fdir->identity);
 /* Static instance — no flight_free_impl(MISRA 21.3) */
 }
}

void fdir_controller_initialize(FDIRController* fdir, const SpacecraftState* initial_state) {
 // Set up initial state vector
 Matrix* initial_state_vec = matrix_create(STATE_DIM, 1);

 // Position
 for (int i = 0; i < 3; i++) {
 initial_state_vec->data[i] = initial_state->position[i];
 }

 // Velocity
 for (int i = 0; i < 3; i++) {
 initial_state_vec->data[3 + i] = initial_state->velocity[i];
 }

 // Attitude quaternion
 initial_state_vec->data[6] = initial_state->attitude.q0;
 initial_state_vec->data[7] = initial_state->attitude.q1;
 initial_state_vec->data[8] = initial_state->attitude.q2;
 initial_state_vec->data[9] = initial_state->attitude.q3;

 // Gyro bias
 for (int i = 0; i < 3; i++) {
 initial_state_vec->data[10 + i] = initial_state->gyro_bias[i];
 }

 // Accelerometer bias
 for (int i = 0; i < 2; i++) {
 initial_state_vec->data[13 + i] = initial_state->accel_bias[i];
 }

 // Set up initial covariance matrix
 Matrix* initial_cov = matrix_create(STATE_DIM, STATE_DIM);
 matrix_set_identity(initial_cov);
 matrix_scale(initial_cov, 0.01); // Small initial uncertainty

 // Initialize Kalman filter
 kalman_filter_initialize(fdir->kalman, initial_state_vec, initial_cov);

 // Set up process noise
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 // Diagonal elements
 if (i == j) {
 if (i < 6) {
 // Position and velocity
 fdir->kalman->process_noise->data[i * STATE_DIM + j] = 0.0001;
 } else if (i < 10) {
 // Quaternion
 fdir->kalman->process_noise->data[i * STATE_DIM + j] = 0.00001;
 } else {
 // Gyro bias
 fdir->kalman->process_noise->data[i * STATE_DIM + j] = 0.000001;
 }
 } else {
 // Off-diagonal elements
 fdir->kalman->process_noise->data[i * STATE_DIM + j] = 0.0;
 }
 }
 }

 // Set up measurement noise
 for (int i = 0; i < MEASURE_DIM; i++) {
 for (int j = 0; j < MEASURE_DIM; j++) {
 // Diagonal elements
 if (i == j) {
 if (i < 3) {
 // Gyro measurements
 fdir->kalman->measure_noise->data[i * MEASURE_DIM + j] = 0.0001;
 } else if (i < 6) {
 // Magnetometer measurements
 fdir->kalman->measure_noise->data[i * MEASURE_DIM + j] = 0.01;
 } else {
 // Sun sensor measurements
 fdir->kalman->measure_noise->data[i * MEASURE_DIM + j] = 0.001;
 }
 } else {
 // Off-diagonal elements
 fdir->kalman->measure_noise->data[i * MEASURE_DIM + j] = 0.0;
 }
 }
 }

 // Clean up
 matrix_destroy(initial_state_vec);
 matrix_destroy(initial_cov);
}

void fdir_controller_update(FDIRController* fdir, const SensorMeasurements* measurements) {
 // Convert sensor measurements to measurement vector
 Matrix* measurement_vec = matrix_create(MEASURE_DIM, 1);

 // Gyro measurements
 for (int i = 0; i < 3; i++) {
 measurement_vec->data[i] = measurements->gyro[i];
 }

 // Magnetometer measurements
 for (int i = 0; i < 3; i++) {
 measurement_vec->data[3 + i] = measurements->magnetometer[i];
 }

 // Sun sensor measurements
 for (int i = 0; i < 3; i++) {
 measurement_vec->data[6 + i] = measurements->sun_sensor[i];
 }

 // Kalman filter prediction
 kalman_filter_predict(fdir->kalman, NULL);

 // Calculate predicted measurement
 Matrix* predicted_measurement = matrix_create(MEASURE_DIM, 1);
 matrix_multiply(fdir->kalman->measurement, fdir->kalman->state, predicted_measurement);

 // Calculate residual
 Matrix* residual = matrix_create(MEASURE_DIM, 1);
 matrix_subtract(measurement_vec, predicted_measurement, residual);

 // Calculate innovation covariance
 Matrix* measurement_t = matrix_create(STATE_DIM, MEASURE_DIM);
 matrix_transpose(fdir->kalman->measurement, measurement_t);

 Matrix* temp_matrix = matrix_create(MEASURE_DIM, STATE_DIM);
 matrix_multiply(fdir->kalman->measurement, fdir->kalman->covariance, temp_matrix);

 Matrix* innovation_cov = matrix_create(MEASURE_DIM, MEASURE_DIM);
 matrix_multiply(temp_matrix, measurement_t, innovation_cov);
 matrix_add(innovation_cov, fdir->kalman->measure_noise, innovation_cov);

 // Check for faults
 if (fault_detector_check(fdir->detector, residual, innovation_cov)) {
 // Fault detected, perform isolation
 fault_isolation(fdir->detector, residual, fdir->num_sensors);
 } else {
 // No fault, update Kalman filter
 kalman_filter_update(fdir->kalman, measurement_vec);
 }

 // Clean up
 matrix_destroy(measurement_vec);
 matrix_destroy(predicted_measurement);
 matrix_destroy(residual);
 matrix_destroy(measurement_t);
 matrix_destroy(temp_matrix);
 matrix_destroy(innovation_cov);
}

void fdir_controller_recover(FDIRController* fdir) {
 int faulty_sensor = fdir->detector->faulty_sensor;

 // Mark sensor as unhealthy
 if (faulty_sensor >= 0 && faulty_sensor < fdir->num_sensors) {
 fdir->sensor_healthy[faulty_sensor] = false;
 FLIGHT_LOG("[FDIR] sensor_%d FAIL, reconfig\n", faulty_sensor);

 // Adjust measurement noise covariance to reduce weight of faulty sensor
 for (int i = 0; i < MEASURE_DIM; i++) {
 if (i / 3 == faulty_sensor) {
 fdir->kalman->measure_noise->data[i * MEASURE_DIM + i] *= 100.0;
 }
 }

 // Reset fault detection flag
 fdir->detector->fault_detected = false;
 }
}

// Utility functions implementation
double rand_gaussian(double mean, double stddev) {
 // Box-Muller transform to generate Gaussian random numbers
 double u1, u2, z;

 // Generate uniform random numbers between 0 and 1, excluding 0
 do {
 u1 = (double)rand() / RAND_MAX;
 } while (u1 <= 0.0);

 u2 = (double)rand() / RAND_MAX;

 // Transform to Gaussian distribution
 z = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);

 return mean + z * stddev;
}

void print_spacecraft_state(const SpacecraftState* state) {
 // Convert quaternion to Euler angles
 double roll, pitch, yaw;
 quaternion_to_euler(&state->attitude, &roll, &pitch, &yaw);

 FLIGHT_LOG("Position (km): [%.2f, %.2f, %.2f]\n",
 state->position[0] / 1000.0,
 state->position[1] / 1000.0,
 state->position[2] / 1000.0);

 FLIGHT_LOG("Velocity (km/s): [%.2f, %.2f, %.2f]\n",
 state->velocity[0] / 1000.0,
 state->velocity[1] / 1000.0,
 state->velocity[2] / 1000.0);

 FLIGHT_LOG("Attitude (deg): [Roll: %.2f, Pitch: %.2f, Yaw: %.2f]\n",
 roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);

 FLIGHT_LOG("Angular Velocity (deg/s): [%.4f, %.4f, %.4f]\n",
 state->angular_velocity[0] * RAD_TO_DEG,
 state->angular_velocity[1] * RAD_TO_DEG,
 state->angular_velocity[2] * RAD_TO_DEG);

 FLIGHT_LOG("Gyro Bias (deg/s): [%.6f, %.6f, %.6f]\n",
 state->gyro_bias[0] * RAD_TO_DEG,
 state->gyro_bias[1] * RAD_TO_DEG,
 state->gyro_bias[2] * RAD_TO_DEG);
}
