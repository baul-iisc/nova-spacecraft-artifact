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
 * Attitude Determination and Control System Algorithm
 *
 * Author : Boul Chandra Garai
 * Variant: HW Trig (CORDIC accelerator for trigonometric functions)
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the core ADCS algorithm with quaternion-based attitude estimation,
 * reaction wheel control law computation, and magnetorquer desaturation logic.
 * This workload dominates in trigonometric calculations for attitude kinematics,
 * matrix operations for moment-of-inertia computations, and quaternion algebra.
 * Runs on every satellite platforms from microsatellites (IMS-1) to heavy
 * GEO communication satellites (GEO communication satellite).
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
#define sin(a) hw_sin(a)
#define cos(a) hw_cos(a)

/* -----------------------------------------------------------------------
 * Telemetry Logging Subsystem
 *
 * In flight software (e.g., crewed-spacecraft OBC), printf/stderr are
 * prohibited. All diagnostic output is routed through a structured
 * telemetry logging API that formats CCSDS housekeeping packets and
 * enqueues them to the downlink buffer. tm_log() emulates this
 * interface using printf as a gem5-simulation-compatible backend.
 * ----------------------------------------------------------------------- */
typedef enum {
 TM_INFO = 0, /* Nominal housekeeping */
 TM_STEP = 1, /* Periodic progress */
 TM_RESULT = 2, /* End-of-run summary */
 TM_WARN = 3 /* Off-nominal condition */
} TmLevel;

static const char *tm_tag[] = {"INFO", "STEP", "RSLT", "WARN"};

#define tm_log(lvl, fmt, ...) \
 FLIGHT_LOG("[ADCS][%s] " fmt, tm_tag[lvl], ##__VA_ARGS__)

// Define constant parameters
#define PI 3.14159265358979323846
#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)

// MEKF state for crewed spacecraft missions:
// attitude_error(3) + angular_velocity(3) + gyro_bias(3) + thruster_misalign(3)
#define STATE_DIM 12
#define ACTUATOR_COUNT 8
#define WHEEL_COUNT 4

// Orbital mechanics constants
#define MU_EARTH 3.986004418e14 // Earth gravitational parameter (m³/s²)
#define R_EARTH 6371000.0 // Earth radius (m)
#define ORBIT_ALTITUDE 600000.0 // LEO altitude (m)
#define SEMI_MAJOR_AXIS (R_EARTH + ORBIT_ALTITUDE) // Semi-major axis (m)

// Struct for 3D vectors
typedef struct {
 double x;
 double y;
 double z;
} Vector3;

// Struct for quaternions
typedef struct {
 double q1; // scalar part
 double q2; // vector part x
 double q3; // vector part y
 double q4; // vector part z
} Quaternion;

// Struct for 3x3 matrices (Direction Cosine Matrix)
typedef struct {
 double m[3][3];
} Matrix3x3;

// Forward declaration for the augmented matrix used in matrix_inverse
typedef struct {
 double data[STATE_DIM][2*STATE_DIM];
} AugmentedMatrix;

// Struct for large matrices
typedef struct {
 double data[STATE_DIM][STATE_DIM];
} LargeMatrix;

// Sensor measurements
typedef struct {
 Vector3 gyro; // Angular velocity from gyroscope
 Vector3 magnetometer; // Magnetic field vector
 Vector3 sun_sensor; // Sun vector
 Vector3 star_tracker; // Star tracker attitude
 double timestamp; // Measurement time
} SensorData;

// Actuator commands
typedef struct {
 double wheel_torque[WHEEL_COUNT]; // Reaction wheel torques
 double magnetic_moment[3]; // Magnetic torquer moments
 double thruster_force[ACTUATOR_COUNT-WHEEL_COUNT-3]; // Thrusters
} ActuatorCommands;

// Spacecraft state
typedef struct {
 Quaternion attitude; // Attitude quaternion
 Vector3 angular_vel; // Angular velocity
 Vector3 position; // Position in orbit
 Vector3 velocity; // Velocity in orbit
 double wheel_momentum[WHEEL_COUNT]; // Wheel angular momentum
 double state_vector[STATE_DIM]; // Complete state vector for EKF
 double timestamp; // State time
} SpacecraftState;

// Function prototypes
void initialize_state(SpacecraftState *state);
void quaternion_multiply(Quaternion *q1, Quaternion *q2, Quaternion *result);
void quaternion_to_dcm(Quaternion *q, Matrix3x3 *dcm);
void quaternion_normalize(Quaternion *q);
void matrix_multiply(LargeMatrix *A, LargeMatrix *B, LargeMatrix *C);
void matrix_transpose(LargeMatrix *A, LargeMatrix *AT);
void matrix_inverse(LargeMatrix *A, LargeMatrix *A_inv);
void matrix_vector_multiply(LargeMatrix *A, double *v_in, double *v_out);
void attitude_determination(SensorData *sensor_data, SpacecraftState *state);
void attitude_control(SpacecraftState *state, ActuatorCommands *commands,
 const Quaternion *q_target);
void compute_orbital_parameters(double time, double *mean_motion, double *orbital_period,
 double *true_anomaly, double *orbital_radius);
void compute_gravity_gradient_torque(SpacecraftState *state, double orbital_radius,
 Vector3 *torque_gg);
void eci_to_body_transform(Quaternion *attitude, Vector3 *vec_eci, Vector3 *vec_body);

// Implementation of core ADCS functions
void initialize_state(SpacecraftState *state) {
 // Initialize quaternion to identity rotation
 state->attitude.q1 = 1.0;
 state->attitude.q2 = 0.0;
 state->attitude.q3 = 0.0;
 state->attitude.q4 = 0.0;

 // Initialize angular velocity to zero
 state->angular_vel.x = 0.0;
 state->angular_vel.y = 0.0;
 state->angular_vel.z = 0.0;

 // Initialize wheels to zero momentum
 for (int i = 0; i < WHEEL_COUNT; i++) {
 state->wheel_momentum[i] = 0.0;
 }

 // Initialize full state vector (used in EKF)
 // States 0- 2: attitude error (δθ)
 // States 3- 5: angular velocity (ω)
 // States 6- 8: gyro bias (b_gyro)
 // States 9-11: thruster misalignment (δα) — crewed mission extension
 for (int i = 0; i < STATE_DIM; i++) {
 state->state_vector[i] = 0.0;
 }

 // Set initial timestamp
 state->timestamp = 0.0;
}

// Quaternion multiplication: result = q1 * q2
void quaternion_multiply(Quaternion *q1, Quaternion *q2, Quaternion *result) {
 result->q1 = q1->q1*q2->q1 - q1->q2*q2->q2 - q1->q3*q2->q3 - q1->q4*q2->q4;
 result->q2 = q1->q1*q2->q2 + q1->q2*q2->q1 + q1->q3*q2->q4 - q1->q4*q2->q3;
 result->q3 = q1->q1*q2->q3 - q1->q2*q2->q4 + q1->q3*q2->q1 + q1->q4*q2->q2;
 result->q4 = q1->q1*q2->q4 + q1->q2*q2->q3 - q1->q3*q2->q2 + q1->q4*q2->q1;
}

// Convert quaternion to Direction Cosine Matrix
void quaternion_to_dcm(Quaternion *q, Matrix3x3 *dcm) {
 double q1 = q->q1, q2 = q->q2, q3 = q->q3, q4 = q->q4;
 double q1_2 = q1*q1, q2_2 = q2*q2, q3_2 = q3*q3, q4_2 = q4*q4;

 // First row
 dcm->m[0][0] = q1_2 + q2_2 - q3_2 - q4_2;
 dcm->m[0][1] = 2.0 * (q2*q3 - q1*q4);
 dcm->m[0][2] = 2.0 * (q2*q4 + q1*q3);

 // Second row
 dcm->m[1][0] = 2.0 * (q2*q3 + q1*q4);
 dcm->m[1][1] = q1_2 - q2_2 + q3_2 - q4_2;
 dcm->m[1][2] = 2.0 * (q3*q4 - q1*q2);

 // Third row
 dcm->m[2][0] = 2.0 * (q2*q4 - q1*q3);
 dcm->m[2][1] = 2.0 * (q3*q4 + q1*q2);
 dcm->m[2][2] = q1_2 - q2_2 - q3_2 + q4_2;
}

// Normalize quaternion to unit length
void quaternion_normalize(Quaternion *q) {
 double norm = sqrt(q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3 + q->q4*q->q4);

 if (norm < 1e-10) {
 // Handle near-zero case
 q->q1 = 1.0;
 q->q2 = q->q3 = q->q4 = 0.0;
 return;
 }

 q->q1 /= norm;
 q->q2 /= norm;
 q->q3 /= norm;
 q->q4 /= norm;
}

// Matrix multiplication: C = A * B (all STATE_DIM x STATE_DIM)
void matrix_multiply(LargeMatrix *A, LargeMatrix *B, LargeMatrix *C) {
 int i, j, k;

 // Initialize result matrix to zero
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 C->data[i][j] = 0.0;
 }
 }

 // Perform matrix multiplication
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 for (k = 0; k < STATE_DIM; k++) {
 C->data[i][j] += A->data[i][k] * B->data[k][j];
 }
 }
 }
}

// Matrix transpose: AT = A^T
void matrix_transpose(LargeMatrix *A, LargeMatrix *AT) {
 int i, j;

 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 AT->data[j][i] = A->data[i][j];
 }
 }
}

// Matrix inverse using Gauss-Jordan elimination with regularization
// This is a computationally intensive operation!
void matrix_inverse(LargeMatrix *A, LargeMatrix *A_inv) {
 int i, j, k;
 double temp;
 AugmentedMatrix augmented;

 // Add small regularization term to diagonal to avoid singularity
 LargeMatrix A_reg;
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 A_reg.data[i][j] = A->data[i][j];
 if (i == j) {
 A_reg.data[i][j] += 1e-10; // Small regularization term
 }
 }
 }

 // Create augmented matrix [A|I]
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < 2 * STATE_DIM; j++) {
 if (j < STATE_DIM) {
 augmented.data[i][j] = A_reg.data[i][j];
 } else {
 augmented.data[i][j] = (i == (j - STATE_DIM)) ? 1.0 : 0.0;
 }
 }
 }

 // Gauss-Jordan elimination
 for (i = 0; i < STATE_DIM; i++) {
 // Find pivot
 int pivot_row = i;
 double max_val = fabs(augmented.data[i][i]);

 // Find the best pivot (row with largest absolute value in column i)
 for (k = i + 1; k < STATE_DIM; k++) {
 if (fabs(augmented.data[k][i]) > max_val) {
 max_val = fabs(augmented.data[k][i]);
 pivot_row = k;
 }
 }

 // Swap rows if better pivot found
 if (pivot_row != i) {
 for (j = 0; j < 2 * STATE_DIM; j++) {
 temp = augmented.data[i][j];
 augmented.data[i][j] = augmented.data[pivot_row][j];
 augmented.data[pivot_row][j] = temp;
 }
 }

 // Get pivot value
 temp = augmented.data[i][i];

 // Check for near-zero pivot
 if (fabs(temp) < 1e-10) {
 // Further regularization if pivot is too small
 temp = (temp >= 0) ? 1e-10 : -1e-10;
 }

 // Scale pivot row
 for (j = 0; j < 2 * STATE_DIM; j++) {
 augmented.data[i][j] /= temp;
 }

 // Eliminate other rows
 for (k = 0; k < STATE_DIM; k++) {
 if (k != i) {
 temp = augmented.data[k][i];
 for (j = 0; j < 2 * STATE_DIM; j++) {
 augmented.data[k][j] -= temp * augmented.data[i][j];
 }
 }
 }
 }

 // Extract inverse matrix
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 A_inv->data[i][j] = augmented.data[i][j + STATE_DIM];

 // Check for NaN and replace with zeros
 if (isnan(A_inv->data[i][j])) {
 A_inv->data[i][j] = 0.0;
 }
 }
 }
}

// Matrix-vector multiplication: v_out = A * v_in
void matrix_vector_multiply(LargeMatrix *A, double *v_in, double *v_out) {
 int i, j;

 for (i = 0; i < STATE_DIM; i++) {
 v_out[i] = 0.0;
 for (j = 0; j < STATE_DIM; j++) {
 v_out[i] += A->data[i][j] * v_in[j];
 }
 }
}

// Extended Kalman Filter for attitude determination
void attitude_determination(SensorData *sensor_data, SpacecraftState *state) {
 int i, j;
 double dt = sensor_data->timestamp - state->timestamp;

 // State transition matrix (12x12 for crewed-spacecraft-class state)
 LargeMatrix F, F_transpose, P, Q, K, H, H_transpose, R, I, temp1, temp2, temp3;

 // Make identity matrix for I
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 I.data[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Initialize matrices to prevent undefined behavior
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 F.data[i][j] = 0.0;
 F_transpose.data[i][j] = 0.0;
 P.data[i][j] = 0.0;
 Q.data[i][j] = 0.0;
 K.data[i][j] = 0.0;
 H.data[i][j] = 0.0;
 H_transpose.data[i][j] = 0.0;
 R.data[i][j] = 0.0;
 temp1.data[i][j] = 0.0;
 temp2.data[i][j] = 0.0;
 temp3.data[i][j] = 0.0;
 }
 }

 // Build state transition matrix F for linearized attitude dynamics
 // State = [δθ(3), ω(3), b_gyro(3), δα_thrust(3)]
 // δθ_dot = ω - b_gyro => F[0:3][3:6] = dt*I, F[0:3][6:9] = -dt*I
 // ω_dot ≈ J^{-1}(τ) => F[3:6][3:6] = I, F[3:6][9:12] = coupling
 // b_dot = 0 (random walk) => F[6:9][6:9] = I
 // δα_dot = 0 (slow drift) => F[9:12][9:12] = I
 for (i = 0; i < STATE_DIM; i++) {
 F.data[i][i] = 1.0; // Identity on diagonal
 }
 // Attitude error driven by angular velocity
 F.data[0][3] = dt; F.data[1][4] = dt; F.data[2][5] = dt;
 // Attitude error driven by negative gyro bias
 F.data[0][6] = -dt; F.data[1][7] = -dt; F.data[2][8] = -dt;
 // Add gyroscopic coupling in angular velocity block (Euler's equation)
 double wx = state->angular_vel.x, wy = state->angular_vel.y, wz = state->angular_vel.z;
 F.data[3][4] = wz * dt; F.data[3][5] = -wy * dt;
 F.data[4][3] = -wz * dt; F.data[4][5] = wx * dt;
 F.data[5][3] = wy * dt; F.data[5][4] = -wx * dt;

 // Thruster misalignment coupling to angular velocity:
 // Misaligned thruster torque perturbs body rates
 F.data[3][9] = 0.01 * dt;
 F.data[4][10] = 0.01 * dt;
 F.data[5][11] = 0.01 * dt;

 // Process noise covariance matrix Q
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 if (i == j) {
 Q.data[i][j] = 0.001 * (1 + 0.1 * i); // Increasing noise with state index
 } else {
 Q.data[i][j] = 0.0;
 }
 }
 }
 // Override: thruster misalignment has slow drift (thermal/mechanical)
 Q.data[9][9] = 1e-6;
 Q.data[10][10] = 1e-6;
 Q.data[11][11] = 1e-6;

 // Observation matrix H: measurements = H * state (9 meas, 12 states)
 // Gyro (y[0..2]) observes angular_velocity + gyro_bias: H_gyro = [0 I I 0]
 // Magnetometer (y[3..5]) observes attitude error: H_mag = [I 0 0 0]
 // Sun sensor (y[6..8]) observes attitude error: H_sun = [I 0 0 0]
 // Rows 9-11: unused (no direct thruster misalignment measurement)
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 H.data[i][j] = 0.0;
 }
 }
 // Gyro rows: observe omega + bias
 H.data[0][3] = 1.0; H.data[0][6] = 1.0; // gyro_x = ω_x + b_x
 H.data[1][4] = 1.0; H.data[1][7] = 1.0; // gyro_y = ω_y + b_y
 H.data[2][5] = 1.0; H.data[2][8] = 1.0; // gyro_z = ω_z + b_z
 // Magnetometer rows: observe attitude error
 H.data[3][0] = 1.0; // mag_x ~ δθ_x
 H.data[4][1] = 1.0; // mag_y ~ δθ_y
 H.data[5][2] = 1.0; // mag_z ~ δθ_z
 // Sun sensor rows: observe attitude error
 H.data[6][0] = 1.0; // sun_x ~ δθ_x
 H.data[7][1] = 1.0; // sun_y ~ δθ_y
 H.data[8][2] = 1.0; // sun_z ~ δθ_z

 // Measurement noise covariance R (diagonal, sensor-specific)
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 R.data[i][j] = 0.0;
 }
 }
 R.data[0][0] = 0.01; R.data[1][1] = 0.01; R.data[2][2] = 0.01; // Gyro noise (rad/s)²
 R.data[3][3] = 0.05; R.data[4][4] = 0.05; R.data[5][5] = 0.05; // Magnetometer noise
 R.data[6][6] = 0.02; R.data[7][7] = 0.02; R.data[8][8] = 0.02; // Sun sensor noise
 // No direct measurement of thruster misalignment — large noise = unobservable
 R.data[9][9] = 1e6;
 R.data[10][10] = 1e6;
 R.data[11][11] = 1e6;

 // Initial state covariance matrix P (diagonal — no fabricated cross-correlations)
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 P.data[i][j] = 0.0;
 }
 }
 // Attitude error uncertainty: 5° ≈ 0.087 rad → σ² ≈ 0.0076
 P.data[0][0] = 0.008; P.data[1][1] = 0.008; P.data[2][2] = 0.008;
 // Angular velocity uncertainty: 0.1 rad/s
 P.data[3][3] = 0.01; P.data[4][4] = 0.01; P.data[5][5] = 0.01;
 // Gyro bias uncertainty: 0.01 rad/s
 P.data[6][6] = 0.0001; P.data[7][7] = 0.0001; P.data[8][8] = 0.0001;
 // Thruster misalignment initial uncertainty: ±0.5° ≈ 0.0087 rad → σ²≈7.6e-5
 P.data[9][9] = 7.6e-5;
 P.data[10][10] = 7.6e-5;
 P.data[11][11] = 7.6e-5;

 // Time update (prediction)
 // x = F*x
 double x_pred[STATE_DIM];
 matrix_vector_multiply(&F, state->state_vector, x_pred);

 // P = F*P*F' + Q
 matrix_transpose(&F, &F_transpose);
 matrix_multiply(&F, &P, &temp1);
 matrix_multiply(&temp1, &F_transpose, &temp2);

 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 P.data[i][j] = temp2.data[i][j] + Q.data[i][j];
 }
 }

 // Measurement update (correction)
 // K = P*H'*(H*P*H' + R)^-1
 matrix_transpose(&H, &H_transpose);
 matrix_multiply(&P, &H_transpose, &temp1);
 matrix_multiply(&H, &P, &temp2);
 matrix_multiply(&temp2, &H_transpose, &temp3);

 // temp3 = H*P*H' + R
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 temp3.data[i][j] += R.data[i][j];
 }
 }

 // Calculate inverse of (H*P*H' + R)
 matrix_inverse(&temp3, &temp2);

 // K = P*H'*temp2
 matrix_multiply(&temp1, &temp2, &K);

 // Create measurement vector y from sensor data (9 measurements)
 double y[STATE_DIM] = {0};
 // Gyro measurements: angular velocity + bias
 y[0] = sensor_data->gyro.x;
 y[1] = sensor_data->gyro.y;
 y[2] = sensor_data->gyro.z;
 // Magnetometer: attitude-dependent body-frame B-field
 y[3] = sensor_data->magnetometer.x;
 y[4] = sensor_data->magnetometer.y;
 y[5] = sensor_data->magnetometer.z;
 // Sun sensor: attitude-dependent body-frame sun vector
 y[6] = sensor_data->sun_sensor.x;
 y[7] = sensor_data->sun_sensor.y;
 y[8] = sensor_data->sun_sensor.z;

 // Calculate innovation: y - H*x_pred
 double innovation[STATE_DIM];
 double Hx[STATE_DIM];
 matrix_vector_multiply(&H, x_pred, Hx);

 for (i = 0; i < STATE_DIM; i++) {
 innovation[i] = y[i] - Hx[i];
 }

 // Update state: x = x_pred + K*innovation
 double K_innovation[STATE_DIM];
 matrix_vector_multiply(&K, innovation, K_innovation);

 for (i = 0; i < STATE_DIM; i++) {
 state->state_vector[i] = x_pred[i] + K_innovation[i];
 }

 // Update covariance: P = (I - K*H)*P
 matrix_multiply(&K, &H, &temp1);
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 temp1.data[i][j] = I.data[i][j] - temp1.data[i][j];
 }
 }
 matrix_multiply(&temp1, &P, &temp2);

 // Copy temp2 back to P
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 P.data[i][j] = temp2.data[i][j];
 }
 }

 // Apply attitude error [0..2] as small rotation to quaternion (MEKF reset)
 double dtheta_x = state->state_vector[0];
 double dtheta_y = state->state_vector[1];
 double dtheta_z = state->state_vector[2];
 double half_angle = 0.5 * sqrt(dtheta_x*dtheta_x + dtheta_y*dtheta_y + dtheta_z*dtheta_z);
 Quaternion dq;
 if (half_angle > 1e-12) {
 double sinc_ha = sin(half_angle) / half_angle * 0.5;
 dq.q1 = cos(half_angle);
 dq.q2 = dtheta_x * sinc_ha;
 dq.q3 = dtheta_y * sinc_ha;
 dq.q4 = dtheta_z * sinc_ha;
 } else {
 dq.q1 = 1.0;
 dq.q2 = 0.5 * dtheta_x;
 dq.q3 = 0.5 * dtheta_y;
 dq.q4 = 0.5 * dtheta_z;
 }
 Quaternion q_updated;
 quaternion_multiply(&dq, &state->attitude, &q_updated);
 state->attitude = q_updated;

 // Normalize quaternion
 quaternion_normalize(&state->attitude);

 // Reset attitude error to zero after applying to quaternion
 state->state_vector[0] = 0.0;
 state->state_vector[1] = 0.0;
 state->state_vector[2] = 0.0;

 // Extract angular velocity from state
 state->angular_vel.x = state->state_vector[3];
 state->angular_vel.y = state->state_vector[4];
 state->angular_vel.z = state->state_vector[5];

 // Update timestamp
 state->timestamp = sensor_data->timestamp;
}

// Attitude control function — Target Tracking Mode
// The desired quaternion q_target is passed as a dynamic input from the
// guidance subsystem (e.g., nadir-pointing, sun-tracking, or commanded slew).
// This replaces the hardcoded identity quaternion used in early prototypes.
void attitude_control(SpacecraftState *state, ActuatorCommands *commands,
 const Quaternion *q_target) {
 int i, j;

 // Control matrices
 LargeMatrix K_p, K_d, J, J_inv, B, BT_B_inv, temp1, temp2;

 // Desired quaternion from guidance subsystem (dynamic target tracking)
 Quaternion q_desired = *q_target;

 // Desired angular velocity
 Vector3 omega_desired;
 omega_desired.x = 0.0;
 omega_desired.y = 0.0;
 omega_desired.z = 0.0;

 // Quaternion error between current and desired attitude
 Quaternion q_current_inv;
 q_current_inv.q1 = state->attitude.q1;
 q_current_inv.q2 = -state->attitude.q2;
 q_current_inv.q3 = -state->attitude.q3;
 q_current_inv.q4 = -state->attitude.q4;

 Quaternion q_error;
 quaternion_multiply(&q_desired, &q_current_inv, &q_error);

 // Angular velocity error
 Vector3 omega_error;
 omega_error.x = state->angular_vel.x - omega_desired.x;
 omega_error.y = state->angular_vel.y - omega_desired.y;
 omega_error.z = state->angular_vel.z - omega_desired.z;

 // Initialize control gain matrices (diagonal — no fabricated cross-coupling)
 // Proportional gain: attitude error (3) + angular vel (3) + bias (3)
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 K_p.data[i][j] = 0.0;
 }
 }
 K_p.data[0][0] = 0.5; K_p.data[1][1] = 0.5; K_p.data[2][2] = 0.5; // Attitude gain
 K_p.data[3][3] = 0.2; K_p.data[4][4] = 0.2; K_p.data[5][5] = 0.2; // Rate gain
 K_p.data[6][6] = 0.05; K_p.data[7][7] = 0.05; K_p.data[8][8] = 0.05; // Bias gain
 K_p.data[9][9] = 0.02; K_p.data[10][10] = 0.02; K_p.data[11][11] = 0.02; // Misalign gain

 // Derivative gain
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 if (i == j && i < 3) {
 K_d.data[i][j] = 1.0; // Damping for attitude
 } else if (i == j && i < 6) {
 K_d.data[i][j] = 0.5; // Damping for rates
 } else if (i == j) {
 K_d.data[i][j] = 0.1; // Damping for other states
 } else {
 K_d.data[i][j] = 0.0; // No cross-coupling in damping
 }
 }
 }

 // Spacecraft inertia tensor (kg·m²)
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 if (i == j && i < 3) {
 J.data[i][j] = 10.0 + 2.0 * i; // Principal moments of inertia
 } else if (i < 3 && j < 3) {
 J.data[i][j] = 0.5; // Products of inertia
 } else if (i == j) {
 J.data[i][j] = 1.0; // Other states
 } else {
 J.data[i][j] = 0.0;
 }
 }
 }

 // Compute inverse of inertia matrix
 matrix_inverse(&J, &J_inv);

 // Initialize temp matrices for intermediate computations
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 temp1.data[i][j] = 0.0;
 temp2.data[i][j] = 0.0;
 }
 }

 // Control influence matrix (maps control inputs to state changes)
 // Initialize B to zero (LargeMatrix is 12x12 but only 8 actuator columns used)
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < STATE_DIM; j++) {
 B.data[i][j] = 0.0;
 }
 }
 for (i = 0; i < STATE_DIM; i++) {
 for (j = 0; j < ACTUATOR_COUNT; j++) {
 if (i < 3 && j < WHEEL_COUNT) {
 // Wheel influence on angular momentum
 double angle = 2.0 * PI * j / WHEEL_COUNT;
 if (i == 0) B.data[i][j] = cos(angle);
 if (i == 1) B.data[i][j] = sin(angle);
 if (i == 2) B.data[i][j] = 0.2; // Small z-axis component
 } else if (i < 3 && j >= WHEEL_COUNT && j < WHEEL_COUNT + 3) {
 // Magnetic torquer influence
 B.data[i][j] = (i == j-WHEEL_COUNT) ? 1.0 : 0.1;
 } else if (i >= 3 && i < 6 && j >= WHEEL_COUNT + 3) {
 // Thruster influence on velocity
 B.data[i][j] = (i-3 == j-(WHEEL_COUNT+3)) ? 1.0 : 0.0;
 } else {
 B.data[i][j] = 0.0;
 }
 }
 }

 // Compute control allocation matrix
 // First get B^T * B
 LargeMatrix BT;
 matrix_transpose(&B, &BT);
 matrix_multiply(&BT, &B, &temp1);

 // Add regularization to ensure invertibility
 for (i = 0; i < STATE_DIM; i++) {
 if (i < ACTUATOR_COUNT) {
 temp1.data[i][i] += 1e-6; // Small damping term
 }
 }

 // Compute (B^T * B)^-1
 matrix_inverse(&temp1, &BT_B_inv);

 // Compute (B^T * B)^-1 * B^T
 matrix_multiply(&BT_B_inv, &BT, &temp2);

 // Convert quaternion error to vector part for control
 double q_error_vec[STATE_DIM] = {0};
 q_error_vec[0] = q_error.q2; // Vector part x
 q_error_vec[1] = q_error.q3; // Vector part y
 q_error_vec[2] = q_error.q4; // Vector part z

 // If scalar part is negative, negate vector part
 if (q_error.q1 < 0) {
 q_error_vec[0] = -q_error_vec[0];
 q_error_vec[1] = -q_error_vec[1];
 q_error_vec[2] = -q_error_vec[2];
 }

 // Angular velocity error
 q_error_vec[3] = omega_error.x;
 q_error_vec[4] = omega_error.y;
 q_error_vec[5] = omega_error.z;

 // Gyro bias: handled internally by MEKF, zeroed for control law
 for (i = 6; i < 9; i++) {
 q_error_vec[i] = 0.0;
 }
 // Thruster misalignment compensation (from EKF estimate)
 q_error_vec[9] = state->state_vector[9];
 q_error_vec[10] = state->state_vector[10];
 q_error_vec[11] = state->state_vector[11];

 // Compute proportional control term: K_p * q_error_vec
 double u_p[STATE_DIM];
 matrix_vector_multiply(&K_p, q_error_vec, u_p);

 // Compute derivative control term: K_d * omega_error
 double omega_error_vec[STATE_DIM];
 for (i = 0; i < STATE_DIM; i++) omega_error_vec[i] = 0.0;
 omega_error_vec[0] = omega_error.x;
 omega_error_vec[1] = omega_error.y;
 omega_error_vec[2] = omega_error.z;
 double u_d[STATE_DIM];
 matrix_vector_multiply(&K_d, omega_error_vec, u_d);

 // Total control
 double u_total[STATE_DIM];
 for (i = 0; i < STATE_DIM; i++) {
 u_total[i] = u_p[i] + u_d[i];
 }

 // Allocate control to actuators
 double actuator_commands[ACTUATOR_COUNT];
 for (i = 0; i < ACTUATOR_COUNT; i++) {
 actuator_commands[i] = 0.0;
 for (j = 0; j < STATE_DIM; j++) {
 actuator_commands[i] += temp2.data[i][j] * u_total[j];
 }
 }

 // Apply actuator commands
 for (i = 0; i < WHEEL_COUNT; i++) {
 commands->wheel_torque[i] = actuator_commands[i];
 }

 for (i = 0; i < 3; i++) {
 commands->magnetic_moment[i] = actuator_commands[i + WHEEL_COUNT];
 }

 for (i = 0; i < ACTUATOR_COUNT - WHEEL_COUNT - 3; i++) {
 commands->thruster_force[i] = actuator_commands[i + WHEEL_COUNT + 3];
 }
}

// Compute orbital parameters (OrbitalMecha BCE - 2%)
// Computes mean motion, orbital period, true anomaly from Kepler's equation
void compute_orbital_parameters(double time, double *mean_motion, double *orbital_period,
 double *true_anomaly, double *orbital_radius) {
 double a = SEMI_MAJOR_AXIS;
 double mu = MU_EARTH;

 // Mean motion: n = sqrt(mu / a^3)
 double n = sqrt(mu / (a * a * a));
 *mean_motion = n;

 // Orbital period: T = 2*pi / n
 *orbital_period = 2.0 * PI / n;

 // Mean anomaly: M = n * t (mod 2*pi)
 double M = fmod(n * time, 2.0 * PI);

 // Solve Kepler's equation M = E - e*sin(E) iteratively (e ~ 0 for circular)
 double e = 0.001; // Near-circular orbit eccentricity
 double E = M; // Eccentric anomaly initial guess
 int iter;
 for (iter = 0; iter < 10; iter++) {
 E = E - (E - e * sin(E) - M) / (1.0 - e * cos(E));
 }

 // True anomaly from eccentric anomaly
 *true_anomaly = 2.0 * atan2(sqrt(1.0 + e) * sin(E / 2.0),
 sqrt(1.0 - e) * cos(E / 2.0));

 // Orbital radius: r = a * (1 - e*cos(E))
 *orbital_radius = a * (1.0 - e * cos(E));
}

// Compute gravity gradient torque (GravityPoten BCE - 3%)
// T_gg = (3*mu/r^3) * (r_hat x (I * r_hat))
// Dominant environmental disturbance torque for LEO spacecraft
void compute_gravity_gradient_torque(SpacecraftState *state, double orbital_radius,
 Vector3 *torque_gg) {
 double mu = MU_EARTH;
 double r = orbital_radius;
 double r3 = r * r * r;
 double coeff = 3.0 * mu / r3;

 // Nadir direction in body frame (transform from ECI)
 // For LEO, nadir ~ -z in orbital frame; transform to body using attitude
 Vector3 nadir_orbit = {0.0, 0.0, -1.0}; /* Approximate nadir in orbit frame (nadir-pointing mode) */
 Vector3 nadir_body;
 eci_to_body_transform(&state->attitude, &nadir_orbit, &nadir_body);

 // Spacecraft principal moments of inertia (from ADCS inertia tensor)
 double Ix = 10.0; // kg*m^2
 double Iy = 12.0;
 double Iz = 14.0;

 // I * r_hat (inertia tensor times nadir unit vector)
 Vector3 I_rhat;
 I_rhat.x = Ix * nadir_body.x;
 I_rhat.y = Iy * nadir_body.y;
 I_rhat.z = Iz * nadir_body.z;

 // T_gg = coeff * (r_hat x I*r_hat)
 torque_gg->x = coeff * (nadir_body.y * I_rhat.z - nadir_body.z * I_rhat.y);
 torque_gg->y = coeff * (nadir_body.z * I_rhat.x - nadir_body.x * I_rhat.z);
 torque_gg->z = coeff * (nadir_body.x * I_rhat.y - nadir_body.y * I_rhat.x);
}

// Transform vector from ECI frame to body frame (Co-ordinateTrans BCE - 4%)
// Uses quaternion-derived DCM: v_body = C * v_eci
void eci_to_body_transform(Quaternion *attitude, Vector3 *vec_eci, Vector3 *vec_body) {
 // Build DCM from quaternion
 Matrix3x3 dcm;
 quaternion_to_dcm(attitude, &dcm);

 // Apply rotation: v_body = DCM * v_eci
 vec_body->x = dcm.m[0][0] * vec_eci->x + dcm.m[0][1] * vec_eci->y + dcm.m[0][2] * vec_eci->z;
 vec_body->y = dcm.m[1][0] * vec_eci->x + dcm.m[1][1] * vec_eci->y + dcm.m[1][2] * vec_eci->z;
 vec_body->z = dcm.m[2][0] * vec_eci->x + dcm.m[2][1] * vec_eci->y + dcm.m[2][2] * vec_eci->z;


}

// Simulate a spacecraft ADCS control loop
int main() {
 int i;
 SpacecraftState state;
 SensorData sensors;
 ActuatorCommands commands;

 // Timing variables
 clock_t start_time, end_time;
 double total_time = 0.0;
 int num_timesteps = 100; // Number of control cycles to run

 // Orbital parameters
 double mean_motion, orbital_period, true_anomaly, orbital_radius;
 Vector3 gravity_torque;
 Vector3 sun_vec_body, mag_vec_body;

 // Target Tracking mode: dynamic pointing target from guidance subsystem
 Quaternion q_target;

 // Initialize state
 initialize_state(&state);

 // Run control loop simulation
 tm_log(TM_INFO, "iter=%d STATE_DIM=%d (crewed spacecraft class)\n",
 num_timesteps, STATE_DIM);

 for (i = 0; i < num_timesteps; i++) {
 // Generate simulated sensor data
 sensors.timestamp = i * 0.01; // 100 Hz simulation

 // Gyro readings
 sensors.gyro.x = 0.01 * sin(i * 0.02);
 sensors.gyro.y = 0.02 * cos(i * 0.01);
 sensors.gyro.z = 0.005 * sin(i * 0.03);

 // Magnetometer readings
 sensors.magnetometer.x = cos(i * 0.01);
 sensors.magnetometer.y = sin(i * 0.01);
 sensors.magnetometer.z = 0.5 * sin(i * 0.005);

 // Sun sensor readings
 sensors.sun_sensor.x = 0.8 * cos(i * 0.003);
 sensors.sun_sensor.y = 0.8 * sin(i * 0.003);
 sensors.sun_sensor.z = 0.6;

 // Star tracker readings
 sensors.star_tracker.x = 0.01 * i;
 sensors.star_tracker.y = 0.01 * cos(i * 0.1);
 sensors.star_tracker.z = 0.01 * sin(i * 0.1);

 // Start timing
 start_time = clock();

 // Compute orbital parameters (OrbitalMecha BCE)
 compute_orbital_parameters(sensors.timestamp, &mean_motion, &orbital_period,
 &true_anomaly, &orbital_radius);

 // Target Tracking: compute nadir-pointing quaternion from orbital
 // position. In flight, this comes from the guidance subsystem
 // (nadir-pointing / sun-tracking / commanded slew).
 {
 double half_ta = true_anomaly * 0.5;
 q_target.q1 = cos(half_ta);
 q_target.q2 = 0.0;
 q_target.q3 = 0.0;
 q_target.q4 = sin(half_ta);
 quaternion_normalize(&q_target);
 }

 // Run attitude determination
 attitude_determination(&sensors, &state);

 // Transform sensor vectors from ECI to body frame (Co-ordinateTrans BCE)
 Vector3 sun_eci = {cos(true_anomaly), sin(true_anomaly), 0.0};
 eci_to_body_transform(&state.attitude, &sun_eci, &sun_vec_body);
 Vector3 mag_eci = {sensors.magnetometer.x, sensors.magnetometer.y,
 sensors.magnetometer.z};
 eci_to_body_transform(&state.attitude, &mag_eci, &mag_vec_body);

 // Compute gravity gradient disturbance torque (GravityPoten BCE)
 compute_gravity_gradient_torque(&state, orbital_radius, &gravity_torque);

 // Run attitude control (Target Tracking mode)
 attitude_control(&state, &commands, &q_target);

 // End timing
 end_time = clock();

 // Add time for this iteration
 total_time += (double)(end_time - start_time) / CLOCKS_PER_SEC;

 // Telemetry: progress report every 10 steps
 if (i % 10 == 0) {
 tm_log(TM_STEP, "step=%d/%d\n", i, num_timesteps);
 }
 }

 // Telemetry: end-of-run summary
 tm_log(TM_RESULT, "results\n");
 tm_log(TM_RESULT, "time=%.6fs avg=%.6fs/iter\n",
 total_time, total_time / num_timesteps);
 tm_log(TM_RESULT, "100Hz CPU util=%.2f%%\n",
 (total_time / num_timesteps) * 100.0);
 tm_log(TM_RESULT, "attitude q=[%.4f, %.4f, %.4f, %.4f]\n",
 state.attitude.q1, state.attitude.q2, state.attitude.q3, state.attitude.q4);
 tm_log(TM_RESULT, "target q=[%.4f, %.4f, %.4f, %.4f]\n",
 q_target.q1, q_target.q2, q_target.q3, q_target.q4);
 tm_log(TM_RESULT, "state_vector (%d elements):\n", STATE_DIM);
 for (i = 0; i < STATE_DIM; i++) {
 tm_log(TM_RESULT, " state[%d]=%.6f\n", i, state.state_vector[i]);
 }
 tm_log(TM_RESULT, "actuator commands:\n");
 for (i = 0; i < 4; i++) {
 tm_log(TM_RESULT, " wheel_torque[%d]=%.6f\n", i, commands.wheel_torque[i]);
 }
 for (i = 0; i < 2; i++) {
 tm_log(TM_RESULT, " magnetic_moment[%d]=%.6f\n", i, commands.magnetic_moment[i]);
 }

 return 0;
}
