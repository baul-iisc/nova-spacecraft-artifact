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
 * Multi-Sensor Fusion for Navigation and Control
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Trig (CORDIC accelerator for trigonometric functions)
 * Application: (see workload description below)
 *
 * Description:
 * Implements multi-sensor fusion using extended Kalman filtering for integrated
 * navigation. This workload fuses gyroscope, accelerometer, star tracker, sun
 * sensor, and GPS data using 18x18 state covariance matrices, coordinate
 * transformations, and state prediction. Standard GNC (Guidance, Navigation and
 * Control) subsystem processing on all spacecraft missions, with enhanced redundancy
 * requirements for human-rated crewed spacecraft.
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
/* Redirect standard trig functions to CORDIC HW accelerator */
#define sin(a) hw_sin(a)
#define cos(a) hw_cos(a)

// Configuration defines
#define STATE_DIM 18 // State: pos(3)+vel(3)+quat(4)+omega(3)+bias(3)+clock(2)
#define MEAS_DIM_STAR 6 // Star tracker measurement dimension
#define MEAS_DIM_GPS 3 // GPS measurement dimension
#define MEAS_DIM_IMU 6 // IMU measurement dimension (3 accel + 3 gyro)
#define NUM_MONTE_CARLO 100 // Number of Monte Carlo runs for statistical analysis
#define VIBRATION_BUF_SIZE 64 // IMU vibration analysis DFT buffer size
#define NUM_FREQ_BINS 32 // Number of DFT frequency bins to compute
#define M_PI_SF 3.14159265358979323846 // Pi constant for DFT/coordinate transforms

// Function prototypes
void initialize_state(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM]);
void propagate_state(double state[STATE_DIM], double dt);
void propagate_covariance(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], double Q[STATE_DIM][STATE_DIM], double dt);
void integrate_star_tracker(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], double measurement[MEAS_DIM_STAR]);
void integrate_gps(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], double measurement[MEAS_DIM_GPS]);
void integrate_imu(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], double measurement[MEAS_DIM_IMU], double dt);

// Generic matrix operations
void matrix_multiply_generic(double *A, double *B, double *C, int m, int n, int p, int lda, int ldb, int ldc);
void matrix_transpose_generic(double *A, double *AT, int m, int n, int lda, int ldat);
void matrix_add_generic(double *A, double *B, double *C, int m, int n, int lda, int ldb, int ldc);
void matrix_subtract_generic(double *A, double *B, double *C, int m, int n, int lda, int ldb, int ldc);
void matrix_inverse_generic(double *A, double *Ainv, int n, int lda, int ldainv);

// Quaternion and vector operations
void quaternion_to_dcm(double q[4], double dcm[3][3]);
void simulate_measurements(double true_state[STATE_DIM], 
 double star_measurement[MEAS_DIM_STAR], 
 double gps_measurement[MEAS_DIM_GPS], 
 double imu_measurement[MEAS_DIM_IMU]);

// FFT BCE - Vibration spectral analysis on IMU accelerometer data
double sensor_vibration_analysis(double accel_samples[VIBRATION_BUF_SIZE], double sample_rate);

// Co-ordinateTrans BCE - ECI/ECEF and body/LVLH coordinate transformations
void eci_to_ecef_transform(double pos_eci[3], double vel_eci[3],
 double pos_ecef[3], double vel_ecef[3],
 double greenwich_angle);
void body_to_lvlh_transform(double vec_body[3], double vec_lvlh[3],
 double q_body_lvlh[4]);

// Main function
int main() {
 // Declare variables
 double state[STATE_DIM] = {0}; // State vector
 double P[STATE_DIM][STATE_DIM] = {0}; // State covariance
 double Q[STATE_DIM][STATE_DIM] = {0}; // Process noise covariance
 double star_measurement[MEAS_DIM_STAR] = {0}; // Star tracker measurement
 double gps_measurement[MEAS_DIM_GPS] = {0}; // GPS measurement
 double imu_measurement[MEAS_DIM_IMU] = {0}; // IMU measurement
 double true_state[STATE_DIM] = {0}; // True state for simulation
 double accel_vibration_buf[VIBRATION_BUF_SIZE] = {0}; // IMU vibration analysis buffer
 int vib_buf_idx = 0; // Circular buffer index
 double pos_ecef[3] = {0}, vel_ecef[3] = {0}; // ECEF frame outputs
 double vec_lvlh[3] = {0}; // LVLH frame output
 
 clock_t start, end;
 double cpu_time_used;
 
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(4517);
 
 // Initialize process noise with physically differentiated values
 // Position uncertainty (km²/s): orbital perturbation uncertainty
 Q[0][0] = Q[1][1] = Q[2][2] = 1e-8;
 // Velocity uncertainty (km²/s³)
 Q[3][3] = Q[4][4] = Q[5][5] = 1e-10;
 // Quaternion (1/s): attitude knowledge uncertainty
 Q[6][6] = Q[7][7] = Q[8][8] = Q[9][9] = 1e-8;
 // Angular rate (rad²/s³): gyro random walk
 Q[10][10] = Q[11][11] = Q[12][12] = 3.5e-7;
 // Accelerometer bias (m²/s⁵)
 Q[13][13] = Q[14][14] = Q[15][15] = 2.4e-6;
 // GPS clock bias, drift
 Q[16][16] = 1e-4; Q[17][17] = 1e-6;
 
 // Start timing
 start = clock();
 
 // Run Monte Carlo simulations for covariance characterization
 for (int mc = 0; mc < NUM_MONTE_CARLO; mc++) {
 // Reset state and covariance
 initialize_state(state, P);
 memcpy(true_state, state, sizeof(double) * STATE_DIM);
 
 // Simulation loop - 100 time steps of 0.1 seconds
 for (int t = 0; t < 100; t++) {
 double dt = 0.1; // Time step in seconds
 
 // Generate simulated measurements
 simulate_measurements(true_state, star_measurement, gps_measurement, imu_measurement);
 
 // Propagate true state (for simulation purposes)
 propagate_state(true_state, dt);
 
 // EKF prediction step
 propagate_state(state, dt);
 propagate_covariance(state, P, Q, dt);
 
 // EKF update steps - different sensors at different rates
 // IMU at 10 Hz (every timestep)
 integrate_imu(state, P, imu_measurement, dt);
 
 if (t % 5 == 0) { // Star tracker at 2 Hz
 integrate_star_tracker(state, P, star_measurement);
 }
 
 if (t % 10 == 0) { // GPS at 1 Hz
 integrate_gps(state, P, gps_measurement);
 }
 
 // FFT BCE: Accumulate accelerometer samples and run vibration analysis
 accel_vibration_buf[vib_buf_idx % VIBRATION_BUF_SIZE] = 
 sqrt(imu_measurement[3]*imu_measurement[3] + 
 imu_measurement[4]*imu_measurement[4] + 
 imu_measurement[5]*imu_measurement[5]);
 vib_buf_idx++;
 if (vib_buf_idx >= VIBRATION_BUF_SIZE && t % VIBRATION_BUF_SIZE == 0) {
 double peak_freq = sensor_vibration_analysis(accel_vibration_buf, 1.0/dt);
 /* Flag structural resonance if peak exceeds reaction wheel bandwidth */
 if (peak_freq > 25.0 && peak_freq < 80.0) {
 FLIGHT_LOG("VIB WARNING: peak %.1f Hz in RWA band\n", peak_freq);
 }
 }
 
 // Co-ordinateTrans BCE: Transform state to ECEF and LVLH frames
 double greenwich_angle = 7.2921159e-5 * (t * dt); // Earth rotation
 eci_to_ecef_transform(&state[0], &state[3], pos_ecef, vel_ecef, greenwich_angle);
 body_to_lvlh_transform(&state[10], vec_lvlh, &state[6]);
 }
 
 // Print progress
 if ((mc + 1) % 10 == 0) {
 FLIGHT_LOG("Completed %d/%d Monte Carlo runs\n", mc + 1, NUM_MONTE_CARLO);
 }
 }
 
 // End timing
 end = clock();
 cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
 
 FLIGHT_LOG("[FUSE] done (%.3fs)\n", cpu_time_used);
 FLIGHT_LOG("Average time per Monte Carlo run: %f seconds\n", cpu_time_used / NUM_MONTE_CARLO);
 
 return 0;
}

/**
 * Initialize state vector and covariance matrix
 */
void initialize_state(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM]) {
 // Initialize state components
 // State vector layout:
 // [0-2] Position (x,y,z) in ECI frame [km]
 // [3-5] Velocity (vx,vy,vz) in ECI frame [km/s]
 // [6-9] Attitude quaternion (scalar last) []
 // [10-12] Angular velocity (wx,wy,wz) [rad/s]
 // [13-15] Accelerometer bias [m/s^2]
 // [16-17] GPS clock bias and drift [m, m/s]
 
 // Position (Earth orbit at approximately 500km altitude)
 state[0] = 6878.0; // x [km]
 state[1] = 0.0; // y [km]
 state[2] = 0.0; // z [km]
 
 // Velocity (circular orbit)
 state[3] = 0.0; // vx [km/s]
 state[4] = 7.6; // vy [km/s]
 state[5] = 0.0; // vz [km/s]
 
 // Attitude quaternion (initially aligned with ECI frame)
 state[6] = 0.0; // qx
 state[7] = 0.0; // qy
 state[8] = 0.0; // qz
 state[9] = 1.0; // qw (scalar part)
 
 // Angular velocity (initially zero)
 state[10] = 0.0; // wx [rad/s]
 state[11] = 0.0; // wy [rad/s]
 state[12] = 0.0; // wz [rad/s]
 
 // Sensor biases (initially zero)
 state[13] = 0.0; // accel bias x [m/s^2]
 state[14] = 0.0; // accel bias y [m/s^2]
 state[15] = 0.0; // accel bias z [m/s^2]
 
 // GPS clock states
 state[16] = 0.0; // clock bias [m]
 state[17] = 0.0; // clock drift [m/s]
 
 // Initialize covariance matrix (diagonal)
 memset(P, 0, sizeof(double) * STATE_DIM * STATE_DIM);
 
 // Position uncertainty [km^2]
 P[0][0] = 1.0e-2;
 P[1][1] = 1.0e-2;
 P[2][2] = 1.0e-2;
 
 // Velocity uncertainty [km^2/s^2]
 P[3][3] = 1.0e-4;
 P[4][4] = 1.0e-4;
 P[5][5] = 1.0e-4;
 
 // Attitude uncertainty [rad^2]
 P[6][6] = 1.0e-6;
 P[7][7] = 1.0e-6;
 P[8][8] = 1.0e-6;
 P[9][9] = 1.0e-6;
 
 // Angular velocity uncertainty [rad^2/s^2]
 P[10][10] = 1.0e-8;
 P[11][11] = 1.0e-8;
 P[12][12] = 1.0e-8;
 
 // Accelerometer bias uncertainty [m^2/s^4]
 P[13][13] = 1.0e-4;
 P[14][14] = 1.0e-4;
 P[15][15] = 1.0e-4;
 
 // GPS clock uncertainty
 P[16][16] = 9.0e2; // clock bias [m^2]
 P[17][17] = 9.0e-2; // clock drift [m^2/s^2]
}

/* -----------------------------------------------------------------------
 * RK4 helper: gravitational acceleration with J2 perturbation at a given
 * ECI position. Factored out so RK4 can evaluate the force
 * field at intermediate points (k2, k3 midpoints).
 * ----------------------------------------------------------------------- */
static void compute_gravity_j2(const double pos[3], double a_grav[3])
{
 const double mu = 398600.4418; /* Earth GM [km^3/s^2] */
 const double J2 = 1.08262668e-3; /* Earth J2 coefficient */
 const double Re = 6378.137; /* Earth equatorial R [km] */

 double r = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
 double r3 = r * r * r;
 double J2_t = 1.5 * J2 * Re * Re / (r * r);
 double z2r2 = pos[2] * pos[2] / (r * r);

 a_grav[0] = -mu * pos[0] / r3 * (1.0 - J2_t * (5.0 * z2r2 - 1.0));
 a_grav[1] = -mu * pos[1] / r3 * (1.0 - J2_t * (5.0 * z2r2 - 1.0));
 a_grav[2] = -mu * pos[2] / r3 * (1.0 - J2_t * (5.0 * z2r2 - 3.0));
}

/* -----------------------------------------------------------------------
 * RK4 helper: quaternion kinematic derivative
 * q_dot = 0.5 * Omega(omega) * q
 * omega is the body angular-velocity vector, assumed constant over the
 * integration step (gyro sample-and-hold model).
 * ----------------------------------------------------------------------- */
static void compute_qdot(const double q[4], const double omega[3],
 double qdot[4])
{
 qdot[0] = 0.5 * ( omega[0]*q[3] + omega[1]*q[2] - omega[2]*q[1]);
 qdot[1] = 0.5 * (-omega[0]*q[2] + omega[1]*q[3] + omega[2]*q[0]);
 qdot[2] = 0.5 * ( omega[0]*q[1] - omega[1]*q[0] + omega[2]*q[3]);
 qdot[3] = 0.5 * (-omega[0]*q[0] - omega[1]*q[1] - omega[2]*q[2]);
}

/**
 * Propagate state using orbital dynamics and quaternion kinematics.
 *
 * Integration method: classical 4th-order Runge-Kutta (RK4).
 * Euler integration was replaced per GNC flight-software practice
 * (crewed spacecraft and deep-space applications), where RK4 is the baseline integrator for
 * energy conservation and orbital stability over extended mission durations.
 *
 * For the translational (pos, vel) sub-state the ODE is
 * d(pos)/dt = vel, d(vel)/dt = a_grav(pos)
 * with a_grav evaluated via compute_gravity_j2() at each RK4 stage.
 *
 * For the quaternion sub-state the ODE is
 * d(q)/dt = 0.5 * Omega(omega) * q
 * with omega held constant over the step (gyro sample-and-hold).
 *
 * GPS clock states are linear and integrated analytically (RK4 is exact
 * for linear ODEs but the closed-form expression is clearer).
 */
void propagate_state(double state[STATE_DIM], double dt) {
 double pos[3], vel[3], q[4], omega[3], accel_bias[3], gps_clock[2];
 int i;

 /* --- extract state components ---------------------------------- */
 memcpy(pos, &state[0], 3 * sizeof(double));
 memcpy(vel, &state[3], 3 * sizeof(double));
 memcpy(q, &state[6], 4 * sizeof(double));
 memcpy(omega, &state[10], 3 * sizeof(double));
 memcpy(accel_bias, &state[13], 3 * sizeof(double));
 memcpy(gps_clock, &state[16], 2 * sizeof(double));

 /* ===============================================================
 * RK4 — translational dynamics (pos, vel)
 * State derivative: d(pos)/dt = vel, d(vel)/dt = a_grav(pos)
 * =============================================================== */
 double k1p[3], k1v[3], k2p[3], k2v[3];
 double k3p[3], k3v[3], k4p[3], k4v[3];
 double ag[3], tp[3], tv[3];

 /* ---- stage k1 ---- */
 compute_gravity_j2(pos, ag);
 for (i = 0; i < 3; i++) { k1p[i] = vel[i] * dt; k1v[i] = ag[i] * dt; }

 /* ---- stage k2 (midpoint using k1) ---- */
 for (i = 0; i < 3; i++) { tp[i] = pos[i] + 0.5 * k1p[i];
 tv[i] = vel[i] + 0.5 * k1v[i]; }
 compute_gravity_j2(tp, ag);
 for (i = 0; i < 3; i++) { k2p[i] = tv[i] * dt; k2v[i] = ag[i] * dt; }

 /* ---- stage k3 (midpoint using k2) ---- */
 for (i = 0; i < 3; i++) { tp[i] = pos[i] + 0.5 * k2p[i];
 tv[i] = vel[i] + 0.5 * k2v[i]; }
 compute_gravity_j2(tp, ag);
 for (i = 0; i < 3; i++) { k3p[i] = tv[i] * dt; k3v[i] = ag[i] * dt; }

 /* ---- stage k4 (full step using k3) ---- */
 for (i = 0; i < 3; i++) { tp[i] = pos[i] + k3p[i];
 tv[i] = vel[i] + k3v[i]; }
 compute_gravity_j2(tp, ag);
 for (i = 0; i < 3; i++) { k4p[i] = tv[i] * dt; k4v[i] = ag[i] * dt; }

 /* ---- weighted combination ---- */
 double new_pos[3], new_vel[3];
 for (i = 0; i < 3; i++) {
 new_pos[i] = pos[i] + (k1p[i] + 2.0*k2p[i] + 2.0*k3p[i] + k4p[i]) / 6.0;
 new_vel[i] = vel[i] + (k1v[i] + 2.0*k2v[i] + 2.0*k3v[i] + k4v[i]) / 6.0;
 }

 /* ===============================================================
 * RK4 — quaternion kinematics
 * d(q)/dt = 0.5 * Omega(omega) * q (omega constant over step)
 * =============================================================== */
 double k1q[4], k2q[4], k3q[4], k4q[4], tq[4], dq[4];

 /* ---- stage k1 ---- */
 compute_qdot(q, omega, dq);
 for (i = 0; i < 4; i++) k1q[i] = dq[i] * dt;

 /* ---- stage k2 ---- */
 for (i = 0; i < 4; i++) tq[i] = q[i] + 0.5 * k1q[i];
 compute_qdot(tq, omega, dq);
 for (i = 0; i < 4; i++) k2q[i] = dq[i] * dt;

 /* ---- stage k3 ---- */
 for (i = 0; i < 4; i++) tq[i] = q[i] + 0.5 * k2q[i];
 compute_qdot(tq, omega, dq);
 for (i = 0; i < 4; i++) k3q[i] = dq[i] * dt;

 /* ---- stage k4 ---- */
 for (i = 0; i < 4; i++) tq[i] = q[i] + k3q[i];
 compute_qdot(tq, omega, dq);
 for (i = 0; i < 4; i++) k4q[i] = dq[i] * dt;

 /* ---- weighted combination ---- */
 double new_q[4];
 for (i = 0; i < 4; i++)
 new_q[i] = q[i] + (k1q[i] + 2.0*k2q[i] + 2.0*k3q[i] + k4q[i]) / 6.0;

 /* Normalize quaternion to unit length */
 double q_norm = sqrt(new_q[0]*new_q[0] + new_q[1]*new_q[1] +
 new_q[2]*new_q[2] + new_q[3]*new_q[3]);
 for (i = 0; i < 4; i++) new_q[i] /= q_norm;

 /* GPS clock states (linear ODE — analytical solution) */
 double new_gps_clock[2];
 new_gps_clock[0] = gps_clock[0] + gps_clock[1] * dt; /* bias += drift*dt */
 new_gps_clock[1] = gps_clock[1]; /* drift is constant */

 /* --- write updated state back ---------------------------------- */
 memcpy(&state[0], new_pos, 3 * sizeof(double));
 memcpy(&state[3], new_vel, 3 * sizeof(double));
 memcpy(&state[6], new_q, 4 * sizeof(double));
 /* Angular velocity and accelerometer biases held constant */
 memcpy(&state[16], new_gps_clock, 2 * sizeof(double));
}

/**
 * Propagate state covariance using linearized system dynamics
 */
void propagate_covariance(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], 
 double Q[STATE_DIM][STATE_DIM], double dt) {
 double F[STATE_DIM][STATE_DIM] = {0}; // State transition matrix
 double FT[STATE_DIM][STATE_DIM] = {0}; // F transpose
 double temp[STATE_DIM][STATE_DIM] = {0};
 double new_P[STATE_DIM][STATE_DIM] = {0};
 
 // Extract state components
 double pos[3], q[4];
 memcpy(pos, &state[0], 3 * sizeof(double));
 memcpy(q, &state[6], 4 * sizeof(double));
 
 // Compute gravitational gradient
 double mu = 398600.4418; // Earth gravitational parameter [km^3/s^2]
 double r = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
 double r3 = r*r*r;
 double r5 = r3*r*r;
 
 // Fill in state transition matrix F
 
 // Position derivative is velocity
 F[0][3] = 1.0;
 F[1][4] = 1.0;
 F[2][5] = 1.0;
 
 // Velocity derivative includes gravity gradient
 double gravity_grad[3][3] = {0};
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 if (i == j) {
 gravity_grad[i][j] = -mu/r3 + 3.0*mu*pos[i]*pos[j]/r5;
 } else {
 gravity_grad[i][j] = 3.0*mu*pos[i]*pos[j]/r5;
 }
 }
 }
 
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 F[i+3][j] = gravity_grad[i][j];
 }
 }
 
 // Quaternion derivative depends on angular velocity
 // These are complex expressions based on quaternion kinematics
 double omega[3];
 memcpy(omega, &state[10], 3 * sizeof(double));
 
 // Quaternion derivative matrix
 double q_deriv[4][3] = {
 { 0.5*q[3], 0.5*q[2], -0.5*q[1]},
 {-0.5*q[2], 0.5*q[3], 0.5*q[0]},
 { 0.5*q[1], -0.5*q[0], 0.5*q[3]},
 {-0.5*q[0], -0.5*q[1], -0.5*q[2]}
 };
 
 for (int i = 0; i < 4; i++) {
 for (int j = 0; j < 3; j++) {
 F[i+6][j+10] = q_deriv[i][j];
 }
 }
 
 // GPS clock bias derivative is clock drift
 F[16][17] = 1.0;
 
 // Scale continuous-time Jacobian by dt for first-order discretization
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 F[i][j] *= dt;
 }
 }
 // Then add identity
 for (int i = 0; i < STATE_DIM; i++) {
 F[i][i] += 1.0;
 }
 
 // Compute F*P*F' + Q
 matrix_transpose_generic(&F[0][0], &FT[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 matrix_multiply_generic(&F[0][0], &P[0][0], &temp[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 matrix_multiply_generic(&temp[0][0], &FT[0][0], &new_P[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 matrix_add_generic(&new_P[0][0], &Q[0][0], &new_P[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 
 // Copy result back to P
 memcpy(P, new_P, sizeof(double) * STATE_DIM * STATE_DIM);
}

/**
 * Integrate star tracker measurement using EKF update step
 */
void integrate_star_tracker(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], 
 double measurement[MEAS_DIM_STAR]) {
 double H[MEAS_DIM_STAR][STATE_DIM] = {0}; // Measurement matrix
 double HT[STATE_DIM][MEAS_DIM_STAR] = {0}; // H transpose
 double R[MEAS_DIM_STAR][MEAS_DIM_STAR] = {0}; // Measurement noise
 double K[STATE_DIM][MEAS_DIM_STAR] = {0}; // Kalman gain
 double S[MEAS_DIM_STAR][MEAS_DIM_STAR] = {0}; // Innovation covariance
 double S_inv[MEAS_DIM_STAR][MEAS_DIM_STAR] = {0}; // Inverse of S
 double HP[MEAS_DIM_STAR][STATE_DIM] = {0}; // H*P
 double I_KH[STATE_DIM][STATE_DIM] = {0}; // I - K*H
 double temp1[STATE_DIM][STATE_DIM] = {0}; // Temporary matrix
 double HPH[MEAS_DIM_STAR][MEAS_DIM_STAR] = {0}; // H*P*H'
 double PHT[STATE_DIM][MEAS_DIM_STAR] = {0}; // P*H'
 
 // Extract attitude quaternion
 double q[4];
 memcpy(q, &state[6], 4 * sizeof(double));
 
 // Expected measurement (quaternion to rotation matrix conversion)
 double dcm[3][3];
 quaternion_to_dcm(q, dcm);
 
 // Measurement model: star tracker gives us attitude information (rotation matrix)
 // We represent it as 3 rows of the rotation matrix (6 values)
 double expected_measurement[MEAS_DIM_STAR];
 expected_measurement[0] = dcm[0][0];
 expected_measurement[1] = dcm[0][1];
 expected_measurement[2] = dcm[1][0];
 expected_measurement[3] = dcm[1][1];
 expected_measurement[4] = dcm[2][0];
 expected_measurement[5] = dcm[2][1];
 
 // Measurement Jacobian (H matrix)
 // This is a complex matrix that relates how the quaternion components
 // affect the DCM elements we're measuring
 // Analytical Jacobian: ∂dcm_element/∂q for each measured DCM element
 // dcm[0][0] = 1 - 2(qy² + qz²)
 H[0][6] = 0.0; H[0][7] = -4.0*q[1]; H[0][8] = -4.0*q[2]; H[0][9] = 0.0;
 // dcm[0][1] = 2(qx*qy - qz*qw)
 H[1][6] = 2.0*q[1]; H[1][7] = 2.0*q[0]; H[1][8] = -2.0*q[3]; H[1][9] = -2.0*q[2];
 // dcm[1][0] = 2(qx*qy + qz*qw)
 H[2][6] = 2.0*q[1]; H[2][7] = 2.0*q[0]; H[2][8] = 2.0*q[3]; H[2][9] = 2.0*q[2];
 // dcm[1][1] = 1 - 2(qx² + qz²)
 H[3][6] = -4.0*q[0]; H[3][7] = 0.0; H[3][8] = -4.0*q[2]; H[3][9] = 0.0;
 // dcm[2][0] = 2(qx*qz - qy*qw)
 H[4][6] = 2.0*q[2]; H[4][7] = -2.0*q[3]; H[4][8] = 2.0*q[0]; H[4][9] = -2.0*q[1];
 // dcm[2][1] = 2(qy*qz + qx*qw)
 H[5][6] = 2.0*q[3]; H[5][7] = 2.0*q[2]; H[5][8] = 2.0*q[1]; H[5][9] = 2.0*q[0];
 
 // Measurement noise (constant in this implementation)
 for (int i = 0; i < MEAS_DIM_STAR; i++) {
 R[i][i] = 1e-5;
 }
 
 // Compute innovation (measurement - expected_measurement)
 double innovation[MEAS_DIM_STAR];
 for (int i = 0; i < MEAS_DIM_STAR; i++) {
 innovation[i] = measurement[i] - expected_measurement[i];
 }
 
 // Compute innovation covariance S = H*P*H' + R
 matrix_transpose_generic(&H[0][0], &HT[0][0], MEAS_DIM_STAR, STATE_DIM, STATE_DIM, MEAS_DIM_STAR);
 matrix_multiply_generic(&H[0][0], &P[0][0], &HP[0][0], MEAS_DIM_STAR, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 matrix_multiply_generic(&HP[0][0], &HT[0][0], &HPH[0][0], MEAS_DIM_STAR, STATE_DIM, MEAS_DIM_STAR, STATE_DIM, MEAS_DIM_STAR, MEAS_DIM_STAR);
 matrix_add_generic(&HPH[0][0], &R[0][0], &S[0][0], MEAS_DIM_STAR, MEAS_DIM_STAR, MEAS_DIM_STAR, MEAS_DIM_STAR, MEAS_DIM_STAR);
 
 // Compute S^-1 (invert innovation covariance)
 matrix_inverse_generic(&S[0][0], &S_inv[0][0], MEAS_DIM_STAR, MEAS_DIM_STAR, MEAS_DIM_STAR);
 
 // Compute Kalman gain K = P*H'*S^-1
 matrix_multiply_generic(&P[0][0], &HT[0][0], &PHT[0][0], STATE_DIM, STATE_DIM, MEAS_DIM_STAR, STATE_DIM, MEAS_DIM_STAR, MEAS_DIM_STAR);
 matrix_multiply_generic(&PHT[0][0], &S_inv[0][0], &K[0][0], STATE_DIM, MEAS_DIM_STAR, MEAS_DIM_STAR, MEAS_DIM_STAR, MEAS_DIM_STAR, MEAS_DIM_STAR);
 
 // Update state: state = state + K*innovation
 double K_innov[STATE_DIM] = {0};
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < MEAS_DIM_STAR; j++) {
 K_innov[i] += K[i][j] * innovation[j];
 }
 state[i] += K_innov[i];
 }
 
 // Normalize quaternion after update
 double q_norm = sqrt(state[6]*state[6] + state[7]*state[7] + 
 state[8]*state[8] + state[9]*state[9]);
 state[6] /= q_norm;
 state[7] /= q_norm;
 state[8] /= q_norm;
 state[9] /= q_norm;
 
 // Update covariance: P = (I - K*H)*P
 // First, compute I - K*H
 for (int i = 0; i < STATE_DIM; i++) {
 I_KH[i][i] = 1.0; // Start with identity matrix
 }
 
 double KH[STATE_DIM][STATE_DIM] = {0};
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 for (int k = 0; k < MEAS_DIM_STAR; k++) {
 KH[i][j] += K[i][k] * H[k][j];
 }
 }
 }
 
 matrix_subtract_generic(&I_KH[0][0], &KH[0][0], &I_KH[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 
 // Then compute (I - K*H)*P
 matrix_multiply_generic(&I_KH[0][0], &P[0][0], &temp1[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 memcpy(P, temp1, sizeof(double) * STATE_DIM * STATE_DIM);
}

/**
 * Integrate GPS measurement using EKF update step
 */
void integrate_gps(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], 
 double measurement[MEAS_DIM_GPS]) {
 double H[MEAS_DIM_GPS][STATE_DIM] = {0}; // Measurement matrix
 double HT[STATE_DIM][MEAS_DIM_GPS] = {0}; // H transpose
 double R[MEAS_DIM_GPS][MEAS_DIM_GPS] = {0}; // Measurement noise
 double K[STATE_DIM][MEAS_DIM_GPS] = {0}; // Kalman gain
 double S[MEAS_DIM_GPS][MEAS_DIM_GPS] = {0}; // Innovation covariance
 double S_inv[MEAS_DIM_GPS][MEAS_DIM_GPS] = {0}; // Inverse of S
 double HP[MEAS_DIM_GPS][STATE_DIM] = {0}; // H*P
 double I_KH[STATE_DIM][STATE_DIM] = {0}; // I - K*H
 double temp1[STATE_DIM][STATE_DIM] = {0}; // Temporary matrix
 double HPH[MEAS_DIM_GPS][MEAS_DIM_GPS] = {0}; // H*P*H'
 double PHT[STATE_DIM][MEAS_DIM_GPS] = {0}; // P*H'
 
 // Extract position
 double pos[3];
 memcpy(pos, &state[0], 3 * sizeof(double));
 
 // Expected measurement (position in ECEF)
 double expected_measurement[MEAS_DIM_GPS];
 memcpy(expected_measurement, pos, sizeof(double) * MEAS_DIM_GPS);
 
 // Account for GPS clock bias (in position units - meters)
 expected_measurement[0] += state[16] / 1000.0; // Convert from m to km
 expected_measurement[1] += state[16] / 1000.0; // Convert from m to km
 expected_measurement[2] += state[16] / 1000.0; // Convert from m to km
 
 // Measurement Jacobian (H matrix)
 // GPS directly measures position plus clock bias
 H[0][0] = 1.0; // x position
 H[1][1] = 1.0; // y position
 H[2][2] = 1.0; // z position
 H[0][16] = 1.0 / 1000.0; // clock bias (m to km)
 H[1][16] = 1.0 / 1000.0; // clock bias (m to km)
 H[2][16] = 1.0 / 1000.0; // clock bias (m to km)
 
 // Measurement noise (constant in this implementation)
 for (int i = 0; i < MEAS_DIM_GPS; i++) {
 R[i][i] = 1e-2; // 100m position accuracy
 }
 
 // Compute innovation (measurement - expected_measurement)
 double innovation[MEAS_DIM_GPS];
 for (int i = 0; i < MEAS_DIM_GPS; i++) {
 innovation[i] = measurement[i] - expected_measurement[i];
 }
 
 // Compute innovation covariance S = H*P*H' + R
 matrix_transpose_generic(&H[0][0], &HT[0][0], MEAS_DIM_GPS, STATE_DIM, STATE_DIM, MEAS_DIM_GPS);
 matrix_multiply_generic(&H[0][0], &P[0][0], &HP[0][0], MEAS_DIM_GPS, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 matrix_multiply_generic(&HP[0][0], &HT[0][0], &HPH[0][0], MEAS_DIM_GPS, STATE_DIM, MEAS_DIM_GPS, STATE_DIM, MEAS_DIM_GPS, MEAS_DIM_GPS);
 matrix_add_generic(&HPH[0][0], &R[0][0], &S[0][0], MEAS_DIM_GPS, MEAS_DIM_GPS, MEAS_DIM_GPS, MEAS_DIM_GPS, MEAS_DIM_GPS);
 
 // Compute S^-1 (invert innovation covariance)
 matrix_inverse_generic(&S[0][0], &S_inv[0][0], MEAS_DIM_GPS, MEAS_DIM_GPS, MEAS_DIM_GPS);
 
 // Compute Kalman gain K = P*H'*S^-1
 matrix_multiply_generic(&P[0][0], &HT[0][0], &PHT[0][0], STATE_DIM, STATE_DIM, MEAS_DIM_GPS, STATE_DIM, MEAS_DIM_GPS, MEAS_DIM_GPS);
 matrix_multiply_generic(&PHT[0][0], &S_inv[0][0], &K[0][0], STATE_DIM, MEAS_DIM_GPS, MEAS_DIM_GPS, MEAS_DIM_GPS, MEAS_DIM_GPS, MEAS_DIM_GPS);
 
 // Update state: state = state + K*innovation
 double K_innov[STATE_DIM] = {0};
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < MEAS_DIM_GPS; j++) {
 K_innov[i] += K[i][j] * innovation[j];
 }
 state[i] += K_innov[i];
 }
 
 // Update covariance: P = (I - K*H)*P
 // First, compute I - K*H
 for (int i = 0; i < STATE_DIM; i++) {
 I_KH[i][i] = 1.0; // Start with identity matrix
 }
 
 double KH[STATE_DIM][STATE_DIM] = {0};
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 for (int k = 0; k < MEAS_DIM_GPS; k++) {
 KH[i][j] += K[i][k] * H[k][j];
 }
 }
 }
 
 matrix_subtract_generic(&I_KH[0][0], &KH[0][0], &I_KH[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 
 // Then compute (I - K*H)*P
 matrix_multiply_generic(&I_KH[0][0], &P[0][0], &temp1[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 memcpy(P, temp1, sizeof(double) * STATE_DIM * STATE_DIM);
}

/**
 * Integrate IMU measurement using EKF update step
 */
void integrate_imu(double state[STATE_DIM], double P[STATE_DIM][STATE_DIM], 
 double measurement[MEAS_DIM_IMU], double dt) {
 (void)dt;
 double H[MEAS_DIM_IMU][STATE_DIM] = {0}; // Measurement matrix
 double HT[STATE_DIM][MEAS_DIM_IMU] = {0}; // H transpose
 double R[MEAS_DIM_IMU][MEAS_DIM_IMU] = {0}; // Measurement noise
 double K[STATE_DIM][MEAS_DIM_IMU] = {0}; // Kalman gain
 double S[MEAS_DIM_IMU][MEAS_DIM_IMU] = {0}; // Innovation covariance
 double S_inv[MEAS_DIM_IMU][MEAS_DIM_IMU] = {0}; // Inverse of S
 double HP[MEAS_DIM_IMU][STATE_DIM] = {0}; // H*P
 double I_KH[STATE_DIM][STATE_DIM] = {0}; // I - K*H
 double temp1[STATE_DIM][STATE_DIM] = {0}; // Temporary matrix
 double HPH[MEAS_DIM_IMU][MEAS_DIM_IMU] = {0}; // H*P*H'
 double PHT[STATE_DIM][MEAS_DIM_IMU] = {0}; // P*H'
 
 // Extract quaternion, angular velocity, and accel biases
 double q[4], omega[3], accel_bias[3];
 memcpy(q, &state[6], 4 * sizeof(double));
 memcpy(omega, &state[10], 3 * sizeof(double));
 memcpy(accel_bias, &state[13], 3 * sizeof(double));
 
 // Convert quaternion to DCM (Direction Cosine Matrix)
 double dcm[3][3];
 quaternion_to_dcm(q, dcm);
 
 // Expected measurement
 double expected_measurement[MEAS_DIM_IMU];
 
 // For gyro - just the angular velocity
 expected_measurement[0] = omega[0];
 expected_measurement[1] = omega[1];
 expected_measurement[2] = omega[2];
 
 // For accelerometer - account for gravity in body frame and bias
 // Compute gravity magnitude from orbital position
 double pos_r[3] = {state[0], state[1], state[2]};
 double r_mag = sqrt(pos_r[0]*pos_r[0] + pos_r[1]*pos_r[1] + pos_r[2]*pos_r[2]);
 double mu_earth = 398600.4418; // km^3/s^2
 double g_inertial[3];
 if (r_mag > 1.0) {
 double r3 = r_mag * r_mag * r_mag;
 for (int idx = 0; idx < 3; idx++) {
 g_inertial[idx] = -mu_earth * pos_r[idx] / r3; // km/s^2, radial towards Earth
 }
 } else {
 // Near-surface fallback
 g_inertial[0] = 0.0;
 g_inertial[1] = 0.0;
 g_inertial[2] = -0.00981; // 9.81 m/s^2 in km/s^2
 }
 
 // Convert gravity from inertial frame to body frame using DCM transpose
 double gravity_body[3] = {0};
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 gravity_body[i] += dcm[j][i] * g_inertial[j];
 }
 }
 
 // Accelerometer measures negative of gravity plus bias
 expected_measurement[3] = -gravity_body[0] + accel_bias[0];
 expected_measurement[4] = -gravity_body[1] + accel_bias[1];
 expected_measurement[5] = -gravity_body[2] + accel_bias[2];
 
 // Measurement Jacobian (H matrix)
 // For gyro part - directly measures angular velocity
 H[0][10] = 1.0; // wx
 H[1][11] = 1.0; // wy
 H[2][12] = 1.0; // wz
 
 // For accelerometer part - complex relationship with attitude and bias
 // Direct bias measurement update
 H[3][13] = 1.0; // accel bias x
 H[4][14] = 1.0; // accel bias y
 H[5][15] = 1.0; // accel bias z
 
 // Some attitude impact on accelerometer readings
 // Computed from quaternion derivative kinematics
 double g_mag = mu_earth / (r_mag * r_mag); // gravity magnitude at altitude km/s^2
 H[3][6] = 2.0 * g_mag; // ∂(DCM^T * g)/∂qx
 H[4][7] = 2.0 * g_mag; // ∂(DCM^T * g)/∂qy
 H[5][8] = 2.0 * g_mag; // ∂(DCM^T * g)/∂qz
 
 // Measurement noise (constant in this implementation)
 for (int i = 0; i < 3; i++) {
 R[i][i] = 1e-4; // gyro noise
 }
 for (int i = 3; i < 6; i++) {
 R[i][i] = 1e-3; // accelerometer noise
 }
 
 // Compute innovation (measurement - expected_measurement)
 double innovation[MEAS_DIM_IMU];
 for (int i = 0; i < MEAS_DIM_IMU; i++) {
 innovation[i] = measurement[i] - expected_measurement[i];
 }
 
 // Compute innovation covariance S = H*P*H' + R
 matrix_transpose_generic(&H[0][0], &HT[0][0], MEAS_DIM_IMU, STATE_DIM, STATE_DIM, MEAS_DIM_IMU);
 matrix_multiply_generic(&H[0][0], &P[0][0], &HP[0][0], MEAS_DIM_IMU, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 matrix_multiply_generic(&HP[0][0], &HT[0][0], &HPH[0][0], MEAS_DIM_IMU, STATE_DIM, MEAS_DIM_IMU, STATE_DIM, MEAS_DIM_IMU, MEAS_DIM_IMU);
 matrix_add_generic(&HPH[0][0], &R[0][0], &S[0][0], MEAS_DIM_IMU, MEAS_DIM_IMU, MEAS_DIM_IMU, MEAS_DIM_IMU, MEAS_DIM_IMU);
 
 // Compute S^-1 (invert innovation covariance)
 matrix_inverse_generic(&S[0][0], &S_inv[0][0], MEAS_DIM_IMU, MEAS_DIM_IMU, MEAS_DIM_IMU);
 
 // Compute Kalman gain K = P*H'*S^-1
 matrix_multiply_generic(&P[0][0], &HT[0][0], &PHT[0][0], STATE_DIM, STATE_DIM, MEAS_DIM_IMU, STATE_DIM, MEAS_DIM_IMU, MEAS_DIM_IMU);
 matrix_multiply_generic(&PHT[0][0], &S_inv[0][0], &K[0][0], STATE_DIM, MEAS_DIM_IMU, MEAS_DIM_IMU, MEAS_DIM_IMU, MEAS_DIM_IMU, MEAS_DIM_IMU);
 
 // Update state: state = state + K*innovation
 double K_innov[STATE_DIM] = {0};
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < MEAS_DIM_IMU; j++) {
 K_innov[i] += K[i][j] * innovation[j];
 }
 state[i] += K_innov[i];
 }
 
 // Normalize quaternion after update
 double q_norm = sqrt(state[6]*state[6] + state[7]*state[7] + 
 state[8]*state[8] + state[9]*state[9]);
 state[6] /= q_norm;
 state[7] /= q_norm;
 state[8] /= q_norm;
 state[9] /= q_norm;
 
 // Update covariance: P = (I - K*H)*P
 // First, compute I - K*H
 for (int i = 0; i < STATE_DIM; i++) {
 I_KH[i][i] = 1.0; // Start with identity matrix
 }
 
 double KH[STATE_DIM][STATE_DIM] = {0};
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 for (int k = 0; k < MEAS_DIM_IMU; k++) {
 KH[i][j] += K[i][k] * H[k][j];
 }
 }
 }
 
 matrix_subtract_generic(&I_KH[0][0], &KH[0][0], &I_KH[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 
 // Then compute (I - K*H)*P
 matrix_multiply_generic(&I_KH[0][0], &P[0][0], &temp1[0][0], STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);
 memcpy(P, temp1, sizeof(double) * STATE_DIM * STATE_DIM);
}

/**
 * Generic matrix multiplication: C = A * B
 * Uses leading dimensions for proper striding
 */
void matrix_multiply_generic(double *A, double *B, double *C, 
 int m, int n, int p, 
 int lda, int ldb, int ldc) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < p; j++) {
 C[i*ldc + j] = 0.0;
 for (int k = 0; k < n; k++) {
 C[i*ldc + j] += A[i*lda + k] * B[k*ldb + j];
 }
 }
 }
}

/**
 * Generic matrix transpose: AT = A'
 * Uses leading dimensions for proper striding
 */
void matrix_transpose_generic(double *A, double *AT, int m, int n, int lda, int ldat) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < n; j++) {
 AT[j*ldat + i] = A[i*lda + j];
 }
 }
}

/**
 * Generic matrix addition: C = A + B
 * Uses leading dimensions for proper striding
 */
void matrix_add_generic(double *A, double *B, double *C, int m, int n, int lda, int ldb, int ldc) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < n; j++) {
 C[i*ldc + j] = A[i*lda + j] + B[i*ldb + j];
 }
 }
}

/**
 * Generic matrix subtraction: C = A - B
 * Uses leading dimensions for proper striding
 */
void matrix_subtract_generic(double *A, double *B, double *C, int m, int n, int lda, int ldb, int ldc) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < n; j++) {
 C[i*ldc + j] = A[i*lda + j] - B[i*ldb + j];
 }
 }
}

/**
 * Generic matrix inversion using Gaussian elimination
 * Uses leading dimensions for proper striding
 */
void matrix_inverse_generic(double *A, double *Ainv, int n, int lda, int ldainv) {
 // Allocate temporary storage for the augmented matrix [A|I]
 double *temp = (double *)flight_malloc_impl(n * (2*n) * sizeof(double));
 if (temp == NULL) {
 FLIGHT_LOG("Memory allocation failed in matrix_inverse_generic\n");
 return;
 }
 
 // Create augmented matrix [A|I]
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 temp[i*(2*n) + j] = A[i*lda + j];
 }
 // Zero the right half (identity portion) before setting diagonals
 for (int j = n; j < 2*n; j++) {
 temp[i*(2*n) + j] = 0.0;
 }
 temp[i*(2*n) + i+n] = 1.0;
 }
 
 // Gaussian elimination
 for (int i = 0; i < n; i++) {
 // Find pivot
 double max_val = fabs(temp[i*(2*n) + i]);
 int max_row = i;
 for (int j = i+1; j < n; j++) {
 if (fabs(temp[j*(2*n) + i]) > max_val) {
 max_val = fabs(temp[j*(2*n) + i]);
 max_row = j;
 }
 }
 
 // Swap rows if needed
 if (max_row != i) {
 for (int j = 0; j < 2*n; j++) {
 double swap = temp[i*(2*n) + j];
 temp[i*(2*n) + j] = temp[max_row*(2*n) + j];
 temp[max_row*(2*n) + j] = swap;
 }
 }
 
 // Scale pivot row
 double pivot = temp[i*(2*n) + i];
 if (fabs(pivot) < 1e-10) {
 // Matrix is singular or nearly singular
 FLIGHT_LOG("Warning: Matrix is nearly singular in matrix_inverse_generic\n");
 pivot = (pivot < 0) ? -1e-10 : 1e-10;
 }
 
 for (int j = 0; j < 2*n; j++) {
 temp[i*(2*n) + j] /= pivot;
 }
 
 // Eliminate other rows
 for (int j = 0; j < n; j++) {
 if (j != i) {
 double factor = temp[j*(2*n) + i];
 for (int k = 0; k < 2*n; k++) {
 temp[j*(2*n) + k] -= factor * temp[i*(2*n) + k];
 }
 }
 }
 }
 
 // Extract inverse matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 Ainv[i*ldainv + j] = temp[i*(2*n) + j+n];
 }
 }
 
 // Free temporary storage
 flight_free_impl(temp);
}

/**
 * Convert quaternion to Direction Cosine Matrix (DCM)
 */
void quaternion_to_dcm(double q[4], double dcm[3][3]) {
 // q = [qx, qy, qz, qw] - scalar last format
 double qx = q[0];
 double qy = q[1];
 double qz = q[2];
 double qw = q[3];
 
 // Compute DCM elements
 dcm[0][0] = 1.0 - 2.0 * (qy*qy + qz*qz);
 dcm[0][1] = 2.0 * (qx*qy - qz*qw);
 dcm[0][2] = 2.0 * (qx*qz + qy*qw);
 
 dcm[1][0] = 2.0 * (qx*qy + qz*qw);
 dcm[1][1] = 1.0 - 2.0 * (qx*qx + qz*qz);
 dcm[1][2] = 2.0 * (qy*qz - qx*qw);
 
 dcm[2][0] = 2.0 * (qx*qz - qy*qw);
 dcm[2][1] = 2.0 * (qy*qz + qx*qw);
 dcm[2][2] = 1.0 - 2.0 * (qx*qx + qy*qy);
}

/**
 * FFT BCE: Analyze IMU accelerometer vibration spectrum using direct DFT.
 * Detects vibration modes from thruster firings, docking events, or mechanical
 * resonances. Computes DFT magnitude for first NUM_FREQ_BINS frequency bins
 * on a small circular buffer of accelerometer magnitude samples.
 * Returns the peak vibration frequency in Hz.
 */
double sensor_vibration_analysis(double accel_samples[VIBRATION_BUF_SIZE], double sample_rate) {
 double dft_magnitude[NUM_FREQ_BINS];
 double peak_magnitude = 0.0;
 int peak_bin = 0;
 
 // Compute DFT magnitude for each frequency bin (O(N^2) direct DFT)
 for (int k = 0; k < NUM_FREQ_BINS; k++) {
 double real_sum = 0.0;
 double imag_sum = 0.0;
 
 for (int n = 0; n < VIBRATION_BUF_SIZE; n++) {
 double angle = 2.0 * M_PI_SF * k * n / VIBRATION_BUF_SIZE;
 real_sum += accel_samples[n] * cos(angle);
 imag_sum -= accel_samples[n] * sin(angle);
 }
 
 // Magnitude = sqrt(Re^2 + Im^2)
 dft_magnitude[k] = sqrt(real_sum * real_sum + imag_sum * imag_sum);
 
 // Skip DC bin (k=0), find peak vibration frequency
 if (k > 0 && dft_magnitude[k] > peak_magnitude) {
 peak_magnitude = dft_magnitude[k];
 peak_bin = k;
 }
 }
 
 // Convert bin index to frequency: f = k * fs / N
 double peak_frequency = (double)peak_bin * sample_rate / VIBRATION_BUF_SIZE;
 return peak_frequency;
}

/**
 * Co-ordinateTrans BCE: Transform position and velocity from ECI (Earth-Centered
 * Inertial) to ECEF (Earth-Centered Earth-Fixed) frame using Greenwich sidereal
 * angle rotation. Core operation in multi-sensor fusion for correlating GPS
 * (ECEF) measurements with inertial navigation state (ECI).
 */
void eci_to_ecef_transform(double pos_eci[3], double vel_eci[3],
 double pos_ecef[3], double vel_ecef[3],
 double greenwich_angle) {
 // Rotation matrix R3(theta) for ECI -> ECEF
 double cos_g = cos(greenwich_angle);
 double sin_g = sin(greenwich_angle);
 double omega_e = 7.2921159e-5; // Earth rotation rate [rad/s]
 
 // R = [cos(g) sin(g) 0]
 // [-sin(g) cos(g) 0]
 // [0 0 1]
 double R[3][3] = {
 { cos_g, sin_g, 0.0},
 {-sin_g, cos_g, 0.0},
 { 0.0, 0.0, 1.0}
 };
 
 // Position: pos_ecef = R * pos_eci
 for (int i = 0; i < 3; i++) {
 pos_ecef[i] = 0.0;
 for (int j = 0; j < 3; j++) {
 pos_ecef[i] += R[i][j] * pos_eci[j];
 }
 }
 
 // Velocity: vel_ecef = R * vel_eci + omega_cross * pos_ecef
 // omega_cross accounts for rotating frame transport term
 for (int i = 0; i < 3; i++) {
 vel_ecef[i] = 0.0;
 for (int j = 0; j < 3; j++) {
 vel_ecef[i] += R[i][j] * vel_eci[j];
 }
 }
 // Add omega x r term (omega = [0, 0, omega_e] in ECEF)
 vel_ecef[0] += omega_e * pos_ecef[1];
 vel_ecef[1] -= omega_e * pos_ecef[0];
}

/**
 * Co-ordinateTrans BCE: Transform a vector from spacecraft body frame to LVLH
 * (Local Vertical Local Horizontal) frame using attitude quaternion.
 * Used to express thruster forces and sensor measurements in the orbital frame.
 */
void body_to_lvlh_transform(double vec_body[3], double vec_lvlh[3],
 double q_body_lvlh[4]) {
 // Convert quaternion to DCM (body -> LVLH rotation matrix)
 double dcm[3][3];
 double qx = q_body_lvlh[0], qy = q_body_lvlh[1];
 double qz = q_body_lvlh[2], qw = q_body_lvlh[3];
 
 dcm[0][0] = 1.0 - 2.0*(qy*qy + qz*qz);
 dcm[0][1] = 2.0*(qx*qy - qz*qw);
 dcm[0][2] = 2.0*(qx*qz + qy*qw);
 dcm[1][0] = 2.0*(qx*qy + qz*qw);
 dcm[1][1] = 1.0 - 2.0*(qx*qx + qz*qz);
 dcm[1][2] = 2.0*(qy*qz - qx*qw);
 dcm[2][0] = 2.0*(qx*qz - qy*qw);
 dcm[2][1] = 2.0*(qy*qz + qx*qw);
 dcm[2][2] = 1.0 - 2.0*(qx*qx + qy*qy);
 
 // vec_lvlh = DCM * vec_body
 for (int i = 0; i < 3; i++) {
 vec_lvlh[i] = 0.0;
 for (int j = 0; j < 3; j++) {
 vec_lvlh[i] += dcm[i][j] * vec_body[j];
 }
 }
}

/**
 * Generate simulated measurements
 */
void simulate_measurements(double true_state[STATE_DIM], 
 double star_measurement[MEAS_DIM_STAR], 
 double gps_measurement[MEAS_DIM_GPS], 
 double imu_measurement[MEAS_DIM_IMU]) {
 // Extract quaternion and position from true state
 double q[4], pos[3], vel[3], omega[3], accel_bias[3];
 memcpy(pos, &true_state[0], 3 * sizeof(double));
 memcpy(vel, &true_state[3], 3 * sizeof(double));
 memcpy(q, &true_state[6], 4 * sizeof(double));
 memcpy(omega, &true_state[10], 3 * sizeof(double));
 memcpy(accel_bias, &true_state[13], 3 * sizeof(double));
 
 // Convert quaternion to DCM
 double dcm[3][3];
 quaternion_to_dcm(q, dcm);
 
 // Create star tracker measurement (parts of DCM with noise)
 star_measurement[0] = dcm[0][0] + (rand() / (double)RAND_MAX - 0.5) * 1e-3;
 star_measurement[1] = dcm[0][1] + (rand() / (double)RAND_MAX - 0.5) * 1e-3;
 star_measurement[2] = dcm[1][0] + (rand() / (double)RAND_MAX - 0.5) * 1e-3;
 star_measurement[3] = dcm[1][1] + (rand() / (double)RAND_MAX - 0.5) * 1e-3;
 star_measurement[4] = dcm[2][0] + (rand() / (double)RAND_MAX - 0.5) * 1e-3;
 star_measurement[5] = dcm[2][1] + (rand() / (double)RAND_MAX - 0.5) * 1e-3;
 
 // Create GPS measurement (position with noise)
 for (int i = 0; i < MEAS_DIM_GPS; i++) {
 gps_measurement[i] = pos[i] + (rand() / (double)RAND_MAX - 0.5) * 1e-1;
 }
 
 // Account for GPS clock bias
 gps_measurement[0] += true_state[16] / 1000.0; // Convert m to km
 gps_measurement[1] += true_state[16] / 1000.0; // Convert m to km
 gps_measurement[2] += true_state[16] / 1000.0; // Convert m to km
 
 // Create IMU measurement
 // Gyro part - angular velocity with noise
 for (int i = 0; i < 3; i++) {
 imu_measurement[i] = omega[i] + (rand() / (double)RAND_MAX - 0.5) * 1e-2;
 }
 
 // Accelerometer part - compute orbital gravity matching filter model
 double mu_sim = 398600.4418; // km^3/s^2
 double r_sim = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
 double r3_sim = r_sim * r_sim * r_sim;
 double g_inertial_sim[3];
 for (int i = 0; i < 3; i++) {
 g_inertial_sim[i] = -mu_sim * pos[i] / r3_sim; // km/s^2
 }
 
 // Transform gravity from inertial to body frame
 double gravity_body[3] = {0};
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 gravity_body[i] += dcm[j][i] * g_inertial_sim[j];
 }
 }
 
 // Calculate acceleration in body frame
 for (int i = 0; i < 3; i++) {
 imu_measurement[i+3] = -gravity_body[i] + accel_bias[i] + 
 (rand() / (double)RAND_MAX - 0.5) * 1e-1;
 }
}
