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
 * Autonomous Precision Guidance Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements autonomous precision guidance using model-predictive trajectory
 * optimization with numerical integration (RK4). This workload computes optimal
 * thrust profiles, state prediction along ballistic arcs, and BLAS-level matrix
 * operations for the guidance law. Representative of launch vehicle upper-stage
 * closed-loop guidance and powered-descent and orbit-insertion maneuver
 * planning processors.
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
#include <stdlib.h> /* rand()/RAND_MAX only — no malloc/free used */
#include <math.h>
#include <string.h>
#include <time.h>
#include "../flight_compliance.h"

// Guidance system state and control dimensions
#define STATE_DIM 15 // State vector dimension
#define CONTROL_DIM 6 // Control vector dimension
#define HORIZON 10 // Prediction horizon
#define TERRAIN_GRID_SIZE 64 /* DEM tile resolution (flight: 128×128 HazDet grid) */
#define TIMESTEPS 50 // Maximum optimization iterations
#define NAV_STATE_DIM 6 // Navigation EKF state dimension (pos[3]+vel[3])
#define NAV_MEAS_DIM 6 // Navigation measurement dimension

// Physical constants
#define GRAVITY 1.625 /* lunar surface gravity (m/s²) for lunar lander PDG */
#define MAX_THRUST 500.0
#define MIN_THRUST 50.0
#define MAX_GIMBAL_ANGLE (15.0 * M_PI / 180.0) // 15 degrees in radians
#define SPACECRAFT_MASS 2500.0
#define TIMESTEP 0.1 // Control timestep (seconds)

/* -----------------------------------------------------------------------
 * Static Memory Pools
 *
 * mission software (launch vehicle/launch vehicle/spacecraft OBC) prohibits dynamic
 * memory allocation after the initialisation phase to prevent heap
 * fragmentation and non-deterministic execution times. All working
 * buffers are pre-allocated in BSS with compile-time-known dimensions.
 * ----------------------------------------------------------------------- */

/* --- compute_pdg working buffers (called once per guidance cycle) ------- */
static double pdg_A[STATE_DIM * STATE_DIM];
static double pdg_B[STATE_DIM * CONTROL_DIM];
static double pdg_Q[STATE_DIM * STATE_DIM];
static double pdg_R[CONTROL_DIM * CONTROL_DIM];
static double pdg_QN[STATE_DIM * STATE_DIM];
static double pdg_x[STATE_DIM * (HORIZON + 1)];
static double pdg_u[CONTROL_DIM * HORIZON];
static double pdg_P[STATE_DIM * STATE_DIM];
static double pdg_K[CONTROL_DIM * STATE_DIM];
static double pdg_Kinv[CONTROL_DIM * CONTROL_DIM];
static double pdg_BT[CONTROL_DIM * STATE_DIM]; /* was loop-local malloc */

/* --- matrix_inverse scratch (max n = STATE_DIM) ----------------------- */
static double inv_augmented[STATE_DIM * (2 * STATE_DIM)];

/* --- main control_sequence buffer ------------------------------------- */
static double ctrl_seq[CONTROL_DIM * HORIZON];

// Terrain and sensor data structures
typedef struct {
 double height[TERRAIN_GRID_SIZE][TERRAIN_GRID_SIZE];
 double slope[TERRAIN_GRID_SIZE][TERRAIN_GRID_SIZE];
 double roughness[TERRAIN_GRID_SIZE][TERRAIN_GRID_SIZE];
} TerrainMap;

typedef struct {
 double position[3]; // x, y, z
 double velocity[3]; // vx, vy, vz
 double attitude[3]; // roll, pitch, yaw
 double angular_rate[3]; // roll rate, pitch rate, yaw rate
 double mass; // Current mass (changes as fuel is consumed)
 double fuel; // Remaining fuel
} SpacecraftState;

typedef struct {
 double thrust; // Main engine thrust
 double gimbal[2]; // Engine gimbal angles (x, y)
 double rcs[3]; // Reaction control system forces
} ControlInput;

// Matrix and vector operations

// Matrix multiplication: C = A * B where A is m x n, B is n x p, C is m x p
void matrix_multiply(double* A, double* B, double* C, int m, int n, int p) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < p; j++) {
 C[i * p + j] = 0.0;
 for (int k = 0; k < n; k++) {
 C[i * p + j] += A[i * n + k] * B[k * p + j];
 }
 }
 }
}

// Matrix-vector multiplication: y = A * x where A is m x n, x is n x 1, y is m x 1
void matrix_vector_multiply(double* A, double* x, double* y, int m, int n) {
 for (int i = 0; i < m; i++) {
 y[i] = 0.0;
 for (int j = 0; j < n; j++) {
 y[i] += A[i * n + j] * x[j];
 }
 }
}

// Matrix transpose: B = A^T where A is m x n, B is n x m
void matrix_transpose(double* A, double* B, int m, int n) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < n; j++) {
 B[j * m + i] = A[i * n + j];
 }
 }
}

// Matrix addition: C = A + B where A, B, C are all m x n
void matrix_add(double* A, double* B, double* C, int m, int n) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < n; j++) {
 C[i * n + j] = A[i * n + j] + B[i * n + j];
 }
 }
}

// Matrix subtraction: C = A - B where A, B, C are all m x n
void matrix_subtract(double* A, double* B, double* C, int m, int n) {
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < n; j++) {
 C[i * n + j] = A[i * n + j] - B[i * n + j];
 }
 }
}

// Matrix inversion for square matrices using Gauss-Jordan elimination
// A is the input matrix, B is the output inverse, n is the dimension
int matrix_inverse(double* A, double* B, int n) {
 /* Use pre-allocated static buffer (sized for max n = STATE_DIM) */
 double* augmented = inv_augmented;
 if (n > STATE_DIM) return 0; /* safety guard */

 // Initialize augmented matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 augmented[i * (2*n) + j] = A[i * n + j];
 }
 for (int j = 0; j < n; j++) {
 augmented[i * (2*n) + n + j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Perform Gauss-Jordan elimination
 for (int i = 0; i < n; i++) {
 // Find pivot
 int pivot = i;
 double max_val = fabs(augmented[i * (2*n) + i]);

 for (int j = i + 1; j < n; j++) {
 double val = fabs(augmented[j * (2*n) + i]);
 if (val > max_val) {
 max_val = val;
 pivot = j;
 }
 }

 // Swap rows if needed
 if (pivot != i) {
 for (int j = 0; j < 2*n; j++) {
 double temp = augmented[i * (2*n) + j];
 augmented[i * (2*n) + j] = augmented[pivot * (2*n) + j];
 augmented[pivot * (2*n) + j] = temp;
 }
 }

 // Check for singularity
 if (fabs(augmented[i * (2*n) + i]) < 1e-10) {
 return 0; // Matrix is singular
 }

 // Scale the pivot row
 double pivot_val = augmented[i * (2*n) + i];
 for (int j = 0; j < 2*n; j++) {
 augmented[i * (2*n) + j] /= pivot_val;
 }

 // Eliminate other rows
 for (int j = 0; j < n; j++) {
 if (j != i) {
 double factor = augmented[j * (2*n) + i];
 for (int k = 0; k < 2*n; k++) {
 augmented[j * (2*n) + k] -= factor * augmented[i * (2*n) + k];
 }
 }
 }
 }

 // Extract the inverse matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 B[i * n + j] = augmented[i * (2*n) + n + j];
 }
 }

 /* Static buffer — no deallocation required */
 return 1; // Success
}

// Rotation matrix calculation based on Euler angles (roll, pitch, yaw)
void rotation_matrix(double roll, double pitch, double yaw, double* R) {
 // Calculate trigonometric values
 double cr = cos(roll);
 double sr = sin(roll);
 double cp = cos(pitch);
 double sp = sin(pitch);
 double cy = cos(yaw);
 double sy = sin(yaw);

 // Create rotation matrix (3x3)
 R[0] = cy * cp;
 R[1] = cy * sp * sr - sy * cr;
 R[2] = cy * sp * cr + sy * sr;

 R[3] = sy * cp;
 R[4] = sy * sp * sr + cy * cr;
 R[5] = sy * sp * cr - cy * sr;

 R[6] = -sp;
 R[7] = cp * sr;
 R[8] = cp * cr;
}

// Generate dynamics matrix A for linearized model around current state
void dynamics_matrix(SpacecraftState* state, double* A) {
 // Initialize to identity matrix
 memset(A, 0, STATE_DIM * STATE_DIM * sizeof(double));

 // Position update from velocity (top-right 3x3 block)
 for (int i = 0; i < 3; i++) {
 A[i * STATE_DIM + i] = 1.0; // Identity for position states
 A[i * STATE_DIM + (i + 3)] = TIMESTEP; // Position updated by velocity
 A[(i + 3) * STATE_DIM + (i + 3)] = 1.0; // Identity for velocity states
 }

 // Attitude update from angular rates (middle-right 3x3 block)
 for (int i = 0; i < 3; i++) {
 A[(i + 6) * STATE_DIM + (i + 6)] = 1.0; // Identity for attitude states
 A[(i + 6) * STATE_DIM + (i + 9)] = TIMESTEP; // Attitude updated by angular rates
 A[(i + 9) * STATE_DIM + (i + 9)] = 1.0; // Identity for angular rate states
 }

 // Mass update
 A[12 * STATE_DIM + 12] = 1.0; // Mass state

 // Fuel update
 A[13 * STATE_DIM + 13] = 1.0; // Fuel state

 // Cross-coupling terms due to orientation and gravity
 double roll = state->attitude[0];
 double pitch = state->attitude[1];

 // Calculate rotation matrix derivatives and include gravity effects
 double cp = cos(pitch);
 double sp = sin(pitch);
 double sr = sin(roll);
 double cr = cos(roll);

 // Gravity effects on velocity (z-component)
 A[5 * STATE_DIM + 7] = GRAVITY * TIMESTEP * cp * cr; // dv_z/d(pitch)
 A[5 * STATE_DIM + 6] = -GRAVITY * TIMESTEP * sp * sr; // dv_z/d(roll)

 // Cross-coupling between angular rates and velocities (Coriolis)
 A[3 * STATE_DIM + 10] = TIMESTEP * state->velocity[2]; // dv_x/d(omega_y)
 A[3 * STATE_DIM + 11] = -TIMESTEP * state->velocity[1]; // dv_x/d(omega_z)

 A[4 * STATE_DIM + 9] = -TIMESTEP * state->velocity[2]; // dv_y/d(omega_x)
 A[4 * STATE_DIM + 11] = TIMESTEP * state->velocity[0]; // dv_y/d(omega_z)

 A[5 * STATE_DIM + 9] = TIMESTEP * state->velocity[1]; // dv_z/d(omega_x)
 A[5 * STATE_DIM + 10] = -TIMESTEP * state->velocity[0]; // dv_z/d(omega_y)

 // Time derivative and mass dependence
 A[14 * STATE_DIM + 14] = 1.0; // Time state
}

// Generate control matrix B for linearized model
void control_matrix(SpacecraftState* state, double* B) {
 // Initialize to zeros
 memset(B, 0, STATE_DIM * CONTROL_DIM * sizeof(double));

 double mass = state->mass;
 double roll = state->attitude[0];
 double pitch = state->attitude[1];
 double yaw = state->attitude[2];

 // Create temporary 3x3 rotation matrix
 double R[9];
 rotation_matrix(roll, pitch, yaw, R);

 // Main engine thrust affects velocity (scaled by orientation and mass)
 // B[3:6, 0] = R * [0, 0, 1]^T * timestep / mass
 B[3 * CONTROL_DIM + 0] = R[2] * TIMESTEP / mass; // x velocity from main thrust
 B[4 * CONTROL_DIM + 0] = R[5] * TIMESTEP / mass; // y velocity from main thrust
 B[5 * CONTROL_DIM + 0] = R[8] * TIMESTEP / mass; // z velocity from main thrust

 // Gimbal angles affect angular rates
 double gimbal_effect = 0.15 * MAX_THRUST / 1200.0; /* MOI about primary axis [kg·m²] */
 B[9 * CONTROL_DIM + 1] = gimbal_effect * TIMESTEP; // x gimbal -> roll rate
 B[10 * CONTROL_DIM + 2] = gimbal_effect * TIMESTEP; // y gimbal -> pitch rate

 // RCS thrusters affect angular rates
 double rcs_effect = 0.5 / 1200.0; /* RCS torque per unit / MOI [kg·m²] */
 B[9 * CONTROL_DIM + 3] = rcs_effect * TIMESTEP; // x RCS -> roll rate
 B[10 * CONTROL_DIM + 4] = rcs_effect * TIMESTEP; // y RCS -> pitch rate
 B[11 * CONTROL_DIM + 5] = rcs_effect * TIMESTEP; // z RCS -> yaw rate

 // Thrust affects mass and fuel consumption
 double fuel_consumption_rate = 3.4e-4; // kg/s per N (bipropellant Isp ~300 s)
 B[12 * CONTROL_DIM + 0] = -fuel_consumption_rate * TIMESTEP; // Mass decrease from thrust
 B[13 * CONTROL_DIM + 0] = -fuel_consumption_rate * TIMESTEP; // Fuel decrease from thrust
}

// --------------------------------------------------------------------------
// Nonlinear dynamics for RK4 numerical integration (NumInte BCE component)
// State: [x,y,z, vx,vy,vz, roll,pitch,yaw, wx,wy,wz, mass, fuel, time]
// Control: [thrust, gimbal_x, gimbal_y, rcs_x, rcs_y, rcs_z]
// --------------------------------------------------------------------------
void guidance_dynamics(double* state, double* control, double* state_dot) {
 double vx = state[3], vy = state[4], vz = state[5];
 double roll = state[6], pitch = state[7], yaw = state[8];
 double wx = state[9], wy = state[10], wz = state[11];
 double mass = state[12];

 double thrust = control[0];
 double gimbal_x = control[1], gimbal_y = control[2];
 double rcs_x = control[3], rcs_y = control[4], rcs_z = control[5];

 // Rotation matrix (body -> inertial)
 double R9[9];
 rotation_matrix(roll, pitch, yaw, R9);

 // Thrust vector in body frame (along z-axis with gimbal deflection)
 double tb[3];
 tb[0] = thrust * sin(gimbal_x);
 tb[1] = thrust * sin(gimbal_y);
 tb[2] = thrust * cos(gimbal_x) * cos(gimbal_y);

 // Transform thrust to inertial frame
 double ti[3];
 ti[0] = R9[0]*tb[0] + R9[1]*tb[1] + R9[2]*tb[2];
 ti[1] = R9[3]*tb[0] + R9[4]*tb[1] + R9[5]*tb[2];
 ti[2] = R9[6]*tb[0] + R9[7]*tb[1] + R9[8]*tb[2];

 double inv_mass = (mass > 10.0) ? 1.0 / mass : 0.1;

 // Position derivatives = velocity
 state_dot[0] = vx;
 state_dot[1] = vy;
 state_dot[2] = vz;

 // Velocity derivatives: thrust + gravity (inertial frame, no Coriolis)
 state_dot[3] = ti[0] * inv_mass;
 state_dot[4] = ti[1] * inv_mass;
 state_dot[5] = ti[2] * inv_mass - GRAVITY;

 // Attitude kinematics (Euler angle rates from body angular rates)
 double cp = cos(pitch), sp = sin(pitch);
 double sr = sin(roll), cr = cos(roll);
 double tp = (fabs(cp) > 1e-6) ? sp / cp : 0.0;
 double secp = (fabs(cp) > 1e-6) ? 1.0 / cp : 1.0;

 state_dot[6] = wx + (wy * sr + wz * cr) * tp; // roll rate
 state_dot[7] = wy * cr - wz * sr; // pitch rate
 state_dot[8] = (wy * sr + wz * cr) * secp; // yaw rate

 // Angular acceleration (rigid-body Euler equations)
 double Ix = 1200.0, Iy = 1200.0, Iz = 800.0;
 double gtorque = 0.15 * thrust;
 state_dot[9] = (gtorque * gimbal_x + rcs_x * 0.5 + (Iy - Iz) * wy * wz) / Ix;
 state_dot[10] = (gtorque * gimbal_y + rcs_y * 0.5 + (Iz - Ix) * wx * wz) / Iy;
 state_dot[11] = (rcs_z * 0.5 + (Ix - Iy) * wx * wy) / Iz;

 // Mass & fuel derivative (fuel consumption proportional to thrust)
 double fuel_rate = 0.05 * thrust;
 state_dot[12] = -fuel_rate;
 state_dot[13] = -fuel_rate;

 // Time derivative
 state_dot[14] = 1.0;
}

// Fourth-order Runge-Kutta integration step (NumInte BCE)
void rk4_step(double* state, double* control, double dt, double* state_next) {
 double k1[STATE_DIM], k2[STATE_DIM], k3[STATE_DIM], k4[STATE_DIM];
 double tmp[STATE_DIM];
 int i;

 // k1 = f(state, control)
 guidance_dynamics(state, control, k1);

 // k2 = f(state + dt/2 * k1, control)
 for (i = 0; i < STATE_DIM; i++)
 tmp[i] = state[i] + 0.5 * dt * k1[i];
 guidance_dynamics(tmp, control, k2);

 // k3 = f(state + dt/2 * k2, control)
 for (i = 0; i < STATE_DIM; i++)
 tmp[i] = state[i] + 0.5 * dt * k2[i];
 guidance_dynamics(tmp, control, k3);

 // k4 = f(state + dt * k3, control)
 for (i = 0; i < STATE_DIM; i++)
 tmp[i] = state[i] + dt * k3[i];
 guidance_dynamics(tmp, control, k4);

 // state_next = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
 for (i = 0; i < STATE_DIM; i++)
 state_next[i] = state[i] + (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
}

// Model predictive control cost matrices setup
void setup_cost_matrices(double* Q, double* R) {
 // State cost matrix Q (STATE_DIM x STATE_DIM)
 memset(Q, 0, STATE_DIM * STATE_DIM * sizeof(double));
 // Heavily penalize position errors
 Q[0 * STATE_DIM + 0] = 100.0; // x position
 Q[1 * STATE_DIM + 1] = 100.0; // y position
 Q[2 * STATE_DIM + 2] = 200.0; // z position (altitude most important)

 // Moderately penalize velocity errors
 Q[3 * STATE_DIM + 3] = 50.0; // x velocity
 Q[4 * STATE_DIM + 4] = 50.0; // y velocity
 Q[5 * STATE_DIM + 5] = 80.0; // z velocity

 // Lightly penalize attitude errors
 Q[6 * STATE_DIM + 6] = 10.0; // roll
 Q[7 * STATE_DIM + 7] = 10.0; // pitch
 Q[8 * STATE_DIM + 8] = 10.0; // yaw

 // Very lightly penalize angular rates
 Q[9 * STATE_DIM + 9] = 5.0; // roll rate
 Q[10 * STATE_DIM + 10] = 5.0; // pitch rate
 Q[11 * STATE_DIM + 11] = 5.0; // yaw rate

 // Fuel conservation
 Q[13 * STATE_DIM + 13] = 1.0; // Fuel state

 // Control cost matrix R (CONTROL_DIM x CONTROL_DIM)
 memset(R, 0, CONTROL_DIM * CONTROL_DIM * sizeof(double));
 // Penalize excessive thrust
 R[0 * CONTROL_DIM + 0] = 0.1; // Main thrust

 // Heavily penalize gimbal and RCS usage
 R[1 * CONTROL_DIM + 1] = 1.0; // X gimbal
 R[2 * CONTROL_DIM + 2] = 1.0; // Y gimbal
 R[3 * CONTROL_DIM + 3] = 0.5; // X RCS
 R[4 * CONTROL_DIM + 4] = 0.5; // Y RCS
 R[5 * CONTROL_DIM + 5] = 0.5; // Z RCS
}

// Terminal cost matrix (higher weights for landing conditions)
void setup_terminal_cost_matrix(double* QN) {
 // Terminal state cost matrix QN (STATE_DIM x STATE_DIM)
 memset(QN, 0, STATE_DIM * STATE_DIM * sizeof(double));

 // Very heavily penalize final position errors
 QN[0 * STATE_DIM + 0] = 500.0; // x position
 QN[1 * STATE_DIM + 1] = 500.0; // y position
 QN[2 * STATE_DIM + 2] = 1000.0; // z position (altitude most important)

 // Very heavily penalize final velocity (must be near zero)
 QN[3 * STATE_DIM + 3] = 800.0; // x velocity
 QN[4 * STATE_DIM + 4] = 800.0; // y velocity
 QN[5 * STATE_DIM + 5] = 1000.0; // z velocity

 // Moderately penalize final attitude (should be upright)
 QN[6 * STATE_DIM + 6] = 100.0; // roll
 QN[7 * STATE_DIM + 7] = 100.0; // pitch
 QN[8 * STATE_DIM + 8] = 50.0; // yaw

 // Lightly penalize final angular rates (should be zero)
 QN[9 * STATE_DIM + 9] = 50.0; // roll rate
 QN[10 * STATE_DIM + 10] = 50.0; // pitch rate
 QN[11 * STATE_DIM + 11] = 50.0; // yaw rate
}

// Terrain evaluation function
double evaluate_landing_site(TerrainMap* terrain, double x, double y) {
 // Convert position to grid coordinates
 int grid_x = (int)((x + 100.0) / 200.0 * (TERRAIN_GRID_SIZE - 1));
 int grid_y = (int)((y + 100.0) / 200.0 * (TERRAIN_GRID_SIZE - 1));

 // Clamp to grid boundaries
 if (grid_x < 0) grid_x = 0;
 if (grid_x >= TERRAIN_GRID_SIZE) grid_x = TERRAIN_GRID_SIZE - 1;
 if (grid_y < 0) grid_y = 0;
 if (grid_y >= TERRAIN_GRID_SIZE) grid_y = TERRAIN_GRID_SIZE - 1;

 // Create cost based on terrain properties
 double height_cost = terrain->height[grid_x][grid_y] * 10.0;
 double slope_cost = terrain->slope[grid_x][grid_y] * 100.0;
 double roughness_cost = terrain->roughness[grid_x][grid_y] * 50.0;

 return height_cost + slope_cost + roughness_cost;
}

// Calculate optimal control sequence using Powered Descent Guidance (PDG)
void compute_pdg(SpacecraftState* current_state, double* target_state,
 TerrainMap* terrain, double* control_sequence) {
 /* All working buffers are pre-allocated in static BSS pools.
 * No dynamic memory allocation occurs inside this function. */
 double* A = pdg_A;
 double* B = pdg_B;
 double* Q = pdg_Q;
 double* R = pdg_R;
 double* QN = pdg_QN;
 double* x = pdg_x;
 double* u = pdg_u;
 double* P = pdg_P;
 double* K = pdg_K;
 double* Kinv = pdg_Kinv;

 // Initialize current state
 for (int i = 0; i < 3; i++) {
 x[i] = current_state->position[i];
 x[i + 3] = current_state->velocity[i];
 x[i + 6] = current_state->attitude[i];
 x[i + 9] = current_state->angular_rate[i];
 }
 x[12] = current_state->mass;
 x[13] = current_state->fuel;
 x[14] = 0.0; // Initial time

 // Initialize control inputs with zeros
 memset(u, 0, CONTROL_DIM * HORIZON * sizeof(double));

 // Setup cost matrices
 setup_cost_matrices(Q, R);
 setup_terminal_cost_matrix(QN);

 // Iterative optimization with trajectory refinement
 for (int iter = 0; iter < TIMESTEPS; iter++) {
 // Update system matrices based on current trajectory
 SpacecraftState iter_state = *current_state;

 // Forward simulation with current control sequence
 for (int k = 0; k < HORIZON; k++) {
 // Update dynamics and control matrices for current state
 dynamics_matrix(&iter_state, A);
 control_matrix(&iter_state, B);

 // Get control inputs for this step
 double uk[CONTROL_DIM];
 for (int i = 0; i < CONTROL_DIM; i++) {
 uk[i] = u[k * CONTROL_DIM + i];
 }

 // Convert state to vector form
 double xk[STATE_DIM];
 for (int i = 0; i < 3; i++) {
 xk[i] = iter_state.position[i];
 xk[i + 3] = iter_state.velocity[i];
 xk[i + 6] = iter_state.attitude[i];
 xk[i + 9] = iter_state.angular_rate[i];
 }
 xk[12] = iter_state.mass;
 xk[13] = iter_state.fuel;
 xk[14] = k * TIMESTEP;

 // Predict next state using RK4 numerical integration
 double x_next[STATE_DIM];
 rk4_step(xk, uk, TIMESTEP, x_next);

 // Update state for next iteration
 for (int i = 0; i < 3; i++) {
 iter_state.position[i] = x_next[i];
 iter_state.velocity[i] = x_next[i + 3];
 iter_state.attitude[i] = x_next[i + 6];
 iter_state.angular_rate[i] = x_next[i + 9];
 }
 iter_state.mass = x_next[12];
 iter_state.fuel = x_next[13];

 // Store state in trajectory
 for (int i = 0; i < STATE_DIM; i++) {
 x[(k + 1) * STATE_DIM + i] = x_next[i];
 }
 }

 // Evaluate terrain cost along predicted trajectory (TerrainEva BCE)
 double total_terrain_cost = 0.0;
 for (int k = 0; k < HORIZON; k++) {
 double xk_pos = x[(k + 1) * STATE_DIM + 0];
 double yk_pos = x[(k + 1) * STATE_DIM + 1];
 total_terrain_cost += evaluate_landing_site(terrain, xk_pos, yk_pos);
 }
 // Augment position cost weights to steer away from rough terrain
 double terrain_weight = total_terrain_cost / (HORIZON * 100.0);
 Q[0 * STATE_DIM + 0] = terrain_weight; // penalise x deviation near rough terrain
 Q[1 * STATE_DIM + 1] = terrain_weight; // penalise y deviation near rough terrain

 // Backward pass for control refinement (Riccati equation)
 // Initialize terminal cost-to-go
 memcpy(P, QN, STATE_DIM * STATE_DIM * sizeof(double));

 // Backward recursion
 for (int k = HORIZON - 1; k >= 0; k--) {
 // Update system matrices for state at time k
 SpacecraftState state_k = *current_state;
 // Set state from trajectory
 for (int i = 0; i < 3; i++) {
 state_k.position[i] = x[k * STATE_DIM + i];
 state_k.velocity[i] = x[k * STATE_DIM + i + 3];
 state_k.attitude[i] = x[k * STATE_DIM + i + 6];
 state_k.angular_rate[i] = x[k * STATE_DIM + i + 9];
 }
 state_k.mass = x[k * STATE_DIM + 12];
 state_k.fuel = x[k * STATE_DIM + 13];

 dynamics_matrix(&state_k, A);
 control_matrix(&state_k, B);

 // Compute B^T * P
 double BT_P[CONTROL_DIM * STATE_DIM];
 double* BT = pdg_BT; /* static buffer */
 matrix_transpose(B, BT, STATE_DIM, CONTROL_DIM);
 matrix_multiply(BT, P, BT_P, CONTROL_DIM, STATE_DIM, STATE_DIM);

 // Compute B^T * P * B
 double BT_P_B[CONTROL_DIM * CONTROL_DIM];
 matrix_multiply(BT_P, B, BT_P_B, CONTROL_DIM, STATE_DIM, CONTROL_DIM);

 // Compute R + B^T * P * B
 double R_BT_P_B[CONTROL_DIM * CONTROL_DIM];
 matrix_add(R, BT_P_B, R_BT_P_B, CONTROL_DIM, CONTROL_DIM);

 // Compute (R + B^T * P * B)^-1
 matrix_inverse(R_BT_P_B, Kinv, CONTROL_DIM);

 // Compute K = (R + B^T * P * B)^-1 * B^T * P * A
 double BT_P_A[CONTROL_DIM * STATE_DIM];
 matrix_multiply(BT_P, A, BT_P_A, CONTROL_DIM, STATE_DIM, STATE_DIM);
 matrix_multiply(Kinv, BT_P_A, K, CONTROL_DIM, CONTROL_DIM, STATE_DIM);

 // Update control for this time step
 double xk[STATE_DIM];
 double target_diff[STATE_DIM];
 for (int i = 0; i < STATE_DIM; i++) {
 xk[i] = x[k * STATE_DIM + i];
 target_diff[i] = target_state[i] - xk[i];
 }

 double uk_update[CONTROL_DIM];
 matrix_vector_multiply(K, target_diff, uk_update, CONTROL_DIM, STATE_DIM);

 // Apply control update with damping to ensure convergence
 double damping = 0.5; // Damping factor for control updates
 for (int i = 0; i < CONTROL_DIM; i++) {
 u[k * CONTROL_DIM + i] += damping * uk_update[i];

 // Apply control constraints
 if (i == 0) { // Main thrust
 if (u[k * CONTROL_DIM + i] < MIN_THRUST) u[k * CONTROL_DIM + i] = MIN_THRUST;
 if (u[k * CONTROL_DIM + i] > MAX_THRUST) u[k * CONTROL_DIM + i] = MAX_THRUST;
 } else if (i == 1 || i == 2) { // Gimbal angles
 if (u[k * CONTROL_DIM + i] < -MAX_GIMBAL_ANGLE) u[k * CONTROL_DIM + i] = -MAX_GIMBAL_ANGLE;
 if (u[k * CONTROL_DIM + i] > MAX_GIMBAL_ANGLE) u[k * CONTROL_DIM + i] = MAX_GIMBAL_ANGLE;
 } else { // RCS thrusters
 double max_rcs = 50.0;
 if (u[k * CONTROL_DIM + i] < -max_rcs) u[k * CONTROL_DIM + i] = -max_rcs;
 if (u[k * CONTROL_DIM + i] > max_rcs) u[k * CONTROL_DIM + i] = max_rcs;
 }
 }

 // Update P for next iteration
 // P = Q + A^T * P * A - A^T * P * B * K
 double AT[STATE_DIM * STATE_DIM];
 matrix_transpose(A, AT, STATE_DIM, STATE_DIM);

 double AT_P[STATE_DIM * STATE_DIM];
 matrix_multiply(AT, P, AT_P, STATE_DIM, STATE_DIM, STATE_DIM);

 double AT_P_A[STATE_DIM * STATE_DIM];
 matrix_multiply(AT_P, A, AT_P_A, STATE_DIM, STATE_DIM, STATE_DIM);

 double AT_P_B[STATE_DIM * CONTROL_DIM];
 matrix_multiply(AT_P, B, AT_P_B, STATE_DIM, STATE_DIM, CONTROL_DIM);

 double AT_P_B_K[STATE_DIM * STATE_DIM];
 matrix_multiply(AT_P_B, K, AT_P_B_K, STATE_DIM, CONTROL_DIM, STATE_DIM);

 double temp[STATE_DIM * STATE_DIM];
 matrix_subtract(AT_P_A, AT_P_B_K, temp, STATE_DIM, STATE_DIM);
 matrix_add(Q, temp, P, STATE_DIM, STATE_DIM);
 }

 }

 // Return the optimized control sequence
 memcpy(control_sequence, u, CONTROL_DIM * HORIZON * sizeof(double));

 /* Static memory — no deallocation required */
}

// --------------------------------------------------------------------------
// Navigation Extended Kalman Filter (KalF BCE component)
// 6-state EKF: position[3] + velocity[3]
// --------------------------------------------------------------------------

// Navigation dynamics (pos + vel, with external acceleration)
static void nav_dynamics_6(double* ns, double* accel, double* nd) {
 // pos_dot = vel
 nd[0] = ns[3];
 nd[1] = ns[4];
 nd[2] = ns[5];
 // vel_dot = commanded acceleration - gravity
 nd[3] = accel[0];
 nd[4] = accel[1];
 nd[5] = accel[2] - GRAVITY;
}

// RK4 for the 6-state navigation model
static void nav_rk4_step(double* ns, double* accel, double dt, double* ns_next) {
 double k1[NAV_STATE_DIM], k2[NAV_STATE_DIM], k3[NAV_STATE_DIM], k4[NAV_STATE_DIM];
 double tmp[NAV_STATE_DIM];
 int i;

 nav_dynamics_6(ns, accel, k1);
 for (i = 0; i < NAV_STATE_DIM; i++) tmp[i] = ns[i] + 0.5*dt*k1[i];
 nav_dynamics_6(tmp, accel, k2);
 for (i = 0; i < NAV_STATE_DIM; i++) tmp[i] = ns[i] + 0.5*dt*k2[i];
 nav_dynamics_6(tmp, accel, k3);
 for (i = 0; i < NAV_STATE_DIM; i++) tmp[i] = ns[i] + dt*k3[i];
 nav_dynamics_6(tmp, accel, k4);

 for (i = 0; i < NAV_STATE_DIM; i++)
 ns_next[i] = ns[i] + (dt/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
}

// Compute Jacobian F = I + dt * A_ct for the 6-state navigation model
static void nav_state_jacobian(double dt, double* F) {
 memset(F, 0, NAV_STATE_DIM * NAV_STATE_DIM * sizeof(double));
 for (int i = 0; i < NAV_STATE_DIM; i++)
 F[i * NAV_STATE_DIM + i] = 1.0;
 // Position rows coupled to velocity:
 for (int i = 0; i < 3; i++)
 F[i * NAV_STATE_DIM + (i + 3)] = dt;
}

// Full EKF predict + update cycle
// nav_state : 6x1 in/out
// P_nav : 6x6 in/out covariance
// accel : 3x1 inertial acceleration from guidance control
// measurement: 6x1 noisy position+velocity observation
// Q_nav : 6x6 process noise covariance
// R_nav : 6x6 measurement noise covariance
void navigation_ekf_step(double* nav_state, double* P_nav,
 double* accel, double* measurement,
 double* Q_nav, double* R_nav, double dt) {
 int i;

 // ---- PREDICT ----
 // Propagate state with RK4
 double x_pred[NAV_STATE_DIM];
 nav_rk4_step(nav_state, accel, dt, x_pred);

 // Jacobian F
 double F[NAV_STATE_DIM * NAV_STATE_DIM];
 nav_state_jacobian(dt, F);

 // P_pred = F * P * F^T + Q
 double FT[NAV_STATE_DIM * NAV_STATE_DIM];
 matrix_transpose(F, FT, NAV_STATE_DIM, NAV_STATE_DIM);

 double FP[NAV_STATE_DIM * NAV_STATE_DIM];
 matrix_multiply(F, P_nav, FP, NAV_STATE_DIM, NAV_STATE_DIM, NAV_STATE_DIM);

 double FP_FT[NAV_STATE_DIM * NAV_STATE_DIM];
 matrix_multiply(FP, FT, FP_FT, NAV_STATE_DIM, NAV_STATE_DIM, NAV_STATE_DIM);

 double P_pred[NAV_STATE_DIM * NAV_STATE_DIM];
 matrix_add(FP_FT, Q_nav, P_pred, NAV_STATE_DIM, NAV_STATE_DIM);

 // ---- UPDATE ----
 // Measurement model H = I (direct observation of pos + vel)
 // Innovation: y = z - H * x_pred = z - x_pred
 double innov[NAV_STATE_DIM];
 for (i = 0; i < NAV_STATE_DIM; i++)
 innov[i] = measurement[i] - x_pred[i];

 // Innovation covariance S = H * P_pred * H^T + R = P_pred + R
 double S[NAV_STATE_DIM * NAV_STATE_DIM];
 matrix_add(P_pred, R_nav, S, NAV_STATE_DIM, NAV_STATE_DIM);

 // S^{-1}
 double S_inv[NAV_STATE_DIM * NAV_STATE_DIM];
 matrix_inverse(S, S_inv, NAV_STATE_DIM);

 // Kalman gain K = P_pred * H^T * S^{-1} = P_pred * S^{-1} (H=I)
 double K[NAV_STATE_DIM * NAV_STATE_DIM];
 matrix_multiply(P_pred, S_inv, K, NAV_STATE_DIM, NAV_STATE_DIM, NAV_STATE_DIM);

 // State update x = x_pred + K * innov
 double K_innov[NAV_STATE_DIM];
 matrix_vector_multiply(K, innov, K_innov, NAV_STATE_DIM, NAV_STATE_DIM);
 for (i = 0; i < NAV_STATE_DIM; i++)
 nav_state[i] = x_pred[i] + K_innov[i];

 // Covariance update P = (I - K*H) * P_pred = (I - K) * P_pred (H=I)
 double ImK[NAV_STATE_DIM * NAV_STATE_DIM];
 memset(ImK, 0, NAV_STATE_DIM * NAV_STATE_DIM * sizeof(double));
 for (i = 0; i < NAV_STATE_DIM; i++)
 ImK[i * NAV_STATE_DIM + i] = 1.0;
 matrix_subtract(ImK, K, ImK, NAV_STATE_DIM, NAV_STATE_DIM);
 matrix_multiply(ImK, P_pred, P_nav, NAV_STATE_DIM, NAV_STATE_DIM, NAV_STATE_DIM);
}

// Initialize a random terrain map
void initialize_terrain(TerrainMap* terrain) {
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(5639);
 for (int i = 0; i < TERRAIN_GRID_SIZE; i++) {
 for (int j = 0; j < TERRAIN_GRID_SIZE; j++) {
 // Height varies from 0 to 10 meters
 terrain->height[i][j] = (double)rand() / RAND_MAX * 10.0;

 // Slope varies from 0 to 30 degrees
 terrain->slope[i][j] = (double)rand() / RAND_MAX * 30.0 * M_PI / 180.0;

 // Roughness varies from 0 to 1 (normalized)
 terrain->roughness[i][j] = (double)rand() / RAND_MAX;
 }
 }

 // Add some spatial coherence by smoothing
 for (int iter = 0; iter < 3; iter++) {
 double temp_height[TERRAIN_GRID_SIZE][TERRAIN_GRID_SIZE];
 double temp_slope[TERRAIN_GRID_SIZE][TERRAIN_GRID_SIZE];
 double temp_roughness[TERRAIN_GRID_SIZE][TERRAIN_GRID_SIZE];

 // Smoothing pass
 for (int i = 1; i < TERRAIN_GRID_SIZE-1; i++) {
 for (int j = 1; j < TERRAIN_GRID_SIZE-1; j++) {
 temp_height[i][j] = (terrain->height[i-1][j] + terrain->height[i+1][j] +
 terrain->height[i][j-1] + terrain->height[i][j+1] +
 terrain->height[i][j]) / 5.0;

 temp_slope[i][j] = (terrain->slope[i-1][j] + terrain->slope[i+1][j] +
 terrain->slope[i][j-1] + terrain->slope[i][j+1] +
 terrain->slope[i][j]) / 5.0;

 temp_roughness[i][j] = (terrain->roughness[i-1][j] + terrain->roughness[i+1][j] +
 terrain->roughness[i][j-1] + terrain->roughness[i][j+1] +
 terrain->roughness[i][j]) / 5.0;
 }
 }

 // Copy back smoothed values
 for (int i = 1; i < TERRAIN_GRID_SIZE-1; i++) {
 for (int j = 1; j < TERRAIN_GRID_SIZE-1; j++) {
 terrain->height[i][j] = temp_height[i][j];
 terrain->slope[i][j] = temp_slope[i][j];
 terrain->roughness[i][j] = temp_roughness[i][j];
 }
 }
 }

 // Create a flat landing zone near the center
 int center = TERRAIN_GRID_SIZE / 2;
 int landing_zone_size = TERRAIN_GRID_SIZE / 6;
 for (int i = center - landing_zone_size; i <= center + landing_zone_size; i++) {
 for (int j = center - landing_zone_size; j <= center + landing_zone_size; j++) {
 if (i >= 0 && i < TERRAIN_GRID_SIZE && j >= 0 && j < TERRAIN_GRID_SIZE) {
 terrain->height[i][j] = 0.0;
 terrain->slope[i][j] = 0.0;
 terrain->roughness[i][j] = 0.1;
 }
 }
 }
}

// Main function to run the Powered Descent Guidance (PDG) algorithm
int main() {
 FLIGHT_LOG("[GNC] init\n");
 FLIGHT_LOG("Matrix dimensions: State=%d, Control=%d, Horizon=%d\n",
 STATE_DIM, CONTROL_DIM, HORIZON);

 // Initialize spacecraft state
 SpacecraftState current_state;
 // Starting at 500m altitude with some initial velocity
 current_state.position[0] = 100.0; // x = 100m
 current_state.position[1] = 50.0; // y = 50m
 current_state.position[2] = 500.0; // z = 500m altitude

 current_state.velocity[0] = -5.0; // vx = -5 m/s
 current_state.velocity[1] = -2.5; // vy = -2.5 m/s
 current_state.velocity[2] = -10.0; // vz = -10 m/s (descending)

 current_state.attitude[0] = 0.05; // roll = 0.05 rad
 current_state.attitude[1] = -0.03; // pitch = -0.03 rad
 current_state.attitude[2] = 0.1; // yaw = 0.1 rad

 current_state.angular_rate[0] = 0.01; // roll rate = 0.01 rad/s
 current_state.angular_rate[1] = -0.01; // pitch rate = -0.01 rad/s
 current_state.angular_rate[2] = 0.005; // yaw rate = 0.005 rad/s

 current_state.mass = SPACECRAFT_MASS;
 current_state.fuel = 500.0; // 500kg of fuel

 // Define target landing state
 double target_state[STATE_DIM];
 // Target is at origin with zero velocity and upright orientation
 target_state[0] = 0.0; // x = 0
 target_state[1] = 0.0; // y = 0
 target_state[2] = 0.0; // z = 0 (ground level)

 target_state[3] = 0.0; // vx = 0
 target_state[4] = 0.0; // vy = 0
 target_state[5] = -2.0; // vz = -2 m/s (gentle touchdown)

 target_state[6] = 0.0; // roll = 0
 target_state[7] = 0.0; // pitch = 0
 target_state[8] = 0.0; // yaw = 0

 target_state[9] = 0.0; // roll rate = 0
 target_state[10] = 0.0; // pitch rate = 0
 target_state[11] = 0.0; // yaw rate = 0

 target_state[12] = current_state.mass - 100.0; // Expected mass after fuel consumption
 target_state[13] = 400.0; // Expected fuel remaining
 target_state[14] = HORIZON * TIMESTEP; // Final time

 // Initialize terrain map
 TerrainMap terrain;
 initialize_terrain(&terrain);

 // Use pre-allocated static control sequence buffer
 double* control_sequence = ctrl_seq;

 // ---- Navigation EKF setup ----
 double nav_state[NAV_STATE_DIM];
 nav_state[0] = current_state.position[0];
 nav_state[1] = current_state.position[1];
 nav_state[2] = current_state.position[2];
 nav_state[3] = current_state.velocity[0];
 nav_state[4] = current_state.velocity[1];
 nav_state[5] = current_state.velocity[2];

 // Initial covariance P0 = diag(10, 10, 10, 1, 1, 1)
 double P_nav[NAV_STATE_DIM * NAV_STATE_DIM];
 memset(P_nav, 0, sizeof(P_nav));
 for (int i = 0; i < 3; i++) P_nav[i * NAV_STATE_DIM + i] = 10.0;
 for (int i = 3; i < 6; i++) P_nav[i * NAV_STATE_DIM + i] = 1.0;

 // Process noise Q_nav = diag(0.01, 0.01, 0.01, 0.1, 0.1, 0.1)
 double Q_nav[NAV_STATE_DIM * NAV_STATE_DIM];
 memset(Q_nav, 0, sizeof(Q_nav));
 for (int i = 0; i < 3; i++) Q_nav[i * NAV_STATE_DIM + i] = 0.01;
 for (int i = 3; i < 6; i++) Q_nav[i * NAV_STATE_DIM + i] = 0.1;

 // Measurement noise R_nav = diag(5, 5, 5, 0.5, 0.5, 0.5)
 double R_nav[NAV_STATE_DIM * NAV_STATE_DIM];
 memset(R_nav, 0, sizeof(R_nav));
 for (int i = 0; i < 3; i++) R_nav[i * NAV_STATE_DIM + i] = 5.0;
 for (int i = 3; i < 6; i++) R_nav[i * NAV_STATE_DIM + i] = 0.5;

 // Start timing the algorithm
 clock_t start_time = clock();

 // Run the Powered Descent Guidance (PDG) algorithm
 compute_pdg(&current_state, target_state, &terrain, control_sequence);

 // ---- Run Navigation EKF alongside guidance trajectory ----
 FLIGHT_LOG("\n[GNC] EKF: steps=%d\n", HORIZON);
 for (int step = 0; step < HORIZON; step++) {
 // Compute inertial acceleration from the optimal control at this step
 double thrust_cmd = control_sequence[step * CONTROL_DIM + 0];
 double inv_m = (current_state.mass > 10.0) ? 1.0 / current_state.mass : 0.1;
 double accel[3];
 accel[0] = 0.0; // lateral acceleration
 accel[1] = 0.0;
 accel[2] = thrust_cmd * inv_m; // vertical thrust accel

 // Simulate noisy measurement (true state + noise)
 double measurement[NAV_STATE_DIM];
 for (int i = 0; i < 3; i++)
 measurement[i] = nav_state[i] + ((double)rand()/RAND_MAX - 0.5) * 4.0;
 for (int i = 3; i < 6; i++)
 measurement[i] = nav_state[i] + ((double)rand()/RAND_MAX - 0.5) * 1.0;

 // EKF predict + update
 navigation_ekf_step(nav_state, P_nav, accel, measurement,
 Q_nav, R_nav, TIMESTEP);
 }
 FLIGHT_LOG("Nav EKF final estimate: pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]\n",
 nav_state[0], nav_state[1], nav_state[2],
 nav_state[3], nav_state[4], nav_state[5]);
 FLIGHT_LOG("Nav EKF P diag: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
 P_nav[0*NAV_STATE_DIM+0], P_nav[1*NAV_STATE_DIM+1], P_nav[2*NAV_STATE_DIM+2],
 P_nav[3*NAV_STATE_DIM+3], P_nav[4*NAV_STATE_DIM+4], P_nav[5*NAV_STATE_DIM+5]);

 // End timing
 clock_t end_time = clock();
 double execution_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;

 // Print execution statistics
 FLIGHT_LOG("PDG algorithm execution time: %.3f seconds\n", execution_time);
 FLIGHT_LOG("Matrix operations performed: %d\n",
 TIMESTEPS * HORIZON * (STATE_DIM * STATE_DIM * STATE_DIM +
 STATE_DIM * STATE_DIM * CONTROL_DIM +
 CONTROL_DIM * CONTROL_DIM * STATE_DIM));
 FLIGHT_LOG("Trigonometric operations performed: %d\n",
 TIMESTEPS * HORIZON * 12); // Approximate count

 // Print out first few control inputs
 FLIGHT_LOG("\nOptimal control sequence (first 3 steps):\n");
 for (int i = 0; i < 3; i++) {
 FLIGHT_LOG("Step %d: Thrust=%.2f, Gimbal X=%.4f, Gimbal Y=%.4f, RCS=[%.2f, %.2f, %.2f]\n",
 i,
 control_sequence[i * CONTROL_DIM],
 control_sequence[i * CONTROL_DIM + 1],
 control_sequence[i * CONTROL_DIM + 2],
 control_sequence[i * CONTROL_DIM + 3],
 control_sequence[i * CONTROL_DIM + 4],
 control_sequence[i * CONTROL_DIM + 5]);
 }

 /* Static memory — no deallocation required */

 return 0;
}
