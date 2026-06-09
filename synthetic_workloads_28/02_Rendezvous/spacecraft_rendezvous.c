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
 * Spacecraft Rendezvous and Proximity Operations Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the rendezvous, proximity operations, and docking (RPOD) guidance
 * algorithm for autonomous docking missions. This workload computes relative
 * orbital mechanics using Hill-Clohessy-Wiltshire equations with 9-state extended
 * Kalman filtering (relative position, velocity, and accelerometer bias),
 * quaternion-based attitude alignment, and approach trajectory planning.
 * Directly applicable to the SPADEX (Space Docking Experiment) mission and
 * future on-orbit servicing and crewed-spacecraft docking operations.
 *
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
#include <time.h>
#include <string.h>
#include "../flight_compliance.h"

#define MATRIX_SIZE 9 // 9-state: relative pos(3) + relative vel(3) + accel bias(3)
#define MAX_TIMESTEPS 4000
#define DOCKING_DISTANCE_THRESHOLD 0.5 // meters
#define DOCKING_ANGLE_THRESHOLD 0.05 // radians (~2.86 deg)
#define SIMULATION_TIMESTEP 0.1 // seconds

/* ── Orbital environment ─────────────────────────────────────────
 * Mean motion n = sqrt(μ/a³) is NOT a compile-time constant.
 * It is dynamically recomputed each GNC cycle from the target's
 * absolute navigation solution (GPS / Star Tracker / ground TLE)
 * to account for:
 * - Atmospheric drag causing orbit decay (≈2 km/year at 400 km)
 * - Non-circularity (e > 0 introduces time-varying a)
 * - Manoeuvre orbit changes
 * The constant below is kept only as an initial / fallback value. */
#define MU_EARTH_M 3.986004418e14 /* m³ s⁻² */
#define R_EARTH_M 6378137.0 /* WGS-84 equatorial radius */
#define INITIAL_ORBIT_ALT_M 400000.0 /* nominal target altitude */
#define DRAG_DECAY_RATE 1.2e-4 /* m/s altitude decay (~3.8 km/yr at 400 km) */

typedef struct {
 double data[MATRIX_SIZE][MATRIX_SIZE];
} Matrix;

typedef struct {
 double x, y, z;
} Vector3;

typedef struct {
 double w, x, y, z;
} Quaternion;

typedef struct {
 Vector3 position; // Position in LVLH frame (m)
 Vector3 velocity; // Velocity in LVLH frame (m/s)
 Vector3 acceleration; // Acceleration in LVLH frame (m/s²)
 Quaternion orientation; // Spacecraft orientation
 Vector3 angular_velocity; // Angular velocity (rad/s)
 double mass; // Mass (kg)
 double inertia_matrix[3][3]; // Inertia matrix
 double orbit_radius; // Absolute orbit radius (m) — from GPS/TLE
} SpacecraftState;

// Function declarations
void matrix_multiply(Matrix *result, Matrix *a, Matrix *b);
void matrix_add(Matrix *result, Matrix *a, Matrix *b);
void matrix_subtract(Matrix *result, Matrix *a, Matrix *b);
void matrix_scale(Matrix *result, Matrix *a, double scalar);
void matrix_transpose(Matrix *result, Matrix *a);
int matrix_inverse(Matrix *result, Matrix *a);
void matrix_identity(Matrix *m);
void matrix_zero(Matrix *m);

Vector3 vector_add(Vector3 a, Vector3 b);
Vector3 vector_subtract(Vector3 a, Vector3 b);
Vector3 vector_scale(Vector3 a, double scalar);
Vector3 vector_cross(Vector3 a, Vector3 b);
double vector_dot(Vector3 a, Vector3 b);
double vector_magnitude(Vector3 a);
Vector3 vector_normalize(Vector3 a);

Quaternion quaternion_multiply(Quaternion a, Quaternion b);
Quaternion quaternion_from_axis_angle(Vector3 axis, double angle);
Vector3 quaternion_rotate_vector(Quaternion q, Vector3 v);
Quaternion quaternion_normalize(Quaternion q);
Quaternion quaternion_conjugate(Quaternion q);

void orbital_mechanics_update(SpacecraftState *state, double mean_motion, double delta_t);
Matrix create_state_transition_matrix(double mean_motion, double delta_t);
Matrix create_process_noise_matrix(double delta_t, double process_noise_scalar);

void compute_relative_navigation(SpacecraftState *chaser, SpacecraftState *target,
 double mean_motion,
 Vector3 *rel_position, Vector3 *rel_velocity);
void kalman_filter_update(Matrix *state, Matrix *covariance,
 Matrix *measurement, Matrix *measurement_noise,
 Matrix *state_transition, Matrix *process_noise);
void compute_approach_trajectory(SpacecraftState *chaser, SpacecraftState *target,
 double mean_motion, Vector3 *thrust_command);
void attitude_control(SpacecraftState *chaser, SpacecraftState *target,
 Vector3 *torque_command);
int check_docking_conditions(SpacecraftState *chaser, SpacecraftState *target);
Vector3 compute_clohessy_wiltshire(SpacecraftState *chaser, SpacecraftState *target, double mean_motion);

int main() {
 // Initialize spacecraft states
 SpacecraftState chaser, target;

 // Initialize Kalman filter
 Matrix kalman_state, kalman_covariance, measurement_noise;

 srand(1597); /* deterministic seed for reproducible gem5 traces */

 /* Compute initial mean motion from target absolute orbit state.
 * Updated each GNC cycle from onboard navigation (GPS/TLE). */
 double target_orbit_radius = R_EARTH_M + INITIAL_ORBIT_ALT_M;

 // Initialize chaser spacecraft (approaching vehicle)
 chaser.position = (Vector3){100.0, 20.0, -30.0}; // 100m behind, 20m to right, 30m below
 chaser.velocity = (Vector3){-0.1, -0.05, 0.02}; // Initial approach velocity
 chaser.acceleration = (Vector3){0.0, 0.0, 0.0};
 chaser.orientation = (Quaternion){1.0, 0.0, 0.0, 0.0}; // Identity quaternion
 chaser.angular_velocity = (Vector3){0.0, 0.0, 0.0};
 chaser.mass = 1340.0; /* MEV-class service vehicle */
 chaser.orbit_radius = R_EARTH_M + INITIAL_ORBIT_ALT_M; /* same initial orbit */

 /* Inertia tensor (SPADEX chaser bus, from CAD model) */
 chaser.inertia_matrix[0][0] = 1185.0;
 chaser.inertia_matrix[0][1] = -12.4;
 chaser.inertia_matrix[0][2] = 5.7;
 chaser.inertia_matrix[1][0] = -12.4;
 chaser.inertia_matrix[1][1] = 1462.0;
 chaser.inertia_matrix[1][2] = -8.3;
 chaser.inertia_matrix[2][0] = 5.7;
 chaser.inertia_matrix[2][1] = -8.3;
 chaser.inertia_matrix[2][2] = 1738.0;

 // Initialize target spacecraft (docking station)
 target.position = (Vector3){0.0, 0.0, 0.0}; // Origin of the reference frame
 target.velocity = (Vector3){0.0, 0.0, 0.0}; // Stationary in reference frame
 target.acceleration = (Vector3){0.0, 0.0, 0.0};
 target.orientation = (Quaternion){1.0, 0.0, 0.0, 0.0}; // Identity quaternion
 target.angular_velocity = (Vector3){0.0, 0.00117, 0.0}; // Tumble rate from attitude telemetry
 target.mass = 20430.0; // ISS-class station module
 target.orbit_radius = R_EARTH_M + INITIAL_ORBIT_ALT_M; /* absolute orbit from TLE */

 // Initialize Kalman filter matrices
 matrix_zero(&kalman_state);
 matrix_identity(&kalman_covariance);
 matrix_scale(&kalman_covariance, &kalman_covariance, 12.5); // Initial uncertainty (conservative)
 matrix_zero(&measurement_noise);

 /* Measurement noise from sensor specifications:
 * Position: LIDAR range ~8.5 mm 1σ → R_pos = 7.2e-5 m²
 * Velocity: Doppler radar ~3 mm/s → R_vel = 9.0e-6 m²/s²
 * Bias: accelerometer stability → R_bias = 1.5e-8 m²/s⁴ */
 for (int i = 0; i < 3; i++)
 measurement_noise.data[i][i] = 7.2e-5;
 for (int i = 3; i < 6; i++)
 measurement_noise.data[i][i] = 9.0e-6;
 for (int i = 6; i < 9; i++)
 measurement_noise.data[i][i] = 1.5e-8;

 // Run simulation
 int iteration = 0;
 int docked = 0;
 double sim_time = 0.0;

 /* Dynamic mean motion — recomputed from target absolute orbit each cycle */
 double mean_motion = sqrt(MU_EARTH_M / (target_orbit_radius * target_orbit_radius * target_orbit_radius));

 Vector3 rel_position, rel_velocity;
 Vector3 thrust_command, torque_command;

 FLIGHT_LOG("[RVD] begin\n");

 // Main simulation loop
 while (iteration < MAX_TIMESTEPS && !docked) {
 // Update simulation time
 sim_time += SIMULATION_TIMESTEP;

 /* ── Dynamic mean motion update from target absolute nav ──
 * In flight this comes from the GPS/Star-Tracker navigation
 * solution. Here we simulate atmospheric drag orbit decay
 * at ~3.8 km/year (INITIAL_ORBIT_ALT_M = 400 km). */
 target_orbit_radius -= DRAG_DECAY_RATE * SIMULATION_TIMESTEP;
 target.orbit_radius = target_orbit_radius;
 chaser.orbit_radius = target_orbit_radius; /* co-orbital */
 mean_motion = sqrt(MU_EARTH_M / (target_orbit_radius * target_orbit_radius * target_orbit_radius));

 // Compute relative navigation
 compute_relative_navigation(&chaser, &target, mean_motion, &rel_position, &rel_velocity);

 // Create state transition matrix
 Matrix state_transition = create_state_transition_matrix(mean_motion, SIMULATION_TIMESTEP);

 // Create process noise matrix
 Matrix process_noise = create_process_noise_matrix(SIMULATION_TIMESTEP, 8.5e-4);

 // Prepare measurement matrix (in this simulation we create simulated measurements)
 Matrix measurement;
 matrix_zero(&measurement);

 // Fill measurement matrix with relative state and add noise
 // Position measurements (with noise)
 measurement.data[0][0] = rel_position.x + ((double)rand() / RAND_MAX - 0.5) * 0.017;
 measurement.data[1][0] = rel_position.y + ((double)rand() / RAND_MAX - 0.5) * 0.017;
 measurement.data[2][0] = rel_position.z + ((double)rand() / RAND_MAX - 0.5) * 0.017;

 /* Velocity measurements – Doppler radar noise ~3 mm/s 1σ */
 measurement.data[3][0] = rel_velocity.x + ((double)rand() / RAND_MAX - 0.5) * 0.006;
 measurement.data[4][0] = rel_velocity.y + ((double)rand() / RAND_MAX - 0.5) * 0.006;
 measurement.data[5][0] = rel_velocity.z + ((double)rand() / RAND_MAX - 0.5) * 0.006;

 // Update Kalman filter
 kalman_filter_update(&kalman_state, &kalman_covariance, &measurement,
 &measurement_noise, &state_transition, &process_noise);

 /* Use KF-filtered relative state for guidance (closes the estimation→control loop) */
 SpacecraftState kf_chaser = chaser;
 kf_chaser.position = (Vector3){
 target.position.x + kalman_state.data[0][0],
 target.position.y + kalman_state.data[1][0],
 target.position.z + kalman_state.data[2][0]
 };
 kf_chaser.velocity = (Vector3){
 target.velocity.x + kalman_state.data[3][0],
 target.velocity.y + kalman_state.data[4][0],
 target.velocity.z + kalman_state.data[5][0]
 };

 compute_approach_trajectory(&kf_chaser, &target, mean_motion, &thrust_command);
 attitude_control(&kf_chaser, &target, &torque_command);

 /* Apply thrust acceleration to translational dynamics */
 chaser.acceleration = thrust_command;

 /* Apply torque to angular rates via Euler's equation:
 * dω/dt = I⁻¹ · (τ - ω × I·ω) (simplified for diagonal-dominant I) */
 chaser.angular_velocity.x += torque_command.x / chaser.inertia_matrix[0][0] * SIMULATION_TIMESTEP;
 chaser.angular_velocity.y += torque_command.y / chaser.inertia_matrix[1][1] * SIMULATION_TIMESTEP;
 chaser.angular_velocity.z += torque_command.z / chaser.inertia_matrix[2][2] * SIMULATION_TIMESTEP;

 // Update orbital mechanics (RK4 with CW dynamics)
 orbital_mechanics_update(&chaser, mean_motion, SIMULATION_TIMESTEP);
 orbital_mechanics_update(&target, mean_motion, SIMULATION_TIMESTEP);

 // Check docking conditions
 docked = check_docking_conditions(&chaser, &target);

 // Display progress every 10 iterations
 if (iteration % 10 == 0) {
 FLIGHT_LOG("Iteration %d: Time=%.1fs, Distance=%.2fm, Relative vel=%.3fm/s\n",
 iteration, sim_time, vector_magnitude(rel_position), vector_magnitude(rel_velocity));
 }

 iteration++;
 }

 // Report results
 if (docked) {
 FLIGHT_LOG("Successfully docked after %.1f seconds (%d iterations)!\n", sim_time, iteration);
 } else {
 FLIGHT_LOG("Failed to dock within maximum iterations.\n");
 }

 return 0;
}

// Matrix operations implementations
void matrix_multiply(Matrix *result, Matrix *a, Matrix *b) {
 Matrix temp;

 // Initialize result matrix with zeros
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 temp.data[i][j] = 0.0;
 }
 }

 // Perform matrix multiplication
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 for (int k = 0; k < MATRIX_SIZE; k++) {
 temp.data[i][j] += a->data[i][k] * b->data[k][j];
 }
 }
 }

 // Copy result back
 memcpy(result, &temp, sizeof(Matrix));
}

void matrix_add(Matrix *result, Matrix *a, Matrix *b) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 result->data[i][j] = a->data[i][j] + b->data[i][j];
 }
 }
}

void matrix_subtract(Matrix *result, Matrix *a, Matrix *b) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 result->data[i][j] = a->data[i][j] - b->data[i][j];
 }
 }
}

void matrix_scale(Matrix *result, Matrix *a, double scalar) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 result->data[i][j] = a->data[i][j] * scalar;
 }
 }
}

void matrix_transpose(Matrix *result, Matrix *a) {
 Matrix temp;

 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 temp.data[j][i] = a->data[i][j];
 }
 }

 memcpy(result, &temp, sizeof(Matrix));
}

void matrix_identity(Matrix *m) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 m->data[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }
}

void matrix_zero(Matrix *m) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 m->data[i][j] = 0.0;
 }
 }
}


/* Matrix inverse using Gaussian elimination with partial pivoting.
 * Returns 0 on success, -1 if matrix is singular.
 *
 * MISRA-C:2012 compliance note: All storage is statically allocated
 * on the stack (augmented[9][18] = 1296 bytes). No heap allocation
 * (malloc/calloc/realloc/free) is used anywhere in this module,
 * satisfying Rule 21.3 and avoiding heap fragmentation during the
 * years-long cruise or multi-orbit approach phase. */
int matrix_inverse(Matrix *result, Matrix *a) {
 double augmented[MATRIX_SIZE][2 * MATRIX_SIZE];
 int pivot_index, i, j, k;
 double max_val, abs_val, pivot_val, temp;

 // Create augmented matrix [A|I]
 for (i = 0; i < MATRIX_SIZE; i++) {
 for (j = 0; j < MATRIX_SIZE; j++) {
 augmented[i][j] = a->data[i][j];
 }
 for (j = 0; j < MATRIX_SIZE; j++) {
 augmented[i][j + MATRIX_SIZE] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Gaussian elimination with partial pivoting
 for (i = 0; i < MATRIX_SIZE; i++) {
 // Find pivot
 max_val = fabs(augmented[i][i]);
 pivot_index = i;

 for (j = i + 1; j < MATRIX_SIZE; j++) {
 abs_val = fabs(augmented[j][i]);
 if (abs_val > max_val) {
 max_val = abs_val;
 pivot_index = j;
 }
 }

 // Check if matrix is singular
 if (max_val < 1e-10) {
 return -1; // Matrix is singular or nearly singular
 }

 // Swap rows if necessary
 if (pivot_index != i) {
 for (j = 0; j < 2 * MATRIX_SIZE; j++) {
 temp = augmented[i][j];
 augmented[i][j] = augmented[pivot_index][j];
 augmented[pivot_index][j] = temp;
 }
 }

 // Scale row to have a unit pivot
 pivot_val = augmented[i][i];
 for (j = 0; j < 2 * MATRIX_SIZE; j++) {
 augmented[i][j] /= pivot_val;
 }

 // Eliminate other rows
 for (j = 0; j < MATRIX_SIZE; j++) {
 if (j != i) {
 temp = augmented[j][i];
 for (k = 0; k < 2 * MATRIX_SIZE; k++) {
 augmented[j][k] -= temp * augmented[i][k];
 }
 }
 }
 }

 // Extract inverse matrix
 for (i = 0; i < MATRIX_SIZE; i++) {
 for (j = 0; j < MATRIX_SIZE; j++) {
 result->data[i][j] = augmented[i][j + MATRIX_SIZE];
 }
 }

 return 0;
}

// Vector operations
Vector3 vector_add(Vector3 a, Vector3 b) {
 Vector3 result;
 result.x = a.x + b.x;
 result.y = a.y + b.y;
 result.z = a.z + b.z;
 return result;
}

Vector3 vector_subtract(Vector3 a, Vector3 b) {
 Vector3 result;
 result.x = a.x - b.x;
 result.y = a.y - b.y;
 result.z = a.z - b.z;
 return result;
}

Vector3 vector_scale(Vector3 a, double scalar) {
 Vector3 result;
 result.x = a.x * scalar;
 result.y = a.y * scalar;
 result.z = a.z * scalar;
 return result;
}

Vector3 vector_cross(Vector3 a, Vector3 b) {
 Vector3 result;
 result.x = a.y * b.z - a.z * b.y;
 result.y = a.z * b.x - a.x * b.z;
 result.z = a.x * b.y - a.y * b.x;
 return result;
}

double vector_dot(Vector3 a, Vector3 b) {
 return a.x * b.x + a.y * b.y + a.z * b.z;
}

double vector_magnitude(Vector3 a) {
 return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

Vector3 vector_normalize(Vector3 a) {
 double mag = vector_magnitude(a);
 Vector3 result;

 if (mag < 1e-10) {
 // Avoid division by zero
 result.x = 0.0;
 result.y = 0.0;
 result.z = 0.0;
 } else {
 result.x = a.x / mag;
 result.y = a.y / mag;
 result.z = a.z / mag;
 }

 return result;
}

// Quaternion operations
Quaternion quaternion_multiply(Quaternion a, Quaternion b) {
 Quaternion result;

 result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
 result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
 result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
 result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

 return result;
}

Quaternion quaternion_from_axis_angle(Vector3 axis, double angle) {
 Quaternion q;
 double half_angle = angle / 2.0;
 double sin_half = sin(half_angle);

 q.w = cos(half_angle);
 q.x = axis.x * sin_half;
 q.y = axis.y * sin_half;
 q.z = axis.z * sin_half;

 return quaternion_normalize(q);
}

Vector3 quaternion_rotate_vector(Quaternion q, Vector3 v) {
 // v' = q * v * q^-1
 Quaternion q_inv = quaternion_conjugate(q);
 Quaternion v_quat = {0.0, v.x, v.y, v.z};

 Quaternion result = quaternion_multiply(q, quaternion_multiply(v_quat, q_inv));

 return (Vector3){result.x, result.y, result.z};
}

Quaternion quaternion_normalize(Quaternion q) {
 double magnitude = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

 if (magnitude < 1e-10) {
 // Return identity quaternion if magnitude is too small
 return (Quaternion){1.0, 0.0, 0.0, 0.0};
 }

 return (Quaternion){
 q.w / magnitude,
 q.x / magnitude,
 q.y / magnitude,
 q.z / magnitude
 };
}

Quaternion quaternion_conjugate(Quaternion q) {
 return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

/* ── Translational dynamics in the Hill (LVLH) rotating frame ────
 * State derivative for the Clohessy-Wiltshire equations with thrust:
 * ẍ = 3n²x + 2nẏ + f_x (radial)
 * ÿ = -2nẋ + f_y (along-track)
 * z̈ = -n²z + f_z (cross-track)
 *
 * The 4th-order Runge-Kutta (RK4) integrator below maintains energy
 * conservation far better than Euler over long durations between
 * GNC cycles, as required for multi-orbit approach phases and
 * SPADEX mission autonomous proximity operations (CDR requirement.). */

typedef struct { double px, py, pz, vx, vy, vz; } OrbState6;

static OrbState6 cw_deriv(OrbState6 s, double n, Vector3 f_ext)
{
 OrbState6 d;
 double n2 = n * n;
 d.px = s.vx;
 d.py = s.vy;
 d.pz = s.vz;
 d.vx = 3.0 * n2 * s.px + 2.0 * n * s.vy + f_ext.x;
 d.vy = -2.0 * n * s.vx + f_ext.y;
 d.vz = -n2 * s.pz + f_ext.z;
 return d;
}

static OrbState6 os6_add(OrbState6 a, OrbState6 b)
{
 return (OrbState6){a.px+b.px, a.py+b.py, a.pz+b.pz,
 a.vx+b.vx, a.vy+b.vy, a.vz+b.vz};
}

static OrbState6 os6_scale(OrbState6 a, double s)
{
 return (OrbState6){a.px*s, a.py*s, a.pz*s,
 a.vx*s, a.vy*s, a.vz*s};
}

void orbital_mechanics_update(SpacecraftState *state, double mean_motion,
 double delta_t)
{
 /* ── RK4 for translational CW dynamics ────────────────────── */
 OrbState6 y0 = {state->position.x, state->position.y, state->position.z,
 state->velocity.x, state->velocity.y, state->velocity.z};
 Vector3 f_ext = state->acceleration; /* thrust + perturbation */
 double dt = delta_t;

 OrbState6 k1 = cw_deriv(y0, mean_motion, f_ext);
 OrbState6 k2 = cw_deriv(os6_add(y0, os6_scale(k1, dt * 0.5)),
 mean_motion, f_ext);
 OrbState6 k3 = cw_deriv(os6_add(y0, os6_scale(k2, dt * 0.5)),
 mean_motion, f_ext);
 OrbState6 k4 = cw_deriv(os6_add(y0, os6_scale(k3, dt)),
 mean_motion, f_ext);

 /* y_new = y0 + (k1 + 2k2 + 2k3 + k4) * dt/6 */
 OrbState6 slope = os6_add(os6_add(k1, os6_scale(k2, 2.0)),
 os6_add(os6_scale(k3, 2.0), k4));
 OrbState6 yf = os6_add(y0, os6_scale(slope, dt / 6.0));

 state->position.x = yf.px;
 state->position.y = yf.py;
 state->position.z = yf.pz;
 state->velocity.x = yf.vx;
 state->velocity.y = yf.vy;
 state->velocity.z = yf.vz;

 /* ── Attitude kinematics (unchanged — quaternion Euler is exact
 * for constant ω over short intervals) ─────────────────── */
 Vector3 rotation_axis = vector_normalize(state->angular_velocity);
 double rotation_angle = vector_magnitude(state->angular_velocity) * delta_t;

 if (rotation_angle > 1e-10) {
 Quaternion rotation = quaternion_from_axis_angle(rotation_axis, rotation_angle);
 state->orientation = quaternion_multiply(rotation, state->orientation);
 state->orientation = quaternion_normalize(state->orientation);
 }
}

// Create state transition matrix for orbital motion
Matrix create_state_transition_matrix(double mean_motion, double delta_t) {
 Matrix stm;
 matrix_identity(&stm);

 /* Relative motion dynamics: Clohessy-Wiltshire (Hill) equations
 * 9-state: relative pos(3), relative vel(3), accel bias(3) */
 double nt = mean_motion * delta_t;
 double c = cos(nt);
 double s = sin(nt);
 double n = mean_motion;

 /* Position-to-position (3x3 upper-left) */
 stm.data[0][0] = 4 - 3 * c; /* radial */
 stm.data[1][0] = 6 * (s - nt); /* along-track from radial */
 stm.data[1][1] = 1; /* along-track */
 stm.data[2][2] = c; /* cross-track */

 /* Position-to-velocity (3x3 upper-right) */
 stm.data[0][3] = s / n;
 stm.data[0][4] = 2 * (1 - c) / n;
 stm.data[1][3] = 2 * (c - 1) / n;
 stm.data[1][4] = (4 * s - 3 * nt) / n;
 stm.data[2][5] = s / n;

 /* Velocity-to-position (3x3 lower-left) */
 stm.data[3][0] = 3 * n * s;
 stm.data[4][0] = 6 * n * (c - 1);
 stm.data[5][2] = -n * s;

 /* Velocity-to-velocity (3x3 center) */
 stm.data[3][3] = c;
 stm.data[3][4] = 2 * s;
 stm.data[4][3] = -2 * s;
 stm.data[4][4] = 4 * c - 3;
 stm.data[5][5] = c;

 /* Accelerometer bias states (random walk): rows 6-8 */
 for (int i = 6; i < 9; i++)
 stm.data[i][i] = 1.0;
 /* Bias-to-velocity coupling: accel bias integrates into velocity */
 for (int i = 0; i < 3; i++)
 stm.data[3 + i][6 + i] = delta_t;

 return stm;
}

/* Build process noise matrix Q for the 9-state CW+bias filter.
 * Unforced relative motion is driven by differential drag/J2 perturbations
 * modelled as white-noise acceleration, and accelerometer bias is a random walk.
 * q_accel = 1e-6 m/s^2/sqrt(Hz) (differential-drag class disturbance)
 * q_bias = 1e-8 m/s^2/sqrt(Hz) (accelerometer bias stability) */
Matrix create_process_noise_matrix(double delta_t, double process_noise_scalar) {
 Matrix pnm;
 matrix_zero(&pnm);

 double q_accel = process_noise_scalar; /* acceleration PSD */
 double q_bias = process_noise_scalar * 0.01; /* bias PSD */
 double dt = delta_t;
 double dt2 = dt * dt;
 double dt3 = dt2 * dt;

 /* Position block: q * dt^3 / 3 (integrated white-noise acceleration) */
 for (int i = 0; i < 3; i++)
 pnm.data[i][i] = q_accel * dt3 / 3.0;

 /* Velocity block: q * dt */
 for (int i = 3; i < 6; i++)
 pnm.data[i][i] = q_accel * dt;

 /* Position-velocity cross-correlation: q * dt^2 / 2 */
 for (int i = 0; i < 3; i++) {
 pnm.data[i][i + 3] = q_accel * dt2 / 2.0;
 pnm.data[i + 3][i] = q_accel * dt2 / 2.0;
 }

 /* Accelerometer bias block: random walk q_bias * dt */
 for (int i = 6; i < 9; i++)
 pnm.data[i][i] = q_bias * dt;

 return pnm;
}

// Compute relative position and velocity between spacecraft
void compute_relative_navigation(SpacecraftState *chaser, SpacecraftState *target,
 double mean_motion,
 Vector3 *rel_position, Vector3 *rel_velocity) {
 // Compute relative position in LVLH frame
 *rel_position = vector_subtract(chaser->position, target->position);

 // Compute relative velocity in LVLH frame
 *rel_velocity = vector_subtract(chaser->velocity, target->velocity);

 /* Coriolis + centrifugal correction for LVLH rotating frame.
 * In Hill frame rotating at n rad/s about z_LVLH:
 * v_inertial = v_LVLH + n × r + (centrifugal terms absorbed by CW)
 * n is dynamically computed from the target absolute orbit. */
 double n = mean_motion;
 rel_velocity->x += n * rel_position->y; /* Coriolis x-component */
 rel_velocity->y -= n * rel_position->x; /* Coriolis y-component */
}

// Implement Kalman filter update
void kalman_filter_update(Matrix *state, Matrix *covariance,
 Matrix *measurement, Matrix *measurement_noise,
 Matrix *state_transition, Matrix *process_noise) {
 // Predict step
 Matrix predicted_state, predicted_covariance;
 Matrix temp, temp2, state_transition_T;

 // Predict state: x_k|k-1 = F * x_k-1|k-1
 matrix_multiply(&predicted_state, state_transition, state);

 // Predict covariance: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
 matrix_transpose(&state_transition_T, state_transition);
 matrix_multiply(&temp, state_transition, covariance);
 matrix_multiply(&temp2, &temp, &state_transition_T);
 matrix_add(&predicted_covariance, &temp2, process_noise);

 // Update step
 // Using identity as measurement model (H)
 Matrix H, H_T, K_num, K_denom, K, innovation, temp3, temp4, I;
 matrix_identity(&H);
 matrix_transpose(&H_T, &H);

 // Compute Kalman gain: K = P_k|k-1 * H^T * (H * P_k|k-1 * H^T + R)^-1
 matrix_multiply(&temp, &H, &predicted_covariance);
 matrix_multiply(&temp2, &temp, &H_T);
 matrix_add(&K_denom, &temp2, measurement_noise);

 matrix_multiply(&K_num, &predicted_covariance, &H_T);

 Matrix K_denom_inv;
 if (matrix_inverse(&K_denom_inv, &K_denom) != 0) {
 // If matrix inversion fails, use a fallback approach
 matrix_identity(&K_denom_inv);
 matrix_scale(&K_denom_inv, &K_denom_inv, 0.001);
 }

 matrix_multiply(&K, &K_num, &K_denom_inv);

 // Update state: x_k|k = x_k|k-1 + K * (z - H * x_k|k-1)
 matrix_multiply(&temp, &H, &predicted_state);
 matrix_subtract(&innovation, measurement, &temp);
 matrix_multiply(&temp2, &K, &innovation);
 matrix_add(state, &predicted_state, &temp2);

 // Update covariance: P_k|k = (I - K * H) * P_k|k-1
 matrix_identity(&I);
 matrix_multiply(&temp3, &K, &H);
 matrix_subtract(&temp4, &I, &temp3);
 matrix_multiply(covariance, &temp4, &predicted_covariance);
}

// Compute approach trajectory using Clohessy-Wiltshire equations
void compute_approach_trajectory(SpacecraftState *chaser, SpacecraftState *target,
 double mean_motion, Vector3 *thrust_command) {

 // Compute relative position and velocity
 Vector3 rel_position = vector_subtract(chaser->position, target->position);
 Vector3 rel_velocity = vector_subtract(chaser->velocity, target->velocity);

 // Calculate distance to target
 double distance = vector_magnitude(rel_position);

 // Initialize thrust command to zero
 thrust_command->x = 0.0;
 thrust_command->y = 0.0;
 thrust_command->z = 0.0;

 // Only use Clohessy-Wiltshire for long distance approaches
 if (distance > 12.0) {
 // Use Clohessy-Wiltshire targeting for approach
 Vector3 desired_velocity = compute_clohessy_wiltshire(chaser, target, mean_motion);

 // Scale down desired velocity based on distance (slow down as we get closer)
 double velocity_scale = fmin(1.0, distance / 45.0);
 desired_velocity = vector_scale(desired_velocity, velocity_scale);

 // Compute velocity error
 Vector3 velocity_error = vector_subtract(desired_velocity, rel_velocity);

 /* PD controller: gains from ω_n=0.02 rad/s, ζ=0.85 design */
 double kp = -0.042; /* position gain (attractive) */
 double kd = 1.15; /* velocity damping gain */

 // Calculate thrust based on position and velocity errors
 thrust_command->x = kp * rel_position.x + kd * velocity_error.x;
 thrust_command->y = kp * rel_position.y + kd * velocity_error.y;
 thrust_command->z = kp * rel_position.z + kd * velocity_error.z;
 }
 // Close approach phase - precision control
 else {
 // Normalize position vector (direction to target)
 Vector3 approach_direction = vector_normalize(rel_position);

 // Get approach speed (negative means approaching)
 double approach_speed = vector_dot(rel_velocity, approach_direction);

 // Calculate optimal approach speed based on distance
 // Closer = slower approach (proportional to square root of distance)
 double optimal_approach_speed = -0.048 * sqrt(distance);

 /* Slow down aggressively inside keep-out sphere */
 if (distance < 4.5) {
 optimal_approach_speed = -0.018 * distance;
 }

 // Get lateral velocity (perpendicular to approach vector)
 Vector3 approach_vel_component = vector_scale(approach_direction, approach_speed);
 Vector3 lateral_velocity = vector_subtract(rel_velocity, approach_vel_component);

 /* Strong damping of lateral drift */
 Vector3 lateral_correction = vector_scale(lateral_velocity, -1.85);

 double approach_speed_error = optimal_approach_speed - approach_speed;
 Vector3 approach_correction = vector_scale(approach_direction, approach_speed_error * 0.48);

 /* Position hold: tighter gain inside keep-out zone */
 double position_gain = -0.022;
 if (distance < 4.5) {
 position_gain = -0.034;
 }
 Vector3 position_correction = vector_scale(rel_position, position_gain);

 // Combine all corrections
 thrust_command->x = lateral_correction.x + approach_correction.x + position_correction.x;
 thrust_command->y = lateral_correction.y + approach_correction.y + position_correction.y;
 thrust_command->z = lateral_correction.z + approach_correction.z + position_correction.z;

 /* Final braking inside capture envelope */
 if (distance < 1.8) {
 thrust_command->x = -rel_velocity.x * 1.95 - rel_position.x * 0.024;
 thrust_command->y = -rel_velocity.y * 1.95 - rel_position.y * 0.024;
 thrust_command->z = -rel_velocity.z * 1.95 - rel_position.z * 0.024;
 }

 if (distance < 4.5) {
 FLIGHT_LOG("Close approach: Dist=%.2fm, Opt_speed=%.4fm/s, Actual=%.4fm/s\n",
 distance, optimal_approach_speed, approach_speed);
 }
 }

 /* Limit thrust: 22 N thrusters on 1340 kg bus → a_max ≈ 0.0164 m/s² */
 double thrust_mag = vector_magnitude(*thrust_command);
 double max_thrust_accel = 22.0 / 1340.0; /* m/s² */

 if (thrust_mag > max_thrust_accel) {
 *thrust_command = vector_scale(*thrust_command, max_thrust_accel / thrust_mag);
 }
}

/* CW two-impulse targeting: compute departure velocity v0 such that
 * r(tf) = 0 under linearised Hill dynamics (Clohessy & Wiltshire, 1960).
 * Uses the closed-form inverse of the position-velocity STM sub-block. */
Vector3 compute_clohessy_wiltshire(SpacecraftState *chaser, SpacecraftState *target, double mean_motion) {
 Vector3 rel_pos = vector_subtract(chaser->position, target->position);
 Vector3 desired_vel;
 double n = mean_motion;

 /* Transfer time: scale with distance, clamp to [10 s, half orbit] */
 double dist = vector_magnitude(rel_pos);
 double tf = 4.0 * sqrt(dist);
 if (tf < 10.0) tf = 10.0;
 double half_period = M_PI / n;
 if (tf > half_period) tf = half_period;

 double nt = n * tf;
 double c = cos(nt), s = sin(nt);

 /* --- In-plane (x-y coupled) CW STM sub-blocks ---
 * Phi_rr = [[4-3c, 0], [6(s-nt), 1]]
 * Phi_rv = [[s/n, 2(1-c)/n], [2(c-1)/n, (4s-3nt)/n]]
 * We need v0 = Phi_rv^{-1} * (r_target - Phi_rr * r0) with r_target = 0 */
 double a11 = s / n, a12 = 2.0 * (1.0 - c) / n;
 double a21 = 2.0 * (c - 1.0) / n, a22 = (4.0 * s - 3.0 * nt) / n;
 double det = a11 * a22 - a12 * a21;
 if (fabs(det) < 1e-12) det = (det >= 0.0 ? 1e-12 : -1e-12);

 double bx = -(4.0 - 3.0 * c) * rel_pos.x;
 double by = -6.0 * (s - nt) * rel_pos.x - rel_pos.y;

 desired_vel.x = ( a22 * bx - a12 * by) / det;
 desired_vel.y = (-a21 * bx + a11 * by) / det;

 /* --- Out-of-plane (z decoupled) ---
 * z(t) = z0*cos(nt) + zd0*sin(nt)/n → for z(tf)=0: zd0 = -z0*n*c/s */
 if (fabs(s) > 1e-12)
 desired_vel.z = -rel_pos.z * n * c / s;
 else
 desired_vel.z = -rel_pos.z * n; /* fallback near nt = kπ */

 return desired_vel;
}

// Compute attitude control
void attitude_control(SpacecraftState *chaser, SpacecraftState *target,
 Vector3 *torque_command) {
 // Desired orientation: align with target
 Quaternion desired_orientation;

 // Compute direction vector to target
 Vector3 direction = vector_subtract(target->position, chaser->position);
 direction = vector_normalize(direction);

 // Create quaternion that aligns +X axis with the direction vector
 Vector3 x_axis = {1.0, 0.0, 0.0};
 Vector3 rotation_axis = vector_cross(x_axis, direction);

 if (vector_magnitude(rotation_axis) < 1e-6) {
 // Vectors are nearly parallel, use identity quaternion
 desired_orientation.w = 1.0;
 desired_orientation.x = 0.0;
 desired_orientation.y = 0.0;
 desired_orientation.z = 0.0;
 } else {
 rotation_axis = vector_normalize(rotation_axis);
 double angle = acos(fmax(-1.0, fmin(1.0, vector_dot(x_axis, direction))));
 desired_orientation = quaternion_from_axis_angle(rotation_axis, angle);
 }

 // Compute quaternion error
 Quaternion q_error = quaternion_multiply(desired_orientation,
 quaternion_conjugate(chaser->orientation));

 // Convert quaternion error to axis-angle representation
 double angle_error = 2.0 * acos(fmax(-1.0, fmin(1.0, q_error.w)));
 Vector3 axis_error;

 if (angle_error < 1e-6) {
 axis_error.x = 0.0;
 axis_error.y = 0.0;
 axis_error.z = 0.0;
 } else {
 double s = sqrt(1.0 - q_error.w * q_error.w);
 axis_error.x = q_error.x / s;
 axis_error.y = q_error.y / s;
 axis_error.z = q_error.z / s;
 axis_error = vector_scale(axis_error, angle_error);
 }

 /* PD gains: ω_n=0.72 rad/s, ζ=0.88 → kp≈0.52, kd≈1.27 */
 double kp_att = 0.518;
 double kd_att = 1.267;

 // Compute control torque
 *torque_command = vector_add(
 vector_scale(axis_error, kp_att),
 vector_scale(chaser->angular_velocity, -kd_att)
 );

 double torque_mag = vector_magnitude(*torque_command);
 double max_torque = 0.048; /* N·m (reaction wheel limit) */

 if (torque_mag > max_torque) {
 *torque_command = vector_scale(*torque_command, max_torque / torque_mag);
 }
}

// Check if docking conditions are met
int check_docking_conditions(SpacecraftState *chaser, SpacecraftState *target) {
 // Compute relative position and velocity
 Vector3 rel_position = vector_subtract(chaser->position, target->position);
 Vector3 rel_velocity = vector_subtract(chaser->velocity, target->velocity);

 double distance = vector_magnitude(rel_position);

 /* Approach velocity component along the line-of-sight */
 Vector3 approach_vector = vector_normalize(rel_position);
 double approach_velocity = -vector_dot(rel_velocity, approach_vector);

 // Check alignment (dot product of X axes)
 Vector3 chaser_x_axis = quaternion_rotate_vector(chaser->orientation, (Vector3){1.0, 0.0, 0.0});
 Vector3 target_x_axis = quaternion_rotate_vector(target->orientation, (Vector3){1.0, 0.0, 0.0});
 double alignment = vector_dot(chaser_x_axis, target_x_axis);

 Vector3 approach_component = vector_scale(approach_vector, -approach_velocity);
 Vector3 lateral_velocity = vector_subtract(rel_velocity, approach_component);
 double lateral_speed = vector_magnitude(lateral_velocity);

 if (distance < 4.5) {
 FLIGHT_LOG("Docking check: Distance=%.2fm, Approach vel=%.4fm/s, Lateral vel=%.4fm/s, Alignment=%.3f\n",
 distance, approach_velocity, lateral_speed, alignment);
 }

 /* SPADEX capture box: 0.5 m, 0.05 m/s axial, 0.03 m/s lateral */
 if (distance < DOCKING_DISTANCE_THRESHOLD &&
 fabs(approach_velocity) < 0.045 &&
 lateral_speed < 0.032 &&
 alignment > cos(DOCKING_ANGLE_THRESHOLD)) {

 FLIGHT_LOG("[RVD] DOCK_OK\n");
 FLIGHT_LOG("Final state: Distance=%.2fm, Approach vel=%.4fm/s, Lateral vel=%.4fm/s, Alignment=%.3f\n",
 distance, approach_velocity, lateral_speed, alignment);

 return 1; // Docking conditions met
 }

 return 0; // Docking conditions not met
}


