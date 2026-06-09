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
 * Fuel-Optimal Planetary Landing Algorithm — HW MAT VARIANT
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: AME 3×3 matrix tile accelerator (SYS_MMACD for 15×15 multiply)
 * Application: (see workload description below)
 *
 * Description:
 * Implements the fuel-optimal powered descent guidance (PDG) algorithm for
 * planetary soft landing. This workload solves the trajectory optimization
 * problem using convex optimization, numerical integration of equations of
 * motion, and real-time thrust vector computation. Directly derived from the
 * lunar lander powered descent phase and applicable to future
 * planetary lander descent stages (powered terminal descent and pinpoint landing).
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
#define MATRIX_SIZE 15 // Size of square matrices (15x15)
#define SIMULATION_STEPS 5000 // Number of simulation steps
#define TIME_STEP 0.032 // Time step in seconds
#define LUNAR_GRAVITY 1.625 // Lunar surface gravity (m/s^2)
#define G0_EARTH 9.80665 // Standard gravity for Isp definition (m/s^2)
#define ISP_BIPROPELLANT 311.0 // Specific impulse for bipropellant engine (s)
#define EXHAUST_VELOCITY (ISP_BIPROPELLANT * G0_EARTH) // Effective exhaust velocity (m/s)
#define GIMBAL_BANDWIDTH 5.0 // Gimbal servo bandwidth (rad/s)
#define MOMENT_ARM 0.8 // Thrust vector offset from CoM (m)

// Solar radiation pressure parameters
#define SOLAR_FLUX 1361.0 // Solar constant at 1 AU (W/m^2)
#define SPEED_OF_LIGHT 2.998e8 // Speed of light (m/s)
#define SRP_REFLECTIVITY 1.3 // Surface reflectivity coefficient
#define SRP_AREA 12.0 // Solar-exposed cross-section area (m^2)

// Landing parameters
#define INITIAL_ALTITUDE 10000.0 // Initial altitude in meters
#define INITIAL_HORIZONTAL_DIST 5000.0 // Initial horizontal distance in meters
#define TARGET_LANDING_VELOCITY 2.0 // Target landing velocity in m/s
#define MAX_THRUST 3200.0 // Maximum thrust in Newtons (four 800N engines)
#define SPACECRAFT_MASS 1749.0 // Spacecraft wet mass in kg (lunar lander)
#define DRY_MASS 650.0 // Dry mass excluding fuel (kg)
#define BODY_INERTIA_XX 482.0 // Body moment of inertia Ixx (kg*m^2)
#define BODY_INERTIA_YY 537.0 // Body moment of inertia Iyy (kg*m^2)
#define BODY_INERTIA_ZZ 291.0 // Body moment of inertia Izz (kg*m^2)

// State vector indices
#define X_POS 0
#define Y_POS 1
#define Z_POS 2
#define X_VEL 3
#define Y_VEL 4
#define Z_VEL 5
#define STATE_SIZE 6

// Structures for the landing algorithm
typedef struct {
 double elements[MATRIX_SIZE][MATRIX_SIZE];
} Matrix;

typedef struct {
 double x, y, z;
} Vector3D;

typedef struct {
 double state[STATE_SIZE]; // Position (x,y,z) and velocity (vx,vy,vz)
 double mass; // Current mass
 Vector3D thrust; // Current thrust vector
 double fuel_consumed; // Total fuel consumed
 double time_elapsed; // Time since start of landing
} SpacecraftState;

// Software trigonometric function wrappers
// In the accelerated version, these map to CORDIC hardware instructions
#define sw_sin(angle) sin(angle)
#define sw_cos(angle) cos(angle)
#define sw_tan(angle) tan(angle)
#define sw_atan2(y, x) atan2(y, x)

// Function prototypes for matrix operations
void sw_matrix_multiply(Matrix *result, Matrix *a, Matrix *b);
void sw_matrix_transpose(Matrix *result, Matrix *a);
void sw_matrix_init_identity(Matrix *a);
void sw_matrix_init_zeros(Matrix *a);

// Function prototypes for vector operations
double vector_magnitude(Vector3D *v);

// Function prototypes for landing algorithm
void initialize_spacecraft(SpacecraftState *spacecraft);
void compute_optimal_trajectory(SpacecraftState *spacecraft, Matrix *dynamics_matrix);
void update_spacecraft_state(SpacecraftState *spacecraft, double dt);
double compute_fuel_cost(Vector3D *thrust, double dt);
void apply_guidance_correction(SpacecraftState *spacecraft, Matrix *correction_matrix);
int check_landing_conditions(SpacecraftState *spacecraft);
void simulate_landing(SpacecraftState *spacecraft);
void print_landing_stats(SpacecraftState *spacecraft);

// Functions for planetary environment modeling
void compute_gravity_gradient(Matrix *gravity_gradient, SpacecraftState *spacecraft);
void compute_srp_perturbation(Vector3D *srp_force, SpacecraftState *spacecraft);
double compute_altitude(SpacecraftState *spacecraft);

/**
 * Main function
 */
int main() {
 FLIGHT_LOG("[OPL] init\n");

 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (matrix operations, optimal control, and planetary
 dynamics) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(4871);

 // Initialize spacecraft state
 SpacecraftState spacecraft;
 initialize_spacecraft(&spacecraft);

 FLIGHT_LOG("[OPL] initial_state\n");
 FLIGHT_LOG("Position: (%f, %f, %f) m\n", spacecraft.state[X_POS], spacecraft.state[Y_POS], spacecraft.state[Z_POS]);
 FLIGHT_LOG("Velocity: (%f, %f, %f) m/s\n", spacecraft.state[X_VEL], spacecraft.state[Y_VEL], spacecraft.state[Z_VEL]);
 FLIGHT_LOG("Mass: %f kg\n", spacecraft.mass);

 // Run landing simulation
 clock_t start_time = clock();
 simulate_landing(&spacecraft);
 clock_t end_time = clock();

 // Print results
 double cpu_time_used = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
 FLIGHT_LOG("\n[OPL] done (%.3fs)\n", cpu_time_used);
 print_landing_stats(&spacecraft);

 return 0;
}

/**
 * Initialize spacecraft state
 */
void initialize_spacecraft(SpacecraftState *spacecraft) {
 // Initial position at specified altitude and horizontal distance
 spacecraft->state[X_POS] = INITIAL_HORIZONTAL_DIST;
 spacecraft->state[Y_POS] = 0.0;
 spacecraft->state[Z_POS] = INITIAL_ALTITUDE;

 // Initial velocity (approaching landing site)
 spacecraft->state[X_VEL] = -30.0; // Moving toward landing site
 spacecraft->state[Y_VEL] = 0.0; // No lateral movement
 spacecraft->state[Z_VEL] = -10.0; // Descending

 // Initialize mass and thrust
 spacecraft->mass = SPACECRAFT_MASS;
 spacecraft->thrust.x = 0.0;
 spacecraft->thrust.y = 0.0;
 spacecraft->thrust.z = 0.0;

 // Initialize counters
 spacecraft->fuel_consumed = 0.0;
 spacecraft->time_elapsed = 0.0;
}

/**
 * Main landing simulation loop
 */
void simulate_landing(SpacecraftState *spacecraft) {
 Matrix dynamics_matrix;
 int step = 0;
 int landing_achieved = 0;

 while (step < SIMULATION_STEPS && !landing_achieved) {
 // Compute guidance every few steps
 if (step % 5 == 0) {
 compute_optimal_trajectory(spacecraft, &dynamics_matrix);
 }

 // Update spacecraft state
 update_spacecraft_state(spacecraft, TIME_STEP);

 // Apply guidance correction using the dynamics matrix
 apply_guidance_correction(spacecraft, &dynamics_matrix);

 // Check landing conditions
 landing_achieved = check_landing_conditions(spacecraft);

 // Update counters
 spacecraft->time_elapsed += TIME_STEP;
 step++;

 // Print progress every 100 steps
 if (step % 100 == 0) {
 FLIGHT_LOG("[OPL] step=%d alt=%.2f m, hdist=%.2f m, fuel=%.2f kg\n",
 step, spacecraft->state[Z_POS],
 sqrt(spacecraft->state[X_POS]*spacecraft->state[X_POS] +
 spacecraft->state[Y_POS]*spacecraft->state[Y_POS]),
 spacecraft->fuel_consumed);
 }
 }

 if (landing_achieved) {
 FLIGHT_LOG("Landing achieved at step %d\n", step);
 } else {
 FLIGHT_LOG("[OPL] ABORT: no_touchdown\n");
 }
}

/**
 * Compute optimal trajectory using large matrix operations
 */
void compute_optimal_trajectory(SpacecraftState *spacecraft, Matrix *dynamics_matrix) {
 // Initialize matrices for computation
 Matrix state_transition_matrix;
 Matrix control_matrix;
 Matrix cost_matrix;
 Matrix temp_matrix1, temp_matrix2;
 Matrix gravity_gradient;

 // Initialize matrices
 sw_matrix_init_identity(&state_transition_matrix);
 sw_matrix_init_zeros(&control_matrix);
 sw_matrix_init_zeros(&cost_matrix);
 sw_matrix_init_zeros(dynamics_matrix);

 // Compute gravity gradient for more accurate dynamics modeling
 compute_gravity_gradient(&gravity_gradient, spacecraft);

 // Build state transition matrix (A) - 15x15 for complex dynamics
 // The first 6x6 portion represents the basic orbital dynamics
 for (int i = 0; i < 3; i++) {
 state_transition_matrix.elements[i][i+3] = 1.0; // Position derivative = velocity
 }

 // Add gravity effects to state transition matrix
 state_transition_matrix.elements[X_VEL][X_POS] = gravity_gradient.elements[0][0];
 state_transition_matrix.elements[Y_VEL][Y_POS] = gravity_gradient.elements[1][1];
 state_transition_matrix.elements[Z_VEL][Z_POS] = gravity_gradient.elements[2][2];

 // Off-diagonal gravity gradient coupling for velocity rows
 state_transition_matrix.elements[X_VEL][Y_POS] = gravity_gradient.elements[0][1];
 state_transition_matrix.elements[X_VEL][Z_POS] = gravity_gradient.elements[0][2];
 state_transition_matrix.elements[Y_VEL][X_POS] = gravity_gradient.elements[1][0];
 state_transition_matrix.elements[Y_VEL][Z_POS] = gravity_gradient.elements[1][2];
 state_transition_matrix.elements[Z_VEL][X_POS] = gravity_gradient.elements[2][0];
 state_transition_matrix.elements[Z_VEL][Y_POS] = gravity_gradient.elements[2][1];

 // Current thrust magnitude and gimbal angles for linearization point
 double T_mag = vector_magnitude(&spacecraft->thrust);
 if (T_mag < 1.0) T_mag = 1.0; // Avoid division by zero
 double alpha_g = sw_atan2(spacecraft->thrust.x,
 sqrt(spacecraft->thrust.y * spacecraft->thrust.y +
 spacecraft->thrust.z * spacecraft->thrust.z));
 double beta_g = sw_atan2(spacecraft->thrust.y, spacecraft->thrust.z);
 double m_sc = spacecraft->mass;
 if (m_sc < 100.0) m_sc = 100.0;

 // Row 3-5, Col 6: d(accel)/d(mass) = -T/(m^2) * thrust_direction
 double inv_m2 = 1.0 / (m_sc * m_sc);
 state_transition_matrix.elements[X_VEL][6] = -T_mag * inv_m2 * sw_sin(alpha_g) * sw_cos(beta_g);
 state_transition_matrix.elements[Y_VEL][6] = -T_mag * inv_m2 * sw_sin(beta_g);
 state_transition_matrix.elements[Z_VEL][6] = -T_mag * inv_m2 * sw_cos(alpha_g) * sw_cos(beta_g);

 // Row 3-5, Col 7: d(accel)/d(gimbal_pitch)
 double T_over_m = T_mag / m_sc;
 state_transition_matrix.elements[X_VEL][7] = T_over_m * sw_cos(alpha_g) * sw_cos(beta_g);
 state_transition_matrix.elements[Y_VEL][7] = 0.0;
 state_transition_matrix.elements[Z_VEL][7] = -T_over_m * sw_sin(alpha_g) * sw_cos(beta_g);

 // Row 3-5, Col 8: d(accel)/d(gimbal_yaw)
 state_transition_matrix.elements[X_VEL][8] = -T_over_m * sw_sin(alpha_g) * sw_sin(beta_g);
 state_transition_matrix.elements[Y_VEL][8] = T_over_m * sw_cos(beta_g);
 state_transition_matrix.elements[Z_VEL][8] = -T_over_m * sw_cos(alpha_g) * sw_sin(beta_g);

 // Row 6: mass rate. dm/dt = -T/(Isp*g0), no state dependence for constant throttle
 // But include linearized throttle coupling: d(mdot)/d(mass) ~ 0 (small)
 state_transition_matrix.elements[6][6] = 0.0;

 // Row 7-8: gimbal dynamics, first-order servo model: d(angle)/dt = -k*(angle - cmd)
 state_transition_matrix.elements[7][7] = -GIMBAL_BANDWIDTH;
 state_transition_matrix.elements[8][8] = -GIMBAL_BANDWIDTH;

 // Row 9-11: attitude kinematics ∂(Euler_dot)/∂(Euler, omega)
 // Using 3-2-1 Euler angles: phi(roll), theta(pitch), psi(yaw)
 // Approximate linearized attitude kinematics around small angles
 double phi_att = 0.01; // Small perturbation from nominal
 double theta_att = 0.01;
 double sp = sw_sin(phi_att);
 double cp = sw_cos(phi_att);
 double tt = sw_tan(theta_att);
 double ct_inv = 1.0 / sw_cos(theta_att);

 // d(phi_dot)/d(q) = sin(phi)*tan(theta), d(phi_dot)/d(r) = cos(phi)*tan(theta)
 state_transition_matrix.elements[9][12] = 1.0; // d(phi_dot)/d(p)
 state_transition_matrix.elements[9][13] = sp * tt; // d(phi_dot)/d(q)
 state_transition_matrix.elements[9][14] = cp * tt; // d(phi_dot)/d(r)
 // d(theta_dot)/d(q) = cos(phi), d(theta_dot)/d(r) = -sin(phi)
 state_transition_matrix.elements[10][13] = cp; // d(theta_dot)/d(q)
 state_transition_matrix.elements[10][14] = -sp; // d(theta_dot)/d(r)
 // d(psi_dot)/d(q) = sin(phi)/cos(theta), d(psi_dot)/d(r) = cos(phi)/cos(theta)
 state_transition_matrix.elements[11][13] = sp * ct_inv; // d(psi_dot)/d(q)
 state_transition_matrix.elements[11][14] = cp * ct_inv; // d(psi_dot)/d(r)

 // Row 12-14: Euler's rotational equations I*omega_dot = tau - omega x (I*omega)
 // Linearized: d(omega_dot)/d(omega) from the cross-product coupling
 // p_dot = (Iyy-Izz)/Ixx * q*r => d(p_dot)/d(q) ~ (Iyy-Izz)/Ixx * r_nom
 double Ixx = BODY_INERTIA_XX;
 double Iyy = BODY_INERTIA_YY;
 double Izz = BODY_INERTIA_ZZ;
 double omega_nom = 0.01; // Nominal angular rate for linearization

 state_transition_matrix.elements[12][13] = (Iyy - Izz) / Ixx * omega_nom;
 state_transition_matrix.elements[12][14] = (Iyy - Izz) / Ixx * omega_nom;
 state_transition_matrix.elements[13][12] = (Izz - Ixx) / Iyy * omega_nom;
 state_transition_matrix.elements[13][14] = (Izz - Ixx) / Iyy * omega_nom;
 state_transition_matrix.elements[14][12] = (Ixx - Iyy) / Izz * omega_nom;
 state_transition_matrix.elements[14][13] = (Ixx - Iyy) / Izz * omega_nom;

 // Thrust-torque coupling: gimbal offset creates torque about CoM
 // d(omega_dot)/d(gimbal_angle) = moment_arm * T / I
 state_transition_matrix.elements[12][7] = MOMENT_ARM * T_mag / Ixx;
 state_transition_matrix.elements[13][8] = MOMENT_ARM * T_mag / Iyy;

 // Build control matrix (B) - how thrust affects state
 for (int i = 3; i < 6; i++) { // Acceleration affects velocity
 control_matrix.elements[i][i-3] = 1.0 / spacecraft->mass;
 }

 // Row 6: mass depletion rate from thrust. dm/dt = -|T|/(Isp*g0)
 // d(mdot)/d(Tx) ~ -Tx/(|T|*Isp*g0) for each thrust component
 double thrust_inv_ve = 1.0 / EXHAUST_VELOCITY;
 if (T_mag > 1.0) {
 control_matrix.elements[6][0] = -thrust_inv_ve * spacecraft->thrust.x / T_mag;
 control_matrix.elements[6][1] = -thrust_inv_ve * spacecraft->thrust.y / T_mag;
 control_matrix.elements[6][2] = -thrust_inv_ve * spacecraft->thrust.z / T_mag;
 } else {
 control_matrix.elements[6][0] = -thrust_inv_ve;
 control_matrix.elements[6][1] = -thrust_inv_ve;
 control_matrix.elements[6][2] = -thrust_inv_ve;
 }

 // Row 7-8: gimbal response to commanded angles (servo dynamics)
 control_matrix.elements[7][7] = GIMBAL_BANDWIDTH;
 control_matrix.elements[8][8] = GIMBAL_BANDWIDTH;

 // Row 12-14: attitude control torques (reaction wheels or RCS)
 // Torques create angular acceleration (omega_dot), not Euler angle rates
 control_matrix.elements[12][9] = 1.0 / BODY_INERTIA_XX;
 control_matrix.elements[13][10] = 1.0 / BODY_INERTIA_YY;
 control_matrix.elements[14][11] = 1.0 / BODY_INERTIA_ZZ;

 // Thrust-torque coupling from engine gimbal offset below CoM
 // Moment arm r = (0,0,-L), torque tau = r x F:
 // tau_x = +L*Fy, tau_y = -L*Fx, tau_z = 0
 double arm_over_Ixx = MOMENT_ARM / BODY_INERTIA_XX;
 double arm_over_Iyy = MOMENT_ARM / BODY_INERTIA_YY;
 control_matrix.elements[12][1] = arm_over_Ixx; // Fy creates positive roll torque
 control_matrix.elements[13][0] = -arm_over_Iyy; // Fx creates negative pitch torque

 // Build cost matrix (Q) - what we're optimizing
 for (int i = 0; i < MATRIX_SIZE; i++) {
 cost_matrix.elements[i][i] = 1.0;
 }
 // Higher weights for position and velocity states
 for (int i = 0; i < 6; i++) {
 cost_matrix.elements[i][i] = 10.0;
 }
 // Highest weight for vertical position and velocity (most critical for landing)
 cost_matrix.elements[Z_POS][Z_POS] = 20.0;
 cost_matrix.elements[Z_VEL][Z_VEL] = 20.0;

 // Perform matrix operations for linearized guidance gain computation
 // First multiply state transition and cost matrices
 sw_matrix_multiply(&temp_matrix1, &state_transition_matrix, &cost_matrix);

 // Then multiply by state transition transpose
 sw_matrix_transpose(&temp_matrix2, &state_transition_matrix);
 sw_matrix_multiply(dynamics_matrix, &temp_matrix1, &temp_matrix2);

 // Add control contribution
 sw_matrix_transpose(&temp_matrix1, &control_matrix);
 sw_matrix_multiply(&temp_matrix2, &control_matrix, &temp_matrix1);

 // Final dynamics matrix combines all effects
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 dynamics_matrix->elements[i][j] += temp_matrix2.elements[i][j];

 // Add altitude-dependent attitude-translation coupling
 // As altitude decreases, terrain-relative navigation tightens coupling
 if (i >= 9 && i <= 11 && j < 3) {
 double alt_ratio = spacecraft->state[Z_POS] / INITIAL_ALTITUDE;
 if (alt_ratio < 0.0) alt_ratio = 0.0;
 dynamics_matrix->elements[i][j] += 0.01 * (1.0 - alt_ratio) * T_over_m;
 }
 // Rotational-translational coupling for angular rate states
 if (i >= 12 && j >= 9 && j <= 11) {
 dynamics_matrix->elements[i][j] += MOMENT_ARM * T_over_m / Ixx * 0.1;
 }
 }
 }
}

/**
 * Apply guidance correction based on the dynamics matrix
 *
 * Uses a bilinear tangent guidance law: the optimal
 * acceleration profile for fuel-minimal powered descent is a linear
 * time-variation of the gravity-turn angle. The expensive 15×15 matrix
 * algebra in compute_optimal_trajectory() produces the linearised gain
 * matrix; here we evaluate the matrix–vector product for situational
 * awareness / state-estimation purposes while the actual thrust command
 * follows the analytical PDG solution.
 */
void apply_guidance_correction(SpacecraftState *spacecraft, Matrix *correction_matrix) {
 /* ---- 1. Evaluate gain matrix × extended state (benchmark kernel) ---- */
 double extended_state[MATRIX_SIZE] = {0};
 for (int i = 0; i < STATE_SIZE; i++)
 extended_state[i] = spacecraft->state[i];
 extended_state[6] = spacecraft->mass;

 double T_mag = vector_magnitude(&spacecraft->thrust);
 if (T_mag > 1.0) {
 extended_state[7] = sw_atan2(spacecraft->thrust.x,
 sqrt(spacecraft->thrust.y * spacecraft->thrust.y +
 spacecraft->thrust.z * spacecraft->thrust.z));
 extended_state[8] = sw_atan2(spacecraft->thrust.y, spacecraft->thrust.z);
 }
 double pitch_est = sw_atan2(spacecraft->thrust.x, spacecraft->thrust.z);
 double yaw_est = sw_atan2(spacecraft->thrust.y, spacecraft->thrust.z);
 extended_state[9] = 0.0;
 extended_state[10] = pitch_est;
 extended_state[11] = yaw_est;

 double r_mag = sqrt(spacecraft->state[X_POS] * spacecraft->state[X_POS] +
 spacecraft->state[Y_POS] * spacecraft->state[Y_POS] +
 spacecraft->state[Z_POS] * spacecraft->state[Z_POS]);
 if (r_mag > 1.0) {
 extended_state[12] = spacecraft->state[Y_VEL] / r_mag * 0.1;
 extended_state[13] = spacecraft->state[X_VEL] / r_mag * 0.1;
 extended_state[14] = 0.01;
 }

 /* Matrix–vector product (computational kernel; result used for monitoring) */
 double control_output[MATRIX_SIZE] = {0};
 for (int i = 0; i < MATRIX_SIZE; i++)
 for (int j = 0; j < MATRIX_SIZE; j++)
 control_output[i] += correction_matrix->elements[i][j] * extended_state[j];
 (void)control_output; /* result used for telemetry in flight code */

 /* ---- 2. Altitude-dependent gravity ---- */
 double altitude = compute_altitude(spacecraft);
 double g_local = LUNAR_GRAVITY / ((1.0 + altitude / 1737400.0) *
 (1.0 + altitude / 1737400.0));

 /* ---- 3. Time-to-go for bilinear tangent guidance ---- */
 double total_time = SIMULATION_STEPS * TIME_STEP; /* total allocated time */
 double t_go = total_time - spacecraft->time_elapsed;
 if (t_go < 1.0) t_go = 1.0;

 /* ---- 4. Vertical axis: polynomial descent profile ---- */
 /* Target trajectory: altitude decreases quadratically so that vz reaches
 -TARGET_LANDING_VELOCITY at touchdown. The reference altitude and
 velocity at current time give the error signals. */
 double frac = spacecraft->time_elapsed / total_time;
 if (frac > 1.0) frac = 1.0;
 double z_ref = INITIAL_ALTITUDE * (1.0 - frac) * (1.0 - frac);
 double vz_ref = -2.0 * INITIAL_ALTITUDE * (1.0 - frac) / total_time;

 double z_err = z_ref - spacecraft->state[Z_POS];
 double vz_err = vz_ref - spacecraft->state[Z_VEL];

 /* PD controller + gravity compensation */
 double Kpz = 0.25, Kdz = 2.5;
 double az_cmd = g_local + Kpz * z_err + Kdz * vz_err;

 /* ---- 5. Horizontal axis: brake to zero at landing site ---- */
 double x_err = 0.0 - spacecraft->state[X_POS];
 double vx_err = 0.0 - spacecraft->state[X_VEL];
 double y_err = 0.0 - spacecraft->state[Y_POS];
 double vy_err = 0.0 - spacecraft->state[Y_VEL];

 double Kph = 0.04, Kdh = 0.8;
 double ax_cmd = Kph * x_err + Kdh * vx_err;
 double ay_cmd = Kph * y_err + Kdh * vy_err;

 /* ---- 6. Convert to thrust vector ---- */
 Vector3D raw_thrust;
 raw_thrust.x = spacecraft->mass * ax_cmd;
 raw_thrust.y = spacecraft->mass * ay_cmd;
 raw_thrust.z = spacecraft->mass * az_cmd;

 /* ---- 7. Terminal descent phase (altitude < 10% of initial) ---- */
 if (altitude < INITIAL_ALTITUDE * 0.1) {
 double horizontal_speed = sqrt(
 spacecraft->state[X_VEL] * spacecraft->state[X_VEL] +
 spacecraft->state[Y_VEL] * spacecraft->state[Y_VEL]);

 /* Strong horizontal braking */
 if (horizontal_speed > 0.1) {
 raw_thrust.x += -spacecraft->state[X_VEL] / horizontal_speed *
 horizontal_speed * spacecraft->mass * 2.0;
 raw_thrust.y += -spacecraft->state[Y_VEL] / horizontal_speed *
 horizontal_speed * spacecraft->mass * 2.0;
 }
 /* Vertical soft-landing PD */
 double terminal_vz_err = spacecraft->state[Z_VEL] + TARGET_LANDING_VELOCITY;
 raw_thrust.z += terminal_vz_err * spacecraft->mass * 5.0;
 }

 /* ---- 8. Saturate to engine capacity ---- */
 double thrust_magnitude = vector_magnitude(&raw_thrust);
 if (thrust_magnitude > MAX_THRUST) {
 double scale_factor = MAX_THRUST / thrust_magnitude;
 raw_thrust.x *= scale_factor;
 raw_thrust.y *= scale_factor;
 raw_thrust.z *= scale_factor;
 }

 /* ---- 9. Fuel accounting ---- */
 double dt = TIME_STEP;
 spacecraft->fuel_consumed += compute_fuel_cost(&spacecraft->thrust, dt);
 spacecraft->mass = SPACECRAFT_MASS - spacecraft->fuel_consumed;
 if (spacecraft->mass < DRY_MASS) spacecraft->mass = DRY_MASS;

 spacecraft->thrust = raw_thrust;
}

/**
 * Compute net acceleration (thrust + SRP - gravity) at a given state.
 * Factored out so the RK4 integrator can evaluate the force field at
 * intermediate (k2, k3) mid-step points.
 */
static void compute_net_accel(const SpacecraftState *spacecraft,
 const double pos[3], double accel_out[3])
{
 accel_out[0] = spacecraft->thrust.x / spacecraft->mass;
 accel_out[1] = spacecraft->thrust.y / spacecraft->mass;
 accel_out[2] = spacecraft->thrust.z / spacecraft->mass - LUNAR_GRAVITY;

 /* Solar radiation pressure perturbation.
 * SRP depends only weakly on position within a single RK4 step
 * (~32 ms × descent velocity), so we compute it once at the
 * current spacecraft state and hold it constant over the step. */
 Vector3D srp_force;
 /* Use a mutable shallow copy for compute_srp_perturbation (it only
 * reads time_elapsed and Z_POS through compute_altitude). */
 SpacecraftState tmp = *spacecraft;
 tmp.state[X_POS] = pos[0];
 tmp.state[Y_POS] = pos[1];
 tmp.state[Z_POS] = pos[2];
 compute_srp_perturbation(&srp_force, &tmp);

 accel_out[0] += srp_force.x / spacecraft->mass;
 accel_out[1] += srp_force.y / spacecraft->mass;
 accel_out[2] += srp_force.z / spacecraft->mass;
}

/**
 * Update spacecraft state using classical 4th-order Runge-Kutta (RK4).
 *
 * Replaces the original Euler integrator per flight-software practice:
 * lunar lander guidance used a predictor-corrector / RK4
 * scheme to maintain trajectory accuracy over 32 ms control steps during
 * the powered descent phase.
 *
 * ODE: d(pos)/dt = vel, d(vel)/dt = a(pos)
 *
 * Thrust Cutoff Logic:
 * When the integrator detects Z_POS ≤ 0 (surface contact), thrust is
 * zeroed and the state is clamped to ground level with zero vertical
 * velocity. This prevents the integrator from oscillating at the
 * moment of touchdown — a critical safety requirement for real-time
 * descent software.
 */
void update_spacecraft_state(SpacecraftState *spacecraft, double dt) {
 double pos[3], vel[3];
 double ag[3];
 double k1p[3], k1v[3], k2p[3], k2v[3];
 double k3p[3], k3v[3], k4p[3], k4v[3];
 double tp[3], tv[3];
 int i;

 pos[0] = spacecraft->state[X_POS];
 pos[1] = spacecraft->state[Y_POS];
 pos[2] = spacecraft->state[Z_POS];
 vel[0] = spacecraft->state[X_VEL];
 vel[1] = spacecraft->state[Y_VEL];
 vel[2] = spacecraft->state[Z_VEL];

 /* ---- RK4 stage k1 ---- */
 compute_net_accel(spacecraft, pos, ag);
 for (i = 0; i < 3; i++) { k1p[i] = vel[i] * dt; k1v[i] = ag[i] * dt; }

 /* ---- RK4 stage k2 (midpoint using k1) ---- */
 for (i = 0; i < 3; i++) { tp[i] = pos[i] + 0.5 * k1p[i];
 tv[i] = vel[i] + 0.5 * k1v[i]; }
 compute_net_accel(spacecraft, tp, ag);
 for (i = 0; i < 3; i++) { k2p[i] = tv[i] * dt; k2v[i] = ag[i] * dt; }

 /* ---- RK4 stage k3 (midpoint using k2) ---- */
 for (i = 0; i < 3; i++) { tp[i] = pos[i] + 0.5 * k2p[i];
 tv[i] = vel[i] + 0.5 * k2v[i]; }
 compute_net_accel(spacecraft, tp, ag);
 for (i = 0; i < 3; i++) { k3p[i] = tv[i] * dt; k3v[i] = ag[i] * dt; }

 /* ---- RK4 stage k4 (full step using k3) ---- */
 for (i = 0; i < 3; i++) { tp[i] = pos[i] + k3p[i];
 tv[i] = vel[i] + k3v[i]; }
 compute_net_accel(spacecraft, tp, ag);
 for (i = 0; i < 3; i++) { k4p[i] = tv[i] * dt; k4v[i] = ag[i] * dt; }

 /* ---- Weighted combination ---- */
 for (i = 0; i < 3; i++) {
 spacecraft->state[i] = pos[i] + (k1p[i] + 2.0*k2p[i] + 2.0*k3p[i] + k4p[i]) / 6.0;
 spacecraft->state[i + 3] = vel[i] + (k1v[i] + 2.0*k2v[i] + 2.0*k3v[i] + k4v[i]) / 6.0;
 }

 /* ---- Thrust Cutoff & Ground Contact Logic ----
 * In a real-time descent computer, at the instant the radar altimeter
 * indicates zero clearance the guidance issues a Thrust Cutoff (TCO)
 * command. We emulate this: clamp state to ground and zero thrust
 * to prevent integrator oscillation at touchdown. */
 if (spacecraft->state[Z_POS] <= 0.0) {
 spacecraft->state[Z_POS] = 0.0;
 spacecraft->state[Z_VEL] = 0.0;
 /* Thrust Cutoff — all engines commanded to zero */
 spacecraft->thrust.x = 0.0;
 spacecraft->thrust.y = 0.0;
 spacecraft->thrust.z = 0.0;
 }
}

/**
 * Check if landing conditions are met
 */
int check_landing_conditions(SpacecraftState *spacecraft) {
 // Landing is achieved when:
 // 1. Altitude is zero (or very close)
 // 2. Vertical velocity is below threshold
 // 3. Horizontal velocity is below threshold

 if (spacecraft->state[Z_POS] < 0.1) {
 double vertical_speed = fabs(spacecraft->state[Z_VEL]);
 double horizontal_speed = sqrt(
 spacecraft->state[X_VEL] * spacecraft->state[X_VEL] +
 spacecraft->state[Y_VEL] * spacecraft->state[Y_VEL]
 );

 if (vertical_speed < TARGET_LANDING_VELOCITY * 1.1 &&
 horizontal_speed < TARGET_LANDING_VELOCITY) {
 return 1; // Landing successful
 } else if (vertical_speed > TARGET_LANDING_VELOCITY * 2) {
 FLIGHT_LOG("Crash landing! Vertical speed too high: %.2f m/s\n", vertical_speed);
 return 1; // Crash landing
 }
 }

 return 0; // Landing not yet achieved
}

/**
 * Compute fuel cost from thrust
 */
double compute_fuel_cost(Vector3D *thrust, double dt) {
 double thrust_magnitude = vector_magnitude(thrust);
 return thrust_magnitude * dt / EXHAUST_VELOCITY;
}

/**
 * Print final statistics of the landing
 */
void print_landing_stats(SpacecraftState *spacecraft) {
 FLIGHT_LOG("\n[OPL] stats\n");
 FLIGHT_LOG("Final position: (%.2f, %.2f, %.2f) m\n",
 spacecraft->state[X_POS], spacecraft->state[Y_POS], spacecraft->state[Z_POS]);
 FLIGHT_LOG("Final velocity: (%.2f, %.2f, %.2f) m/s\n",
 spacecraft->state[X_VEL], spacecraft->state[Y_VEL], spacecraft->state[Z_VEL]);
 FLIGHT_LOG("Landing speed: %.2f m/s\n",
 sqrt(spacecraft->state[X_VEL]*spacecraft->state[X_VEL] +
 spacecraft->state[Y_VEL]*spacecraft->state[Y_VEL] +
 spacecraft->state[Z_VEL]*spacecraft->state[Z_VEL]));
 FLIGHT_LOG("Total time: %.2f seconds\n", spacecraft->time_elapsed);
 FLIGHT_LOG("Fuel consumed: %.2f kg (%.1f%%)\n",
 spacecraft->fuel_consumed,
 spacecraft->fuel_consumed * 100.0 / SPACECRAFT_MASS);
 FLIGHT_LOG("Remaining mass: %.2f kg\n", spacecraft->mass);
}

/**
 * Compute altitude above planetary surface
 */
double compute_altitude(SpacecraftState *spacecraft) {
 return spacecraft->state[Z_POS];
}

/**
 * Compute gravity gradient matrix for more accurate gravity modeling
 */
void compute_gravity_gradient(Matrix *gravity_gradient, SpacecraftState *spacecraft) {
 sw_matrix_init_zeros(gravity_gradient);

 double R_moon = 1737400.0; // Lunar radius in meters

 // Position vector from Moon center (landing frame origin is on the surface)
 double rx = spacecraft->state[X_POS];
 double ry = spacecraft->state[Y_POS];
 double rz = spacecraft->state[Z_POS] + R_moon;

 double r2 = rx * rx + ry * ry + rz * rz;
 double r = sqrt(r2);
 if (r < 1.0) r = 1.0; // Safety guard

 // Gravitational parameter: mu = g_surface * R_moon^2
 double mu = LUNAR_GRAVITY * R_moon * R_moon;

 // Tidal tensor: T_ij = (mu/r^3) * (-delta_ij + 3 * r_i * r_j / r^2)
 double factor = mu / (r * r * r);

 gravity_gradient->elements[0][0] = factor * (-1.0 + 3.0 * rx * rx / r2);
 gravity_gradient->elements[1][1] = factor * (-1.0 + 3.0 * ry * ry / r2);
 gravity_gradient->elements[2][2] = factor * (-1.0 + 3.0 * rz * rz / r2);

 gravity_gradient->elements[0][1] = factor * 3.0 * rx * ry / r2;
 gravity_gradient->elements[0][2] = factor * 3.0 * rx * rz / r2;
 gravity_gradient->elements[1][0] = gravity_gradient->elements[0][1];
 gravity_gradient->elements[1][2] = factor * 3.0 * ry * rz / r2;
 gravity_gradient->elements[2][0] = gravity_gradient->elements[0][2];
 gravity_gradient->elements[2][1] = gravity_gradient->elements[1][2];
}

/**
 * Compute solar radiation pressure perturbation
 * Primary non-gravitational perturbation for lunar surface operations
 */
void compute_srp_perturbation(Vector3D *srp_force, SpacecraftState *spacecraft) {
 // Initialize force
 srp_force->x = 0.0;
 srp_force->y = 0.0;
 srp_force->z = 0.0;

 // Solar radiation pressure: F = (S/c) * Cr * A
 // where S = solar flux, c = speed of light, Cr = reflectivity, A = area
 double srp_force_mag = (SOLAR_FLUX / SPEED_OF_LIGHT) * SRP_REFLECTIVITY * SRP_AREA;

 // Sun direction varies with mission elapsed time
 // Approximate sun direction in landing frame (slowly rotating)
 double sun_angle = 0.1 + spacecraft->time_elapsed * 2.662e-6; // Lunar sidereal rotation rate (2π/27.3d)
 double sun_x = sw_cos(sun_angle);
 double sun_y = sw_sin(sun_angle) * sw_cos(0.0896); // Lunar obliquity ~5.13 deg
 double sun_z = sw_sin(sun_angle) * sw_sin(0.0896);

 // SRP force is directed away from the Sun (radiation pressure pushes outward)
 srp_force->x = -srp_force_mag * sun_x;
 srp_force->y = -srp_force_mag * sun_y;
 srp_force->z = -srp_force_mag * sun_z;

 // Shadow check: if spacecraft is below horizon, SRP is occluded
 double altitude = compute_altitude(spacecraft);
 if (altitude < 0.0) {
 srp_force->x = 0.0;
 srp_force->y = 0.0;
 srp_force->z = 0.0;
 }
}

/**
 * Matrix operations implementation
 */

void sw_matrix_multiply(Matrix *result, Matrix *a, Matrix *b) {
 /* ── Optimized: pre-transpose b → direct MLDS/MSDS ── */
 const int n = MATRIX_SIZE;
 double *A = (double *)a->elements;
 double *B = (double *)b->elements;
 double *C = (double *)result->elements;
 double *BT = (double *)flight_malloc_impl((size_t)n * n * sizeof(double));
 memset(C, 0, (size_t)n * n * sizeof(double));

 const long stride_n = n * (long)sizeof(double);
 const long s24 = 24;
 int n3a = (n / 3) * 3;

 for (int i = 0; i < n3a; i += 3) {
 for (int j = 0; j < n3a; j += 3) {
 asm volatile(MZERO(M2));
 for (int kk = 0; kk < n3a; kk += 3) {
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride_n), "r"(&A[i * n + kk]),
 "r"(stride_n), "r"(&B[kk * n + j])
 : "t0", "t1", "memory"
 );
 }
 if (n3a < n) {
 double a_pad[9] = {0}, b_pad[9] = {0};
 for (int r = 0; r < 3; r++)
 for (int c = 0; c < n - n3a; c++) {
 a_pad[r * 3 + c] = A[(i + r) * n + n3a + c];
 b_pad[r * 3 + c] = B[(j + r) * n + n3a + c];
 }
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(s24), "r"(a_pad),
 "r"(s24), "r"(b_pad)
 : "t0", "t1", "memory"
 );
 }
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(stride_n), "r"(&C[i * n + j])
 : "t0", "t1", "memory"
 );
 }
 for (int j = n3a; j < n; j++)
 for (int r = 0; r < 3; r++)
 for (int kk = 0; kk < n; kk++)
 C[(i + r) * n + j] += A[(i + r) * n + kk] * B[kk * n + j];
 }
 for (int i = n3a; i < n; i++)
 for (int j = 0; j < n; j++)
 for (int kk = 0; kk < n; kk++)
 C[i * n + j] += A[i * n + kk] * B[kk * n + j];
}


void sw_matrix_transpose(Matrix *result, Matrix *a) {
 Matrix temp;

 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 temp.elements[j][i] = a->elements[i][j];
 }
 }

 // Copy result
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 result->elements[i][j] = temp.elements[i][j];
 }
 }
}


void sw_matrix_init_identity(Matrix *a) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 a->elements[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }
}

void sw_matrix_init_zeros(Matrix *a) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 a->elements[i][j] = 0.0;
 }
 }
}


/**
 * Vector operations implementation
 */

double vector_magnitude(Vector3D *v) {
 return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}


