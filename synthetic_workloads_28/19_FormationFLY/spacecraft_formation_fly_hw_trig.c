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
 * Spacecraft Formation Flying Control Algorithm
 * HW Trig (CORDIC accelerator for trigonometric functions)
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the relative orbit control algorithm for multi-spacecraft formation
 * flying. This workload computes relative state estimation, UKF-based filtering
 * with sigma-point propagation, orbital element transformations, and inter-
 * spacecraft collision avoidance. Applicable to formation flying technology
 * demonstrators and future distributed aperture missions for solar observation
 * and gravitational wave detection.
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
#include <stdint.h>
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
// Define formation size and matrix dimensions
#define NUM_SPACECRAFT 3 // Number of spacecraft in formation
#define STATE_DIM 6 // State dimension per spacecraft (pos + vel)
#define AUGMENTED_STATE_DIM 18 // Total state dimension for all spacecraft (NUM_SPACECRAFT * STATE_DIM)
#define SIGMA_POINTS 37 // Sigma points for UKF (2*AUGMENTED_STATE_DIM + 1)
#define MATRIX_DIM 18 // Dimension for large matrix operations (= AUGMENTED_STATE_DIM)
#define HARMONICS_DEGREE 10 // Degree of spherical harmonics used for gravity model
#define MEAS_DIM 12 // Measurement dimension per spacecraft

// Physical constants
#define MU_EARTH 3.986004418e14 // Earth's gravitational parameter (m^3/s^2)
#define J2_EARTH 1.08262668e-3 // Earth's J2 gravitational coefficient
#define J3_EARTH -2.53265648533e-6 // Earth's J3 gravitational coefficient
#define J4_EARTH -1.61962159137e-6 // Earth's J4 gravitational coefficient
#define EARTH_RADIUS 6378.137e3 // Earth radius in meters
#define SOLAR_PRESSURE 4.56e-6 // Solar radiation pressure (N/m^2)
#define DRAG_COEFFICIENT 2.2 // Typical drag coefficient

// Structures remain similar but with expanded state dimensions
typedef struct {
 double position[3]; // Position in ECI frame [x, y, z] (m)
 double velocity[3]; // Velocity in ECI frame [vx, vy, vz] (m/s)
 double quaternion[4]; // Attitude quaternion [q0, q1, q2, q3]
 double angular_rates[3]; // Angular rates [wx, wy, wz] (rad/s)
 double mass; // Spacecraft mass (kg)
 double inertia_tensor[3][3]; // Inertia tensor (kg*m^2)
 double control_forces[3]; // Control forces (N)
 double control_torques[3]; // Control torques (N*m)
 double perturbation_params[3]; // Environmental perturbation parameters
 double extended_state[2]; // Extended state for high-order dynamics
} SpacecraftState;

typedef struct {
 double desired_separation[NUM_SPACECRAFT][NUM_SPACECRAFT][3]; // Desired separation between spacecraft (m)
 double desired_attitude[NUM_SPACECRAFT][4]; // Desired attitude quaternion per spacecraft
 double reference_orbit[6]; // Reference orbit elements [a, e, i, Ω, ω, M]
 double formation_center[3]; // Formation center position
 int leader_idx; // Index of the leader spacecraft
 double config_matrix[MATRIX_DIM][MATRIX_DIM]; // Configuration matrix for formation control
} FormationConfig;

typedef struct {
 double relative_position[NUM_SPACECRAFT][NUM_SPACECRAFT][3]; // Measured relative positions
 double relative_velocity[NUM_SPACECRAFT][NUM_SPACECRAFT][3]; // Measured relative velocities
 double relative_attitude[NUM_SPACECRAFT][NUM_SPACECRAFT][4]; // Measured relative quaternions
 double absolute_position[NUM_SPACECRAFT][3]; // GPS/GNSS measurements
 double absolute_velocity[NUM_SPACECRAFT][3]; // GPS/GNSS velocity measurements
 double star_tracker[NUM_SPACECRAFT][MEAS_DIM]; // Formation sensor measurements
 double range_measurements[NUM_SPACECRAFT][NUM_SPACECRAFT]; // Inter-satellite range measurements
 double bearing_measurements[NUM_SPACECRAFT][NUM_SPACECRAFT][2]; // Inter-satellite bearing measurements
 int measurement_valid[NUM_SPACECRAFT][NUM_SPACECRAFT]; // Validity flags for measurements
} RelativeMeasurements;

typedef struct {
 double state_estimate[AUGMENTED_STATE_DIM]; // State estimate vector
 double covariance[AUGMENTED_STATE_DIM][AUGMENTED_STATE_DIM]; // State covariance matrix
 double process_noise[AUGMENTED_STATE_DIM][AUGMENTED_STATE_DIM]; // Process noise covariance
 double measurement_noise[MEAS_DIM*NUM_SPACECRAFT][MEAS_DIM*NUM_SPACECRAFT]; // Measurement noise
 double sigma_points[SIGMA_POINTS][AUGMENTED_STATE_DIM]; // Sigma points
 double sigma_weights_mean[SIGMA_POINTS]; // Weights for mean calculation
 double sigma_weights_covariance[SIGMA_POINTS]; // Weights for covariance calculation
 double alpha; // UKF parameter alpha
 double beta; // UKF parameter beta
 double kappa; // UKF parameter kappa
 double harmonics_coefficients[HARMONICS_DEGREE][HARMONICS_DEGREE]; // Spherical harmonics coefficients
} UKFState;

typedef struct {
 double collision_probability[NUM_SPACECRAFT][NUM_SPACECRAFT]; // Probability matrix
 double minimum_safe_distance; // Min safe distance (m)
 double avoidance_gain; // Gain for avoidance controller
 double time_horizon; // Prediction time horizon (s)
 double risk_matrix[MATRIX_DIM][MATRIX_DIM]; // Extended risk assessment matrix
} CollisionAssessment;

/* CORDIC accelerator intrinsics for trigonometric functions */
#define sw_sin(angle) hw_sin(angle)
#define sw_cos(angle) hw_cos(angle)
#define sw_asin(angle) hw_asin(angle)
#define sw_acos(angle) hw_acos(angle)
#define sw_tan(angle) hw_tan(angle)
#define sw_atan(angle) hw_atan(angle)
#define sw_atan2(y, x) hw_atan2(y, x)

// Forward declarations for all functions
void sw_matrix_multiply(double *A, double *B, double *C, int m, int n, int p);
void sw_matrix_transpose(double *A, double *AT, int m, int n);
void sw_matrix_inverse(double *A, double *A_inv, int n);
void sw_matrix_add(double *A, double *B, double *C, int m, int n);
void sw_cholesky_decomposition(double *A, double *L, int n);
void sw_quaternion_multiply(double *q1, double *q2, double *result);
void sw_quaternion_conjugate(double *q, double *result);
void sw_quaternion_normalize(double *q);
void sw_compute_relative_quaternion(double *q1, double *q2, double *q_rel);
void sw_orbital_elements_to_state(double *elements, double *position, double *velocity);
void sw_hill_frame_transformation(double *position_eci, double *velocity_eci, double *position_hill, double *dcm_eci_to_hill);

void initialize_spacecraft_states(SpacecraftState *spacecraft, FormationConfig *config);
void initialize_ukf(UKFState *ukf);
void generate_sigma_points(UKFState *ukf);
void propagate_states(SpacecraftState *spacecraft, FormationConfig *config, double dt);
void propagate_sigma_points(UKFState *ukf, double dt);
void compute_measurement_predictions(UKFState *ukf, double *predicted_measurements);
void update_ukf_state(UKFState *ukf, RelativeMeasurements *measurements);
void compute_control_inputs(SpacecraftState *spacecraft, UKFState *ukf, FormationConfig *config, CollisionAssessment *collision);
void assess_collision_probability(SpacecraftState *spacecraft, UKFState *ukf, CollisionAssessment *collision);
void apply_collision_avoidance(SpacecraftState *spacecraft, CollisionAssessment *collision);
void apply_gravity_perturbations(SpacecraftState *spacecraft, UKFState *ukf, double dt);
void apply_relative_orbit_control(SpacecraftState *spacecraft, FormationConfig *config);
void apply_attitude_control(SpacecraftState *spacecraft, FormationConfig *config);
void simulate_measurements(SpacecraftState *spacecraft, RelativeMeasurements *measurements);
void add_measurement_noise(RelativeMeasurements *measurements, UKFState *ukf);
void compute_spherical_harmonics(double *position, double *acceleration, UKFState *ukf);
void compute_solar_radiation_pressure(SpacecraftState *spacecraft, double *acceleration);
void compute_atmospheric_drag(SpacecraftState *spacecraft, double *acceleration);
double compute_legendre_polynomial(int n, int m, double x);
double compute_associated_legendre(int n, int m, double x);
double run_formation_simulation(int timesteps);
double rand_double(double min, double max);
void randomize_matrix(double *A, int m, int n);
void compute_intersatellite_baseline_covariance(double *Prel, double *Pchief, double *Pdeputy, double *H_baseline, double *K_gain, double *temp);
void compute_relative_orbit_geometry(double *roe_state, int size);

/**
 * Software matrix multiplication: C = A * B
 */
void sw_matrix_multiply(double *A, double *B, double *C, int m, int n, int p) {
 // A is m×n, B is n×p, C is m×p

 // Initialize C to zeros
 memset(C, 0, m * p * sizeof(double));

 // Standard matrix multiplication
 for (int i = 0; i < m; i++) {
 for (int j = 0; j < p; j++) {
 for (int k = 0; k < n; k++) {
 C[i * p + j] += A[i * n + k] * B[k * p + j];
 }
 }
 }
}

/**
 * Software matrix transpose: AT = A^T
 */
void sw_matrix_transpose(double *A, double *AT, int m, int n) {
 // A is m×n, AT is n×m

 for (int i = 0; i < m; i++) {
 for (int j = 0; j < n; j++) {
 AT[j * m + i] = A[i * n + j];
 }
 }
}

/**
 * Software matrix addition: C = A + B
 */
void sw_matrix_add(double *A, double *B, double *C, int m, int n) {
 for (int i = 0; i < m * n; i++) {
 C[i] = A[i] + B[i];
 }
}

/**
 * Software matrix subtraction: C = A - B
 */

/**
 * Software matrix scaling: B = A * scalar
 */

/**
 * Software Cholesky decomposition of a symmetric positive-definite matrix
 */
void sw_cholesky_decomposition(double *A, double *L, int n) {
 // Initialize L to zeros
 memset(L, 0, n * n * sizeof(double));

 for (int i = 0; i < n; i++) {
 for (int j = 0; j <= i; j++) {
 double sum = 0;

 if (j == i) { // Diagonal elements
 for (int k = 0; k < j; k++) {
 sum += L[j * n + k] * L[j * n + k];
 }
 L[j * n + j] = sqrt(A[j * n + j] - sum);
 } else {
 for (int k = 0; k < j; k++) {
 sum += L[i * n + k] * L[j * n + k];
 }
 L[i * n + j] = (A[i * n + j] - sum) / L[j * n + j];
 }
 }
 }
}

/**
 * Software matrix inverse using Gauss-Jordan elimination
 */
void sw_matrix_inverse(double *A, double *A_inv, int n) {
 /* Static augmented buffer — max dim = MEAS_DIM*NUM_SPACECRAFT (MISRA 21.3) */
#define MAX_INV_DIM (MEAS_DIM * NUM_SPACECRAFT)
 static double inv_aug[MAX_INV_DIM * 2 * MAX_INV_DIM];
 if (n > MAX_INV_DIM) { /* identity fallback for oversized request */
 for (int r = 0; r < n * n; r++) A_inv[r] = 0.0;
 for (int r = 0; r < n; r++) A_inv[r * n + r] = 1.0;
 return;
 }
 double *aug = inv_aug;

 // Initialize augmented matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 aug[i * (2 * n) + j] = A[i * n + j];
 }
 for (int j = 0; j < n; j++) {
 aug[i * (2 * n) + (j + n)] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Gauss-Jordan elimination
 for (int i = 0; i < n; i++) {
 // Find pivot
 double pivot = aug[i * (2 * n) + i];
 int pivot_row = i;

 for (int j = i + 1; j < n; j++) {
 if (fabs(aug[j * (2 * n) + i]) > fabs(pivot)) {
 pivot = aug[j * (2 * n) + i];
 pivot_row = j;
 }
 }

 // Swap rows if needed
 if (pivot_row != i) {
 for (int j = 0; j < 2 * n; j++) {
 double temp = aug[i * (2 * n) + j];
 aug[i * (2 * n) + j] = aug[pivot_row * (2 * n) + j];
 aug[pivot_row * (2 * n) + j] = temp;
 }
 }

 // Scale pivot row
 double scale = 1.0 / aug[i * (2 * n) + i];
 for (int j = 0; j < 2 * n; j++) {
 aug[i * (2 * n) + j] *= scale;
 }

 // Eliminate other rows
 for (int j = 0; j < n; j++) {
 if (j != i) {
 double factor = aug[j * (2 * n) + i];
 for (int k = 0; k < 2 * n; k++) {
 aug[j * (2 * n) + k] -= factor * aug[i * (2 * n) + k];
 }
 }
 }
 }

 // Extract inverse from augmented matrix
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 A_inv[i * n + j] = aug[i * (2 * n) + (j + n)];
 }
 }

 /* No heap free — static pool (MISRA 21.3) */
}

/**
 * Software quaternion multiplication
 */
void sw_quaternion_multiply(double *q1, double *q2, double *result) {
 result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
 result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
 result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
 result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

/**
 * Software quaternion conjugate
 */
void sw_quaternion_conjugate(double *q, double *result) {
 result[0] = q[0];
 result[1] = -q[1];
 result[2] = -q[2];
 result[3] = -q[3];
}

/**
 * Software quaternion normalization
 */
void sw_quaternion_normalize(double *q) {
 double mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

 if (mag > 1e-10) {
 q[0] /= mag;
 q[1] /= mag;
 q[2] /= mag;
 q[3] /= mag;
 }
}

/**
 * Software quaternion to DCM conversion
 */

/**
 * Software DCM to quaternion conversion
 */

/**
 * Software relative quaternion computation
 */
void sw_compute_relative_quaternion(double *q1, double *q2, double *q_rel) {
 double q1_conj[4];
 sw_quaternion_conjugate(q1, q1_conj);
 sw_quaternion_multiply(q2, q1_conj, q_rel);
}

/**
 * Software hill frame transformation
 */
void sw_hill_frame_transformation(double *position_eci, double *velocity_eci, double *position_hill, double *dcm_eci_to_hill) {
 // Compute unit vectors for Hill frame (radial, along-track, cross-track)
 double r_hat[3], v_hat[3], h_hat[3];

 // r_hat is along the radial direction
 double r_norm = sqrt(position_eci[0]*position_eci[0] +
 position_eci[1]*position_eci[1] +
 position_eci[2]*position_eci[2]);

 /* Singularity guard: degenerate position at origin */
 if (r_norm < 1e-12) {
 /* Fallback identity DCM + zero Hill position */
 for (int i = 0; i < 9; i++) dcm_eci_to_hill[i] = 0.0;
 dcm_eci_to_hill[0] = dcm_eci_to_hill[4] = dcm_eci_to_hill[8] = 1.0;
 position_hill[0] = position_hill[1] = position_hill[2] = 0.0;
 return;
 }

 for (int i = 0; i < 3; i++) {
 r_hat[i] = position_eci[i] / r_norm;
 }

 // h_hat is perpendicular to the orbital plane
 // h = r × v
 h_hat[0] = position_eci[1] * velocity_eci[2] - position_eci[2] * velocity_eci[1];
 h_hat[1] = position_eci[2] * velocity_eci[0] - position_eci[0] * velocity_eci[2];
 h_hat[2] = position_eci[0] * velocity_eci[1] - position_eci[1] * velocity_eci[0];

 double h_norm = sqrt(h_hat[0]*h_hat[0] + h_hat[1]*h_hat[1] + h_hat[2]*h_hat[2]);

 /* Singularity guard: r parallel to v (degenerate orbital plane) */
 if (h_norm < 1e-12) {
 /* Construct orthogonal basis via Gram-Schmidt with auxiliary vector */
 double aux[3] = {0.0, 0.0, 1.0};
 /* If r_hat is near Z-axis, switch auxiliary to X-axis */
 if (fabs(r_hat[2]) > 0.9) {
 aux[0] = 1.0; aux[1] = 0.0; aux[2] = 0.0;
 }
 /* h_hat = aux × r_hat (guaranteed non-zero) */
 h_hat[0] = aux[1] * r_hat[2] - aux[2] * r_hat[1];
 h_hat[1] = aux[2] * r_hat[0] - aux[0] * r_hat[2];
 h_hat[2] = aux[0] * r_hat[1] - aux[1] * r_hat[0];
 h_norm = sqrt(h_hat[0]*h_hat[0] + h_hat[1]*h_hat[1] + h_hat[2]*h_hat[2]);
 }

 for (int i = 0; i < 3; i++) {
 h_hat[i] /= h_norm;
 }

 // v_hat is along-track: v_hat = h_hat × r_hat
 v_hat[0] = h_hat[1] * r_hat[2] - h_hat[2] * r_hat[1];
 v_hat[1] = h_hat[2] * r_hat[0] - h_hat[0] * r_hat[2];
 v_hat[2] = h_hat[0] * r_hat[1] - h_hat[1] * r_hat[0];

 // Populate the DCM from ECI to Hill frame
 for (int i = 0; i < 3; i++) {
 dcm_eci_to_hill[i*3 + 0] = r_hat[i]; // First column: r_hat (radial)
 dcm_eci_to_hill[i*3 + 1] = v_hat[i]; // Second column: v_hat (along-track)
 dcm_eci_to_hill[i*3 + 2] = h_hat[i]; // Third column: h_hat (cross-track)
 }

 // Transform position to Hill frame coordinates
 position_hill[0] = r_norm;
 position_hill[1] = 0.0; // Along-track position is zero by definition
 position_hill[2] = 0.0; // Cross-track position is zero by definition
}

/**
 * Run intensive matrix operations to software implementation
 */
void compute_intersatellite_baseline_covariance(double *Prel, double *Pchief, double *Pdeputy, double *H_baseline, double *K_gain, double *temp) {
 /* Compute relative baseline covariance from chief/deputy state covariances.
 P_rel = H * (P_chief + P_deputy) * H^T + process noise
 Then compute Kalman gain K = P_rel * H^T * (H*P_rel*H^T + R)^{-1} */

 int n = MATRIX_DIM;

 /* Combined covariance: temp = P_chief + P_deputy */
 sw_matrix_add(Pchief, Pdeputy, temp, n, n);

 /* Project into baseline frame: Prel = H * temp * H^T */
 sw_matrix_multiply(H_baseline, temp, K_gain, n, n, n); /* K_gain = H * temp */
 sw_matrix_transpose(H_baseline, Prel, n, n); /* Prel = H^T (temporary) */
 sw_matrix_multiply(K_gain, Prel, temp, n, n, n); /* temp = H*temp*H^T */

 /* Add process noise to ensure positive definiteness */
 for (int i = 0; i < n; i++) {
 temp[i * n + i] += 1e-6;
 }

 /* Cholesky factorization of innovation covariance S = H*P*H^T + R */
 double *S = Prel; /* reuse buffer */
 for (int i = 0; i < n * n; i++) S[i] = temp[i];
 for (int i = 0; i < n; i++) S[i * n + i] += 0.01; /* measurement noise R */
 sw_cholesky_decomposition(S, K_gain, n); /* K_gain = L such that S = L*L^T */

 /* Compute S^{-1} via back substitution through L */
 sw_matrix_inverse(S, Prel, n); /* Prel = S^{-1} */

 /* Kalman gain: K = P_predicted * H^T * S^{-1} */
 sw_matrix_transpose(H_baseline, K_gain, n, n); /* K_gain = H^T */
 sw_matrix_multiply(temp, K_gain, S, n, n, n); /* S = P*H^T */
 sw_matrix_multiply(S, Prel, K_gain, n, n, n); /* K = P*H^T*S^{-1} */

 /* Updated covariance: P_rel = (I - K*H) * P_predicted */
 sw_matrix_multiply(K_gain, H_baseline, S, n, n, n); /* S = K*H */
 for (int i = 0; i < n; i++) S[i * n + i] -= 1.0; /* S = K*H - I */
 sw_matrix_multiply(S, temp, Prel, n, n, n); /* Prel = (K*H - I)*P */
 for (int i = 0; i < n * n; i++) Prel[i] = -Prel[i]; /* Prel = (I - K*H)*P */
}

/**
 * Run intensive trigonometric operations on a matrix
 */
void compute_relative_orbit_geometry(double *roe_state, int size) {
 /* Convert Cartesian relative state to curvilinear relative orbit elements
 and compute bearing/range/range-rate observables for inter-satellite links.
 Uses spherical coordinate transforms for the Hill-frame geometry. */

 int n = size * size;
 static double roe_obs[MATRIX_DIM * MATRIX_DIM]; /* static pool (MISRA 21.3) */
 double *obs = roe_obs;

 /* Pass 1: Cartesian -> spherical (azimuth, elevation, range) */
 for (int i = 0; i < n; i++) {
 double x = roe_state[i];
 /* Relative position components (encode in periodic form) */
 double rho = sqrt(x * x + 1.0); /* range magnitude */
 double az = sw_atan2(sw_sin(x), sw_cos(x)); /* azimuth angle */
 double el = sw_asin(x / rho); /* elevation angle */

 /* Along-track angular rate from Hill-frame kinematics */
 double theta_dot = sw_cos(el) / (rho * rho);
 obs[i] = az * sw_cos(el) + theta_dot;
 }

 /* Pass 2: Hill-frame rotation matrix elements for each deputy */
 for (int i = 0; i < n; i++) {
 double theta = obs[i];
 /* 3-1-3 Euler angle rotation chief LVLH -> deputy LVLH */
 double c1 = sw_cos(theta);
 double s1 = sw_sin(theta);
 double c2 = sw_cos(2.0 * theta);
 double s2 = sw_sin(2.0 * theta);

 /* Relative inclination vector (ix, iy) via atan2 */
 double ix = sw_atan2(s1 * c2, c1 * s2 + 0.01);
 double iy = sw_acos(fmax(-0.99, fmin(0.99, c1 * c2)));

 roe_state[i] = ix * sw_cos(iy) + s1 * sw_sin(iy);
 }

 /* Pass 3: Bearing-rate observables for ISL link budget */
 for (int i = 0; i < n; i++) {
 double phi = roe_state[i];

 /* LOS angle in chief Hill frame */
 double los_az = sw_atan2(sw_sin(phi), sw_cos(phi));

 /* Bearing rate: d(az)/dt from angular momentum conservation */
 double r_rel = sqrt(phi * phi + 0.5);
 r_rel = fmax(0.1, r_rel);

 /* Legendre-polynomial gravity perturbation on relative motion */
 double P2 = 0.5 * (3.0 * sw_cos(los_az) * sw_cos(los_az) - 1.0);
 double P3 = 0.5 * sw_cos(los_az) * (5.0 * sw_cos(los_az) * sw_cos(los_az) - 3.0);

 double bearing_rate = (P2 + 0.1 * P3) / (r_rel * r_rel);
 roe_state[i] = bearing_rate;
 }

 /* No heap free — static pool (MISRA 21.3) */
}

/**
 * Computes Legendre polynomial P_n(x)
 */
double compute_legendre_polynomial(int n, int m, double x) {
 // For m > 0, compute associated Legendre polynomial
 if (m > 0) {
 return compute_associated_legendre(n, m, x);
 }

 // Base cases
 if (n == 0) return 1.0;
 if (n == 1) return x;

 // Recurrence relation for Legendre polynomials
 double p_n_minus_2 = 1.0; // P_0(x)
 double p_n_minus_1 = x; // P_1(x)
 double p_n = 0.0;

 for (int i = 2; i <= n; i++) {
 // Using software implementation for trigonometric calculations
 p_n = ((2.0 * i - 1.0) * x * p_n_minus_1 - (i - 1.0) * p_n_minus_2) / i;
 p_n_minus_2 = p_n_minus_1;
 p_n_minus_1 = p_n;
 }

 return p_n;
}

/**
 * Computes associated Legendre polynomial P_n^m(x)
 */
double compute_associated_legendre(int n, int m, double x) {
 /* Bounds check (m must lie in [0, n]). */
 if (m < 0 || m > n) return 0.0;

 /* Iterative implementation (MISRA-C:2012 Rule 17.2: no recursion).
  * Step 1: seed P_m^m(x) = (-1)^m * (2m-1)!! * (1 - x^2)^(m/2). */
 double oneMinusX2 = 1.0 - x * x;
 double factor = 1.0;
 double pow_term = 1.0;
 for (int i = 1; i <= m; i++) {
 factor *= (double)(2 * i - 1);
 pow_term *= oneMinusX2;
 }
 double pmm = ((m & 1) ? -1.0 : 1.0) * factor * sqrt(pow_term);
 if (n == m) return pmm;

 /* Step 2: bootstrap P_(m+1)^m(x) = x * (2m+1) * P_m^m(x). */
 double pmp1 = x * (2.0 * m + 1.0) * pmm;
 if (n == m + 1) return pmp1;

 /* Step 3: forward two-term recurrence for n > m+1. */
 double p_n_minus_2 = pmm;
 double p_n_minus_1 = pmp1;
 double p_n = 0.0;
 for (int i = m + 2; i <= n; i++) {
 p_n = ((2.0 * i - 1.0) * x * p_n_minus_1 - (double)(i + m - 1) * p_n_minus_2) / (double)(i - m);
 p_n_minus_2 = p_n_minus_1;
 p_n_minus_1 = p_n;
 }

 return p_n;
}

/**
 * Compute gravity perturbations using spherical harmonics
 */
void compute_spherical_harmonics(double *position, double *acceleration, UKFState *ukf) {
 // Cartesian to spherical coordinates
 double r = sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);
 double phi = atan2(position[1], position[0]);
 double theta = acos(position[2] / r);

 // Initialize acceleration
 acceleration[0] = 0.0;
 acceleration[1] = 0.0;
 acceleration[2] = 0.0;

 // Earth radius to r ratio
 double ae_over_r = EARTH_RADIUS / r;

 // Loop through all degrees and orders of spherical harmonics
 for (int n = 0; n < HARMONICS_DEGREE; n++) {
 double ae_over_r_n = pow(ae_over_r, n);

 for (int m = 0; m <= n; m++) {
 // Compute Legendre polynomial
 double P_nm = compute_legendre_polynomial(n, m, cos(theta));

 // Spherical harmonic gravity coefficients
 double C_nm = ukf->harmonics_coefficients[n][m];
 double S_nm = (m == 0) ? 0.0 : ukf->harmonics_coefficients[m][n]; // Swap indices for S terms

 // Compute trigonometric terms using software implementation
 double cos_m_phi = sw_cos(m * phi);
 double sin_m_phi = sw_sin(m * phi);

 // Accumulate gravitational acceleration (zonal harmonic expansion)
 // This is a computationally intensive part with many trigonometric terms
 double common_factor = -MU_EARTH / (r * r) * ae_over_r_n * (n + 1);
 double term = C_nm * cos_m_phi + S_nm * sin_m_phi;

 // Add this term's contribution
 acceleration[0] += common_factor * P_nm * term * sin(theta) * cos(phi);
 acceleration[1] += common_factor * P_nm * term * sin(theta) * sin(phi);
 acceleration[2] += common_factor * P_nm * term * cos(theta);
 }
 }
}

/**
 * Compute solar radiation pressure acceleration
 */
void compute_solar_radiation_pressure(SpacecraftState *spacecraft, double *acceleration) {
 // Epoch-dependent solar direction vector

 // Generate a unit vector pointing "toward the Sun"
 double sun_direction[3] = {
 sw_cos(0.1) * sw_sin(0.2),
 sw_sin(0.1) * sw_sin(0.2),
 sw_cos(0.2)
 };

 // Normalize the direction
 double norm = sqrt(sun_direction[0]*sun_direction[0] +
 sun_direction[1]*sun_direction[1] +
 sun_direction[2]*sun_direction[2]);

 sun_direction[0] /= norm;
 sun_direction[1] /= norm;
 sun_direction[2] /= norm;

 // Effective cross-sectional area
 double cross_sectional_area = 2.0; // m^2

 // Apply solar pressure to get acceleration
 double scale = -SOLAR_PRESSURE * cross_sectional_area / spacecraft->mass;

 for (int i = 0; i < 3; i++) {
 acceleration[i] = scale * sun_direction[i];
 }
}

/**
 * Compute atmospheric drag acceleration using exponential atmosphere model.
 * Valid for LEO formation-flying altitudes (~300-600 km).
 * rho(h) = rho0 * exp(-(h - h0) / H)
 * a_drag = -0.5 * rho * Cd * A/m * |v|^2 * v_hat
 */
void compute_atmospheric_drag(SpacecraftState *spacecraft, double *acceleration) {
 /* Exponential atmosphere reference parameters (h0 = 400 km) */
 const double rho0 = 3.725e-12; /* kg/m^3 at reference altitude */
 const double h0 = 400000.0; /* reference altitude (m) */
 const double H = 58500.0; /* scale height (m) */
 const double area = 2.0; /* cross-sectional area (m^2), same as SRP */

 /* Compute altitude above Earth surface (R_earth = 6371 km) */
 double r = sqrt(spacecraft->position[0] * spacecraft->position[0] +
 spacecraft->position[1] * spacecraft->position[1] +
 spacecraft->position[2] * spacecraft->position[2]);
 double altitude = r - EARTH_RADIUS; /* m above surface */

 /* Atmospheric density at current altitude */
 double rho = rho0 * exp(-(altitude - h0) / H);
 if (rho < 1e-30) rho = 0.0; /* negligible above ~800 km */

 /* Velocity magnitude */
 double v_mag = sqrt(spacecraft->velocity[0] * spacecraft->velocity[0] +
 spacecraft->velocity[1] * spacecraft->velocity[1] +
 spacecraft->velocity[2] * spacecraft->velocity[2]);

 /* Drag acceleration: a = -0.5 * rho * Cd * A/m * |v| * v */
 double coeff = -0.5 * rho * DRAG_COEFFICIENT * area / spacecraft->mass;
 for (int i = 0; i < 3; i++) {
 acceleration[i] = coeff * v_mag * spacecraft->velocity[i];
 }
}

/**
 * Apply gravity perturbations
 */
void apply_gravity_perturbations(SpacecraftState *spacecraft, UKFState *ukf, double dt) {
 double acceleration[3];
 compute_spherical_harmonics(spacecraft->position, acceleration, ukf);

 // Apply acceleration to velocity (integrate over timestep)
 for (int i = 0; i < 3; i++) {
 spacecraft->velocity[i] += acceleration[i] * dt;
 }
}

/**
 * Software computation of orbital elements from position and velocity
 */

/**
 * Software conversion from orbital elements to position and velocity
 */
void sw_orbital_elements_to_state(double *elements, double *position, double *velocity) {
 double a = elements[0]; // Semi-major axis
 double e = elements[1]; // Eccentricity
 double i = elements[2]; // Inclination
 double Omega = elements[3]; // Right ascension of ascending node
 double omega = elements[4]; // Argument of periapsis
 double M = elements[5]; // Mean anomaly

 // Compute eccentric anomaly from mean anomaly (iterative solution to Kepler's equation)
 double E = M; // Initial guess
 double dE;
 for (int iter = 0; iter < 10; iter++) {
 dE = (M - E + e * sw_sin(E)) / (1.0 - e * sw_cos(E));
 E += dE;
 if (fabs(dE) < 1e-8) break;
 }

 // Compute true anomaly from eccentric anomaly
 double nu = 2.0 * sw_atan2(sqrt(1.0 + e) * sw_sin(E/2.0), sqrt(1.0 - e) * sw_cos(E/2.0));

 // Compute position and velocity in the orbital plane
 double r = a * (1.0 - e * sw_cos(E)); // Orbital radius

 double x_orb = r * sw_cos(nu);
 double y_orb = r * sw_sin(nu);

 double p = a * (1.0 - e * e); // Semi-latus rectum
 double h = sqrt(MU_EARTH * p); // Angular momentum

 double v_x_orb = -MU_EARTH / h * sw_sin(nu);
 double v_y_orb = MU_EARTH / h * (e + sw_cos(nu));

 // Rotation matrices for orbital plane to ECI transformation
 double cos_Omega = sw_cos(Omega);
 double sin_Omega = sw_sin(Omega);
 double cos_i = sw_cos(i);
 double sin_i = sw_sin(i);
 double cos_omega = sw_cos(omega);
 double sin_omega = sw_sin(omega);

 // Compute DCM (Direction Cosine Matrix) for orbital to ECI transformation
 double P11 = cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i;
 double P12 = -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i;
 double P13 = sin_Omega * sin_i;

 double P21 = sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i;
 double P22 = -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i;
 double P23 = -cos_Omega * sin_i;

 double P31 = sin_omega * sin_i;
 double P32 = cos_omega * sin_i;
 double P33 = cos_i;
 (void)P13;
 (void)P23;
 (void)P33;

 // Transform position from orbital plane to ECI
 position[0] = P11 * x_orb + P12 * y_orb;
 position[1] = P21 * x_orb + P22 * y_orb;
 position[2] = P31 * x_orb + P32 * y_orb;

 // Transform velocity from orbital plane to ECI
 velocity[0] = P11 * v_x_orb + P12 * v_y_orb;
 velocity[1] = P21 * v_x_orb + P22 * v_y_orb;
 velocity[2] = P31 * v_x_orb + P32 * v_y_orb;
}

/**
 * Generate a random double in a given range
 */
double rand_double(double min, double max) {
 return min + ((double)rand() / RAND_MAX) * (max - min);
}

/**
 * Randomize a matrix with values in [-1, 1]
 */
void randomize_matrix(double *A, int m, int n) {
 for (int i = 0; i < m * n; i++) {
 A[i] = rand_double(-1.0, 1.0);
 }
}

/**
 * Print a matrix (for debugging)
 */

/**
 * Initialize spacecraft states based on configuration
 */
void initialize_spacecraft_states(SpacecraftState *spacecraft, FormationConfig *config) {
 // Convert reference orbital elements to position and velocity
 double ref_position[3], ref_velocity[3];
 sw_orbital_elements_to_state(config->reference_orbit, ref_position, ref_velocity);

 // Set the leader at the reference position
 memcpy(spacecraft[config->leader_idx].position, ref_position, 3 * sizeof(double));
 memcpy(spacecraft[config->leader_idx].velocity, ref_velocity, 3 * sizeof(double));

 // Identity quaternion for leader
 spacecraft[config->leader_idx].quaternion[0] = 1.0;
 spacecraft[config->leader_idx].quaternion[1] = 0.0;
 spacecraft[config->leader_idx].quaternion[2] = 0.0;
 spacecraft[config->leader_idx].quaternion[3] = 0.0;

 // Zero angular rates for leader
 memset(spacecraft[config->leader_idx].angular_rates, 0, 3 * sizeof(double));

 // Set follower spacecraft states based on desired relative positions
 double dcm_eci_to_hill[3][3];
 double hill_position[3];

 // Compute hill frame transformation using software implementation
 sw_hill_frame_transformation(ref_position, ref_velocity, hill_position, (double *)dcm_eci_to_hill);

 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 if (i == config->leader_idx) continue; // Skip leader

 // Initialize mass and inertia tensor for each spacecraft
 spacecraft[i].mass = 100.0 + 10.0 * i; // Vary mass slightly

 // Diagonal inertia tensor (principal axes aligned)
 memset(spacecraft[i].inertia_tensor, 0, 9 * sizeof(double));
 spacecraft[i].inertia_tensor[0][0] = 10.0 + i * 0.5;
 spacecraft[i].inertia_tensor[1][1] = 12.0 + i * 0.5;
 spacecraft[i].inertia_tensor[2][2] = 8.0 + i * 0.5;

 // Relative position in Hill frame
 double rel_hill[3];
 rel_hill[0] = config->desired_separation[config->leader_idx][i][0];
 rel_hill[1] = config->desired_separation[config->leader_idx][i][1];
 rel_hill[2] = config->desired_separation[config->leader_idx][i][2];

 // Convert to ECI frame
 for (int j = 0; j < 3; j++) {
 spacecraft[i].position[j] = ref_position[j];
 for (int k = 0; k < 3; k++) {
 // dcm_eci_to_hill is the transpose of dcm_hill_to_eci
 spacecraft[i].position[j] += dcm_eci_to_hill[k][j] * rel_hill[k];
 }
 }

 // Calculate velocity in ECI frame (using CW equations approximation)
 double n = sqrt(MU_EARTH / pow(config->reference_orbit[0], 3)); // Mean motion

 double rel_hill_vel[3];
 rel_hill_vel[0] = 0;
 rel_hill_vel[1] = -2 * n * rel_hill[0];
 rel_hill_vel[2] = 0;

 for (int j = 0; j < 3; j++) {
 spacecraft[i].velocity[j] = ref_velocity[j];
 for (int k = 0; k < 3; k++) {
 spacecraft[i].velocity[j] += dcm_eci_to_hill[k][j] * rel_hill_vel[k];
 }
 }

 // Set attitude quaternion (a small random deviation from leader)
 double roll = rand_double(-5.0, 5.0) * M_PI / 180.0; // ±5 deg in roll
 double pitch = rand_double(-5.0, 5.0) * M_PI / 180.0; // ±5 deg in pitch
 double yaw = rand_double(-5.0, 5.0) * M_PI / 180.0; // ±5 deg in yaw

 // Simple euler-to-quaternion conversion
 double cr = sw_cos(roll * 0.5);
 double sr = sw_sin(roll * 0.5);
 double cp = sw_cos(pitch * 0.5);
 double sp = sw_sin(pitch * 0.5);
 double cy = sw_cos(yaw * 0.5);
 double sy = sw_sin(yaw * 0.5);

 spacecraft[i].quaternion[0] = cr * cp * cy + sr * sp * sy;
 spacecraft[i].quaternion[1] = sr * cp * cy - cr * sp * sy;
 spacecraft[i].quaternion[2] = cr * sp * cy + sr * cp * sy;
 spacecraft[i].quaternion[3] = cr * cp * sy - sr * sp * cy;

 // Normalize quaternion
 sw_quaternion_normalize(spacecraft[i].quaternion);

 // Set small random angular rates
 spacecraft[i].angular_rates[0] = rand_double(-0.01, 0.01); // rad/s
 spacecraft[i].angular_rates[1] = rand_double(-0.01, 0.01); // rad/s
 spacecraft[i].angular_rates[2] = rand_double(-0.01, 0.01); // rad/s

 // Initialize control inputs to zero
 memset(spacecraft[i].control_forces, 0, 3 * sizeof(double));
 memset(spacecraft[i].control_torques, 0, 3 * sizeof(double));

 // Initialize perturbation parameters
 spacecraft[i].perturbation_params[0] = rand_double(0.1, 1.0);
 spacecraft[i].perturbation_params[1] = rand_double(0.1, 1.0);
 spacecraft[i].perturbation_params[2] = rand_double(0.1, 1.0);

 // Initialize extended state
 spacecraft[i].extended_state[0] = rand_double(-0.1, 0.1);
 spacecraft[i].extended_state[1] = rand_double(-0.1, 0.1);
 }
}

/**
 * Initialize the UKF for state estimation
 */
void initialize_ukf(UKFState *ukf) {
 // Initialize UKF parameters
 ukf->alpha = 0.1; // Spread parameter (1e-4 to 1)
 ukf->beta = 2.0; // Optimal for Gaussian distributions
 ukf->kappa = 0.0; // Scaling parameter (0 or 3-n)

 // Lambda parameter for sigma point calculation
 double lambda = ukf->alpha * ukf->alpha * (AUGMENTED_STATE_DIM + ukf->kappa) - AUGMENTED_STATE_DIM;

 // Initialize weights for mean and covariance calculation
 ukf->sigma_weights_mean[0] = lambda / (AUGMENTED_STATE_DIM + lambda);
 ukf->sigma_weights_covariance[0] = ukf->sigma_weights_mean[0] + (1 - ukf->alpha * ukf->alpha + ukf->beta);

 for (int i = 1; i < SIGMA_POINTS; i++) {
 ukf->sigma_weights_mean[i] = 1.0 / (2 * (AUGMENTED_STATE_DIM + lambda));
 ukf->sigma_weights_covariance[i] = ukf->sigma_weights_mean[i];
 }

 // Initialize state estimate, covariance, and process noise

 // Zero out the state estimate initially
 memset(ukf->state_estimate, 0, AUGMENTED_STATE_DIM * sizeof(double));

 // Initialize covariance matrix (diagonal)
 memset(ukf->covariance, 0, AUGMENTED_STATE_DIM * AUGMENTED_STATE_DIM * sizeof(double));
 for (int i = 0; i < AUGMENTED_STATE_DIM; i++) {
 // Position uncertainty: 10m
 if (i % STATE_DIM < 3) {
 ukf->covariance[i][i] = 100.0; // 10^2
 }
 // Velocity uncertainty: 0.1 m/s
 else {
 ukf->covariance[i][i] = 0.01; // 0.1^2
 }
 }

 // Initialize process noise covariance (diagonal)
 memset(ukf->process_noise, 0, AUGMENTED_STATE_DIM * AUGMENTED_STATE_DIM * sizeof(double));
 for (int i = 0; i < AUGMENTED_STATE_DIM; i++) {
 // Position process noise
 if (i % STATE_DIM < 3) {
 ukf->process_noise[i][i] = 0.01; // 0.1^2 m^2
 }
 // Velocity process noise
 else {
 ukf->process_noise[i][i] = 0.0001; // 0.01^2 (m/s)^2
 }
 }

 // Initialize measurement noise covariance (diagonal)
 memset(ukf->measurement_noise, 0, MEAS_DIM * NUM_SPACECRAFT * MEAS_DIM * NUM_SPACECRAFT * sizeof(double));
 for (int i = 0; i < MEAS_DIM * NUM_SPACECRAFT; i++) {
 int meas_type = i % MEAS_DIM;

 // Position measurement noise (GPS): 5m
 if (meas_type < 3) {
 ukf->measurement_noise[i][i] = 25.0; // 5^2
 }
 // Velocity measurement noise (GPS): 0.05 m/s
 else if (meas_type < 6) {
 ukf->measurement_noise[i][i] = 0.0025; // 0.05^2
 }
 // Relative position measurement noise (inter-satellite link): 0.1m
 else if (meas_type < 9) {
 ukf->measurement_noise[i][i] = 0.01; // 0.1^2
 }
 // Range measurement noise: 0.05m
 else if (meas_type < 10) {
 ukf->measurement_noise[i][i] = 0.0025; // 0.05^2
 }
 // Extended nonlinear measurement noise
 else {
 ukf->measurement_noise[i][i] = 0.01;
 }
 }
}

/**
 * Generate sigma points for the UKF (continued)
 */
void generate_sigma_points(UKFState *ukf) {
 // Square root of state dimension + lambda
 double alpha_squared = ukf->alpha * ukf->alpha;
 double lambda = alpha_squared * (AUGMENTED_STATE_DIM + ukf->kappa) - AUGMENTED_STATE_DIM;
 double c = AUGMENTED_STATE_DIM + lambda;

 if (c < 0) c = 0.0001; // Avoid numerical issues
 c = sqrt(c);

 // Compute square root of the covariance matrix via Cholesky decomposition
 static double chol_buf[AUGMENTED_STATE_DIM * AUGMENTED_STATE_DIM]; /* MISRA 21.3 */
 double *cholesky = chol_buf;
 sw_cholesky_decomposition((double *)ukf->covariance, cholesky, AUGMENTED_STATE_DIM);

 // Set the first sigma point to the mean
 memcpy(ukf->sigma_points[0], ukf->state_estimate, AUGMENTED_STATE_DIM * sizeof(double));

 // Generate the other sigma points
 for (int i = 0; i < AUGMENTED_STATE_DIM; i++) {
 // Positive direction sigma points
 for (int j = 0; j < AUGMENTED_STATE_DIM; j++) {
 ukf->sigma_points[i+1][j] = ukf->state_estimate[j] + c * cholesky[j * AUGMENTED_STATE_DIM + i];
 }

 // Negative direction sigma points
 for (int j = 0; j < AUGMENTED_STATE_DIM; j++) {
 ukf->sigma_points[i+1+AUGMENTED_STATE_DIM][j] = ukf->state_estimate[j] - c * cholesky[j * AUGMENTED_STATE_DIM + i];
 }
 }
 /* chol_buf is static — no free needed (MISRA 21.3) */
}

/**
 * Propagate spacecraft states with advanced dynamics
 */
void propagate_states(SpacecraftState *spacecraft, FormationConfig *config, double dt) {
 (void)config;
 // For each spacecraft
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 // 1. Update position based on velocity
 for (int j = 0; j < 3; j++) {
 spacecraft[i].position[j] += spacecraft[i].velocity[j] * dt;
 }

 // 2. Gravitational acceleration (central body)
 double r = sqrt(
 spacecraft[i].position[0] * spacecraft[i].position[0] +
 spacecraft[i].position[1] * spacecraft[i].position[1] +
 spacecraft[i].position[2] * spacecraft[i].position[2]
 );

 double gravity_acc[3];
 for (int j = 0; j < 3; j++) {
 gravity_acc[j] = -MU_EARTH * spacecraft[i].position[j] / (r * r * r);
 }

 // 3. Control acceleration
 double control_acc[3];
 for (int j = 0; j < 3; j++) {
 control_acc[j] = spacecraft[i].control_forces[j] / spacecraft[i].mass;
 }

 // 4. Solar radiation pressure
 double srp_acc[3];
 compute_solar_radiation_pressure(&spacecraft[i], srp_acc);

 // 4b. Atmospheric drag
 double drag_acc[3];
 compute_atmospheric_drag(&spacecraft[i], drag_acc);

 // 5. Update velocity
 for (int j = 0; j < 3; j++) {
 spacecraft[i].velocity[j] += (gravity_acc[j] + control_acc[j] + srp_acc[j] + drag_acc[j]) * dt;
 }

 // 6. Update attitude quaternion based on angular rates
 double w[3];
 memcpy(w, spacecraft[i].angular_rates, 3 * sizeof(double));

 double q[4];
 memcpy(q, spacecraft[i].quaternion, 4 * sizeof(double));

 // Quaternion derivative
 double q_dot[4];
 q_dot[0] = 0.5 * (-w[0] * q[1] - w[1] * q[2] - w[2] * q[3]);
 q_dot[1] = 0.5 * (w[0] * q[0] + w[2] * q[2] - w[1] * q[3]);
 q_dot[2] = 0.5 * (w[1] * q[0] - w[2] * q[1] + w[0] * q[3]);
 q_dot[3] = 0.5 * (w[2] * q[0] + w[1] * q[1] - w[0] * q[2]);

 // Update quaternion
 for (int j = 0; j < 4; j++) {
 spacecraft[i].quaternion[j] += q_dot[j] * dt;
 }

 // Normalize quaternion
 sw_quaternion_normalize(spacecraft[i].quaternion);

 // 7. Update angular rates based on torques
 double omega[3];
 memcpy(omega, spacecraft[i].angular_rates, 3 * sizeof(double));

 // Compute omega_dot using Euler's rotational equations
 double omega_dot[3];
 double inertia[3];
 inertia[0] = spacecraft[i].inertia_tensor[0][0];
 inertia[1] = spacecraft[i].inertia_tensor[1][1];
 inertia[2] = spacecraft[i].inertia_tensor[2][2];

 omega_dot[0] = (spacecraft[i].control_torques[0] + (inertia[1] - inertia[2]) * omega[1] * omega[2]) / inertia[0];
 omega_dot[1] = (spacecraft[i].control_torques[1] + (inertia[2] - inertia[0]) * omega[2] * omega[0]) / inertia[1];
 omega_dot[2] = (spacecraft[i].control_torques[2] + (inertia[0] - inertia[1]) * omega[0] * omega[1]) / inertia[2];

 // Update angular rates
 for (int j = 0; j < 3; j++) {
 spacecraft[i].angular_rates[j] += omega_dot[j] * dt;
 }

 // 8. Update extended state with perturbation-driven evolution
 for (int j = 0; j < 2; j++) {
 double val = spacecraft[i].extended_state[j];
 val += 0.01 * sw_sin(val) + 0.005 * sw_cos(2.0 * val);
 spacecraft[i].extended_state[j] = val;
 }
 }
}

/**
 * Propagate sigma points through the system dynamics
 */
void propagate_sigma_points(UKFState *ukf, double dt) {
 // For each sigma point
 for (int i = 0; i < SIGMA_POINTS; i++) {
 // Get state vector for this sigma point
 double *state = ukf->sigma_points[i];

 // For each spacecraft in the formation
 for (int j = 0; j < NUM_SPACECRAFT; j++) {
 int state_offset = j * STATE_DIM;

 // Extract translational state (pos + vel) from UKF state vector
 double position[3], velocity[3];

 for (int k = 0; k < 3; k++) {
 position[k] = state[state_offset + k];
 velocity[k] = state[state_offset + 3 + k];
 }

 // 1. Update position based on velocity
 for (int k = 0; k < 3; k++) {
 position[k] += velocity[k] * dt;
 }

 // 2. Update velocity based on gravitational acceleration
 double r = sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);

 for (int k = 0; k < 3; k++) {
 velocity[k] -= MU_EARTH * position[k] / (r*r*r) * dt;
 }

 // 3. Apply J2 zonal harmonic perturbation
 double x = position[0];
 double y = position[1];
 double z = position[2];

 double r2 = r * r;
 double r5 = r2 * r2 * r;

 double coef = 1.5 * J2_EARTH * MU_EARTH * EARTH_RADIUS * EARTH_RADIUS / r5 * dt;
 double z2_r2 = 5.0 * z * z / r2;

 velocity[0] += coef * x * (z2_r2 - 1.0);
 velocity[1] += coef * y * (z2_r2 - 1.0);
 velocity[2] += coef * z * (z2_r2 - 3.0);

 // Store updated translational state back to UKF state vector
 for (int k = 0; k < 3; k++) {
 state[state_offset + k] = position[k];
 state[state_offset + 3 + k] = velocity[k];
 }
 }
 }
}

/**
 * Compute measurement predictions from the propagated sigma points
 */
void compute_measurement_predictions(UKFState *ukf, double *predicted_measurements) {
 // Initialize predicted measurements to zero
 memset(predicted_measurements, 0, MEAS_DIM * NUM_SPACECRAFT * sizeof(double));

 // For each sigma point
 for (int i = 0; i < SIGMA_POINTS; i++) {
 // Get state vector for this sigma point
 double *state = ukf->sigma_points[i];

 // For each spacecraft
 for (int j = 0; j < NUM_SPACECRAFT; j++) {
 int state_offset = j * STATE_DIM;
 int meas_offset = j * MEAS_DIM;

 // Extract translational state from UKF sigma point
 double position[3], velocity[3];

 for (int k = 0; k < 3; k++) {
 position[k] = state[state_offset + k];
 velocity[k] = state[state_offset + 3 + k];
 }

 // Predict measurements with UKF sigma-point weights
 double weight = ukf->sigma_weights_mean[i];

 // Position measurements (GPS-like)
 for (int k = 0; k < 3; k++) {
 predicted_measurements[meas_offset + k] += weight * position[k];
 }

 // Velocity measurements (GPS-like)
 for (int k = 0; k < 3; k++) {
 predicted_measurements[meas_offset + 3 + k] += weight * velocity[k];
 }

 // Inter-spacecraft relative position to chief (formation keeping)
 double chief_pos[3];
 int chief_off = 0; /* chief is spacecraft 0 */
 for (int k = 0; k < 3; k++)
 chief_pos[k] = state[chief_off + k];
 double rel_pos[3];
 for (int k = 0; k < 3; k++) {
 rel_pos[k] = position[k] - chief_pos[k];
 predicted_measurements[meas_offset + 6 + k] += weight * rel_pos[k];
 }

 // Inter-spacecraft range measurement
 double rel_range = sqrt(rel_pos[0]*rel_pos[0] + rel_pos[1]*rel_pos[1] + rel_pos[2]*rel_pos[2]);
 predicted_measurements[meas_offset + 9] += weight * rel_range;

 // Nonlinear position/velocity coupling measurement
 double extended_meas = sw_sin(position[0]/1000.0) * sw_cos(position[1]/1000.0) + sw_tan(velocity[2]/10.0);
 predicted_measurements[meas_offset + 10] += weight * extended_meas;

 // Specific orbital energy (vis-viva derived)
 double r_mag = sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);
 double v2 = velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2];
 double spec_energy = 0.5 * v2 - MU_EARTH / r_mag;
 predicted_measurements[meas_offset + 11] += weight * spec_energy;
 }
 }
}

/**
 * Update the UKF state estimate with the latest measurements
 */
void update_ukf_state(UKFState *ukf, RelativeMeasurements *measurements) {
 /* ---- Static BSS pools (MISRA 21.3: no malloc/free) ---- */
#define TOTAL_MEAS (MEAS_DIM * NUM_SPACECRAFT)
 static double ukf_pred_meas[TOTAL_MEAS];
 static double ukf_act_meas[TOTAL_MEAS];
 static double ukf_innov[TOTAL_MEAS];
 static double ukf_Pxy[AUGMENTED_STATE_DIM * TOTAL_MEAS];
 static double ukf_S[TOTAL_MEAS * TOTAL_MEAS];
 static double ukf_sigma_meas[SIGMA_POINTS * TOTAL_MEAS];
 static double ukf_S_inv[TOTAL_MEAS * TOTAL_MEAS];
 static double ukf_K[AUGMENTED_STATE_DIM * TOTAL_MEAS];
 static double ukf_K_S[AUGMENTED_STATE_DIM * TOTAL_MEAS];
 static double ukf_K_S_KT[AUGMENTED_STATE_DIM * AUGMENTED_STATE_DIM];
 static double ukf_KT[TOTAL_MEAS * AUGMENTED_STATE_DIM];

 // Compute predicted measurements
 double *predicted_measurements = ukf_pred_meas;
 compute_measurement_predictions(ukf, predicted_measurements);

 // Construct actual measurement vector
 double *actual_measurements = ukf_act_meas;

 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 int meas_offset = i * MEAS_DIM;

 // Copy position measurements
 for (int j = 0; j < 3; j++) {
 actual_measurements[meas_offset + j] = measurements->absolute_position[i][j];
 }

 // Copy velocity measurements
 for (int j = 0; j < 3; j++) {
 actual_measurements[meas_offset + 3 + j] = measurements->absolute_velocity[i][j];
 }

 // Copy formation/extended sensor measurements (relative pos, range, nonlinear)
 for (int j = 0; j < MEAS_DIM - 6; j++) {
 actual_measurements[meas_offset + 6 + j] = measurements->star_tracker[i][j];
 }
 }

 // Calculate measurement innovation
 double *innovation = ukf_innov;
 for (int i = 0; i < MEAS_DIM * NUM_SPACECRAFT; i++) {
 innovation[i] = actual_measurements[i] - predicted_measurements[i];
 }

 // Calculate innovation covariance and cross-correlation
 double *Pxy = ukf_Pxy;
 double *S = ukf_S;

 memset(Pxy, 0, AUGMENTED_STATE_DIM * MEAS_DIM * NUM_SPACECRAFT * sizeof(double));
 memset(S, 0, MEAS_DIM * NUM_SPACECRAFT * MEAS_DIM * NUM_SPACECRAFT * sizeof(double));

 // Precompute per-sigma-point measurements for proper UKF cross-correlation
 int total_meas = MEAS_DIM * NUM_SPACECRAFT;
 double *sigma_meas = ukf_sigma_meas;
 for (int sp_i = 0; sp_i < SIGMA_POINTS; sp_i++) {
 double *sp = ukf->sigma_points[sp_i];
 for (int sc = 0; sc < NUM_SPACECRAFT; sc++) {
 int so = sc * STATE_DIM;
 int mo = sc * MEAS_DIM;
 // Position
 for (int m = 0; m < 3; m++)
 sigma_meas[sp_i * total_meas + mo + m] = sp[so + m];
 // Velocity
 for (int m = 0; m < 3; m++)
 sigma_meas[sp_i * total_meas + mo + 3 + m] = sp[so + 3 + m];
 // Inter-spacecraft relative position to chief
 for (int m = 0; m < 3; m++)
 sigma_meas[sp_i * total_meas + mo + 6 + m] = sp[so + m] - sp[m];
 // Inter-spacecraft range
 {
 double rdx = sp[so + 0] - sp[0];
 double rdy = sp[so + 1] - sp[1];
 double rdz = sp[so + 2] - sp[2];
 sigma_meas[sp_i * total_meas + mo + 9] = sqrt(rdx*rdx + rdy*rdy + rdz*rdz);
 }
 // Nonlinear position/velocity coupling
 sigma_meas[sp_i * total_meas + mo + 10] =
 sw_sin(sp[so + 0]/1000.0) * sw_cos(sp[so + 1]/1000.0) + sw_tan(sp[so + 5]/10.0);
 // Specific orbital energy
 {
 double r_m = sqrt(sp[so+0]*sp[so+0] + sp[so+1]*sp[so+1] + sp[so+2]*sp[so+2]);
 double v2_m = sp[so+3]*sp[so+3] + sp[so+4]*sp[so+4] + sp[so+5]*sp[so+5];
 sigma_meas[sp_i * total_meas + mo + 11] = 0.5 * v2_m - MU_EARTH / r_m;
 }
 }
 }

 // For each sigma point - compute cross-correlation Pxy and innovation covariance S
 for (int i = 0; i < SIGMA_POINTS; i++) {
 double weight = ukf->sigma_weights_covariance[i];

 // For each state dimension
 for (int j = 0; j < AUGMENTED_STATE_DIM; j++) {
 // State difference from mean
 double state_diff = ukf->sigma_points[i][j] - ukf->state_estimate[j];

 // For each measurement dimension
 for (int k = 0; k < total_meas; k++) {
 // Per-sigma-point measurement difference from predicted mean
 double meas_diff_k = sigma_meas[i * total_meas + k] - predicted_measurements[k];

 // Cross-correlation: Pxy = Sum w_i * (x_i - x_mean) * (h(x_i) - y_mean)^T
 Pxy[j * total_meas + k] += weight * state_diff * meas_diff_k;

 // Innovation covariance: S = Sum w_i * (h(x_i) - y_mean) * (h(x_i) - y_mean)^T
 if (j < total_meas) {
 double meas_diff_j = sigma_meas[i * total_meas + j] - predicted_measurements[j];
 S[j * total_meas + k] += weight * meas_diff_j * meas_diff_k;
 }
 }
 }
 }

 /* sigma_meas is static — no free needed */

 // Add measurement noise to innovation covariance
 for (int i = 0; i < MEAS_DIM * NUM_SPACECRAFT; i++) {
 for (int j = 0; j < MEAS_DIM * NUM_SPACECRAFT; j++) {
 S[i * (MEAS_DIM * NUM_SPACECRAFT) + j] += ukf->measurement_noise[i][j];
 }
 }

 // Calculate Kalman gain: K = Pxy * S^-1
 double *S_inv = ukf_S_inv;
 sw_matrix_inverse(S, S_inv, MEAS_DIM * NUM_SPACECRAFT);

 double *K = ukf_K;
 sw_matrix_multiply(Pxy, S_inv, K, AUGMENTED_STATE_DIM, MEAS_DIM * NUM_SPACECRAFT, MEAS_DIM * NUM_SPACECRAFT);

 // Update state estimate: x = x + K * innovation
 for (int i = 0; i < AUGMENTED_STATE_DIM; i++) {
 for (int j = 0; j < MEAS_DIM * NUM_SPACECRAFT; j++) {
 ukf->state_estimate[i] += K[i * (MEAS_DIM * NUM_SPACECRAFT) + j] * innovation[j];
 }
 }

 // Update state covariance: P = P - K * S * K^T using software implementation
 double *K_S = ukf_K_S;
 sw_matrix_multiply(K, S, K_S, AUGMENTED_STATE_DIM, MEAS_DIM * NUM_SPACECRAFT, MEAS_DIM * NUM_SPACECRAFT);

 double *K_S_KT = ukf_K_S_KT;
 double *KT = ukf_KT;

 sw_matrix_transpose(K, KT, AUGMENTED_STATE_DIM, MEAS_DIM * NUM_SPACECRAFT);
 sw_matrix_multiply(K_S, KT, K_S_KT, AUGMENTED_STATE_DIM, MEAS_DIM * NUM_SPACECRAFT, AUGMENTED_STATE_DIM);

 // P = P - K_S_KT
 for (int i = 0; i < AUGMENTED_STATE_DIM; i++) {
 for (int j = 0; j < AUGMENTED_STATE_DIM; j++) {
 ukf->covariance[i][j] -= K_S_KT[i * AUGMENTED_STATE_DIM + j];
 }
 }

 /* ---- Covariance numerical-stability repair ---- */
 /* (a) Symmetry enforcement: P = 0.5*(P + P^T) */
 for (int i = 0; i < AUGMENTED_STATE_DIM; i++) {
 for (int j = i + 1; j < AUGMENTED_STATE_DIM; j++) {
 double avg = 0.5 * (ukf->covariance[i][j] + ukf->covariance[j][i]);
 ukf->covariance[i][j] = avg;
 ukf->covariance[j][i] = avg;
 }
 }
 /* (b) Positive-definiteness floor: clamp diagonal >= 1e-12 */
 for (int i = 0; i < AUGMENTED_STATE_DIM; i++) {
 if (ukf->covariance[i][i] < 1e-12) {
 ukf->covariance[i][i] = 1e-12;
 }
 }

 /* All buffers are static — no free needed (MISRA 21.3) */
}

/**
 * Compute control inputs for each spacecraft
 */
void compute_control_inputs(SpacecraftState *spacecraft, UKFState *ukf, FormationConfig *config, CollisionAssessment *collision) {
 (void)ukf;
 // Apply formation control
 apply_relative_orbit_control(spacecraft, config);

 // Apply attitude control
 apply_attitude_control(spacecraft, config);

 // Apply collision avoidance
 apply_collision_avoidance(spacecraft, collision);
}

/**
 * Apply relative orbit control to maintain formation
 */
void apply_relative_orbit_control(SpacecraftState *spacecraft, FormationConfig *config) {
 int leader_idx = config->leader_idx;

 // Compute Hill frame transformation for the leader
 double hill_position[3];
 double dcm_eci_to_hill[3][3];
 sw_hill_frame_transformation(spacecraft[leader_idx].position, spacecraft[leader_idx].velocity, hill_position, (double *)dcm_eci_to_hill);

 // For each follower spacecraft
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 if (i == leader_idx) continue; // Skip leader

 // Compute relative position in Hill frame
 double rel_pos_eci[3];
 for (int j = 0; j < 3; j++) {
 rel_pos_eci[j] = spacecraft[i].position[j] - spacecraft[leader_idx].position[j];
 }

 double rel_pos_hill[3] = {0};
 for (int j = 0; j < 3; j++) {
 for (int k = 0; k < 3; k++) {
 rel_pos_hill[j] += dcm_eci_to_hill[j][k] * rel_pos_eci[k];
 }
 }

 // Compute relative velocity in Hill frame
 double rel_vel_eci[3];
 for (int j = 0; j < 3; j++) {
 rel_vel_eci[j] = spacecraft[i].velocity[j] - spacecraft[leader_idx].velocity[j];
 }

 double rel_vel_hill[3] = {0};
 for (int j = 0; j < 3; j++) {
 for (int k = 0; k < 3; k++) {
 rel_vel_hill[j] += dcm_eci_to_hill[j][k] * rel_vel_eci[k];
 }
 }

 // Compute desired relative position in Hill frame
 double desired_rel_pos_hill[3];
 desired_rel_pos_hill[0] = config->desired_separation[leader_idx][i][0];
 desired_rel_pos_hill[1] = config->desired_separation[leader_idx][i][1];
 desired_rel_pos_hill[2] = config->desired_separation[leader_idx][i][2];

 // Compute position error in Hill frame
 double pos_error_hill[3];
 for (int j = 0; j < 3; j++) {
 pos_error_hill[j] = desired_rel_pos_hill[j] - rel_pos_hill[j];
 }

 // Mean motion
 double orbit_radius = sqrt(
 spacecraft[leader_idx].position[0] * spacecraft[leader_idx].position[0] +
 spacecraft[leader_idx].position[1] * spacecraft[leader_idx].position[1] +
 spacecraft[leader_idx].position[2] * spacecraft[leader_idx].position[2]
 );
 double n = sqrt(MU_EARTH / (orbit_radius * orbit_radius * orbit_radius));

 // Compute desired relative velocity in Hill frame (CW equations - linearized)
 double desired_rel_vel_hill[3];
 desired_rel_vel_hill[0] = 0;
 desired_rel_vel_hill[1] = -2 * n * desired_rel_pos_hill[0];
 desired_rel_vel_hill[2] = 0;

 // Compute velocity error in Hill frame
 double vel_error_hill[3];
 for (int j = 0; j < 3; j++) {
 vel_error_hill[j] = desired_rel_vel_hill[j] - rel_vel_hill[j];
 }

 // PD controller parameters
 double kp = 0.1;
 double kd = 0.5;

 // Compute control acceleration in Hill frame
 double control_acc_hill[3];
 for (int j = 0; j < 3; j++) {
 control_acc_hill[j] = kp * pos_error_hill[j] + kd * vel_error_hill[j];
 }

 // Transform control acceleration to ECI frame
 double control_acc_eci[3] = {0};
 for (int j = 0; j < 3; j++) {
 for (int k = 0; k < 3; k++) {
 // dcm_eci_to_hill is the transpose of dcm_hill_to_eci
 control_acc_eci[j] += dcm_eci_to_hill[k][j] * control_acc_hill[k];
 }
 }

 // Apply control acceleration as force
 for (int j = 0; j < 3; j++) {
 spacecraft[i].control_forces[j] = spacecraft[i].mass * control_acc_eci[j];
 }
 }
}

/**
 * Apply attitude control to align with desired orientation
 */
void apply_attitude_control(SpacecraftState *spacecraft, FormationConfig *config) {
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 // Compute error quaternion
 double q_current[4];
 memcpy(q_current, spacecraft[i].quaternion, 4 * sizeof(double));

 double q_desired[4];
 memcpy(q_desired, config->desired_attitude[i], 4 * sizeof(double));

 double q_error[4];
 double q_current_conj[4];
 sw_quaternion_conjugate(q_current, q_current_conj);

 // q_error = q_desired * q_current_conjugate
 sw_quaternion_multiply(q_desired, q_current_conj, q_error);

 // Extract error axis and angle
 double angle;
 double axis[3];

 // Avoid numerical issues
 if (fabs(q_error[0]) > 0.9999) {
 angle = 0;
 axis[0] = 1.0;
 axis[1] = 0.0;
 axis[2] = 0.0;
 } else {
 angle = 2 * sw_acos(q_error[0]);
 double s = sqrt(1 - q_error[0] * q_error[0]);
 axis[0] = q_error[1] / s;
 axis[1] = q_error[2] / s;
 axis[2] = q_error[3] / s;
 }

 // PD controller
 double kp = 0.1;
 double kd = 0.5;

 // Control torque = -kp*angle*axis - kd*omega
 for (int j = 0; j < 3; j++) {
 spacecraft[i].control_torques[j] = -kp * angle * axis[j] - kd * spacecraft[i].angular_rates[j];
 }
 }
}

/**
 * Assess collision probability between spacecraft in the formation
 */
void assess_collision_probability(SpacecraftState *spacecraft, UKFState *ukf, CollisionAssessment *collision) {
 (void)ukf;
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 for (int j = i + 1; j < NUM_SPACECRAFT; j++) {
 // Compute current relative position
 double rel_pos[3];
 for (int k = 0; k < 3; k++) {
 rel_pos[k] = spacecraft[j].position[k] - spacecraft[i].position[k];
 }

 // Current distance
 double distance = sqrt(
 rel_pos[0] * rel_pos[0] +
 rel_pos[1] * rel_pos[1] +
 rel_pos[2] * rel_pos[2]
 );

 // Compute relative velocity
 double rel_vel[3];
 for (int k = 0; k < 3; k++) {
 rel_vel[k] = spacecraft[j].velocity[k] - spacecraft[i].velocity[k];
 }

 // Project forward in time
 double min_distance = distance;
 double time_to_min_distance = 0.0;

 // Numerical search for minimum distance
 for (double t = 0.0; t <= collision->time_horizon; t += 1.0) {
 double future_rel_pos[3];
 for (int k = 0; k < 3; k++) {
 future_rel_pos[k] = rel_pos[k] + rel_vel[k] * t;
 }

 double future_distance = sqrt(
 future_rel_pos[0] * future_rel_pos[0] +
 future_rel_pos[1] * future_rel_pos[1] +
 future_rel_pos[2] * future_rel_pos[2]
 );

 if (future_distance < min_distance) {
 min_distance = future_distance;
 time_to_min_distance = t;
 }
 }
 (void)time_to_min_distance;

 // Compute collision probability based on minimum distance
 // Simple model: probability decreases exponentially with distance
 double prob;
 if (min_distance < collision->minimum_safe_distance) {
 prob = 1.0;
 } else {
 prob = exp(-(min_distance - collision->minimum_safe_distance) / 10.0);
 }

 // Update collision probability
 collision->collision_probability[i][j] = prob;
 collision->collision_probability[j][i] = prob;
 }
 }
}

/**
 * Apply collision avoidance forces to spacecraft
 */
void apply_collision_avoidance(SpacecraftState *spacecraft, CollisionAssessment *collision) {
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 double avoidance_force[3] = {0, 0, 0};

 for (int j = 0; j < NUM_SPACECRAFT; j++) {
 if (i == j) continue;

 // If collision probability is significant
 if (collision->collision_probability[i][j] > 0.01) {
 // Compute relative position vector
 double rel_pos[3];
 for (int k = 0; k < 3; k++) {
 rel_pos[k] = spacecraft[i].position[k] - spacecraft[j].position[k];
 }

 // Normalize
 double distance = sqrt(
 rel_pos[0] * rel_pos[0] +
 rel_pos[1] * rel_pos[1] +
 rel_pos[2] * rel_pos[2]
 );

 if (distance < 1e-6) continue; // Avoid division by zero

 for (int k = 0; k < 3; k++) {
 rel_pos[k] /= distance;
 }

 // Compute avoidance force (pointing away from other spacecraft)
 // Force magnitude increases with collision probability
 double force_magnitude = collision->avoidance_gain * collision->collision_probability[i][j];

 for (int k = 0; k < 3; k++) {
 avoidance_force[k] += force_magnitude * rel_pos[k];
 }
 }
 }

 // Apply avoidance force
 for (int k = 0; k < 3; k++) {
 spacecraft[i].control_forces[k] += avoidance_force[k];
 }
 }
}

/**
 * Simulate measurements for all spacecraft
 */
void simulate_measurements(SpacecraftState *spacecraft, RelativeMeasurements *measurements) {
 // For each pair of spacecraft
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 for (int j = 0; j < NUM_SPACECRAFT; j++) {
 if (i == j) {
 measurements->measurement_valid[i][j] = 0;
 continue;
 }

 // Compute relative position
 for (int k = 0; k < 3; k++) {
 measurements->relative_position[i][j][k] = spacecraft[j].position[k] - spacecraft[i].position[k];
 }

 // Compute relative velocity
 for (int k = 0; k < 3; k++) {
 measurements->relative_velocity[i][j][k] = spacecraft[j].velocity[k] - spacecraft[i].velocity[k];
 }

 // Compute relative quaternion using software implementation
 sw_compute_relative_quaternion(spacecraft[i].quaternion, spacecraft[j].quaternion, measurements->relative_attitude[i][j]);

 // Compute range measurement
 double range_sq = 0;
 for (int k = 0; k < 3; k++) {
 range_sq += measurements->relative_position[i][j][k] * measurements->relative_position[i][j][k];
 }
 measurements->range_measurements[i][j] = sqrt(range_sq);

 // Simple line-of-sight model for bearing
 double rel_pos_normalized[3];
 for (int k = 0; k < 3; k++) {
 rel_pos_normalized[k] = measurements->relative_position[i][j][k] / measurements->range_measurements[i][j];
 }

 // Azimuth and elevation from relative state
 measurements->bearing_measurements[i][j][0] = sw_atan2(rel_pos_normalized[1], rel_pos_normalized[0]);
 measurements->bearing_measurements[i][j][1] = sw_asin(rel_pos_normalized[2]);

 // Set measurement as valid
 measurements->measurement_valid[i][j] = 1;
 }

 // Simulate absolute position measurements (e.g., from GPS)
 for (int k = 0; k < 3; k++) {
 measurements->absolute_position[i][k] = spacecraft[i].position[k];
 measurements->absolute_velocity[i][k] = spacecraft[i].velocity[k];
 }

 // Simulate formation-keeping sensor measurements
 // Inter-spacecraft relative position to chief
 int chief = 0;
 for (int k = 0; k < 3; k++) {
 measurements->star_tracker[i][k] = spacecraft[i].position[k] - spacecraft[chief].position[k];
 }

 // Inter-spacecraft range
 {
 double dx = spacecraft[i].position[0] - spacecraft[chief].position[0];
 double dy = spacecraft[i].position[1] - spacecraft[chief].position[1];
 double dz = spacecraft[i].position[2] - spacecraft[chief].position[2];
 measurements->star_tracker[i][3] = sqrt(dx*dx + dy*dy + dz*dz);
 }

 // Nonlinear position/velocity coupling measurement
 measurements->star_tracker[i][4] = sw_sin(spacecraft[i].position[0]/1000.0)
 * sw_cos(spacecraft[i].position[1]/1000.0)
 + sw_tan(spacecraft[i].velocity[2]/10.0);

 // Specific orbital energy (vis-viva)
 {
 double r_m = sqrt(spacecraft[i].position[0]*spacecraft[i].position[0]
 + spacecraft[i].position[1]*spacecraft[i].position[1]
 + spacecraft[i].position[2]*spacecraft[i].position[2]);
 double v2 = spacecraft[i].velocity[0]*spacecraft[i].velocity[0]
 + spacecraft[i].velocity[1]*spacecraft[i].velocity[1]
 + spacecraft[i].velocity[2]*spacecraft[i].velocity[2];
 measurements->star_tracker[i][5] = 0.5 * v2 - MU_EARTH / r_m;
 }
 }
}

/**
 * Add noise to measurements to simulate real sensor behavior
 */
void add_measurement_noise(RelativeMeasurements *measurements, UKFState *ukf) {
 (void)ukf;
 // For each pair of spacecraft
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 for (int j = 0; j < NUM_SPACECRAFT; j++) {
 if (!measurements->measurement_valid[i][j]) continue;

 // Add noise to relative position (e.g., 5cm standard deviation)
 for (int k = 0; k < 3; k++) {
 measurements->relative_position[i][j][k] += rand_double(-0.05, 0.05);
 }

 // Add noise to relative velocity (e.g., 1mm/s standard deviation)
 for (int k = 0; k < 3; k++) {
 measurements->relative_velocity[i][j][k] += rand_double(-0.001, 0.001);
 }

 // Add noise to range measurement (e.g., 1cm standard deviation)
 measurements->range_measurements[i][j] += rand_double(-0.01, 0.01);

 // Add noise to bearing measurements (e.g., 0.1 degree standard deviation)
 for (int k = 0; k < 2; k++) {
 measurements->bearing_measurements[i][j][k] += rand_double(-0.001745, 0.001745); // 0.1 deg in rad
 }
 }

 // Add noise to absolute position (e.g., GPS noise ~2m)
 for (int k = 0; k < 3; k++) {
 measurements->absolute_position[i][k] += rand_double(-2.0, 2.0);
 }

 // Add noise to absolute velocity (e.g., 0.02 m/s)
 for (int k = 0; k < 3; k++) {
 measurements->absolute_velocity[i][k] += rand_double(-0.02, 0.02);
 }

 // Add noise to star tracker measurements (formation sensor noise)
 // Relative position noise: 0.1m
 for (int k = 0; k < 3; k++) {
 measurements->star_tracker[i][k] += rand_double(-0.1, 0.1);
 }

 // Range noise: 0.05m
 measurements->star_tracker[i][3] += rand_double(-0.05, 0.05);

 // Add noise to extended measurements
 for (int k = 4; k < MEAS_DIM; k++) {
 measurements->star_tracker[i][k] += rand_double(-0.001, 0.001);
 }
 }
}

/**
 * Main simulation function - runs the algorithm for a specified number of timesteps
 */
double run_formation_simulation(int timesteps) {
 // Initialize formation of spacecraft
 SpacecraftState spacecraft[NUM_SPACECRAFT];
 FormationConfig config;
 RelativeMeasurements measurements;
 UKFState ukf;
 CollisionAssessment collision;

 // Fixed simulation time step
 double dt = 0.1; // 10 Hz update rate

 // Initialize formation configuration
 config.leader_idx = 0;

 // Reference orbit (LEO, nearly circular)
 config.reference_orbit[0] = 7000e3; // Semi-major axis (m)
 config.reference_orbit[1] = 0.001; // Eccentricity
 config.reference_orbit[2] = 51.6 * M_PI / 180.0; // Inclination (rad) - ISS-like
 config.reference_orbit[3] = 0.0; // Right ascension of ascending node (rad)
 config.reference_orbit[4] = 0.0; // Argument of perigee (rad)
 config.reference_orbit[5] = 0.0; // Mean anomaly (rad)

 // Initialize the large configuration matrix
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 // Create a configuration that will vary with trigonometric patterns
 config.config_matrix[i][j] = 0.01 * sw_sin(0.1 * i) * sw_cos(0.1 * j);
 }
 }

 // Initialize spherical harmonics coefficients
 for (int n = 0; n < HARMONICS_DEGREE; n++) {
 for (int m = 0; m <= n; m++) {
 // Orbital perturbation coefficients
 if (n == 2 && m == 0) {
 ukf.harmonics_coefficients[n][m] = -J2_EARTH;
 } else if (n == 3 && m == 0) {
 ukf.harmonics_coefficients[n][m] = J3_EARTH;
 } else if (n == 4 && m == 0) {
 ukf.harmonics_coefficients[n][m] = -J4_EARTH;
 } else {
 // Random small coefficients for higher order terms
 ukf.harmonics_coefficients[n][m] = 1e-6 * rand_double(-1.0, 1.0);
 }
 }
 }

 // Initialize desired formation shape (reference case: tetrahedron)
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 for (int j = 0; j < NUM_SPACECRAFT; j++) {
 if (i == j) {
 config.desired_separation[i][j][0] = 0.0;
 config.desired_separation[i][j][1] = 0.0;
 config.desired_separation[i][j][2] = 0.0;
 } else {
 // Create a complex formation with trigonometric patterns
 double angle1 = 2.0 * M_PI * j / NUM_SPACECRAFT;
 double angle2 = M_PI * i / NUM_SPACECRAFT;
 double radius = 100.0; // 100m nominal separation

 // Using software trigonometry
 config.desired_separation[i][j][0] = radius * sw_cos(angle1) * sw_sin(angle2);
 config.desired_separation[i][j][1] = radius * sw_sin(angle1) * sw_sin(angle2);
 config.desired_separation[i][j][2] = radius * sw_cos(angle2);
 }
 }
 }

 // Set desired attitudes (all spacecraft pointing in the same direction initially)
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 config.desired_attitude[i][0] = 1.0; // Identity quaternion
 config.desired_attitude[i][1] = 0.0;
 config.desired_attitude[i][2] = 0.0;
 config.desired_attitude[i][3] = 0.0;
 }

 // Initialize spacecraft states based on configuration
 initialize_spacecraft_states(spacecraft, &config);

 // Initialize UKF for state estimation
 initialize_ukf(&ukf);

 // Initialize collision assessment
 collision.minimum_safe_distance = 10.0; // 10 meter minimum safe distance
 collision.avoidance_gain = 2.0; // Gain for avoidance controller
 collision.time_horizon = 60.0; // 60 second prediction horizon

 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 for (int j = 0; j < NUM_SPACECRAFT; j++) {
 collision.collision_probability[i][j] = 0.0;
 }
 }

 // Initialize risk matrix
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 collision.risk_matrix[i][j] = 0.01 * rand_double(0.0, 1.0);
 }
 }

 // Static matrices for computations (MISRA 21.3: no malloc/free)
 static double sim_A[MATRIX_DIM * MATRIX_DIM];
 static double sim_B[MATRIX_DIM * MATRIX_DIM];
 static double sim_C[MATRIX_DIM * MATRIX_DIM];
 static double sim_D[MATRIX_DIM * MATRIX_DIM];
 static double sim_E[MATRIX_DIM * MATRIX_DIM];
 static double sim_F[MATRIX_DIM * MATRIX_DIM];
 double *A = sim_A;
 double *B = sim_B;
 double *C = sim_C;
 double *D = sim_D;
 double *E = sim_E;
 double *F = sim_F;

 // Initialize matrices with random values for computation
 randomize_matrix(A, MATRIX_DIM, MATRIX_DIM);
 randomize_matrix(B, MATRIX_DIM, MATRIX_DIM);
 randomize_matrix(E, MATRIX_DIM, MATRIX_DIM);

 // For timing
 clock_t start, end;
 double elapsed_time;

 // Start timing
 start = clock();

 // Main simulation loop
 for (int iter = 0; iter < timesteps; iter++) {
 // 1. Simulate sensor measurements
 simulate_measurements(spacecraft, &measurements);
 add_measurement_noise(&measurements, &ukf);

 // 2. State estimation using UKF
 // Generate sigma points
 generate_sigma_points(&ukf);

 // Propagate sigma points through dynamics
 propagate_sigma_points(&ukf, dt);

 // Predict measurements
 double predicted_measurements[MEAS_DIM * NUM_SPACECRAFT];
 compute_measurement_predictions(&ukf, predicted_measurements);

 // Update UKF state with measurements
 update_ukf_state(&ukf, &measurements);

 // 3. Assess collision probabilities
 assess_collision_probability(spacecraft, &ukf, &collision);

 // 4. Compute control inputs (including collision avoidance)
 compute_control_inputs(spacecraft, &ukf, &config, &collision);

 // 5. Apply controls and propagate dynamics
 propagate_states(spacecraft, &config, dt);

 // 6. Apply high-order gravity perturbations
 for (int i = 0; i < NUM_SPACECRAFT; i++) {
 apply_gravity_perturbations(&spacecraft[i], &ukf, dt);
 }

 /* 7. Update inter-satellite baseline covariance from chief/deputy filters */
 compute_intersatellite_baseline_covariance(A, B, C, D, E, F);

 /* 8. Convert relative states to curvilinear orbit elements + bearing observables */
 compute_relative_orbit_geometry(A, MATRIX_DIM);
 compute_relative_orbit_geometry(B, MATRIX_DIM);

 /* 9. Formation reconfiguration delta-V cost matrix (Gauss variational eqs) */
 sw_matrix_multiply(A, B, C, MATRIX_DIM, MATRIX_DIM, MATRIX_DIM);
 sw_matrix_transpose(C, D, MATRIX_DIM, MATRIX_DIM);
 sw_matrix_multiply(D, A, E, MATRIX_DIM, MATRIX_DIM, MATRIX_DIM);
 sw_matrix_transpose(E, F, MATRIX_DIM, MATRIX_DIM);
 sw_matrix_multiply(F, B, A, MATRIX_DIM, MATRIX_DIM, MATRIX_DIM);
 sw_matrix_inverse(A, C, MATRIX_DIM);

 /* 10. Update collision risk via Mahalanobis distance and probability integral */
 for (int i = 0; i < MATRIX_DIM * MATRIX_DIM; i++) {
 double sigma = fabs(C[i]) + 1e-3;
 /* Mahalanobis distance in miss-distance plane */
 double d_miss = sqrt(C[i] * C[i] + 0.01) / sigma;
 /* Approximate collision probability: 1 - exp(-0.5*d^2) */
 double p_col = 1.0 - sw_cos(d_miss * 0.1) * sw_cos(d_miss * 0.1);
 C[i] = fmax(0.0, fmin(1.0, p_col));
 }

 /* Update risk matrix: angular separation between each spacecraft pair */
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 double rij = collision.risk_matrix[i][j];

 /* Angular separation in Hill frame */
 double theta_ij = sw_atan2((double)(i + 1), (double)(j + 1));
 double phi_ij = sw_asin(fmax(-0.99, fmin(0.99, rij * 0.01)));

 /* Projected miss distance */
 double dx = sw_cos(theta_ij) * sw_cos(phi_ij);
 double dy = sw_sin(theta_ij) * sw_cos(phi_ij);
 double miss_dist = sqrt(dx * dx + dy * dy + 1e-6);

 collision.risk_matrix[i][j] = miss_dist;
 }
 }
 }

 // End timing
 end = clock();
 elapsed_time = ((double)(end - start)) / CLOCKS_PER_SEC;

 /* All buffers are static — no free needed (MISRA 21.3) */

 return elapsed_time;
}

/**
 * Main function
 */
int main(int argc, char *argv[]) {
 int timesteps = 100;

 if (argc > 1) {
 timesteps = atoi(argv[1]);
 }

 FLIGHT_LOG("[FFS] iter=%d\n", timesteps);

 double elapsed_time = run_formation_simulation(timesteps);

 FLIGHT_LOG("[FFS] done (%.6fs)\n", elapsed_time);
 FLIGHT_LOG("Average time per iteration: %.6f seconds\n", elapsed_time / timesteps);

 return 0;
}
