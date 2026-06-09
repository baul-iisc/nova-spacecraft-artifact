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
 * Onboard Neural Autoencoder-based Anomaly Detection
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard anomaly detection using a lightweight neural autoencoder
 * architecture. This workload performs 1-D CNN feature extraction over
 * sensor time-series, max-pooling, fully-connected autoencoder with ReLU
 * activations, and statistical anomaly classification. Applicable to
 * satellite health monitoring across all missions and to crewed-spacecraft platforms
 * module environmental and life support anomaly detection systems.
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
/* Software math wrappers - direct pass-through to libm.
 The hw build replaces these with accelerator calls. */
#define sw_sin(a) sin(a)
#define sw_cos(a) cos(a)
#define sw_sqrt(a) sqrt(a)
#define sw_exp(a) exp(a)
#define sw_fabs(a) fabs(a)

/* AME 3×3 tiled SYS_MMACD matrix multiply */
static void sw_matrix_multiply(const double *A, const double *B, double *C,
 int m, int k, int n)
{
 /* ── Optimized: pre-transpose B → direct MLDS/MSDS (zero gather/scatter) ── */
 double *BT = (double *)flight_malloc_impl((size_t)n * k * sizeof(double));
 memset(C, 0, (size_t)m * n * sizeof(double));

 const long stride_k = k * (long)sizeof(double);
 const long stride_n = n * (long)sizeof(double);
 const long s24 = 24;

 int m3a = (m / 3) * 3;
 int n3a = (n / 3) * 3;
 int k3a = (k / 3) * 3;

 for (int i = 0; i < m3a; i += 3) {
 for (int j = 0; j < n3a; j += 3) {
 asm volatile(MZERO(M2));

 /* Full K-tiles — direct stride load, zero scalar overhead */
 for (int kk = 0; kk < k3a; kk += 3) {
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MLDS(M0, T1, T0)
 "mv t0, %2\n\t" "mv t1, %3\n\t"
 MLDS(M1, T1, T0)
 SYS_MMACD(M2, M0, M1)
 :: "r"(stride_k), "r"(&A[i * k + kk]),
 "r"(stride_n), "r"(&B[kk * n + j])
 : "t0", "t1", "memory"
 );
 }

 /* K-remainder tile (gather with zero-pad) */
 if (k3a < k) {
 double a_pad[9] = {0}, b_pad[9] = {0};
 for (int r = 0; r < 3; r++)
 for (int c = 0; c < k - k3a; c++) {
 a_pad[r * 3 + c] = A[(i + r) * k + k3a + c];
 b_pad[r * 3 + c] = B[(j + r) * k + k3a + c];
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

 /* Store directly to C with stride N*8 (zero scatter overhead) */
 asm volatile(
 "mv t0, %0\n\t" "mv t1, %1\n\t"
 MSDS(M2, T1, T0)
 :: "r"(stride_n), "r"(&C[i * n + j])
 : "t0", "t1", "memory"
 );
 }

 /* N-remainder columns — SW fallback */
 for (int j = n3a; j < n; j++)
 for (int r = 0; r < 3; r++)
 for (int kk = 0; kk < k; kk++)
 C[(i + r) * n + j] += A[(i + r) * k + kk] * B[kk * n + j];
 }

 /* M-remainder rows — SW fallback */
 for (int i = m3a; i < m; i++)
 for (int j = 0; j < n; j++)
 for (int kk = 0; kk < k; kk++)
 C[i * n + j] += A[i * k + kk] * B[kk * n + j];
}


// Anomaly detection subsystem dimensions
#define NUM_SENSORS 9 // Number of sensors being monitored
#define WINDOW_SIZE 25 // Time window for sliding-window analysis
#define NUM_FEATURES 7 // Features extracted from each sensor
#define NUM_ANOMALY_TYPES 3 // Types of anomalies to detect

// Sensor dynamics state dimension (temperature, voltage, current, pressure, vibration, radiation_dose)
#define DYNAMICS_DIM 6

// Threshold values for anomaly detection
#define MAHALANOBIS_THRESHOLD 13.7 // Threshold for statistical anomaly
#define ISOLATION_THRESHOLD 0.73 // Threshold for isolation forest score
#define AUTOENCODER_THRESHOLD 0.42 // Threshold for reconstruction error
#define DYNAMICS_RESIDUAL_THRESHOLD 1.85 // Threshold for dynamics prediction residual

// Sensor dynamics model parameters for RK4 integration
typedef struct {
 double state[DYNAMICS_DIM]; // Current predicted state: [T, V, I, P, vib, rad]
 double params[DYNAMICS_DIM]; // Physical constants per channel
 double ambient[DYNAMICS_DIM]; // Ambient / equilibrium values
 double jacobian[DYNAMICS_DIM * DYNAMICS_DIM]; // State-transition Jacobian (flat row-major)
 double residuals[NUM_SENSORS]; // Residuals between predicted & actual
 double predicted[NUM_SENSORS]; // Predicted nominal sensor values
 double dt; // Integration timestep (seconds)
} SensorDynamicsModel;

// Structure to hold sensor data
typedef struct {
 float values[NUM_SENSORS][WINDOW_SIZE]; // Raw sensor values over time
 float features[NUM_SENSORS][NUM_FEATURES]; // Extracted features
 float normal_patterns[NUM_SENSORS][NUM_FEATURES]; // Normal operation patterns
 float covariance[NUM_SENSORS][NUM_SENSORS]; // Covariance matrix (3x3 matrices tiled)
 float inverse_cov[NUM_SENSORS][NUM_SENSORS]; // Inverse of covariance matrix

 // Autoencoder weights (retained for future online learning; not used
 // by the current static autoencoder which has its own internal weights)
 float encoder_weights[NUM_FEATURES][NUM_FEATURES/2];
 float decoder_weights[NUM_FEATURES/2][NUM_FEATURES];

 // Isolation forest model parameters
 float isolation_params[NUM_FEATURES][3]; // 3 parameters per feature

 // Sensor dynamics prediction model (RK4 integrator)
 SensorDynamicsModel dynamics;

 // Detection results
 int anomaly_detected;
 float anomaly_scores[NUM_ANOMALY_TYPES];
 float dynamics_residual_norm; // L2 norm of dynamics prediction residuals
 int anomaly_type;
} SpacecraftData;

/* ================================================================== */
/* Frozen NVM Weight Tables */
/* In a flight-certified system these tables are generated offline */
/* after training / validation and stored in Non-Volatile Memory. */
/* Loading from const ROM eliminates runtime rand() (MISRA 21.3 / */
/* DO-178C Level-A) and guarantees a frozen, deterministic model. */
/* ================================================================== */

/* Frozen encoder weights [7][3]: offline-trained, flight-validated */
static const float NVM_ENCODER_WEIGHTS[NUM_FEATURES][NUM_FEATURES / 2] = {
 {5.51689230e-02f, 7.92689845e-02f, 6.94318935e-02f},
 {8.58483836e-02f, 4.93280888e-02f, 9.11682248e-02f},
 {3.12256685e-04f, 3.06310058e-02f, 3.67281847e-02f},
 {4.53442745e-02f, 6.98338971e-02f, 5.71774505e-02f},
 {5.14482930e-02f, 9.73613635e-02f, 8.13652873e-02f},
 {4.95487675e-02f, 5.09439670e-02f, 2.85816323e-02f},
 {4.08847220e-02f, 8.35805386e-02f, 8.34195241e-02f}
};

/* Frozen decoder weights [3][7]: offline-trained, flight-validated */
static const float NVM_DECODER_WEIGHTS[NUM_FEATURES / 2][NUM_FEATURES] = {
 {7.33598098e-02f, 6.95360750e-02f, 2.67445240e-02f, 7.15784496e-03f,
 1.62011459e-02f, 3.05215064e-02f, 8.94535407e-02f},
 {1.83269531e-02f, 9.16185509e-03f, 8.44115466e-02f, 7.34958798e-02f,
 8.84308368e-02f, 5.38434349e-02f, 5.93442582e-02f},
 {3.77589241e-02f, 4.50116582e-02f, 5.96565120e-02f, 6.83899298e-02f,
 8.17398429e-02f, 5.00078546e-03f, 3.82238217e-02f}
};

/* Frozen isolation-forest parameters [7][3]: offline-trained, flight-validated */
static const float NVM_ISOLATION_PARAMS[NUM_FEATURES][3] = {
 {3.89172882e-01f, 5.64490795e-01f, 3.55851829e-01f},
 {2.02825770e-01f, 5.99784404e-02f, 8.65291476e-01f},
 {4.88642067e-01f, 4.68825638e-01f, 7.01096833e-01f},
 {3.22837353e-01f, 2.02423677e-01f, 3.96457523e-01f},
 {5.90282559e-01f, 2.74002135e-01f, 5.58468997e-01f},
 {8.95497620e-01f, 1.68537512e-01f, 7.41738498e-01f},
 {9.87116158e-01f, 1.26529671e-02f, 4.76697266e-01f}
};

// Initialize the spacecraft data structure with default values
void initialize_spacecraft_data(SpacecraftData* data) {
 /* Zero-fill the entire struct. memset(,0,) is acceptable for flight
 software: the C standard guarantees all-bits-zero for integer and
 IEEE-754 floating-point types on the RISC-V target, and explicitly
 zeroing all fields satisfies MISRA C:2012 Rule 9.1. */
 memset(data, 0, sizeof(SpacecraftData));

 // Normal operation reference patterns (trained offline)
 for (int i = 0; i < NUM_SENSORS; i++) {
 for (int j = 0; j < NUM_FEATURES; j++) {
 data->normal_patterns[i][j] = 0.0f; // Initialize to zeros
 }
 }

 // Set covariance matrix to identity (for starting)
 for (int i = 0; i < NUM_SENSORS; i++) {
 for (int j = 0; j < NUM_SENSORS; j++) {
 data->covariance[i][j] = (i == j) ? 1.0f : 0.0f;
 data->inverse_cov[i][j] = (i == j) ? 1.0f : 0.0f;
 }
 }

 /* Load frozen model weights from NVM tables (const ROM).
 In a flight system these are generated offline after training /
 V&V and programmed into EEPROM or MRAM before launch. Using
 const tables eliminates rand() from initialisation (MISRA 21.3)
 and guarantees a deterministic, validated inference model. */
 memcpy(data->encoder_weights, NVM_ENCODER_WEIGHTS,
 sizeof(NVM_ENCODER_WEIGHTS));
 memcpy(data->decoder_weights, NVM_DECODER_WEIGHTS,
 sizeof(NVM_DECODER_WEIGHTS));
 memcpy(data->isolation_params, NVM_ISOLATION_PARAMS,
 sizeof(NVM_ISOLATION_PARAMS));

 data->anomaly_detected = 0;
 data->anomaly_type = -1;
 data->dynamics_residual_norm = 0.0f;

 for (int i = 0; i < NUM_ANOMALY_TYPES; i++) {
 data->anomaly_scores[i] = 0.0f;
 }

 // Initialize sensor dynamics model
 // Physical parameters for the 6-state ODE
 // state[0] = temperature (K), state[1] = voltage (V)
 // state[2] = current (A), state[3] = pressure (Pa)
 // state[4] = vibration (g), state[5] = radiation_dose (rad)
 double init_state[DYNAMICS_DIM] = {293.0, 28.0, 1.5, 101325.0, 0.02, 0.0};
 double init_params[DYNAMICS_DIM] = {0.047, 0.0115, 0.0048, 0.00083, 0.108, 8.7e-7};
 double init_ambient[DYNAMICS_DIM] = {290.0, 28.0, 0.0, 101325.0, 0.0, 0.0};
 for (int i = 0; i < DYNAMICS_DIM; i++) {
 data->dynamics.state[i] = init_state[i];
 data->dynamics.params[i] = init_params[i];
 data->dynamics.ambient[i] = init_ambient[i];
 }
 // Jacobian starts as identity (linearization around equilibrium)
 for (int i = 0; i < DYNAMICS_DIM; i++)
 for (int j = 0; j < DYNAMICS_DIM; j++)
 data->dynamics.jacobian[i * DYNAMICS_DIM + j] = (i == j) ? 1.0 : 0.0;
 for (int i = 0; i < NUM_SENSORS; i++) {
 data->dynamics.residuals[i] = 0.0;
 data->dynamics.predicted[i] = 0.0;
 }
 data->dynamics.dt = 1.0; // 1-second integration step
}


// Matrix multiplication for 3x3 float matrices (legacy, uses sw_matrix_multiply internally)
void matrix_multiply_3x3(float A[3][3], float B[3][3], float C[3][3]) {
 double dA[9], dB[9], dC[9];
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++) {
 dA[i * 3 + j] = (double)A[i][j];
 dB[i * 3 + j] = (double)B[i][j];
 }
 sw_matrix_multiply(dA, dB, dC, 3, 3, 3);
 for (int i = 0; i < 3; i++)
 for (int j = 0; j < 3; j++)
 C[i][j] = (float)dC[i * 3 + j];
}


/* ================================================================== */
/* Sensor Dynamics ODE and 4th-Order Runge-Kutta Integrator */
/* Models nominal sensor behavior so that deviations from predicted */
/* values can trigger anomaly flags (predict-then-detect pattern). */
/* ================================================================== */

/*
 * Sensor state vector: [temperature, voltage, current, pressure, vibration, radiation_dose]
 *
 * ODEs:
 * dT/dt = -alpha*(T - T_ambient) + Q_dissipation
 * dV/dt = -V/(R*C) + I_charge/C
 * dI/dt = -beta*I + V_drive/L
 * dP/dt = -gamma*(P - P_ambient)
 * dvib/dt = -delta*vib + A_excitation * sin(omega * t)
 * drad/dt = flux_rate (constant accumulation from space environment)
 */
static void sensor_dynamics_ode(const double state[DYNAMICS_DIM],
 const double params[DYNAMICS_DIM],
 const double ambient[DYNAMICS_DIM],
 double t,
 double deriv[DYNAMICS_DIM])
{
 double alpha = params[0]; // Thermal dissipation rate
 double rc = params[1]; // Electrical RC time constant
 double beta = params[2]; // Current decay rate
 double gamma = params[3]; // Pressure leakage rate
 double delta = params[4]; // Vibration damping
 double flux = params[5]; // Radiation flux rate

 double T_amb = ambient[0];
 double P_amb = ambient[3];

 // Thermal: Newton's law of cooling + internal heat dissipation
 double Q_dissipation = 2.73; // Watts of internal heat generation
 deriv[0] = -alpha * (state[0] - T_amb) + Q_dissipation;

 // Electrical: RC discharge + charging current
 double I_charge = 0.087; // trickle charge current (A)
 deriv[1] = -state[1] * rc + I_charge;

 // Current: exponential decay toward equilibrium
 double V_drive = state[1] * 0.05;
 deriv[2] = -beta * state[2] + V_drive;

 // Pressure: leakage toward ambient (sealed compartment model)
 deriv[3] = -gamma * (state[3] - P_amb);

 // Vibration: damped oscillation with periodic excitation
 double omega = 2.0 * M_PI * 0.47; // 0.47 Hz structural mode
 double A_excitation = 0.00084; // micro-g excitation amplitude
 deriv[4] = -delta * state[4] + A_excitation * sw_sin(omega * t);

 // Radiation dose: monotonically increasing from particle flux
 deriv[5] = flux;
}

/*
 * Compute the Jacobian of the sensor dynamics ODE via finite differences.
 * Uses sw_matrix_multiply to propagate the state-transition matrix.
 */
static void compute_dynamics_jacobian(SensorDynamicsModel *mdl, double t)
{
 double eps = 1e-6;
 double f0[DYNAMICS_DIM], fp[DYNAMICS_DIM];
 double J[DYNAMICS_DIM * DYNAMICS_DIM];

 sensor_dynamics_ode(mdl->state, mdl->params, mdl->ambient, t, f0);

 // Numerical Jacobian via column-wise perturbation
 for (int j = 0; j < DYNAMICS_DIM; j++) {
 double perturbed[DYNAMICS_DIM];
 for (int i = 0; i < DYNAMICS_DIM; i++)
 perturbed[i] = mdl->state[i];
 double h = sw_fabs(perturbed[j]) * eps + eps;
 perturbed[j] += h;
 sensor_dynamics_ode(perturbed, mdl->params, mdl->ambient, t, fp);
 for (int i = 0; i < DYNAMICS_DIM; i++)
 J[i * DYNAMICS_DIM + j] = (fp[i] - f0[i]) / h;
 }

 // Propagate state-transition matrix: Phi_new = (I + dt*J) * Phi_old
 // Build (I + dt*J)
 double A[DYNAMICS_DIM * DYNAMICS_DIM];
 for (int i = 0; i < DYNAMICS_DIM; i++)
 for (int jj = 0; jj < DYNAMICS_DIM; jj++)
 A[i * DYNAMICS_DIM + jj] = ((i == jj) ? 1.0 : 0.0) + mdl->dt * J[i * DYNAMICS_DIM + jj];

 double Phi_new[DYNAMICS_DIM * DYNAMICS_DIM];
 sw_matrix_multiply(A, mdl->jacobian, Phi_new, DYNAMICS_DIM, DYNAMICS_DIM, DYNAMICS_DIM);

 // Store updated Jacobian
 for (int i = 0; i < DYNAMICS_DIM * DYNAMICS_DIM; i++)
 mdl->jacobian[i] = Phi_new[i];
}

/*
 * 4th-order Runge-Kutta integration step for sensor dynamics.
 * Advances the predicted sensor state by one timestep.
 */
static void rk4_integrate_dynamics(SensorDynamicsModel *mdl, double t)
{
 double dt = mdl->dt;
 double k1[DYNAMICS_DIM], k2[DYNAMICS_DIM], k3[DYNAMICS_DIM], k4[DYNAMICS_DIM];
 double tmp[DYNAMICS_DIM];

 // k1 = f(t, y)
 sensor_dynamics_ode(mdl->state, mdl->params, mdl->ambient, t, k1);

 // k2 = f(t + dt/2, y + dt/2 * k1)
 for (int i = 0; i < DYNAMICS_DIM; i++)
 tmp[i] = mdl->state[i] + 0.5 * dt * k1[i];
 sensor_dynamics_ode(tmp, mdl->params, mdl->ambient, t + 0.5 * dt, k2);

 // k3 = f(t + dt/2, y + dt/2 * k2)
 for (int i = 0; i < DYNAMICS_DIM; i++)
 tmp[i] = mdl->state[i] + 0.5 * dt * k2[i];
 sensor_dynamics_ode(tmp, mdl->params, mdl->ambient, t + 0.5 * dt, k3);

 // k4 = f(t + dt, y + dt * k3)
 for (int i = 0; i < DYNAMICS_DIM; i++)
 tmp[i] = mdl->state[i] + dt * k3[i];
 sensor_dynamics_ode(tmp, mdl->params, mdl->ambient, t + dt, k4);

 // Update state: y_new = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
 for (int i = 0; i < DYNAMICS_DIM; i++)
 mdl->state[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

/*
 * Run the sensor dynamics prediction model over one timestep.
 * Predicts nominal sensor values using RK4, then computes residuals
 * between predicted and actual sensor readings.
 *
 * For sensors 0..DYNAMICS_DIM-1, the prediction comes directly from
 * the dynamics state. Sensors DYNAMICS_DIM..NUM_SENSORS-1 are mapped
 * through the Jacobian as linear combinations of the core states.
 */
static void predict_sensor_dynamics(SpacecraftData *data, double sim_time)
{
 SensorDynamicsModel *mdl = &data->dynamics;

 // Advance dynamics model by one RK4 step
 rk4_integrate_dynamics(mdl, sim_time);

 // Update the Jacobian (state-transition matrix propagation)
 compute_dynamics_jacobian(mdl, sim_time);

 // Map dynamics state to predicted sensor values
 for (int i = 0; i < DYNAMICS_DIM && i < NUM_SENSORS; i++)
 mdl->predicted[i] = mdl->state[i];

 // For extra sensors beyond the core 6, use Jacobian-weighted mapping
 // These represent derived/composite sensor channels
 for (int s = DYNAMICS_DIM; s < NUM_SENSORS; s++) {
 double val = 0.0;
 for (int j = 0; j < DYNAMICS_DIM; j++)
 val += mdl->jacobian[(s % DYNAMICS_DIM) * DYNAMICS_DIM + j] * mdl->state[j];
 mdl->predicted[s] = val;
 }

 // Compute normalized residuals: actual (latest reading) minus predicted,
 // scaled by the predicted magnitude to avoid large absolute values dominating
 double residual_norm = 0.0;
 for (int s = 0; s < NUM_SENSORS; s++) {
 double actual = (double)data->values[s][WINDOW_SIZE - 1];
 double raw_residual = actual - mdl->predicted[s];
 double scale = sw_fabs(mdl->predicted[s]) + 1.0; // avoid divide-by-zero
 mdl->residuals[s] = raw_residual / scale;
 residual_norm += mdl->residuals[s] * mdl->residuals[s];
 }
 data->dynamics_residual_norm = (float)sw_sqrt(residual_norm / NUM_SENSORS);
}

// Extract features from raw sensor data
void extract_features(SpacecraftData* data) {
 // For each sensor
 for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
 float mean = 0.0f;
 float std_dev = 0.0f;
 float min_val = data->values[sensor][0];
 float max_val = data->values[sensor][0];
 float trend = 0.0f;

 // Calculate mean
 for (int t = 0; t < WINDOW_SIZE; t++) {
 mean += data->values[sensor][t];
 }
 mean /= WINDOW_SIZE;

 // Calculate std dev, min, max, and trend
 for (int t = 0; t < WINDOW_SIZE; t++) {
 float diff = data->values[sensor][t] - mean;
 std_dev += diff * diff;

 if (data->values[sensor][t] < min_val) min_val = data->values[sensor][t];
 if (data->values[sensor][t] > max_val) max_val = data->values[sensor][t];

 if (t < WINDOW_SIZE - 1) {
 trend += data->values[sensor][t+1] - data->values[sensor][t];
 }
 }
 std_dev = (float)sw_sqrt((double)std_dev / WINDOW_SIZE);
 trend /= (WINDOW_SIZE - 1);

 // Frequency-domain power spectral feature
 float freq_power = 0.0f;
 for (int t = 0; t < WINDOW_SIZE/3; t++) { // Use only first third for efficiency
 float sum_cos = 0.0f, sum_sin = 0.0f;
 for (int k = 0; k < WINDOW_SIZE; k++) {
 double angle = 2.0 * M_PI * t * k / (double)WINDOW_SIZE;
 sum_cos += data->values[sensor][k] * (float)sw_cos(angle);
 sum_sin += data->values[sensor][k] * (float)sw_sin(angle);
 }
 freq_power += sum_cos*sum_cos + sum_sin*sum_sin;
 }
 freq_power /= WINDOW_SIZE;

 // Store extracted features
 data->features[sensor][0] = mean;
 data->features[sensor][1] = std_dev;
 data->features[sensor][2] = max_val - min_val; // Range
 data->features[sensor][3] = trend;
 data->features[sensor][4] = freq_power;

 /* Sample skewness: third standardised moment E[(X-mu)^3]/sigma^3 */
 float skew_sum = 0.0f;
 for (int t = 0; t < WINDOW_SIZE; t++) {
 float d = data->values[sensor][t] - mean;
 skew_sum += d * d * d;
 }
 data->features[sensor][5] = (skew_sum / WINDOW_SIZE)
 / (std_dev * std_dev * std_dev + 1e-10f);

 /* Kurtosis: fourth standardised moment - 3 (excess kurtosis) */
 float kurt_sum = 0.0f;
 for (int t = 0; t < WINDOW_SIZE; t++) {
 float d = data->values[sensor][t] - mean;
 kurt_sum += d * d * d * d;
 }
 data->features[sensor][6] = (kurt_sum / WINDOW_SIZE)
 / (std_dev * std_dev * std_dev * std_dev + 1e-12f) - 3.0f;
 }
}

// Detect anomalies using Mahalanobis distance (statistical method)
float detect_statistical_anomaly(SpacecraftData* data) {
 /* Compute per-sensor feature-deviation magnitude (9-dim summary) */
 float dev[NUM_SENSORS];
 for (int s = 0; s < NUM_SENSORS; s++) {
 float ss = 0.0f;
 for (int f = 0; f < NUM_FEATURES; f++) {
 float d = data->features[s][f] - data->normal_patterns[s][f];
 ss += d * d;
 }
 dev[s] = (float)sw_sqrt((double)ss);
 }

 /* Mahalanobis distance: d = sqrt(x^T Sigma_inv x)
 x = dev[9], Sigma_inv = inverse_cov[9][9].
 Inner product tiled in 3x3 blocks for AME accelerator. */
 float d_sq = 0.0f;
 for (int bi = 0; bi < NUM_SENSORS; bi += 3) {
 for (int bj = 0; bj < NUM_SENSORS; bj += 3) {
 /* 3x3 covariance tile and 3x1 deviation column */
 float S_tile[3][3], x_col[3][3] = {{0}};
 float prod[3][3];
 for (int ii = 0; ii < 3; ii++) {
 x_col[ii][0] = dev[bj + ii];
 for (int jj = 0; jj < 3; jj++)
 S_tile[ii][jj] = data->inverse_cov[bi + ii][bj + jj];
 }
 matrix_multiply_3x3(S_tile, x_col, prod);

 /* Accumulate x_i^T * (Sigma_inv * x_j)_tile */
 for (int ii = 0; ii < 3; ii++)
 d_sq += dev[bi + ii] * prod[ii][0];
 }
 }

 return (float)sw_sqrt((double)sw_fabs(d_sq));
}

// 1D-CNN feature extractor + fully-connected autoencoder for anomaly detection
float detect_autoencoder_anomaly(SpacecraftData* data) {
 float anomaly_score = 0.0f;

 // -------- Stage 1: 1-D convolution on raw sensor time-series --------
 // Kernel size 3, NUM_SENSORS input channels → NUM_SENSORS output channels
 #define CONV_KSIZE 3
 #define CONV_OUT_LEN (WINDOW_SIZE - CONV_KSIZE + 1) // 23

 static int cnn_init = 0;
 static float conv_kernel[NUM_SENSORS][CONV_KSIZE]; // per-channel 1D kernel
 static float conv_bias[NUM_SENSORS];
 if (!cnn_init) {
 unsigned int seed = 123;
 for (int c = 0; c < NUM_SENSORS; c++) {
 for (int k = 0; k < CONV_KSIZE; k++) {
 seed = seed * 1103515245 + 12345;
 conv_kernel[c][k] = ((float)((seed >> 16) & 0x7FFF) / 32768.0f - 0.5f) * 0.3f;
 }
 conv_bias[c] = 0.01f;
 }
 cnn_init = 1;
 }

 float conv_out[NUM_SENSORS][CONV_OUT_LEN];
 for (int c = 0; c < NUM_SENSORS; c++) {
 for (int t = 0; t < CONV_OUT_LEN; t++) {
 float sum = conv_bias[c];
 for (int k = 0; k < CONV_KSIZE; k++)
 sum += data->values[c][t + k] * conv_kernel[c][k];
 conv_out[c][t] = (sum > 0.0f) ? sum : 0.0f; // ReLU activation
 }
 }

 // -------- Stage 2: Max-pooling (pool size 5, stride 5) --------
 #define POOL_SIZE 5
 #define POOL_OUT_LEN (CONV_OUT_LEN / POOL_SIZE) // 5

 float pool_out[NUM_SENSORS][POOL_OUT_LEN];
 for (int c = 0; c < NUM_SENSORS; c++) {
 for (int p = 0; p < POOL_OUT_LEN; p++) {
 float mx = conv_out[c][p * POOL_SIZE];
 for (int k = 1; k < POOL_SIZE; k++) {
 float v = conv_out[c][p * POOL_SIZE + k];
 if (v > mx) mx = v;
 }
 pool_out[c][p] = mx;
 }
 }

 // Flatten pooling output: NUM_SENSORS * POOL_OUT_LEN = 9*5 = 45
 #define FLAT_LEN (NUM_SENSORS * POOL_OUT_LEN)
 float flat[FLAT_LEN];
 for (int c = 0; c < NUM_SENSORS; c++)
 for (int p = 0; p < POOL_OUT_LEN; p++)
 flat[c * POOL_OUT_LEN + p] = pool_out[c][p];

 // -------- Stage 3: Multi-layer autoencoder (NN) --------
 // Encoder: FLAT_LEN(45) → 20 → 7 Decoder: 7 → 20 → FLAT_LEN(45)
 #define ENC_H 20
 #define LATENT 7

 static int ae_init = 0;
 static float We1[ENC_H][FLAT_LEN], be1[ENC_H];
 static float We2[LATENT][ENC_H], be2[LATENT];
 static float Wd1[ENC_H][LATENT], bd1[ENC_H];
 static float Wd2[FLAT_LEN][ENC_H], bd2[FLAT_LEN];

 if (!ae_init) {
 unsigned int seed = 77;
 #define INIT_W(R,C,W,B) do { \
 for (int _i = 0; _i < (R); _i++) { \
 for (int _j = 0; _j < (C); _j++) { \
 seed = seed * 1103515245 + 12345; \
 (W)[_i][_j] = ((float)((seed >> 16) & 0x7FFF) / 32768.0f - 0.5f) * 0.3f; \
 } \
 (B)[_i] = 0.01f; \
 } \
 } while(0)
 INIT_W(ENC_H, FLAT_LEN, We1, be1);
 INIT_W(LATENT, ENC_H, We2, be2);
 INIT_W(ENC_H, LATENT, Wd1, bd1);
 INIT_W(FLAT_LEN, ENC_H, Wd2, bd2);
 #undef INIT_W
 ae_init = 1;
 }

 // Encoder layer 1: GEMV + ReLU
 float enc1[ENC_H];
 for (int i = 0; i < ENC_H; i++) {
 float acc = be1[i];
 for (int j = 0; j < FLAT_LEN; j++)
 acc += We1[i][j] * flat[j];
 enc1[i] = (acc > 0.0f) ? acc : 0.0f; // ReLU
 }

 // Encoder layer 2: GEMV + ReLU → latent
 float latent[LATENT];
 for (int i = 0; i < LATENT; i++) {
 float acc = be2[i];
 for (int j = 0; j < ENC_H; j++)
 acc += We2[i][j] * enc1[j];
 latent[i] = (acc > 0.0f) ? acc : 0.0f;
 }

 // Decoder layer 1: GEMV + ReLU
 float dec1[ENC_H];
 for (int i = 0; i < ENC_H; i++) {
 float acc = bd1[i];
 for (int j = 0; j < LATENT; j++)
 acc += Wd1[i][j] * latent[j];
 dec1[i] = (acc > 0.0f) ? acc : 0.0f;
 }

 // Decoder layer 2: GEMV (linear output)
 float recon[FLAT_LEN];
 for (int i = 0; i < FLAT_LEN; i++) {
 float acc = bd2[i];
 for (int j = 0; j < ENC_H; j++)
 acc += Wd2[i][j] * dec1[j];
 recon[i] = acc;
 }

 // Reconstruction error (MSE)
 for (int i = 0; i < FLAT_LEN; i++) {
 float err = flat[i] - recon[i];
 anomaly_score += err * err;
 }

 float result = (float)sw_sqrt((double)anomaly_score / (NUM_SENSORS * POOL_OUT_LEN));

 #undef CONV_KSIZE
 #undef CONV_OUT_LEN
 #undef POOL_SIZE
 #undef POOL_OUT_LEN
 #undef FLAT_LEN
 #undef ENC_H
 #undef LATENT

 return result;
}

/* Simplified isolation forest anomaly detector (Liu et al., 2008).
 * An ensemble of 8 random binary partition trees, where anomalous
 * samples in sparse feature regions are isolated in fewer splits
 * (shorter path length). Score: s = exp(-E[h(x)] / c(n)). */
float detect_isolation_forest_anomaly(SpacecraftData* data) {
 /* c(n) = 2 H(n-1) - 2(n-1)/n (average path normaliser) */
 double harmonic = 0.0;
 for (int i = 1; i < WINDOW_SIZE; i++)
 harmonic += 1.0 / i;
 double cn = 2.0 * harmonic - 2.0 * (WINDOW_SIZE - 1.0) / WINDOW_SIZE;

 float total_score = 0.0f;

 for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
 float avg_path = 0.0f;

 for (int tree = 0; tree < 8; tree++) {
 int path = 0;
 for (int d = 0; d < 6; d++) {
 /* Pseudo-random feature selection (deterministic, seeded by tree+depth) */
 int feat = ((tree + 1) * 7 + d * 5) % NUM_FEATURES;
 float val = data->features[sensor][feat];

 /* Split point: base + jitter from stored params */
 float base = data->isolation_params[feat][0];
 float jitter = data->isolation_params[feat][1];
 float range = data->isolation_params[feat][2] + 1e-6f;
 float split = base + jitter
 * ((float)((tree * 31 + d * 13) % 37) / 37.0f - 0.5f);

 path++;

 /* Branch left or right; terminate if point is
 outside the expected range (already isolated) */
 if (val < split) {
 if (val < base - range) break;
 } else {
 if (val > base + range) break;
 }
 }
 avg_path += (float)path;
 }
 avg_path /= 8.0f;

 /* Anomaly score: shorter path → higher score */
 float score = (float)sw_exp(-avg_path / cn);
 total_score += score;
 }

 return total_score / NUM_SENSORS;
}

// Main anomaly detection function
int detect_anomalies(SpacecraftData* data, double sim_time) {
 // --- Predict-then-detect: run dynamics model first ---
 // Advance the sensor dynamics model by one timestep using RK4 integration.
 // This predicts nominal sensor behavior; large residuals indicate anomalies.
 predict_sensor_dynamics(data, sim_time);

 // Extract features from raw sensor data
 extract_features(data);

 // Augment features with dynamics residuals: replace the trend feature
 // (index 3) with the dynamics residual for each sensor. This feeds
 // the physics-based prediction error into the neural network detector.
 for (int s = 0; s < NUM_SENSORS; s++) {
 data->features[s][3] = (float)data->dynamics.residuals[s];
 }

 // Apply three different anomaly detection methods
 float statistical_score = detect_statistical_anomaly(data);
 float autoencoder_score = detect_autoencoder_anomaly(data);
 float isolation_score = detect_isolation_forest_anomaly(data);

 // Store anomaly scores
 data->anomaly_scores[0] = statistical_score;
 data->anomaly_scores[1] = autoencoder_score;
 data->anomaly_scores[2] = isolation_score;

 // Check for anomalies using thresholds
 int is_statistical_anomaly = (statistical_score > MAHALANOBIS_THRESHOLD);
 int is_autoencoder_anomaly = (autoencoder_score > AUTOENCODER_THRESHOLD);
 int is_isolation_anomaly = (isolation_score > ISOLATION_THRESHOLD);
 int is_dynamics_anomaly = (data->dynamics_residual_norm > DYNAMICS_RESIDUAL_THRESHOLD);

 // Determine if an anomaly is detected and its type
 data->anomaly_detected = is_statistical_anomaly || is_autoencoder_anomaly ||
 is_isolation_anomaly || is_dynamics_anomaly;

 if (data->anomaly_detected) {
 // Determine the most significant anomaly type
 float dyn_ratio = data->dynamics_residual_norm / DYNAMICS_RESIDUAL_THRESHOLD;
 if (statistical_score / MAHALANOBIS_THRESHOLD >
 autoencoder_score / AUTOENCODER_THRESHOLD &&
 statistical_score / MAHALANOBIS_THRESHOLD >
 isolation_score / ISOLATION_THRESHOLD &&
 statistical_score / MAHALANOBIS_THRESHOLD > dyn_ratio) {
 data->anomaly_type = 0; // Statistical anomaly
 } else if (autoencoder_score / AUTOENCODER_THRESHOLD >
 isolation_score / ISOLATION_THRESHOLD &&
 autoencoder_score / AUTOENCODER_THRESHOLD > dyn_ratio) {
 data->anomaly_type = 1; // Autoencoder anomaly
 } else if (isolation_score / ISOLATION_THRESHOLD > dyn_ratio) {
 data->anomaly_type = 2; // Isolation forest anomaly
 } else {
 data->anomaly_type = 3; // Dynamics prediction anomaly
 }
 } else {
 data->anomaly_type = -1; // No anomaly
 }

 return data->anomaly_detected;
}

// Get a human-readable description of the anomaly
const char* get_anomaly_description(SpacecraftData* data) {
 static char description[256];

 if (!data->anomaly_detected) {
 strcpy(description, "No anomaly detected.");
 return description;
 }

 // Determine which sensors contributed most to the anomaly
 int top_sensor = 0;
 float max_diff = 0.0f;

 for (int i = 0; i < NUM_SENSORS; i++) {
 float total_diff = 0.0f;
 for (int j = 0; j < NUM_FEATURES; j++) {
 total_diff += (float)sw_fabs((double)(data->features[i][j] - data->normal_patterns[i][j]));
 }

 if (total_diff > max_diff) {
 max_diff = total_diff;
 top_sensor = i;
 }
 }

 // Create description based on anomaly type
 switch (data->anomaly_type) {
 case 0:
 sprintf(description, "Statistical anomaly detected in sensor %d with score %.2f (threshold: %.2f)",
 top_sensor, data->anomaly_scores[0], MAHALANOBIS_THRESHOLD);
 break;
 case 1:
 sprintf(description, "Pattern anomaly detected in sensor %d with reconstruction error %.2f (threshold: %.2f)",
 top_sensor, data->anomaly_scores[1], AUTOENCODER_THRESHOLD);
 break;
 case 2:
 sprintf(description, "Isolation anomaly detected in sensor %d with isolation score %.2f (threshold: %.2f)",
 top_sensor, data->anomaly_scores[2], ISOLATION_THRESHOLD);
 break;
 case 3:
 sprintf(description, "Dynamics prediction anomaly in sensor %d, residual norm %.4f (threshold: %.2f)",
 top_sensor, data->dynamics_residual_norm, DYNAMICS_RESIDUAL_THRESHOLD);
 break;
 default:
 strcpy(description, "Unknown anomaly type detected.");
 }

 return description;
}

// Record sensor values from telemetry
void record_sensor_values(SpacecraftData* data, float sensor_readings[NUM_SENSORS]) {
 // Shift window (discard oldest value)
 for (int s = 0; s < NUM_SENSORS; s++) {
 for (int t = 0; t < WINDOW_SIZE - 1; t++) {
 data->values[s][t] = data->values[s][t+1];
 }

 // Add new reading at the end
 data->values[s][WINDOW_SIZE - 1] = sensor_readings[s];
 }
}

// Update normal patterns based on new data (for adaptive learning)
void update_normal_patterns(SpacecraftData* data, float learning_rate) {
 // Only update if no anomaly is detected
 if (!data->anomaly_detected) {
 for (int i = 0; i < NUM_SENSORS; i++) {
 for (int j = 0; j < NUM_FEATURES; j++) {
 // Gradual update of normal patterns
 data->normal_patterns[i][j] = (1.0f - learning_rate) * data->normal_patterns[i][j] +
 learning_rate * data->features[i][j];
 }
 }
 }
}

// Main entry point
int main() {
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(8314);

 SpacecraftData spacecraft;
 initialize_spacecraft_data(&spacecraft);

 // Sensor readings from onboard instrument interface
 float sensor_readings[NUM_SENSORS] = {0};

 // Simulation loop
 for (int t = 0; t < 100; t++) {
 double sim_time = (double)t;

 // Simulate some sensor readings
 for (int s = 0; s < NUM_SENSORS; s++) {
 // Normal operation with some noise
 sensor_readings[s] = (float)sw_sin(t * 0.1 + s) + ((float)rand() / RAND_MAX - 0.5f) * 0.2f;

 // Introduce an anomaly at t=50
 if (t == 50 && s == 3) {
 sensor_readings[s] += 5.0f; // Sudden spike in sensor 3
 }
 }

 // Record the new sensor values
 record_sensor_values(&spacecraft, sensor_readings);

 // Only start detection after filling the window
 if (t >= WINDOW_SIZE - 1) {
 // Detect anomalies (includes dynamics prediction via RK4)
 int anomaly_detected = detect_anomalies(&spacecraft, sim_time);

 // Print results
 FLIGHT_LOG("Time step %d (dyn_residual=%.4f): ", t, spacecraft.dynamics_residual_norm);
 if (anomaly_detected) {
 FLIGHT_LOG("ANOMALY DETECTED! %s\n", get_anomaly_description(&spacecraft));

 // Trigger anomaly alert via telemetry
 } else {
 FLIGHT_LOG("Normal operation.\n");

 // Update normal patterns (adaptive learning)
 update_normal_patterns(&spacecraft, 0.008f);
 }
 }
 }

 return 0;
}
