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
 * Space Robotic Arm Control System
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements the real-time control system for a multi-DOF space robotic arm.
 * This workload performs forward/inverse kinematics using Jacobian matrices,
 * trajectory generation with quintic polynomial interpolation, numerical integration
 * for dynamics simulation, and computed torque control. Applicable to the planned
 * space station robotic arm and the lunar surface sample
 * collection mechanism.
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

/* ---- Fixed-memory telemetry system (flight-style) ---- */
typedef enum { TM_INFO = 0, TM_STEP, TM_RSLT, TM_WARN } TmLevel;
static const char *const TM_TAG[] = {"INFO", "STEP", "RSLT", "WARN"};
#define tm_log(lvl, fmt, ...) \
 FLIGHT_LOG("[ARM][%s] " fmt "\n", TM_TAG[(lvl)], ##__VA_ARGS__)

/* Constants and Definitions */
// Hardware specifications
#define NUM_JOINTS 7 // 7-DOF robotic arm
#define NUM_SENSORS 28 // 4 sensors per joint
#define MATRIX_SIZE 18 // Padded for 3x3 tiled AME processing of arm dynamics
#define SMALL_MATRIX_SIZE 6 // For Jacobian

// Control parameters
#define CONTROL_RATE_HZ 100 // 100 Hz control loop
#define TELEMETRY_RATE_HZ 10 // 10 Hz telemetry
#define TIME_STEP (1.0 / CONTROL_RATE_HZ)

// Arm parameters (based on real Canadarm2 specs)
#define SPDM_LENGTH 3.4 // Special Purpose Dexterous Manipulator length (m)
#define BOOM_LENGTH 17.6 // Main boom length (m)
#define ARM_MASS 1800.0 // Arm mass (kg)
#define MAX_PAYLOAD_MASS 116000.0 // Maximum payload (kg)

// Environment parameters
#define NUM_OBSTACLES 5 // Number of obstacles in environment
#define WORKSPACE_SIZE 20.0 // Workspace size (m)
#define MANIP_IDX_THRESHOLD 1e-4 // Manipulability index lockout threshold

// Task scenarios
#define NUM_SCENARIOS 3
#define SCENARIO_CAPTURE 0 // Satellite capture
#define SCENARIO_INSPECTION 1 // Station inspection
#define SCENARIO_CONSTRUCTION 2 // Assembly task

/* Data structures */
typedef struct {
 double data[MATRIX_SIZE][MATRIX_SIZE];
} Matrix;

typedef struct {
 double data[SMALL_MATRIX_SIZE][SMALL_MATRIX_SIZE];
} SmallMatrix;

typedef struct {
 double data[SMALL_MATRIX_SIZE][NUM_JOINTS];
} JacobianMatrix;

typedef struct {
 double x, y, z;
} Vector3D;

typedef struct {
 Vector3D position;
 double quaternion[4]; // [w, x, y, z] format
} Pose;

typedef struct {
 Vector3D position; // Position (m)
 Vector3D velocity; // Linear velocity (m/s)
 Vector3D acceleration; // Linear acceleration (m/s^2)
 double quaternion[4]; // Orientation as quaternion [w, x, y, z]
 Vector3D angular_vel; // Angular velocity (rad/s)
 Vector3D angular_acc; // Angular acceleration (rad/s^2)
} BodyState;

typedef struct {
 double position; // Joint position (rad)
 double velocity; // Joint velocity (rad/s)
 double acceleration; // Joint acceleration (rad/s^2)
 double torque; // Joint torque (N·m)
 double motor_current; // Motor current (A)
 double temperature; // Joint temperature (°C)
} JointState;

typedef struct {
 double raw_values[NUM_SENSORS]; // Raw sensor readings
 double filtered_values[NUM_SENSORS]; // Filtered sensor readings
 double timestamps[NUM_SENSORS]; // Timestamp for each reading
 bool valid_flags[NUM_SENSORS]; // Validity flags
 double noise_covariance[NUM_SENSORS]; // Sensor noise covariance
} SensorData;

typedef struct {
 Vector3D position;
 double radius;
} Obstacle;

typedef struct {
 Vector3D position; // Position (m)
 double radius; // Bounding radius (m)
 double mass; // Mass (kg)
 Vector3D com; // Center of mass offset (m)
 Matrix inertia; // Inertia tensor (kg·m^2)
} ArmSegment;

typedef struct {
 // Physical state
 BodyState base_state; // State of the arm base (station attachment point)
 ArmSegment segments[NUM_JOINTS]; // Arm segments
 JointState joint_states[NUM_JOINTS]; // Current joint states
 Pose end_effector; // End effector pose
 double payload_mass; // Current payload mass (kg)
 
 // Sensors and control
 SensorData sensors; // Sensor readings
 double control_gains[NUM_JOINTS][3]; // PID gains for each joint
 double joint_limits[NUM_JOINTS][2]; // Min/max joint limits
 double joint_velocity_limits[NUM_JOINTS]; // Max velocity for each joint
 double joint_torque_limits[NUM_JOINTS]; // Max torque for each joint
 
 // DH parameters for kinematics
 double dh_params[NUM_JOINTS][4]; // [theta, d, a, alpha]
 
 // Control signals
 double cmd_joint_pos[NUM_JOINTS]; // Commanded joint positions
 double cmd_joint_vel[NUM_JOINTS]; // Commanded joint velocities
 double cmd_joint_acc[NUM_JOINTS]; // Commanded joint accelerations
 double cmd_joint_torque[NUM_JOINTS]; // Commanded joint torques
 
 // Path planning
 Pose target_pose; // Target end effector pose
 Pose trajectory[100]; // Planned trajectory
 int trajectory_length; // Number of points in trajectory
 int trajectory_index; // Current position in trajectory
 
 // Environment
 Obstacle obstacles[NUM_OBSTACLES]; // Obstacles in the environment
} RoboticArm;

/* Function prototypes */
// Initialization
void initializeArm(RoboticArm *arm);
void initializeMatrix(Matrix *m, double val);
void initializeObstacles(RoboticArm *arm);
void setArmScenario(RoboticArm *arm, int scenario);

// Sensor processing
void acquireSensorData(RoboticArm *arm);
void filterSensorData(RoboticArm *arm);
void detectSensorFaults(RoboticArm *arm);

// Kinematics
void updateForwardKinematics(RoboticArm *arm);
void calculateJacobian(RoboticArm *arm, JacobianMatrix *jacobian);
void solveInverseKinematics(RoboticArm *arm, Pose *target_pose);

// Path planning
void generateTrajectory(RoboticArm *arm, Pose *start, Pose *end, int steps);
bool checkCollision(RoboticArm *arm, Pose *pose);
void planPath(RoboticArm *arm, Pose *target);
void optimizeTrajectory(RoboticArm *arm);

// Dynamics and control
void calculateDynamicsModel(RoboticArm *arm, Matrix *mass_matrix, double *coriolis, double *gravity);
void compensateNonlinearDynamics(RoboticArm *arm);
void computeControlLaw(RoboticArm *arm);
void applyJointLimits(RoboticArm *arm);

// Math utilities
Matrix multiplyMatrices(Matrix *a, Matrix *b);
Matrix inverseMatrix(Matrix *m);
void quaternionMultiply(double *result, double *q1, double *q2);
void quaternionFromEuler(double *quaternion, double roll, double pitch, double yaw);
double quaternionNorm(double *q);
void quaternionNormalize(double *q);
void quaternionConjugate(double *result, double *q);

// Manipulability
double compute_manipulability_index(JacobianMatrix *jacobian);

// Simulation control
void runSimulation(RoboticArm *arm, double duration);
void updateArmState(RoboticArm *arm, double dt);
void displayTelemetry(RoboticArm *arm, double time);

/**
 * Main function - initializes the system and runs simulation
 */
int main() {
 tm_log(TM_INFO, "init");
 
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (robotics kinematics/dynamics, matrix ops) as real
 mission telemetry. Fixed seed ensures reproducible execution traces for
 gem5 HW/SW comparison. */
 srand(7129);
 
 // Initialize the robotic arm system
 RoboticArm arm;
 memset(&arm, 0, sizeof(RoboticArm)); // Ensure clean initialization
 
 initializeArm(&arm);
 initializeObstacles(&arm);
 
 tm_log(TM_INFO, "ready");
 tm_log(TM_INFO, "%d DOF arm with %d sensors", NUM_JOINTS, NUM_SENSORS);
 tm_log(TM_INFO, "control_rate=%d Hz", CONTROL_RATE_HZ);
 tm_log(TM_INFO, "matrix_dim=%dx%d", MATRIX_SIZE, MATRIX_SIZE);
 
 // Run full simulation
 double sim_time = 2.0;
 tm_log(TM_INFO, "t_sim=%.1fs", sim_time);
 runSimulation(&arm, sim_time);
 
 tm_log(TM_RSLT, "done");
 return 0;
}

/**
 * Initialize the robotic arm with realistic parameters
 */
void initializeArm(RoboticArm *arm) {
 // Initialize base state (attached to space station)
 arm->base_state.position = (Vector3D){0.0, 0.0, 0.0};
 arm->base_state.velocity = (Vector3D){0.0, 0.0, 0.0};
 arm->base_state.acceleration = (Vector3D){0.0, 0.0, 0.0};
 arm->base_state.quaternion[0] = 1.0; // w
 arm->base_state.quaternion[1] = 0.0; // x
 arm->base_state.quaternion[2] = 0.0; // y
 arm->base_state.quaternion[3] = 0.0; // z
 arm->base_state.angular_vel = (Vector3D){0.0, 0.0, 0.0};
 arm->base_state.angular_acc = (Vector3D){0.0, 0.0, 0.0};
 
 // Initialize arm segments (based on Canadarm2 specs)
 // Segment properties are approximated from real-world values
 double segment_lengths[NUM_JOINTS] = {
 1.2, 2.5, SPDM_LENGTH, 2.8, BOOM_LENGTH/2, BOOM_LENGTH/2, 1.5
 };
 
 double segment_masses[NUM_JOINTS] = {
 180.0, 220.0, 250.0, 300.0, 350.0, 300.0, 200.0
 };
 
 for (int i = 0; i < NUM_JOINTS; i++) {
 arm->segments[i].position = (Vector3D){0.0, 0.0, 0.0}; // Will be updated by FK
 arm->segments[i].radius = 0.5; // 0.5m radius for collision detection
 arm->segments[i].mass = segment_masses[i];
 arm->segments[i].com = (Vector3D){segment_lengths[i]/2, 0.0, 0.0}; // COM at middle of segment
 
 // Initialize inertia tensor (solid cylinder link model)
 // Axial: Ixx = m*r²/2, Perpendicular: Iyy = Izz = m*(3r²+L²)/12
 initializeMatrix(&arm->segments[i].inertia, 0.0);
 double r2 = arm->segments[i].radius * arm->segments[i].radius;
 double L2 = segment_lengths[i] * segment_lengths[i];
 double Ixx = arm->segments[i].mass * r2 / 2.0;
 double Iyy = arm->segments[i].mass * (3.0 * r2 + L2) / 12.0;
 double Izz = Iyy;
 
 arm->segments[i].inertia.data[0][0] = Ixx;
 arm->segments[i].inertia.data[1][1] = Iyy;
 arm->segments[i].inertia.data[2][2] = Izz;
 }
 
 // Initialize joint states
 for (int i = 0; i < NUM_JOINTS; i++) {
 arm->joint_states[i].position = 0.0;
 arm->joint_states[i].velocity = 0.0;
 arm->joint_states[i].acceleration = 0.0;
 arm->joint_states[i].torque = 0.0;
 arm->joint_states[i].motor_current = 0.0;
 arm->joint_states[i].temperature = 20.0; // 20°C ambient
 
 // Initialize control commands
 arm->cmd_joint_pos[i] = 0.0;
 arm->cmd_joint_vel[i] = 0.0;
 arm->cmd_joint_acc[i] = 0.0;
 arm->cmd_joint_torque[i] = 0.0;
 }
 
 // DH parameters for SSRMS (Canadarm2) - [theta, d, a, alpha]
 // These values are approximated for simulation purposes
 arm->dh_params[0][0] = 0.0; // Variable (joint angle)
 arm->dh_params[0][1] = 0.0; // Link offset
 arm->dh_params[0][2] = 0.0; // Link length
 arm->dh_params[0][3] = M_PI/2; // Link twist
 
 arm->dh_params[1][0] = 0.0; // Variable (joint angle)
 arm->dh_params[1][1] = 0.0; // Link offset
 arm->dh_params[1][2] = segment_lengths[0]; // Link length
 arm->dh_params[1][3] = -M_PI/2; // Link twist
 
 arm->dh_params[2][0] = 0.0; // Variable (joint angle)
 arm->dh_params[2][1] = 0.0; // Link offset
 arm->dh_params[2][2] = segment_lengths[1]; // Link length
 arm->dh_params[2][3] = M_PI/2; // Link twist
 
 arm->dh_params[3][0] = 0.0; // Variable (joint angle)
 arm->dh_params[3][1] = 0.0; // Link offset
 arm->dh_params[3][2] = segment_lengths[2]; // Link length
 arm->dh_params[3][3] = -M_PI/2; // Link twist
 
 arm->dh_params[4][0] = 0.0; // Variable (joint angle)
 arm->dh_params[4][1] = 0.0; // Link offset
 arm->dh_params[4][2] = segment_lengths[3]; // Link length
 arm->dh_params[4][3] = M_PI/2; // Link twist
 
 arm->dh_params[5][0] = 0.0; // Variable (joint angle)
 arm->dh_params[5][1] = 0.0; // Link offset
 arm->dh_params[5][2] = segment_lengths[4]; // Link length
 arm->dh_params[5][3] = -M_PI/2; // Link twist
 
 arm->dh_params[6][0] = 0.0; // Variable (joint angle)
 arm->dh_params[6][1] = 0.0; // Link offset
 arm->dh_params[6][2] = segment_lengths[5]; // Link length
 arm->dh_params[6][3] = 0.0; // Link twist
 
 // Set per-joint limits (in radians) reflecting different joint types
 // Joint 0 (shoulder yaw): ±270°
 arm->joint_limits[0][0] = -270.0 * M_PI / 180.0;
 arm->joint_limits[0][1] = 270.0 * M_PI / 180.0;
 // Joint 1 (shoulder pitch): ±260°
 arm->joint_limits[1][0] = -260.0 * M_PI / 180.0;
 arm->joint_limits[1][1] = 260.0 * M_PI / 180.0;
 // Joint 2 (elbow): ±260°
 arm->joint_limits[2][0] = -260.0 * M_PI / 180.0;
 arm->joint_limits[2][1] = 260.0 * M_PI / 180.0;
 // Joint 3 (elbow roll): ±540°
 arm->joint_limits[3][0] = -540.0 * M_PI / 180.0;
 arm->joint_limits[3][1] = 540.0 * M_PI / 180.0;
 // Joint 4 (wrist pitch): ±260°
 arm->joint_limits[4][0] = -260.0 * M_PI / 180.0;
 arm->joint_limits[4][1] = 260.0 * M_PI / 180.0;
 // Joint 5 (wrist yaw): ±270°
 arm->joint_limits[5][0] = -270.0 * M_PI / 180.0;
 arm->joint_limits[5][1] = 270.0 * M_PI / 180.0;
 // Joint 6 (wrist roll): ±540°
 arm->joint_limits[6][0] = -540.0 * M_PI / 180.0;
 arm->joint_limits[6][1] = 540.0 * M_PI / 180.0;
 for (int i = 0; i < NUM_JOINTS; i++) {
 arm->joint_velocity_limits[i] = 0.1; // Max 0.1 rad/s (~6 deg/s)
 arm->joint_torque_limits[i] = 100.0; // Max 100 N·m (realistic spacecraft arm)
 }
 
 // Set per-joint PID gains reflecting different inertias
 // Gains are in Nm/rad (P), Nm/(rad·s) (I), Nm·s/rad (D) — sized so
 // that typical position errors (~0.5 rad) produce torques well within
 // the 100 Nm joint limit.
 // Joint 0 (shoulder yaw): high inertia
 arm->control_gains[0][0] = 25.0; arm->control_gains[0][1] = 5.0; arm->control_gains[0][2] = 8.0;
 // Joint 1 (shoulder pitch)
 arm->control_gains[1][0] = 22.0; arm->control_gains[1][1] = 4.5; arm->control_gains[1][2] = 7.0;
 // Joint 2 (elbow)
 arm->control_gains[2][0] = 18.0; arm->control_gains[2][1] = 4.0; arm->control_gains[2][2] = 6.0;
 // Joint 3 (elbow roll)
 arm->control_gains[3][0] = 15.0; arm->control_gains[3][1] = 3.5; arm->control_gains[3][2] = 5.0;
 // Joint 4 (wrist pitch)
 arm->control_gains[4][0] = 12.0; arm->control_gains[4][1] = 3.0; arm->control_gains[4][2] = 4.0;
 // Joint 5 (wrist yaw)
 arm->control_gains[5][0] = 10.0; arm->control_gains[5][1] = 2.5; arm->control_gains[5][2] = 3.0;
 // Joint 6 (wrist roll): low inertia
 arm->control_gains[6][0] = 8.0; arm->control_gains[6][1] = 2.0; arm->control_gains[6][2] = 2.0;
 
 // Set initial end effector pose to default
 arm->end_effector.position = (Vector3D){0.0, 0.0, 0.0};
 arm->end_effector.quaternion[0] = 1.0; // w
 arm->end_effector.quaternion[1] = 0.0; // x
 arm->end_effector.quaternion[2] = 0.0; // y
 arm->end_effector.quaternion[3] = 0.0; // z
 
 // Init sensor data
 for (int i = 0; i < NUM_SENSORS; i++) {
 arm->sensors.raw_values[i] = 0.0;
 arm->sensors.filtered_values[i] = 0.0;
 arm->sensors.timestamps[i] = 0.0;
 arm->sensors.valid_flags[i] = true;
 arm->sensors.noise_covariance[i] = 0.001; // 0.001 rad^2 variance
 }
 
 // No payload initially
 arm->payload_mass = 0.0;
 
 // Update forward kinematics to set initial state
 updateForwardKinematics(arm);
}

/**
 * Initialize obstacles in the environment
 */
void initializeObstacles(RoboticArm *arm) {
 // Create some obstacles in the workspace
 for (int i = 0; i < NUM_OBSTACLES; i++) {
 arm->obstacles[i].position.x = (double)rand() / RAND_MAX * WORKSPACE_SIZE - WORKSPACE_SIZE/2;
 arm->obstacles[i].position.y = (double)rand() / RAND_MAX * WORKSPACE_SIZE - WORKSPACE_SIZE/2;
 arm->obstacles[i].position.z = (double)rand() / RAND_MAX * WORKSPACE_SIZE - WORKSPACE_SIZE/2;
 arm->obstacles[i].radius = 0.5 + (double)rand() / RAND_MAX * 1.5; // 0.5m to 2.0m radius
 }
}

/**
 * Setup a specific operational scenario
 */
void setArmScenario(RoboticArm *arm, int scenario) {
 switch (scenario) {
 case SCENARIO_CAPTURE:
 // Satellite capture scenario
 tm_log(TM_INFO, "scenario=CAPTURE");
 // Set target as a satellite position
 arm->target_pose.position = (Vector3D){10.0, 5.0, 3.0};
 quaternionFromEuler(arm->target_pose.quaternion, 0.2, 0.1, 0.3);
 arm->payload_mass = 8000.0; // 8000kg satellite
 break;
 
 case SCENARIO_INSPECTION:
 // Station inspection scenario
 tm_log(TM_INFO, "scenario=INSPECTION");
 // Set target as inspection point
 arm->target_pose.position = (Vector3D){-8.0, 2.0, 7.0};
 quaternionFromEuler(arm->target_pose.quaternion, -0.1, 0.4, -0.2);
 arm->payload_mass = 500.0; // 500kg inspection equipment
 break;
 
 case SCENARIO_CONSTRUCTION:
 // Construction/assembly scenario
 tm_log(TM_INFO, "scenario=CONSTRUCTION");
 // Set target as assembly location
 arm->target_pose.position = (Vector3D){6.0, -7.0, -4.0};
 quaternionFromEuler(arm->target_pose.quaternion, 0.5, -0.3, 0.1);
 arm->payload_mass = 2500.0; // 2500kg module
 break;
 
 default:
 tm_log(TM_WARN, "unknown scenario, using default");
 arm->target_pose.position = (Vector3D){5.0, 0.0, 0.0};
 quaternionFromEuler(arm->target_pose.quaternion, 0.0, 0.0, 0.0);
 arm->payload_mass = 0.0;
 }
 
 // Plan path to the target
 planPath(arm, &arm->target_pose);
}

/**
 * Initialize a matrix with a value
 */
void initializeMatrix(Matrix *m, double val) {
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 m->data[i][j] = val;
 }
 }
}

/**
 * Matrix multiplication - core computation intensive operation
 * This is a key function to replace with hardware acceleration
 */
Matrix multiplyMatrices(Matrix *a, Matrix *b) {
 Matrix result;
 initializeMatrix(&result, 0.0);
 
 // Traditional matrix multiplication algorithm
 for (int i = 0; i < MATRIX_SIZE; i++) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 for (int k = 0; k < MATRIX_SIZE; k++) {
 result.data[i][j] += a->data[i][k] * b->data[k][j];
 }
 }
 }
 
 return result;
}

/**
 * Matrix inverse (using Gauss-Jordan elimination)
 * This is another key function for hardware acceleration
 */
Matrix inverseMatrix(Matrix *m) {
 Matrix result;
 Matrix temp = *m; // Make a copy of the input matrix
 
 // Initialize result as identity matrix
 initializeMatrix(&result, 0.0);
 for (int i = 0; i < MATRIX_SIZE; i++) {
 result.data[i][i] = 1.0;
 }
 
 // Gauss-Jordan elimination
 for (int i = 0; i < MATRIX_SIZE; i++) {
 // Find pivot
 double pivot = temp.data[i][i];
 int pivotRow = i;
 
 // Look for a better pivot if current one is close to zero
 for (int j = i + 1; j < MATRIX_SIZE; j++) {
 if (fabs(temp.data[j][i]) > fabs(pivot)) {
 pivot = temp.data[j][i];
 pivotRow = j;
 }
 }
 
 // Swap rows if needed
 if (pivotRow != i) {
 for (int j = 0; j < MATRIX_SIZE; j++) {
 double tempVal = temp.data[i][j];
 temp.data[i][j] = temp.data[pivotRow][j];
 temp.data[pivotRow][j] = tempVal;
 
 tempVal = result.data[i][j];
 result.data[i][j] = result.data[pivotRow][j];
 result.data[pivotRow][j] = tempVal;
 }
 }
 
 // Scale pivot row
 pivot = temp.data[i][i];
 if (fabs(pivot) < 1e-10) {
 // Matrix is singular, return identity
 for (int r = 0; r < MATRIX_SIZE; r++) {
 for (int c = 0; c < MATRIX_SIZE; c++) {
 result.data[r][c] = (r == c) ? 1.0 : 0.0;
 }
 }
 return result;
 }
 
 for (int j = 0; j < MATRIX_SIZE; j++) {
 temp.data[i][j] /= pivot;
 result.data[i][j] /= pivot;
 }
 
 // Eliminate other rows
 for (int j = 0; j < MATRIX_SIZE; j++) {
 if (j != i) {
 double factor = temp.data[j][i];
 for (int k = 0; k < MATRIX_SIZE; k++) {
 temp.data[j][k] -= factor * temp.data[i][k];
 result.data[j][k] -= factor * result.data[i][k];
 }
 }
 }
 }
 
 return result;
}

/**
 * Quaternion operations
 */
void quaternionMultiply(double *result, double *q1, double *q2) {
 // Quaternion multiplication: result = q1 * q2
 // Using the formula for quaternion multiplication
 result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]; // w
 result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]; // x
 result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]; // y
 result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]; // z
}

double quaternionNorm(double *q) {
 return sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

void quaternionNormalize(double *q) {
 double norm = quaternionNorm(q);
 if (norm > 1e-10) {
 q[0] /= norm;
 q[1] /= norm;
 q[2] /= norm;
 q[3] /= norm;
 } else {
 // Default to identity quaternion if norm is too small
 q[0] = 1.0;
 q[1] = q[2] = q[3] = 0.0;
 }
}

void quaternionConjugate(double *result, double *q) {
 result[0] = q[0]; // w
 result[1] = -q[1]; // -x
 result[2] = -q[2]; // -y
 result[3] = -q[3]; // -z
}

void quaternionFromEuler(double *quaternion, double roll, double pitch, double yaw) {
 // Convert Euler angles to quaternion
 // Using the ZYX convention (yaw, pitch, roll)
 double cr = cos(roll * 0.5);
 double sr = sin(roll * 0.5);
 double cp = cos(pitch * 0.5);
 double sp = sin(pitch * 0.5);
 double cy = cos(yaw * 0.5);
 double sy = sin(yaw * 0.5);
 
 quaternion[0] = cr * cp * cy + sr * sp * sy; // w
 quaternion[1] = sr * cp * cy - cr * sp * sy; // x
 quaternion[2] = cr * sp * cy + sr * cp * sy; // y
 quaternion[3] = cr * cp * sy - sr * sp * cy; // z
 
 // Ensure the quaternion is normalized
 quaternionNormalize(quaternion);
}


/**
 * Acquire sensor data from all arm sensors
 */
void acquireSensorData(RoboticArm *arm) {
 // Read current joint positions from encoder interface
 // For simulation, we generate realistic sensor readings with noise
 
 // Reset all sensors to valid state at each acquisition cycle
 for (int i = 0; i < NUM_SENSORS; i++) {
 arm->sensors.valid_flags[i] = true;
 }
 
 // Use static variables to maintain consistent noise patterns
 static double noise_offsets[NUM_SENSORS] = {0};
 
 // Initialize noise offsets if not already done
 static int offsets_initialized = 0;
 if (!offsets_initialized) {
 for (int i = 0; i < NUM_SENSORS; i++) {
 noise_offsets[i] = ((double)rand() / RAND_MAX - 0.5) * 0.001; // Small persistent bias
 }
 offsets_initialized = 1;
 }
 
 // Joint position sensors (4 per joint: primary, secondary, tertiary, quaternary)
 for (int i = 0; i < NUM_JOINTS; i++) {
 double true_position = arm->joint_states[i].position;
 double true_velocity = arm->joint_states[i].velocity;
 double true_torque = arm->joint_states[i].torque;
 
 // Generate consistent timestamps
 double current_time = (double)clock() / CLOCKS_PER_SEC;
 
 // Primary position sensor - small consistent bias + small random noise
 arm->sensors.raw_values[i*4] = true_position + 
 noise_offsets[i*4] + ((double)rand() / RAND_MAX - 0.5) * 0.0005;
 arm->sensors.timestamps[i*4] = current_time;
 
 // Secondary position sensor - different small bias + small random noise
 arm->sensors.raw_values[i*4+1] = true_position + 
 noise_offsets[i*4+1] + ((double)rand() / RAND_MAX - 0.5) * 0.0005;
 arm->sensors.timestamps[i*4+1] = current_time;
 
 // Velocity sensor
 arm->sensors.raw_values[i*4+2] = true_velocity + 
 noise_offsets[i*4+2] + ((double)rand() / RAND_MAX - 0.5) * 0.001;
 arm->sensors.timestamps[i*4+2] = current_time;
 
 // Torque sensor
 arm->sensors.raw_values[i*4+3] = true_torque + 
 noise_offsets[i*4+3] + ((double)rand() / RAND_MAX - 0.5) * 0.5;
 arm->sensors.timestamps[i*4+3] = current_time;
 }
 
 // Much rarer sensor fault simulation (1 in 100,000 chance)
 if (rand() % 100000 == 0) {
 int sensor_idx = rand() % NUM_SENSORS;
 arm->sensors.valid_flags[sensor_idx] = false;
 tm_log(TM_WARN, "sensor_fault id=%d", sensor_idx);
 }
}

/**
 * Filter sensor data to reduce noise
 */
void filterSensorData(RoboticArm *arm) {
 // Apply Kalman filtering to sensor readings
 
 // For each sensor
 for (int i = 0; i < NUM_SENSORS; i++) {
 if (!arm->sensors.valid_flags[i]) {
 // Skip invalid sensors
 continue;
 }
 
 // Simple alpha-beta filter 
 // Simple state estimator (Kalman filter applied in extended variant)
 
 // Use static arrays to maintain state between calls
 static double prev_filtered[NUM_SENSORS] = {0};
 static double prev_velocity[NUM_SENSORS] = {0};
 static int filter_initialized = 0;
 
 // Initialize filter state if not already done
 if (!filter_initialized) {
 for (int j = 0; j < NUM_SENSORS; j++) {
 prev_filtered[j] = arm->sensors.raw_values[j];
 prev_velocity[j] = 0.0;
 }
 filter_initialized = 1;
 }
 
 // Filter parameters - adjust these for stability
 double alpha = 0.5; // Position filter gain (reduced from 0.7)
 double beta = 0.1; // Velocity filter gain (reduced from 0.3)
 
 // For joint position sensors, use stronger filtering
 if (i % 4 < 2) {
 alpha = 0.3; // Even smoother filtering for position sensors
 beta = 0.05; // Reduce velocity gain to avoid oscillations
 }
 
 // Get time delta between readings
 double dt = TIME_STEP; // Use control loop time step for consistency
 
 // Prediction step
 double prediction = prev_filtered[i] + prev_velocity[i] * dt;
 
 // Update step
 arm->sensors.filtered_values[i] = prediction + 
 alpha * (arm->sensors.raw_values[i] - prediction);
 
 double new_velocity = prev_velocity[i] + 
 (beta / dt) * (arm->sensors.raw_values[i] - prediction);
 
 // Apply limits to velocity to avoid filter instability
 if (new_velocity > 1.0) new_velocity = 1.0;
 if (new_velocity < -1.0) new_velocity = -1.0;
 
 // Store for next iteration
 prev_filtered[i] = arm->sensors.filtered_values[i];
 prev_velocity[i] = new_velocity;
 }
}

/**
 * Detect sensor faults using data validation
 */
void detectSensorFaults(RoboticArm *arm) {
 // Use a static flag to control output during simulation
 static int silent_mode = 0;
 
 // Detect faults by comparing redundant sensors
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Compare primary and secondary position sensors
 double pos1 = arm->sensors.filtered_values[i*4];
 double pos2 = arm->sensors.filtered_values[i*4+1];
 
 // If difference is too large, mark one as invalid
 // Use a more reasonable threshold (0.005 rad ≈ 0.3 degrees)
 if (fabs(pos1 - pos2) > 0.005 && fabs(pos1 - pos2) < 10.0) { 
 // Only report realistic disagreements - this prevents the excessive warnings
 // Also, only report when not in silent mode
 static int report_count = 0;
 if (!silent_mode && report_count < 5) { // Limit reporting to first 5 occurrences
 tm_log(TM_WARN, "sensor_disagree joint=%d delta=%.6f rad",
 i, fabs(pos1 - pos2));
 report_count++;
 }
 
 // Determine which one is likely faulty
 // Mark the joint further from the commanded position
 double cmd_pos = arm->cmd_joint_pos[i];
 if (fabs(pos1 - cmd_pos) > fabs(pos2 - cmd_pos)) {
 arm->sensors.valid_flags[i*4] = false;
 } else {
 arm->sensors.valid_flags[i*4+1] = false;
 }
 }
 }
}

/**
 * Update forward kinematics to compute end-effector pose from joint angles
 * Heavy use of trigonometry functions - suitable for hardware acceleration
 */
void updateForwardKinematics(RoboticArm *arm) {
 // Transform matrices for each joint
 Matrix transforms[NUM_JOINTS];
 
 // Initialize all transform matrices
 for (int i = 0; i < NUM_JOINTS; i++) {
 initializeMatrix(&transforms[i], 0.0);
 }
 
 // Calculate transformation matrix for each joint using DH parameters
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Get DH parameters
 double theta = arm->dh_params[i][0] + arm->joint_states[i].position;
 double d = arm->dh_params[i][1];
 double a = arm->dh_params[i][2];
 double alpha = arm->dh_params[i][3];
 
 // Calculate trig functions (this is where hardware acceleration would help)
 double cos_theta = cos(theta);
 double sin_theta = sin(theta);
 double cos_alpha = cos(alpha);
 double sin_alpha = sin(alpha);
 
 // Build the DH transformation matrix
 transforms[i].data[0][0] = cos_theta;
 transforms[i].data[0][1] = -sin_theta * cos_alpha;
 transforms[i].data[0][2] = sin_theta * sin_alpha;
 transforms[i].data[0][3] = a * cos_theta;
 
 transforms[i].data[1][0] = sin_theta;
 transforms[i].data[1][1] = cos_theta * cos_alpha;
 transforms[i].data[1][2] = -cos_theta * sin_alpha;
 transforms[i].data[1][3] = a * sin_theta;
 
 transforms[i].data[2][0] = 0;
 transforms[i].data[2][1] = sin_alpha;
 transforms[i].data[2][2] = cos_alpha;
 transforms[i].data[2][3] = d;
 
 transforms[i].data[3][0] = 0;
 transforms[i].data[3][1] = 0;
 transforms[i].data[3][2] = 0;
 transforms[i].data[3][3] = 1;
 }
 
 // Compute the end-effector transform by multiplying all joint transforms
 Matrix end_transform;
 initializeMatrix(&end_transform, 0.0);
 
 // Start with identity matrix
 for (int i = 0; i < 4; i++) {
 end_transform.data[i][i] = 1.0;
 }
 
 // Multiply all transforms (T0_N = T0_1 * T1_2 * ... * TN-1_N)
 for (int i = 0; i < NUM_JOINTS; i++) {
 Matrix temp = end_transform;
 end_transform = multiplyMatrices(&temp, &transforms[i]);
 }
 
 // Extract position from the transformation matrix
 arm->end_effector.position.x = end_transform.data[0][3];
 arm->end_effector.position.y = end_transform.data[1][3];
 arm->end_effector.position.z = end_transform.data[2][3];
 
 // Extract rotation matrix
 Matrix rotation;
 initializeMatrix(&rotation, 0.0);
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 rotation.data[i][j] = end_transform.data[i][j];
 }
 }
 
 // Convert rotation matrix to quaternion
 // This is a computation-intensive operation with many trig functions
 double trace = rotation.data[0][0] + rotation.data[1][1] + rotation.data[2][2];
 
 if (trace > 0) {
 double S = sqrt(trace + 1.0) * 2.0;
 arm->end_effector.quaternion[0] = 0.25 * S;
 arm->end_effector.quaternion[1] = (rotation.data[2][1] - rotation.data[1][2]) / S;
 arm->end_effector.quaternion[2] = (rotation.data[0][2] - rotation.data[2][0]) / S;
 arm->end_effector.quaternion[3] = (rotation.data[1][0] - rotation.data[0][1]) / S;
 } else if (rotation.data[0][0] > rotation.data[1][1] && rotation.data[0][0] > rotation.data[2][2]) {
 double S = sqrt(1.0 + rotation.data[0][0] - rotation.data[1][1] - rotation.data[2][2]) * 2.0;
 arm->end_effector.quaternion[0] = (rotation.data[2][1] - rotation.data[1][2]) / S;
 arm->end_effector.quaternion[1] = 0.25 * S;
 arm->end_effector.quaternion[2] = (rotation.data[0][1] + rotation.data[1][0]) / S;
 arm->end_effector.quaternion[3] = (rotation.data[0][2] + rotation.data[2][0]) / S;
 } else if (rotation.data[1][1] > rotation.data[2][2]) {
 double S = sqrt(1.0 + rotation.data[1][1] - rotation.data[0][0] - rotation.data[2][2]) * 2.0;
 arm->end_effector.quaternion[0] = (rotation.data[0][2] - rotation.data[2][0]) / S;
 arm->end_effector.quaternion[1] = (rotation.data[0][1] + rotation.data[1][0]) / S;
 arm->end_effector.quaternion[2] = 0.25 * S;
 arm->end_effector.quaternion[3] = (rotation.data[1][2] + rotation.data[2][1]) / S;
 } else {
 double S = sqrt(1.0 + rotation.data[2][2] - rotation.data[0][0] - rotation.data[1][1]) * 2.0;
 arm->end_effector.quaternion[0] = (rotation.data[1][0] - rotation.data[0][1]) / S;
 arm->end_effector.quaternion[1] = (rotation.data[0][2] + rotation.data[2][0]) / S;
 arm->end_effector.quaternion[2] = (rotation.data[1][2] + rotation.data[2][1]) / S;
 arm->end_effector.quaternion[3] = 0.25 * S;
 }
 
 // Normalize the quaternion
 quaternionNormalize(arm->end_effector.quaternion);
}

/**
 * Calculate the Jacobian matrix for the arm
 * This maps joint velocities to end-effector velocities
 */
void calculateJacobian(RoboticArm *arm, JacobianMatrix *jacobian) {
 // Initialize Jacobian matrix
 for (int i = 0; i < SMALL_MATRIX_SIZE; i++) {
 for (int j = 0; j < NUM_JOINTS; j++) {
 jacobian->data[i][j] = 0.0;
 }
 }
 
 // Save current joint positions
 double original_positions[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 original_positions[i] = arm->joint_states[i].position;
 }
 
 // Current end-effector pose
 Pose current_pose = arm->end_effector;
 
 // Small delta for numerical differentiation
 const double delta = 0.0001; // 0.1 mrad
 
 // For each joint, perturb and calculate partial derivatives
 for (int j = 0; j < NUM_JOINTS; j++) {
 // Perturb this joint
 arm->joint_states[j].position += delta;
 
 // Recalculate forward kinematics
 updateForwardKinematics(arm);
 
 // Get new end-effector pose
 Pose perturbed_pose = arm->end_effector;
 
 // Calculate position difference
 Vector3D pos_diff = {
 (perturbed_pose.position.x - current_pose.position.x) / delta,
 (perturbed_pose.position.y - current_pose.position.y) / delta,
 (perturbed_pose.position.z - current_pose.position.z) / delta
 };
 
 // Fill in the position part of the Jacobian (first 3 rows)
 jacobian->data[0][j] = pos_diff.x;
 jacobian->data[1][j] = pos_diff.y;
 jacobian->data[2][j] = pos_diff.z;
 
 // Calculate orientation difference (quaternion derivative)
 // Small-angle rotation update
 double dq[4];
 double inv_current[4];
 quaternionConjugate(inv_current, current_pose.quaternion);
 quaternionMultiply(dq, perturbed_pose.quaternion, inv_current);
 
 // Convert quaternion derivative to angular velocity
 // For small rotations: w ≈ 2*[qx, qy, qz]/dt if qw ≈ 1
 if (dq[0] > 0.999) { // Small rotation approximation is valid
 jacobian->data[3][j] = 2.0 * dq[1] / delta; // wx
 jacobian->data[4][j] = 2.0 * dq[2] / delta; // wy
 jacobian->data[5][j] = 2.0 * dq[3] / delta; // wz
 } else {
 // For larger rotations, use the full formula
 double angle = 2.0 * acos(dq[0]) / delta;
 double sin_half_angle = sin(acos(dq[0]));
 
 if (sin_half_angle > 1e-10) {
 jacobian->data[3][j] = angle * dq[1] / sin_half_angle;
 jacobian->data[4][j] = angle * dq[2] / sin_half_angle;
 jacobian->data[5][j] = angle * dq[3] / sin_half_angle;
 } else {
 jacobian->data[3][j] = jacobian->data[4][j] = jacobian->data[5][j] = 0.0;
 }
 }
 
 // Restore the original joint position
 arm->joint_states[j].position = original_positions[j];
 }
 
 // Restore the end-effector pose by recalculating forward kinematics
 updateForwardKinematics(arm);
}

/**
 * Compute Yoshikawa manipulability index: w = sqrt(det(J * J^T))
 * Returns 0 when the arm approaches a kinematic singularity ("lock" position).
 * safety metric: halt motion when w < MANIP_IDX_THRESHOLD.
 */
double compute_manipulability_index(JacobianMatrix *jacobian) {
 /* J is SMALL_MATRIX_SIZE x NUM_JOINTS (6×7).
 JJT = J * J^T is SMALL_MATRIX_SIZE × SMALL_MATRIX_SIZE (6×6).
 Compute det(JJT) via LU factorisation with partial pivoting. */
 double JJT[SMALL_MATRIX_SIZE][SMALL_MATRIX_SIZE];
 for (int i = 0; i < SMALL_MATRIX_SIZE; i++) {
 for (int j = 0; j < SMALL_MATRIX_SIZE; j++) {
 double sum = 0.0;
 for (int k = 0; k < NUM_JOINTS; k++) {
 sum += jacobian->data[i][k] * jacobian->data[j][k];
 }
 JJT[i][j] = sum;
 }
 }

 /* In-place LU with partial pivoting — determinant = product of diag(U) * sign */
 double det = 1.0;
 int sign = 1;
 for (int col = 0; col < SMALL_MATRIX_SIZE; col++) {
 /* pivot */
 int piv = col;
 double best = fabs(JJT[col][col]);
 for (int row = col + 1; row < SMALL_MATRIX_SIZE; row++) {
 if (fabs(JJT[row][col]) > best) {
 best = fabs(JJT[row][col]);
 piv = row;
 }
 }
 if (piv != col) {
 for (int k = 0; k < SMALL_MATRIX_SIZE; k++) {
 double tmp = JJT[col][k];
 JJT[col][k] = JJT[piv][k];
 JJT[piv][k] = tmp;
 }
 sign = -sign;
 }
 double d = JJT[col][col];
 if (fabs(d) < 1e-30) return 0.0; /* singular */
 det *= d;
 /* eliminate below */
 for (int row = col + 1; row < SMALL_MATRIX_SIZE; row++) {
 double f = JJT[row][col] / d;
 for (int k = col + 1; k < SMALL_MATRIX_SIZE; k++) {
 JJT[row][k] -= f * JJT[col][k];
 }
 }
 }
 det *= sign;
 /* det(JJT) ≥ 0 by construction; guard floating-point noise */
 return (det > 0.0) ? sqrt(det) : 0.0;
}

/**
 * Solve inverse kinematics to find joint angles for a target pose
 * This is a highly compute-intensive function with trigonometry and matrix operations
 */
void solveInverseKinematics(RoboticArm *arm, Pose *target_pose) {
 // Using damped least squares method (Levenberg-Marquardt)
 
 // Maximum number of iterations
 const int max_iterations = 100;
 
 // Convergence tolerance
 const double tolerance_pos = 0.001; // 1mm
 const double tolerance_ori = 0.001; // ~0.05 degrees
 
 // Damping factor
 const double lambda = 0.1;
 
 // Jacobian matrix
 JacobianMatrix jacobian;
 
 // Iterate until convergence or max iterations
 for (int iter = 0; iter < max_iterations; iter++) {
 // Update forward kinematics for current joint positions
 updateForwardKinematics(arm);
 
 // Calculate error between current and target poses
 Vector3D pos_error = {
 target_pose->position.x - arm->end_effector.position.x,
 target_pose->position.y - arm->end_effector.position.y,
 target_pose->position.z - arm->end_effector.position.z
 };
 
 // Calculate orientation error (quaternion difference)
 double orientation_error[4];
 double inv_current[4];
 quaternionConjugate(inv_current, arm->end_effector.quaternion);
 quaternionMultiply(orientation_error, target_pose->quaternion, inv_current);
 
 // Convert quaternion difference to axis-angle representation
 Vector3D axis_angle = {0.0, 0.0, 0.0};
 
 if (orientation_error[0] < 0.999999) {
 // Extract axis-angle from quaternion
 double angle = 2.0 * acos(orientation_error[0]);
 double sin_half_angle = sqrt(1.0 - orientation_error[0] * orientation_error[0]);
 
 if (sin_half_angle > 1e-10) {
 axis_angle.x = orientation_error[1] / sin_half_angle * angle;
 axis_angle.y = orientation_error[2] / sin_half_angle * angle;
 axis_angle.z = orientation_error[3] / sin_half_angle * angle;
 }
 }
 
 // Check for convergence
 double pos_error_mag = sqrt(pos_error.x*pos_error.x + 
 pos_error.y*pos_error.y + 
 pos_error.z*pos_error.z);
 
 double ori_error_mag = sqrt(axis_angle.x*axis_angle.x + 
 axis_angle.y*axis_angle.y + 
 axis_angle.z*axis_angle.z);
 
 if (pos_error_mag < tolerance_pos && ori_error_mag < tolerance_ori) {
 // Solution found
 break;
 }
 
 // Calculate Jacobian matrix
 calculateJacobian(arm, &jacobian);
 
 /* --- Manipulability Index safety check (singularity guard) --- */
 double w_manip = compute_manipulability_index(&jacobian);
 double lambda_eff = lambda;
 if (w_manip < MANIP_IDX_THRESHOLD) {
 /* Arm is near a kinematic lock position — boost DLS damping
 to prevent divergent joint velocities. */
 static int manip_warn_count = 0;
 if (manip_warn_count < 3) {
 tm_log(TM_WARN, "manip_idx=%.2e < threshold, boosting DLS damping", w_manip);
 manip_warn_count++;
 }
 lambda_eff = lambda * 10.0;
 }
 
 // Construct the error vector
 double error[SMALL_MATRIX_SIZE] = {
 pos_error.x, pos_error.y, pos_error.z,
 axis_angle.x, axis_angle.y, axis_angle.z
 };
 
 // Solve the system (J * dq = error) using damped least squares
 // This involves computing (J^T * J + λI)^-1 * J^T * error
 
 // Step 1: Calculate J^T * J (a 7x7 matrix for 7 joints)
 double JtJ[NUM_JOINTS][NUM_JOINTS] = {0};
 for (int i = 0; i < NUM_JOINTS; i++) {
 for (int j = 0; j < NUM_JOINTS; j++) {
 for (int k = 0; k < SMALL_MATRIX_SIZE; k++) {
 JtJ[i][j] += jacobian.data[k][i] * jacobian.data[k][j];
 }
 }
 }
 
 // Step 2: Add damping factor λI (uses lambda_eff for singularity-aware DLS)
 for (int i = 0; i < NUM_JOINTS; i++) {
 JtJ[i][i] += lambda_eff;
 }
 
 // Step 3: Calculate J^T * error
 double Jt_error[NUM_JOINTS] = {0};
 for (int i = 0; i < NUM_JOINTS; i++) {
 for (int j = 0; j < SMALL_MATRIX_SIZE; j++) {
 Jt_error[i] += jacobian.data[j][i] * error[j];
 }
 }
 
 // Step 4: Solve the system (JtJ) * dq = Jt_error
 // Using Gauss-Jordan elimination
 double augmented[NUM_JOINTS][NUM_JOINTS+1];
 
 // Create augmented matrix [JtJ | Jt_error]
 for (int i = 0; i < NUM_JOINTS; i++) {
 for (int j = 0; j < NUM_JOINTS; j++) {
 augmented[i][j] = JtJ[i][j];
 }
 augmented[i][NUM_JOINTS] = Jt_error[i];
 }
 
 // Perform Gauss-Jordan elimination
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Find pivot
 int pivot_row = i;
 double pivot_val = fabs(augmented[i][i]);
 
 for (int j = i+1; j < NUM_JOINTS; j++) {
 if (fabs(augmented[j][i]) > pivot_val) {
 pivot_row = j;
 pivot_val = fabs(augmented[j][i]);
 }
 }
 
 // Swap rows if needed
 if (pivot_row != i) {
 for (int j = 0; j <= NUM_JOINTS; j++) {
 double temp = augmented[i][j];
 augmented[i][j] = augmented[pivot_row][j];
 augmented[pivot_row][j] = temp;
 }
 }
 
 // Normalize the pivot row
 double pivot = augmented[i][i];
 if (fabs(pivot) < 1e-10) {
 // Nearly singular matrix, use a small value
 pivot = (pivot < 0) ? -1e-10 : 1e-10;
 }
 
 for (int j = i; j <= NUM_JOINTS; j++) {
 augmented[i][j] /= pivot;
 }
 
 // Eliminate other rows
 for (int j = 0; j < NUM_JOINTS; j++) {
 if (j != i) {
 double factor = augmented[j][i];
 for (int k = i; k <= NUM_JOINTS; k++) {
 augmented[j][k] -= factor * augmented[i][k];
 }
 }
 }
 }
 
 // Extract the solution (dq values)
 double dq[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 dq[i] = augmented[i][NUM_JOINTS];
 }
 
 // Update joint positions
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Apply joint position update with a damping factor
 // to ensure smooth convergence
 double alpha = 0.5; // Step size factor
 arm->joint_states[i].position += alpha * dq[i];
 
 // Apply joint limits
 if (arm->joint_states[i].position < arm->joint_limits[i][0]) {
 arm->joint_states[i].position = arm->joint_limits[i][0];
 } else if (arm->joint_states[i].position > arm->joint_limits[i][1]) {
 arm->joint_states[i].position = arm->joint_limits[i][1];
 }
 }
 }
 
 // Update the commanded joint positions
 for (int i = 0; i < NUM_JOINTS; i++) {
 arm->cmd_joint_pos[i] = arm->joint_states[i].position;
 }
}

/**
 * Generate a smooth trajectory between two poses
 */
void generateTrajectory(RoboticArm *arm, Pose *start, Pose *end, int steps) {
 // Clear any existing trajectory
 arm->trajectory_length = steps;
 arm->trajectory_index = 0;
 
 // Generate a smooth trajectory using quintic polynomials
 // This ensures continuous position, velocity, and acceleration
 
 for (int i = 0; i < steps; i++) {
 double t = (double)i / (steps - 1); // Normalized time from 0 to 1
 
 // Quintic polynomial blending function
 // s(t) = 10t^3 - 15t^4 + 6t^5
 // This has zero velocity and acceleration at endpoints
 double s = 10 * pow(t, 3) - 15 * pow(t, 4) + 6 * pow(t, 5);
 
 // Interpolate position
 arm->trajectory[i].position.x = start->position.x + s * (end->position.x - start->position.x);
 arm->trajectory[i].position.y = start->position.y + s * (end->position.y - start->position.y);
 arm->trajectory[i].position.z = start->position.z + s * (end->position.z - start->position.z);
 
 // Interpolate orientation using quaternion slerp
 double dot = start->quaternion[0]*end->quaternion[0] + 
 start->quaternion[1]*end->quaternion[1] + 
 start->quaternion[2]*end->quaternion[2] + 
 start->quaternion[3]*end->quaternion[3];
 
 // If quaternions are nearly opposite, flip one to get the shorter path
 double end_q[4];
 if (dot < 0) {
 dot = -dot;
 for (int k = 0; k < 4; k++) end_q[k] = -end->quaternion[k];
 } else {
 for (int k = 0; k < 4; k++) end_q[k] = end->quaternion[k];
 }
 
 // Perform spherical linear interpolation (SLERP)
 if (dot > 0.9995) {
 // If quaternions are nearly the same, use linear interpolation
 arm->trajectory[i].quaternion[0] = start->quaternion[0] + s * (end_q[0] - start->quaternion[0]);
 arm->trajectory[i].quaternion[1] = start->quaternion[1] + s * (end_q[1] - start->quaternion[1]);
 arm->trajectory[i].quaternion[2] = start->quaternion[2] + s * (end_q[2] - start->quaternion[2]);
 arm->trajectory[i].quaternion[3] = start->quaternion[3] + s * (end_q[3] - start->quaternion[3]);
 
 // Normalize the result
 quaternionNormalize(arm->trajectory[i].quaternion);
 } else {
 // Use SLERP for larger rotations
 double theta = acos(dot);
 double sin_theta = sin(theta);
 
 double ratio1 = sin((1.0 - s) * theta) / sin_theta;
 double ratio2 = sin(s * theta) / sin_theta;
 
 // Calculate interpolated quaternion
 arm->trajectory[i].quaternion[0] = ratio1 * start->quaternion[0] + ratio2 * end_q[0];
 arm->trajectory[i].quaternion[1] = ratio1 * start->quaternion[1] + ratio2 * end_q[1];
 arm->trajectory[i].quaternion[2] = ratio1 * start->quaternion[2] + ratio2 * end_q[2];
 arm->trajectory[i].quaternion[3] = ratio1 * start->quaternion[3] + ratio2 * end_q[3];
 }
 }
}

/**
 * Check for collision between the arm and obstacles
 */
bool checkCollision(RoboticArm *arm, Pose *pose) {
 // Workspace boundary collision check
 // Collision detection using axis-aligned bounding box checks
 
 // Current joint positions
 double original_positions[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 original_positions[i] = arm->joint_states[i].position;
 }
 
 // Set arm to the pose to check
 solveInverseKinematics(arm, pose);
 updateForwardKinematics(arm);
 
 // Check for collisions with each segment of the arm
 bool collision = false;
 
 // For each obstacle
 for (int i = 0; i < NUM_OBSTACLES; i++) {
 // Check end-effector position
 // Check all arm segments against workspace boundaries
 double dx = arm->end_effector.position.x - arm->obstacles[i].position.x;
 double dy = arm->end_effector.position.y - arm->obstacles[i].position.y;
 double dz = arm->end_effector.position.z - arm->obstacles[i].position.z;
 
 double distance = sqrt(dx*dx + dy*dy + dz*dz);
 
 // Check if distance is less than sum of radii
 if (distance < (arm->segments[NUM_JOINTS-1].radius + arm->obstacles[i].radius)) {
 collision = true;
 break;
 }
 }
 
 // Restore original joint positions
 for (int i = 0; i < NUM_JOINTS; i++) {
 arm->joint_states[i].position = original_positions[i];
 }
 updateForwardKinematics(arm);
 
 return collision;
}

/**
 * Plan a collision-free path to the target pose
 */
void planPath(RoboticArm *arm, Pose *target) {
 tm_log(TM_INFO, "path_plan target=(%.2f,%.2f,%.2f)",
 target->position.x, target->position.y, target->position.z);
 
 // Check if direct path is possible
 Pose current = arm->end_effector;
 
 // Generate a direct trajectory and check for collisions
 int steps = 20;
 Pose test_trajectory[20];
 
 for (int i = 0; i < steps; i++) {
 double t = (double)i / (steps - 1);
 
 // Linear interpolation 
 test_trajectory[i].position.x = current.position.x + t * (target->position.x - current.position.x);
 test_trajectory[i].position.y = current.position.y + t * (target->position.y - current.position.y);
 test_trajectory[i].position.z = current.position.z + t * (target->position.z - current.position.z);
 
 // Linear interpolation of quaternion with renormalization
 for (int j = 0; j < 4; j++) {
 test_trajectory[i].quaternion[j] = current.quaternion[j] + t * (target->quaternion[j] - current.quaternion[j]);
 }
 quaternionNormalize(test_trajectory[i].quaternion);
 
 // Check for collision
 if (checkCollision(arm, &test_trajectory[i])) {
 tm_log(TM_WARN, "collision at step %d, rerouting via waypoint", i);
 
 // Path planning using workspace-aware trajectory optimization
 // like RRT (Rapidly-exploring Random Tree) or PRM (Probabilistic Roadmap)
 
 // For this simulation, we'll use a simple waypoint approach
 // Find a waypoint above the direct path
 Pose waypoint = current;
 waypoint.position.x = (current.position.x + target->position.x) / 2.0;
 waypoint.position.y = (current.position.y + target->position.y) / 2.0;
 waypoint.position.z = (current.position.z + target->position.z) / 2.0 + 5.0; // 5m above midpoint
 
 // Generate trajectory through waypoint
 generateTrajectory(arm, &current, &waypoint, steps/2);
 generateTrajectory(arm, &waypoint, target, steps/2);
 
 tm_log(TM_INFO, "waypoint=(%.2f,%.2f,%.2f)",
 waypoint.position.x, waypoint.position.y, waypoint.position.z);
 
 arm->trajectory_length = steps;
 return;
 }
 }
 
 // If no collisions, use direct path
 tm_log(TM_INFO, "direct path collision-free");
 generateTrajectory(arm, &current, target, steps);
 
 // Optimize the trajectory
 optimizeTrajectory(arm);
}

/**
 * Optimize the trajectory for minimum jerk
 */
void optimizeTrajectory(RoboticArm *arm) {
 // This function would optimize the trajectory to minimize jerk
 // For simulation purposes, we'll use a simple smoothing algorithm
 
 // Skip if trajectory is too short
 if (arm->trajectory_length < 5) return;
 
 // Smooth position trajectories
 for (int iter = 0; iter < 3; iter++) { // 3 smoothing passes
 for (int i = 2; i < arm->trajectory_length - 2; i++) {
 // 5-point moving average
 Vector3D avg_pos = {0, 0, 0};
 
 for (int j = -2; j <= 2; j++) {
 avg_pos.x += arm->trajectory[i+j].position.x;
 avg_pos.y += arm->trajectory[i+j].position.y;
 avg_pos.z += arm->trajectory[i+j].position.z;
 }
 
 avg_pos.x /= 5.0;
 avg_pos.y /= 5.0;
 avg_pos.z /= 5.0;
 
 // Update position with weighted average (preserve start/end points)
 double weight = 0.5; // 50% weight to smoothed value
 arm->trajectory[i].position.x = (1.0-weight) * arm->trajectory[i].position.x + weight * avg_pos.x;
 arm->trajectory[i].position.y = (1.0-weight) * arm->trajectory[i].position.y + weight * avg_pos.y;
 arm->trajectory[i].position.z = (1.0-weight) * arm->trajectory[i].position.z + weight * avg_pos.z;
 }
 }
 
 tm_log(TM_INFO, "trajectory optimised");
}

/**
 * Calculate the dynamic model of the arm for control
 * This includes mass matrix, Coriolis forces, and gravity effects
 */
void calculateDynamicsModel(RoboticArm *arm, Matrix *mass_matrix, double *coriolis, double *gravity) {
 // Recursion guard: when computing Coriolis terms via finite differences,
 // the recursive calls only need the mass matrix — skip Coriolis/gravity
 // to prevent infinite recursion.
 static int computing_mass_matrix_only = 0;

 // Initialize matrices
 initializeMatrix(mass_matrix, 0.0);
 
 // Initialize vectors
 for (int i = 0; i < NUM_JOINTS; i++) {
 coriolis[i] = 0.0;
 gravity[i] = 0.0;
 }
 
 // Mass matrix calculation (inertia matrix)
 // Dynamics model using link mass/inertia properties
 // Compute dynamic parameters from link geometry
 
 // Diagonal components (joint inertias)
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Base inertia for each joint
 double inertia = arm->segments[i].mass;
 
 // Add effect of distal segments
 for (int j = i+1; j < NUM_JOINTS; j++) {
 // Distance to COM of segment j
 double distance = 0.0;
 for (int k = i+1; k <= j; k++) {
 distance += arm->segments[k-1].com.x; // Accumulated link COM offset
 }
 
 // Add parallel axis theorem contribution
 inertia += arm->segments[j].mass * distance * distance;
 }
 
 // Add payload contribution if this is the last joint
 if (i == NUM_JOINTS-1 && arm->payload_mass > 0.0) {
 double end_effector_distance = arm->end_effector.position.x - 
 arm->segments[NUM_JOINTS-1].position.x;
 inertia += arm->payload_mass * end_effector_distance * end_effector_distance;
 }
 
 // Set diagonal element
 mass_matrix->data[i][i] = inertia;
 }
 
 // Off-diagonal elements (coupling inertias via Lagrangian dynamics)
 // M_ij = Σ_k m_k * (∂p_k/∂q_i)^T * (∂p_k/∂q_j) + trace(I_k * R_k_i^T * R_k_j)
 for (int i = 0; i < NUM_JOINTS; i++) {
 for (int j = i+1; j < NUM_JOINTS; j++) {
 double coupling = 0.0;

 // Coupling from distal masses: sum over links beyond both joints
 for (int k = j; k < NUM_JOINTS; k++) {
 double m_k = arm->segments[k].mass;
 double d_ik = 0.0, d_jk = 0.0;
 for (int l = i; l <= k; l++) d_ik += arm->segments[l].com.x;
 for (int l = j; l <= k; l++) d_jk += arm->segments[l].com.x;

 // Configuration-dependent coupling: m_k * d_ik * d_jk * cos(q_i - q_j)
 double q_diff = arm->joint_states[i].position - arm->joint_states[j].position;
 coupling += m_k * d_ik * d_jk * cos(q_diff);
 }

 mass_matrix->data[i][j] = coupling;
 mass_matrix->data[j][i] = coupling;
 }
 }
 
 // Coriolis and centrifugal terms via Christoffel symbols of the first kind
 // c_{ijk} = 0.5 * (∂M_ij/∂q_k + ∂M_ik/∂q_j - ∂M_jk/∂q_i)
 // C_i = Σ_j Σ_k c_{ijk} * q̇_j * q̇_k
 if (!computing_mass_matrix_only) {
 double dM[NUM_JOINTS][NUM_JOINTS][NUM_JOINTS]; // ∂M_ij/∂q_k
 double dq = 1e-6; // finite-difference step
 double saved_pos[NUM_JOINTS];

 // Save current positions
 for (int n = 0; n < NUM_JOINTS; n++)
 saved_pos[n] = arm->joint_states[n].position;

 // Compute ∂M/∂q_k by central finite difference
 // Set recursion guard: inner calls compute mass matrix only
 computing_mass_matrix_only = 1;
 for (int k = 0; k < NUM_JOINTS; k++) {
 Matrix M_plus, M_minus;
 double c_plus[NUM_JOINTS], g_plus[NUM_JOINTS];
 double c_minus[NUM_JOINTS], g_minus[NUM_JOINTS];

 arm->joint_states[k].position = saved_pos[k] + dq;
 calculateDynamicsModel(arm, &M_plus, c_plus, g_plus);
 arm->joint_states[k].position = saved_pos[k] - dq;
 calculateDynamicsModel(arm, &M_minus, c_minus, g_minus);
 arm->joint_states[k].position = saved_pos[k];

 for (int i = 0; i < NUM_JOINTS; i++)
 for (int j = 0; j < NUM_JOINTS; j++)
 dM[i][j][k] = (M_plus.data[i][j] - M_minus.data[i][j]) / (2.0 * dq);
 }

 // Restore positions
 for (int n = 0; n < NUM_JOINTS; n++)
 arm->joint_states[n].position = saved_pos[n];

 // Clear recursion guard
 computing_mass_matrix_only = 0;

 // Compute Christoffel symbols → Coriolis vector
 for (int i = 0; i < NUM_JOINTS; i++) {
 coriolis[i] = 0.0;
 for (int j = 0; j < NUM_JOINTS; j++) {
 for (int k = 0; k < NUM_JOINTS; k++) {
 double c_ijk = 0.5 * (dM[i][j][k] + dM[i][k][j] - dM[j][k][i]);
 coriolis[i] += c_ijk *
 arm->joint_states[j].velocity *
 arm->joint_states[k].velocity;
 }
 }
 }
 } /* end if (!computing_mass_matrix_only) */
 
 // Gravity-gradient torque in orbital environment
 // In LEO, g-gradient torque on each link: τ_i = 3·ω₀²·(I_max - I_min)·sin(2·θ_i)/2
 // where ω₀ is orbital angular velocity (~0.001 rad/s for LEO)
 double omega0_sq = 1.0e-6; // (orbital rate)^2 ≈ (0.001)^2
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Gravity-gradient torque depends on link attitude relative to nadir
 double theta_i = arm->joint_states[i].position;
 double I_xx = arm->segments[i].mass * 0.5; // approx principal MOI
 double I_zz = arm->segments[i].mass * 0.05; // minor MOI
 gravity[i] = 1.5 * omega0_sq * (I_xx - I_zz) * sin(2.0 * theta_i);

 // Add cumulative effect from distal links
 for (int j = i + 1; j < NUM_JOINTS; j++) {
 double d = 0.0;
 for (int k = i; k < j; k++) d += arm->segments[k].com.x;
 gravity[i] += omega0_sq * arm->segments[j].mass * d * sin(theta_i);
 }
 }
}

/**
 * Compensate for nonlinear dynamics in the control
 */
void compensateNonlinearDynamics(RoboticArm *arm) {
 // Calculate dynamic model
 Matrix mass_matrix;
 double coriolis[NUM_JOINTS];
 double gravity[NUM_JOINTS];
 
 calculateDynamicsModel(arm, &mass_matrix, coriolis, gravity);
 
 // Compensate for Coriolis forces and gravity
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Feed-forward compensation
 arm->cmd_joint_torque[i] += coriolis[i] + gravity[i];
 }
 
 // Add inertia scaling for commanded accelerations
 for (int i = 0; i < NUM_JOINTS; i++) {
 double inertia_torque = 0.0;
 
 for (int j = 0; j < NUM_JOINTS; j++) {
 inertia_torque += mass_matrix.data[i][j] * arm->cmd_joint_acc[j];
 }
 
 arm->cmd_joint_torque[i] += inertia_torque;
 }
}

/**
 * Compute the control law for the arm
 */
void computeControlLaw(RoboticArm *arm) {
 // Get the target pose from the trajectory
 Pose target;
 
 if (arm->trajectory_index < arm->trajectory_length) {
 target = arm->trajectory[arm->trajectory_index];
 arm->trajectory_index++;
 } else {
 target = arm->trajectory[arm->trajectory_length - 1];
 }
 
 // Solve inverse kinematics to get target joint positions
 double original_positions[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 original_positions[i] = arm->joint_states[i].position;
 }
 
 solveInverseKinematics(arm, &target);
 
 // Get the target joint positions
 double target_positions[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 target_positions[i] = arm->joint_states[i].position;
 
 // Restore current positions
 arm->joint_states[i].position = original_positions[i];
 }
 
 // PID control for each joint
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Current joint state
 double position = arm->joint_states[i].position;
 
 // Target position and calculated velocity
 double target_position = target_positions[i];
 
 // Save previous error for derivative term
 static double prev_error[NUM_JOINTS] = {0};
 static double integral[NUM_JOINTS] = {0};
 
 // Calculate error
 double error = target_position - position;
 
 // Update integral term with anti-windup
 integral[i] += error * TIME_STEP;
 double max_integral = 5.0; // Anti-windup limit (keeps Ki·∫e within ~25 Nm)
 if (integral[i] > max_integral) integral[i] = max_integral;
 if (integral[i] < -max_integral) integral[i] = -max_integral;
 
 // Calculate derivative (velocity error)
 double derivative = (error - prev_error[i]) / TIME_STEP;
 prev_error[i] = error;
 
 // PID gains
 double Kp = arm->control_gains[i][0];
 double Ki = arm->control_gains[i][1];
 double Kd = arm->control_gains[i][2];
 
 // Calculate control signal (torque)
 double control = Kp * error + Ki * integral[i] + Kd * derivative;
 
 // Set commanded acceleration and torque
 // NOTE: cmd_joint_acc is used by compensateNonlinearDynamics as the
 // desired joint acceleration for the M·q̈ feed-forward term. Setting
 // it to the PID error-derivative is incorrect (it is not an
 // acceleration command) and produces enormous inertia torques because
 // the mass-matrix diagonal can be 10 000+ kg·m². Set to zero so
 // that compensateNonlinearDynamics only adds Coriolis + gravity
 // feed-forward, while the PID alone provides the corrective torque.
 arm->cmd_joint_acc[i] = 0.0;
 arm->cmd_joint_torque[i] = control;
 }
 
 // Compensate for nonlinear dynamics
 compensateNonlinearDynamics(arm);
 
 // Apply joint limits
 applyJointLimits(arm);
}

/**
 * Apply joint limits to the commanded values
 */
void applyJointLimits(RoboticArm *arm) {
 for (int i = 0; i < NUM_JOINTS; i++) {
 // Position limits
 if (arm->cmd_joint_pos[i] < arm->joint_limits[i][0]) {
 arm->cmd_joint_pos[i] = arm->joint_limits[i][0];
 } else if (arm->cmd_joint_pos[i] > arm->joint_limits[i][1]) {
 arm->cmd_joint_pos[i] = arm->joint_limits[i][1];
 }
 
 // Velocity limits
 if (arm->cmd_joint_vel[i] < -arm->joint_velocity_limits[i]) {
 arm->cmd_joint_vel[i] = -arm->joint_velocity_limits[i];
 } else if (arm->cmd_joint_vel[i] > arm->joint_velocity_limits[i]) {
 arm->cmd_joint_vel[i] = arm->joint_velocity_limits[i];
 }
 
 // Torque limits
 if (arm->cmd_joint_torque[i] < -arm->joint_torque_limits[i]) {
 arm->cmd_joint_torque[i] = -arm->joint_torque_limits[i];
 } else if (arm->cmd_joint_torque[i] > arm->joint_torque_limits[i]) {
 arm->cmd_joint_torque[i] = arm->joint_torque_limits[i];
 }
 }
}

/**
 * Run the simulation for a specified duration
 */
void runSimulation(RoboticArm *arm, double duration) {
 double time = 0.0;
 int step = 0;
 
 // Set up a scenario
 setArmScenario(arm, SCENARIO_CAPTURE);
 
 // Control loop
 while (time < duration) {
 // Sensor data acquisition and processing
 acquireSensorData(arm);
 filterSensorData(arm);
 detectSensorFaults(arm);
 
 // Update forward kinematics
 updateForwardKinematics(arm);
 
 // Compute control law
 computeControlLaw(arm);
 
 // Update system state
 updateArmState(arm, TIME_STEP);
 
 // Display telemetry at lower rate
 if (step % (CONTROL_RATE_HZ / TELEMETRY_RATE_HZ) == 0) {
 displayTelemetry(arm, time);
 }
 
 // Update time and step counter
 time += TIME_STEP;
 step++;
 }
}

/**
 * Update the arm state based on commanded values
 */
void updateArmState(RoboticArm *arm, double dt) {
 /* 4th-order Runge-Kutta integration of coupled manipulator dynamics.
 State vector per joint: [q, q̇]. Equation of motion:
 M(q)·q̈ = τ - C(q,q̇)·q̇ - G(q)
 We solve for q̈ = M⁻¹·(τ - C·q̇ - G) at each RK4 stage. */

 double q0[NUM_JOINTS], qd0[NUM_JOINTS];
 double k1_q[NUM_JOINTS], k1_qd[NUM_JOINTS];
 double k2_q[NUM_JOINTS], k2_qd[NUM_JOINTS];
 double k3_q[NUM_JOINTS], k3_qd[NUM_JOINTS];
 double k4_q[NUM_JOINTS], k4_qd[NUM_JOINTS];

 // Save initial state
 for (int i = 0; i < NUM_JOINTS; i++) {
 q0[i] = arm->joint_states[i].position;
 qd0[i] = arm->joint_states[i].velocity;
 }

 /* Helper: compute joint accelerations from dynamics model */
 #define COMPUTE_ACCEL(q_arr, qd_arr, acc_arr) do { \
 for (int _i = 0; _i < NUM_JOINTS; _i++) { \
 arm->joint_states[_i].position = (q_arr)[_i]; \
 arm->joint_states[_i].velocity = (qd_arr)[_i]; \
 } \
 Matrix _M; double _C[NUM_JOINTS], _G[NUM_JOINTS]; \
 calculateDynamicsModel(arm, &_M, _C, _G); \
 Matrix _Minv = inverseMatrix(&_M); \
 for (int _i = 0; _i < NUM_JOINTS; _i++) { \
 (acc_arr)[_i] = 0.0; \
 for (int _j = 0; _j < NUM_JOINTS; _j++) \
 (acc_arr)[_i] += _Minv.data[_i][_j] * \
 (arm->cmd_joint_torque[_j] - _C[_j] - _G[_j]); \
 double _ma = 1.0; \
 if ((acc_arr)[_i] > _ma) (acc_arr)[_i] = _ma; \
 if ((acc_arr)[_i] < -_ma) (acc_arr)[_i] = -_ma; \
 } \
 } while(0)

 double a_tmp[NUM_JOINTS];

 /* --- Stage 1 (t = t0) --- */
 COMPUTE_ACCEL(q0, qd0, a_tmp);
 for (int i = 0; i < NUM_JOINTS; i++) {
 k1_q[i] = qd0[i];
 k1_qd[i] = a_tmp[i];
 }

 /* --- Stage 2 (t = t0 + dt/2) --- */
 {
 double q_mid[NUM_JOINTS], qd_mid[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 q_mid[i] = q0[i] + 0.5 * dt * k1_q[i];
 qd_mid[i] = qd0[i] + 0.5 * dt * k1_qd[i];
 }
 COMPUTE_ACCEL(q_mid, qd_mid, a_tmp);
 for (int i = 0; i < NUM_JOINTS; i++) {
 k2_q[i] = qd_mid[i];
 k2_qd[i] = a_tmp[i];
 }
 }

 /* --- Stage 3 (t = t0 + dt/2) --- */
 {
 double q_mid[NUM_JOINTS], qd_mid[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 q_mid[i] = q0[i] + 0.5 * dt * k2_q[i];
 qd_mid[i] = qd0[i] + 0.5 * dt * k2_qd[i];
 }
 COMPUTE_ACCEL(q_mid, qd_mid, a_tmp);
 for (int i = 0; i < NUM_JOINTS; i++) {
 k3_q[i] = qd_mid[i];
 k3_qd[i] = a_tmp[i];
 }
 }

 /* --- Stage 4 (t = t0 + dt) --- */
 {
 double q_end[NUM_JOINTS], qd_end[NUM_JOINTS];
 for (int i = 0; i < NUM_JOINTS; i++) {
 q_end[i] = q0[i] + dt * k3_q[i];
 qd_end[i] = qd0[i] + dt * k3_qd[i];
 }
 COMPUTE_ACCEL(q_end, qd_end, a_tmp);
 for (int i = 0; i < NUM_JOINTS; i++) {
 k4_q[i] = qd_end[i];
 k4_qd[i] = a_tmp[i];
 }
 }

 #undef COMPUTE_ACCEL

 /* --- Weighted combination --- */
 for (int i = 0; i < NUM_JOINTS; i++) {
 arm->joint_states[i].position = q0[i] + (dt / 6.0) *
 (k1_q[i] + 2.0 * k2_q[i] + 2.0 * k3_q[i] + k4_q[i]);
 arm->joint_states[i].velocity = qd0[i] + (dt / 6.0) *
 (k1_qd[i] + 2.0 * k2_qd[i] + 2.0 * k3_qd[i] + k4_qd[i]);

 // Apply velocity limits
 if (arm->joint_states[i].velocity > arm->joint_velocity_limits[i])
 arm->joint_states[i].velocity = arm->joint_velocity_limits[i];
 else if (arm->joint_states[i].velocity < -arm->joint_velocity_limits[i])
 arm->joint_states[i].velocity = -arm->joint_velocity_limits[i];

 // Apply position limits
 if (arm->joint_states[i].position < arm->joint_limits[i][0]) {
 arm->joint_states[i].position = arm->joint_limits[i][0];
 arm->joint_states[i].velocity = 0;
 } else if (arm->joint_states[i].position > arm->joint_limits[i][1]) {
 arm->joint_states[i].position = arm->joint_limits[i][1];
 arm->joint_states[i].velocity = 0;
 }

 // Record final acceleration
 arm->joint_states[i].acceleration = (arm->joint_states[i].velocity - qd0[i]) / dt;

 // Update joint torque telemetry from computed command torque
 arm->joint_states[i].torque = arm->cmd_joint_torque[i];

 // Motor current model: I = τ/Kt where Kt is torque constant (Nm/A)
 // Space-rated brushless DC actuators: Kt ~ 2-10 Nm/A
 double Kt = 5.0;
 arm->joint_states[i].motor_current = arm->cmd_joint_torque[i] / Kt;

 // Thermal model: dT/dt = (R·I² - k_cool·(T - T_amb)) / C_thermal
 // R – winding resistance (Ω)
 // k_cool – conductive/radiative cooling coefficient (W/°C)
 // C_thermal – motor + gearbox thermal capacitance (J/°C)
 double R = 1.0, k_cool = 0.5, T_amb = 20.0, C_thermal = 100.0;
 double I = arm->joint_states[i].motor_current;
 double dTdt = (R * I * I - k_cool * (arm->joint_states[i].temperature - T_amb)) / C_thermal;
 arm->joint_states[i].temperature += dTdt * dt;
 }
 
 // Update forward kinematics to get new end-effector pose
 updateForwardKinematics(arm);
}

/**
 * Display telemetry information
 */
void displayTelemetry(RoboticArm *arm, double time) {
 tm_log(TM_STEP, "t=%.2fs", time);
 tm_log(TM_STEP, "ee_pos=(%.3f,%.3f,%.3f) m",
 arm->end_effector.position.x,
 arm->end_effector.position.y,
 arm->end_effector.position.z);
 
 double error_x = arm->target_pose.position.x - arm->end_effector.position.x;
 double error_y = arm->target_pose.position.y - arm->end_effector.position.y;
 double error_z = arm->target_pose.position.z - arm->end_effector.position.z;
 double pos_error = sqrt(error_x*error_x + error_y*error_y + error_z*error_z);
 
 tm_log(TM_STEP, "pos_err=%.3f m", pos_error);
 
 // Display joint data for the first and last joints
 tm_log(TM_STEP, "J1 pos=%.2f vel=%.2f tau=%.2f T=%.1f",
 arm->joint_states[0].position,
 arm->joint_states[0].velocity,
 arm->joint_states[0].torque,
 arm->joint_states[0].temperature);
 
 tm_log(TM_STEP, "J%d pos=%.2f vel=%.2f tau=%.2f T=%.1f",
 NUM_JOINTS,
 arm->joint_states[NUM_JOINTS-1].position,
 arm->joint_states[NUM_JOINTS-1].velocity,
 arm->joint_states[NUM_JOINTS-1].torque,
 arm->joint_states[NUM_JOINTS-1].temperature);
}
