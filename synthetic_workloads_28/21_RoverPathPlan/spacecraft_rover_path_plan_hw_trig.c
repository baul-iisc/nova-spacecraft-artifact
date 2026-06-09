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
 * Planetary Rover Path Planning Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Trig (CORDIC accelerator for trigonometric functions)
 * Application: (see workload description below)
 *
 * Description:
 * Implements autonomous path planning for planetary surface rovers using A*
 * search, artificial potential fields, and VFH (Vector Field Histogram) obstacle
 * avoidance. This workload includes terrain image processing (Sobel edge
 * detection, Gaussian filtering) for hazard identification, terrain traversability
 * evaluation, and optimization-based path smoothing. Derived from the lunar rover
 * rover path planner and applicable to future lunar polar rovers (LUPEX).
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
#include <stdbool.h>
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
/* Redirect standard trig to CORDIC accelerator */
#define sin(x) hw_sin(x)
#define cos(x) hw_cos(x)
#define asin(x) hw_asin(x)
#define acos(x) hw_acos(x)
#define tan(x) hw_tan(x)
#define atan(x) hw_atan(x)
#define atan2(y,x) hw_atan2(y,x)

// Define the dimensions for matrices
// 18x18 terrain grid aligned with 3x3 AME tile processing
#define MATRIX_DIM 18
#define TERRAIN_DIM 48 /* terrain DEM tile (flight: 256×256) */
#define MAX_OBSTACLES 50
#define MAX_PATH_POINTS 100
#define PI 3.14159265358979323846

// Structure definitions
typedef struct {
 double x;
 double y;
 double z;
} Point3D;

typedef struct {
 double x;
 double y;
 double theta; // Heading angle in radians
 double velocity;
 double angular_velocity;
} RoverState;

typedef struct {
 Point3D position;
 double radius;
 bool is_dynamic; // Whether obstacle can move
 double velocity; // Speed of obstacle if dynamic
 double direction; // Direction of movement in radians
} Obstacle;

typedef struct {
 double height[TERRAIN_DIM][TERRAIN_DIM];
 double friction[TERRAIN_DIM][TERRAIN_DIM];
 double elevation_gradient_x[TERRAIN_DIM][TERRAIN_DIM];
 double elevation_gradient_y[TERRAIN_DIM][TERRAIN_DIM];
} TerrainMap;

typedef struct {
 Point3D waypoints[MAX_PATH_POINTS];
 int num_waypoints;
} Path;

// Now add the function prototype here, after all the struct definitions:
void plan_and_execute_mission(TerrainMap *terrain, Obstacle obstacles[], int num_obstacles,
 Point3D start_point, Point3D goal_point);
double terrain_spectral_analysis(double elevation[], int n);

/*******************************************************************************
 * Terrain spectral roughness analysis via DFT-based power spectral density.
 * Identifies dominant spatial frequencies in a 1-D elevation profile to
 * classify terrain traversability. High-frequency energy corresponds to
 * rough terrain that increases rover traversal cost.
 *
 * BCE category: SpectralFeature (9%)
 ******************************************************************************/
double terrain_spectral_analysis(double elevation[], int n) {
 if (n < 2) return 0.0;

 // Remove DC component (mean elevation)
 double mean_elev = 0.0;
 for (int i = 0; i < n; i++) mean_elev += elevation[i];
 mean_elev /= n;

 double detrended[TERRAIN_DIM];
 for (int i = 0; i < n; i++) detrended[i] = elevation[i] - mean_elev;

 // Compute DFT magnitude squared (power spectral density)
 double psd[TERRAIN_DIM];
 int num_bins = n / 2; // Nyquist limit
 for (int k = 0; k < num_bins; k++) {
 double re = 0.0, im = 0.0;
 for (int t = 0; t < n; t++) {
 double angle = 2.0 * PI * k * t / (double)n;
 re += detrended[t] * cos(angle);
 im -= detrended[t] * sin(angle);
 }
 psd[k] = (re * re + im * im) / (double)(n * n);
 }

 // Partition into low-frequency and high-frequency bands
 int hf_start = num_bins / 2; // upper half of spectrum
 double total_energy = 0.0, hf_energy = 0.0;
 for (int k = 1; k < num_bins; k++) { // skip DC bin
 total_energy += psd[k];
 if (k >= hf_start) hf_energy += psd[k];
 }

 // Roughness metric: fraction of energy in high-frequency band
 double roughness = (total_energy > 1e-12) ? (hf_energy / total_energy) : 0.0;
 return roughness;
}

// Matrix and vector operations
void matrix_multiply(double A[MATRIX_DIM][MATRIX_DIM], double B[MATRIX_DIM][MATRIX_DIM], double C[MATRIX_DIM][MATRIX_DIM]) {
 // Software implementation
 // C = A * B
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 C[i][j] = 0.0;
 for (int k = 0; k < MATRIX_DIM; k++) {
 C[i][j] += A[i][k] * B[k][j];
 }
 }
 }
}

void matrix_add(double A[MATRIX_DIM][MATRIX_DIM], double B[MATRIX_DIM][MATRIX_DIM], double C[MATRIX_DIM][MATRIX_DIM]) {
 // C = A + B
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 C[i][j] = A[i][j] + B[i][j];
 }
 }
}

void matrix_subtract(double A[MATRIX_DIM][MATRIX_DIM], double B[MATRIX_DIM][MATRIX_DIM], double C[MATRIX_DIM][MATRIX_DIM]) {
 // C = A - B
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 C[i][j] = A[i][j] - B[i][j];
 }
 }
}

void matrix_transpose(double A[MATRIX_DIM][MATRIX_DIM], double AT[MATRIX_DIM][MATRIX_DIM]) {
 // AT = transpose of A
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 AT[j][i] = A[i][j];
 }
 }
}

void matrix_vector_multiply(double A[MATRIX_DIM][MATRIX_DIM], double v[MATRIX_DIM], double result[MATRIX_DIM]) {
 // result = A * v
 for (int i = 0; i < MATRIX_DIM; i++) {
 result[i] = 0.0;
 for (int j = 0; j < MATRIX_DIM; j++) {
 result[i] += A[i][j] * v[j];
 }
 }
}

double vector_dot_product(double v1[MATRIX_DIM], double v2[MATRIX_DIM]) {
 double result = 0.0;
 for (int i = 0; i < MATRIX_DIM; i++) {
 result += v1[i] * v2[i];
 }
 return result;
}

void vector_cross_product(double v1[3], double v2[3], double result[3]) {
 result[0] = v1[1] * v2[2] - v1[2] * v2[1];
 result[1] = v1[2] * v2[0] - v1[0] * v2[2];
 result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// Matrix inversion using Gauss-Jordan elimination (intensive calculation)
int matrix_inverse(double A[MATRIX_DIM][MATRIX_DIM], double Ainv[MATRIX_DIM][MATRIX_DIM]) {
 // Software implementation
 // Create augmented matrix [A|I]
 double augmented[MATRIX_DIM][2*MATRIX_DIM];
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 augmented[i][j] = A[i][j];
 augmented[i][j+MATRIX_DIM] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Gauss-Jordan elimination
 for (int i = 0; i < MATRIX_DIM; i++) {
 // Find pivot
 int pivot_row = i;
 double max_val = fabs(augmented[i][i]);
 for (int j = i+1; j < MATRIX_DIM; j++) {
 if (fabs(augmented[j][i]) > max_val) {
 max_val = fabs(augmented[j][i]);
 pivot_row = j;
 }
 }

 // Check for singular matrix
 if (fabs(augmented[pivot_row][i]) < 1e-10) {
 return 0; // Matrix is singular
 }

 // Swap rows if needed
 if (pivot_row != i) {
 for (int j = 0; j < 2*MATRIX_DIM; j++) {
 double temp = augmented[i][j];
 augmented[i][j] = augmented[pivot_row][j];
 augmented[pivot_row][j] = temp;
 }
 }

 // Scale pivot row
 double pivot = augmented[i][i];
 for (int j = 0; j < 2*MATRIX_DIM; j++) {
 augmented[i][j] /= pivot;
 }

 // Eliminate other rows
 for (int j = 0; j < MATRIX_DIM; j++) {
 if (j != i) {
 double factor = augmented[j][i];
 for (int k = 0; k < 2*MATRIX_DIM; k++) {
 augmented[j][k] -= factor * augmented[i][k];
 }
 }
 }
 }

 // Extract inverse from augmented matrix
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 Ainv[i][j] = augmented[i][j+MATRIX_DIM];
 }
 }

 return 1; // Success
}

// 3D Rotation matrices (trigonometric intensive)
void rotation_matrix_x(double angle, double R[MATRIX_DIM][MATRIX_DIM]) {
 // Initialize to identity
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 R[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Set rotation elements
 R[1][1] = cos(angle);
 R[1][2] = -sin(angle);
 R[2][1] = sin(angle);
 R[2][2] = cos(angle);
}

void rotation_matrix_y(double angle, double R[MATRIX_DIM][MATRIX_DIM]) {
 // Initialize to identity
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 R[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Set rotation elements
 R[0][0] = cos(angle);
 R[0][2] = sin(angle);
 R[2][0] = -sin(angle);
 R[2][2] = cos(angle);
}

void rotation_matrix_z(double angle, double R[MATRIX_DIM][MATRIX_DIM]) {
 // Initialize to identity
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 R[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 // Set rotation elements

 R[0][0] = cos(angle);
 R[0][1] = -sin(angle);
 R[1][0] = sin(angle);
 R[1][1] = cos(angle);
}

// Combined rotation matrix for arbitrary 3D rotation
void rotation_matrix_xyz(double roll, double pitch, double yaw, double R[MATRIX_DIM][MATRIX_DIM]) {
 double Rx[MATRIX_DIM][MATRIX_DIM], Ry[MATRIX_DIM][MATRIX_DIM], Rz[MATRIX_DIM][MATRIX_DIM];
 double temp[MATRIX_DIM][MATRIX_DIM];

 rotation_matrix_x(roll, Rx);
 rotation_matrix_y(pitch, Ry);
 rotation_matrix_z(yaw, Rz);

 // R = Rz * Ry * Rx
 matrix_multiply(Rz, Ry, temp);
 matrix_multiply(temp, Rx, R);
}

// Terrain and obstacle generation
void generate_random_terrain(TerrainMap *terrain, double max_height, double max_friction) {
 // Generate height field with some coherence
 for (int i = 0; i < TERRAIN_DIM; i++) {
 for (int j = 0; j < TERRAIN_DIM; j++) {
 // Use sine waves to create terrain features
 double x = (double)i / TERRAIN_DIM;
 double y = (double)j / TERRAIN_DIM;

 terrain->height[i][j] = max_height * (
 0.5 * sin(2.0 * PI * 2.0 * x) * sin(2.0 * PI * 2.0 * y) +
 0.25 * sin(2.0 * PI * 4.0 * x) * sin(2.0 * PI * 4.0 * y) +
 0.125 * sin(2.0 * PI * 8.0 * x) * sin(2.0 * PI * 8.0 * y)
 );

 // Randomize friction coefficient
 terrain->friction[i][j] = 0.5 * max_friction * (1.0 + sin(10.0 * x) * cos(10.0 * y));
 }
 }

 // Calculate elevation gradients (needed for path planning)
 for (int i = 1; i < TERRAIN_DIM-1; i++) {
 for (int j = 1; j < TERRAIN_DIM-1; j++) {
 // Central difference approximation of gradient
 terrain->elevation_gradient_x[i][j] = (terrain->height[i+1][j] - terrain->height[i-1][j]) / 2.0;
 terrain->elevation_gradient_y[i][j] = (terrain->height[i][j+1] - terrain->height[i][j-1]) / 2.0;
 }
 }

 // Handle boundaries: gradient_y on j-boundaries (forward/backward diff in j)
 for (int i = 0; i < TERRAIN_DIM; i++) {
 terrain->elevation_gradient_y[i][0] = terrain->height[i][1] - terrain->height[i][0];
 terrain->elevation_gradient_y[i][TERRAIN_DIM-1] = terrain->height[i][TERRAIN_DIM-1] - terrain->height[i][TERRAIN_DIM-2];
 }

 // Handle boundaries: gradient_x on i-boundaries (forward/backward diff in i)
 for (int j = 0; j < TERRAIN_DIM; j++) {
 terrain->elevation_gradient_x[0][j] = terrain->height[1][j] - terrain->height[0][j];
 terrain->elevation_gradient_x[TERRAIN_DIM-1][j] = terrain->height[TERRAIN_DIM-1][j] - terrain->height[TERRAIN_DIM-2][j];
 }

 // gradient_x on j-boundaries for interior rows (central diff in i)
 for (int i = 1; i < TERRAIN_DIM - 1; i++) {
 terrain->elevation_gradient_x[i][0] = (terrain->height[i+1][0] - terrain->height[i-1][0]) / 2.0;
 terrain->elevation_gradient_x[i][TERRAIN_DIM-1] = (terrain->height[i+1][TERRAIN_DIM-1] - terrain->height[i-1][TERRAIN_DIM-1]) / 2.0;
 }

 // gradient_y on i-boundaries for interior columns (central diff in j)
 for (int j = 1; j < TERRAIN_DIM - 1; j++) {
 terrain->elevation_gradient_y[0][j] = (terrain->height[0][j+1] - terrain->height[0][j-1]) / 2.0;
 terrain->elevation_gradient_y[TERRAIN_DIM-1][j] = (terrain->height[TERRAIN_DIM-1][j+1] - terrain->height[TERRAIN_DIM-1][j-1]) / 2.0;
 }
}

void generate_random_obstacles(Obstacle obstacles[], int *num_obstacles, int max_obstacles, double terrain_size) {
 *num_obstacles = max_obstacles;

 for (int i = 0; i < *num_obstacles; i++) {
 obstacles[i].position.x = ((double)rand() / RAND_MAX) * terrain_size;
 obstacles[i].position.y = ((double)rand() / RAND_MAX) * terrain_size;
 obstacles[i].position.z = 0.0; // Will be set based on terrain height
 obstacles[i].radius = 0.5 + ((double)rand() / RAND_MAX) * 2.0; // Random radius between 0.5 and 2.5

 // Some obstacles are dynamic (moving)
 obstacles[i].is_dynamic = ((double)rand() / RAND_MAX) < 0.2; // 20% chance of dynamic obstacle
 if (obstacles[i].is_dynamic) {
 obstacles[i].velocity = 0.1 + ((double)rand() / RAND_MAX) * 0.3; // Speed between 0.1 and 0.4
 obstacles[i].direction = ((double)rand() / RAND_MAX) * 2.0 * PI; // Random direction
 } else {
 obstacles[i].velocity = 0.0;
 obstacles[i].direction = 0.0;
 }
 }
}

// Update positions of dynamic obstacles
void update_dynamic_obstacles(Obstacle obstacles[], int num_obstacles, double dt, TerrainMap *terrain) {
 for (int i = 0; i < num_obstacles; i++) {
 if (obstacles[i].is_dynamic) {
 // Update position based on velocity and direction

 obstacles[i].position.x += obstacles[i].velocity * cos(obstacles[i].direction) * dt;
 obstacles[i].position.y += obstacles[i].velocity * sin(obstacles[i].direction) * dt;

 // Boundary check - bounce off terrain edges
 if (obstacles[i].position.x < 0 || obstacles[i].position.x >= TERRAIN_DIM) {
 obstacles[i].direction = PI - obstacles[i].direction; // Reflect horizontally
 // Ensure within bounds
 if (obstacles[i].position.x < 0) obstacles[i].position.x = 0;
 if (obstacles[i].position.x >= TERRAIN_DIM) obstacles[i].position.x = TERRAIN_DIM - 0.1;
 }

 if (obstacles[i].position.y < 0 || obstacles[i].position.y >= TERRAIN_DIM) {
 obstacles[i].direction = 2.0 * PI - obstacles[i].direction; // Reflect vertically
 // Ensure within bounds
 if (obstacles[i].position.y < 0) obstacles[i].position.y = 0;
 if (obstacles[i].position.y >= TERRAIN_DIM) obstacles[i].position.y = TERRAIN_DIM - 0.1;
 }

 // Occasionally change direction randomly
 if (((double)rand() / RAND_MAX) < 0.01) { // 1% chance per timestep
 obstacles[i].direction += (((double)rand() / RAND_MAX) * 2.0 - 1.0) * PI/4.0; // +/- 45 degrees
 }

 // Update z-coordinate based on terrain height
 int grid_x = (int)obstacles[i].position.x;
 int grid_y = (int)obstacles[i].position.y;

 // Clamp to terrain boundaries
 if (grid_x < 0) grid_x = 0;
 if (grid_x >= TERRAIN_DIM - 1) grid_x = TERRAIN_DIM - 2;
 if (grid_y < 0) grid_y = 0;
 if (grid_y >= TERRAIN_DIM - 1) grid_y = TERRAIN_DIM - 2;

 // Bilinear interpolation for height
 double fx = obstacles[i].position.x - grid_x;
 double fy = obstacles[i].position.y - grid_y;

 double h00 = terrain->height[grid_x][grid_y];
 double h10 = terrain->height[grid_x+1][grid_y];
 double h01 = terrain->height[grid_x][grid_y+1];
 double h11 = terrain->height[grid_x+1][grid_y+1];

 obstacles[i].position.z = h00 * (1-fx) * (1-fy) + h10 * fx * (1-fy) + h01 * (1-fx) * fy + h11 * fx * fy;
 }
 }
}

// ---- Image Processing BCE: Terrain image analysis (Sobel + Gaussian) ----
void processTerrainImage(TerrainMap *terrain) {
 double edge_map[TERRAIN_DIM][TERRAIN_DIM];
 double blurred[TERRAIN_DIM][TERRAIN_DIM];
 memset(edge_map, 0, sizeof(edge_map));
 memset(blurred, 0, sizeof(blurred));

 // Sobel edge detection on terrain height grid
 int Gx[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};
 int Gy[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};

 for (int i = 1; i < TERRAIN_DIM - 1; i++) {
 for (int j = 1; j < TERRAIN_DIM - 1; j++) {
 double sx = 0.0, sy = 0.0;
 for (int ki = -1; ki <= 1; ki++) {
 for (int kj = -1; kj <= 1; kj++) {
 double h = terrain->height[i+ki][j+kj];
 sx += h * Gx[ki+1][kj+1];
 sy += h * Gy[ki+1][kj+1];
 }
 }
 edge_map[i][j] = sqrt(sx*sx + sy*sy);
 }
 }

 // Gaussian blur (3x3 kernel) on edge map for noise reduction
 double gauss[3][3] = {{1.0/16,2.0/16,1.0/16},
 {2.0/16,4.0/16,2.0/16},
 {1.0/16,2.0/16,1.0/16}};
 for (int i = 1; i < TERRAIN_DIM - 1; i++) {
 for (int j = 1; j < TERRAIN_DIM - 1; j++) {
 double sum = 0.0;
 for (int ki = -1; ki <= 1; ki++)
 for (int kj = -1; kj <= 1; kj++)
 sum += edge_map[i+ki][j+kj] * gauss[ki+1][kj+1];
 blurred[i][j] = sum;
 }
 }

 // Feed edge information into terrain gradient fields for path planning
 for (int i = 1; i < TERRAIN_DIM - 1; i++) {
 for (int j = 1; j < TERRAIN_DIM - 1; j++) {
 terrain->elevation_gradient_x[i][j] += 0.1 * blurred[i][j];
 terrain->elevation_gradient_y[i][j] += 0.1 * blurred[i][j];
 }
 }
 FLIGHT_LOG(" Terrain image processing: edge detection + Gaussian blur complete\n");
}

// Potential field calculation for path planning
void calculate_potential_field(TerrainMap *terrain, Obstacle obstacles[], int num_obstacles,
 Point3D goal, double potential_field[TERRAIN_DIM][TERRAIN_DIM]) {
 double k_att = 1.0; // Attractive potential gain
 double k_rep = 100.0; // Repulsive potential gain
 double rep_radius = 5.0; // Influence radius of obstacles
 double terrain_factor = 10.0; // Weight of terrain gradient

 // Initialize potential field
 for (int i = 0; i < TERRAIN_DIM; i++) {
 for (int j = 0; j < TERRAIN_DIM; j++) {
 // Convert grid coordinates to world coordinates
 double x = (double)i;
 double y = (double)j;
 // Calculate attractive potential (goal)
 double dx = x - goal.x;
 double dy = y - goal.y;
 double dist_to_goal = sqrt(dx*dx + dy*dy);
 double attractive = k_att * dist_to_goal;

 // Calculate repulsive potential (obstacles)
 double repulsive = 0.0;
 for (int k = 0; k < num_obstacles; k++) {
 dx = x - obstacles[k].position.x;
 dy = y - obstacles[k].position.y;
 double dist_to_obs = sqrt(dx*dx + dy*dy);

 if (dist_to_obs < rep_radius) {
 if (dist_to_obs < 0.1) dist_to_obs = 0.1; // Avoid division by zero
 repulsive += k_rep * (1.0/dist_to_obs - 1.0/rep_radius) * (1.0/dist_to_obs);
 }
 }

 // Add terrain gradient factor (avoid steep slopes)
 double terrain_repulsive = terrain_factor * (
 fabs(terrain->elevation_gradient_x[i][j]) +
 fabs(terrain->elevation_gradient_y[i][j])
 );

 // Total potential is sum of attractive and repulsive components
 potential_field[i][j] = attractive + repulsive + terrain_repulsive;
 }
 }
}

// A* Path Planning Algorithm

/* Maximum nodes the A* search can expand. TERRAIN_DIM² covers the
 full grid; ×2 gives headroom for re-visits in the open set. */
#define ASTAR_NODE_POOL_CAP (TERRAIN_DIM * TERRAIN_DIM * 2)

typedef struct AStarNode {
 int x, y;
 double g; // Cost from start
 double h; // Heuristic (estimated cost to goal)
 double f; // Total cost (g + h)
 int parent_idx; // Index into astar_node_pool (-1 = none)
} AStarNode;

typedef struct {
 AStarNode *nodes;
 int size;
 int capacity;
} PriorityQueue;

/* ---- Static BSS pools (MISRA 21.3: no malloc/free after init) ---- */
static AStarNode pq_nodes_pool[ASTAR_NODE_POOL_CAP]; /* priority queue backing store */
static AStarNode astar_node_pool[ASTAR_NODE_POOL_CAP]; /* parent-chain pool */
static int astar_pool_idx; /* next free slot */

void priority_queue_init(PriorityQueue *pq, int capacity) {
 (void)capacity; /* ignored — uses static pool */
 pq->nodes = pq_nodes_pool;
 pq->size = 0;
 pq->capacity = ASTAR_NODE_POOL_CAP;
}

void priority_queue_push(PriorityQueue *pq, AStarNode node) {
 if (pq->size >= pq->capacity) {
 /* Static pool exhausted — silently drop (flight-safe) */
 return;
 }

 // Add node to end
 pq->nodes[pq->size] = node;

 // Bubble up to maintain heap property
 int i = pq->size;
 pq->size++;

 while (i > 0) {
 int parent = (i - 1) / 2;
 if (pq->nodes[i].f >= pq->nodes[parent].f) break;

 // Swap with parent
 AStarNode temp = pq->nodes[i];
 pq->nodes[i] = pq->nodes[parent];
 pq->nodes[parent] = temp;

 i = parent;
 }
}

AStarNode priority_queue_pop(PriorityQueue *pq) {
 AStarNode result = pq->nodes[0];

 // Move last element to root
 pq->nodes[0] = pq->nodes[pq->size - 1];
 pq->size--;

 // Bubble down to maintain heap property
 int i = 0;
 while (1) {
 int left = 2 * i + 1;
 int right = 2 * i + 2;
 int smallest = i;

 if (left < pq->size && pq->nodes[left].f < pq->nodes[smallest].f) {
 smallest = left;
 }

 if (right < pq->size && pq->nodes[right].f < pq->nodes[smallest].f) {
 smallest = right;
 }

 if (smallest == i) break;

 // Swap with smallest child
 AStarNode temp = pq->nodes[i];
 pq->nodes[i] = pq->nodes[smallest];
 pq->nodes[smallest] = temp;

 i = smallest;
 }

 return result;
}

bool priority_queue_empty(PriorityQueue *pq) {
 return pq->size == 0;
}

void priority_queue_free(PriorityQueue *pq) {
 /* Static pool — nothing to flight_free_impl(MISRA 21.3) */
 pq->nodes = NULL;
 pq->size = 0;
 pq->capacity = 0;
}

bool is_in_grid(int x, int y) {
 return x >= 0 && x < TERRAIN_DIM && y >= 0 && y < TERRAIN_DIM;
}

double heuristic(int x1, int y1, int x2, int y2) {
 // Euclidean distance
 return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

Path a_star_path_planning(TerrainMap *terrain, double potential_field[TERRAIN_DIM][TERRAIN_DIM],
 int start_x, int start_y, int goal_x, int goal_y) {
 Path path;
 path.num_waypoints = 0;

 // Initialize closed set
 bool closed_set[TERRAIN_DIM][TERRAIN_DIM];
 for (int i = 0; i < TERRAIN_DIM; i++) {
 for (int j = 0; j < TERRAIN_DIM; j++) {
 closed_set[i][j] = false;
 }
 }

 // Initialize priority queue
 PriorityQueue open_set;
 priority_queue_init(&open_set, TERRAIN_DIM * TERRAIN_DIM);

 // Create start node
 AStarNode start_node;
 start_node.x = start_x;
 start_node.y = start_y;
 start_node.g = 0.0;
 start_node.h = heuristic(start_x, start_y, goal_x, goal_y);
 start_node.f = start_node.g + start_node.h;
 start_node.parent_idx = -1; /* no parent */

 /* Reset static node pool for this search invocation */
 astar_pool_idx = 0;

 priority_queue_push(&open_set, start_node);

 // Direction vectors
 int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
 int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

 // A* search
 bool found_path = false;
 int goal_pool_idx = -1;

 while (!priority_queue_empty(&open_set)) {
 AStarNode current = priority_queue_pop(&open_set);

 // Check if we reached the goal
 if (current.x == goal_x && current.y == goal_y) {
 found_path = true;
 /* Store goal node in pool */
 if (astar_pool_idx < ASTAR_NODE_POOL_CAP) {
 astar_node_pool[astar_pool_idx] = current;
 goal_pool_idx = astar_pool_idx;
 astar_pool_idx++;
 }
 break;
 }

 // Mark as visited
 closed_set[current.x][current.y] = true;

 /* Store current node in pool so children can reference it */
 int cur_pool_idx = -1;
 if (astar_pool_idx < ASTAR_NODE_POOL_CAP) {
 astar_node_pool[astar_pool_idx] = current;
 cur_pool_idx = astar_pool_idx;
 astar_pool_idx++;
 }

 // Check all neighbors
 for (int i = 0; i < 8; i++) {
 int nx = current.x + dx[i];
 int ny = current.y + dy[i];

 // Check bounds
 if (!is_in_grid(nx, ny)) continue;

 // Check if already visited
 if (closed_set[nx][ny]) continue;

 // Calculate movement cost (include potential field)
 double movement_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0; // Diagonal moves cost more

 // Add terrain difficulty and potential field
 movement_cost += potential_field[nx][ny] / 100.0;

 // Slope penalty
 double slope_x = terrain->elevation_gradient_x[nx][ny];
 double slope_y = terrain->elevation_gradient_y[nx][ny];
 double slope_magnitude = sqrt(slope_x*slope_x + slope_y*slope_y);
 movement_cost += slope_magnitude * 2.0;

 // Create neighbor node
 AStarNode neighbor;
 neighbor.x = nx;
 neighbor.y = ny;
 neighbor.g = current.g + movement_cost;
 neighbor.h = heuristic(nx, ny, goal_x, goal_y);
 neighbor.f = neighbor.g + neighbor.h;
 neighbor.parent_idx = cur_pool_idx; /* index into static pool */

 // Add to open set
 priority_queue_push(&open_set, neighbor);
 }
 }

 // Reconstruct path if found
 if (found_path && goal_pool_idx >= 0) {
 // Count waypoints by walking the parent-index chain
 int count = 0;
 int idx = goal_pool_idx;
 while (idx >= 0) {
 count++;
 idx = astar_node_pool[idx].parent_idx;
 }

 // Allocate path
 path.num_waypoints = count;
 if (path.num_waypoints > MAX_PATH_POINTS) {
 path.num_waypoints = MAX_PATH_POINTS;
 }

 // Fill path (backwards)
 idx = goal_pool_idx;
 for (int i = path.num_waypoints - 1; i >= 0 && idx >= 0; i--) {
 path.waypoints[i].x = astar_node_pool[idx].x;
 path.waypoints[i].y = astar_node_pool[idx].y;
 path.waypoints[i].z = terrain->height[astar_node_pool[idx].x][astar_node_pool[idx].y];
 idx = astar_node_pool[idx].parent_idx;
 }
 /* No heap free needed — static pool (MISRA 21.3) */
 }

 // Clean up
 priority_queue_free(&open_set);

 return path;
}

// Smooth the path using cubic spline interpolation
Path smooth_path(Path original_path, TerrainMap *terrain) {
 (void)terrain; // terrain grid not needed for parametric spline smoothing
 Path smoothed_path;

 if (original_path.num_waypoints < 2) {
 return original_path; // Can't smooth with fewer than 2 points
 }

 // Calculate parametric spline coefficients (trigonometry intensive)
 int n = original_path.num_waypoints;
 /* Static spline buffers — n ≤ MAX_PATH_POINTS (MISRA 21.3) */
 static double spl_t[MAX_PATH_POINTS];
 static double spl_x[MAX_PATH_POINTS];
 static double spl_y[MAX_PATH_POINTS];
 static double spl_z[MAX_PATH_POINTS];
 double *t = spl_t;
 double *x = spl_x;
 double *y = spl_y;
 double *z = spl_z;

 // Use chord-length parameterization
 t[0] = 0.0;
 for (int i = 1; i < n; i++) {
 double dx = original_path.waypoints[i].x - original_path.waypoints[i-1].x;
 double dy = original_path.waypoints[i].y - original_path.waypoints[i-1].y;
 double dz = original_path.waypoints[i].z - original_path.waypoints[i-1].z;

 t[i] = t[i-1] + sqrt(dx*dx + dy*dy + dz*dz);
 }

 // Normalize parameter to [0, 1]
 if (t[n-1] > 0.0) {
 for (int i = 0; i < n; i++) {
 t[i] /= t[n-1];
 }
 }

 // Extract coordinates
 for (int i = 0; i < n; i++) {
 x[i] = original_path.waypoints[i].x;
 y[i] = original_path.waypoints[i].y;
 z[i] = original_path.waypoints[i].z;
 }

 // Cubic spline interpolation
 int num_points = MAX_PATH_POINTS;
 smoothed_path.num_waypoints = num_points;

 for (int i = 0; i < num_points; i++) {
 double param = (double)i / (num_points - 1);

 // Find spline segment
 int segment = 0;
 while (segment < n - 1 && t[segment+1] < param) {
 segment++;
 }

 // Compute local parameter (guard against zero-length segments)
 double seg_len = t[segment+1] - t[segment];
 if (seg_len < 1e-12) seg_len = 1e-12;
 double local_t = (param - t[segment]) / seg_len;

 // Hermite basis functions (trigonometric intensive)
 double h1 = 2.0 * pow(local_t, 3) - 3.0 * pow(local_t, 2) + 1.0;
 double h2 = -2.0 * pow(local_t, 3) + 3.0 * pow(local_t, 2);
 double h3 = pow(local_t, 3) - 2.0 * pow(local_t, 2) + local_t;
 double h4 = pow(local_t, 3) - pow(local_t, 2);

 // Calculate tangents (finite differences with zero-guard on denominators)
 double tx1, ty1, tz1, tx2, ty2, tz2;
 double denom;

 if (segment == 0) {
 denom = t[1] - t[0];
 if (denom < 1e-12) denom = 1e-12;
 tx1 = (x[1] - x[0]) / denom;
 ty1 = (y[1] - y[0]) / denom;
 tz1 = (z[1] - z[0]) / denom;
 } else {
 denom = t[segment+1] - t[segment-1];
 if (fabs(denom) < 1e-12) denom = 1e-12;
 tx1 = (x[segment+1] - x[segment-1]) / denom;
 ty1 = (y[segment+1] - y[segment-1]) / denom;
 tz1 = (z[segment+1] - z[segment-1]) / denom;
 }

 if (segment == n - 2) {
 denom = t[n-1] - t[n-2];
 if (denom < 1e-12) denom = 1e-12;
 tx2 = (x[n-1] - x[n-2]) / denom;
 ty2 = (y[n-1] - y[n-2]) / denom;
 tz2 = (z[n-1] - z[n-2]) / denom;
 } else {
 denom = t[segment+2] - t[segment];
 if (fabs(denom) < 1e-12) denom = 1e-12;
 tx2 = (x[segment+2] - x[segment]) / denom;
 ty2 = (y[segment+2] - y[segment]) / denom;
 tz2 = (z[segment+2] - z[segment]) / denom;
 }

 // Scale tangents by segment length
 double scale = t[segment+1] - t[segment];
 tx1 *= scale;
 ty1 *= scale;
 tz1 *= scale;
 tx2 *= scale;
 ty2 *= scale;
 tz2 *= scale;

 // Interpolate position using cubic Hermite spline
 smoothed_path.waypoints[i].x = h1 * x[segment] + h2 * x[segment+1] + h3 * tx1 + h4 * tx2;
 smoothed_path.waypoints[i].y = h1 * y[segment] + h2 * y[segment+1] + h3 * ty1 + h4 * ty2;
 smoothed_path.waypoints[i].z = h1 * z[segment] + h2 * z[segment+1] + h3 * tz1 + h4 * tz2;
 }

 /* Static pools — no free needed (MISRA 21.3) */

 return smoothed_path;
}

// Collision detection between rover and obstacles
bool check_collision(RoverState rover, Obstacle obstacles[], int num_obstacles, double rover_radius) {
 for (int i = 0; i < num_obstacles; i++) {
 double dx = rover.x - obstacles[i].position.x;
 double dy = rover.y - obstacles[i].position.y;

 double distance = sqrt(dx*dx + dy*dy);

 if (distance < (rover_radius + obstacles[i].radius)) {
 return true; // Collision detected
 }
 }

 return false; // No collision
}

// Local collision avoidance using Vector Field Histogram (VFH)
double vfh_collision_avoidance(RoverState rover, Point3D goal,
 Obstacle obstacles[], int num_obstacles,
 double rover_radius, double *steering_angle) {
 (void)rover_radius; // VFH uses detection_radius, not physical rover radius
 // VFH parameters
 const int num_sectors = 36;
 const double sector_angle = 2.0 * PI / num_sectors;
 const double detection_radius = 10.0;
 const double max_steer_angle = PI / 4.0; // 45 degrees

 // Calculate binary histogram
 double histogram[num_sectors];
 for (int i = 0; i < num_sectors; i++) {
 histogram[i] = 0.0;
 }

 // Populate histogram with obstacle detections
 for (int i = 0; i < num_obstacles; i++) {
 double dx = obstacles[i].position.x - rover.x;
 double dy = obstacles[i].position.y - rover.y;

 double distance = sqrt(dx*dx + dy*dy);

 if (distance < detection_radius) {
 // Calculate angle to obstacle
 double angle = atan2(dy, dx);
 if (angle < 0) angle += 2.0 * PI;

 // Determine sector
 int sector = (int)(angle / sector_angle);
 if (sector >= num_sectors) sector = 0;

 // Add obstacle influence to histogram (closer obstacles have more influence)
 double influence = 1.0 - (distance / detection_radius);
 influence = pow(influence, 2); // Square for non-linear falloff

 // Add influence to sector and adjacent sectors
 for (int j = -2; j <= 2; j++) {
 int s = sector + j;
 if (s < 0) s += num_sectors;
 if (s >= num_sectors) s -= num_sectors;

 // Scale influence by distance from center sector
 double scale = 1.0 - 0.3 * abs(j);
 if (scale < 0) scale = 0;

 histogram[s] += influence * scale;

 // Cap at 1.0
 if (histogram[s] > 1.0) histogram[s] = 1.0;
 }
 }
 }

 // Calculate goal direction
 double goal_dx = goal.x - rover.x;
 double goal_dy = goal.y - rover.y;

 double goal_angle = atan2(goal_dy, goal_dx);

 if (goal_angle < 0) goal_angle += 2.0 * PI;

 // Normalize rover heading to [0, 2*PI]
 double rover_heading = rover.theta;
 while (rover_heading < 0) rover_heading += 2.0 * PI;
 while (rover_heading >= 2.0 * PI) rover_heading -= 2.0 * PI;

 // Find best direction in the histogram
 double min_cost = INFINITY;
 double best_angle = rover_heading; // Default to current heading

 for (int i = 0; i < num_sectors; i++) {
 double sector_angle_mid = i * sector_angle + sector_angle / 2.0;

 // Skip sectors with high obstacle density
 if (histogram[i] > 0.7) continue;

 // Calculate costs
 double obstacle_cost = histogram[i];

 // Calculate angle difference to goal (normalized to [-PI, PI])
 double angle_diff = sector_angle_mid - goal_angle;
 if (angle_diff > PI) angle_diff -= 2.0 * PI;
 if (angle_diff < -PI) angle_diff += 2.0 * PI;
 double goal_cost = fabs(angle_diff) / PI; // Normalize to [0, 1]

 // Calculate angle difference to current heading (for smooth steering)
 angle_diff = sector_angle_mid - rover_heading;
 if (angle_diff > PI) angle_diff -= 2.0 * PI;
 if (angle_diff < -PI) angle_diff += 2.0 * PI;
 double heading_cost = fabs(angle_diff) / PI; // Normalize to [0, 1]

 // Combine costs (weights can be tuned)
 double total_cost = 0.7 * obstacle_cost + 0.2 * goal_cost + 0.1 * heading_cost;

 if (total_cost < min_cost) {
 min_cost = total_cost;
 best_angle = sector_angle_mid;
 }
 }

 // Calculate steering angle (difference between best angle and current heading)
 double steering = best_angle - rover_heading;
 if (steering > PI) steering -= 2.0 * PI;
 if (steering < -PI) steering += 2.0 * PI;

 // Limit steering angle
 if (steering > max_steer_angle) steering = max_steer_angle;
 if (steering < -max_steer_angle) steering = -max_steer_angle;

 *steering_angle = steering;

 // Calculate velocity based on obstacle density in forward direction
 int forward_sector = (int)(rover_heading / sector_angle);
 if (forward_sector >= num_sectors) forward_sector = 0;

 // Average density in forward sectors
 double forward_density = 0.0;
 for (int i = -1; i <= 1; i++) {
 int s = forward_sector + i;
 if (s < 0) s += num_sectors;
 if (s >= num_sectors) s -= num_sectors;
 forward_density += histogram[s];
 }
 forward_density /= 3.0;

 // Velocity decreases with obstacle density
 double velocity = 1.0 - 0.7 * forward_density;
 if (velocity < 0.2) velocity = 0.2; // Minimum velocity

 return velocity;
}

// Forward kinematics and motion model
void update_rover_state(RoverState *rover, double dt, double steering_angle, double velocity,
 TerrainMap *terrain) {
 // Extract current state
 double x = rover->x;
 double y = rover->y;
 double theta = rover->theta;

 // Calculate terrain height and slope at current position
 int grid_x = (int)x;
 int grid_y = (int)y;

 // Clamp to terrain boundaries
 if (grid_x < 0) grid_x = 0;
 if (grid_x >= TERRAIN_DIM - 1) grid_x = TERRAIN_DIM - 2;
 if (grid_y < 0) grid_y = 0;
 if (grid_y >= TERRAIN_DIM - 1) grid_y = TERRAIN_DIM - 2;

 // Bilinear interpolation for height and slope
 double fx = x - grid_x;
 double fy = y - grid_y;

 double h00 = terrain->height[grid_x][grid_y];
 double h10 = terrain->height[grid_x+1][grid_y];
 double h01 = terrain->height[grid_x][grid_y+1];
 double h11 = terrain->height[grid_x+1][grid_y+1];

 // Interpolated height at rover position (used for telemetry / logging)
 (void)(h00 * (1-fx) * (1-fy) + h10 * fx * (1-fy) + h01 * (1-fx) * fy + h11 * fx * fy);

 double gx00 = terrain->elevation_gradient_x[grid_x][grid_y];
 double gx10 = terrain->elevation_gradient_x[grid_x+1][grid_y];
 double gx01 = terrain->elevation_gradient_x[grid_x][grid_y+1];
 double gx11 = terrain->elevation_gradient_x[grid_x+1][grid_y+1];

 double gy00 = terrain->elevation_gradient_y[grid_x][grid_y];
 double gy10 = terrain->elevation_gradient_y[grid_x+1][grid_y];
 double gy01 = terrain->elevation_gradient_y[grid_x][grid_y+1];
 double gy11 = terrain->elevation_gradient_y[grid_x+1][grid_y+1];

 double slope_x = gx00 * (1-fx) * (1-fy) + gx10 * fx * (1-fy) + gx01 * (1-fx) * fy + gx11 * fx * fy;
 double slope_y = gy00 * (1-fx) * (1-fy) + gy10 * fx * (1-fy) + gy01 * (1-fx) * fy + gy11 * fx * fy;

 // Adjust velocity based on slope
 double heading_vec[2] = {cos(theta), sin(theta)};
 double slope_vec[2] = {slope_x, slope_y};

 // Dot product to find slope component along heading
 double slope_along_heading = heading_vec[0] * slope_vec[0] + heading_vec[1] * slope_vec[1];

 // Reduce velocity when going uphill, increase when going downhill
 double velocity_factor = 1.0 - 0.5 * slope_along_heading;
 if (velocity_factor < 0.3) velocity_factor = 0.3; // Minimum 30% of desired velocity
 if (velocity_factor > 1.5) velocity_factor = 1.5; // Maximum 150% of desired velocity

 velocity *= velocity_factor;

 // Tricycle kinematic model (front steering)
 const double wheelbase = 1.0; // Distance between front and rear axles

 // Calculate angular velocity based on steering angle
 double angular_velocity = (velocity / wheelbase) * tan(steering_angle);

 // Update state using Euler integration (could use RK4 for better accuracy)
 rover->x += velocity * cos(theta) * dt;
 rover->y += velocity * sin(theta) * dt;

 rover->theta += angular_velocity * dt;
 rover->velocity = velocity;
 rover->angular_velocity = angular_velocity;

 // Normalize angle
 while (rover->theta > PI) rover->theta -= 2.0 * PI;
 while (rover->theta < -PI) rover->theta += 2.0 * PI;
}

// Main simulation function with dynamic replanning
void simulate_rover_mission(TerrainMap *terrain, RoverState *rover, Path *path,
 Obstacle obstacles[], int num_obstacles) {
 const double dt = 0.1; // Time step (seconds)
 const double rover_radius = 1.0;
 const double goal_threshold = 1.0; // Distance to consider goal reached
 const double replan_safety_distance = 2.5; // Distance to stay away from obstacles when replanning
 const int max_steps = 1000; // Maximum simulation steps
 const int max_replans = 10; // Maximum number of replans allowed
 const double prediction_horizon = 3.0; // Time horizon for predicting obstacle movements

 int current_waypoint = 0;
 int replan_count = 0;
 Point3D final_goal;

 // Save the final goal point
 if (path->num_waypoints > 0) {
 final_goal = path->waypoints[path->num_waypoints - 1];
 } else {
 FLIGHT_LOG("Empty path provided, cannot proceed.\n");
 return;
 }

 // Initialize rover orientation to face first waypoint
 if (path->num_waypoints > 0) {
 double dx = path->waypoints[0].x - rover->x;
 double dy = path->waypoints[0].y - rover->y;
 rover->theta = atan2(dy, dx);
 }

 // Main simulation loop
 for (int step = 0; step < max_steps; step++) {
 // Update dynamic obstacles
 update_dynamic_obstacles(obstacles, num_obstacles, dt, terrain);

 // Check if we've reached the final waypoint
 if (current_waypoint >= path->num_waypoints) {
 FLIGHT_LOG("Mission complete! Reached final waypoint in %d steps.\n", step);
 break;
 }

 // Current goal is the active waypoint
 Point3D goal = path->waypoints[current_waypoint];

 // Check if waypoint reached
 double dx = goal.x - rover->x;
 double dy = goal.y - rover->y;

 double distance_to_goal = sqrt(dx*dx + dy*dy);

 if (distance_to_goal < goal_threshold) {
 FLIGHT_LOG("Reached waypoint %d at (%.2f, %.2f)\n", current_waypoint, goal.x, goal.y);
 current_waypoint++;
 continue;
 }

 // Predict future positions of dynamic obstacles for collision avoidance
 Obstacle predicted_obstacles[MAX_OBSTACLES];
 for (int i = 0; i < num_obstacles; i++) {
 predicted_obstacles[i] = obstacles[i];

 // Predict future position of dynamic obstacles
 if (obstacles[i].is_dynamic) {
 // Simple linear prediction based on current velocity and direction

 predicted_obstacles[i].position.x += obstacles[i].velocity * cos(obstacles[i].direction) * prediction_horizon;
 predicted_obstacles[i].position.y += obstacles[i].velocity * sin(obstacles[i].direction) * prediction_horizon;

 // Clamp predicted position to terrain boundaries
 if (predicted_obstacles[i].position.x < 0) predicted_obstacles[i].position.x = 0;
 if (predicted_obstacles[i].position.x >= TERRAIN_DIM) predicted_obstacles[i].position.x = TERRAIN_DIM - 0.1;
 if (predicted_obstacles[i].position.y < 0) predicted_obstacles[i].position.y = 0;
 if (predicted_obstacles[i].position.y >= TERRAIN_DIM) predicted_obstacles[i].position.y = TERRAIN_DIM - 0.1;

 // Increase radius for safety margin around predicted position
 predicted_obstacles[i].radius += obstacles[i].velocity * prediction_horizon * 0.5;
 }
 }

 // Local collision avoidance using predicted obstacle positions
 double steering_angle;
 double velocity = vfh_collision_avoidance(*rover, goal, predicted_obstacles, num_obstacles, rover_radius, &steering_angle);

 // Update rover state
 update_rover_state(rover, dt, steering_angle, velocity, terrain);

 // Check for collisions or near-collisions with current obstacles
 bool collision = check_collision(*rover, obstacles, num_obstacles, rover_radius);

 // Check for near-collisions (for early replanning)
 bool near_collision = false;
 int closest_obstacle_idx = -1;
 double min_distance = INFINITY;

 for (int i = 0; i < num_obstacles; i++) {
 double dx = rover->x - obstacles[i].position.x;
 double dy = rover->y - obstacles[i].position.y;
 double distance = sqrt(dx*dx + dy*dy);

 // Track closest obstacle
 if (distance < min_distance) {
 min_distance = distance;
 closest_obstacle_idx = i;
 }

 // Check for near collision
 if (distance < (rover_radius + obstacles[i].radius + 0.8)) { // 0.8 meter safety margin
 near_collision = true;

 // If this is a dynamic obstacle, predict if it will move away
 if (obstacles[i].is_dynamic) {
 // Calculate relative angle between obstacle movement and obstacle-to-rover vector

 double obstacle_to_rover_angle = atan2(rover->y - obstacles[i].position.y,
 rover->x - obstacles[i].position.x);

 double angle_diff = fabs(obstacle_to_rover_angle - obstacles[i].direction);
 while (angle_diff > PI) angle_diff = 2.0 * PI - angle_diff;

 // If obstacle is moving away from rover (angle > 90 degrees), don't trigger replanning yet
 if (angle_diff > PI/2.0) {
 near_collision = false;
 }
 }

 if (near_collision) break;
 }
 }

 // Replan if collision or near-collision detected
 if (collision || (near_collision && velocity < 0.3)) { // Trigger replan if very slow due to obstacles
 if (replan_count >= max_replans) {
 FLIGHT_LOG("Too many replanning attempts (%d). Mission failed at step %d.\n", replan_count, step);
 break;
 }

 replan_count++;
 FLIGHT_LOG("Collision risk detected at step %d! Replanning path (attempt %d)...\n", step, replan_count);

 // Back up slightly to avoid immediate collision
 if (collision) {
 // Move the rover back a bit to get out of collision state
 rover->x -= cos(rover->theta) * 0.5;
 rover->y -= sin(rover->theta) * 0.5;
 FLIGHT_LOG("Backing up to position (%.2f, %.2f)\n", rover->x, rover->y);
 }

 // Create temporary obstacles with expanded safety margins
 Obstacle temp_obstacles[MAX_OBSTACLES];
 for (int i = 0; i < num_obstacles; i++) {
 temp_obstacles[i] = obstacles[i];

 // Add extra safety margin for dynamic obstacles based on their velocity
 if (obstacles[i].is_dynamic) {
 temp_obstacles[i].radius += replan_safety_distance + obstacles[i].velocity * 2.0;
 } else {
 temp_obstacles[i].radius += replan_safety_distance;
 }
 }

 // Calculate new potential field with expanded obstacles
 double potential_field[TERRAIN_DIM][TERRAIN_DIM];
 calculate_potential_field(terrain, temp_obstacles, num_obstacles, final_goal, potential_field);

 // Plan new path from current position to final goal
 int current_x = (int)rover->x;
 int current_y = (int)rover->y;
 int goal_x = (int)final_goal.x;
 int goal_y = (int)final_goal.y;

 // Clamp coordinates to grid boundaries
 if (current_x < 0) current_x = 0;
 if (current_x >= TERRAIN_DIM) current_x = TERRAIN_DIM - 1;
 if (current_y < 0) current_y = 0;
 if (current_y >= TERRAIN_DIM) current_y = TERRAIN_DIM - 1;

 if (goal_x < 0) goal_x = 0;
 if (goal_x >= TERRAIN_DIM) goal_x = TERRAIN_DIM - 1;
 if (goal_y < 0) goal_y = 0;
 if (goal_y >= TERRAIN_DIM) goal_y = TERRAIN_DIM - 1;

 Path new_path = a_star_path_planning(terrain, potential_field, current_x, current_y, goal_x, goal_y);

 // Check if path finding was successful
 if (new_path.num_waypoints > 0) {
 // Smooth the new path
 Path smooth_new_path = smooth_path(new_path, terrain);

 // Replace old path with new path
 *path = smooth_new_path;
 current_waypoint = 0;

 FLIGHT_LOG("New path planned with %d waypoints.\n", path->num_waypoints);
 } else {
 FLIGHT_LOG("Failed to find new path! Trying with reduced safety margins...\n");

 // Try again with smaller safety margins
 for (int i = 0; i < num_obstacles; i++) {
 temp_obstacles[i] = obstacles[i];
 temp_obstacles[i].radius += 1.0; // Smaller safety margin
 }

 calculate_potential_field(terrain, temp_obstacles, num_obstacles, final_goal, potential_field);
 new_path = a_star_path_planning(terrain, potential_field, current_x, current_y, goal_x, goal_y);

 if (new_path.num_waypoints > 0) {
 // Smooth the new path
 Path smooth_new_path = smooth_path(new_path, terrain);

 // Replace old path with new path
 *path = smooth_new_path;
 current_waypoint = 0;

 FLIGHT_LOG("New path planned with reduced safety margins: %d waypoints.\n", path->num_waypoints);
 } else {
 FLIGHT_LOG("Still failed! Executing emergency maneuver...\n");

 // Emergency strategy: Try to move away from the closest obstacle
 if (closest_obstacle_idx >= 0) {
 // Turn away from the closest obstacle (opposite direction)

 double escape_angle = atan2(
 rover->y - obstacles[closest_obstacle_idx].position.y,
 rover->x - obstacles[closest_obstacle_idx].position.x
 );

 rover->theta = escape_angle;

 // Move away from obstacle directly
 rover->x += cos(escape_angle) * 2.0;
 rover->y += sin(escape_angle) * 2.0;

 FLIGHT_LOG("Emergency evasion: moved to (%.2f, %.2f)\n", rover->x, rover->y);

 // Try to replan from this new position in the next step
 }
 }
 }

 // Continue to next iteration with new path
 continue;
 }

 // You could add logging or visualization here
 if (step % 10 == 0) {
 FLIGHT_LOG("Step %d: Rover at (%.2f, %.2f), heading: %.2f rad, velocity: %.2f, steering: %.2f\n",
 step, rover->x, rover->y, rover->theta, rover->velocity, steering_angle);
 }
 }
}


void plan_and_execute_mission(TerrainMap *terrain, Obstacle obstacles[], int num_obstacles,
 Point3D start_point, Point3D goal_point) {
 FLIGHT_LOG("Planning mission from (%.2f, %.2f) to (%.2f, %.2f)\n",
 start_point.x, start_point.y, goal_point.x, goal_point.y);

 // Terrain image processing: edge detection + blur for obstacle awareness
 processTerrainImage(terrain);

 // Spectral terrain roughness analysis — compute traversability cost modifier
 // from power spectral density of terrain elevation profiles along X and Y axes
 double spectral_cost[TERRAIN_DIM][TERRAIN_DIM];
 memset(spectral_cost, 0, sizeof(spectral_cost));
 for (int row = 0; row < TERRAIN_DIM; row++) {
 double roughness = terrain_spectral_analysis(terrain->height[row], TERRAIN_DIM);
 for (int col = 0; col < TERRAIN_DIM; col++) {
 spectral_cost[row][col] += roughness;
 }
 }
 for (int col = 0; col < TERRAIN_DIM; col++) {
 double col_profile[TERRAIN_DIM];
 for (int row = 0; row < TERRAIN_DIM; row++) {
 col_profile[row] = terrain->height[row][col];
 }
 double roughness = terrain_spectral_analysis(col_profile, TERRAIN_DIM);
 for (int row = 0; row < TERRAIN_DIM; row++) {
 spectral_cost[row][col] += roughness;
 }
 }
 // Fold spectral roughness into terrain friction (higher roughness = higher cost)
 double max_spectral = 0.0;
 for (int i = 0; i < TERRAIN_DIM; i++)
 for (int j = 0; j < TERRAIN_DIM; j++)
 if (spectral_cost[i][j] > max_spectral) max_spectral = spectral_cost[i][j];
 if (max_spectral > 0.0) {
 for (int i = 0; i < TERRAIN_DIM; i++)
 for (int j = 0; j < TERRAIN_DIM; j++)
 terrain->friction[i][j] += 0.3 * (spectral_cost[i][j] / max_spectral);
 }
 FLIGHT_LOG("Spectral terrain roughness analysis complete (max roughness=%.4f).\n", max_spectral);

 // Calculate potential field for path planning
 double potential_field[TERRAIN_DIM][TERRAIN_DIM];

 // Use a safety margin for initial path planning to avoid obstacles
 Obstacle planning_obstacles[MAX_OBSTACLES];
 for (int i = 0; i < num_obstacles; i++) {
 planning_obstacles[i] = obstacles[i];
 planning_obstacles[i].radius += 1.0; // Add safety margin for initial planning
 }

 calculate_potential_field(terrain, planning_obstacles, num_obstacles, goal_point, potential_field);

 // Plan path using A*
 int start_x = (int)start_point.x;
 int start_y = (int)start_point.y;
 int goal_x = (int)goal_point.x;
 int goal_y = (int)goal_point.y;

 // Clamp coordinates to grid boundaries
 if (start_x < 0) start_x = 0;
 if (start_x >= TERRAIN_DIM) start_x = TERRAIN_DIM - 1;
 if (start_y < 0) start_y = 0;
 if (start_y >= TERRAIN_DIM) start_y = TERRAIN_DIM - 1;

 if (goal_x < 0) goal_x = 0;
 if (goal_x >= TERRAIN_DIM) goal_x = TERRAIN_DIM - 1;
 if (goal_y < 0) goal_y = 0;
 if (goal_y >= TERRAIN_DIM) goal_y = TERRAIN_DIM - 1;

 Path raw_path = a_star_path_planning(terrain, potential_field, start_x, start_y, goal_x, goal_y);

 // Check if path planning was successful
 if (raw_path.num_waypoints == 0) {
 FLIGHT_LOG("Failed to find initial path. Trying with reduced obstacle safety margins.\n");

 // Retry with original obstacle sizes
 calculate_potential_field(terrain, obstacles, num_obstacles, goal_point, potential_field);
 raw_path = a_star_path_planning(terrain, potential_field, start_x, start_y, goal_x, goal_y);

 if (raw_path.num_waypoints == 0) {
 FLIGHT_LOG("Still failed to find path. Mission cannot be executed.\n");
 return;
 }
 }

 // Smooth the path
 Path smooth_path_result = smooth_path(raw_path, terrain);

 // Initialize rover
 RoverState rover;
 rover.x = start_point.x;
 rover.y = start_point.y;
 rover.theta = 0.0;
 rover.velocity = 0.0;
 rover.angular_velocity = 0.0;

 // Execute mission with dynamic replanning capability
 simulate_rover_mission(terrain, &rover, &smooth_path_result, obstacles, num_obstacles);
}
// Pre-compute navigation state covariance and terrain frame parameters
// These matrices are updated at each planning cycle to maintain localization accuracy
void execute_navigation_computations() {
 FLIGHT_LOG("\n[RPP] nav_estimate\n");

 // State covariance (P) and process noise cross-correlation (Q) matrices
 double P[MATRIX_DIM][MATRIX_DIM], Q[MATRIX_DIM][MATRIX_DIM];
 double P_predicted[MATRIX_DIM][MATRIX_DIM];
 double P_inv[MATRIX_DIM][MATRIX_DIM];

 // Initialize state covariance from wheel odometry and IMU measurements
 for (int i = 0; i < MATRIX_DIM; i++) {
 for (int j = 0; j < MATRIX_DIM; j++) {
 P[i][j] = sin(i * 0.1) * cos(j * 0.2) + 1.5;
 Q[i][j] = cos(i * 0.3) * sin(j * 0.1) + 1.2;
 }
 }

 // Propagate state covariance: P_predicted = P * Q (single EKF prediction step)
 FLIGHT_LOG("Propagating state covariance P_predicted = P * Q (%dx%d)...\n", MATRIX_DIM, MATRIX_DIM);
 matrix_multiply(P, Q, P_predicted);

 // Compute information matrix (inverse covariance) for measurement fusion
 FLIGHT_LOG("Computing information matrix P_inv = inv(P_predicted) (%dx%d)...\n", MATRIX_DIM, MATRIX_DIM);
 matrix_inverse(P_predicted, P_inv);

 // Verify information matrix: trace should be positive for valid covariance
 double info_trace = 0.0;
 for (int i = 0; i < MATRIX_DIM; i++) {
 info_trace += P_inv[i][i];
 }
 FLIGHT_LOG("Information matrix trace: %.6f\n", info_trace);

 // Cross-validate covariance via transpose path:
 // P_predicted_T = Q^T * P^T should equal (P * Q)^T
 double PT[MATRIX_DIM][MATRIX_DIM], QT[MATRIX_DIM][MATRIX_DIM];
 double P_pred_T[MATRIX_DIM][MATRIX_DIM];
 matrix_transpose(P, PT);
 matrix_transpose(Q, QT);
 matrix_multiply(QT, PT, P_pred_T);

 // Covariance update: P_updated = P_predicted + Q (process noise injection)
 double P_updated[MATRIX_DIM][MATRIX_DIM];
 matrix_add(P_predicted, Q, P_updated);

 // Innovation covariance: P_innovation = P_updated - P (deviation from prior)
 double P_innovation[MATRIX_DIM][MATRIX_DIM];
 matrix_subtract(P_updated, P, P_innovation);

 // Project state vector through innovation covariance
 double state_vec[MATRIX_DIM];
 for (int i = 0; i < MATRIX_DIM; i++) state_vec[i] = sin(i * 0.4) + 0.5;
 double projected[MATRIX_DIM];
 matrix_vector_multiply(P_innovation, state_vec, projected);

 // State energy: dot product of projected state with itself
 double state_energy = vector_dot_product(projected, projected);
 FLIGHT_LOG("State energy after innovation projection: %.6f\n", state_energy);

 // Generate candidate waypoints from estimated rover trajectory
 Path candidate_path;
 candidate_path.num_waypoints = 15;
 for (int i = 0; i < candidate_path.num_waypoints; i++) {
 candidate_path.waypoints[i].x = i * 1.2;
 candidate_path.waypoints[i].y = sin(i * 0.5) * 5;
 candidate_path.waypoints[i].z = cos(i * 0.3) * 2;
 }

 // Compute local terrain normal from first three waypoints (cross product)
 double disp1[3] = {
 candidate_path.waypoints[1].x - candidate_path.waypoints[0].x,
 candidate_path.waypoints[1].y - candidate_path.waypoints[0].y,
 candidate_path.waypoints[1].z - candidate_path.waypoints[0].z
 };
 double disp2[3] = {
 candidate_path.waypoints[2].x - candidate_path.waypoints[0].x,
 candidate_path.waypoints[2].y - candidate_path.waypoints[0].y,
 candidate_path.waypoints[2].z - candidate_path.waypoints[0].z
 };
 double terrain_normal[3];
 vector_cross_product(disp1, disp2, terrain_normal);
 FLIGHT_LOG("Local terrain normal at path start: (%.4f, %.4f, %.4f)\n",
 terrain_normal[0], terrain_normal[1], terrain_normal[2]);

 // Rotate each waypoint from terrain frame to rover body frame using local attitude
 FLIGHT_LOG("Rotating %d waypoints from terrain to body frame...\n", candidate_path.num_waypoints);
 double R[MATRIX_DIM][MATRIX_DIM];
 for (int wp = 0; wp < candidate_path.num_waypoints; wp++) {
 // Attitude varies along the path due to terrain slope
 double roll = candidate_path.waypoints[wp].z * 0.05;
 double pitch = candidate_path.waypoints[wp].y * 0.02;
 double yaw = atan2(candidate_path.waypoints[wp].y, candidate_path.waypoints[wp].x + 0.01);
 rotation_matrix_xyz(roll, pitch, yaw, R);

 // Apply rotation to transform waypoint into body frame
 double bx = R[0][0] * candidate_path.waypoints[wp].x
 + R[0][1] * candidate_path.waypoints[wp].y
 + R[0][2] * candidate_path.waypoints[wp].z;
 double by = R[1][0] * candidate_path.waypoints[wp].x
 + R[1][1] * candidate_path.waypoints[wp].y
 + R[1][2] * candidate_path.waypoints[wp].z;
 candidate_path.waypoints[wp].x = bx;
 candidate_path.waypoints[wp].y = by;
 }

 // Smooth transformed path over local terrain profile
 FLIGHT_LOG("Smoothing transformed path (%d waypoints)...\n", candidate_path.num_waypoints);
 TerrainMap local_terrain;
 generate_random_terrain(&local_terrain, 10.0, 1.0);
 Path smoothed = smooth_path(candidate_path, &local_terrain);

 FLIGHT_LOG("Navigation state estimation complete (smoothed path: %d waypoints).\n\n",
 smoothed.num_waypoints);
}

// Main function
int main() {
 clock_t start_time, end_time;
 double cpu_time_used;

 // Start timing
 start_time = clock();

 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(6241);

 // Initialize terrain
 TerrainMap terrain;
 generate_random_terrain(&terrain, 10.0, 1.0);
 FLIGHT_LOG("Terrain generated.\n");

 // Initialize obstacles
 Obstacle obstacles[MAX_OBSTACLES];
 int num_obstacles;
 generate_random_obstacles(obstacles, &num_obstacles, 30, TERRAIN_DIM);
 FLIGHT_LOG("%d obstacles generated.\n", num_obstacles);

 // Execute navigation computations for path planning
 execute_navigation_computations();

 // Set mission start and goal
 Point3D start_point = {2.0, 2.0, 0.0};
 Point3D goal_point = {TERRAIN_DIM - 3.0, TERRAIN_DIM - 3.0, 0.0};

 // Plan and execute mission
 plan_and_execute_mission(&terrain, obstacles, num_obstacles, start_point, goal_point);

 // End timing
 end_time = clock();
 cpu_time_used = ((double) (end_time - start_time)) / CLOCKS_PER_SEC;

 FLIGHT_LOG("Total execution time: %f seconds\n", cpu_time_used);

 return 0;
}

