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
 * Vision-Based Navigation Algorithm
 * HW TRIG VARIANT: All trig ops use CORDIC accelerator
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with CORDIC Trigonometric Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements vision-based navigation (VBN) for autonomous hazard detection and
 * safe landing on planetary surfaces. This workload processes onboard camera
 * imagery using feature extraction, image processing (edge detection, template
 * matching), and trajectory correction from visual cues. Derived from the
 * lunar lander hazard detection and avoidance (HDA) system
 * and applicable to future lunar polar exploration missions (LUPEX).
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
#define PI 3.14159265358979323846

/* VBN navigation filter state layout:
 * [0..2] position (LVLH, km)
 * [3..5] velocity (LVLH, km/s)
 * [6..9] attitude (quaternion, scalar-first)
 * [10..12] body rates (rad/s)
 * [13..15] gyro bias (rad/s)
 * [16..17] alt-bias, clock-drift
 * Total: 18 states.
 * Measurement: 6 star bearings + 3 landmark boresight angles
 * + 3 terrain feature positions + 3 altimeter/doppler
 * + 3 IMU pseudo-measurements = 18
 */
#define FEATURE_COUNT 64 // Number of features to extract
#define STATE_DIM 18 // State vector dimension
#define MEASUREMENT_DIM 18 // Measurement vector dimension
#define STAR_CATALOG_SIZE 150 // Number of stars in catalog
#define IMAGE_WIDTH 1024 // Image width
#define IMAGE_HEIGHT 1024 // Image height

/* RANSAC outlier rejection (typical star-tracker angular noise ~10-30 arcsec) */
#define RANSAC_TIMESTEPS 120
#define RANSAC_THRESHOLD 0.0015 /* rad (~5.2 arcmin) */
#define DFT_PATCH_SIZE 32 /* terrain spectral analysis window (px) */

/* lunar lander camera parameters (12.5 mm / 5.5 µm pixel) */
#define FOCAL_LENGTH_PX 2272.7 /* focal length in pixels */
#define PRINCIPAL_X 512.8
#define PRINCIPAL_Y 511.3

// Data structures
typedef struct {
 double data[STATE_DIM][STATE_DIM];
} Matrix;

typedef struct {
 double data[STATE_DIM];
} Vector;

typedef struct {
 double x, y;
} Point2D;

typedef struct {
 double x, y, z;
} Point3D;

typedef struct {
 Point2D pixel;
 Point3D vector;
 double magnitude;
 int id;
 bool matched;
} Star;

typedef struct {
 double q0, q1, q2, q3; // Quaternion components
} Quaternion;

typedef struct {
 Quaternion attitude; // Spacecraft attitude quaternion
 Vector velocity; // Spacecraft velocity vector
 Point3D position; // Spacecraft position
 Vector angularRate; // Angular rate
} SpacecraftState;

typedef struct {
 double timestamp;
 unsigned char data[IMAGE_HEIGHT][IMAGE_WIDTH];
} Image;

typedef struct {
 int id;
 Point3D direction;
 double magnitude;
} CatalogStar;

// Global variables
Matrix kalmanStateCovariance;
Matrix processNoiseCovariance;
Matrix measurementNoiseCovariance;
Matrix kalmanGain;
Matrix stateTransitionMatrix;
Matrix measurementMatrix;
Vector stateVector;
CatalogStar starCatalog[STAR_CATALOG_SIZE];
SpacecraftState currentState;

// Function prototypes
void initializeSystem();
void processImage(Image* image);
void extractFeatures(Image* image, Star* detectedStars, int* starCount);
void matchStars(Star* detectedStars, int starCount);
void estimatePose(Star* detectedStars, int starCount);
void updateKalmanFilter(Vector* measurement);
void applyQUESTAlgorithm(Star* detectedStars, int starCount, Quaternion* attitude);
void matrixMultiply(Matrix* A, Matrix* B, Matrix* result);
void matrixVectorMultiply(Matrix* A, Vector* v, Vector* result);
void matrixAdd(Matrix* A, Matrix* B, Matrix* result);
void matrixSubtract(Matrix* A, Matrix* B, Matrix* result);
void matrixTranspose(Matrix* A, Matrix* result);
void matrixInverse(Matrix* A, Matrix* result);
Point3D quaternionRotateVector(Quaternion* q, Point3D* v);
void runRANSAC(Star* detectedStars, int starCount, Quaternion* bestAttitude);

void propagateState(SpacecraftState *s, double dt);
void rk4_integrate(double state[6], double dt, double mu);
void spectralFeatureExtract(Image *img, double spectrum[STATE_DIM]);
void generateTrajectory(SpacecraftState *s, double waypoints[][3], int npts);

// Main function
int main() {
 clock_t start, end;
 double cpu_time_used;

 setvbuf(stdout, NULL, _IONBF, 0);

 FLIGHT_LOG("[VBN] init\n");

 // Initialize the system
 start = clock();
 initializeSystem();
 end = clock();
 cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
 FLIGHT_LOG("[VBN] init_t=%.3fs\n", cpu_time_used);

 // Simulate image capture and processing
 Image image;

 // Allocate memory for image data
 memset(&image, 0, sizeof(Image));

 // Generate a simulated reference image
 for (int i = 0; i < IMAGE_HEIGHT; i++) {
 for (int j = 0; j < IMAGE_WIDTH; j++) {
 // Create some random star-like patterns
 if (rand() % 10000 < 10) {
 image.data[i][j] = 255;
 } else {
 image.data[i][j] = 0;
 }
 }
 }

 // Process the image
 start = clock();
 for (int i = 0; i < 5; i++) { // Run multiple iterations for statistical averaging
 processImage(&image);
 }
 end = clock();
 cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
 FLIGHT_LOG("Average image processing time (5 iterations): %f seconds\n", cpu_time_used / 5.0);

 /* Terrain spectral analysis for hazard detection */
 double spectrum[STATE_DIM];
 spectralFeatureExtract(&image, spectrum);

 /* Hazard assessment: high-freq energy indicates rough terrain */
 double low_e = 0.0, high_e = 0.0;
 for (int k = 0; k < STATE_DIM; k++) {
 if (k < STATE_DIM / 3)
 low_e += spectrum[k];
 else
 high_e += spectrum[k];
 }
 double roughness = high_e / (low_e + 1e-12);
 int terrain_safe = (roughness < 1.85);
 FLIGHT_LOG("Hazard assessment: roughness=%.4f site %s\n",
 roughness, terrain_safe ? "SAFE" : "HAZARDOUS — retarget");

 /* Hazard avoidance trajectory generation */
 int n_waypoints = terrain_safe ? 16 : 10; /* shorter divert if hazardous */
 double waypoints[18][3];
 generateTrajectory(&currentState, waypoints, n_waypoints);

 /* Print trajectory waypoints (first, mid, last) */
 FLIGHT_LOG("Trajectory (%d waypoints):\n", n_waypoints);
 FLIGHT_LOG(" start : (%.3f, %.3f, %.3f) km\n",
 waypoints[0][0], waypoints[0][1], waypoints[0][2]);
 int mid = n_waypoints / 2;
 FLIGHT_LOG(" mid : (%.3f, %.3f, %.3f) km\n",
 waypoints[mid][0], waypoints[mid][1], waypoints[mid][2]);
 FLIGHT_LOG(" target: (%.3f, %.3f, %.3f) km\n",
 waypoints[n_waypoints-1][0], waypoints[n_waypoints-1][1],
 waypoints[n_waypoints-1][2]);

 /* RK4 orbit propagation along the planned trajectory */
 {
 double orb_state[6] = {
 currentState.position.x * 1000.0, currentState.position.y * 1000.0,
 currentState.position.z * 1000.0,
 currentState.velocity.data[0] * 1000.0,
 currentState.velocity.data[1] * 1000.0,
 currentState.velocity.data[2] * 1000.0
 };
 for (int i = 0; i < 500; i++)
 rk4_integrate(orb_state, 0.1, 4.9028e12);
 currentState.position.x = orb_state[0] / 1000.0;
 currentState.position.y = orb_state[1] / 1000.0;
 currentState.position.z = orb_state[2] / 1000.0;
 }

 /* State prediction: propagate navigation filter between image frames */
 for (int i = 0; i < 200; i++)
 propagateState(&currentState, 0.1);

 /* Kalman filter trace (navigation uncertainty) */
 double cov_trace = 0.0;
 for (int i = 0; i < STATE_DIM; i++)
 cov_trace += kalmanStateCovariance.data[i][i];

 // Print final state
 FLIGHT_LOG("\nFinal spacecraft state:\n");
 FLIGHT_LOG("Position: (%.6f, %.6f, %.6f) km\n",
 currentState.position.x, currentState.position.y, currentState.position.z);
 FLIGHT_LOG("Attitude quaternion: (%.6f, %.6f, %.6f, %.6f)\n",
 currentState.attitude.q0, currentState.attitude.q1,
 currentState.attitude.q2, currentState.attitude.q3);
 FLIGHT_LOG("EKF covariance trace: %.6e\n", cov_trace);
 FLIGHT_LOG("Terrain safe: %s\n", terrain_safe ? "YES" : "NO");

 return 0;
}

// Initialize the system
void initializeSystem() {
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(1783);

 // Initialize spacecraft state
 currentState.attitude.q0 = 1.0;
 currentState.attitude.q1 = 0.0;
 currentState.attitude.q2 = 0.0;
 currentState.attitude.q3 = 0.0;

 // Initialize state vector
 for (int i = 0; i < STATE_DIM; i++) {
 stateVector.data[i] = 0.0;
 }

 // Initialize star catalog with random star directions
 for (int i = 0; i < STAR_CATALOG_SIZE; i++) {
 starCatalog[i].id = i;

 // Generate random unit vector for star direction
 double theta = ((double)rand() / RAND_MAX) * 2.0 * PI;
 double phi = ((double)rand() / RAND_MAX) * PI;

 starCatalog[i].direction.x = hw_sin(phi) * hw_cos(theta);
 starCatalog[i].direction.y = hw_sin(phi) * hw_sin(theta);
 starCatalog[i].direction.z = hw_cos(phi);

 // Random magnitude between 1 and 6
 starCatalog[i].magnitude = 1.0 + ((double)rand() / RAND_MAX) * 5.0;
 }

 /* Initialise Kalman filter with physically meaningful covariances.
 * P0 diagonal: pos 1 km, vel 0.01 km/s, quat 0.01 rad, rates 1e-4 rad/s,
 * gyro bias 5e-5 rad/s, alt-bias 0.05 km, clock 1e-6 s */
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 kalmanStateCovariance.data[i][j] = 0.0;
 double p0_diag[STATE_DIM] = {
 1.0, 1.0, 1.0, /* position variance (km²) */
 1e-4, 1e-4, 1e-4, /* velocity variance (km²/s²) */
 1e-4, 1e-4, 1e-4, 1e-4, /* quaternion variance */
 1e-8, 1e-8, 1e-8, /* body-rate variance (rad²/s²) */
 2.5e-9, 2.5e-9, 2.5e-9, /* gyro bias variance (rad²/s²) */
 2.5e-3, /* altimeter bias (km²) */
 1e-12 /* clock drift (s²) */
 };
 for (int i = 0; i < STATE_DIM; i++)
 kalmanStateCovariance.data[i][i] = p0_diag[i];

 /* Process noise Q (continuous-white-noise acceleration model) */
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 processNoiseCovariance.data[i][j] = 0.0;
 double q_diag[STATE_DIM] = {
 1.2e-8, 1.2e-8, 1.2e-8, /* position process noise */
 4.7e-10, 4.7e-10, 4.7e-10, /* velocity */
 8.1e-7, 8.1e-7, 8.1e-7, 8.1e-7,/* quaternion diffusion */
 3.6e-10, 3.6e-10, 3.6e-10, /* rate random walk */
 1.8e-12, 1.8e-12, 1.8e-12, /* gyro bias stability */
 6.4e-9, /* altimeter drift */
 2.1e-14 /* clock drift rate */
 };
 for (int i = 0; i < STATE_DIM; i++)
 processNoiseCovariance.data[i][i] = q_diag[i];

 /* Measurement noise R: star-tracker 8" 1σ per axis, landmark 0.05°,
 * terrain feature 15 m, altimeter/Doppler mixed, IMU pseudo-meas */
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 measurementNoiseCovariance.data[i][j] = 0.0;
 double r_diag[MEASUREMENT_DIM] = {
 1.5e-9, 1.5e-9, 1.5e-9, 1.5e-9, 1.5e-9, 1.5e-9, /* star bearings (rad²) */
 7.6e-7, 7.6e-7, 7.6e-7, /* landmark angles */
 2.25e-4, 2.25e-4, 2.25e-4, /* terrain features km² */
 6.1e-6, 4.8e-8, 6.1e-6, /* altimeter/Doppler */
 4.2e-8, 4.2e-8, 4.2e-8 /* IMU pseudo-meas */
 };
 for (int i = 0; i < MEASUREMENT_DIM; i++)
 measurementNoiseCovariance.data[i][i] = r_diag[i];

 /* State transition: identity + velocity coupling (linearised at init) */
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 stateTransitionMatrix.data[i][j] = (i == j) ? 1.0 : 0.0;
 stateTransitionMatrix.data[0][3] = 0.1;
 stateTransitionMatrix.data[1][4] = 0.1;
 stateTransitionMatrix.data[2][5] = 0.1;
 stateTransitionMatrix.data[6][10] = 0.1;
 stateTransitionMatrix.data[7][11] = 0.1;
 stateTransitionMatrix.data[8][12] = 0.1;

 /* Measurement matrix H: identity mapping (direct observation model) */
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 measurementMatrix.data[i][j] = (i == j) ? 1.0 : 0.0;

 FLIGHT_LOG("[VBN] catalog=%d stars\n", STAR_CATALOG_SIZE);
}

// Process a single image frame
void processImage(Image* image) {
 Star detectedStars[FEATURE_COUNT];
 int starCount = 0;

 // Step 1: Extract features (stars) from the image
 extractFeatures(image, detectedStars, &starCount);

 // Step 2: Match detected stars with catalog
 matchStars(detectedStars, starCount);

 // Step 3: Estimate spacecraft pose (RANSAC + QUEST)
 estimatePose(detectedStars, starCount);

 /* Step 4: Build measurement from QUEST attitude + simulated sensors.
 * Star bearing residuals come from the attitude determination;
 * the remaining channels use the current state with sensor noise. */
 Vector measurement;
 for (int i = 0; i < MEASUREMENT_DIM; i++)
 measurement.data[i] = stateVector.data[i];
 /* Inject attitude residual from QUEST into quaternion measurement slots */
 measurement.data[6] = currentState.attitude.q0 + ((double)rand()/RAND_MAX - 0.5) * 7.8e-5;
 measurement.data[7] = currentState.attitude.q1 + ((double)rand()/RAND_MAX - 0.5) * 7.8e-5;
 measurement.data[8] = currentState.attitude.q2 + ((double)rand()/RAND_MAX - 0.5) * 7.8e-5;
 measurement.data[9] = currentState.attitude.q3 + ((double)rand()/RAND_MAX - 0.5) * 7.8e-5;
 /* Position from terrain features + noise */
 measurement.data[0] = currentState.position.x + ((double)rand()/RAND_MAX - 0.5) * 0.015;
 measurement.data[1] = currentState.position.y + ((double)rand()/RAND_MAX - 0.5) * 0.015;
 measurement.data[2] = currentState.position.z + ((double)rand()/RAND_MAX - 0.5) * 0.015;
 updateKalmanFilter(&measurement);
}

// Extract star features from image
void extractFeatures(Image* image, Star* detectedStars, int* starCount) {
 *starCount = 0;

 // Simple centroid detection algorithm
 for (int i = 1; i < IMAGE_HEIGHT - 1; i++) {
 for (int j = 1; j < IMAGE_WIDTH - 1; j++) {
 // Check if this pixel is bright and local maximum
 if (image->data[i][j] > 200 &&
 image->data[i][j] >= image->data[i-1][j] &&
 image->data[i][j] >= image->data[i+1][j] &&
 image->data[i][j] >= image->data[i][j-1] &&
 image->data[i][j] >= image->data[i][j+1]) {

 // Compute centroid more precisely using surrounding pixels
 double sumX = 0, sumY = 0, sumWeight = 0;

 for (int di = -2; di <= 2; di++) {
 for (int dj = -2; dj <= 2; dj++) {
 int ni = i + di;
 int nj = j + dj;

 if (ni >= 0 && ni < IMAGE_HEIGHT && nj >= 0 && nj < IMAGE_WIDTH) {
 double weight = image->data[ni][nj];
 sumX += nj * weight;
 sumY += ni * weight;
 sumWeight += weight;
 }
 }
 }

 if (sumWeight > 0 && *starCount < FEATURE_COUNT) {
 detectedStars[*starCount].pixel.x = sumX / sumWeight;
 detectedStars[*starCount].pixel.y = sumY / sumWeight;

 /* Convert pixel to unit vector via camera intrinsics */
 double fx = FOCAL_LENGTH_PX;
 double fy = FOCAL_LENGTH_PX;
 double cx = PRINCIPAL_X;
 double cy = PRINCIPAL_Y;

 double normalizedX = (detectedStars[*starCount].pixel.x - cx) / fx;
 double normalizedY = (detectedStars[*starCount].pixel.y - cy) / fy;

 double norm = sqrt(normalizedX*normalizedX + normalizedY*normalizedY + 1.0);
 detectedStars[*starCount].vector.x = normalizedX / norm;
 detectedStars[*starCount].vector.y = normalizedY / norm;
 detectedStars[*starCount].vector.z = 1.0 / norm;

 detectedStars[*starCount].magnitude = sumWeight / 5.0;
 detectedStars[*starCount].id = -1; // Not matched yet
 detectedStars[*starCount].matched = false;

 (*starCount)++;

 // Skip nearby pixels to avoid detecting the same star multiple times
 j += 3;
 }
 }
 }
 }

 FLIGHT_LOG("Extracted %d stars from image\n", *starCount);
}

// Match detected stars with star catalog
void matchStars(Star* detectedStars, int starCount) {
 // Geometric hash matching approach
 const double angleThreshold = 0.0087; /* ~0.5 deg inter-star angular tolerance */

 // Calculate angles between all pairs of detected stars
 double (*detectedAngles)[FEATURE_COUNT] = flight_malloc_impl(FEATURE_COUNT * sizeof(double[FEATURE_COUNT]));
 for (int i = 0; i < starCount; i++) {
 for (int j = i+1; j < starCount; j++) {
 // Compute dot product between unit vectors
 double dotProduct =
 detectedStars[i].vector.x * detectedStars[j].vector.x +
 detectedStars[i].vector.y * detectedStars[j].vector.y +
 detectedStars[i].vector.z * detectedStars[j].vector.z;

 // Clamp dot product to [-1, 1] range to avoid numerical issues
 if (dotProduct > 1.0) dotProduct = 1.0;
 if (dotProduct < -1.0) dotProduct = -1.0;

 // Calculate angle
 detectedAngles[i][j] = hw_acos(dotProduct);
 detectedAngles[j][i] = detectedAngles[i][j];
 }
 }

 // Calculate angles between all pairs of catalog stars
 double (*catalogAngles)[STAR_CATALOG_SIZE] = flight_malloc_impl(STAR_CATALOG_SIZE * sizeof(double[STAR_CATALOG_SIZE]));
 for (int i = 0; i < STAR_CATALOG_SIZE; i++) {
 for (int j = i+1; j < STAR_CATALOG_SIZE; j++) {
 // Compute dot product between unit vectors
 double dotProduct =
 starCatalog[i].direction.x * starCatalog[j].direction.x +
 starCatalog[i].direction.y * starCatalog[j].direction.y +
 starCatalog[i].direction.z * starCatalog[j].direction.z;

 // Clamp dot product to [-1, 1] range
 if (dotProduct > 1.0) dotProduct = 1.0;
 if (dotProduct < -1.0) dotProduct = -1.0;

 // Calculate angle
 catalogAngles[i][j] = hw_acos(dotProduct);
 catalogAngles[j][i] = catalogAngles[i][j];
 }
 }

 // For each detected star, find potential matches in catalog
 int matchCount = 0;
 for (int i = 0; i < starCount; i++) {
 int bestMatch = -1;
 int bestMatchCount = 0;

 for (int j = 0; j < STAR_CATALOG_SIZE; j++) {
 int currentMatchCount = 0;

 // For each other detected star, see if the angle is preserved
 for (int k = 0; k < starCount; k++) {
 if (k == i) continue;

 // For each star in catalog, check if angle is similar
 for (int l = 0; l < STAR_CATALOG_SIZE; l++) {
 if (l == j) continue;

 double angleDiff = fabs(detectedAngles[i][k] - catalogAngles[j][l]);
 if (angleDiff < angleThreshold) {
 currentMatchCount++;
 }
 }
 }

 if (currentMatchCount > bestMatchCount) {
 bestMatchCount = currentMatchCount;
 bestMatch = j;
 }
 }

 // If we have a good match, record it
 if (bestMatchCount > 3) {
 detectedStars[i].id = bestMatch;
 detectedStars[i].matched = true;
 matchCount++;
 }
 }

 flight_free_impl(detectedAngles);
 flight_free_impl(catalogAngles);

 FLIGHT_LOG("Matched %d stars with catalog\n", matchCount);
}

// Estimate spacecraft pose
void estimatePose(Star* detectedStars, int starCount) {
 // Use RANSAC to find the best pose estimation
 Quaternion bestAttitude;
 runRANSAC(detectedStars, starCount, &bestAttitude);

 // Update the spacecraft state with the best attitude
 currentState.attitude = bestAttitude;

 /* Integrate velocity in the body-frame direction determined by attitude */
 Point3D body_vhat = {1.0, 0.0, 0.0};
 Point3D inertial_v = quaternionRotateVector(&bestAttitude, &body_vhat);
 double v_mag = sqrt(currentState.velocity.data[0]*currentState.velocity.data[0] +
 currentState.velocity.data[1]*currentState.velocity.data[1] +
 currentState.velocity.data[2]*currentState.velocity.data[2]);
 double dt_frame = 0.125; /* inter-frame interval (8 Hz camera) */
 currentState.position.x += inertial_v.x * v_mag * dt_frame;
 currentState.position.y += inertial_v.y * v_mag * dt_frame;
 currentState.position.z += inertial_v.z * v_mag * dt_frame;
}

// Update Kalman filter with new measurements
void updateKalmanFilter(Vector* measurement) {
 // Predict step
 Matrix tempMatrix1, tempMatrix2;
 Vector tempVector;

 // Compute predicted state: x' = F*x
 matrixVectorMultiply(&stateTransitionMatrix, &stateVector, &tempVector);
 memcpy(&stateVector, &tempVector, sizeof(Vector));

 // Compute predicted covariance: P' = F*P*F^T + Q
 matrixTranspose(&stateTransitionMatrix, &tempMatrix1);
 matrixMultiply(&stateTransitionMatrix, &kalmanStateCovariance, &tempMatrix2);
 matrixMultiply(&tempMatrix2, &tempMatrix1, &tempMatrix1);
 matrixAdd(&tempMatrix1, &processNoiseCovariance, &kalmanStateCovariance);

 // Update step

 // Innovation: y = z - H*x
 Vector innovation;
 matrixVectorMultiply(&measurementMatrix, &stateVector, &tempVector);
 for (int i = 0; i < MEASUREMENT_DIM; i++) {
 innovation.data[i] = measurement->data[i] - tempVector.data[i];
 }

 // Innovation covariance: S = H*P*H^T + R
 Matrix innovationCovariance;
 matrixTranspose(&measurementMatrix, &tempMatrix1);
 matrixMultiply(&measurementMatrix, &kalmanStateCovariance, &tempMatrix2);
 matrixMultiply(&tempMatrix2, &tempMatrix1, &tempMatrix1);
 matrixAdd(&tempMatrix1, &measurementNoiseCovariance, &innovationCovariance);

 // Kalman gain: K = P * H^T * S^{-1}
 matrixTranspose(&measurementMatrix, &tempMatrix2); // H^T
 matrixMultiply(&kalmanStateCovariance, &tempMatrix2, &tempMatrix1); // P*H^T
 Matrix S_inv;
 matrixInverse(&innovationCovariance, &S_inv); // S^{-1}
 matrixMultiply(&tempMatrix1, &S_inv, &kalmanGain); // K = P*H^T*S^{-1}

 // Update state: x = x + K*y
 matrixVectorMultiply(&kalmanGain, &innovation, &tempVector);
 for (int i = 0; i < STATE_DIM; i++) {
 stateVector.data[i] += tempVector.data[i];
 }

 // Update covariance: P = (I - K*H)*P
 Matrix identity;
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 identity.data[i][j] = (i == j) ? 1.0 : 0.0;
 }
 }

 matrixMultiply(&kalmanGain, &measurementMatrix, &tempMatrix1);
 matrixSubtract(&identity, &tempMatrix1, &tempMatrix2);
 matrixMultiply(&tempMatrix2, &kalmanStateCovariance, &kalmanStateCovariance);

 /* Renormalise quaternion portion of state (indices 6..9) */
 double qn = sqrt(stateVector.data[6]*stateVector.data[6] +
 stateVector.data[7]*stateVector.data[7] +
 stateVector.data[8]*stateVector.data[8] +
 stateVector.data[9]*stateVector.data[9]);
 if (qn > 1e-12) {
 for (int i = 6; i <= 9; i++)
 stateVector.data[i] /= qn;
 }
}

// QUEST algorithm for attitude determination from matched star observations
void applyQUESTAlgorithm(Star* detectedStars, int starCount, Quaternion* attitude) {
 // Count matched stars
 int matchedCount = 0;
 for (int i = 0; i < starCount; i++) {
 if (detectedStars[i].matched) {
 matchedCount++;
 }
 }

 if (matchedCount < 2) {
 // Not enough matched stars for QUEST
 attitude->q0 = 1.0;
 attitude->q1 = 0.0;
 attitude->q2 = 0.0;
 attitude->q3 = 0.0;
 return;
 }

 // Create attitude profile matrix B
 Matrix B;
 memset(&B, 0, sizeof(Matrix));

 double weights[FEATURE_COUNT];
 double weightSum = 0.0;

 // Calculate weights based on star brightness
 for (int i = 0; i < starCount; i++) {
 if (detectedStars[i].matched) {
 weights[i] = 1.0 / (1.0 + detectedStars[i].magnitude);
 weightSum += weights[i];
 } else {
 weights[i] = 0.0;
 }
 }

 // Normalize weights
 for (int i = 0; i < starCount; i++) {
 weights[i] /= weightSum;
 }

 // Build B matrix
 for (int i = 0; i < starCount; i++) {
 if (detectedStars[i].matched) {
 int catalogIndex = detectedStars[i].id;

 // Reference vector (catalog)
 Point3D r_i = starCatalog[catalogIndex].direction;

 // Observed vector (from camera)
 Point3D b_i = detectedStars[i].vector;

 // Accumulate weighted outer product
 B.data[0][0] += weights[i] * b_i.x * r_i.x;
 B.data[0][1] += weights[i] * b_i.x * r_i.y;
 B.data[0][2] += weights[i] * b_i.x * r_i.z;

 B.data[1][0] += weights[i] * b_i.y * r_i.x;
 B.data[1][1] += weights[i] * b_i.y * r_i.y;
 B.data[1][2] += weights[i] * b_i.y * r_i.z;

 B.data[2][0] += weights[i] * b_i.z * r_i.x;
 B.data[2][1] += weights[i] * b_i.z * r_i.y;
 B.data[2][2] += weights[i] * b_i.z * r_i.z;
 }
 }

 // Calculate the S matrix
 Matrix S;
 S.data[0][0] = B.data[0][0] + B.data[1][1] + B.data[2][2];

 S.data[0][1] = B.data[1][2] - B.data[2][1];
 S.data[0][2] = B.data[2][0] - B.data[0][2];
 S.data[0][3] = B.data[0][1] - B.data[1][0];

 S.data[1][0] = B.data[1][2] - B.data[2][1];
 S.data[1][1] = B.data[0][0] - B.data[1][1] - B.data[2][2];
 S.data[1][2] = B.data[0][1] + B.data[1][0];
 S.data[1][3] = B.data[0][2] + B.data[2][0];

 S.data[2][0] = B.data[2][0] - B.data[0][2];
 S.data[2][1] = B.data[0][1] + B.data[1][0];
 S.data[2][2] = -B.data[0][0] + B.data[1][1] - B.data[2][2];
 S.data[2][3] = B.data[1][2] + B.data[2][1];

 S.data[3][0] = B.data[0][1] - B.data[1][0];
 S.data[3][1] = B.data[0][2] + B.data[2][0];
 S.data[3][2] = B.data[1][2] + B.data[2][1];
 S.data[3][3] = -B.data[0][0] - B.data[1][1] + B.data[2][2];

 // Find the maximum eigenvalue and corresponding eigenvector
 // Eigenvalue estimation via power iteration

 // Running 10 iterations of power method
 Vector v;
 v.data[0] = 1.0;
 v.data[1] = 0.0;
 v.data[2] = 0.0;
 v.data[3] = 0.0;

 for (int iter = 0; iter < 10; iter++) {
 Vector vNew;

 // Matrix-vector multiplication
 for (int i = 0; i < 4; i++) {
 vNew.data[i] = 0.0;
 for (int j = 0; j < 4; j++) {
 vNew.data[i] += S.data[i][j] * v.data[j];
 }
 }

 // Normalize
 double norm = 0.0;
 for (int i = 0; i < 4; i++) {
 norm += vNew.data[i] * vNew.data[i];
 }
 norm = sqrt(norm);

 for (int i = 0; i < 4; i++) {
 v.data[i] = vNew.data[i] / norm;
 }
 }

 // The resulting eigenvector is our quaternion
 attitude->q0 = v.data[0];
 attitude->q1 = v.data[1];
 attitude->q2 = v.data[2];
 attitude->q3 = v.data[3];

 // Ensure unit quaternion
 double norm = sqrt(attitude->q0*attitude->q0 + attitude->q1*attitude->q1 +
 attitude->q2*attitude->q2 + attitude->q3*attitude->q3);

 attitude->q0 /= norm;
 attitude->q1 /= norm;
 attitude->q2 /= norm;
 attitude->q3 /= norm;
}

// Matrix operations
void matrixMultiply(Matrix* A, Matrix* B, Matrix* result) {
 // Initialize result matrix to zeros
 memset(result, 0, sizeof(Matrix));

 // Perform matrix multiplication
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 for (int k = 0; k < STATE_DIM; k++) {
 result->data[i][j] += A->data[i][k] * B->data[k][j];
 }
 }
 }
}

void matrixVectorMultiply(Matrix* A, Vector* v, Vector* result) {
 // Initialize result vector to zeros
 for (int i = 0; i < STATE_DIM; i++) {
 result->data[i] = 0.0;
 }

 // Perform matrix-vector multiplication
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 result->data[i] += A->data[i][j] * v->data[j];
 }
 }
}

void matrixAdd(Matrix* A, Matrix* B, Matrix* result) {
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 result->data[i][j] = A->data[i][j] + B->data[i][j];
 }
 }
}

void matrixSubtract(Matrix* A, Matrix* B, Matrix* result) {
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 result->data[i][j] = A->data[i][j] - B->data[i][j];
 }
 }
}

void matrixTranspose(Matrix* A, Matrix* result) {
 for (int i = 0; i < STATE_DIM; i++) {
 for (int j = 0; j < STATE_DIM; j++) {
 result->data[i][j] = A->data[j][i];
 }
 }
}

/* Gauss-Jordan matrix inversion with partial pivoting */
void matrixInverse(Matrix* A, Matrix* result) {
 double aug[STATE_DIM][2 * STATE_DIM];
 /* Build augmented matrix [A | I] */
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 aug[i][j] = A->data[i][j];
 aug[i][j + STATE_DIM] = (i == j) ? 1.0 : 0.0;
 }
 /* Forward elimination with partial pivoting */
 for (int col = 0; col < STATE_DIM; col++) {
 int best = col;
 double best_val = fabs(aug[col][col]);
 for (int row = col + 1; row < STATE_DIM; row++)
 if (fabs(aug[row][col]) > best_val) { best_val = fabs(aug[row][col]); best = row; }
 if (best != col)
 for (int k = 0; k < 2 * STATE_DIM; k++) {
 double tmp = aug[col][k]; aug[col][k] = aug[best][k]; aug[best][k] = tmp;
 }
 double pivot = aug[col][col];
 if (fabs(pivot) < 1e-14) pivot = 1e-14;
 for (int k = 0; k < 2 * STATE_DIM; k++) aug[col][k] /= pivot;
 for (int row = 0; row < STATE_DIM; row++) {
 if (row == col) continue;
 double factor = aug[row][col];
 for (int k = 0; k < 2 * STATE_DIM; k++)
 aug[row][k] -= factor * aug[col][k];
 }
 }
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 result->data[i][j] = aug[i][j + STATE_DIM];
}

// Quaternion operations
Point3D quaternionRotateVector(Quaternion* q, Point3D* v) {
 // Convert quaternion to rotation matrix
 double R[3][3];
 double q0q0 = q->q0 * q->q0;
 double q1q1 = q->q1 * q->q1;
 double q2q2 = q->q2 * q->q2;
 double q3q3 = q->q3 * q->q3;

 double q0q1 = q->q0 * q->q1;
 double q0q2 = q->q0 * q->q2;
 double q0q3 = q->q0 * q->q3;
 double q1q2 = q->q1 * q->q2;
 double q1q3 = q->q1 * q->q3;
 double q2q3 = q->q2 * q->q3;

 R[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
 R[0][1] = 2.0 * (q1q2 - q0q3);
 R[0][2] = 2.0 * (q1q3 + q0q2);

 R[1][0] = 2.0 * (q1q2 + q0q3);
 R[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
 R[1][2] = 2.0 * (q2q3 - q0q1);

 R[2][0] = 2.0 * (q1q3 - q0q2);
 R[2][1] = 2.0 * (q2q3 + q0q1);
 R[2][2] = q0q0 - q1q1 - q2q2 + q3q3;

 // Rotate the vector
 Point3D result;
 result.x = R[0][0] * v->x + R[0][1] * v->y + R[0][2] * v->z;
 result.y = R[1][0] * v->x + R[1][1] * v->y + R[1][2] * v->z;
 result.z = R[2][0] * v->x + R[2][1] * v->y + R[2][2] * v->z;

 return result;
}

// RANSAC algorithm for robust pose estimation
void runRANSAC(Star* detectedStars, int starCount, Quaternion* bestAttitude) {
 int bestInlierCount = 0;
 double bestError = INFINITY;

 // Initialize best attitude as identity quaternion
 bestAttitude->q0 = 1.0;
 bestAttitude->q1 = 0.0;
 bestAttitude->q2 = 0.0;
 bestAttitude->q3 = 0.0;

 // Count matched stars
 int matchedCount = 0;
 for (int i = 0; i < starCount; i++) {
 if (detectedStars[i].matched) {
 matchedCount++;
 }
 }

 if (matchedCount < 3) {
 return; // Not enough matches for RANSAC
 }

 // Create array of matched stars
 Star matchedStars[FEATURE_COUNT];
 int matchedIdx = 0;
 for (int i = 0; i < starCount; i++) {
 if (detectedStars[i].matched) {
 matchedStars[matchedIdx++] = detectedStars[i];
 }
 }

 // RANSAC iterations
 for (int iter = 0; iter < RANSAC_TIMESTEPS; iter++) {
 // Select 3 random matched stars
 int idx1 = rand() % matchedCount;
 int idx2 = rand() % matchedCount;
 int idx3 = rand() % matchedCount;

 // Ensure we have 3 different stars
 if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
 continue;
 }

 // Create a small subset of stars for initial attitude estimation
 Star subset[3];
 subset[0] = matchedStars[idx1];
 subset[1] = matchedStars[idx2];
 subset[2] = matchedStars[idx3];

 // Estimate attitude from this subset
 Quaternion candidateAttitude;
 applyQUESTAlgorithm(subset, 3, &candidateAttitude);

 // Count inliers and compute error
 int inlierCount = 0;
 double totalError = 0.0;

 for (int i = 0; i < matchedCount; i++) {
 int catalogIdx = matchedStars[i].id;

 // Get catalog vector
 Point3D catalogVec = starCatalog[catalogIdx].direction;

 // Rotate catalog vector using candidate attitude
 Point3D rotatedVec = quaternionRotateVector(&candidateAttitude, &catalogVec);

 // Calculate angle between rotated catalog vector and observed vector
 double dotProduct =
 rotatedVec.x * matchedStars[i].vector.x +
 rotatedVec.y * matchedStars[i].vector.y +
 rotatedVec.z * matchedStars[i].vector.z;

 // Clamp dot product
 if (dotProduct > 1.0) dotProduct = 1.0;
 if (dotProduct < -1.0) dotProduct = -1.0;

 double angle = hw_acos(dotProduct);

 if (angle < RANSAC_THRESHOLD) {
 inlierCount++;
 totalError += angle;
 }
 }

 // Check if this is the best model so far
 if (inlierCount > bestInlierCount ||
 (inlierCount == bestInlierCount && totalError < bestError)) {
 bestInlierCount = inlierCount;
 bestError = totalError;
 *bestAttitude = candidateAttitude;
 }
 }

 // Refine attitude using all inliers (optional)
 if (bestInlierCount > 3) {
 // Create a subset of all inliers
 Star inliers[FEATURE_COUNT];
 int inlierIdx = 0;

 for (int i = 0; i < matchedCount; i++) {
 int catalogIdx = matchedStars[i].id;

 // Get catalog vector
 Point3D catalogVec = starCatalog[catalogIdx].direction;

 // Rotate catalog vector using best attitude
 Point3D rotatedVec = quaternionRotateVector(bestAttitude, &catalogVec);

 // Calculate angle between rotated catalog vector and observed vector
 double dotProduct =
 rotatedVec.x * matchedStars[i].vector.x +
 rotatedVec.y * matchedStars[i].vector.y +
 rotatedVec.z * matchedStars[i].vector.z;

 // Clamp dot product
 if (dotProduct > 1.0) dotProduct = 1.0;
 if (dotProduct < -1.0) dotProduct = -1.0;

 double angle = hw_acos(dotProduct);

 if (angle < RANSAC_THRESHOLD) {
 inliers[inlierIdx++] = matchedStars[i];
 }
 }

 // Re-estimate attitude using all inliers
 applyQUESTAlgorithm(inliers, inlierIdx, bestAttitude);
 }
}



/* ===================================================================
 * State Prediction Module
 * Propagate spacecraft navigation state using linearised dynamics
 * =================================================================== */
void propagateState(SpacecraftState *s, double dt) {
 /* Build 18×18 state transition matrix Φ = I + F·dt (first order) */
 Matrix phi;
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 phi.data[i][j] = (i == j) ? 1.0 : 0.0;

 /* Position rows coupled to velocity */
 phi.data[0][3] = dt; phi.data[1][4] = dt; phi.data[2][5] = dt;
 /* Attitude rows coupled to angular rates */
 phi.data[6][9] = dt; phi.data[7][10] = dt; phi.data[8][11] = dt;
 /* Cross-coupling from attitude to position (gravity-gradient) */
 double w = 0.000890; /* orbital rate rad/s (100 km lunar orbit) */
 phi.data[0][6] = 3.0*w*w*dt*dt/2.0;
 phi.data[1][7] = -w*w*dt*dt/2.0;

 /* x_new = Φ * x */
 Vector new_state;
 matrixVectorMultiply(&phi, &stateVector, &new_state);
 for (int i = 0; i < STATE_DIM; i++)
 stateVector.data[i] = new_state.data[i];

 /* Propagate covariance: P = Φ P Φ^T + Q (3×3-blocked) */
 Matrix temp, phiT;
 matrixMultiply(&phi, &kalmanStateCovariance, &temp);
 matrixTranspose(&phi, &phiT);
 matrixMultiply(&temp, &phiT, &kalmanStateCovariance);
 matrixAdd(&kalmanStateCovariance, &processNoiseCovariance, &kalmanStateCovariance);

 /* Update position from state vector */
 s->position.x += s->velocity.data[0] * dt;
 s->position.y += s->velocity.data[1] * dt;
 s->position.z += s->velocity.data[2] * dt;
}

/* ===================================================================
 * Numerical Integration Module
 * Classical RK4 integrator for spacecraft equations of motion
 * =================================================================== */
static void eom_accel(const double state[6], double mu, double acc[3]) {
 double r = sqrt(state[0]*state[0] + state[1]*state[1] + state[2]*state[2]);
 double r3 = r * r * r;
 if (r3 < 1e-10) r3 = 1e-10;
 for (int i = 0; i < 3; i++)
 acc[i] = -mu * state[i] / r3;
}

void rk4_integrate(double state[6], double dt, double mu) {
 double k1[6], k2[6], k3[6], k4[6], tmp[6];
 double acc[3];

 /* k1 */
 eom_accel(state, mu, acc);
 for (int i = 0; i < 3; i++) { k1[i] = state[3+i]; k1[3+i] = acc[i]; }

 /* k2 */
 for (int i = 0; i < 6; i++) tmp[i] = state[i] + 0.5*dt*k1[i];
 eom_accel(tmp, mu, acc);
 for (int i = 0; i < 3; i++) { k2[i] = tmp[3+i]; k2[3+i] = acc[i]; }

 /* k3 */
 for (int i = 0; i < 6; i++) tmp[i] = state[i] + 0.5*dt*k2[i];
 eom_accel(tmp, mu, acc);
 for (int i = 0; i < 3; i++) { k3[i] = tmp[3+i]; k3[3+i] = acc[i]; }

 /* k4 */
 for (int i = 0; i < 6; i++) tmp[i] = state[i] + dt*k3[i];
 eom_accel(tmp, mu, acc);
 for (int i = 0; i < 3; i++) { k4[i] = tmp[3+i]; k4[3+i] = acc[i]; }

 /* Combine */
 for (int i = 0; i < 6; i++)
 state[i] += dt * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6.0;
}

/* ===================================================================
 * Spectral Feature Extraction Module
 * DFT-based terrain frequency analysis for hazard detection
 * =================================================================== */
void spectralFeatureExtract(Image *img, double spectrum[STATE_DIM]) {
 /* Extract a DFT_PATCH_SIZE × DFT_PATCH_SIZE terrain patch centred on
 * the image and compute 1-D DFT of each row, then average the magnitude
 * spectra. Only STATE_DIM frequency bins are kept for downstream use. */
 int cx = IMAGE_WIDTH / 2, cy = IMAGE_HEIGHT / 2;
 int half = DFT_PATCH_SIZE / 2;

 double full_spectrum[DFT_PATCH_SIZE];
 for (int k = 0; k < DFT_PATCH_SIZE; k++) full_spectrum[k] = 0.0;

 for (int row = 0; row < DFT_PATCH_SIZE; row++) {
 int iy = cy - half + row;
 if (iy < 0 || iy >= IMAGE_HEIGHT) continue;
 for (int k = 0; k < DFT_PATCH_SIZE; k++) {
 double re = 0.0, im = 0.0;
 for (int n = 0; n < DFT_PATCH_SIZE; n++) {
 int ix = cx - half + n;
 if (ix < 0 || ix >= IMAGE_WIDTH) continue;
 double val = (double)img->data[iy][ix];
 double angle = -2.0 * PI * k * n / (double)DFT_PATCH_SIZE;
 re += val * hw_cos(angle);
 im += val * hw_sin(angle);
 }
 full_spectrum[k] += sqrt(re * re + im * im);
 }
 }
 /* Normalise and down-sample into STATE_DIM bins */
 for (int k = 0; k < STATE_DIM; k++) {
 int src = k * DFT_PATCH_SIZE / STATE_DIM;
 spectrum[k] = full_spectrum[src] / (double)DFT_PATCH_SIZE;
 }
}

/* ===================================================================
 * Trajectory Generation Module
 * Generate a powered-descent hazard-avoidance trajectory.
 * Uses matrix operations for 3-DOF guidance and constraint evaluation.
 * =================================================================== */
void generateTrajectory(SpacecraftState *s, double waypoints[][3], int npts) {
 /* Initial and target states */
 double alt = sqrt(s->position.x*s->position.x +
 s->position.y*s->position.y +
 s->position.z*s->position.z);
 double target_alt = 0.0; /* surface */
 double total_time = 127.5; /* descent duration (s) */
 double dt_seg = total_time / (npts - 1);

 /* Build 3×npts waypoint matrix as polynomial trajectory */
 /* x(t) = a0 + a1*t + a2*t^2 + a3*t^3 via matrix solve */
 Matrix A_poly, A_inv, B_rhs, coeffs;
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 A_poly.data[i][j] = 0.0;

 /* Fill Vandermonde-like constraint matrix for 4 boundary conditions
 per axis, padded to 18×18 for matrix operations.
 Row 0: position at t=0 [1, 0, 0, 0]
 Row 1: velocity at t=0 [0, 1, 0, 0]
 Row 2: position at t=T [1, T, T², T³]
 Row 3: velocity at t=T [0, 1, 2T, 3T²] */
 for (int i = 0; i < 4; i++) {
 double t = (i < 2) ? 0.0 : total_time;
 double val = 1.0;
 for (int j = 0; j < 4; j++) {
 A_poly.data[i][j] = val;
 val *= t;
 }
 }
 /* Fix rows 1 and 3: these are derivative (velocity) constraints */
 /* Row 1: dx/dt evaluated at t=0 → [0, 1, 0, 0] */
 A_poly.data[1][0] = 0.0;
 A_poly.data[1][1] = 1.0;
 A_poly.data[1][2] = 0.0;
 A_poly.data[1][3] = 0.0;
 /* Row 3: dx/dt evaluated at t=T → [0, 1, 2T, 3T²] */
 A_poly.data[3][0] = 0.0;
 A_poly.data[3][1] = 1.0;
 A_poly.data[3][2] = 2.0 * total_time;
 A_poly.data[3][3] = 3.0 * total_time * total_time;
 /* Fill remaining diagonal to make invertible */
 for (int i = 4; i < STATE_DIM; i++)
 A_poly.data[i][i] = 1.0;

 matrixInverse(&A_poly, &A_inv);

 /* RHS: start at current position, end at target with zero velocity */
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 B_rhs.data[i][j] = 0.0;

 B_rhs.data[0][0] = s->position.x; /* x(0) = x0 */
 B_rhs.data[1][0] = s->velocity.data[0]; /* x'(0) = v0 */
 B_rhs.data[2][0] = 0.0; /* x(T) = 0 (landing site) */
 B_rhs.data[3][0] = 0.0; /* x'(T) = 0 */

 B_rhs.data[0][1] = s->position.y;
 B_rhs.data[1][1] = s->velocity.data[1];

 B_rhs.data[0][2] = alt;
 B_rhs.data[1][2] = s->velocity.data[2];
 B_rhs.data[2][2] = target_alt;

 matrixMultiply(&A_inv, &B_rhs, &coeffs);

 /* Evaluate trajectory at npts waypoints */
 for (int p = 0; p < npts; p++) {
 double t = p * dt_seg;
 for (int ax = 0; ax < 3; ax++) {
 double val = 0.0, tp = 1.0;
 for (int c = 0; c < 4; c++) {
 val += coeffs.data[c][ax] * tp;
 tp *= t;
 }
 waypoints[p][ax] = val;
 }
 }
}
