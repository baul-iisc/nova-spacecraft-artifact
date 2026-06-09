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
 * Orbit Determination and Propagation Algorithm
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard orbit determination and numerical propagation for autonomous
 * satellite station-keeping. This workload performs high-fidelity orbit propagation
 * with J2 gravitational harmonics, Kalman-filtered state estimation from
 * ground tracking data, and coordinate transformations (ECI/ECEF). Used across
 * all satellite missions managed by mission control (Telemetry, Tracking and
 * Command Network) for orbit maintenance and collision avoidance.
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

// OD filter dimensions (8-state EKF: pos(3)+vel(3)+clk_bias+clk_drift)
#define MAX_MEASUREMENTS 100
#define NUM_TIMESTEPS 200
#define DELTA_T 10.0 // Time step in seconds
#define MAX_SMALL_MATRIX_DIM 16 // Maximum dimension for small matrix

// Constants
#define PI 3.141592653589793
#define MU 3.986004418e14 // Earth gravitational parameter (m^3/s^2)
#define J2 1.08262668e-3 // Earth J2 perturbation coefficient
#define RE 6378137.0 // Earth radius (m)
#define C_LIGHT 299792458.0 // Speed of light (m/s)
#define OMEGA_EARTH 7.2921150e-5 // Earth rotation rate (rad/s)
#define AU_M 1.495978707e11 // Astronomical unit in metres
#define MU_MOON 4.9048695e12 // Moon gravitational parameter (m³/s²)
#define DEG2RAD (PI / 180.0)

/* Module-scope Julian date updated each integration step.
 * Used by calculate_perturbations() for solar/lunar ephemeris
 * so the function signature stays ABI-stable. */
static double g_epoch_jd = 2451545.0; /* default J2000.0 */

// Software trigonometric function wrappers
// In the accelerated version, these map to CORDIC hardware instructions
#define sw_sin(angle) sin(angle)
#define sw_cos(angle) cos(angle)

// Structure for spacecraft state
typedef struct {
 double position[3]; // Position vector (m) [x,y,z] in ECI
 double velocity[3]; // Velocity vector (m/s)
 double acceleration[3]; // Acceleration vector (m/s^2)
 double orbital_elements[6]; // [a,e,i,Ω,ω,M] - semi-major axis, eccentricity, inclination, RAAN, arg of periapsis, mean anomaly
 double clock_bias; // Clock bias (s)
 double clock_drift; // Clock drift (s/s)
} SpacecraftState;

// Structure for ground station
typedef struct {
 double position[3]; // ECEF position (m)
 double elevation_mask; // Minimum elevation angle (rad)
} GroundStation;

// Structure for GNSS satellite
typedef struct {
 int prn; // Satellite ID
 double position[3]; // Position in ECEF (m)
 double velocity[3]; // Velocity in ECEF (m/s)
 double clock_bias; // Clock bias (s)
 double clock_drift; // Clock drift (s/s)
} GNSSSatellite;

// Structure for pseudorange measurement
typedef struct {
 int prn; // Satellite ID
 double pseudorange; // Pseudorange measurement (m)
 double doppler; // Doppler measurement (Hz)
 double elevation; // Elevation angle (rad)
 double azimuth; // Azimuth angle (rad)
 double weight; // Measurement weight
 double sat_pos_eci[3]; // GNSS satellite position in ECI (m)
} Measurement;

// Structure for the Earth Orientation Parameters
typedef struct {
 double x_pole; // Pole coordinates (rad)
 double y_pole;
 double ut1_utc; // UT1-UTC (s)
 double lod; // Length of day (s)
 double dpsi; // Nutation corrections (rad)
 double deps;
} EOP;

// Function prototypes for matrix operations
 
// Function for small matrix inverse (up to MAX_SMALL_MATRIX_DIM x MAX_SMALL_MATRIX_DIM)
void small_matrix_inverse(double A[][MAX_SMALL_MATRIX_DIM], 
 double A_inv[][MAX_SMALL_MATRIX_DIM], 
 int n);

// Function to multiply two matrices

// Function to multiply a matrix with a vector

// Function to transpose a matrix

// Function to compute matrix inverse (Gauss-Jordan elimination)

// Function for small matrix inverse (max MAX_SMALL_MATRIX_DIM x MAX_SMALL_MATRIX_DIM)
void small_matrix_inverse(double A[][MAX_SMALL_MATRIX_DIM], 
 double A_inv[][MAX_SMALL_MATRIX_DIM], 
 int n) {
 // Verify n is within bounds
 if (n <= 0 || n > MAX_SMALL_MATRIX_DIM) {
 FLIGHT_LOG("Error: Invalid matrix size for inversion: %d (max is %d)\n", n, MAX_SMALL_MATRIX_DIM);
 return;
 }
 
 double aug[MAX_SMALL_MATRIX_DIM][2*MAX_SMALL_MATRIX_DIM];
 double ratio;
 
 // Create augmented matrix [A|I]
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 aug[i][j] = A[i][j];
 aug[i][j+n] = (i == j) ? 1.0 : 0.0;
 }
 }
 
 // Gaussian elimination
 for (int i = 0; i < n; i++) {
 // Partial pivoting
 double max_val = fabs(aug[i][i]);
 int max_row = i;
 for (int k = i+1; k < n; k++) {
 if (fabs(aug[k][i]) > max_val) {
 max_val = fabs(aug[k][i]);
 max_row = k;
 }
 }
 
 // Swap rows if needed
 if (max_row != i) {
 for (int j = 0; j < 2*n; j++) {
 double temp = aug[i][j];
 aug[i][j] = aug[max_row][j];
 aug[max_row][j] = temp;
 }
 }
 
 // Scale the pivot row
 double pivot = aug[i][i];
 if (fabs(pivot) < 1e-10) {
 // Matrix is singular or nearly singular
 FLIGHT_LOG("Warning: Near-singular matrix encountered in small_matrix_inverse()\n");
 // Add a small regularization term
 aug[i][i] += 1e-6;
 pivot = aug[i][i];
 }
 
 for (int j = 0; j < 2*n; j++) {
 aug[i][j] /= pivot;
 }
 
 // Eliminate other rows
 for (int k = 0; k < n; k++) {
 if (k != i) {
 ratio = aug[k][i];
 for (int j = 0; j < 2*n; j++) {
 aug[k][j] -= ratio * aug[i][j];
 }
 }
 }
 }
 
 // Extract the inverse
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 A_inv[i][j] = aug[i][j+n];
 }
 }
}

// Function to add two matrices

// Function to subtract two matrices

// Vector operations
double vector_magnitude(double v[3]) {
 return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void vector_normalize(double v[3]) {
 double mag = vector_magnitude(v);
 if (mag > 1e-10) { // Avoid division by near-zero
 v[0] /= mag;
 v[1] /= mag;
 v[2] /= mag;
 } else {
 v[0] = 0.0;
 v[1] = 0.0;
 v[2] = 0.0;
 }
}

void vector_cross(double a[3], double b[3], double result[3]) {
 result[0] = a[1]*b[2] - a[2]*b[1];
 result[1] = a[2]*b[0] - a[0]*b[2];
 result[2] = a[0]*b[1] - a[1]*b[0];
}

double vector_dot(double a[3], double b[3]) {
 return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}


void vector_subtract(double a[3], double b[3], double result[3]) {
 result[0] = a[0] - b[0];
 result[1] = a[1] - b[1];
 result[2] = a[2] - b[2];
}

void vector_scale(double v[3], double scale, double result[3]) {
 result[0] = v[0] * scale;
 result[1] = v[1] * scale;
 result[2] = v[2] * scale;
}

// Sensor platform attitude quaternion for star tracker and ranging instrument pointing
typedef struct {
 double q[4]; // Quaternion [w, x, y, z] (scalar-first convention)
} AttitudeQuaternion;

// Normalize attitude quaternion to unit magnitude
void quaternion_normalize(AttitudeQuaternion *quat) {
 double norm = sqrt(quat->q[0]*quat->q[0] + quat->q[1]*quat->q[1] +
 quat->q[2]*quat->q[2] + quat->q[3]*quat->q[3]);
 if (norm > 1e-10) {
 for (int i = 0; i < 4; i++) quat->q[i] /= norm;
 } else {
 quat->q[0] = 1.0; quat->q[1] = 0.0;
 quat->q[2] = 0.0; quat->q[3] = 0.0;
 }
}

// Quaternion multiplication: result = a * b (Hamilton product)
void quaternion_multiply(AttitudeQuaternion *result, AttitudeQuaternion *a, AttitudeQuaternion *b) {
 result->q[0] = a->q[0]*b->q[0] - a->q[1]*b->q[1] - a->q[2]*b->q[2] - a->q[3]*b->q[3];
 result->q[1] = a->q[0]*b->q[1] + a->q[1]*b->q[0] + a->q[2]*b->q[3] - a->q[3]*b->q[2];
 result->q[2] = a->q[0]*b->q[2] - a->q[1]*b->q[3] + a->q[2]*b->q[0] + a->q[3]*b->q[1];
 result->q[3] = a->q[0]*b->q[3] + a->q[1]*b->q[2] - a->q[2]*b->q[1] + a->q[3]*b->q[0];
}

// Rotate a 3D vector by quaternion: v_out = q * v_in * q_conj
void quaternion_rotate_vector(AttitudeQuaternion *q, double v_in[3], double v_out[3]) {
 double w = q->q[0], x = q->q[1], y = q->q[2], z = q->q[3];
 double t0 = 2.0 * (x*v_in[0] + y*v_in[1] + z*v_in[2]);
 double t1 = w*w - (x*x + y*y + z*z);

 v_out[0] = t1*v_in[0] + t0*x + 2.0*w*(y*v_in[2] - z*v_in[1]);
 v_out[1] = t1*v_in[1] + t0*y + 2.0*w*(z*v_in[0] - x*v_in[2]);
 v_out[2] = t1*v_in[2] + t0*z + 2.0*w*(x*v_in[1] - y*v_in[0]);
}

// Convert attitude quaternion to Direction Cosine Matrix (body-to-inertial)
void quaternion_to_dcm(AttitudeQuaternion *q, double dcm[3][3]) {
 double w = q->q[0], x = q->q[1], y = q->q[2], z = q->q[3];

 dcm[0][0] = 1.0 - 2.0*(y*y + z*z);
 dcm[0][1] = 2.0*(x*y - w*z);
 dcm[0][2] = 2.0*(x*z + w*y);
 dcm[1][0] = 2.0*(x*y + w*z);
 dcm[1][1] = 1.0 - 2.0*(x*x + z*z);
 dcm[1][2] = 2.0*(y*z - w*x);
 dcm[2][0] = 2.0*(x*z - w*y);
 dcm[2][1] = 2.0*(y*z + w*x);
 dcm[2][2] = 1.0 - 2.0*(x*x + y*y);
}

// Compute attitude quaternion from orbital position and velocity (LVLH nadir-pointing)
// z_body = -r_hat (nadir), y_body = -(r x v)/|r x v| (orbit anti-normal)
void quaternion_from_orbital_frame(double pos[3], double vel[3], AttitudeQuaternion *q_att) {
 double r_mag = vector_magnitude(pos);
 if (r_mag < 1e-10) {
 q_att->q[0] = 1.0; q_att->q[1] = 0.0;
 q_att->q[2] = 0.0; q_att->q[3] = 0.0;
 return;
 }

 // Body z-axis points nadir
 double z_body[3];
 vector_scale(pos, -1.0/r_mag, z_body);

 // Orbit normal h = r x v
 double h[3];
 vector_cross(pos, vel, h);
 double h_mag = vector_magnitude(h);
 if (h_mag < 1e-10) h_mag = 1.0;

 // Body y-axis is anti-normal to orbit plane
 double y_body[3];
 vector_scale(h, -1.0/h_mag, y_body);

 // Body x-axis completes the triad (roughly along-track)
 double x_body[3];
 vector_cross(y_body, z_body, x_body);
 double x_mag = vector_magnitude(x_body);
 if (x_mag > 1e-10) vector_scale(x_body, 1.0/x_mag, x_body);

 // DCM columns are the body axes expressed in ECI (body-to-inertial)
 double dcm[3][3];
 dcm[0][0] = x_body[0]; dcm[0][1] = y_body[0]; dcm[0][2] = z_body[0];
 dcm[1][0] = x_body[1]; dcm[1][1] = y_body[1]; dcm[1][2] = z_body[1];
 dcm[2][0] = x_body[2]; dcm[2][1] = y_body[2]; dcm[2][2] = z_body[2];

 // Extract quaternion from DCM (Shepperd's method)
 double trace = dcm[0][0] + dcm[1][1] + dcm[2][2];
 if (trace > 0) {
 double s = 0.5 / sqrt(trace + 1.0);
 q_att->q[0] = 0.25 / s;
 q_att->q[1] = (dcm[2][1] - dcm[1][2]) * s;
 q_att->q[2] = (dcm[0][2] - dcm[2][0]) * s;
 q_att->q[3] = (dcm[1][0] - dcm[0][1]) * s;
 } else if (dcm[0][0] > dcm[1][1] && dcm[0][0] > dcm[2][2]) {
 double s = 2.0 * sqrt(1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2]);
 q_att->q[0] = (dcm[2][1] - dcm[1][2]) / s;
 q_att->q[1] = 0.25 * s;
 q_att->q[2] = (dcm[0][1] + dcm[1][0]) / s;
 q_att->q[3] = (dcm[0][2] + dcm[2][0]) / s;
 } else if (dcm[1][1] > dcm[2][2]) {
 double s = 2.0 * sqrt(1.0 + dcm[1][1] - dcm[0][0] - dcm[2][2]);
 q_att->q[0] = (dcm[0][2] - dcm[2][0]) / s;
 q_att->q[1] = (dcm[0][1] + dcm[1][0]) / s;
 q_att->q[2] = 0.25 * s;
 q_att->q[3] = (dcm[1][2] + dcm[2][1]) / s;
 } else {
 double s = 2.0 * sqrt(1.0 + dcm[2][2] - dcm[0][0] - dcm[1][1]);
 q_att->q[0] = (dcm[1][0] - dcm[0][1]) / s;
 q_att->q[1] = (dcm[0][2] + dcm[2][0]) / s;
 q_att->q[2] = (dcm[1][2] + dcm[2][1]) / s;
 q_att->q[3] = 0.25 * s;
 }
 quaternion_normalize(q_att);
}

// Transform measurement vectors from body frame to ECI using attitude quaternion
// Applies antenna phase center offset and pointing correction
void transform_sensor_measurements(AttitudeQuaternion *q_att, Measurement *measurements,
 int num_meas, double sc_pos[3]) {
 /* Phase center offset varies with spacecraft position due to
 gravitational and thermal flexure — apply first-order correction */
 double alt = sqrt(sc_pos[0]*sc_pos[0] + sc_pos[1]*sc_pos[1] + sc_pos[2]*sc_pos[2]);
 double pco_shift = (alt > 6.5e6) ? 0.003 * (alt - 6.371e6) / 1e6 : 0.0; /* mm-level */

 // GNSS antenna phase center offset in body frame (meters)
 double antenna_offset_body[3] = {0.0, 0.0, -1.5 + pco_shift}; // 1.5m along zenith axis
 double antenna_offset_eci[3];

 // Rotate antenna offset from body frame to ECI
 quaternion_rotate_vector(q_att, antenna_offset_body, antenna_offset_eci);

 // Star tracker boresight in body frame (zenith-pointing for GNSS receiver)
 double boresight_body[3] = {0.0, 0.0, -1.0};
 double boresight_eci[3];
 quaternion_rotate_vector(q_att, boresight_body, boresight_eci);

 // Get body-to-inertial DCM for full rotation of observation geometry
 double dcm_b2i[3][3];
 quaternion_to_dcm(q_att, dcm_b2i);

 for (int i = 0; i < num_meas; i++) {
 // Observation direction in body frame from elevation/azimuth
 double el = measurements[i].elevation;
 double az = measurements[i].azimuth;
 double obs_body[3];
 obs_body[0] = sw_cos(el) * sw_cos(az);
 obs_body[1] = sw_cos(el) * sw_sin(az);
 obs_body[2] = sw_sin(el);

 // Transform observation direction to ECI using DCM
 double obs_eci[3];
 for (int j = 0; j < 3; j++) {
 obs_eci[j] = dcm_b2i[j][0]*obs_body[0] +
 dcm_b2i[j][1]*obs_body[1] +
 dcm_b2i[j][2]*obs_body[2];
 }

 // Angle between antenna boresight and satellite direction
 double cos_angle = boresight_eci[0]*obs_eci[0] +
 boresight_eci[1]*obs_eci[1] +
 boresight_eci[2]*obs_eci[2];

 // Apply antenna gain pattern correction (cosine roll-off model)
 double gain_factor = (cos_angle > 0.0) ? cos_angle : 0.1;
 measurements[i].weight *= gain_factor;

 // Correct pseudorange for antenna phase center offset projected onto line-of-sight
 double offset_projection = antenna_offset_eci[0]*obs_eci[0] +
 antenna_offset_eci[1]*obs_eci[1] +
 antenna_offset_eci[2]*obs_eci[2];
 measurements[i].pseudorange -= offset_projection;
 }
}

// Helper functions to prevent NaN values
double safe_acos(double x) {
 if (x > 1.0) return 0.0;
 if (x < -1.0) return PI;
 return acos(x);
}

double safe_asin(double x) {
 if (x > 1.0) return PI/2.0;
 if (x < -1.0) return -PI/2.0;
 return asin(x);
}

double safe_sqrt(double x) {
 return sqrt(fmax(0.0, x));
}

// Time conversions
typedef struct {
 int year;
 int month;
 int day;
 int hour;
 int minute;
 double second;
} DateTime;

// Julian Date calculation
double calculate_julian_date(DateTime time) {
 int y = time.year;
 int m = time.month;
 int d = time.day;
 
 // Adjust for January and February
 if (m <= 2) {
 y -= 1;
 m += 12;
 }
 
 int a = y / 100;
 int b = 2 - a + (a / 4);
 
 // Calculate Julian day
 double jd = floor(365.25 * (y + 4716)) + floor(30.6001 * (m + 1)) + d + b - 1524.5;
 
 // Add time of day
 jd += time.hour / 24.0 + time.minute / 1440.0 + time.second / 86400.0;
 
 return jd;
}

// Calculate Greenwich Mean Sidereal Time (GMST)
double calculate_gmst(double jd) {
 double t = (jd - 2451545.0) / 36525.0;
 double gmst = 67310.54841 + 
 (876600.0 * 3600.0 + 8640184.812866) * t + 
 0.093104 * t * t - 
 6.2e-6 * t * t * t;
 
 // Convert to degrees and reduce to [0, 360)
 gmst = fmod(gmst / 240.0, 360.0);
 if (gmst < 0) gmst += 360.0;
 
 return gmst * PI / 180.0; // Return in radians
}

// Conversion from ECEF to ECI

// Conversion from ECI to ECEF
void eci_to_ecef(double eci[3], double ecef[3], double gmst) {
 double cos_gmst = cos(gmst);
 double sin_gmst = sin(gmst);
 
 ecef[0] = eci[0] * cos_gmst + eci[1] * sin_gmst;
 ecef[1] = -eci[0] * sin_gmst + eci[1] * cos_gmst;
 ecef[2] = eci[2];
}

// ECEF to geodetic conversion (iterative algorithm)
void ecef_to_geodetic(double ecef[3], double *lat, double *lon, double *alt) {
 double x = ecef[0];
 double y = ecef[1];
 double z = ecef[2];
 
 // WGS84 ellipsoid parameters
 double a = 6378137.0; // semi-major axis (m)
 double f = 1.0 / 298.257223563; // flattening
 double b = a * (1.0 - f); // semi-minor axis
 double e2 = f * (2.0 - f); // eccentricity squared
 
 // Calculate longitude
 *lon = atan2(y, x);
 
 // Initial values
 double p = safe_sqrt(x*x + y*y);
 if (p < 1e-10) {
 // Handle pole case
 *lat = (z >= 0) ? PI/2.0 : -PI/2.0;
 *alt = fabs(z) - b;
 return;
 }
 
 double theta = atan2(z * a, p * b);
 
 // Calculate latitude and height using iteration
 *lat = atan2(z + e2 * b * pow(sw_sin(theta), 3), p - e2 * a * pow(sw_cos(theta), 3));
 double N = a / safe_sqrt(1.0 - e2 * sw_sin(*lat) * sw_sin(*lat));
 *alt = p / sw_cos(*lat) - N;
 
 // Refine iteration
 for (int i = 0; i < 3; i++) {
 *lat = atan2(z + e2 * N * sw_sin(*lat), p);
 N = a / safe_sqrt(1.0 - e2 * sw_sin(*lat) * sw_sin(*lat));
 *alt = p / sw_cos(*lat) - N;
 }
}

// Geodetic to ECEF conversion
void geodetic_to_ecef(double lat, double lon, double alt, double ecef[3]) {
 // WGS84 ellipsoid parameters
 double a = 6378137.0; // semi-major axis (m)
 double f = 1.0 / 298.257223563; // flattening
 double e2 = f * (2.0 - f); // eccentricity squared
 
 double N = a / safe_sqrt(1.0 - e2 * sw_sin(lat) * sw_sin(lat));
 
 ecef[0] = (N + alt) * sw_cos(lat) * sw_cos(lon);
 ecef[1] = (N + alt) * sw_cos(lat) * sw_sin(lon);
 ecef[2] = (N * (1.0 - e2) + alt) * sw_sin(lat);
}

// Convert state vector to orbital elements
void state_vector_to_orbital_elements(double r[3], double v[3], double elements[6]) {
 // Compute orbital elements from position and velocity vectors
 double mu = MU; // Earth's gravitational parameter
 
 // Magnitude of position and velocity
 double r_mag = vector_magnitude(r);
 double v_mag = vector_magnitude(v);
 
 // Angular momentum vector
 double h[3];
 vector_cross(r, v, h);
 double h_mag = vector_magnitude(h);
 
 // Check for valid orbit
 if (r_mag < 1e-10 || h_mag < 1e-10) {
 // Invalid orbit - use reasonable defaults
 elements[0] = RE + 700e3; // 700 km altitude
 elements[1] = 0.001; // Nearly circular
 elements[2] = 0.0; // Equatorial
 elements[3] = 0.0; // Arbitrary RAAN
 elements[4] = 0.0; // Arbitrary argument of perigee
 elements[5] = 0.0; // Arbitrary mean anomaly
 return;
 }
 
 // Node vector (points towards ascending node)
 double k[3] = {0.0, 0.0, 1.0}; // Unit vector in z-direction
 double n[3];
 vector_cross(k, h, n);
 double n_mag = vector_magnitude(n);
 
 // Eccentricity vector
 double e[3], temp[3];
 double r_dot_v = vector_dot(r, v);
 
 vector_cross(v, h, temp);
 vector_scale(temp, 1.0/mu, temp);
 
 double r_scaled[3];
 vector_scale(r, 1.0/r_mag, r_scaled);
 
 vector_subtract(temp, r_scaled, e);
 double e_mag = vector_magnitude(e);
 
 // Semi-major axis
 double energy = 0.5 * v_mag * v_mag - mu / r_mag;
 double a;
 
 if (fabs(energy) < 1e-10) {
 // Parabolic orbit (e = 1)
 a = INFINITY;
 e_mag = 1.0;
 } else {
 a = -mu / (2.0 * energy);
 if (a < 0) {
 // Hyperbolic orbit - cap at reasonable values for simulation
 a = RE + 10000e3; // Cap at high altitude
 e_mag = 0.9; // Set to highly eccentric ellipse
 }
 }
 
 // Inclination
 double i = safe_acos(h[2] / h_mag);
 
 // Right Ascension of the Ascending Node (RAAN)
 double Omega;
 if (n_mag < 1e-10) {
 Omega = 0.0; // For equatorial orbits, RAAN is undefined
 } else {
 Omega = safe_acos(n[0] / n_mag);
 if (n[1] < 0) Omega = 2.0 * PI - Omega;
 }
 
 // Argument of periapsis
 double omega;
 if (n_mag < 1e-10) {
 omega = atan2(e[1], e[0]); // For equatorial orbits, use e vector in xy-plane
 } else if (e_mag < 1e-10) {
 omega = 0.0; // For circular orbits, argument of periapsis is undefined
 } else {
 double n_dot_e = vector_dot(n, e);
 omega = safe_acos(n_dot_e / (n_mag * e_mag));
 if (e[2] < 0) omega = 2.0 * PI - omega;
 }
 
 // True anomaly
 double nu;
 if (e_mag < 1e-10) {
 nu = atan2(r[1], r[0]) - Omega; // For circular orbits, measure from RAAN
 if (nu < 0) nu += 2.0 * PI;
 } else {
 double e_dot_r = vector_dot(e, r);
 nu = safe_acos(e_dot_r / (e_mag * r_mag));
 if (r_dot_v < 0) nu = 2.0 * PI - nu;
 }
 
 // Convert true anomaly to mean anomaly
 double E;
 if (e_mag < 1e-10) {
 E = nu; // For circular orbits, E = nu
 } else if (e_mag >= 1.0) {
 E = asinh(tan(nu/2.0) * safe_sqrt((e_mag+1.0)/(e_mag-1.0)));
 } else {
 E = 2.0 * atan(safe_sqrt((1.0-e_mag)/(1.0+e_mag)) * tan(nu/2.0)); // Eccentric anomaly
 }
 
 double M;
 if (e_mag >= 1.0) {
 M = e_mag * sinh(E) - E; // Hyperbolic mean anomaly
 } else {
 M = E - e_mag * sw_sin(E); // Mean anomaly
 }
 
 // Store orbital elements (ensure valid values)
 elements[0] = a; // Semi-major axis (m)
 elements[1] = fmin(0.999, e_mag); // Eccentricity (cap at 0.999 for stability)
 elements[2] = i; // Inclination (rad)
 elements[3] = fmod(Omega, 2.0*PI); // RAAN (rad)
 elements[4] = fmod(omega, 2.0*PI); // Argument of periapsis (rad)
 elements[5] = fmod(M + 2.0*PI, 2.0*PI); // Mean anomaly (rad) (ensure positive)
}

// Convert orbital elements to state vector
void orbital_elements_to_state_vector(double elements[6], double r[3], double v[3]) {
 double a = elements[0]; // Semi-major axis (m)
 double e = elements[1]; // Eccentricity
 double i = elements[2]; // Inclination (rad)
 double Omega = elements[3]; // RAAN (rad)
 double omega = elements[4]; // Argument of periapsis (rad)
 double M = elements[5]; // Mean anomaly (rad)
 
 // Check for valid elements
 if (a <= 0.0 || e >= 1.0 || !isfinite(a)) {
 // Invalid elements, use default LEO orbit
 a = RE + 700e3;
 e = 0.001;
 // Keep other elements as is
 }
 
 // Solve Kepler's equation for eccentric anomaly
 double E = M; // Initial guess
 double dE;
 
 // Newton-Raphson iteration
 for (int iter = 0; iter < 20; iter++) {
 dE = (E - e * sw_sin(E) - M) / (1.0 - e * sw_cos(E));
 E -= dE;
 if (fabs(dE) < 1e-12) break;
 }
 
 // Calculate position and velocity in orbital plane
 double cos_E = sw_cos(E);
 double sin_E = sw_sin(E);
 
 // Position in orbital frame
 double x_orb = a * (cos_E - e);
 double y_orb = a * safe_sqrt(1.0 - e*e) * sin_E;
 double z_orb = 0.0;
 
 // Velocity in orbital frame
 double mu = MU;
 double factor = sqrt(mu / a) / (1.0 - e * cos_E);
 double vx_orb = -factor * sin_E;
 double vy_orb = factor * safe_sqrt(1.0 - e*e) * cos_E;
 double vz_orb = 0.0;
 
 // Rotation matrices for orbital plane to ECI transformation
 double cos_i = sw_cos(i);
 double sin_i = sw_sin(i);
 double cos_Omega = sw_cos(Omega);
 double sin_Omega = sw_sin(Omega);
 double cos_omega = sw_cos(omega);
 double sin_omega = sw_sin(omega);
 
 // Apply rotation for argument of periapsis
 double x_temp = x_orb * cos_omega - y_orb * sin_omega;
 double y_temp = x_orb * sin_omega + y_orb * cos_omega;
 double z_temp = z_orb;
 
 double vx_temp = vx_orb * cos_omega - vy_orb * sin_omega;
 double vy_temp = vx_orb * sin_omega + vy_orb * cos_omega;
 double vz_temp = vz_orb;
 
 // Apply rotation for inclination
 double x_temp2 = x_temp;
 double y_temp2 = y_temp * cos_i - z_temp * sin_i;
 double z_temp2 = y_temp * sin_i + z_temp * cos_i;
 
 double vx_temp2 = vx_temp;
 double vy_temp2 = vy_temp * cos_i - vz_temp * sin_i;
 double vz_temp2 = vy_temp * sin_i + vz_temp * cos_i;
 
 // Apply rotation for RAAN
 r[0] = x_temp2 * cos_Omega - y_temp2 * sin_Omega;
 r[1] = x_temp2 * sin_Omega + y_temp2 * cos_Omega;
 r[2] = z_temp2;
 
 v[0] = vx_temp2 * cos_Omega - vy_temp2 * sin_Omega;
 v[1] = vx_temp2 * sin_Omega + vy_temp2 * cos_Omega;
 v[2] = vz_temp2;
}

/* ---------------------------------------------------------------
 * Low-precision solar ephemeris (Meeus, "Astronomical Algorithms")
 * Returns the geocentric Sun position in ECI (metres).
 * Accuracy ≈ 0.01° over 1900-2100 — more than sufficient for SRP.
 * --------------------------------------------------------------- */
static void compute_sun_eci(double jd, double sun_eci[3]) {
 /* Julian centuries from J2000.0 */
 double T = (jd - 2451545.0) / 36525.0;

 /* Mean longitude (deg) */
 double L0 = fmod(280.46646 + 36000.76983 * T + 0.0003032 * T * T, 360.0);
 if (L0 < 0.0) L0 += 360.0;

 /* Mean anomaly (deg) */
 double M = fmod(357.52911 + 35999.05029 * T - 0.0001537 * T * T, 360.0);
 if (M < 0.0) M += 360.0;
 double M_rad = M * DEG2RAD;

 /* Equation of center (deg) */
 double C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * sw_sin(M_rad)
 + (0.019993 - 0.000101 * T) * sw_sin(2.0 * M_rad)
 + 0.000289 * sw_sin(3.0 * M_rad);

 /* Sun's true longitude and true anomaly (deg) */
 double sun_lon = (L0 + C) * DEG2RAD;

 /* Sun–Earth distance (AU) */
 double v_deg = M + C;
 double v_rad = v_deg * DEG2RAD;
 double R_AU = (1.000001018 * (1.0 - 0.016708634 * 0.016708634))
 / (1.0 + 0.016708634 * sw_cos(v_rad));
 double R_m = R_AU * AU_M;

 /* Mean obliquity of the ecliptic (deg → rad) */
 double eps = (23.439291 - 0.0130042 * T) * DEG2RAD;

 /* Geocentric equatorial (ECI) coordinates of the Sun */
 sun_eci[0] = R_m * sw_cos(sun_lon);
 sun_eci[1] = R_m * sw_sin(sun_lon) * sw_cos(eps);
 sun_eci[2] = R_m * sw_sin(sun_lon) * sw_sin(eps);
}

/* ---------------------------------------------------------------
 * Low-precision lunar ephemeris (simplified Brown / Meeus Ch. 47)
 * Returns the geocentric Moon position in ECI (metres).
 * --------------------------------------------------------------- */
static void compute_moon_eci(double jd, double moon_eci[3]) {
 double T = (jd - 2451545.0) / 36525.0;

 /* Fundamental arguments (degrees) */
 double Lp = fmod(218.3165 + 481267.8813 * T, 360.0);
 if (Lp < 0.0) Lp += 360.0;
 double D = fmod(297.8502 + 445267.1115 * T, 360.0);
 if (D < 0.0) D += 360.0;
 double Mp = fmod(134.9634 + 477198.8676 * T, 360.0);
 if (Mp < 0.0) Mp += 360.0;
 double F = fmod(93.2721 + 483202.0175 * T, 360.0);
 if (F < 0.0) F += 360.0;

 double D_r = D * DEG2RAD;
 double Mp_r = Mp * DEG2RAD;
 double F_r = F * DEG2RAD;

 /* Ecliptic longitude and latitude (degrees) */
 double lon = Lp
 + 6.289 * sw_sin(Mp_r)
 - 1.274 * sw_sin(2.0 * D_r - Mp_r)
 + 0.658 * sw_sin(2.0 * D_r)
 + 0.214 * sw_sin(2.0 * Mp_r)
 - 0.186 * sw_sin(D_r);
 double lat = 5.128 * sw_sin(F_r)
 + 0.281 * sw_sin(Mp_r + F_r)
 - 0.278 * sw_sin(F_r - Mp_r);

 /* Distance (km → m) */
 double dist_km = 385001.0
 - 20905.0 * sw_cos(Mp_r)
 - 3699.0 * sw_cos(2.0 * D_r - Mp_r)
 - 2956.0 * sw_cos(2.0 * D_r);
 double dist_m = dist_km * 1000.0;

 double lon_r = lon * DEG2RAD;
 double lat_r = lat * DEG2RAD;

 /* Mean obliquity */
 double eps = (23.439291 - 0.0130042 * T) * DEG2RAD;

 /* Ecliptic → ECI */
 double xecl = dist_m * sw_cos(lat_r) * sw_cos(lon_r);
 double yecl = dist_m * sw_cos(lat_r) * sw_sin(lon_r);
 double zecl = dist_m * sw_sin(lat_r);

 moon_eci[0] = xecl;
 moon_eci[1] = yecl * sw_cos(eps) - zecl * sw_sin(eps);
 moon_eci[2] = yecl * sw_sin(eps) + zecl * sw_cos(eps);
}

/* ---------------------------------------------------------------
 * Cylindrical Earth-shadow test.
 * Returns 1.0 if the spacecraft is sunlit, 0.0 if eclipsed.
 * Uses a simple cylinder model with the Earth radius.
 * --------------------------------------------------------------- */
static double earth_shadow_factor(const double pos[3], const double sun_eci[3]) {
 /* Unit vector from Earth to Sun */
 double sun_mag = sqrt(sun_eci[0]*sun_eci[0] + sun_eci[1]*sun_eci[1] + sun_eci[2]*sun_eci[2]);
 if (sun_mag < 1.0) return 1.0; /* safety */
 double s_hat[3] = { sun_eci[0]/sun_mag, sun_eci[1]/sun_mag, sun_eci[2]/sun_mag };

 /* Project spacecraft position onto sun direction */
 double proj = pos[0]*s_hat[0] + pos[1]*s_hat[1] + pos[2]*s_hat[2];

 /* If projection is positive, spacecraft is on the sunward side → sunlit */
 if (proj >= 0.0) return 1.0;

 /* Perpendicular distance from Earth–Sun line */
 double perp_sq = (pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]) - proj*proj;
 if (perp_sq < RE * RE) return 0.0; /* inside cylindrical shadow */

 return 1.0;
}

// Calculate accelerations due to forces other than point-mass gravity
void calculate_perturbations(double pos[3], double vel[3], double acc[3]) {
 // Initialize acceleration vector to zero
 acc[0] = 0.0;
 acc[1] = 0.0;
 acc[2] = 0.0;
 
 // Check for valid position
 double r = vector_magnitude(pos);
 if (r < RE || !isfinite(r)) {
 // Invalid position, return zero acceleration
 return;
 }
 
 double r2 = r * r;
 double r5 = r2 * r2 * r;
 
 double x = pos[0];
 double y = pos[1];
 double z = pos[2];
 double z2 = z * z;
 
 // J2 perturbation
 double j2_factor = 1.5 * J2 * MU * RE * RE / r5;
 double j2_common = 5.0 * z2 / r2 - 1.0;
 
 acc[0] += j2_factor * x * j2_common;
 acc[1] += j2_factor * y * j2_common;
 acc[2] += j2_factor * z * (5.0 * z2 / r2 - 3.0);
 
 // Solar radiation pressure perturbation
 // Compute Sun position from low-precision solar ephemeris (Meeus)
 double sun_eci[3];
 compute_sun_eci(g_epoch_jd, sun_eci);

 // Unit vector from spacecraft to Sun
 double sc_to_sun[3] = { sun_eci[0] - pos[0],
 sun_eci[1] - pos[1],
 sun_eci[2] - pos[2] };
 double sc_to_sun_mag = sqrt(sc_to_sun[0]*sc_to_sun[0]
 + sc_to_sun[1]*sc_to_sun[1]
 + sc_to_sun[2]*sc_to_sun[2]);
 double sun_dir[3] = {0.0, 0.0, 0.0};
 if (sc_to_sun_mag > 1.0) {
 sun_dir[0] = sc_to_sun[0] / sc_to_sun_mag;
 sun_dir[1] = sc_to_sun[1] / sc_to_sun_mag;
 sun_dir[2] = sc_to_sun[2] / sc_to_sun_mag;
 }

 // Eclipse check — zero SRP when spacecraft is in Earth's shadow
 double shadow = earth_shadow_factor(pos, sun_eci);

 double spacecraft_area = 10.0; // m²
 double spacecraft_mass = 1000.0; // kg
 double solar_pressure = 4.56e-6; // N/m²
 double srp_factor = shadow * solar_pressure * spacecraft_area / spacecraft_mass;

 acc[0] += srp_factor * sun_dir[0];
 acc[1] += srp_factor * sun_dir[1];
 acc[2] += srp_factor * sun_dir[2];
 
 // Atmospheric drag perturbation
 // Exponential atmospheric density model
 double rho0 = 3.614e-13; // kg/m³ (density at 700 km)
 double h0 = 700000.0; // m (reference altitude)
 double H = 88667.0; // m (scale height at 700 km)
 
 double alt = r - RE;
 if (alt > 0) { // Only apply drag above Earth's surface
 double density = rho0 * exp(-(alt - h0) / H);
 
 double v_mag = vector_magnitude(vel);
 if (v_mag > 0) { // Avoid division by zero
 double Cd = 2.2; // drag coefficient
 double drag_factor = -0.5 * density * Cd * spacecraft_area / spacecraft_mass * v_mag;
 
 acc[0] += drag_factor * vel[0];
 acc[1] += drag_factor * vel[1];
 acc[2] += drag_factor * vel[2];
 }
 }
 
 // Third-body lunar gravitational perturbation
 // Compute Moon position from low-precision lunar ephemeris (Meeus Ch.47)
 double moon_pos[3];
 compute_moon_eci(g_epoch_jd, moon_pos);
 
 double sc_to_moon[3];
 vector_subtract(moon_pos, pos, sc_to_moon);
 double r_moon = vector_magnitude(sc_to_moon);
 
 if (r_moon > 1000.0) { // Avoid singularity
 double moon_factor = MU_MOON / (r_moon * r_moon * r_moon);
 acc[0] += moon_factor * sc_to_moon[0];
 acc[1] += moon_factor * sc_to_moon[1];
 acc[2] += moon_factor * sc_to_moon[2];
 }
}

// Add a helper function to check for valid states and prevent NaN/infinity
static int is_valid_state(double *vector, int size) {
 for (int i = 0; i < size; i++) {
 if (!isfinite(vector[i])) {
 return 0;
 }
 }
 return 1;
}

// Modify propagate_orbit to improve numerical stability
void propagate_orbit(SpacecraftState *state, double dt) {
 double r0[3], v0[3], a0[3];
 double r1[3], v1[3], a1[3];
 double r2[3], v2[3], a2[3];
 double r3[3], v3[3], a3[3];
 
 double kr1[3], kv1[3];
 double kr2[3], kv2[3];
 double kr3[3], kv3[3];
 double kr4[3], kv4[3];
 
 // Copy initial state
 memcpy(r0, state->position, sizeof(r0));
 memcpy(v0, state->velocity, sizeof(v0));
 
 // Check for valid initial state
 double r_mag = vector_magnitude(r0);
 if (r_mag < RE || !is_valid_state(r0, 3) || !is_valid_state(v0, 3)) {
 // Invalid initial state, reset to reasonable orbit
 state->position[0] = RE + 700e3;
 state->position[1] = 0.0;
 state->position[2] = 0.0;
 
 state->velocity[0] = 0.0;
 state->velocity[1] = sqrt(MU / (RE + 700e3));
 state->velocity[2] = 0.0;
 
 state->clock_bias = 0.0;
 state->clock_drift = 0.0;
 
 memcpy(r0, state->position, sizeof(r0));
 memcpy(v0, state->velocity, sizeof(v0));
 r_mag = vector_magnitude(r0);
 }
 
 // Calculate initial acceleration
 double r_cubed = r_mag * r_mag * r_mag;
 
 // Two-body acceleration
 for (int i = 0; i < 3; i++) {
 a0[i] = -MU * r0[i] / r_cubed;
 }
 
 // Add perturbations
 double perturbations[3];
 calculate_perturbations(r0, v0, perturbations);
 for (int i = 0; i < 3; i++) {
 if (!isfinite(perturbations[i])) {
 perturbations[i] = 0.0; // Prevent NaN perturbations
 }
 a0[i] += perturbations[i];
 }
 
 // First step
 for (int i = 0; i < 3; i++) {
 kr1[i] = dt * v0[i];
 kv1[i] = dt * a0[i];
 r1[i] = r0[i] + 0.5 * kr1[i];
 v1[i] = v0[i] + 0.5 * kv1[i];
 }
 
 // Calculate acceleration at midpoint 1
 r_mag = vector_magnitude(r1);
 if (r_mag < RE || !is_valid_state(r1, 3) || !is_valid_state(v1, 3)) {
 // If midpoint is invalid, use simpler propagation
 for (int i = 0; i < 3; i++) {
 state->position[i] = r0[i] + v0[i] * dt;
 state->velocity[i] = v0[i] + a0[i] * dt;
 }
 } else {
 r_cubed = r_mag * r_mag * r_mag;
 
 for (int i = 0; i < 3; i++) {
 a1[i] = -MU * r1[i] / r_cubed;
 }
 
 calculate_perturbations(r1, v1, perturbations);
 for (int i = 0; i < 3; i++) {
 if (!isfinite(perturbations[i])) {
 perturbations[i] = 0.0;
 }
 a1[i] += perturbations[i];
 }
 
 // Second step
 for (int i = 0; i < 3; i++) {
 kr2[i] = dt * v1[i];
 kv2[i] = dt * a1[i];
 r2[i] = r0[i] + 0.5 * kr2[i];
 v2[i] = v0[i] + 0.5 * kv2[i];
 }
 
 // Calculate acceleration at midpoint 2
 r_mag = vector_magnitude(r2);
 if (r_mag < RE || !is_valid_state(r2, 3) || !is_valid_state(v2, 3)) {
 // If midpoint is invalid, use simpler propagation
 for (int i = 0; i < 3; i++) {
 state->position[i] = r0[i] + v0[i] * dt;
 state->velocity[i] = v0[i] + a0[i] * dt;
 }
 } else {
 r_cubed = r_mag * r_mag * r_mag;
 
 for (int i = 0; i < 3; i++) {
 a2[i] = -MU * r2[i] / r_cubed;
 }
 
 calculate_perturbations(r2, v2, perturbations);
 for (int i = 0; i < 3; i++) {
 if (!isfinite(perturbations[i])) {
 perturbations[i] = 0.0;
 }
 a2[i] += perturbations[i];
 }
 
 // Third step
 for (int i = 0; i < 3; i++) {
 kr3[i] = dt * v2[i];
 kv3[i] = dt * a2[i];
 r3[i] = r0[i] + kr3[i];
 v3[i] = v0[i] + kv3[i];
 }
 
 // Calculate acceleration at endpoint
 r_mag = vector_magnitude(r3);
 if (r_mag < RE || !is_valid_state(r3, 3) || !is_valid_state(v3, 3)) {
 // If endpoint is invalid, use simpler propagation
 for (int i = 0; i < 3; i++) {
 state->position[i] = r0[i] + v0[i] * dt;
 state->velocity[i] = v0[i] + a0[i] * dt;
 }
 } else {
 r_cubed = r_mag * r_mag * r_mag;
 
 for (int i = 0; i < 3; i++) {
 a3[i] = -MU * r3[i] / r_cubed;
 }
 
 calculate_perturbations(r3, v3, perturbations);
 for (int i = 0; i < 3; i++) {
 if (!isfinite(perturbations[i])) {
 perturbations[i] = 0.0;
 }
 a3[i] += perturbations[i];
 }
 
 // Fourth step
 for (int i = 0; i < 3; i++) {
 kr4[i] = dt * v3[i];
 kv4[i] = dt * a3[i];
 }
 
 // Combine steps
 for (int i = 0; i < 3; i++) {
 state->position[i] = r0[i] + (kr1[i] + 2.0*kr2[i] + 2.0*kr3[i] + kr4[i]) / 6.0;
 state->velocity[i] = v0[i] + (kv1[i] + 2.0*kv2[i] + 2.0*kv3[i] + kv4[i]) / 6.0;
 }
 }
 }
 }
 
 // Verify final position is not below Earth's surface
 r_mag = vector_magnitude(state->position);
 if (r_mag < RE || !is_valid_state(state->position, 3) || !is_valid_state(state->velocity, 3)) {
 // Reset to circular orbit at safe altitude
 state->position[0] = RE + 700e3;
 state->position[1] = 0.0;
 state->position[2] = 0.0;
 
 state->velocity[0] = 0.0;
 state->velocity[1] = sqrt(MU / (RE + 700e3));
 state->velocity[2] = 0.0;
 
 state->clock_bias = 0.0;
 state->clock_drift = 0.0;
 }
 
 // Update acceleration for output
 r_mag = vector_magnitude(state->position);
 r_cubed = r_mag * r_mag * r_mag;
 
 for (int i = 0; i < 3; i++) {
 state->acceleration[i] = -MU * state->position[i] / r_cubed;
 }
 
 calculate_perturbations(state->position, state->velocity, perturbations);
 for (int i = 0; i < 3; i++) {
 if (!isfinite(perturbations[i])) {
 perturbations[i] = 0.0;
 }
 state->acceleration[i] += perturbations[i];
 }
 
 // Update orbital elements
 state_vector_to_orbital_elements(state->position, state->velocity, state->orbital_elements);
 
 // Update clock with bounds checking
 if (isfinite(state->clock_drift) && isfinite(state->clock_bias)) {
 state->clock_bias += state->clock_drift * dt;
 // Limit clock bias to prevent overflow
 if (fabs(state->clock_bias) > 1e-3) {
 state->clock_bias = state->clock_bias > 0 ? 1e-3 : -1e-3;
 }
 
 // Update clock drift with random walk, limit magnitude
 double drift_change = ((double)rand() / RAND_MAX - 0.5) * 1.0e-10 * dt;
 if (isfinite(drift_change)) {
 state->clock_drift += drift_change;
 if (fabs(state->clock_drift) > 1e-8) {
 state->clock_drift = state->clock_drift > 0 ? 1e-8 : -1e-8;
 }
 }
 } else {
 state->clock_bias = 0.0;
 state->clock_drift = 0.0;
 }
}

// Generate simulated GNSS satellite constellation
void generate_gnss_constellation(GNSSSatellite *satellites, int num_satellites, double jd) {
 // GPS constellation (Walker 24/6/1)
 double a = 26560e3; // Semi-major axis
 double e = 0.01; // Eccentricity
 double i = 55.0 * PI/180.0; // Inclination
 
 for (int sat = 0; sat < num_satellites; sat++) {
 // Assign PRN
 satellites[sat].prn = sat + 1;
 
 // Calculate orbital plane and slot
 int plane = sat % 6;
 int slot = sat / 6;
 
 // RAAN for each plane (evenly distributed)
 double Omega = plane * 60.0 * PI/180.0;
 
 // Argument of latitude for each slot (evenly distributed within plane)
 double u = slot * 72.0 * PI/180.0 + plane * 10.0 * PI/180.0;
 
 // Split into argument of perigee and mean anomaly
 double omega = 0.0; // For near-circular orbits, can be assumed zero
 double M = u; // For near-circular orbits, u ≈ M
 
 // Define orbital elements
 double elements[6] = {a, e, i, Omega, omega, M};
 
 // Convert to state vector
 orbital_elements_to_state_vector(elements, satellites[sat].position, satellites[sat].velocity);
 
 // Convert from ECI to ECEF
 double gmst = calculate_gmst(jd);
 double pos_eci[3], vel_eci[3];
 memcpy(pos_eci, satellites[sat].position, sizeof(pos_eci));
 memcpy(vel_eci, satellites[sat].velocity, sizeof(vel_eci));
 
 eci_to_ecef(pos_eci, satellites[sat].position, gmst);
 
 // For velocity, need to account for Earth rotation
 double earth_rot[3] = {0.0, 0.0, OMEGA_EARTH};
 double rot_effect[3];
 vector_cross(earth_rot, pos_eci, rot_effect);
 vector_subtract(vel_eci, rot_effect, vel_eci);
 eci_to_ecef(vel_eci, satellites[sat].velocity, gmst);
 
 // Add clock errors
 satellites[sat].clock_bias = ((double)rand() / RAND_MAX - 0.5) * 1.0e-6; // ~microsecond level
 satellites[sat].clock_drift = ((double)rand() / RAND_MAX - 0.5) * 1.0e-9; // ~nanosecond/s level
 }
}

// Modify generate_measurements to ensure at least some measurements
void generate_measurements(SpacecraftState *state, GNSSSatellite *satellites, 
 int num_satellites, double jd, Measurement *measurements, 
 int *num_measurements) {
 // Convert spacecraft position to ECEF
 double sc_pos_ecef[3];
 double gmst = calculate_gmst(jd);
 eci_to_ecef(state->position, sc_pos_ecef, gmst);
 
 *num_measurements = 0;
 
 for (int i = 0; i < num_satellites && *num_measurements < MAX_MEASUREMENTS; i++) {
 // Vector from spacecraft to satellite
 double range_vector[3];
 vector_subtract(satellites[i].position, sc_pos_ecef, range_vector);
 double range = vector_magnitude(range_vector);
 
 // Relax range check to allow more measurements
 if (range > 200000e3 || !isfinite(range)) {
 continue;
 }
 
 // Unit vector in direction of satellite
 double unit_vector[3];
 for (int j = 0; j < 3; j++) {
 unit_vector[j] = range_vector[j] / range;
 }
 
 // Calculate elevation angle
 double lat, lon, alt;
 ecef_to_geodetic(sc_pos_ecef, &lat, &lon, &alt);
 
 // Local up direction in ECEF
 double up[3];
 geodetic_to_ecef(lat, lon, alt + 1000.0, up);
 vector_subtract(up, sc_pos_ecef, up);
 vector_normalize(up);
 
 double elevation = safe_asin(vector_dot(unit_vector, up));
 
 // Calculate azimuth
 double north[3];
 double dlat = 0.001; // Small change in latitude
 double north_pos[3];
 geodetic_to_ecef(lat + dlat, lon, alt, north_pos);
 vector_subtract(north_pos, sc_pos_ecef, north);
 vector_normalize(north);
 
 double east[3];
 vector_cross(up, north, east);
 vector_normalize(east);
 
 double azimuth = atan2(vector_dot(unit_vector, east), vector_dot(unit_vector, north));
 if (azimuth < 0) azimuth += 2.0 * PI;
 
 // Relax elevation mask to 5 degrees to increase visibility
 if (elevation * 180.0 / PI > 5.0) {
 measurements[*num_measurements].prn = satellites[i].prn;
 
 // Calculate signal transit time
 double transit_time = range / C_LIGHT;
 
 // Account for Earth rotation during signal transit
 double rotation_angle = OMEGA_EARTH * transit_time;
 double sat_pos_rotated[3];
 sat_pos_rotated[0] = satellites[i].position[0] * sw_cos(rotation_angle) + 
 satellites[i].position[1] * sw_sin(rotation_angle);
 sat_pos_rotated[1] = -satellites[i].position[0] * sw_sin(rotation_angle) + 
 satellites[i].position[1] * sw_cos(rotation_angle);
 sat_pos_rotated[2] = satellites[i].position[2];
 
 // Recalculate range with rotated satellite position
 vector_subtract(sat_pos_rotated, sc_pos_ecef, range_vector);
 range = vector_magnitude(range_vector);
 
 // Calculate pseudorange with clock errors
 double pseudorange = range + 
 C_LIGHT * (state->clock_bias - satellites[i].clock_bias);
 
 // Apply satellite clock correction (as done in real GNSS receivers)
 pseudorange += C_LIGHT * satellites[i].clock_bias;
 
 // Add measurement noise
 double sigma_range = 2.0; // 2 meters pseudorange noise
 pseudorange += ((double)rand() / RAND_MAX - 0.5) * 2.0 * sigma_range;
 
 // Calculate Doppler
 double relative_vel[3];
 vector_subtract(satellites[i].velocity, state->velocity, relative_vel);
 double radial_velocity = vector_dot(relative_vel, unit_vector);
 double doppler = -radial_velocity / (C_LIGHT / 1575.42e6); // For L1 signal
 
 // Add Doppler noise
 double sigma_doppler = 0.1; // 0.1 Hz doppler noise
 doppler += ((double)rand() / RAND_MAX - 0.5) * 2.0 * sigma_doppler;
 
 // Store measurements
 measurements[*num_measurements].pseudorange = pseudorange;
 measurements[*num_measurements].doppler = doppler;
 measurements[*num_measurements].elevation = elevation;
 measurements[*num_measurements].azimuth = azimuth;
 measurements[*num_measurements].weight = 1.0 / (sigma_range * sigma_range); // Inverse variance weighting
 
 // Store GNSS satellite position in ECI (rotate ECEF back to ECI)
 {
 double cgmst = sw_cos(gmst), sgmst = sw_sin(gmst);
 measurements[*num_measurements].sat_pos_eci[0] = sat_pos_rotated[0]*cgmst - sat_pos_rotated[1]*sgmst;
 measurements[*num_measurements].sat_pos_eci[1] = sat_pos_rotated[0]*sgmst + sat_pos_rotated[1]*cgmst;
 measurements[*num_measurements].sat_pos_eci[2] = sat_pos_rotated[2];
 }
 
 (*num_measurements)++;
 }
 }
 
 // Ensure at least 4 measurements by relaxing constraints if necessary
 if (*num_measurements < 4) {
 *num_measurements = 0;
 for (int i = 0; i < num_satellites && *num_measurements < 4; i++) {
 double range_vector[3];
 vector_subtract(satellites[i].position, sc_pos_ecef, range_vector);
 double range = vector_magnitude(range_vector);
 
 if (!isfinite(range)) {
 continue;
 }
 
 double unit_vector[3];
 for (int j = 0; j < 3; j++) {
 unit_vector[j] = range_vector[j] / range;
 }
 
 double lat, lon, alt;
 ecef_to_geodetic(sc_pos_ecef, &lat, &lon, &alt);
 
 double up[3];
 geodetic_to_ecef(lat, lon, alt + 1000.0, up);
 vector_subtract(up, sc_pos_ecef, up);
 vector_normalize(up);
 
 double elevation = safe_asin(vector_dot(unit_vector, up));
 
 double north[3];
 double dlat = 0.001;
 double north_pos[3];
 geodetic_to_ecef(lat + dlat, lon, alt, north_pos);
 vector_subtract(north_pos, sc_pos_ecef, north);
 vector_normalize(north);
 
 double east[3];
 vector_cross(up, north, east);
 vector_normalize(east);
 
 double azimuth = atan2(vector_dot(unit_vector, east), vector_dot(unit_vector, north));
 if (azimuth < 0) azimuth += 2.0 * PI;
 
 measurements[*num_measurements].prn = satellites[i].prn;
 
 double transit_time = range / C_LIGHT;
 
 double rotation_angle = OMEGA_EARTH * transit_time;
 double sat_pos_rotated[3];
 sat_pos_rotated[0] = satellites[i].position[0] * sw_cos(rotation_angle) + 
 satellites[i].position[1] * sw_sin(rotation_angle);
 sat_pos_rotated[1] = -satellites[i].position[0] * sw_sin(rotation_angle) + 
 satellites[i].position[1] * sw_cos(rotation_angle);
 sat_pos_rotated[2] = satellites[i].position[2];
 
 vector_subtract(sat_pos_rotated, sc_pos_ecef, range_vector);
 range = vector_magnitude(range_vector);
 
 double pseudorange = range + 
 C_LIGHT * (state->clock_bias - satellites[i].clock_bias);
 
 // Apply satellite clock correction (as done in real GNSS receivers)
 pseudorange += C_LIGHT * satellites[i].clock_bias;
 
 double sigma_range = 5.0; // Increased noise for relaxed constraints
 pseudorange += ((double)rand() / RAND_MAX - 0.5) * 2.0 * sigma_range;
 
 double relative_vel[3];
 vector_subtract(satellites[i].velocity, state->velocity, relative_vel);
 double radial_velocity = vector_dot(relative_vel, unit_vector);
 double doppler = -radial_velocity / (C_LIGHT / 1575.42e6);
 
 double sigma_doppler = 0.2; // Increased noise
 doppler += ((double)rand() / RAND_MAX - 0.5) * 2.0 * sigma_doppler;
 
 measurements[*num_measurements].pseudorange = pseudorange;
 measurements[*num_measurements].doppler = doppler;
 measurements[*num_measurements].elevation = elevation;
 measurements[*num_measurements].azimuth = azimuth;
 measurements[*num_measurements].weight = 1.0 / (sigma_range * sigma_range);
 
 // Store GNSS satellite position in ECI (rotate ECEF back to ECI)
 {
 double cgmst = sw_cos(gmst), sgmst = sw_sin(gmst);
 measurements[*num_measurements].sat_pos_eci[0] = sat_pos_rotated[0]*cgmst - sat_pos_rotated[1]*sgmst;
 measurements[*num_measurements].sat_pos_eci[1] = sat_pos_rotated[0]*sgmst + sat_pos_rotated[1]*cgmst;
 measurements[*num_measurements].sat_pos_eci[2] = sat_pos_rotated[2];
 }
 
 (*num_measurements)++;
 }
 }
}

// Modify least_squares_orbit_determination to improve matrix conditioning
void least_squares_orbit_determination(SpacecraftState *state, Measurement *measurements, 
 int num_measurements, double dt) {
 (void)dt;
 if (num_measurements < 4) {
 FLIGHT_LOG("Not enough measurements for orbit determination\n");
 return;
 }
 
 // State vector: [x, y, z, vx, vy, vz, clock_bias, clock_drift]
 double x[8];
 x[0] = state->position[0];
 x[1] = state->position[1];
 x[2] = state->position[2];
 x[3] = state->velocity[0];
 x[4] = state->velocity[1];
 x[5] = state->velocity[2];
 x[6] = state->clock_bias;
 x[7] = state->clock_drift;
 
 // Allocate matrices
 double H[MAX_MEASUREMENTS][8];
 double W[MAX_MEASUREMENTS][MAX_MEASUREMENTS];
 double y[MAX_MEASUREMENTS];
 double HTW[8][MAX_MEASUREMENTS];
 double HTWH[8][8];
 double HTWH_inv[8][8];
 double HTWy[8];
 double dx[8];
 
 // Initialize weight matrix (diagonal)
 for (int i = 0; i < MAX_MEASUREMENTS; i++) {
 for (int j = 0; j < MAX_MEASUREMENTS; j++) {
 W[i][j] = (i == j && i < num_measurements) ? measurements[i].weight : 0.0;
 }
 }
 
 // Increase regularization for better conditioning
 double lambda = 1e-4;
 
 // Iterative least squares
 for (int iter = 0; iter < 10; iter++) {
 // Predict measurements
 for (int i = 0; i < num_measurements; i++) {
 double sat_pos[3] = {
 measurements[i].sat_pos_eci[0],
 measurements[i].sat_pos_eci[1],
 measurements[i].sat_pos_eci[2]
 };
 
 double range_vector[3];
 vector_subtract(sat_pos, x, range_vector);
 double geometric_range = vector_magnitude(range_vector);
 
 if (geometric_range < 1e-10) {
 geometric_range = 1.0;
 }
 
 double pred_pseudorange = geometric_range + C_LIGHT * x[6];
 
 y[i] = measurements[i].pseudorange - pred_pseudorange;
 
 if (fabs(y[i]) > 10000.0) {
 y[i] = (y[i] > 0) ? 10000.0 : -10000.0;
 }
 
 for (int j = 0; j < 3; j++) {
 H[i][j] = -range_vector[j] / geometric_range;
 H[i][j+3] = 0.0;
 }
 H[i][6] = C_LIGHT;
 H[i][7] = 0.0;
 }
 
 // Calculate H' * W
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < num_measurements; j++) {
 HTW[i][j] = 0.0;
 for (int k = 0; k < num_measurements; k++) {
 HTW[i][j] += H[k][i] * W[k][j];
 }
 }
 }
 
 // Calculate H' * W * H
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 HTWH[i][j] = 0.0;
 for (int k = 0; k < num_measurements; k++) {
 HTWH[i][j] += HTW[i][k] * H[k][j];
 }
 }
 }
 
 // Add regularization
 for (int i = 0; i < 8; i++) {
 HTWH[i][i] += lambda;
 }
 
 // Scale matrix
 double scale_factors[8] = {1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3, 1e7, 1e10};
 double scaled_HTWH[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 scaled_HTWH[i][j] = HTWH[i][j] * scale_factors[i] * scale_factors[j];
 }
 }
 
 // Calculate inverse
 double scaled_HTWH_inv[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 small_matrix_inverse(scaled_HTWH, scaled_HTWH_inv, 8);
 
 // Unscale the inverse
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 HTWH_inv[i][j] = scaled_HTWH_inv[i][j] * scale_factors[i] * scale_factors[j];
 }
 }
 
 // Calculate H' * W * y
 for (int i = 0; i < 8; i++) {
 HTWy[i] = 0.0;
 for (int j = 0; j < num_measurements; j++) {
 HTWy[i] += HTW[i][j] * y[j];
 }
 }
 
 // Calculate state correction
 for (int i = 0; i < 8; i++) {
 dx[i] = 0.0;
 for (int j = 0; j < 8; j++) {
 dx[i] += HTWH_inv[i][j] * HTWy[j];
 }
 
 if (!isfinite(dx[i])) {
 dx[i] = 0.0;
 } else if (i < 3 && fabs(dx[i]) > 1000.0) { // Reduced limit for stability
 dx[i] = (dx[i] > 0) ? 1000.0 : -1000.0;
 } else if (i >= 3 && i < 6 && fabs(dx[i]) > 10.0) {
 dx[i] = (dx[i] > 0) ? 10.0 : -10.0;
 } else if (i == 6 && fabs(dx[i]) > 1e-7) {
 dx[i] = (dx[i] > 0) ? 1e-7 : -1e-7;
 } else if (i == 7 && fabs(dx[i]) > 1e-9) {
 dx[i] = (dx[i] > 0) ? 1e-9 : -1e-9;
 }
 }
 
 // Apply correction
 for (int i = 0; i < 8; i++) {
 x[i] += dx[i];
 }
 
 // Check convergence
 double norm_dx = 0.0;
 for (int i = 0; i < 8; i++) {
 norm_dx += dx[i] * dx[i];
 }
 
 if (sqrt(norm_dx) < 1e-6) {
 break;
 }
 }
 
 // Update state with estimated values
 for (int i = 0; i < 3; i++) {
 state->position[i] = x[i];
 state->velocity[i] = x[i+3];
 }
 state->clock_bias = x[6];
 state->clock_drift = x[7];
 
 // Update orbital elements
 state_vector_to_orbital_elements(state->position, state->velocity, state->orbital_elements);
}

void extended_kalman_filter(SpacecraftState *state, Measurement *measurements, 
 int num_measurements, double dt) {
 static double P[8][8]; // State covariance matrix
 static int initialized = 0;
 
 // State vector: [x, y, z, vx, vy, vz, clock_bias, clock_drift]
 double x[8];
 x[0] = state->position[0];
 x[1] = state->position[1];
 x[2] = state->position[2];
 x[3] = state->velocity[0];
 x[4] = state->velocity[1];
 x[5] = state->velocity[2];
 x[6] = state->clock_bias;
 x[7] = state->clock_drift;
 
 // Check for valid state
 for (int i = 0; i < 8; i++) {
 if (!isfinite(x[i])) {
 // Invalid state - reset to default values
 x[0] = RE + 700e3;
 x[1] = 0.0;
 x[2] = 0.0;
 x[3] = 0.0;
 x[4] = sqrt(MU / (RE + 700e3));
 x[5] = 0.0;
 x[6] = 0.0;
 x[7] = 0.0;
 
 for (int j = 0; j < 3; j++) {
 state->position[j] = x[j];
 state->velocity[j] = x[j+3];
 }
 state->clock_bias = x[6];
 state->clock_drift = x[7];
 
 initialized = 0; // Reset covariance
 break;
 }
 }
 
 // Initialize covariance if first run
 if (!initialized) {
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 P[i][j] = 0.0;
 }
 }
 
 // Position uncertainty (1 km)^2
 for (int i = 0; i < 3; i++) {
 P[i][i] = 1.0e6;
 }
 
 // Velocity uncertainty (1 m/s)^2
 for (int i = 3; i < 6; i++) {
 P[i][i] = 1.0;
 }
 
 // Clock bias uncertainty (1 microsecond)^2
 P[6][6] = 1.0e-12;
 
 // Clock drift uncertainty (1 ns/s)^2
 P[7][7] = 1.0e-18;
 
 initialized = 1;
 }
 
 // State transition matrix
 double F[8][8];
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 F[i][j] = 0.0;
 }
 }
 
 // Position elements
 for (int i = 0; i < 3; i++) {
 F[i][i] = 1.0;
 F[i][i+3] = dt;
 }
 
 // Velocity elements (including J2 perturbation effects)
 double r = vector_magnitude(state->position);
 if (r < RE || !isfinite(r)) {
 r = RE + 700e3; // Default to LEO altitude if invalid
 }
 
 double r2 = r * r;
 double r3 = r2 * r;
 double r5 = r3 * r2;
 
 // Two-body gravity gradient
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 if (i == j) {
 F[i+3][j] = (-MU / r3 + 3.0 * MU * state->position[i] * state->position[j] / r5) * dt;
 } else {
 F[i+3][j] = (3.0 * MU * state->position[i] * state->position[j] / r5) * dt;
 }
 }
 F[i+3][i+3] = 1.0;
 }
 
 // Clock elements
 F[6][6] = 1.0;
 F[6][7] = dt;
 F[7][7] = 1.0;
 
 // Process noise matrix
 double Q[8][8];
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 Q[i][j] = 0.0;
 }
 }
 
 // Position process noise (assuming constant acceleration)
 double sigma_a = 1.0e-5; // m/s^2
 for (int i = 0; i < 3; i++) {
 Q[i][i] = sigma_a * sigma_a * dt*dt*dt*dt / 4.0;
 Q[i][i+3] = sigma_a * sigma_a * dt*dt*dt / 2.0;
 Q[i+3][i] = sigma_a * sigma_a * dt*dt*dt / 2.0;
 Q[i+3][i+3] = sigma_a * sigma_a * dt*dt;
 }
 
 // Clock process noise (two-state clock model)
 double h0 = 2.0e-20; // Clock bias variance coefficient
 double h1 = 2.0e-21; // Clock drift variance coefficient
 double h2 = 2.0e-22; // Clock drift rate variance coefficient
 
 Q[6][6] = h0*dt + h1*dt*dt*dt/3.0 + h2*dt*dt*dt*dt*dt/20.0;
 Q[6][7] = h1*dt*dt/2.0 + h2*dt*dt*dt*dt/8.0;
 Q[7][6] = h1*dt*dt/2.0 + h2*dt*dt*dt*dt/8.0;
 Q[7][7] = h1*dt + h2*dt*dt*dt/3.0;
 
 // Propagate state (using the same propagator as before)
 SpacecraftState temp_state;
 memcpy(temp_state.position, state->position, sizeof(temp_state.position));
 memcpy(temp_state.velocity, state->velocity, sizeof(temp_state.velocity));
 temp_state.clock_bias = state->clock_bias;
 temp_state.clock_drift = state->clock_drift;
 
 propagate_orbit(&temp_state, dt);
 
 // Predicted state
 for (int i = 0; i < 3; i++) {
 x[i] = temp_state.position[i];
 x[i+3] = temp_state.velocity[i];
 }
 x[6] = temp_state.clock_bias;
 x[7] = temp_state.clock_drift;
 
 // Propagate covariance: P = F*P*F' + Q
 double FP[8][8];
 double FPFT[8][8];
 
 // Calculate F*P
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 FP[i][j] = 0.0;
 for (int k = 0; k < 8; k++) {
 FP[i][j] += F[i][k] * P[k][j];
 }
 }
 }
 
 // Calculate F*P*F'
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 FPFT[i][j] = 0.0;
 for (int k = 0; k < 8; k++) {
 FPFT[i][j] += FP[i][k] * F[j][k];
 }
 }
 }
 
 // Add process noise: P = F*P*F' + Q
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 P[i][j] = FPFT[i][j] + Q[i][j];
 }
 }
 
 // Measurement update (if measurements available)
 if (num_measurements > 0) {
 // Limit number of measurements to MAX_SMALL_MATRIX_DIM
 if (num_measurements > MAX_SMALL_MATRIX_DIM) {
 num_measurements = MAX_SMALL_MATRIX_DIM;
 }
 
 // Measurement Jacobian
 double H[MAX_MEASUREMENTS][8];
 
 // Measurement noise covariance
 double R[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 for (int i = 0; i < MAX_SMALL_MATRIX_DIM; i++) {
 for (int j = 0; j < MAX_SMALL_MATRIX_DIM; j++) {
 R[i][j] = 0.0;
 }
 }
 
 // Measurement residuals
 double z[MAX_MEASUREMENTS];
 double z_pred[MAX_MEASUREMENTS];
 
 // Compute predicted measurements and Jacobian
 for (int i = 0; i < num_measurements; i++) {
 // Using range-rate measurement model
 double range_vector[3];
 double sat_pos[3] = {
 measurements[i].sat_pos_eci[0],
 measurements[i].sat_pos_eci[1],
 measurements[i].sat_pos_eci[2]
 };
 
 vector_subtract(sat_pos, x, range_vector);
 double range = vector_magnitude(range_vector);
 
 if (range < 1e-10) {
 // Avoid division by zero
 range = 1.0;
 }
 
 // Predicted pseudorange
 z_pred[i] = range + C_LIGHT * x[6];
 
 // Measurement
 z[i] = measurements[i].pseudorange;
 
 // Jacobian for pseudorange
 for (int j = 0; j < 3; j++) {
 H[i][j] = -range_vector[j] / range; // Position partials
 H[i][j+3] = 0.0; // Velocity partials
 }
 H[i][6] = C_LIGHT; // Clock bias partial
 H[i][7] = 0.0; // Clock drift partial
 
 // Measurement noise (pseudorange noise)
 R[i][i] = 4.0; // 2 meters standard deviation, squared
 }
 
 // Scale matrices to improve conditioning
 double scale_factors[8] = {1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3, 1e7, 1e10};
 
 // Scaled covariance matrix
 double P_scaled[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 P_scaled[i][j] = P[i][j] * scale_factors[i] * scale_factors[j];
 }
 }
 
 // Scaled measurement Jacobian
 double H_scaled[MAX_MEASUREMENTS][MAX_SMALL_MATRIX_DIM];
 for (int i = 0; i < num_measurements; i++) {
 for (int j = 0; j < 8; j++) {
 H_scaled[i][j] = H[i][j] / scale_factors[j];
 }
 }
 
 // Compute Kalman gain: K = P*H'*(H*P*H' + R)^-1
 double PH[8][MAX_MEASUREMENTS];
 double HPH[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 double HPHR[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 double HPHR_inv[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 double K[8][MAX_MEASUREMENTS];
 
 // Calculate P*H' using scaled matrices
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < num_measurements; j++) {
 PH[i][j] = 0.0;
 for (int k = 0; k < 8; k++) {
 PH[i][j] += P_scaled[i][k] * H_scaled[j][k]; // Note H' is transposed
 }
 }
 }
 
 // Calculate H*P*H'
 for (int i = 0; i < num_measurements; i++) {
 for (int j = 0; j < num_measurements; j++) {
 HPH[i][j] = 0.0;
 for (int k = 0; k < 8; k++) {
 HPH[i][j] += H_scaled[i][k] * PH[k][j];
 }
 }
 }
 
 // Add measurement noise: S = H*P*H' + R
 for (int i = 0; i < num_measurements; i++) {
 for (int j = 0; j < num_measurements; j++) {
 HPHR[i][j] = HPH[i][j] + R[i][j];
 }
 }
 
 // Regularize to ensure matrix is invertible
 for (int i = 0; i < num_measurements; i++) {
 HPHR[i][i] += 1e-6;
 }
 
 // Invert innovation covariance matrix
 small_matrix_inverse(HPHR, HPHR_inv, num_measurements);
 
 // Calculate Kalman gain: K = P*H'*S^-1
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < num_measurements; j++) {
 K[i][j] = 0.0;
 for (int k = 0; k < num_measurements; k++) {
 K[i][j] += PH[i][k] * HPHR_inv[k][j];
 }
 // Unscale Kalman gain
 K[i][j] /= scale_factors[i];
 }
 }
 
 // Update state: x = x + K*(z - z_pred)
 double innovation[MAX_MEASUREMENTS];
 for (int i = 0; i < num_measurements; i++) {
 innovation[i] = z[i] - z_pred[i];
 
 // Limit innovations to reasonable values
 if (!isfinite(innovation[i]) || fabs(innovation[i]) > 10000.0) {
 innovation[i] = 0.0;
 }
 }
 
 for (int i = 0; i < 8; i++) {
 double correction = 0.0;
 for (int j = 0; j < num_measurements; j++) {
 correction += K[i][j] * innovation[j];
 }
 
 // Limit corrections to reasonable values
 if (!isfinite(correction)) {
 correction = 0.0;
 } else if (i < 3 && fabs(correction) > 10000.0) {
 correction = (correction > 0) ? 10000.0 : -10000.0;
 } else if (i >= 3 && i < 6 && fabs(correction) > 100.0) {
 correction = (correction > 0) ? 100.0 : -100.0;
 } else if (i == 6 && fabs(correction) > 1e-6) {
 correction = (correction > 0) ? 1e-6 : -1e-6;
 } else if (i == 7 && fabs(correction) > 1e-9) {
 correction = (correction > 0) ? 1e-9 : -1e-9;
 }
 
 x[i] += correction;
 }
 
 // Update covariance: P = (I - K*H)*P
 double KH[8][8];
 double IKH[8][8];
 
 // Calculate K*H
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 KH[i][j] = 0.0;
 for (int k = 0; k < num_measurements; k++) {
 KH[i][j] += K[i][k] * H[k][j];
 }
 }
 }
 
 // Calculate I - K*H
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 IKH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
 }
 }
 
 // Calculate (I - K*H)*P
 double temp_P[8][8];
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 temp_P[i][j] = 0.0;
 for (int k = 0; k < 8; k++) {
 temp_P[i][j] += IKH[i][k] * P[k][j];
 }
 }
 }
 
 // Copy back to P
 for (int i = 0; i < 8; i++) {
 for (int j = 0; j < 8; j++) {
 P[i][j] = temp_P[i][j];
 }
 }
 }
 
 // Update state with estimated values
 for (int i = 0; i < 3; i++) {
 state->position[i] = x[i];
 state->velocity[i] = x[i+3];
 }
 state->clock_bias = x[6];
 state->clock_drift = x[7];
 
 // Update orbital elements
 state_vector_to_orbital_elements(state->position, state->velocity, state->orbital_elements);
}

void unscented_kalman_filter(SpacecraftState *state, Measurement *measurements, 
 int num_measurements, double dt) {
 static double P[8][8]; // State covariance matrix
 static int initialized = 0;
 
 // Number of state variables
 const int n = 8;
 
 // State vector
 double x[8];
 x[0] = state->position[0];
 x[1] = state->position[1];
 x[2] = state->position[2];
 x[3] = state->velocity[0];
 x[4] = state->velocity[1];
 x[5] = state->velocity[2];
 x[6] = state->clock_bias;
 x[7] = state->clock_drift;
 
 // Check for valid state
 for (int i = 0; i < 8; i++) {
 if (!isfinite(x[i])) {
 // Invalid state - reset to default values
 x[0] = RE + 700e3;
 x[1] = 0.0;
 x[2] = 0.0;
 x[3] = 0.0;
 x[4] = sqrt(MU / (RE + 700e3));
 x[5] = 0.0;
 x[6] = 0.0;
 x[7] = 0.0;
 
 for (int j = 0; j < 3; j++) {
 state->position[j] = x[j];
 state->velocity[j] = x[j+3];
 }
 state->clock_bias = x[6];
 state->clock_drift = x[7];
 
 initialized = 0; // Reset covariance
 break;
 }
 }
 
 // Initialize covariance if first run
 if (!initialized) {
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 P[i][j] = 0.0;
 }
 }
 
 // Position uncertainty (1 km)^2
 for (int i = 0; i < 3; i++) {
 P[i][i] = 1.0e6;
 }
 
 // Velocity uncertainty (1 m/s)^2
 for (int i = 3; i < 6; i++) {
 P[i][i] = 1.0;
 }
 
 // Clock bias uncertainty (1 microsecond)^2
 P[6][6] = 1.0e-12;
 
 // Clock drift uncertainty (1 ns/s)^2
 P[7][7] = 1.0e-18;
 
 initialized = 1;
 }
 
 // UKF parameters
 double alpha = 1.0; // Spread of sigma points (1.0 avoids catastrophic cancellation for n=8)
 double beta = 2.0; // Prior knowledge of distribution (2 for Gaussian)
 double kappa = 0.0; // Secondary scaling parameter
 
 double lambda = alpha * alpha * (n + kappa) - n;
 
 // Calculate weights
 double Wm[2*n+1]; // Weights for mean
 double Wc[2*n+1]; // Weights for covariance
 
 Wm[0] = lambda / (n + lambda);
 Wc[0] = Wm[0] + (1 - alpha*alpha + beta);
 
 for (int i = 1; i <= 2*n; i++) {
 Wm[i] = 1.0 / (2 * (n + lambda));
 Wc[i] = Wm[i];
 }
 
 // Scale matrices to improve conditioning
 double scale_factors[8] = {1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3, 1e7, 1e10};
 double P_scaled[8][8];
 double x_scaled[8];
 
 // Scale state and covariance
 for (int i = 0; i < n; i++) {
 x_scaled[i] = x[i] * scale_factors[i];
 for (int j = 0; j < n; j++) {
 P_scaled[i][j] = P[i][j] * scale_factors[i] * scale_factors[j];
 }
 }
 
 // Calculate square root of P using Cholesky decomposition
 double L[n][n]; // Lower triangular matrix
 
 // Initialize L to zeros
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 L[i][j] = 0.0;
 }
 }
 
 // Perform Cholesky decomposition on scaled covariance
 for (int j = 0; j < n; j++) {
 double sum = 0.0;
 for (int k = 0; k < j; k++) {
 sum += L[j][k] * L[j][k];
 }
 double diag_term = P_scaled[j][j] - sum;
 if (diag_term < 1e-10) {
 // Not positive definite - add small value to diagonal
 diag_term = 1e-6;
 }
 L[j][j] = safe_sqrt(diag_term);
 
 for (int i = j+1; i < n; i++) {
 sum = 0.0;
 for (int k = 0; k < j; k++) {
 sum += L[i][k] * L[j][k];
 }
 
 if (fabs(L[j][j]) > 1e-10) {
 L[i][j] = (P_scaled[i][j] - sum) / L[j][j];
 } else {
 L[i][j] = 0.0;
 }
 }
 }
 
 // Scale by sqrt(n+lambda)
 double scale = safe_sqrt(n + lambda);
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 L[i][j] *= scale;
 }
 }
 
 // Generate sigma points in scaled space
 double chi_scaled[2*n+1][n];
 
 // Central point
 for (int j = 0; j < n; j++) {
 chi_scaled[0][j] = x_scaled[j];
 }
 
 // Remaining sigma points
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 chi_scaled[i+1][j] = x_scaled[j] + L[j][i];
 chi_scaled[i+n+1][j] = x_scaled[j] - L[j][i];
 }
 }
 
 // Unscale sigma points
 double chi[2*n+1][n];
 for (int i = 0; i < 2*n+1; i++) {
 for (int j = 0; j < n; j++) {
 chi[i][j] = chi_scaled[i][j] / scale_factors[j];
 }
 }
 
 // Propagate sigma points through process model
 double chi_pred[2*n+1][n];
 
 for (int i = 0; i < 2*n+1; i++) {
 // Create temporary state
 SpacecraftState temp_state;
 for (int j = 0; j < 3; j++) {
 temp_state.position[j] = chi[i][j];
 temp_state.velocity[j] = chi[i][j+3];
 }
 temp_state.clock_bias = chi[i][6];
 temp_state.clock_drift = chi[i][7];
 
 // Propagate
 propagate_orbit(&temp_state, dt);
 
 // Store propagated sigma point
 for (int j = 0; j < 3; j++) {
 chi_pred[i][j] = temp_state.position[j];
 chi_pred[i][j+3] = temp_state.velocity[j];
 }
 chi_pred[i][6] = temp_state.clock_bias;
 chi_pred[i][7] = temp_state.clock_drift;
 }
 
 // Calculate predicted mean
 double x_pred[n];
 for (int j = 0; j < n; j++) {
 x_pred[j] = 0.0;
 for (int i = 0; i < 2*n+1; i++) {
 x_pred[j] += Wm[i] * chi_pred[i][j];
 }
 }
 
 // Calculate predicted covariance
 double P_pred[n][n];
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 P_pred[i][j] = 0.0;
 for (int k = 0; k < 2*n+1; k++) {
 P_pred[i][j] += Wc[k] * (chi_pred[k][i] - x_pred[i]) * (chi_pred[k][j] - x_pred[j]);
 }
 }
 }
 
 // Add process noise
 double Q[n][n];
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 Q[i][j] = 0.0;
 }
 }
 
 // Position process noise
 double sigma_a = 1.0e-5; // m/s^2
 for (int i = 0; i < 3; i++) {
 Q[i][i] = sigma_a * sigma_a * dt*dt*dt*dt / 4.0;
 Q[i][i+3] = sigma_a * sigma_a * dt*dt*dt / 2.0;
 Q[i+3][i] = sigma_a * sigma_a * dt*dt*dt / 2.0;
 Q[i+3][i+3] = sigma_a * sigma_a * dt*dt;
 }
 
 // Clock process noise
 double h0 = 2.0e-20; // Clock bias variance coefficient
 double h1 = 2.0e-21; // Clock drift variance coefficient
 double h2 = 2.0e-22; // Clock drift rate variance coefficient
 
 Q[6][6] = h0*dt + h1*dt*dt*dt/3.0 + h2*dt*dt*dt*dt*dt/20.0;
 Q[6][7] = h1*dt*dt/2.0 + h2*dt*dt*dt*dt/8.0;
 Q[7][6] = h1*dt*dt/2.0 + h2*dt*dt*dt*dt/8.0;
 Q[7][7] = h1*dt + h2*dt*dt*dt/3.0;
 
 // Add process noise to predicted covariance
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 P_pred[i][j] += Q[i][j];
 }
 }
 
 // Measurement update (if measurements available)
 if (num_measurements > 0) {
 // Limit number of measurements to MAX_SMALL_MATRIX_DIM
 if (num_measurements > MAX_SMALL_MATRIX_DIM) {
 num_measurements = MAX_SMALL_MATRIX_DIM;
 }
 
 // Transform sigma points into measurement space
 double Z[2*n+1][MAX_MEASUREMENTS];
 
 for (int i = 0; i < 2*n+1; i++) {
 for (int j = 0; j < num_measurements; j++) {
 // Pseudorange measurement model
 double range_vector[3];
 double sat_pos[3] = {
 measurements[j].sat_pos_eci[0],
 measurements[j].sat_pos_eci[1],
 measurements[j].sat_pos_eci[2]
 };
 
 vector_subtract(sat_pos, chi_pred[i], range_vector);
 double range = vector_magnitude(range_vector);
 
 if (range < 1e-10) {
 range = 1.0; // Avoid division by zero
 }
 
 // Predicted pseudorange
 Z[i][j] = range + C_LIGHT * chi_pred[i][6];
 }
 }
 
 // Calculate predicted measurement mean
 double z_pred[MAX_MEASUREMENTS];
 for (int j = 0; j < num_measurements; j++) {
 z_pred[j] = 0.0;
 for (int i = 0; i < 2*n+1; i++) {
 z_pred[j] += Wm[i] * Z[i][j];
 }
 }
 
 // Calculate innovation covariance
 double Pzz[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 for (int i = 0; i < num_measurements; i++) {
 for (int j = 0; j < num_measurements; j++) {
 Pzz[i][j] = 0.0;
 for (int k = 0; k < 2*n+1; k++) {
 Pzz[i][j] += Wc[k] * (Z[k][i] - z_pred[i]) * (Z[k][j] - z_pred[j]);
 }
 }
 }
 
 // Add measurement noise
 double R[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 for (int i = 0; i < MAX_SMALL_MATRIX_DIM; i++) {
 for (int j = 0; j < MAX_SMALL_MATRIX_DIM; j++) {
 R[i][j] = 0.0;
 }
 }
 
 for (int i = 0; i < num_measurements; i++) {
 R[i][i] = 4.0; // 2 meters standard deviation, squared
 }
 
 for (int i = 0; i < num_measurements; i++) {
 for (int j = 0; j < num_measurements; j++) {
 Pzz[i][j] += R[i][j];
 }
 }
 
 // Regularize to ensure matrix is invertible
 for (int i = 0; i < num_measurements; i++) {
 Pzz[i][i] += 1e-6;
 }
 
 // Calculate cross-correlation
 double Pxz[n][MAX_MEASUREMENTS];
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < num_measurements; j++) {
 Pxz[i][j] = 0.0;
 for (int k = 0; k < 2*n+1; k++) {
 Pxz[i][j] += Wc[k] * (chi_pred[k][i] - x_pred[i]) * (Z[k][j] - z_pred[j]);
 }
 }
 }
 
 // Calculate Kalman gain
 double K[n][MAX_MEASUREMENTS];
 double Pzz_inv[MAX_SMALL_MATRIX_DIM][MAX_SMALL_MATRIX_DIM];
 
 // Invert innovation covariance
 small_matrix_inverse(Pzz, Pzz_inv, num_measurements);
 
 // K = Pxz * Pzz^-1
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < num_measurements; j++) {
 K[i][j] = 0.0;
 for (int k = 0; k < num_measurements; k++) {
 K[i][j] += Pxz[i][k] * Pzz_inv[k][j];
 }
 }
 }
 
 // Actual measurements
 double z[MAX_MEASUREMENTS];
 for (int i = 0; i < num_measurements; i++) {
 z[i] = measurements[i].pseudorange;
 }
 
 // Update state
 for (int i = 0; i < n; i++) {
 double correction = 0.0;
 for (int j = 0; j < num_measurements; j++) {
 double innovation = z[j] - z_pred[j];
 
 // Limit innovations to reasonable values
 if (!isfinite(innovation) || fabs(innovation) > 10000.0) {
 innovation = 0.0;
 }
 
 correction += K[i][j] * innovation;
 }
 
 // Limit corrections to reasonable values
 if (!isfinite(correction)) {
 correction = 0.0;
 } else if (i < 3 && fabs(correction) > 10000.0) {
 correction = (correction > 0) ? 10000.0 : -10000.0;
 } else if (i >= 3 && i < 6 && fabs(correction) > 100.0) {
 correction = (correction > 0) ? 100.0 : -100.0;
 } else if (i == 6 && fabs(correction) > 1e-6) {
 correction = (correction > 0) ? 1e-6 : -1e-6;
 } else if (i == 7 && fabs(correction) > 1e-9) {
 correction = (correction > 0) ? 1e-9 : -1e-9;
 }
 
 x_pred[i] += correction;
 }
 
 // Update covariance
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 double sum = 0.0;
 for (int k = 0; k < num_measurements; k++) {
 for (int l = 0; l < num_measurements; l++) {
 sum += K[i][k] * Pzz[k][l] * K[j][l];
 }
 }
 P[i][j] = P_pred[i][j] - sum;
 }
 }
 } else {
 // No measurements, just propagate
 for (int i = 0; i < n; i++) {
 for (int j = 0; j < n; j++) {
 P[i][j] = P_pred[i][j];
 }
 }
 }
 
 // Update state with estimated values
 for (int i = 0; i < 3; i++) {
 state->position[i] = x_pred[i];
 state->velocity[i] = x_pred[i+3];
 }
 state->clock_bias = x_pred[6];
 state->clock_drift = x_pred[7];
 
 // Update orbital elements
 state_vector_to_orbital_elements(state->position, state->velocity, state->orbital_elements);
}
// Function to perform GNSS-based precise orbit determination
void gnss_orbit_determination() {
 // Initialize spacecraft state
 SpacecraftState state;
 
 // LEO satellite in near-circular orbit at 700 km altitude
 double a = RE + 700e3; // Semi-major axis (m)
 double e = 0.001; // Eccentricity
 double i = 98.0 * PI/180.0; // Inclination (rad) - sun-synchronous
 double Omega = 30.0 * PI/180.0; // RAAN (rad)
 double omega = 0.0; // Argument of perigee (rad)
 double M = 0.0; // Mean anomaly (rad)
 
 // Initialize orbital elements
 state.orbital_elements[0] = a;
 state.orbital_elements[1] = e;
 state.orbital_elements[2] = i;
 state.orbital_elements[3] = Omega;
 state.orbital_elements[4] = omega;
 state.orbital_elements[5] = M;
 
 // Convert to state vector
 orbital_elements_to_state_vector(state.orbital_elements, state.position, state.velocity);
 
 // Initialize clocks
 state.clock_bias = 1.0e-7; // 100 ns initial bias
 state.clock_drift = 1.0e-10; // 0.1 ns/s initial drift
 
 // Initialize GNSS constellation
 GNSSSatellite satellites[32]; // Standard GPS constellation size
 
 // Initial date (arbitrary)
 DateTime date = {2023, 1, 1, 12, 0, 0.0};
 double jd = calculate_julian_date(date);
 g_epoch_jd = jd; /* seed ephemeris before first propagation */
 
 // Generate constellation
 generate_gnss_constellation(satellites, 32, jd);
 
 // Measurement storage
 Measurement measurements[MAX_MEASUREMENTS];
 int num_measurements;
 
 // Initialize sensor platform attitude quaternion (nadir-pointing LVLH frame)
 AttitudeQuaternion sc_attitude;
 quaternion_from_orbital_frame(state.position, state.velocity, &sc_attitude);
 quaternion_normalize(&sc_attitude);
 FLIGHT_LOG("Initial attitude quaternion: [%.6f, %.6f, %.6f, %.6f]\n",
 sc_attitude.q[0], sc_attitude.q[1], sc_attitude.q[2], sc_attitude.q[3]);
 
 // Main simulation loop
 FLIGHT_LOG("[ODNP] begin\n");
 for (int step = 0; step < NUM_TIMESTEPS; step++) {
 // Current time
 double current_time = step * DELTA_T;
 
 // Update Julian date and module-scope epoch for ephemeris
 jd += DELTA_T / 86400.0;
 g_epoch_jd = jd;
 
 // Propagate GNSS constellation
 for (int i = 0; i < 32; i++) {
 // Rotate satellites in their orbital planes
 double angular_velocity = sqrt(MU / pow(26560e3, 3)); // Orbital angular velocity
 double rotation_angle = angular_velocity * DELTA_T;
 
 double px = satellites[i].position[0];
 double py = satellites[i].position[1];
 double pz = satellites[i].position[2];
 double vx_s = satellites[i].velocity[0];
 double vy_s = satellites[i].velocity[1];
 double vz_s = satellites[i].velocity[2];

 // Orbit normal (angular momentum direction)
 double hx = py*vz_s - pz*vy_s;
 double hy = pz*vx_s - px*vz_s;
 double hz = px*vy_s - py*vx_s;
 double hmag = sqrt(hx*hx + hy*hy + hz*hz);
 if (hmag < 1e-10) hmag = 1.0;
 double kx = hx/hmag, ky = hy/hmag, kz = hz/hmag;

 // Rodrigues rotation about orbit normal
 double ca = sw_cos(rotation_angle), sa = sw_sin(rotation_angle);
 double kdp = kx*px + ky*py + kz*pz;
 satellites[i].position[0] = px*ca + (ky*pz - kz*py)*sa + kx*kdp*(1.0 - ca);
 satellites[i].position[1] = py*ca + (kz*px - kx*pz)*sa + ky*kdp*(1.0 - ca);
 satellites[i].position[2] = pz*ca + (kx*py - ky*px)*sa + kz*kdp*(1.0 - ca);

 // Also rotate velocity
 double kdv = kx*vx_s + ky*vy_s + kz*vz_s;
 satellites[i].velocity[0] = vx_s*ca + (ky*vz_s - kz*vy_s)*sa + kx*kdv*(1.0 - ca);
 satellites[i].velocity[1] = vy_s*ca + (kz*vx_s - kx*vz_s)*sa + ky*kdv*(1.0 - ca);
 satellites[i].velocity[2] = vz_s*ca + (kx*vy_s - ky*vx_s)*sa + kz*kdv*(1.0 - ca);
 
 // Update clocks
 satellites[i].clock_bias += satellites[i].clock_drift * DELTA_T;
 satellites[i].clock_drift += ((double)rand() / RAND_MAX - 0.5) * 1.0e-12 * DELTA_T;
 }
 
 // Generate measurements
 generate_measurements(&state, satellites, 32, jd, measurements, &num_measurements);
 
 // Print number of visible satellites
 FLIGHT_LOG("Step %d: Time = %.1f s, Visible satellites = %d\n", 
 step, current_time, num_measurements);
 
 // Transform GNSS measurements from body frame to ECI using attitude quaternion
 transform_sensor_measurements(&sc_attitude, measurements, num_measurements, state.position);

 // Compute body-to-inertial DCM for telemetry and state output
 double dcm_att[3][3];
 quaternion_to_dcm(&sc_attitude, dcm_att);

 // Verify attitude DCM orthogonality (trace should be near 1+2*cos(theta))
 double dcm_trace = dcm_att[0][0] + dcm_att[1][1] + dcm_att[2][2];
 if (step % 50 == 0) {
 FLIGHT_LOG(" Attitude: q=[%.4f,%.4f,%.4f,%.4f] DCM_trace=%.6f\n",
 sc_attitude.q[0], sc_attitude.q[1],
 sc_attitude.q[2], sc_attitude.q[3], dcm_trace);
 }

 // Process measurements with different filters
 int filter_propagated = 0; // Track whether filter already propagated state
 if (step % 3 == 0) {
 // Least squares (every 3rd step)
 FLIGHT_LOG(" Using least squares\n");
 least_squares_orbit_determination(&state, measurements, num_measurements, DELTA_T);
 } else if (step % 3 == 1) {
 // Extended Kalman Filter
 FLIGHT_LOG(" Using EKF\n");
 extended_kalman_filter(&state, measurements, num_measurements, DELTA_T);
 filter_propagated = 1; // EKF propagates internally
 } else {
 // Unscented Kalman Filter
 FLIGHT_LOG(" Using UKF\n");
 unscented_kalman_filter(&state, measurements, num_measurements, DELTA_T);
 filter_propagated = 1; // UKF propagates internally
 }
 
 // Propagate sensor platform attitude quaternion
 // Orbital angular velocity for nadir-pointing maintenance
 double r_current = vector_magnitude(state.position);
 double v_current = vector_magnitude(state.velocity);
 if (r_current > RE && v_current > 0) {
 double omega_orb = v_current / r_current;
 double half_angle = 0.5 * omega_orb * DELTA_T;

 // Compute orbit normal in ECI for rotation axis
 double h_vec[3];
 vector_cross(state.position, state.velocity, h_vec);
 double h_mag_att = vector_magnitude(h_vec);
 if (h_mag_att > 1e-10) {
 double rot_axis[3];
 vector_scale(h_vec, 1.0/h_mag_att, rot_axis);

 // Incremental rotation quaternion about orbit normal
 AttitudeQuaternion dq;
 dq.q[0] = sw_cos(half_angle);
 dq.q[1] = rot_axis[0] * sw_sin(half_angle);
 dq.q[2] = rot_axis[1] * sw_sin(half_angle);
 dq.q[3] = rot_axis[2] * sw_sin(half_angle);

 // Add small gravity-gradient torque perturbation
 double gg_angle = 0.5 * 3.0 * MU / (r_current * r_current * r_current) * DELTA_T * 0.001;
 AttitudeQuaternion dq_gg;
 dq_gg.q[0] = sw_cos(gg_angle);
 dq_gg.q[1] = sw_sin(gg_angle) * 0.01;
 dq_gg.q[2] = sw_sin(gg_angle) * 0.005;
 dq_gg.q[3] = 0.0;
 quaternion_normalize(&dq_gg);

 // Compose rotations: attitude = dq * dq_gg * attitude
 AttitudeQuaternion temp_q;
 quaternion_multiply(&temp_q, &dq_gg, &sc_attitude);
 quaternion_multiply(&sc_attitude, &dq, &temp_q);
 quaternion_normalize(&sc_attitude);
 }
 }
 
 // Update state for next iteration (propagate orbit)
 // Only propagate if the filter didn't already propagate internally
 if (!filter_propagated) {
 propagate_orbit(&state, DELTA_T);
 }
 
 // Print current state
 FLIGHT_LOG(" Position: [%.3f, %.3f, %.3f] km\n", 
 state.position[0]/1000.0, state.position[1]/1000.0, state.position[2]/1000.0);
 FLIGHT_LOG(" Velocity: [%.3f, %.3f, %.3f] km/s\n", 
 state.velocity[0]/1000.0, state.velocity[1]/1000.0, state.velocity[2]/1000.0);
 FLIGHT_LOG(" Orbital elements: a=%.3f km, e=%.6f, i=%.2f deg\n", 
 state.orbital_elements[0]/1000.0, 
 state.orbital_elements[1], 
 state.orbital_elements[2] * 180.0/PI);
 FLIGHT_LOG(" Clock: bias=%.3f ns, drift=%.3f ns/s\n", 
 state.clock_bias * 1.0e9, state.clock_drift * 1.0e9);
 }
 
 // Final state
 FLIGHT_LOG("\n[ODNP] final_state (steps=%d)\n", NUM_TIMESTEPS);
 FLIGHT_LOG("Position (ECI): [%.3f, %.3f, %.3f] km\n", 
 state.position[0]/1000.0, state.position[1]/1000.0, state.position[2]/1000.0);
 FLIGHT_LOG("Velocity (ECI): [%.3f, %.3f, %.3f] km/s\n", 
 state.velocity[0]/1000.0, state.velocity[1]/1000.0, state.velocity[2]/1000.0);
 
 // Final orbital elements
 FLIGHT_LOG("[ODNP] elements\n");
 FLIGHT_LOG(" Semi-major axis: %.3f km\n", state.orbital_elements[0]/1000.0);
 FLIGHT_LOG(" Eccentricity: %.6f\n", state.orbital_elements[1]);
 FLIGHT_LOG(" Inclination: %.2f deg\n", state.orbital_elements[2] * 180.0/PI);
 FLIGHT_LOG(" RAAN: %.2f deg\n", state.orbital_elements[3] * 180.0/PI);
 FLIGHT_LOG(" Argument of perigee: %.2f deg\n", state.orbital_elements[4] * 180.0/PI);
 FLIGHT_LOG(" Mean anomaly: %.2f deg\n", state.orbital_elements[5] * 180.0/PI);
 
 // Final clock state
 FLIGHT_LOG("[ODNP] clock\n");
 FLIGHT_LOG(" Clock bias: %.3f ns\n", state.clock_bias * 1.0e9);
 FLIGHT_LOG(" Clock drift: %.3f ns/s\n", state.clock_drift * 1.0e9);
}

// Main function
int main() {
 clock_t start, end;
 double cpu_time_used;
 
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(4219);

 start = clock();
 gnss_orbit_determination();
 end = clock();
 
 cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
 FLIGHT_LOG("\n[ODNP] done (%.3fs)\n", cpu_time_used);
 
 return 0;
}
