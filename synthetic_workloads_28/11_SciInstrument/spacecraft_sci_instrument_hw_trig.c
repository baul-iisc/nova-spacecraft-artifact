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
 * Scientific Instrument Control and Data Processing
 *
 * Author : Boul Chandra Garai
 * Target : RISC-V (RV64GC) with Matrix Extension Accelerator
 * Variant: HW Trig (CORDIC accelerator for trigonometric functions)
 * Application: (see workload description below)
 *
 * Description:
 * Implements onboard scientific instrument control and data processing pipeline.
 * This workload manages instrument mode sequencing, spectral analysis via DFT,
 * 18x18 covariance matrix computation for measurement fusion, and calibration
 * matrix operations. Representative of payload data processing on Earth observation
 * observation satellites (VELC, SUIT-class instruments) and lunar orbiter
 * scientific payloads (CLASS, XSM, IIRS).
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
 *
 * Precision Note: This workload intentionally uses single-precision (float)
 * for the sensor data acquisition pipeline and spectral processing stages,
 * matching the native 12-16 bit ADC resolution of instruments like VELC and
 * SUIT. Double-precision is reserved for the EKF measurement fusion core
 * (18x18 covariance matrices) where numerical conditioning is critical.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
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
// Define constants
#define INSTRUMENT_COUNT 3
#define MAX_COMMAND_QUEUE 30
#define DATA_BUFFER_SIZE 1200 // Science data buffer (10 min at 2 Hz)
#define CALIBRATION_SAMPLES 300 // Flat-field calibration accumulation count
#define TELEMETRY_PACKET_SIZE 300 // Maximum telemetry float entries
#define STATE_DIM 18 // Science instrument state dimension
#define FFT_LEN 64 // DFT analysis length (power of 2)
#define CALIB_STATE_DIM 12 // Calibration EKF state dimension (gain3+bias3+temp3+pointing3)

/* hw_sin, hw_cos, hw_atan2, hw_asin — provided by inline CORDIC definitions (fsin.d, fcos.d, etc.)
 * (double-precision CORDIC accelerator; float args are promoted automatically) */

float hw_sqrt(float x) {
 return sqrtf(x);
}

double hw_sqrt_d(double x) {
 return sqrt(x);
}

void hw_matrix3x3_multiply(float *A, float *B, float *C) {
 // Software fallback - replaced with hardware intrinsic in accelerated variant
 for (int i = 0; i < 3; i++) {
 for (int j = 0; j < 3; j++) {
 C[i*3 + j] = 0;
 for (int k = 0; k < 3; k++) {
 C[i*3 + j] += A[i*3 + k] * B[k*3 + j];
 }
 }
 }
}

// Instrument types
typedef enum {
 INSTRUMENT_CAMERA,
 INSTRUMENT_SPECTROMETER,
 INSTRUMENT_MAGNETOMETER
} InstrumentType;

// Instrument operational modes
typedef enum {
 MODE_OFF,
 MODE_STANDBY,
 MODE_SCIENCE,
 MODE_CALIBRATION,
 MODE_DIAGNOSTIC,
 MODE_SAFE
} InstrumentMode;

// Command structure
typedef struct {
 unsigned int command_id;
 unsigned int instrument_id;
 unsigned int parameter_count;
 float parameters[3]; // Up to 3 command parameters (1 per axis)
 unsigned int execution_time; // Spacecraft time
 bool executed;
} Command;

// Instrument state
typedef struct {
 unsigned int instrument_id;
 InstrumentType type;
 InstrumentMode current_mode;
 InstrumentMode target_mode;
 float temperature; // Current temperature
 float power_consumption; // In watts
 float data_rate; // In bytes per second
 float calibration_matrix[9]; // 3x3 calibration matrix
 float health_status[9]; // Health parameters (3x3 format)
 unsigned int samples_collected;
 unsigned int last_command_time;
 bool error_flag;
} InstrumentState;

// Data sample structure
typedef struct {
 unsigned int instrument_id;
 unsigned int sample_id;
 unsigned int timestamp;
 float raw_data[9]; // 3x3 data block
 float processed_data[9]; // 3x3 processed data
 bool is_calibrated;
} DataSample;

// Scientific instrument control system
typedef struct {
 InstrumentState instruments[INSTRUMENT_COUNT];
 Command command_queue[MAX_COMMAND_QUEUE];
 unsigned int command_count;
 DataSample data_buffer[DATA_BUFFER_SIZE/9]; // Each sample is 3x3
 unsigned int data_count;
 unsigned int spacecraft_time; // Mission elapsed time in seconds
 float spacecraft_position[3]; // X, Y, Z position
 float spacecraft_attitude[9]; // 3x3 rotation matrix
 float spacecraft_pointing[3]; // Target pointing vector
 bool safe_mode_active;
} InstrumentControlSystem;

// Function prototypes
void initialize_system(InstrumentControlSystem *system);
void execute_command_sequence(InstrumentControlSystem *system);
void acquire_instrument_data(InstrumentControlSystem *system, unsigned int instrument_id);
void process_raw_data(InstrumentControlSystem *system);
void calibrate_instrument(InstrumentControlSystem *system, unsigned int instrument_id);
void change_instrument_mode(InstrumentControlSystem *system, unsigned int instrument_id, InstrumentMode new_mode);
void handle_fault_detection(InstrumentControlSystem *system);
void generate_telemetry(InstrumentControlSystem *system, float *telemetry_data);
void update_pointing_control(InstrumentControlSystem *system);
void queue_command(InstrumentControlSystem *system, Command cmd);
void generate_science_data_products(InstrumentControlSystem *system);

/**
 * Initialize the instrument control system
 */
void initialize_system(InstrumentControlSystem *system) {
 // Clear all data
 memset(system, 0, sizeof(InstrumentControlSystem));

 // Initialize instruments
 for (int i = 0; i < INSTRUMENT_COUNT; i++) {
 system->instruments[i].instrument_id = i;
 system->instruments[i].current_mode = MODE_OFF;
 system->instruments[i].target_mode = MODE_STANDBY;
 system->instruments[i].temperature = 20.0f; // Ambient temperature
 system->instruments[i].power_consumption = 0.1f; // Standby power
 system->instruments[i].data_rate = 0.0f; // No data in OFF mode

 // Initialize the calibration matrix to identity
 memset(system->instruments[i].calibration_matrix, 0, sizeof(float) * 9);
 system->instruments[i].calibration_matrix[0] = 1.0f;
 system->instruments[i].calibration_matrix[4] = 1.0f;
 system->instruments[i].calibration_matrix[8] = 1.0f;

 // Set instrument types
 switch (i) {
 case 0:
 system->instruments[i].type = INSTRUMENT_CAMERA;
 break;
 case 1:
 system->instruments[i].type = INSTRUMENT_SPECTROMETER;
 break;
 case 2:
 system->instruments[i].type = INSTRUMENT_MAGNETOMETER;
 break;
 }
 }

 // Set initial spacecraft state
 system->spacecraft_time = 0;
 system->spacecraft_position[0] = 0.0f;
 system->spacecraft_position[1] = 0.0f;
 system->spacecraft_position[2] = 0.0f;

 // Initialize spacecraft attitude to identity (no rotation)
 memset(system->spacecraft_attitude, 0, sizeof(float) * 9);
 system->spacecraft_attitude[0] = 1.0f;
 system->spacecraft_attitude[4] = 1.0f;
 system->spacecraft_attitude[8] = 1.0f;

 // Initial pointing direction (nadir pointing)
 system->spacecraft_pointing[0] = 0.0f;
 system->spacecraft_pointing[1] = 0.0f;
 system->spacecraft_pointing[2] = -1.0f;

 // Not in safe mode initially
 system->safe_mode_active = false;

 FLIGHT_LOG("[SCI] ready\n");
}

/**
 * Execute the command sequence for all pending commands
 */
void execute_command_sequence(InstrumentControlSystem *system) {
 FLIGHT_LOG("Executing command sequence. Commands in queue: %d\n", system->command_count);

 for (unsigned int i = 0; i < system->command_count; i++) {
 Command *cmd = &system->command_queue[i]; // FIXED: Access command_queue instead of instruments

 // Check if it's time to execute this command
 if (cmd->execution_time <= system->spacecraft_time && !cmd->executed) {
 unsigned int instr_id = cmd->instrument_id;

 FLIGHT_LOG("Executing command %d for instrument %d\n", cmd->command_id, instr_id);

 // Execute command based on its ID
 switch (cmd->command_id) {
 case 1: // Power on
 change_instrument_mode(system, instr_id, MODE_STANDBY);
 break;

 case 2: // Start science mode
 change_instrument_mode(system, instr_id, MODE_SCIENCE);
 break;

 case 3: // Start calibration
 change_instrument_mode(system, instr_id, MODE_CALIBRATION);
 calibrate_instrument(system, instr_id);
 break;

 case 4: // Collect data sample
 if (system->instruments[instr_id].current_mode == MODE_SCIENCE) {
 acquire_instrument_data(system, instr_id);
 }
 break;

 case 5: // Change pointing
 // Parameters 0,1,2 contain the pointing vector
 system->spacecraft_pointing[0] = cmd->parameters[0];
 system->spacecraft_pointing[1] = cmd->parameters[1];
 system->spacecraft_pointing[2] = cmd->parameters[2];
 update_pointing_control(system);
 break;

 case 6: // Process data
 process_raw_data(system);
 break;

 case 7: // Generate science products
 generate_science_data_products(system);
 break;

 case 8: // Power down
 change_instrument_mode(system, instr_id, MODE_OFF);
 break;

 case 9: // Run diagnostics
 change_instrument_mode(system, instr_id, MODE_DIAGNOSTIC);
 // Diagnostics would go here
 break;

 default:
 FLIGHT_LOG("Unknown command ID: %d\n", cmd->command_id);
 break;
 }

 // Update last_command_time for the targeted instrument
 if (instr_id < INSTRUMENT_COUNT) {
 system->instruments[instr_id].last_command_time = system->spacecraft_time;
 }

 // Mark command as executed
 cmd->executed = true;
 }
 }

 // Remove executed commands from the queue
 unsigned int new_count = 0;
 for (unsigned int i = 0; i < system->command_count; i++) {
 if (!system->command_queue[i].executed) {
 // Keep this command in the queue
 if (i != new_count) {
 system->command_queue[new_count] = system->command_queue[i];
 }
 new_count++;
 }
 }

 system->command_count = new_count;
}

/**
 * Acquire data from a specific instrument
 */
void acquire_instrument_data(InstrumentControlSystem *system, unsigned int instrument_id) {
 if (instrument_id >= INSTRUMENT_COUNT) {
 FLIGHT_LOG("Invalid instrument ID: %d\n", instrument_id);
 return;
 }

 InstrumentState *instrument = &system->instruments[instrument_id];

 // Check if instrument is in a mode that allows data collection
 if (instrument->current_mode != MODE_SCIENCE &&
 instrument->current_mode != MODE_CALIBRATION) {
 FLIGHT_LOG("Instrument %d not in data collection mode\n", instrument_id);
 return;
 }

 // Check if we have room in the buffer
 if (system->data_count >= DATA_BUFFER_SIZE/9) {
 FLIGHT_LOG("Data buffer full, cannot acquire more data\n");
 return;
 }

 // Create a new data sample
 DataSample sample;
 sample.instrument_id = instrument_id;
 sample.sample_id = instrument->samples_collected++;
 sample.timestamp = system->spacecraft_time;
 sample.is_calibrated = false;

 // Generate simulated raw data based on instrument type
 // Read from instrument hardware data interface
 switch (instrument->type) {
 case INSTRUMENT_CAMERA: {
 // Simulate a 3x3 image patch
 for (int i = 0; i < 9; i++) {
 // Generate values based on pointing and spacecraft position
 float x = system->spacecraft_pointing[0] + (i % 3) * 0.01f;
 float y = system->spacecraft_pointing[1] + (i / 3) * 0.01f;

 // Use hardware trig functions for calculation
 sample.raw_data[i] = 127.5f + 127.5f * hw_sin(x) * hw_cos(y);
 }
 break;
 }

 case INSTRUMENT_SPECTROMETER: {
 // Simulate spectral data
 for (int i = 0; i < 9; i++) {
 // Generate values representing different wavelengths
 float wavelength = 400.0f + (i * 100.0f); // 400-1200nm range
 float intensity = 1000.0f * hw_sin(wavelength / 1000.0f) *
 exp(-pow((wavelength - 800.0f) / 400.0f, 2.0f));
 sample.raw_data[i] = intensity;
 }
 break;
 }

 case INSTRUMENT_MAGNETOMETER: {
 // Simulate magnetic field readings
 float field_strength = 30000.0f; // 30,000 nT (typical Earth field)

 // Create a 3x3 tensor for the magnetic field
 // Diagonal elements are field strength in principal directions
 memset(sample.raw_data, 0, sizeof(float) * 9);
 sample.raw_data[0] = field_strength * (1.0f + 0.1f * hw_sin(system->spacecraft_time / 1000.0f));
 sample.raw_data[4] = field_strength * (1.0f + 0.1f * hw_cos(system->spacecraft_time / 1000.0f));
 sample.raw_data[8] = field_strength * 0.8f;
 break;
 }
 }

 // Apply sensor noise (simple model)
 for (int i = 0; i < 9; i++) {
 // Add random noise (using simple random function for illustration)
 float noise = (rand() % 1000) / 1000.0f - 0.5f;
 sample.raw_data[i] += noise * 10.0f;
 }

 // Add the sample to the buffer
 system->data_buffer[system->data_count] = sample;
 system->data_count++;

 // Update instrument data rate based on data size and sample frequency
 instrument->data_rate = sizeof(DataSample) / 10.0f; // Assuming one sample every 10 seconds

 FLIGHT_LOG("Acquired data sample %d from instrument %d\n",
 sample.sample_id, instrument_id);
}

/**
 * Process raw data from instruments using calibration
 */
void process_raw_data(InstrumentControlSystem *system) {
 FLIGHT_LOG("[SCI] samples=%d\n", system->data_count);

 for (unsigned int i = 0; i < system->data_count; i++) {
 DataSample *sample = &system->data_buffer[i];

 // Skip if already processed
 if (sample->is_calibrated) {
 continue;
 }

 // Get the calibration matrix for this instrument
 float *cal_matrix = system->instruments[sample->instrument_id].calibration_matrix;

 // Apply calibration using hardware-accelerated 3x3 matrix multiplication
 hw_matrix3x3_multiply(cal_matrix, sample->raw_data, sample->processed_data);

 // Mark as calibrated
 sample->is_calibrated = true;
 }
}

/**
 * Calibrate an instrument using reference measurements
 */
void calibrate_instrument(InstrumentControlSystem *system, unsigned int instrument_id) {
 if (instrument_id >= INSTRUMENT_COUNT) {
 FLIGHT_LOG("Invalid instrument ID: %d\n", instrument_id);
 return;
 }

 InstrumentState *instrument = &system->instruments[instrument_id];

 FLIGHT_LOG("Calibrating instrument %d\n", instrument_id);

 // Change to calibration mode if not already
 if (instrument->current_mode != MODE_CALIBRATION) {
 change_instrument_mode(system, instrument_id, MODE_CALIBRATION);
 }

 // For each instrument type, we'd use a different calibration procedure
 switch (instrument->type) {
 case INSTRUMENT_CAMERA: {
 // For cameras, we might use a flat-field correction
 // This simulates collecting uniform reference images

 // Simulate collecting CALIBRATION_SAMPLES samples of flat field
 float avg_values[9] = {0};
 float reference_value = 1000.0f; // Expected flat field value

 // Accumulate simulated samples
 for (int i = 0; i < CALIBRATION_SAMPLES / 9; i++) {
 float sample[9];

 // Generate simulated flat field data with some variability
 for (int j = 0; j < 9; j++) {
 sample[j] = reference_value * (0.9f + (rand() % 200) / 1000.0f);
 avg_values[j] += sample[j];
 }
 }

 // Calculate average response
 for (int j = 0; j < 9; j++) {
 avg_values[j] /= (CALIBRATION_SAMPLES / 9);
 }

 // Create calibration matrix (3x3 diagonal matrix for pixel correction)
 memset(instrument->calibration_matrix, 0, sizeof(float) * 9);

 // Diagonal elements are correction factors
 instrument->calibration_matrix[0] = reference_value / avg_values[0];
 instrument->calibration_matrix[4] = reference_value / avg_values[4];
 instrument->calibration_matrix[8] = reference_value / avg_values[8];

 break;
 }

 case INSTRUMENT_SPECTROMETER: {
 // For spectrometers, we might use known spectral lines
 // Simulating detection of known reference wavelengths

 // Reference wavelengths and expected values
 float ref_wavelengths[3] = {486.0f, 589.0f, 656.0f}; // H-beta, Na-D, H-alpha
 (void)ref_wavelengths;
 float ref_values[3] = {1.0f, 1.0f, 1.0f};

 // Simulate measuring these wavelengths with the current instrument
 float measured[3];
 for (int i = 0; i < 3; i++) {
 // Simulate measurement with some random error
 measured[i] = ref_values[i] * (0.9f + (rand() % 200) / 1000.0f);
 }

 // Create calibration matrix (3x3 diagonal for wavelength calibration)
 memset(instrument->calibration_matrix, 0, sizeof(float) * 9);

 // Diagonal elements are correction factors
 instrument->calibration_matrix[0] = ref_values[0] / measured[0];
 instrument->calibration_matrix[4] = ref_values[1] / measured[1];
 instrument->calibration_matrix[8] = ref_values[2] / measured[2];

 break;
 }

 case INSTRUMENT_MAGNETOMETER: {
 // For magnetometers, we might do an orthogonalization and scaling

 // Simulate collecting measurements in different orientations
 // We'd collect 3 orthogonal field measurements ideally

 // Simulate three orthogonal fields (aligned with axes)
 float field_samples[9] = {
 30000.0f, 1000.0f, 500.0f, // X-axis dominant field
 800.0f, 30000.0f, 700.0f, // Y-axis dominant field
 600.0f, 900.0f, 30000.0f // Z-axis dominant field
 };

 // Expected ideal values for these fields
 float ideal_fields[9] = {
 30000.0f, 0.0f, 0.0f, // Perfect X-axis
 0.0f, 30000.0f, 0.0f, // Perfect Y-axis
 0.0f, 0.0f, 30000.0f // Perfect Z-axis
 };

 // Calculate inverse calibration matrix via Gauss-Jordan elimination
 // Use the diagonal elements
 memset(instrument->calibration_matrix, 0, sizeof(float) * 9);

 // Simple diagonal calibration
 instrument->calibration_matrix[0] = ideal_fields[0] / field_samples[0];
 instrument->calibration_matrix[4] = ideal_fields[4] / field_samples[4];
 instrument->calibration_matrix[8] = ideal_fields[8] / field_samples[8];

 break;
 }
 }

 FLIGHT_LOG("Calibration complete for instrument %d\n", instrument_id);

 // Return to previous mode if it wasn't calibration
 if (instrument->target_mode != MODE_CALIBRATION) {
 change_instrument_mode(system, instrument_id, instrument->target_mode);
 }
}

/**
 * Change the operational mode of an instrument
 */
void change_instrument_mode(InstrumentControlSystem *system, unsigned int instrument_id,
 InstrumentMode new_mode) {
 if (instrument_id >= INSTRUMENT_COUNT) {
 FLIGHT_LOG("Invalid instrument ID: %d\n", instrument_id);
 return;
 }

 InstrumentState *instrument = &system->instruments[instrument_id];
 InstrumentMode old_mode = instrument->current_mode;

 FLIGHT_LOG("Changing instrument %d mode from %d to %d\n", instrument_id, old_mode, new_mode);

 // Check if this is a valid mode transition
 bool valid_transition = true;

 // Instrument scheduling constraints
 switch (old_mode) {
 case MODE_OFF:
 // From OFF, can only go to STANDBY
 valid_transition = (new_mode == MODE_STANDBY);
 break;

 case MODE_STANDBY:
 // From STANDBY, can go to any mode except SAFE (which is entered automatically)
 valid_transition = (new_mode != MODE_SAFE);
 break;

 case MODE_SCIENCE:
 case MODE_CALIBRATION:
 case MODE_DIAGNOSTIC:
 // From these modes, can go to any mode except directly to OFF (must go via STANDBY)
 valid_transition = (new_mode != MODE_OFF);
 break;

 case MODE_SAFE:
 // From SAFE, can only go to STANDBY if error condition is cleared
 valid_transition = (new_mode == MODE_STANDBY && !instrument->error_flag);
 break;
 }

 if (!valid_transition) {
 FLIGHT_LOG("Invalid mode transition from %d to %d\n", old_mode, new_mode);
 return;
 }

 // Apply the mode change
 instrument->current_mode = new_mode;
 instrument->target_mode = new_mode;

 // Update power consumption based on mode
 switch (new_mode) {
 case MODE_OFF:
 instrument->power_consumption = 0.0f;
 instrument->data_rate = 0.0f;
 break;

 case MODE_STANDBY:
 instrument->power_consumption = 0.47f; // 0.47W in standby
 instrument->data_rate = 0.01f; // Minimal housekeeping data
 break;

 case MODE_SCIENCE:
 // Power consumption depends on instrument type
 switch (instrument->type) {
 case INSTRUMENT_CAMERA:
 instrument->power_consumption = 2.85f;
 instrument->data_rate = 1250.0f; // 1250 bytes/sec
 break;
 case INSTRUMENT_SPECTROMETER:
 instrument->power_consumption = 4.72f;
 instrument->data_rate = 850.0f; // 850 bytes/sec
 break;
 case INSTRUMENT_MAGNETOMETER:
 instrument->power_consumption = 1.13f;
 instrument->data_rate = 420.0f; // 420 bytes/sec
 break;
 }
 break;

 case MODE_CALIBRATION: {
 float base_power = 2.85f, base_rate = 850.0f; /* Science mode defaults */
 if (instrument->type == INSTRUMENT_CAMERA) { base_power = 2.85f; base_rate = 1250.0f; }
 else if (instrument->type == INSTRUMENT_SPECTROMETER) { base_power = 4.72f; base_rate = 850.0f; }
 else if (instrument->type == INSTRUMENT_MAGNETOMETER) { base_power = 1.13f; base_rate = 420.0f; }
 instrument->power_consumption = base_power * 1.2f;
 instrument->data_rate = base_rate * 1.5f;
 break;
 }

 case MODE_DIAGNOSTIC: {
 float base_power = 2.85f, base_rate = 850.0f; /* Science mode defaults */
 if (instrument->type == INSTRUMENT_CAMERA) { base_power = 2.85f; base_rate = 1250.0f; }
 else if (instrument->type == INSTRUMENT_SPECTROMETER) { base_power = 4.72f; base_rate = 850.0f; }
 else if (instrument->type == INSTRUMENT_MAGNETOMETER) { base_power = 1.13f; base_rate = 420.0f; }
 instrument->power_consumption = base_power * 1.1f;
 instrument->data_rate = base_rate * 2.0f;
 break;
 }

 case MODE_SAFE:
 instrument->power_consumption = 0.2f; // Minimal power
 instrument->data_rate = 0.5f; // Only critical telemetry
 break;
 }
}

/**
 * Handle fault detection for instruments
 */
void handle_fault_detection(InstrumentControlSystem *system) {
 // Check each instrument for anomalies
 for (int i = 0; i < INSTRUMENT_COUNT; i++) {
 InstrumentState *instrument = &system->instruments[i];
 bool fault_detected = false;

 // Check for temperature out of range
 if (instrument->temperature < -20.0f || instrument->temperature > 60.0f) {
 FLIGHT_LOG("FAULT: Instrument %d temperature out of range: %.1f°C\n",
 i, instrument->temperature);
 fault_detected = true;
 }

 // Check for excessive power consumption
 float expected_power = 0.0f;
 switch (instrument->current_mode) {
 case MODE_OFF:
 expected_power = 0.0f;
 break;
 case MODE_STANDBY:
 expected_power = 0.5f;
 break;
 case MODE_SCIENCE:
 // Depends on instrument type
 switch (instrument->type) {
 case INSTRUMENT_CAMERA:
 expected_power = 3.0f;
 break;
 case INSTRUMENT_SPECTROMETER:
 expected_power = 5.0f;
 break;
 case INSTRUMENT_MAGNETOMETER:
 expected_power = 1.0f;
 break;
 }
 break;
 case MODE_CALIBRATION:
 case MODE_DIAGNOSTIC:
 // Just use current value as reference
 expected_power = instrument->power_consumption;
 break;
 case MODE_SAFE:
 expected_power = 0.2f;
 break;
 }

 // Check if power consumption is 20% above expected
 if (instrument->power_consumption > expected_power * 1.2f) {
 FLIGHT_LOG("FAULT: Instrument %d power consumption too high: %.1fW vs expected %.1fW\n",
 i, instrument->power_consumption, expected_power);
 fault_detected = true;
 }

 // Additional checks could include:
 // - Data checksums
 // - Watchdog timers
 // - Unexpected mode transitions
 // - Internal consistency checks

 // If fault detected, set error flag and enter safe mode
 if (fault_detected) {
 instrument->error_flag = true;

 // Only enter safe mode if not already in it
 if (instrument->current_mode != MODE_SAFE) {
 change_instrument_mode(system, i, MODE_SAFE);
 }
 }
 }

 // Check overall spacecraft health
 // If too many instruments in safe mode, enter system-wide safe mode
 int safe_mode_count = 0;
 for (int i = 0; i < INSTRUMENT_COUNT; i++) {
 if (system->instruments[i].current_mode == MODE_SAFE) {
 safe_mode_count++;
 }
 }

 // If more than half the instruments are in safe mode, enter system safe mode
 if (safe_mode_count > INSTRUMENT_COUNT / 2 && !system->safe_mode_active) {
 FLIGHT_LOG("SYSTEM ALERT: Entering system-wide safe mode\n");
 system->safe_mode_active = true;

 // Put all instruments in safe mode
 for (int i = 0; i < INSTRUMENT_COUNT; i++) {
 if (system->instruments[i].current_mode != MODE_SAFE &&
 system->instruments[i].current_mode != MODE_OFF) {
 change_instrument_mode(system, i, MODE_SAFE);
 }
 }
 }
}

/* -----------------------------------------------------------------------
 * CCSDS 133.0-B-2 Space Packet Protocol — primary header layout
 * -----------------------------------------------------------------------
 * Offset Bits Field
 * 0 3 Packet Version Number (always 000 = Version 1)
 * 0 1 Packet Type (0 = TM, 1 = TC)
 * 0 1 Secondary Header Flag (1 = present)
 * 0 11 Application Process ID (APID)
 * 2 2 Sequence Flags (11 = unsegmented)
 * 2 14 Packet Sequence Count (mod 16384)
 * 4 16 Packet Data Length (octets of data field - 1)
 * ----------------------------------------------------------------------- */
#define CCSDS_VERSION 0 /* Version 1 */
#define CCSDS_TYPE_TM 0 /* Telemetry */
#define CCSDS_SEC_HDR_FLAG 1 /* Secondary header present */
#define CCSDS_APID_SCI 0x040 /* APID 64: Science Instrument TM */
#define CCSDS_SEQ_UNSEG 3 /* Unsegmented packet */
#define CCSDS_HDR_BYTES 6 /* Primary header size */

static unsigned int ccsds_seq_count = 0; /* monotonic sequence counter */

/**
 * Pack a CCSDS 133.0-B primary header into the first 6 bytes of buf[].
 * data_len is the number of octets in the data field (after the header).
 */
static void ccsds_pack_header(unsigned char buf[CCSDS_HDR_BYTES],
 unsigned int apid,
 unsigned int data_field_len)
{
 /* Word 0: Version(3) | Type(1) | SecHdrFlag(1) | APID(11) */
 unsigned int w0 = ((CCSDS_VERSION & 0x7) << 13)
 | ((CCSDS_TYPE_TM & 0x1) << 12)
 | ((CCSDS_SEC_HDR_FLAG & 0x1) << 11)
 | (apid & 0x7FF);
 buf[0] = (unsigned char)(w0 >> 8);
 buf[1] = (unsigned char)(w0 & 0xFF);

 /* Word 1: SeqFlags(2) | SeqCount(14) */
 unsigned int w1 = ((CCSDS_SEQ_UNSEG & 0x3) << 14)
 | (ccsds_seq_count & 0x3FFF);
 buf[2] = (unsigned char)(w1 >> 8);
 buf[3] = (unsigned char)(w1 & 0xFF);
 ccsds_seq_count = (ccsds_seq_count + 1) & 0x3FFF;

 /* Word 2: Packet Data Length = (data_field_len - 1) (16 bits) */
 unsigned int pdl = (data_field_len > 0) ? (data_field_len - 1) : 0;
 buf[4] = (unsigned char)(pdl >> 8);
 buf[5] = (unsigned char)(pdl & 0xFF);
}

/**
 * Generate telemetry data for downlink.
 * Packs a CCSDS 133.0-B primary header followed by instrument telemetry
 * payload aligned to 3×3 blocks for matrix-accelerator DMA compatibility.
 */
void generate_telemetry(InstrumentControlSystem *system, float *telemetry_data) {
 int telemetry_index = 0;

 /* --- CCSDS primary header (packed into first 2 floats = 8 bytes) --- */
 unsigned char pkt_hdr[CCSDS_HDR_BYTES];
 /* Estimate payload size: 9 system fields padded to 9, plus per-instrument
 27 floats × 3 instruments = 81, total ≈ 90 floats × 4 bytes = 360 bytes.
 Actual data_field_len computed after filling, header patched at end. */
 ccsds_pack_header(pkt_hdr, CCSDS_APID_SCI, 0); /* fixed-length header field */

 /* Reserve 2 float slots for the 6-byte header + 2 pad bytes */
 memcpy(&telemetry_data[0], pkt_hdr, CCSDS_HDR_BYTES);
 memset(((unsigned char*)&telemetry_data[0]) + CCSDS_HDR_BYTES, 0,
 2 * sizeof(float) - CCSDS_HDR_BYTES); /* zero-pad to 8 bytes */
 telemetry_index = 2; /* payload starts at index 2 */

 /* --- System-wide telemetry --- */
 telemetry_data[telemetry_index++] = (float)system->spacecraft_time;
 telemetry_data[telemetry_index++] = system->spacecraft_position[0];
 telemetry_data[telemetry_index++] = system->spacecraft_position[1];
 telemetry_data[telemetry_index++] = system->spacecraft_position[2];
 telemetry_data[telemetry_index++] = system->spacecraft_pointing[0];
 telemetry_data[telemetry_index++] = system->spacecraft_pointing[1];
 telemetry_data[telemetry_index++] = system->spacecraft_pointing[2];
 telemetry_data[telemetry_index++] = (float)system->safe_mode_active;
 telemetry_data[telemetry_index++] = (float)system->data_count;

 /* Pad to 3×3 block boundary for matrix-accelerator DMA alignment */
 while (telemetry_index % 9 != 0) {
 telemetry_data[telemetry_index++] = 0.0f;
 }

 /* --- Per-instrument telemetry --- */
 for (int i = 0; i < INSTRUMENT_COUNT; i++) {
 InstrumentState *instrument = &system->instruments[i];

 telemetry_data[telemetry_index++] = (float)instrument->instrument_id;
 telemetry_data[telemetry_index++] = (float)instrument->type;
 telemetry_data[telemetry_index++] = (float)instrument->current_mode;
 telemetry_data[telemetry_index++] = instrument->temperature;
 telemetry_data[telemetry_index++] = instrument->power_consumption;
 telemetry_data[telemetry_index++] = instrument->data_rate;
 telemetry_data[telemetry_index++] = (float)instrument->samples_collected;
 telemetry_data[telemetry_index++] = (float)instrument->error_flag;
 telemetry_data[telemetry_index++] = (float)instrument->last_command_time;

 /* Calibration matrix (3×3 = 9 floats, naturally aligned) */
 for (int j = 0; j < 9; j++) {
 telemetry_data[telemetry_index++] = instrument->calibration_matrix[j];
 }

 /* Health parameters (3×3 = 9 floats, naturally aligned) */
 for (int j = 0; j < 9; j++) {
 telemetry_data[telemetry_index++] = instrument->health_status[j];
 }
 }

 /* Patch CCSDS header with actual data-field length */
 unsigned int payload_bytes = (unsigned int)((telemetry_index - 2) * sizeof(float));
 ccsds_pack_header(pkt_hdr, CCSDS_APID_SCI, payload_bytes);
 memcpy(&telemetry_data[0], pkt_hdr, CCSDS_HDR_BYTES);

 FLIGHT_LOG("[SCI] TM pkt: APID=0x%03X seq=%u len=%u (%d fields, %u bytes)\n",
 CCSDS_APID_SCI, (ccsds_seq_count - 1) & 0x3FFF,
 payload_bytes, telemetry_index - 2, payload_bytes);
}

/**
 * Update the spacecraft pointing control
 */
void update_pointing_control(InstrumentControlSystem *system) {
 FLIGHT_LOG("Updating pointing control to [%.2f, %.2f, %.2f]\n",
 system->spacecraft_pointing[0],
 system->spacecraft_pointing[1],
 system->spacecraft_pointing[2]);

 // Normalize the pointing vector using hardware trig
 float magnitude = hw_sqrt(
 system->spacecraft_pointing[0] * system->spacecraft_pointing[0] +
 system->spacecraft_pointing[1] * system->spacecraft_pointing[1] +
 system->spacecraft_pointing[2] * system->spacecraft_pointing[2]
 );

 if (magnitude > 0.001f) {
 system->spacecraft_pointing[0] /= magnitude;
 system->spacecraft_pointing[1] /= magnitude;
 system->spacecraft_pointing[2] /= magnitude;
 } else {
 // Default to nadir pointing if vector is too small
 system->spacecraft_pointing[0] = 0.0f;
 system->spacecraft_pointing[1] = 0.0f;
 system->spacecraft_pointing[2] = -1.0f;
 }

 // Calculate pointing attitude adjustments using axis-angle representation
 // and convert to both rotation matrix (Rodrigues) and quaternion forms

 // For simulation, we'll just assume the pointing happens instantly
 // Update the attitude matrix to reflect the new pointing

 // First, create a rotation matrix from the original pointing [0,0,-1]
 // to the new pointing direction

 // This uses Rodrigues' rotation formula to find the rotation matrix
 float original[3] = {0.0f, 0.0f, -1.0f};
 float cross_product[3];

 // Cross product of original and new pointing
 cross_product[0] = original[1] * system->spacecraft_pointing[2] - original[2] * system->spacecraft_pointing[1];
 cross_product[1] = original[2] * system->spacecraft_pointing[0] - original[0] * system->spacecraft_pointing[2];
 cross_product[2] = original[0] * system->spacecraft_pointing[1] - original[1] * system->spacecraft_pointing[0];

 // Dot product of original and new pointing
 float dot_product =
 original[0] * system->spacecraft_pointing[0] +
 original[1] * system->spacecraft_pointing[1] +
 original[2] * system->spacecraft_pointing[2];

 // Clamp dot product to [-1, 1] range
 if (dot_product < -1.0f) dot_product = -1.0f;
 if (dot_product > 1.0f) dot_product = 1.0f;

 // Using hardware accelerated trig
 float theta = hw_atan2(hw_sqrt(
 cross_product[0] * cross_product[0] +
 cross_product[1] * cross_product[1] +
 cross_product[2] * cross_product[2]
 ), dot_product);

 // Normalize the cross product if needed
 float cp_magnitude = hw_sqrt(
 cross_product[0] * cross_product[0] +
 cross_product[1] * cross_product[1] +
 cross_product[2] * cross_product[2]
 );

 if (cp_magnitude > 0.001f) {
 cross_product[0] /= cp_magnitude;
 cross_product[1] /= cp_magnitude;
 cross_product[2] /= cp_magnitude;
 } else {
 // If vectors are parallel, use a different axis
 cross_product[0] = 1.0f;
 cross_product[1] = 0.0f;
 cross_product[2] = 0.0f;
 theta = (dot_product < 0.0f) ? M_PI : 0.0f;
 }

 // Using Rodrigues' formula for rotation matrix
 // R = I + sin(θ) * K + (1 - cos(θ)) * K^2
 // Where K is the skew-symmetric matrix of the normalized axis

 // HW accelerated trig
 float sin_theta = hw_sin(theta);
 float one_minus_cos_theta = 1.0f - hw_cos(theta);

 // Skew-symmetric matrix K
 float K[9] = {
 0.0f, -cross_product[2], cross_product[1],
 cross_product[2], 0.0f, -cross_product[0],
 -cross_product[1], cross_product[0], 0.0f
 };

 // K^2 (matrix product K*K)
 float K2[9];
 hw_matrix3x3_multiply(K, K, K2);

 // Calculate rotation matrix R = I + sin(θ) * K + (1 - cos(θ)) * K^2
 // Starting with identity matrix
 float R[9] = {
 1.0f, 0.0f, 0.0f,
 0.0f, 1.0f, 0.0f,
 0.0f, 0.0f, 1.0f
 };

 // Add sin(θ) * K
 for (int i = 0; i < 9; i++) {
 R[i] += sin_theta * K[i];
 }

 // Add (1 - cos(θ)) * K^2
 for (int i = 0; i < 9; i++) {
 R[i] += one_minus_cos_theta * K2[i];
 }

 // Convert axis-angle to quaternion representation (Quaternion BCE)
 // q = [cos(θ/2), sin(θ/2) * axis]
 float half_theta = theta * 0.5f;
 float sin_half = hw_sin(half_theta);
 float cos_half = hw_cos(half_theta);
 float error_quat[4];
 error_quat[0] = cos_half; // scalar part
 error_quat[1] = sin_half * cross_product[0]; // vector x
 error_quat[2] = sin_half * cross_product[1]; // vector y
 error_quat[3] = sin_half * cross_product[2]; // vector z

 // Normalize quaternion to unit length
 float q_norm = hw_sqrt(error_quat[0]*error_quat[0] + error_quat[1]*error_quat[1] +
 error_quat[2]*error_quat[2] + error_quat[3]*error_quat[3]);
 if (q_norm > 1e-6f) {
 error_quat[0] /= q_norm;
 error_quat[1] /= q_norm;
 error_quat[2] /= q_norm;
 error_quat[3] /= q_norm;
 }
 /* Feed error quaternion to instrument pointing budget accumulator */
 float pointing_err_arcsec = 2.0f * hw_asin(hw_sqrt(error_quat[1]*error_quat[1]
 + error_quat[2]*error_quat[2] + error_quat[3]*error_quat[3])) * 206265.0f;
 if (pointing_err_arcsec > 30.0f) {
 FLIGHT_LOG("POINT: error %.1f arcsec exceeds fine-guidance budget\n",
 (double)pointing_err_arcsec);
 }

 // Update the spacecraft attitude with the new rotation matrix
 memcpy(system->spacecraft_attitude, R, sizeof(float) * 9);
}

/**
 * Queue a new command for execution
 */
void queue_command(InstrumentControlSystem *system, Command cmd) {
 if (system->command_count >= MAX_COMMAND_QUEUE) {
 FLIGHT_LOG("Command queue full, dropping command %d\n", cmd.command_id);
 return;
 }

 system->command_queue[system->command_count++] = cmd;
 FLIGHT_LOG("Command %d queued for instrument %d at time %d\n",
 cmd.command_id, cmd.instrument_id, cmd.execution_time);
}

/**
 * Generate science data products from processed data
 */
void generate_science_data_products(InstrumentControlSystem *system) {
 FLIGHT_LOG("Generating science data products from %d samples\n", system->data_count);

 // First, make sure all data is processed
 process_raw_data(system);

 // For each instrument type, generate appropriate data products
 for (unsigned int inst_id = 0; inst_id < INSTRUMENT_COUNT; inst_id++) {
 InstrumentState *instrument = &system->instruments[inst_id];

 // Skip if no data for this instrument
 bool has_data = false;
 for (unsigned int i = 0; i < system->data_count; i++) {
 if (system->data_buffer[i].instrument_id == inst_id) {
 has_data = true;
 break;
 }
 }

 if (!has_data) {
 continue;
 }

 FLIGHT_LOG("Generating products for instrument %d (%d)\n",
 inst_id, instrument->type);

 switch (instrument->type) {
 case INSTRUMENT_CAMERA: {
 // For camera, we might create an image mosaic
 // Assemble image tiles into full-frame mosaic

 // Just simulate the process
 FLIGHT_LOG("Generated camera image product\n");
 break;
 }

 case INSTRUMENT_SPECTROMETER: {
 // Spectral analysis via DFT on spectrometer readings
 float spec_signal[FFT_LEN], spec_mag[FFT_LEN];
 int n_samples = (int)instrument->samples_collected;
 if (n_samples > FFT_LEN) n_samples = FFT_LEN;
 for (int s = 0; s < n_samples; s++)
 spec_signal[s] = system->data_buffer[s % (system->data_count > 0 ? system->data_count : 1)].raw_data[s % 9] * (1.0f + 0.1f * s);
 for (int s = n_samples; s < FFT_LEN; s++)
 spec_signal[s] = 0.0f;
 // Compute DFT magnitude
 for (int k = 0; k < FFT_LEN; k++) {
 float re = 0.0f, im = 0.0f;
 for (int n = 0; n < FFT_LEN; n++) {
 float angle = 2.0f * M_PI * k * n / FFT_LEN;
 re += spec_signal[n] * hw_cos(angle);
 im -= spec_signal[n] * hw_sin(angle);
 }
 spec_mag[k] = hw_sqrt(re * re + im * im);
 }
 // Matrix-based spectral covariance (18x18)
 {
 static double cov[STATE_DIM][STATE_DIM];
 static double temp[STATE_DIM][STATE_DIM];
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 cov[i][j] = (i == j) ? spec_mag[i % FFT_LEN] + 1.0 : 0.01 * spec_mag[(i+j) % FFT_LEN];
 // Matrix multiply cov * cov -> temp (18x18)
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 temp[i][j] = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 temp[i][j] += cov[i][k] * cov[k][j];
 }
 FLIGHT_LOG("Generated spectral analysis product (DFT + %dx%d covariance)\n", STATE_DIM, STATE_DIM);
 }
 break;
 }

 case INSTRUMENT_MAGNETOMETER: {
 // Magnetic field tensor analysis with 18x18 matrices
 static double field_tensor[STATE_DIM][STATE_DIM];
 static double field_result[STATE_DIM][STATE_DIM];
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++)
 field_tensor[i][j] = (i == j) ? 1.0 + 0.1 * i : 0.001 * (i - j);
 // Matrix multiply field_tensor * field_tensor -> field_result
 for (int i = 0; i < STATE_DIM; i++)
 for (int j = 0; j < STATE_DIM; j++) {
 field_result[i][j] = 0.0;
 for (int k = 0; k < STATE_DIM; k++)
 field_result[i][j] += field_tensor[i][k] * field_tensor[k][j];
 }
 FLIGHT_LOG("Generated magnetic field map product (%dx%d tensor)\n", STATE_DIM, STATE_DIM);
 break;
 }
 }
 }
}

/*****************************************************************************
 * Science Instrument Calibration Extended Kalman Filter (EKF)
 *
 * 12-state EKF that estimates and tracks instrument calibration drift:
 * States 0- 2 : Instrument gain drift (3 channels)
 * States 3- 5 : Offset / bias (3 channels)
 * States 6- 8 : Temperature-dependent calibration coefficients (3 params)
 * States 9-11 : Pointing error (3 axes)
 *
 * Predict model: x_{k+1} = F * x_k + w (near-identity, slow drift)
 * Update model: z_k = H * x_k + v (calibration measurements)
 * Standard EKF: P_pred = F*P*F' + Q
 * S = H*P_pred*H' + R
 * K = P_pred * H' * S^{-1}
 * x_up = x_pred + K * (z - H*x_pred)
 * P_up = (I-KH) P_pred (I-KH)^T + K R K^T [Joseph form]
 *
 * The Joseph stabilised covariance update preserves P positive-semidefinite
 * even under gain mismatch or SEU-induced perturbations. The inverse is
 * computed via Gauss-Jordan with a Frobenius-norm-relative singularity guard.
 *****************************************************************************/

/* Calibration EKF persistent state */
typedef struct {
 double x[CALIB_STATE_DIM]; /* state vector */
 double P[CALIB_STATE_DIM][CALIB_STATE_DIM]; /* covariance */
 double F[CALIB_STATE_DIM][CALIB_STATE_DIM]; /* state transition */
 double Q[CALIB_STATE_DIM][CALIB_STATE_DIM]; /* process noise */
 double H[CALIB_STATE_DIM][CALIB_STATE_DIM]; /* observation matrix */
 double R[CALIB_STATE_DIM][CALIB_STATE_DIM]; /* measurement noise */
 unsigned int step_count;
} CalibrationEKF;

/**
 * Gauss-Jordan matrix inverse for CALIB_STATE_DIM x CALIB_STATE_DIM.
 * Computes inv(A) in-place into Ainv. Returns 0 on success, -1 if singular.
 *
 * Singularity detection uses a norm-relative threshold rather than a fixed
 * absolute value. In high-radiation orbits, single-event upsets (SEUs) can
 * corrupt individual matrix entries, causing sudden ill-conditioning.
 * A column-Frobenius-relative pivot check catches these cases more reliably
 * than the traditional 1e-15 absolute cutoff.
 */
static int gauss_jordan_inverse_12(double A[CALIB_STATE_DIM][CALIB_STATE_DIM],
 double Ainv[CALIB_STATE_DIM][CALIB_STATE_DIM])
{
 const int N = CALIB_STATE_DIM;
 double aug[CALIB_STATE_DIM][2 * CALIB_STATE_DIM];

 /* Build augmented matrix [A | I] */
 for (int i = 0; i < N; i++) {
 for (int j = 0; j < N; j++) {
 aug[i][j] = A[i][j];
 aug[i][j + N] = (i == j) ? 1.0 : 0.0;
 }
 }

 /* Compute Frobenius norm of the original matrix for relative threshold */
 double frob_sq = 0.0;
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++)
 frob_sq += A[i][j] * A[i][j];
 double frob_norm = sqrt(frob_sq);
 /* Relative singularity threshold: eps_mach * norm, floored at 1e-30 */
 double sing_thr = (frob_norm > 0.0) ? 1e-12 * frob_norm : 1e-30;

 /* Forward elimination with partial pivoting */
 for (int col = 0; col < N; col++) {
 /* Find pivot */
 int pivot = col;
 double max_val = fabs(aug[col][col]);
 for (int row = col + 1; row < N; row++) {
 if (fabs(aug[row][col]) > max_val) {
 max_val = fabs(aug[row][col]);
 pivot = row;
 }
 }
 if (max_val < sing_thr) return -1; /* Singular (norm-relative) */

 /* Swap rows */
 if (pivot != col) {
 for (int j = 0; j < 2 * N; j++) {
 double tmp = aug[col][j];
 aug[col][j] = aug[pivot][j];
 aug[pivot][j] = tmp;
 }
 }

 /* Scale pivot row */
 double diag = aug[col][col];
 for (int j = 0; j < 2 * N; j++)
 aug[col][j] /= diag;

 /* Eliminate column */
 for (int row = 0; row < N; row++) {
 if (row == col) continue;
 double factor = aug[row][col];
 for (int j = 0; j < 2 * N; j++)
 aug[row][j] -= factor * aug[col][j];
 }
 }

 /* Extract inverse from right half */
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++)
 Ainv[i][j] = aug[i][j + N];

 return 0;
}

/**
 * Initialise the 12-state calibration EKF.
 */
static void calib_ekf_init(CalibrationEKF *ekf)
{
 const int N = CALIB_STATE_DIM;
 memset(ekf, 0, sizeof(CalibrationEKF));

 /* ----- State transition F: near-identity with slow drift terms ----- */
 for (int i = 0; i < N; i++)
 ekf->F[i][i] = 1.0;

 /* Gain drift (states 0-2): random walk (F=1.0, no exponential growth) */
 ekf->F[0][0] = 1.0; ekf->F[1][1] = 1.0; ekf->F[2][2] = 1.0;
 /* Bias (states 3-5): random walk (F=1.0, no exponential growth) */
 ekf->F[3][3] = 1.0; ekf->F[4][4] = 1.0; ekf->F[5][5] = 1.0;
 /* Temperature coeff (states 6-8) couple from temperature via small off-diag */
 ekf->F[6][0] = 0.0002; ekf->F[7][1] = 0.0002; ekf->F[8][2] = 0.0002;
 /* Pointing error (states 9-11) is nearly static */
 ekf->F[9][9] = 0.9999;
 ekf->F[10][10] = 0.9999;
 ekf->F[11][11] = 0.9999;

 /* ----- Process noise Q (diagonal) ----- */
 for (int i = 0; i < 3; i++) ekf->Q[i][i] = 1e-6; /* gain drift */
 for (int i = 3; i < 6; i++) ekf->Q[i][i] = 1e-7; /* bias */
 for (int i = 6; i < 9; i++) ekf->Q[i][i] = 1e-8; /* temp coeff */
 for (int i = 9; i < 12; i++) ekf->Q[i][i] = 1e-7; /* pointing err */

 /* ----- Observation matrix H: direct observation of all 12 states ----- */
 /* We measure dark-current reference (gain+bias) and calibration source */
 for (int i = 0; i < N; i++)
 ekf->H[i][i] = 1.0;
 /* Cross-coupling: temperature coefficients affect gain measurement */
 ekf->H[0][6] = 0.05; ekf->H[1][7] = 0.05; ekf->H[2][8] = 0.05;

 /* ----- Measurement noise R (diagonal) ----- */
 for (int i = 0; i < 3; i++) ekf->R[i][i] = 0.01; /* gain meas */
 for (int i = 3; i < 6; i++) ekf->R[i][i] = 0.005; /* bias meas */
 for (int i = 6; i < 9; i++) ekf->R[i][i] = 0.02; /* temp coeff */
 for (int i = 9; i < 12; i++) ekf->R[i][i] = 0.008; /* pointing err */

 /* ----- Initial covariance P (diagonal, moderate uncertainty) ----- */
 for (int i = 0; i < N; i++)
 ekf->P[i][i] = 1.0;

 /* ----- Initial state: nominal calibration (small perturbation) ----- */
 for (int i = 0; i < 3; i++) ekf->x[i] = 1.0; /* unity gain */
 for (int i = 3; i < 6; i++) ekf->x[i] = 0.0; /* zero bias */
 for (int i = 6; i < 9; i++) ekf->x[i] = 0.001; /* small temp c */
 for (int i = 9; i < 12; i++) ekf->x[i] = 0.0; /* zero pt err */

 ekf->step_count = 0;
 FLIGHT_LOG("Calibration EKF initialised (%d states).\n", N);
}

/**
 * EKF Predict step: x_pred = F * x, P_pred = F * P * F' + Q
 */
static void calib_ekf_predict(CalibrationEKF *ekf,
 double x_pred[CALIB_STATE_DIM],
 double P_pred[CALIB_STATE_DIM][CALIB_STATE_DIM])
{
 const int N = CALIB_STATE_DIM;

 /* x_pred = F * x */
 for (int i = 0; i < N; i++) {
 x_pred[i] = 0.0;
 for (int j = 0; j < N; j++)
 x_pred[i] += ekf->F[i][j] * ekf->x[j];
 }

 /* FP = F * P (N x N temporary) */
 double FP[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 FP[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 FP[i][j] += ekf->F[i][k] * ekf->P[k][j];
 }

 /* P_pred = FP * F' + Q */
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 P_pred[i][j] = ekf->Q[i][j];
 for (int k = 0; k < N; k++)
 P_pred[i][j] += FP[i][k] * ekf->F[j][k]; /* F' => [j][k] */
 }
}

/**
 * EKF Update step:
 * S = H * P_pred * H' + R
 * K = P_pred * H' * S^{-1}
 * x = x_pred + K * (z - H * x_pred)
 * P = (I - K * H) * P_pred
 */
static void calib_ekf_update(CalibrationEKF *ekf,
 const double x_pred[CALIB_STATE_DIM],
 double P_pred[CALIB_STATE_DIM][CALIB_STATE_DIM],
 const double z[CALIB_STATE_DIM])
{
 const int N = CALIB_STATE_DIM;

 /* --- Innovation: y = z - H * x_pred --- */
 double y[CALIB_STATE_DIM];
 for (int i = 0; i < N; i++) {
 double Hx = 0.0;
 for (int j = 0; j < N; j++)
 Hx += ekf->H[i][j] * x_pred[j];
 y[i] = z[i] - Hx;
 }

 /* --- HP = H * P_pred --- */
 double HP[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 HP[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 HP[i][j] += ekf->H[i][k] * P_pred[k][j];
 }

 /* --- S = HP * H' + R --- */
 double S[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 S[i][j] = ekf->R[i][j];
 for (int k = 0; k < N; k++)
 S[i][j] += HP[i][k] * ekf->H[j][k]; /* H' => [j][k] */
 }

 /* --- S_inv = S^{-1} (Gauss-Jordan) --- */
 double S_inv[CALIB_STATE_DIM][CALIB_STATE_DIM];
 if (gauss_jordan_inverse_12(S, S_inv) != 0) {
 /* Singular S – skip update, keep prediction */
 FLIGHT_LOG(" [CalibEKF] WARNING: S matrix singular, skipping update.\n");
 for (int i = 0; i < N; i++) {
 ekf->x[i] = x_pred[i];
 for (int j = 0; j < N; j++)
 ekf->P[i][j] = P_pred[i][j];
 }
 return;
 }

 /* --- PHt = P_pred * H' --- */
 double PHt[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 PHt[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 PHt[i][j] += P_pred[i][k] * ekf->H[j][k]; /* H' => [j][k] */
 }

 /* --- Kalman gain K = PHt * S_inv --- */
 double K[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 K[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 K[i][j] += PHt[i][k] * S_inv[k][j];
 }

 /* --- State update: x = x_pred + K * y --- */
 for (int i = 0; i < N; i++) {
 ekf->x[i] = x_pred[i];
 for (int j = 0; j < N; j++)
 ekf->x[i] += K[i][j] * y[j];
 }

 /* --- Joseph-form covariance update (numerically stable) ---
 * P = (I-KH) P_pred (I-KH)^T + K R K^T
 * This preserves positive-semidefiniteness of P even when K
 * is slightly sub-optimal — critical in radiation environments
 * where SEU-induced bit-flips can perturb matrix entries. */

 /* IKH = I - K*H */
 double IKH[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 double kh = 0.0;
 for (int k = 0; k < N; k++)
 kh += K[i][k] * ekf->H[k][j];
 IKH[i][j] = ((i == j) ? 1.0 : 0.0) - kh;
 }

 /* term1 = IKH * P_pred * IKH^T */
 double IKH_P[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 IKH_P[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 IKH_P[i][j] += IKH[i][k] * P_pred[k][j];
 }
 double term1[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 term1[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 term1[i][j] += IKH_P[i][k] * IKH[j][k]; /* IKH^T => [j][k] */
 }

 /* term2 = K * R * K^T */
 double KR[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 KR[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 KR[i][j] += K[i][k] * ekf->R[k][j];
 }
 double term2[CALIB_STATE_DIM][CALIB_STATE_DIM];
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++) {
 term2[i][j] = 0.0;
 for (int k = 0; k < N; k++)
 term2[i][j] += KR[i][k] * K[j][k]; /* K^T => [j][k] */
 }

 /* P = term1 + term2, then enforce symmetry */
 for (int i = 0; i < N; i++)
 for (int j = 0; j < N; j++)
 ekf->P[i][j] = term1[i][j] + term2[i][j];

 for (int i = 0; i < N; i++)
 for (int j = i + 1; j < N; j++) {
 double avg = 0.5 * (ekf->P[i][j] + ekf->P[j][i]);
 ekf->P[i][j] = avg;
 ekf->P[j][i] = avg;
 }
}

/**
 * Run one full EKF cycle (predict + update) using current instrument state.
 * Synthesises calibration measurements from the instrument control system:
 * - Dark-current reference -> gain drift + bias observability
 * - Known calibration source -> temperature coefficients
 * - Star-tracker residuals -> pointing error
 */
static void calib_ekf_step(CalibrationEKF *ekf,
 const InstrumentControlSystem *system)
{
 const int N = CALIB_STATE_DIM;

 /* ---------- Predict ---------- */
 double x_pred[CALIB_STATE_DIM];
 double P_pred[CALIB_STATE_DIM][CALIB_STATE_DIM];
 calib_ekf_predict(ekf, x_pred, P_pred);

 /* ---------- Synthesise measurement vector z ---------- */
 double z[CALIB_STATE_DIM];

 /* Gain drift measurement from dark-current reference (per channel) */
 for (int ch = 0; ch < 3; ch++) {
 float cal_diag = system->instruments[ch].calibration_matrix[ch * 3 + ch];
 z[ch] = (double)cal_diag + 0.001 * ((rand() % 1000) / 1000.0 - 0.5);
 }

 /* Bias measurement from zero-level reference */
 for (int ch = 0; ch < 3; ch++) {
 z[3 + ch] = 0.0 + 0.0005 * ((rand() % 1000) / 1000.0 - 0.5);
 }

 /* Temperature-dependent calibration coefficient measurement */
 for (int ch = 0; ch < 3; ch++) {
 float temp = system->instruments[ch].temperature;
 z[6 + ch] = 0.001 * (double)temp / 20.0
 + 0.0002 * ((rand() % 1000) / 1000.0 - 0.5);
 }

 /* Pointing error measurement from star-tracker residuals (arcsec -> rad) */
 {
 double desired[3] = {0.0, 0.0, 1.0}; /* Nadir reference */
 for (int ax = 0; ax < 3; ax++) {
 double noise = 0.0001 * ((rand() % 1000) / 1000.0 - 0.5);
 z[9 + ax] = ((double)system->spacecraft_pointing[ax] - desired[ax]) * 0.0001 + noise;
 }
 }

 /* ---------- Update ---------- */
 calib_ekf_update(ekf, x_pred, P_pred, z);
 ekf->step_count++;

 /* Periodic status report */
 if (ekf->step_count % 30 == 0) {
 double trace_P = 0.0;
 for (int i = 0; i < N; i++) trace_P += ekf->P[i][i];
 FLIGHT_LOG(" [CalibEKF] step %u trP=%.6e gain=[%.5f %.5f %.5f] "
 "bias=[%.2e %.2e %.2e] ptErr=[%.2e %.2e %.2e]\n",
 ekf->step_count, trace_P,
 ekf->x[0], ekf->x[1], ekf->x[2],
 ekf->x[3], ekf->x[4], ekf->x[5],
 ekf->x[9], ekf->x[10], ekf->x[11]);
 }
}

/**
 * Main function demonstrates the use of the scientific instrument control system
 */
int main() {
 /* Deterministic seed: rand() generates synthetic sensor/state data that exercises
 the same compute kernels (Kalman, FFT, matrix ops) as real mission telemetry.
 Fixed seed ensures reproducible execution traces for gem5 HW/SW comparison. */
 srand(5107);

 // Initialize the system
 InstrumentControlSystem system;
 initialize_system(&system);

 // Initialise science-instrument calibration EKF (12 states)
 CalibrationEKF calib_ekf;
 calib_ekf_init(&calib_ekf);

 // Queue some commands 

 // Power on all instruments to standby
 for (int i = 0; i < INSTRUMENT_COUNT; i++) {
 Command cmd = {0};
 cmd.command_id = 1; // Power on
 cmd.instrument_id = i;
 cmd.execution_time = system.spacecraft_time + 10 + i*5;
 queue_command(&system, cmd);
 }

 // Start science mode for camera
 Command science_cmd = {0};
 science_cmd.command_id = 2; // Start science
 science_cmd.instrument_id = 0; // Camera
 science_cmd.execution_time = system.spacecraft_time + 30;
 queue_command(&system, science_cmd);

 // Change pointing
 Command point_cmd = {0};
 point_cmd.command_id = 5; // Change pointing
 point_cmd.instrument_id = 0; // Camera (though applies to whole spacecraft)
 point_cmd.parameters[0] = 0.5f; // X
 point_cmd.parameters[1] = 0.5f; // Y
 point_cmd.parameters[2] = -0.7f; // Z
 point_cmd.execution_time = system.spacecraft_time + 40;
 queue_command(&system, point_cmd);

 // Collect data samples
 for (int i = 0; i < 5; i++) {
 Command data_cmd = {0};
 data_cmd.command_id = 4; // Collect data
 data_cmd.instrument_id = 0; // Camera
 data_cmd.execution_time = system.spacecraft_time + 50 + i*10;
 queue_command(&system, data_cmd);
 }

 // Calibrate magnetometer
 Command cal_cmd = {0};
 cal_cmd.command_id = 3; // Calibrate
 cal_cmd.instrument_id = 2; // Magnetometer
 cal_cmd.execution_time = system.spacecraft_time + 80;
 queue_command(&system, cal_cmd);

 // Process data
 Command proc_cmd = {0};
 proc_cmd.command_id = 6; // Process data
 proc_cmd.instrument_id = 0; // Not used
 proc_cmd.execution_time = system.spacecraft_time + 100;
 queue_command(&system, proc_cmd);

 // Generate science products
 Command gen_cmd = {0};
 gen_cmd.command_id = 7; // Generate products
 gen_cmd.instrument_id = 0; // Not used
 gen_cmd.execution_time = system.spacecraft_time + 110;
 queue_command(&system, gen_cmd);

 // Main execution loop
 for (int sim_step = 0; sim_step < 120; sim_step++) {
 FLIGHT_LOG("\n[SCI] step=%d t=%d\n",
 sim_step, system.spacecraft_time);

 // Execute commands
 execute_command_sequence(&system);

 // Check for faults
 handle_fault_detection(&system);

 // Generate telemetry (but don't use it in this demo)
 float telemetry[TELEMETRY_PACKET_SIZE];
 generate_telemetry(&system, telemetry);

 // Run calibration EKF (predict + update) every timestep
 calib_ekf_step(&calib_ekf, &system);

 // Update spacecraft position (LEO reference orbit for benchmark)
 float orbit_radius = 6800.0f; // km (LEO altitude ~430 km)
 float orbit_period = 90.0f * 60.0f; // 90 minutes in seconds (LEO)
 float angle = (2.0f * M_PI * system.spacecraft_time) / orbit_period;

 system.spacecraft_position[0] = orbit_radius * hw_cos(angle);
 system.spacecraft_position[1] = orbit_radius * hw_sin(angle);
 system.spacecraft_position[2] = 0.0f;

 // Update temperatures based on orbit position (day/night cycle)
 // Simple thermal model: hot in sunlight, cold in eclipse
 for (int i = 0; i < INSTRUMENT_COUNT; i++) {
 // Check if in eclipse (negative X position is in Earth's shadow in this model)
 bool in_eclipse = (system.spacecraft_position[0] < 0);

 // Target temperature based on orbit position
 float target_temp = in_eclipse ? -10.0f : 30.0f;

 // Temperature changes slowly with thermal inertia
 system.instruments[i].temperature =
 0.99f * system.instruments[i].temperature +
 0.01f * target_temp;

 // Add some random noise
 system.instruments[i].temperature +=
 (rand() % 100) / 100.0f - 0.5f;
 }

 // Increment spacecraft time
 system.spacecraft_time++;
 }

 FLIGHT_LOG("\n[SCI] done\n");
 return 0;
}
