/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright 2025 the authors of IEEE TC paper TC-2025-09-0830
 * ("Characterizing and Accelerating Spacecraft Onboard Workloads
 *  on RISC-V Platform").  Licensed under the Apache License,
 * Version 2.0; see LICENSE at the root of this repository.
 *
 * Synthetic workload derived for academic research from
 * production spacecraft onboard flight software characteristics.
 */
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
#ifndef SPACECRAFT_AI_H
#define SPACECRAFT_AI_H

/*
 * spacecraft_ai.h -- public interface
 *
 * Ten on-board AI modules:
 * 1. Optical nav (CNN + EKF) 6. Hyperspectral classifier
 * 2. Hazard segmentation 7. FDIR autoencoder
 * 3. Multi-sensor fusion (EKF + ML) 8. Science prioritisation
 * 4. Docking pose estimation 9. Adaptive compression
 * 5. Swarm consensus 10. IMU bias (GRU)
 *
 * Weight arrays are static; replace stubs with trained values before flight.
 *
 * STATIC ALLOCATION POLICY
 * ~~~~~~~~~~~~~~~~~~~~~~~~
 * No malloc / calloc / realloc / free anywhere in this codebase.
 * All buffers are either:
 * (a) file-scope static arrays (BSS / data segment),
 * (b) function-scope static arrays (BSS, persistent across calls), or
 * (c) small stack temporaries (<512 B per call frame).
 * This guarantees deterministic WCET with no heap fragmentation,
 * a hard requirement for Class-A flight software.
 */

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
/* NOTE: <stdlib.h> deliberately omitted — no dynamic allocation permitted.
 * See STATIC ALLOCATION POLICY above. */

/* compile-time sizing */
#define SC_EKF_STATE_DIM 15 /* pos(3) vel(3) att(4) gyro_b(3) acc_b(2) */
#define SC_EKF_MEAS_DIM_OPT 5 /* pixel centroid(2) + crater diameters(3) */
#define SC_EKF_MEAS_DIM_FUSE 10 /* all sensors combined */

#define SC_IMG_W 128
#define SC_IMG_H 128
#define SC_IMG_C 3 /* RGB channels */

#define SC_HYPER_BANDS 64 /* hyperspectral channels */
#define SC_HYPER_CLASSES 12 /* surface material classes */

#define SC_SWARM_MAX_NODES 16
#define SC_FDIR_SENSOR_DIM 32 /* telemetry vector length */
#define SC_FDIR_LATENT_DIM 8 /* autoencoder bottleneck */
#define SC_SCI_FEATURE_DIM 24
#define SC_SCI_LATENT_DIM 6

#define SC_IMU_WIN 20 /* GRU look-back window */
#define SC_IMU_HIDDEN 16

#define SC_COMPRESS_BLOCK 64 /* adaptive block size */

#define PI_F 3.14159265358979f
#define EPS 1e-9f

/* WGS-84 constants */
#define SC_MU_EARTH 3.986004418e14f /* m^3/s^2 geocentric GM */
#define SC_R_EARTH 6.378137e6f /* m equatorial radius */

/* fixed-size types */
typedef float Mat[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM];
typedef float Vec[SC_EKF_STATE_DIM];
typedef float Quat[4]; /* w x y z */

/* result structs (one per workload) */
typedef struct {
 float position[3]; /* m ECI */
 float velocity[3]; /* m/s */
 float attitude[4]; /* quaternion w x y z */
 float P[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM]; /* covariance */
 float innovation_norm;
 uint8_t converged;
} OptNavResult;

typedef struct {
 uint8_t label_map[SC_IMG_H][SC_IMG_W]; /* 0=safe 1=hazard 2=unknown */
 float confidence[SC_IMG_H][SC_IMG_W];
 float hazard_fraction; /* 0-1 */
 uint8_t safe_landing_zone_found;
 float best_lz_center[2]; /* pixel coords */
} HazardResult;

typedef struct {
 float state[SC_EKF_STATE_DIM];
 float P[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM];
 float ml_bias_correction[6]; /* gyro(3)+accel(3) */
 float sensor_weights[8]; /* adaptive per-sensor trust */
 float integrity_flags; /* RAIM-style */
} FusionResult;

typedef struct {
 float relative_pos[3]; /* m target-to-chaser in LVLH */
 float relative_vel[3]; /* m/s */
 float relative_att[4]; /* quaternion */
 float pose_covariance[6][6];
 uint8_t docking_phase; /* 0=far 1=approach 2=final 3=capture */
 float approach_corridor_score; /* 1=centred */
} DockingResult;

typedef struct {
 float consensus_state[6]; /* pos(3) vel(3) average across swarm */
 float agent_weights[SC_SWARM_MAX_NODES];
 uint8_t n_active;
 uint8_t topology_connected;
 float convergence_metric;
} SwarmResult;

typedef struct {
 uint8_t predicted_class;
 float class_probs[SC_HYPER_CLASSES];
 float confidence;
 char label[32];
} HyperResult;

typedef struct {
 float reconstruction_error;
 float anomaly_score; /* z-score from running stats */
 uint8_t fault_detected;
 uint8_t fault_component; /* which subsystem flagged */
 char fault_description[64];
 uint8_t recovery_action; /* action index for FDIR table */
} FDIRResult;

typedef struct {
 float priority_score; /* 0-1, higher -> downlink first */
 float novelty;
 float scientific_value;
 uint8_t recommend_downlink;
 uint8_t recommend_onboard_process;
} ScienceResult;

typedef struct {
 float compression_ratio;
 uint32_t compressed_bytes;
 uint8_t quality_level; /* 0-7 */
 float psnr_estimate;
} CompressResult;

typedef struct {
 float bias_gyro[3]; /* rad/s */
 float bias_accel[3]; /* m/s² */
 float scale_factor[6];
 float uncertainty[6];
} IMUBiasResult;

/* main context struct (BSS-resident) */
typedef struct SpacecraftAI {
 /* EKF states */
 float ekf_x[SC_EKF_STATE_DIM];
 float ekf_P[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM];

 /* running statistics for FDIR */
 float fdir_mu[SC_FDIR_SENSOR_DIM];
 float fdir_sigma[SC_FDIR_SENSOR_DIM];
 uint32_t fdir_n;

 /* GRU hidden state for IMU bias */
 float gru_h[SC_IMU_HIDDEN];

 /* swarm adjacency & states */
 float swarm_adj[SC_SWARM_MAX_NODES][SC_SWARM_MAX_NODES];
 float swarm_x[SC_SWARM_MAX_NODES][6];

 /* telemetry cycle counter */
 uint64_t tick;
} SpacecraftAI;

/* public API */
void spacecraft_ai_init(SpacecraftAI *ctx);

OptNavResult optical_nav_cnn_ekf (SpacecraftAI *ctx,
 const float img[SC_IMG_H][SC_IMG_W][SC_IMG_C],
 const float imu_gyro[3],
 float dt);

HazardResult hazard_detection_segment (const float img[SC_IMG_H][SC_IMG_W][SC_IMG_C]);

FusionResult multisensor_fusion_ekf (SpacecraftAI *ctx,
 const float imu_acc[3],
 const float imu_gyro[3],
 const float gps[3],
 const float star_q[4],
 float dt);

DockingResult autonomous_docking_vision (const float img[SC_IMG_H][SC_IMG_W][SC_IMG_C],
 const float imu_acc[3]);

SwarmResult swarm_consensus (SpacecraftAI *ctx,
 uint8_t node_id,
 const float neighbor_states[][6],
 uint8_t n_neighbors);

HyperResult hyperspectral_classify (const float spectrum[SC_HYPER_BANDS]);

FDIRResult fdir_autoencoder (SpacecraftAI *ctx,
 const float telemetry[SC_FDIR_SENSOR_DIM]);

ScienceResult science_prioritize (const float features[SC_SCI_FEATURE_DIM]);

CompressResult adaptive_compress (const uint8_t *data,
 size_t n_bytes,
 float quality_target);

IMUBiasResult imu_bias_network (SpacecraftAI *ctx,
 const float raw_gyro[3],
 const float raw_accel[3]);

#endif /* SPACECRAFT_AI_H */
