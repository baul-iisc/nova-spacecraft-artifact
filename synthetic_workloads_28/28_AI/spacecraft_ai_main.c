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
/*
 * spacecraft_ai_main.c -- demo driver
 *
 * Exercises all 10 AI workloads with synthetic sensor data.
 *
 * Build: make
 * Run: ./spacecraft_ai
 */

/* Enable clock_gettime on POSIX (must precede all system headers) */
#ifndef __riscv
 #if !defined(_POSIX_C_SOURCE) || _POSIX_C_SOURCE < 199309L
 #undef _POSIX_C_SOURCE
 #define _POSIX_C_SOURCE 199309L
 #endif
#endif

#include "spacecraft_ai.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../flight_compliance.h"

/* ========================================================================= */
/* HIGH-RESOLUTION TIMER (HRT) */
/* ========================================================================= */
/* RISC-V: rdcycle CSR gives cycle-accurate, monotonic count with zero */
/* kernel overhead. Ideal for flight-grade WCET measurement. */
/* Fallback: clock_gettime(CLOCK_MONOTONIC) on Linux/POSIX (ns precision). */
/* The abstraction returns elapsed microseconds between two snapshots. */

#ifdef __riscv
 /* RISC-V cycle counter — accessible in U-mode when mcounteren.CY=1 */
 typedef uint64_t hrt_t;
 static inline hrt_t hrt_now(void) {
 uint64_t cy;
 __asm__ volatile ("rdcycle %0" : "=r"(cy));
 return cy;
 }
 /* Convert cycles to microseconds. SC_CPU_MHZ must be defined at build
 * time for the target core clock, e.g. -DSC_CPU_MHZ=100 for 100 MHz. */
 #ifndef SC_CPU_MHZ
 #define SC_CPU_MHZ 100 /* default: 100 MHz (typical RISC-V SoC) */
 #endif
 static inline double hrt_elapsed_us(hrt_t start, hrt_t end) {
 return (double)(end - start) / (double)SC_CPU_MHZ;
 }
#else
 /* POSIX fallback: clock_gettime(CLOCK_MONOTONIC) */
 #include <time.h>
 typedef struct timespec hrt_t;
 static inline hrt_t hrt_now(void) {
 hrt_t ts;
 clock_gettime(CLOCK_MONOTONIC, &ts);
 return ts;
 }
 static inline double hrt_elapsed_us(hrt_t start, hrt_t end) {
 double s = (double)(end.tv_sec - start.tv_sec) * 1e6;
 double ns = (double)(end.tv_nsec - start.tv_nsec) / 1e3;
 return s + ns;
 }
#endif

/* --- synthetic sensor generators ---------------------------------------- */

/* Deterministic pseudo-random float in [lo, hi] */
static uint32_t _seed = 0xCAFEBABE;
static float frand(float lo, float hi) {
 _seed = _seed * 1664525u + 1013904223u;
 float t = (float)(_seed >> 8) / (float)(1u<<24);
 return lo + t*(hi-lo);
}

/* Lunar surface image (craters + varying albedo) */
static void gen_lunar_image(float img[SC_IMG_H][SC_IMG_W][SC_IMG_C]) {
 for (int y=0;y<SC_IMG_H;y++)
 for (int x=0;x<SC_IMG_W;x++) {
 /* Base albedo with slight noise */
 float base = 0.35f + frand(-0.05f,0.05f);
 /* Two synthetic craters */
 float d1 = sqrtf((float)((y-40)*(y-40)+(x-60)*(x-60)));
 float d2 = sqrtf((float)((y-90)*(y-90)+(x-80)*(x-80)));
 float c1 = (d1 < 20.f) ? (0.1f + d1/40.f) : base;
 float c2 = (d2 < 12.f) ? (0.2f + d2/20.f) : base;
 float pix = fminf(c1, c2);
 img[y][x][0] = pix;
 img[y][x][1] = pix * 0.95f;
 img[y][x][2] = pix * 0.90f;
 }
}

/* IMU: near-zero specific force in free-fall, gyro noise ~1 mrad/s */
static void gen_imu(float gyro[3], float accel[3]) {
 gyro[0] = frand(-0.001f, 0.001f);
 gyro[1] = frand(-0.001f, 0.001f);
 gyro[2] = frand(-0.001f, 0.001f);
 /* In LEO free-fall, accelerometer reads drag + attitude-control
 forces only; gravity cancels out. Typical: <1e-3 m/s^2. */
 accel[0] = frand(-5e-4f, 5e-4f);
 accel[1] = frand(-5e-4f, 5e-4f);
 accel[2] = frand(-5e-4f, 5e-4f);
}

/* GPS position (LEO circular, 400 km altitude) */
static void gen_gps(float gps[3], float t_sec) {
 float r = 6778e3f;
 float omega = sqrtf(SC_MU_EARTH / (r * r * r));
 float theta = t_sec * omega;
 gps[0] = r * cosf(theta);
 gps[1] = r * sinf(theta);
 gps[2] = frand(-10.f, 10.f);
}

/* Star-tracker quaternion, nadir-pointing tracks orbital rate */
static void gen_star_quat(float q[4], float t_sec) {
 float r = 6778e3f;
 float omega = sqrtf(SC_MU_EARTH / (r * r * r));
 float angle = t_sec * omega;
 q[0] = cosf(angle*0.5f);
 q[1] = sinf(angle*0.5f)*0.001f;
 q[2] = sinf(angle*0.5f)*0.001f;
 q[3] = sinf(angle*0.5f);
 /* normalise */
 float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
 for (int i=0;i<4;i++) q[i]/=n;
}

/* Docking target image (LED marker ring) */
static void gen_docking_image(float img[SC_IMG_H][SC_IMG_W][SC_IMG_C]) {
 memset(img, 0, SC_IMG_H*SC_IMG_W*3*sizeof(float));
 /* Bright LED ring at target */
 for (int a=0; a<360; a+=15) {
 float rad = PI_F * a / 180.f;
 int y = (int)(64 + 20*sinf(rad));
 int x = (int)(64 + 20*cosf(rad));
 if (y>=0&&y<SC_IMG_H&&x>=0&&x<SC_IMG_W) {
 img[y][x][0]=1.f; img[y][x][1]=1.f; img[y][x][2]=0.f;
 }
 }
}

/* Hyperspectral signature (iron-oxide regolith) */
static void gen_hyperspectral(float spec[SC_HYPER_BANDS]) {
 for (int i=0;i<SC_HYPER_BANDS;i++) {
 float wl = 400.f + (float)i * (2500.f-400.f)/SC_HYPER_BANDS;
 /* Rough iron-oxide reflectance shape */
 spec[i] = 0.1f + 0.3f*(1.f-expf(-wl/800.f))
 - 0.15f*expf(-((wl-950.f)*(wl-950.f))/(2.f*50.f*50.f))
 + frand(-0.02f,0.02f);
 spec[i] = fmaxf(0.f, spec[i]);
 }
}

/* Nominal telemetry vector */
static void gen_telemetry_nominal(float tele[SC_FDIR_SENSOR_DIM]) {
 /* Power subsystem */
 tele[0]=28.2f; tele[1]=27.9f; tele[2]=0.85f; tele[3]=42.1f;
 /* Thermal */
 tele[4]=22.f; tele[5]=24.f; tele[6]=19.f; tele[7]=26.f;
 /* ADCS */
 tele[8]=0.001f; tele[9]=0.002f; tele[10]=-0.001f;
 tele[11]=9.81f; tele[12]=-0.05f;tele[13]=0.03f;
 /* Propulsion */
 tele[14]=2.1f; tele[15]=2.0f; tele[16]=310.f;tele[17]=0.95f;
 /* Comms */
 tele[18]=-85.f; tele[19]=48.f; tele[20]=0.02f;tele[21]=1200.f;
 /* Payload */
 tele[22]=12.f; tele[23]=0.3f; tele[24]=1.0f; tele[25]=25.f;
 /* OBC */
 tele[26]=45.f; tele[27]=0.32f; tele[28]=0.f; tele[29]=42.f;
 /* Spare channels */
 tele[30]=1.f; tele[31]=1.f;
}

/* Inject fault on a channel */
static void inject_fault(float tele[SC_FDIR_SENSOR_DIM], int channel, float spike) {
 tele[channel] += spike;
}

/* Science feature vector (spectral indices + context) */
static void gen_science_features(float feat[SC_SCI_FEATURE_DIM], int tick) {
 /* Spectral ratios, SNR, orbital params */
 for (int i=0;i<SC_SCI_FEATURE_DIM;i++)
 feat[i] = sinf((float)(i+tick)*0.3f)*0.5f + 0.5f + frand(-0.05f,0.05f);
}

/* --- print helpers ------------------------------------------------------ */
static void print_separator(void) {
 FLIGHT_LOG("-----------------------------------------------------------------------");
}

static void print_vec3(const char *label, const float v[3]) {
 FLIGHT_LOG(" %-20s %+12.4f %+12.4f %+12.4f\n", label, v[0],v[1],v[2]);
}

static void print_quat(const char *label, const float q[4]) {
 FLIGHT_LOG(" %-20s w=%+.4f x=%+.4f y=%+.4f z=%+.4f\n",
 label, q[0],q[1],q[2],q[3]);
}

/* -- main -- */
int main(void) {
 FLIGHT_LOG("\n");
 FLIGHT_LOG("== SPACECRAFT AI WORKLOADS ==========================================\n");
 FLIGHT_LOG(" 10 on-board modules, dt = 0.1 s, 10 ticks\n");
#ifdef __riscv
 FLIGHT_LOG(" Timer: RISC-V rdcycle @ %d MHz\n", SC_CPU_MHZ);
#else
 FLIGHT_LOG(" Timer: clock_gettime(CLOCK_MONOTONIC)\n");
#endif
 FLIGHT_LOG("=====================================================================\n\n");

 hrt_t t_total_start = hrt_now();

 /* context on BSS */
 static SpacecraftAI ctx;
 spacecraft_ai_init(&ctx);

 static float lunar_img[SC_IMG_H][SC_IMG_W][SC_IMG_C];
 static float dock_img[SC_IMG_H][SC_IMG_W][SC_IMG_C];

 /* seed EKF from first GPS fix */
 {
 float gps0[3];
 gen_gps(gps0, 0.f);
 for (int i = 0; i < 3; i++) ctx.ekf_x[i] = gps0[i];
 /* Approximate circular-orbit velocity at 400 km altitude */
 float v_orb = sqrtf(3.986004418e14f / 6778000.f); /* ~7669 m/s */
 ctx.ekf_x[3] = 0.f;
 ctx.ekf_x[4] = v_orb;
 ctx.ekf_x[5] = 0.f;
 }

 /* --- 10-tick run loop ------------------------------------------------ */
 for (int tick = 0; tick < 10; tick++) {
 hrt_t t_tick_start = hrt_now();
 FLIGHT_LOG("\n>> TICK %d (dt = 0.1 s)\n", tick);
 FLIGHT_LOG("-----------------------------------------------------------------------\n");

 float dt = 0.1f;
 float gyro[3], accel[3], gps[3];
 float star_q[4] = {1.f, 0.f, 0.f, 0.f}; /* init to identity */
 gen_imu(gyro, accel);
 gen_gps(gps, (float)tick * dt);
 gen_star_quat(star_q, (float)tick * dt);
 gen_lunar_image(lunar_img);
 gen_docking_image(dock_img);

 /* fusion first: drives the shared EKF predict/update */

 /* --- [3] fusion ------------------------------------------------- */
 FLIGHT_LOG("\n[3] MULTI-SENSOR FUSION EKF + ML BIAS\n");
 FusionResult fus = multisensor_fusion_ekf(&ctx,
 accel, gyro, gps, star_q, dt);
 print_vec3("Fused position (m)", fus.state);
 FLIGHT_LOG(" %-20s gps=%.3f att=%.3f\n", "Sensor weights",
 fus.sensor_weights[0], fus.sensor_weights[1]);
 FLIGHT_LOG(" %-20s gyro=[%+.5f %+.5f %+.5f]\n",
 "ML bias corr (gyro)",
 fus.ml_bias_correction[0],
 fus.ml_bias_correction[1],
 fus.ml_bias_correction[2]);
 FLIGHT_LOG(" %-20s %s\n", "Integrity",
 fus.integrity_flags > 0.5f ? "PASS" : "FAIL");

 /* --- [1] optical nav -------------------------------------------- */
 FLIGHT_LOG("\n[1] OPTICAL NAVIGATION CNN + EKF\n");
 const float (*cimg)[SC_IMG_W][SC_IMG_C] =
 (const float(*)[SC_IMG_W][SC_IMG_C])lunar_img;
 OptNavResult nav = optical_nav_cnn_ekf(&ctx, cimg, gyro, dt);
 print_vec3("Position (m)", nav.position);
 print_vec3("Velocity (m/s)", nav.velocity);
 print_quat("Attitude (q)", nav.attitude);
 FLIGHT_LOG(" %-20s %.4f\n", "Innovation norm", nav.innovation_norm);
 FLIGHT_LOG(" %-20s %s\n", "Converged", nav.converged ? "YES" : "NO");

 /* --- [2] hazard detection ---------------------------------------- */
 FLIGHT_LOG("\n[2] HAZARD DETECTION SEGMENTATION\n");
 HazardResult haz = hazard_detection_segment(
 (const float(*)[SC_IMG_W][SC_IMG_C])lunar_img);
 FLIGHT_LOG(" %-20s %.3f\n", "Hazard fraction", haz.hazard_fraction);
 FLIGHT_LOG(" %-20s %s\n", "Safe LZ found",
 haz.safe_landing_zone_found ? "YES" : "NO");
 if (haz.safe_landing_zone_found)
 FLIGHT_LOG(" %-20s (%.1f, %.1f) px\n", "Best LZ centre",
 haz.best_lz_center[0], haz.best_lz_center[1]);

 /* --- [4] docking ------------------------------------------------ */
 FLIGHT_LOG("\n[4] AUTONOMOUS DOCKING VISION\n");
 DockingResult dck = autonomous_docking_vision(
 (const float(*)[SC_IMG_W][SC_IMG_C])dock_img,
 accel);
 print_vec3("Rel. position (m)", dck.relative_pos);
 print_quat("Rel. attitude (q)", dck.relative_att);
 static const char *phase_str[] = {
 "FAR_APPROACH","PROXIMITY_OPS","FINAL_APPROACH","CAPTURE" };
 FLIGHT_LOG(" %-20s %s\n", "Docking phase", phase_str[dck.docking_phase]);
 FLIGHT_LOG(" %-20s %.3f\n","Corridor score", dck.approach_corridor_score);

 /* --- [5] swarm consensus ---------------------------------------- */
 if (tick == 0) {
 FLIGHT_LOG("\n[5] COOPERATIVE SWARM CONSENSUS\n");
 /* Simulate 3 neighbours */
 float nb[3][6];
 for (int k=0;k<3;k++)
 for (int i=0;i<6;i++) nb[k][i] = frand(-10.f,10.f);
 SwarmResult sw = swarm_consensus(&ctx, 0,
 (const float(*)[6])nb, 3);
 FLIGHT_LOG(" %-20s %d active, connected=%s\n",
 "Swarm topology", sw.n_active,
 sw.topology_connected?"YES":"NO");
 FLIGHT_LOG(" %-20s %.5f\n", "Convergence metric", sw.convergence_metric);
 FLIGHT_LOG(" %-20s [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
 "Consensus state",
 sw.consensus_state[0], sw.consensus_state[1],
 sw.consensus_state[2], sw.consensus_state[3],
 sw.consensus_state[4], sw.consensus_state[5]);
 }

 /* --- [6] hyperspectral ------------------------------------------ */
 if (tick == 0) {
 FLIGHT_LOG("\n[6] HYPERSPECTRAL CNN CLASSIFICATION\n");
 float spectrum[SC_HYPER_BANDS];
 gen_hyperspectral(spectrum);
 HyperResult hyp = hyperspectral_classify(spectrum);
 FLIGHT_LOG(" %-20s %s (class %d)\n",
 "Predicted material", hyp.label, hyp.predicted_class);
 FLIGHT_LOG(" %-20s %.2f%%\n", "Confidence", hyp.confidence*100.f);
 FLIGHT_LOG(" Top-3 classes: ");
 /* find top 3 */
 float probs[SC_HYPER_CLASSES];
 memcpy(probs, hyp.class_probs, sizeof(probs));
 for (int top=0;top<3;top++) {
 int bst=0;
 for (int i=1;i<SC_HYPER_CLASSES;i++)
 if (probs[i]>probs[bst]) bst=i;
 FLIGHT_LOG("[class %d: %.1f%%] ", bst, probs[bst]*100.f);
 probs[bst]=-1.f;
 }
 putchar('\n');
 }

 /* --- [7] FDIR --------------------------------------------------- */
 FLIGHT_LOG("\n[7] FDIR AUTOENCODER\n");
 float tele[SC_FDIR_SENSOR_DIM];
 gen_telemetry_nominal(tele);
 /* Inject fault on tick 5 */
 if (tick == 5) inject_fault(tele, 0, 15.f); /* voltage spike */
 FDIRResult fdir = fdir_autoencoder(&ctx, tele);
 FLIGHT_LOG(" %-20s %.5f\n", "Recon. error", fdir.reconstruction_error);
 FLIGHT_LOG(" %-20s %.3f\n", "Anomaly z-score", fdir.anomaly_score);
 if (fdir.fault_detected)
 FLIGHT_LOG(" %-20s %s\n",
 "FAULT DETECTED", fdir.fault_description);
 else
 FLIGHT_LOG(" %-20s OK (all subsystems nominal)\n", "FDIR status");

 /* --- [8] science priority ---------------------------------------- */
 if (tick % 3 == 0) {
 FLIGHT_LOG("\n[8] SCIENCE PRIORITISATION AUTOENCODER\n");
 float feat[SC_SCI_FEATURE_DIM];
 gen_science_features(feat, tick);
 ScienceResult sci = science_prioritize(feat);
 FLIGHT_LOG(" %-20s %.4f\n", "Priority score", sci.priority_score);
 FLIGHT_LOG(" %-20s %.4f\n", "Novelty", sci.novelty);
 FLIGHT_LOG(" %-20s %.4f\n", "Scientific value", sci.scientific_value);
 FLIGHT_LOG(" %-20s downlink=%s onboard_process=%s\n", "Decisions",
 sci.recommend_downlink ? "YES" : "NO",
 sci.recommend_onboard_process ? "YES" : "NO");
 }

 /* --- [9] compression --------------------------------------------- */
 if (tick == 3) {
 FLIGHT_LOG("\n[9] ADAPTIVE COMPRESSION\n");
 /* Simulate 4 KB of science data */
 static uint8_t raw_data[4096];
 for (int i=0;i<4096;i++)
 raw_data[i] = (uint8_t)((sinf(i*0.1f)+1.f)*127.f);
 CompressResult cmp = adaptive_compress(raw_data, 4096, 0.75f);
 FLIGHT_LOG(" %-20s %.2f:1\n", "Compression ratio", cmp.compression_ratio);
 FLIGHT_LOG(" %-20s %u bytes\n","Compressed size", cmp.compressed_bytes);
 FLIGHT_LOG(" %-20s %d/7\n", "Quality level", cmp.quality_level);
 FLIGHT_LOG(" %-20s %.1f dB\n","Est. PSNR", cmp.psnr_estimate);
 }

 /* --- [10] IMU bias ----------------------------------------------- */
 FLIGHT_LOG("\n[10] IMU BIAS LEARNING NETWORK (GRU)\n");
 IMUBiasResult bias = imu_bias_network(&ctx, gyro, accel);
 FLIGHT_LOG(" %-20s [%+.6f %+.6f %+.6f] rad/s\n",
 "Gyro bias est.",
 bias.bias_gyro[0], bias.bias_gyro[1], bias.bias_gyro[2]);
 FLIGHT_LOG(" %-20s [%+.6f %+.6f %+.6f] m/s²\n",
 "Accel bias est.",
 bias.bias_accel[0], bias.bias_accel[1], bias.bias_accel[2]);
 FLIGHT_LOG(" %-20s [%.4f %.4f %.4f %.4f %.4f %.4f]\n",
 "Scale factors",
 bias.scale_factor[0], bias.scale_factor[1], bias.scale_factor[2],
 bias.scale_factor[3], bias.scale_factor[4], bias.scale_factor[5]);

 hrt_t t_tick_end = hrt_now();
 FLIGHT_LOG("\n [HRT] tick %d: %.1f us\n", tick,
 hrt_elapsed_us(t_tick_start, t_tick_end));
 print_separator();
 }

 hrt_t t_total_end = hrt_now();

 FLIGHT_LOG("\n=====================================================================\n");
 FLIGHT_LOG(" Done. Context: %zu bytes\n", sizeof(SpacecraftAI));
 FLIGHT_LOG(" Total HRT: %.1f us (%.3f ms)\n",
 hrt_elapsed_us(t_total_start, t_total_end),
 hrt_elapsed_us(t_total_start, t_total_end) / 1e3);
 FLIGHT_LOG("=====================================================================\n\n");

 return 0;
}
