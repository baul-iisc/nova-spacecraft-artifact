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
 * spacecraft_ai_weights.c -- static weight tables
 *
 * Xavier-initialised stubs that exercise every code path.
 * Replace with trained values for flight.
 *
 * Layout: dense W[out][in] row-major, conv K[kh*kw*in_c*out_c].
 */
#include "spacecraft_ai.h"
#include <math.h>
#include "../flight_compliance.h"

/* --- deterministic PRNG (Lehmer LCG) ----------------------------------- */
static float init_w(uint32_t *s, int fan_in) {
 *s = *s * 1664525u + 1013904223u;
 float u = (float)(*s >> 8) / (float)(1u << 24); /* [0,1) */
 float xavier = sqrtf(6.f / (float)(fan_in + fan_in));
 return (u * 2.f - 1.f) * xavier;
}

/* Counter-based seeding (reproducible across builds, unlike address-based). */
static uint32_t wfill_ctr_;

#define WFILL(arr, fan) do { \
 uint32_t _s = wfill_ctr_++ * 2654435761u + 0xdeadbeef; \
 for (size_t _i = 0; _i < sizeof(arr)/sizeof(float); _i++) \
 (arr)[_i] = init_w(&_s, (fan)); \
} while(0)

/* --- 1. optical nav CNN ------------------------------------------------ */
float optcnn_conv1_K[5*5*3*8]; /* 5x5 kernel, 3 in, 8 out */
float optcnn_conv1_b[8];
float optcnn_conv2_K[3*3*8*16];
float optcnn_conv2_b[16];
float optcnn_fc1_W[32*16]; /* 32 outputs x 16 gap features */
float optcnn_fc1_b[32];
float optcnn_fc2_W[5*32]; /* regression: 2 pixel + 3 crater radii */
float optcnn_fc2_b[5];

/* --- 2. hazard segmentation -------------------------------------------- */
float haz_enc1_K[3*3*3*16];
float haz_enc1_b[16];
float haz_enc2_K[3*3*16*16];
float haz_enc2_b[16];
float haz_dec1_K[3*3*16*8]; /* kept for future transposed-conv path */
float haz_dec1_W[8*16]; /* dense 16->8 weight */
float haz_dec1_b[8];
float haz_cls_W[3*8]; /* 1x1 conv -> 3 class logits */
float haz_cls_b[3];

/* --- 3. ML bias correction MLP ----------------------------------------- */
float bias_W1[20*10], bias_b1[20];
float bias_W2[20*20], bias_b2[20];
float bias_W3[ 6*20], bias_b3[ 6];

/* --- 4. docking pose CNN ----------------------------------------------- */
float dock_conv1_K[5*5*3*8];
float dock_conv1_b[8];
float dock_conv2_K[3*3*8*16];
float dock_conv2_b[16];
float dock_fc1_W[32*16];
float dock_fc1_b[32];
float dock_fc2_W[6*32];
float dock_fc2_b[6];

/* --- 5. swarm graph-attention ------------------------------------------ */
float swarm_att_W[16*6]; float swarm_att_b[16]; /* attention key/query */
float swarm_agg_W[32*16]; float swarm_agg_b[32];
float swarm_out_W[ 6*32]; float swarm_out_b[ 6];

/* --- 6. hyperspectral 1-D CNN ------------------------------------------ */
float hyper_conv1_K[5*1*32]; /* 1D conv: kernel 5, in_c=1(band), out_c=32*/
float hyper_conv1_b[32];
float hyper_conv2_K[3*32*16];
float hyper_conv2_b[16];
float hyper_fc1_W[32*16];
float hyper_fc1_b[32];
float hyper_fc2_W[SC_HYPER_CLASSES*32];
float hyper_fc2_b[SC_HYPER_CLASSES];

/* --- 7. FDIR autoencoder ----------------------------------------------- */
float fdir_enc_W1[16*SC_FDIR_SENSOR_DIM], fdir_enc_b1[16];
float fdir_enc_W2[SC_FDIR_LATENT_DIM*16], fdir_enc_b2[SC_FDIR_LATENT_DIM];
float fdir_dec_W1[16*SC_FDIR_LATENT_DIM], fdir_dec_b1[16];
float fdir_dec_W2[SC_FDIR_SENSOR_DIM*16], fdir_dec_b2[SC_FDIR_SENSOR_DIM];

/* per-component anomaly threshold (tuned offline per subsystem) */
const float fdir_threshold[SC_FDIR_SENSOR_DIM] = {
 /* Power */ 0.12f,0.12f,0.12f,0.12f,
 /* Thermal*/ 0.10f,0.10f,0.10f,0.10f,
 /* ADCS */ 0.08f,0.08f,0.08f,0.08f,0.08f,0.08f,
 /* Propulsion*/0.15f,0.15f,0.15f,0.15f,
 /* Comms */ 0.11f,0.11f,0.11f,0.11f,
 /* Payload*/ 0.13f,0.13f,0.13f,0.13f,
 /* OBC */ 0.09f,0.09f,0.09f,0.09f,
 /* Spare */ 0.10f,0.10f
};

/* --- 8. science autoencoder -------------------------------------------- */
float sci_enc_W1[16*SC_SCI_FEATURE_DIM], sci_enc_b1[16];
float sci_enc_W2[SC_SCI_LATENT_DIM*16], sci_enc_b2[SC_SCI_LATENT_DIM];
float sci_dec_W1[16*SC_SCI_LATENT_DIM], sci_dec_b1[16];
float sci_dec_W2[SC_SCI_FEATURE_DIM*16], sci_dec_b2[SC_SCI_FEATURE_DIM];

/* Learned value regressor on top of latent code */
float sci_val_W[1*SC_SCI_LATENT_DIM], sci_val_b[1];

/* --- 9. compression MLP ------------------------------------------------ */
float cmp_W1[16*8], cmp_b1[16];
float cmp_W2[ 8*16], cmp_b2[8];
float cmp_W3[ 1*8], cmp_b3[1];

/* --- 10. IMU bias GRU -------------------------------------------------- */
float gru_Wz[SC_IMU_HIDDEN*6], gru_Uz[SC_IMU_HIDDEN*SC_IMU_HIDDEN], gru_bz[SC_IMU_HIDDEN];
float gru_Wr[SC_IMU_HIDDEN*6], gru_Ur[SC_IMU_HIDDEN*SC_IMU_HIDDEN], gru_br[SC_IMU_HIDDEN];
float gru_Wh[SC_IMU_HIDDEN*6], gru_Uh[SC_IMU_HIDDEN*SC_IMU_HIDDEN], gru_bh[SC_IMU_HIDDEN];
float gru_out_W[6*SC_IMU_HIDDEN], gru_out_b[6];

/* --- fill all tables --------------------------------------------------- */
void weights_init(void) {
 wfill_ctr_ = 0; /* reset so results are identical on every call */
 WFILL(optcnn_conv1_K, 5*5*3); WFILL(optcnn_conv1_b, 8);
 WFILL(optcnn_conv2_K, 3*3*8); WFILL(optcnn_conv2_b,16);
 WFILL(optcnn_fc1_W, 16); WFILL(optcnn_fc1_b, 32);
 WFILL(optcnn_fc2_W, 32); WFILL(optcnn_fc2_b, 5);

 WFILL(haz_enc1_K,3*3*3); WFILL(haz_enc1_b,16);
 WFILL(haz_enc2_K,3*3*16); WFILL(haz_enc2_b,16);
 WFILL(haz_dec1_K,3*3*16); WFILL(haz_dec1_W,16); WFILL(haz_dec1_b, 8);
 WFILL(haz_cls_W, 8); WFILL(haz_cls_b, 3);

 WFILL(bias_W1,10); WFILL(bias_b1,20);
 WFILL(bias_W2,20); WFILL(bias_b2,20);
 WFILL(bias_W3,20); WFILL(bias_b3, 6);

 WFILL(dock_conv1_K,5*5*3); WFILL(dock_conv1_b, 8);
 WFILL(dock_conv2_K,3*3*8); WFILL(dock_conv2_b,16);
 WFILL(dock_fc1_W, 16); WFILL(dock_fc1_b, 32);
 WFILL(dock_fc2_W, 32); WFILL(dock_fc2_b, 6);

 WFILL(swarm_att_W,6); WFILL(swarm_att_b,16);
 WFILL(swarm_agg_W,16); WFILL(swarm_agg_b,32);
 WFILL(swarm_out_W,32); WFILL(swarm_out_b, 6);

 WFILL(hyper_conv1_K,5); WFILL(hyper_conv1_b,32);
 WFILL(hyper_conv2_K,3*32); WFILL(hyper_conv2_b,16);
 WFILL(hyper_fc1_W,16); WFILL(hyper_fc1_b,32);
 WFILL(hyper_fc2_W,32); WFILL(hyper_fc2_b,SC_HYPER_CLASSES);

 WFILL(fdir_enc_W1,SC_FDIR_SENSOR_DIM); WFILL(fdir_enc_b1,16);
 WFILL(fdir_enc_W2,16); WFILL(fdir_enc_b2,SC_FDIR_LATENT_DIM);
 WFILL(fdir_dec_W1,SC_FDIR_LATENT_DIM); WFILL(fdir_dec_b1,16);
 WFILL(fdir_dec_W2,16); WFILL(fdir_dec_b2,SC_FDIR_SENSOR_DIM);

 WFILL(sci_enc_W1,SC_SCI_FEATURE_DIM); WFILL(sci_enc_b1,16);
 WFILL(sci_enc_W2,16); WFILL(sci_enc_b2,SC_SCI_LATENT_DIM);
 WFILL(sci_dec_W1,SC_SCI_LATENT_DIM); WFILL(sci_dec_b1,16);
 WFILL(sci_dec_W2,16); WFILL(sci_dec_b2,SC_SCI_FEATURE_DIM);
 WFILL(sci_val_W, SC_SCI_LATENT_DIM); WFILL(sci_val_b, 1);

 WFILL(cmp_W1,8); WFILL(cmp_b1,16);
 WFILL(cmp_W2,16); WFILL(cmp_b2,8);
 WFILL(cmp_W3,8); WFILL(cmp_b3,1);

 WFILL(gru_Wz,6); WFILL(gru_Uz,SC_IMU_HIDDEN); WFILL(gru_bz,SC_IMU_HIDDEN);
 WFILL(gru_Wr,6); WFILL(gru_Ur,SC_IMU_HIDDEN); WFILL(gru_br,SC_IMU_HIDDEN);
 WFILL(gru_Wh,6); WFILL(gru_Uh,SC_IMU_HIDDEN); WFILL(gru_bh,SC_IMU_HIDDEN);
 WFILL(gru_out_W,SC_IMU_HIDDEN); WFILL(gru_out_b,6);
}
