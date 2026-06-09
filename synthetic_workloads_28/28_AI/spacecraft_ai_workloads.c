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
 * spacecraft_ai_workloads.c -- all 10 AI workloads
 */
#include "spacecraft_ai.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "../flight_compliance.h"

/* forward declarations of weights from spacecraft_ai_weights.c */
extern float optcnn_conv1_K[], optcnn_conv1_b[];
extern float optcnn_conv2_K[], optcnn_conv2_b[];
extern float optcnn_fc1_W[], optcnn_fc1_b[];
extern float optcnn_fc2_W[], optcnn_fc2_b[];

extern float haz_enc1_K[], haz_enc1_b[];
extern float haz_enc2_K[], haz_enc2_b[];
extern float haz_dec1_K[], haz_dec1_W[], haz_dec1_b[];
extern float haz_cls_W[], haz_cls_b[];

extern float bias_W1[], bias_b1[];
extern float bias_W2[], bias_b2[];
extern float bias_W3[], bias_b3[];

extern float dock_conv1_K[], dock_conv1_b[];
extern float dock_conv2_K[], dock_conv2_b[];
extern float dock_fc1_W[], dock_fc1_b[];
extern float dock_fc2_W[], dock_fc2_b[];

extern float swarm_att_W[], swarm_att_b[];
extern float swarm_agg_W[], swarm_agg_b[];
extern float swarm_out_W[], swarm_out_b[];

extern float hyper_conv1_K[], hyper_conv1_b[];
extern float hyper_conv2_K[], hyper_conv2_b[];
extern float hyper_fc1_W[], hyper_fc1_b[];
extern float hyper_fc2_W[], hyper_fc2_b[];

extern float fdir_enc_W1[], fdir_enc_b1[];
extern float fdir_enc_W2[], fdir_enc_b2[];
extern float fdir_dec_W1[], fdir_dec_b1[];
extern float fdir_dec_W2[], fdir_dec_b2[];
extern const float fdir_threshold[];

extern float sci_enc_W1[], sci_enc_b1[];
extern float sci_enc_W2[], sci_enc_b2[];
extern float sci_dec_W1[], sci_dec_b1[];
extern float sci_dec_W2[], sci_dec_b2[];
extern float sci_val_W[], sci_val_b[];

extern float cmp_W1[], cmp_b1[];
extern float cmp_W2[], cmp_b2[];
extern float cmp_W3[], cmp_b3[];

extern float gru_Wz[], gru_Uz[], gru_bz[];
extern float gru_Wr[], gru_Ur[], gru_br[];
extern float gru_Wh[], gru_Uh[], gru_bh[];
extern float gru_out_W[], gru_out_b[];

/* forward declarations of math helpers */
void dense_relu (const float*,const float*,const float*,float*,int,int);
void dense_sigmoid(const float*,const float*,const float*,float*,int,int);
void dense_linear (const float*,const float*,const float*,float*,int,int);
void conv2d_relu (const float*,int,int,int,const float*,int,int,int,const float*,float*);
void maxpool2x2 (const float*,int,int,int,float*);
void global_avg_pool(const float*,int,int,int,float*);
void gru_cell (const float*,const float*,const float*,
 const float*,const float*,const float*,
 const float*,const float*,const float*,
 const float*,float*,int,int);
void softmax (float*,int);
float sigmoid (float);
float relu (float);
float vec3_norm (const float[3]);
void vec3_normalize(float[3]);
void quat_mul (const float[4],const float[4],float[4]);
void quat_normalize(float[4]);
void quat_rotate (const float[4],const float[3],float[3]);
void rotvec_to_quat(const float[3],float[4]);
void mat_identity(float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM]);
void mat_add (const float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM],
 const float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM],
 float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM]);
void mat_mul (const float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM],
 const float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM],
 float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM]);
void mat_transpose(const float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM],
 float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM]);
void mat_sym_inv (float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM]);
void mat_vec_mul (const float[SC_EKF_STATE_DIM][SC_EKF_STATE_DIM],
 const float[SC_EKF_STATE_DIM],float[SC_EKF_STATE_DIM]);
void weights_init(void);

/* -- process noise (typical LEO) -- */
static const float Q_diag[SC_EKF_STATE_DIM] = {
 1e-4f,1e-4f,1e-4f, /* position σ² m² */
 1e-6f,1e-6f,1e-6f, /* velocity σ² (m/s)² */
 1e-7f,1e-7f,1e-7f,1e-7f, /* attitude quaternion */
 1e-8f,1e-8f,1e-8f, /* gyro bias σ² (rad/s)²*/
 1e-7f,1e-7f /* accel bias σ² (m/s²)²*/
};

#define N SC_EKF_STATE_DIM

/* --- init -------------------------------------------------------------- */
void spacecraft_ai_init(SpacecraftAI *ctx) {
 memset(ctx, 0, sizeof(*ctx));
 /* P0: large for pos/vel so first GPS fix dominates */
 for (int i = 0; i < 3; i++) ctx->ekf_P[i][i] = 1e10f; /* position (m^2) */
 for (int i = 3; i < 6; i++) ctx->ekf_P[i][i] = 1e6f; /* velocity (m/s)^2 */
 for (int i = 6; i < 10; i++) ctx->ekf_P[i][i] = 0.1f; /* attitude */
 for (int i = 10; i < 13; i++) ctx->ekf_P[i][i] = 1e-4f; /* gyro bias */
 for (int i = 13; i < 15; i++) ctx->ekf_P[i][i] = 1e-4f; /* accel bias */
 ctx->ekf_x[6] = 1.f; /* unit quaternion w=1 */
 memset(ctx->gru_h, 0, sizeof(ctx->gru_h));
 /* init swarm adjacency as fully connected stub */
 for (int i=0;i<SC_SWARM_MAX_NODES;i++)
 for (int j=0;j<SC_SWARM_MAX_NODES;j++)
 ctx->swarm_adj[i][j] = (i!=j) ? 1.f : 0.f;
 weights_init();
}

/* --- 1. optical nav CNN + EKF ------------------------------------------ */

/* CNN feature extractor -> [cx, cy, r1, r2, r3] */
static void opt_cnn_forward(
 const float img[SC_IMG_H][SC_IMG_W][SC_IMG_C],
 float feat[5])
{
 /* Treat input as flat buffer for conv layer */
 const float *in0 = &img[0][0][0];

 /* Layer 1: Conv 5x5, 3->8, valid -> 124x124x8 */
 static float c1[124*124*8];
 conv2d_relu(in0, SC_IMG_H, SC_IMG_W, SC_IMG_C,
 optcnn_conv1_K, 5, 5, 8, optcnn_conv1_b, c1);

 /* Pool -> 62x62x8 */
 static float p1[62*62*8];
 maxpool2x2(c1, 124, 124, 8, p1);

 /* Layer 2: Conv 3x3, 8->16, valid -> 60x60x16 */
 static float c2[60*60*16];
 conv2d_relu(p1, 62, 62, 8, optcnn_conv2_K, 3, 3, 16, optcnn_conv2_b, c2);

 /* Pool -> 30x30x16 */
 static float p2[30*30*16];
 maxpool2x2(c2, 60, 60, 16, p2);

 /* Global average pool -> 16 */
 float gap[16];
 global_avg_pool(p2, 30, 30, 16, gap);

 /* FC1: 16->32 relu */
 float fc1[32];
 dense_relu(optcnn_fc1_W, optcnn_fc1_b, gap, fc1, 16, 32);

 /* FC2: 32->5 linear (regression) */
 dense_linear(optcnn_fc2_W, optcnn_fc2_b, fc1, feat, 32, 5);
}

/* EKF predict step (IMU-driven kinematics) */
static void ekf_predict(float x[N], float P[N][N],
 const float gyro[3], float dt)
{
 float F[N][N];
 mat_identity(F);

 /* Position += velocity * dt */
 F[0][3]=dt; F[1][4]=dt; F[2][5]=dt;

 /* Attitude quaternion kinematics: q_dot = 0.5 * Omega(omega) * q */
 float bg[3] = { x[10], x[11], x[12] };
 float w[3] = { gyro[0]-bg[0], gyro[1]-bg[1], gyro[2]-bg[2] };
 float rv[3] = { w[0]*dt, w[1]*dt, w[2]*dt };
 Quat dq; rotvec_to_quat(rv, dq);
 Quat q_old = { x[6], x[7], x[8], x[9] };
 Quat q_new;
 quat_mul(q_old, dq, q_new);
 quat_normalize(q_new);
 for (int i=0;i<4;i++) x[6+i] = q_new[i];

 /* Position propagation */
 x[0] += x[3]*dt;
 x[1] += x[4]*dt;
 x[2] += x[5]*dt;

 /* Two-body gravitational acceleration (Keplerian) */
 float r_sq = x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
 if (r_sq > 1e6f) { /* guard: only when position is initialised */
 float r_mag = sqrtf(r_sq);
 float ar = -SC_MU_EARTH / (r_sq * r_mag); /* -mu/r^3 */
 x[3] += ar * x[0] * dt;
 x[4] += ar * x[1] * dt;
 x[5] += ar * x[2] * dt;
 }

 /* P = F P F' + Q */
 float FP[N][N], FPFt[N][N];
 mat_mul((const float(*)[N])F, (const float(*)[N])P, FP);
 float Ft[N][N]; mat_transpose((const float(*)[N])F, Ft);
 mat_mul((const float(*)[N])FP, (const float(*)[N])Ft, FPFt);
 for (int i=0;i<N;i++) FPFt[i][i] += Q_diag[i];
 memcpy(P, FPFt, sizeof(FPFt));
}

/* EKF update step (optical measurement) */
static void ekf_update_opt(float x[N], float P[N][N],
 const float z[5], /* measured: cx cy r1 r2 r3 */
 float *innov_norm)
{
 /* Observation H: pinhole camera, nadir-pointing.
 f_pix / altitude gives pixel/metre sensitivity.
 Innovation gate (50 px) prevents update with uncalibrated CNN. */
 float H[5][N];
 memset(H, 0, sizeof(H));
 float r_sq = x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
 float r_mag = sqrtf(r_sq + EPS);
 float alt = fmaxf(r_mag - SC_R_EARTH, 200.f); /* orbital alt (m) */
 float f_pix = 500.f; /* focal length (px) */
 float h_pos = f_pix / (alt + EPS); /* px per metre */
 float h_rad = -50.f / (alt * alt + EPS); /* crater radius sensitivity */
 H[0][0] = h_pos;
 H[1][1] = h_pos;
 H[2][2] = h_rad;
 H[3][2] = h_rad;
 H[4][2] = h_rad;

 /* h(x): centroid + 3 crater radii */
 float hx[5];
 hx[0] = 64.f + H[0][0] * (x[0] - r_mag); /* cf. nadir reference */
 hx[1] = 64.f + H[1][1] * x[1];
 hx[2] = 32.f + H[2][2] * (r_mag - SC_R_EARTH - 400e3f);
 hx[3] = hx[2] * 0.9f;
 hx[4] = hx[2] * 0.8f;

 /* Innovation y = z - h(x) */
 float innov[5];
 for (int i=0;i<5;i++) innov[i] = z[i] - hx[i];

 /* innovation norm */
 { float s2=0.f;
 for (int i=0;i<5;i++) s2+=innov[i]*innov[i];
 *innov_norm=sqrtf(s2);
 }

 /* gate: reject if innovation exceeds pixel-space threshold */
 if (*innov_norm > 50.f) return;

 /* S = H P H' + R (5x5 only, computed inline) */
 float R_diag[5] = {4.f,4.f,9.f,9.f,9.f}; /* pixel² */
 float HP[5][N], S[5][5], PHt[N][5];
 memset(HP,0,sizeof(HP));
 for (int i=0;i<5;i++)
 for (int k=0;k<N;k++) if (H[i][k]!=0.f)
 for (int j=0;j<N;j++)
 HP[i][j] += H[i][k]*P[k][j];

 for (int i=0;i<5;i++)
 for (int j=0;j<5;j++) {
 float s=0.f;
 for (int k=0;k<N;k++) s+=HP[i][k]*H[j][k];
 S[i][j] = s + (i==j ? R_diag[i] : 0.f);
 }

 /* Sinv via 5x5 Cholesky */
 float Sinv[5][5];
 memcpy(Sinv,S,sizeof(S));
 float L5[5][5]={0};
 for (int i=0;i<5;i++) {
 for (int j=0;j<=i;j++) {
 float sv=Sinv[i][j];
 for (int k=0;k<j;k++) sv-=L5[i][k]*L5[j][k];
 L5[i][j]=(i==j)?sqrtf(sv+EPS):sv/L5[j][j];
 }
 }
 float L5inv[5][5]={0};
 for (int i=0;i<5;i++) {
 L5inv[i][i]=1.f/L5[i][i];
 for (int j=i+1;j<5;j++) {
 float sv=0.f;
 for (int k=i;k<j;k++) sv+=L5[j][k]*L5inv[k][i];
 L5inv[j][i]=-sv/L5[j][j];
 }
 }
 for (int i=0;i<5;i++) for (int j=0;j<5;j++) {
 float sv=0.f;
 for (int k=0;k<5;k++) sv+=L5inv[k][i]*L5inv[k][j];
 Sinv[i][j]=sv;
 }

 /* K = P H' S^{-1} (Nx5) */
 for (int i=0;i<N;i++) for (int j=0;j<5;j++) {
 float sv=0.f;
 for (int k=0;k<N;k++) sv+=P[i][k]*H[j][k];
 PHt[i][j]=sv;
 }
 float K[N][5]={0};
 for (int i=0;i<N;i++) for (int j=0;j<5;j++) {
 float sv=0.f;
 for (int k=0;k<5;k++) sv+=PHt[i][k]*Sinv[k][j];
 K[i][j]=sv;
 }

 /* x = x + K·y */
 for (int i=0;i<N;i++) {
 float sv=0.f;
 for (int j=0;j<5;j++) sv+=K[i][j]*innov[j];
 x[i]+=sv;
 }
 quat_normalize((float*)&x[6]);

 /* P = (I - K H) P */
 float KH[N][N]={0};
 for (int i=0;i<N;i++) for (int j=0;j<N;j++)
 for (int k=0;k<5;k++) KH[i][j]+=K[i][k]*H[k][j];
 float IKH[N][N]; mat_identity(IKH);
 for (int i=0;i<N;i++) for (int j=0;j<N;j++) IKH[i][j]-=KH[i][j];
 float newP[N][N]={0};
 for (int i=0;i<N;i++) for (int k=0;k<N;k++) if(IKH[i][k]!=0.f)
 for (int j=0;j<N;j++) newP[i][j]+=IKH[i][k]*P[k][j];
 /* symmetrise */
 for (int i=0;i<N;i++) for (int j=0;j<N;j++)
 P[i][j]=0.5f*(newP[i][j]+newP[j][i]);

 /* (innovation norm already computed above, before gate check) */
}

OptNavResult optical_nav_cnn_ekf(SpacecraftAI *ctx,
 const float img[SC_IMG_H][SC_IMG_W][SC_IMG_C],
 const float imu_gyro[3], float dt)
{
 OptNavResult r = {0};

 /* Prediction handled by fusion; only measurement update here. */
 (void)imu_gyro; (void)dt;

 /* CNN feature extraction */
 float feat[5];
 opt_cnn_forward(img, feat);

 /* Clamp to plausible pixel range [0,127] */
 for (int i=0;i<2;i++) feat[i] = fmaxf(0.f, fminf(127.f, feat[i]));
 for (int i=2;i<5;i++) feat[i] = fmaxf(1.f, fminf(64.f, feat[i]));

 /* EKF update (gated inside ekf_update_opt) */
 ekf_update_opt(ctx->ekf_x, ctx->ekf_P, feat, &r.innovation_norm);

 /* 5. Fill result */
 for (int i=0;i<3;i++) r.position[i] = ctx->ekf_x[i];
 for (int i=0;i<3;i++) r.velocity[i] = ctx->ekf_x[3+i];
 for (int i=0;i<4;i++) r.attitude[i] = ctx->ekf_x[6+i];
 memcpy(r.P, ctx->ekf_P, sizeof(r.P));
 r.converged = (r.innovation_norm < 50.f);
 return r;
}

/* --- 2. hazard segmentation -------------------------------------------- */
HazardResult hazard_detection_segment(
 const float img[SC_IMG_H][SC_IMG_W][SC_IMG_C])
{
 HazardResult res;
 memset(&res, 0, sizeof(res));

 int patch = 8; /* process image in 8x8 blocks */
 int n_py = SC_IMG_H / patch;
 int n_px = SC_IMG_W / patch;

 float best_safe = -1e30f;
 int best_y=-1, best_x=-1;
 int n_hazard=0, n_total = n_py * n_px;

 for (int py = 0; py < n_py; py++)
 for (int px = 0; px < n_px; px++) {
 /* Extract patch -> 8x8x3 flat */
 float patch_in[8*8*3];
 for (int dy=0;dy<patch;dy++)
 for (int dx=0;dx<patch;dx++)
 for (int c=0;c<3;c++)
 patch_in[(dy*patch+dx)*3+c] =
 img[py*patch+dy][px*patch+dx][c];

 /* Encode: Conv3x3(3->16) over 8x8 -> 6x6x16 */
 float enc1[6*6*16];
 conv2d_relu(patch_in, patch,patch,3,
 haz_enc1_K,3,3,16,haz_enc1_b, enc1);

 /* Conv3x3(16->16) -> 4x4x16 */
 float enc2[4*4*16];
 conv2d_relu(enc1,6,6,16, haz_enc2_K,3,3,16,haz_enc2_b, enc2);

 /* GAP -> 16 */
 float gap[16];
 global_avg_pool(enc2, 4,4,16, gap);

 /* Classify: FC 16->8 relu -> 8->3 linear -> softmax */
 float h1[8];
 dense_relu(haz_dec1_W, haz_dec1_b, gap, h1, 16, 8);
 float logits[3];
 dense_linear(haz_cls_W, haz_cls_b, h1, logits, 8, 3);
 softmax(logits, 3);

 /* Assign label to each pixel in patch */
 uint8_t label = 0;
 float conf = logits[0];
 for (int c=1;c<3;c++) if (logits[c]>conf) { conf=logits[c]; label=(uint8_t)c; }

 for (int dy=0;dy<patch;dy++)
 for (int dx=0;dx<patch;dx++) {
 res.label_map[py*patch+dy][px*patch+dx] = label;
 res.confidence[py*patch+dy][px*patch+dx] = conf;
 }

 if (label==1) n_hazard++;
 if (label==0 && logits[0]>best_safe) {
 best_safe=logits[0];
 best_y=py; best_x=px;
 }
 }

 res.hazard_fraction = (float)n_hazard / (float)(n_total + 1);
 if (best_y>=0) {
 res.safe_landing_zone_found = 1;
 res.best_lz_center[0] = (float)(best_y*patch + patch/2);
 res.best_lz_center[1] = (float)(best_x*patch + patch/2);
 }
 return res;
}

/* --- 3. multi-sensor fusion EKF + ML bias ------------------------------ */
FusionResult multisensor_fusion_ekf(SpacecraftAI *ctx,
 const float imu_acc[3], const float imu_gyro[3],
 const float gps[3], const float star_q[4],
 float dt)
{
 FusionResult res = {0};

 /* -- IMU predict -- */
 ekf_predict(ctx->ekf_x, ctx->ekf_P, imu_gyro, dt);

 /* Velocity update from accelerometer (body->inertial via attitude) */
 float q[4] = { ctx->ekf_x[6], ctx->ekf_x[7],
 ctx->ekf_x[8], ctx->ekf_x[9] };
 float acc_inertial[3];
 quat_rotate(q, imu_acc, acc_inertial);
 /* Specific force in inertial frame (gravity handled in predict) */
 for (int i=0;i<3;i++) ctx->ekf_x[3+i] += acc_inertial[i]*dt;

 /* -- GPS update -- */
 float innov_pos[3];
 for (int i=0;i<3;i++) innov_pos[i] = gps[i] - ctx->ekf_x[i];

 /* Scalar Kalman update per axis (R = 25 m^2, i.e. 5 m 1-sigma) */
 for (int i=0;i<3;i++) {
 float ki = ctx->ekf_P[i][i] / (ctx->ekf_P[i][i] + 25.f);
 ctx->ekf_x[i] += ki * innov_pos[i];
 ctx->ekf_P[i][i] *= (1.f - ki);
 }

 /* -- star tracker update -- */
 float innov_q[4];
 for (int i=0;i<4;i++) innov_q[i] = star_q[i] - ctx->ekf_x[6+i];
 float R_att = 1e-6f; /* star tracker noise ~1 arcsec */
 for (int i=0;i<4;i++) {
 float ki = ctx->ekf_P[6+i][6+i]/(ctx->ekf_P[6+i][6+i]+R_att);
 ctx->ekf_x[6+i] += ki * innov_q[i];
 ctx->ekf_P[6+i][6+i] *= (1.f-ki);
 }
 quat_normalize((float*)&ctx->ekf_x[6]);

 /* --- ML bias correction --------------------------------------------- */
 /* Build 10-element sensor residual vector */
 float residual[10];
 for (int i=0;i<3;i++) residual[i] = innov_pos[i];
 for (int i=0;i<4;i++) residual[3+i] = innov_q[i];
 for (int i=0;i<3;i++) residual[7+i] = imu_acc[i] - acc_inertial[i];
 /* Run bias MLP */
 float h1[20], h2[20], bias_out[6];
 dense_relu (bias_W1, bias_b1, residual, h1, 10, 20);
 dense_relu (bias_W2, bias_b2, h1, h2, 20, 20);
 dense_linear(bias_W3, bias_b3, h2, bias_out,20, 6);

 /* Clamp to plausible bias range: gyro 10 mrad/s, accel 100 mm/s^2 */
 for (int i=0;i<3;i++) bias_out[i] = fmaxf(-0.01f, fminf(0.01f, bias_out[i]));
 for (int i=3;i<6;i++) bias_out[i] = fmaxf(-0.1f, fminf(0.1f, bias_out[i]));

 for (int i=0;i<6;i++) res.ml_bias_correction[i] = bias_out[i];

 /* Apply a fraction of learned correction to gyro/accel bias states */
 float alpha = 0.01f; /* slow adaptation */
 for (int i=0;i<3;i++) ctx->ekf_x[10+i] += alpha * bias_out[i];
 for (int i=0;i<2;i++) ctx->ekf_x[13+i] += alpha * bias_out[3+i];

 /* --- RAIM-like integrity check -------------------------------------- */
 float gps_innov_norm = sqrtf(innov_pos[0]*innov_pos[0]+
 innov_pos[1]*innov_pos[1]+
 innov_pos[2]*innov_pos[2]);
 float att_innov_norm = sqrtf(innov_q[0]*innov_q[0]+innov_q[1]*innov_q[1]+
 innov_q[2]*innov_q[2]+innov_q[3]*innov_q[3]);
 res.sensor_weights[0] = 1.f / (1.f + gps_innov_norm*0.1f);
 res.sensor_weights[1] = 1.f / (1.f + att_innov_norm*10.f);
 res.integrity_flags = (gps_innov_norm < 50.f && att_innov_norm < 0.1f) ? 1.f : 0.f;

 for (int i=0;i<N;i++) res.state[i] = ctx->ekf_x[i];
 memcpy(res.P, ctx->ekf_P, sizeof(res.P));
 return res;
}

/* --- 4. docking pose --------------------------------------------------- */
DockingResult autonomous_docking_vision(
 const float img[SC_IMG_H][SC_IMG_W][SC_IMG_C],
 const float imu_acc[3])
{
 DockingResult res = {0};

 /* --- CNN pose regression -------------------------------------------- */
 const float *in0 = &img[0][0][0];

 static float dc1[124*124*8];
 conv2d_relu(in0, SC_IMG_H, SC_IMG_W, SC_IMG_C,
 dock_conv1_K,5,5,8, dock_conv1_b, dc1);
 static float dp1[62*62*8];
 maxpool2x2(dc1,124,124,8,dp1);

 static float dc2[60*60*16];
 conv2d_relu(dp1,62,62,8,dock_conv2_K,3,3,16,dock_conv2_b,dc2);
 static float dp2[30*30*16];
 maxpool2x2(dc2,60,60,16,dp2);

 float gap[16];
 global_avg_pool(dp2,30,30,16,gap);

 float fc1[32];
 dense_relu (dock_fc1_W, dock_fc1_b, gap, fc1, 16, 32);
 float pose6[6];
 dense_linear(dock_fc2_W, dock_fc2_b, fc1, pose6, 32, 6);

 /* pose6: [dx, dy, dz, rx, ry, rz] in metres / radians */
 for (int i=0;i<3;i++) res.relative_pos[i] = pose6[i];

 /* Convert Euler to quaternion */
 float rx=pose6[3], ry=pose6[4], rz=pose6[5];
 float cr=cosf(rx*0.5f), sr=sinf(rx*0.5f);
 float cp=cosf(ry*0.5f), sp=sinf(ry*0.5f);
 float cy=cosf(rz*0.5f), sy=sinf(rz*0.5f);
 res.relative_att[0] = cr*cp*cy+sr*sp*sy;
 res.relative_att[1] = sr*cp*cy-cr*sp*sy;
 res.relative_att[2] = cr*sp*cy+sr*cp*sy;
 res.relative_att[3] = cr*cp*sy-sr*sp*cy;

 /* Relative velocity (differencing with acc integration) */
 for (int i=0;i<3;i++) res.relative_vel[i] = imu_acc[i] * 0.1f;

 /* Estimate range */
 float range = sqrtf(pose6[0]*pose6[0]+pose6[1]*pose6[1]+pose6[2]*pose6[2]);

 /* Determine docking phase */
 if (range > 200.f) res.docking_phase = 0; /* far approach */
 else if (range > 20.f) res.docking_phase = 1; /* proximity ops */
 else if (range > 1.f) res.docking_phase = 2; /* final approach */
 else res.docking_phase = 3; /* capture zone */

 /* Approach corridor score: 1 = perfectly centred */
 float lateral = sqrtf(pose6[0]*pose6[0]+pose6[1]*pose6[1]);
 res.approach_corridor_score = expf(-lateral/(0.05f*range+0.1f));

 /* Simple pose covariance (function of range) */
 float sig_p = 0.002f * range + 0.01f;
 float sig_a = 0.001f * range + 0.001f;
 for (int i=0;i<3;i++) res.pose_covariance[i][i] = sig_p*sig_p;
 for (int i=3;i<6;i++) res.pose_covariance[i][i] = sig_a*sig_a;

 return res;
}

/* --- 5. swarm consensus ------------------------------------------------ */
SwarmResult swarm_consensus(SpacecraftAI *ctx,
 uint8_t node_id,
 const float neighbor_states[][6],
 uint8_t n_neighbors)
{
 SwarmResult res = {0};
 if (n_neighbors > SC_SWARM_MAX_NODES-1) n_neighbors = SC_SWARM_MAX_NODES-1;

 float *own = ctx->swarm_x[node_id];

 /* --- attention scores ------------------------------------------------ */
 float att_scores[SC_SWARM_MAX_NODES];
 float att_sum = 0.f;
 for (int k = 0; k < n_neighbors; k++) {
 /* Attention key: MLP(own_state || neighbour_state) */
 float diff[6];
 for (int i=0;i<6;i++) diff[i] = neighbor_states[k][i] - own[i];
 float h1[16];
 dense_relu(swarm_att_W, swarm_att_b, diff, h1, 6, 16);
 float score = 0.f;
 for (int i=0;i<16;i++) score += h1[i];
 att_scores[k] = expf(score * 0.1f);
 att_sum += att_scores[k];
 }
 /* Normalise */
 for (int k=0;k<n_neighbors;k++) att_scores[k] /= (att_sum + EPS);

 /* -- weighted aggregation -- */
 float agg[16]; memset(agg,0,sizeof(agg));
 for (int k=0;k<n_neighbors;k++) {
 float h1[16];
 dense_relu(swarm_att_W, swarm_att_b,
 (const float*)neighbor_states[k], h1, 6, 16);
 for (int i=0;i<16;i++) agg[i] += att_scores[k] * h1[i];
 }

 /* -- consensus update -- */
 float h2[32], consensus_update[6];
 dense_relu (swarm_agg_W, swarm_agg_b, agg, h2, 16, 32);
 dense_linear(swarm_out_W, swarm_out_b, h2, consensus_update, 32, 6);

 /* Laplacian consensus: x = x + alpha*(consensus_update - own) */
 float alpha = 0.1f;
 for (int i=0;i<6;i++)
 own[i] += alpha * (consensus_update[i] - own[i]);

 /* Fill result */
 for (int i=0;i<6;i++) res.consensus_state[i] = own[i];
 res.n_active = n_neighbors + 1;
 res.topology_connected = (n_neighbors >= 1) ? 1 : 0;

 /* Convergence: average state deviation from consensus */
 float dev = 0.f;
 for (int i=0;i<6;i++) dev += (consensus_update[i]-own[i])*(consensus_update[i]-own[i]);
 res.convergence_metric = sqrtf(dev);

 for (int k=0;k<n_neighbors;k++) res.agent_weights[k]=att_scores[k];
 return res;
}

/* --- 6. hyperspectral 1-D CNN ------------------------------------------ */
static const char *hyper_labels[SC_HYPER_CLASSES] = {
 "basalt","anorthosite","olivine","pyroxene","ilmenite",
 "regolith","ice_water","ice_co2","sulfur","iron_oxide",
 "organics","unknown"
};

HyperResult hyperspectral_classify(const float spectrum[SC_HYPER_BANDS]) {
 HyperResult res = {0};

 /* Normalise spectrum to [0,1] */
 float spec_n[SC_HYPER_BANDS];
 float mn=spectrum[0], mx=spectrum[0];
 for (int i=1;i<SC_HYPER_BANDS;i++) {
 if (spectrum[i]<mn) mn=spectrum[i];
 if (spectrum[i]>mx) mx=spectrum[i];
 }
 float rng = mx - mn + EPS;
 for (int i=0;i<SC_HYPER_BANDS;i++) spec_n[i]=(spectrum[i]-mn)/rng;

 /* 1-D Conv layer 1: kernel=5, 1->32 channels -> 60 outputs */
 float c1[60*32];
 memset(c1,0,sizeof(c1));
 for (int o=0;o<60;o++)
 for (int oc=0;oc<32;oc++) {
 float acc = hyper_conv1_b[oc];
 for (int k=0;k<5;k++) {
 int idx = o+k;
 acc += spec_n[idx] * hyper_conv1_K[(k*1+0)*32+oc];
 }
 c1[o*32+oc] = relu(acc);
 }

 /* 1-D avg pool (factor 2) -> 30x32 */
 float p1[30*32];
 for (int o=0;o<30;o++) for (int oc=0;oc<32;oc++)
 p1[o*32+oc] = 0.5f*(c1[(2*o)*32+oc]+c1[(2*o+1)*32+oc]);

 /* 1-D Conv layer 2: kernel=3, 32->16 -> 28 outputs */
 float c2[28*16];
 memset(c2,0,sizeof(c2));
 for (int o=0;o<28;o++)
 for (int oc=0;oc<16;oc++) {
 float acc = hyper_conv2_b[oc];
 for (int k=0;k<3;k++)
 for (int ic=0;ic<32;ic++)
 acc += p1[(o+k)*32+ic] * hyper_conv2_K[(k*32+ic)*16+oc];
 c2[o*16+oc] = relu(acc);
 }

 /* Global average pool -> 16 */
 float gap[16] = {0};
 for (int o=0;o<28;o++) for (int oc=0;oc<16;oc++) gap[oc] += c2[o*16+oc]/28.f;

 /* FC: 16->32 relu */
 float fc1[32];
 dense_relu(hyper_fc1_W, hyper_fc1_b, gap, fc1, 16, 32);

 /* FC: 32->12 softmax */
 dense_linear(hyper_fc2_W, hyper_fc2_b, fc1, res.class_probs, 32, SC_HYPER_CLASSES);
 softmax(res.class_probs, SC_HYPER_CLASSES);

 /* Select argmax */
 res.predicted_class = 0;
 res.confidence = res.class_probs[0];
 for (int i=1;i<SC_HYPER_CLASSES;i++) {
 if (res.class_probs[i] > res.confidence) {
 res.confidence = res.class_probs[i];
 res.predicted_class = (uint8_t)i;
 }
 }
 snprintf(res.label, sizeof(res.label), "%s", hyper_labels[res.predicted_class]);
 return res;
}

/* --- 7. FDIR autoencoder ----------------------------------------------- */

static const char *fdir_components[] = {
 "Power","Power","Power","Power",
 "Thermal","Thermal","Thermal","Thermal",
 "ADCS","ADCS","ADCS","ADCS","ADCS","ADCS",
 "Propulsion","Propulsion","Propulsion","Propulsion",
 "Comms","Comms","Comms","Comms",
 "Payload","Payload","Payload","Payload",
 "OBC","OBC","OBC","OBC",
 "Spare","Spare"
};

static const char *fdir_recovery[] = {
 "NO_ACTION","RESET_COMPONENT","SWITCH_REDUNDANT",
 "SAFE_MODE","EMERGENCY_SHUTDOWN"
};

FDIRResult fdir_autoencoder(SpacecraftAI *ctx,
 const float telemetry[SC_FDIR_SENSOR_DIM])
{
 FDIRResult res; memset(&res,0,sizeof(res));

 /* Normalise with running mean/variance (Welford) */
 ctx->fdir_n++;
 float tele_n[SC_FDIR_SENSOR_DIM];
 for (int i=0;i<SC_FDIR_SENSOR_DIM;i++) {
 float delta = telemetry[i] - ctx->fdir_mu[i];
 ctx->fdir_mu[i] += delta / (float)ctx->fdir_n;
 float delta2 = telemetry[i] - ctx->fdir_mu[i];
 ctx->fdir_sigma[i] += delta * delta2;
 float var = (ctx->fdir_n>1) ? ctx->fdir_sigma[i]/(ctx->fdir_n-1) : 1.f;
 tele_n[i] = (telemetry[i]-ctx->fdir_mu[i])/(sqrtf(var)+EPS);
 }

 /* Encoder: 32->16 relu -> 8 linear */
 float enc1[16], latent[SC_FDIR_LATENT_DIM];
 dense_relu (fdir_enc_W1, fdir_enc_b1, tele_n, enc1, SC_FDIR_SENSOR_DIM, 16);
 dense_linear(fdir_enc_W2, fdir_enc_b2, enc1, latent, 16, SC_FDIR_LATENT_DIM);

 /* Decoder: 8->16 relu -> 32 linear */
 float dec1[16], recon[SC_FDIR_SENSOR_DIM];
 dense_relu (fdir_dec_W1, fdir_dec_b1, latent, dec1, SC_FDIR_LATENT_DIM, 16);
 dense_linear(fdir_dec_W2, fdir_dec_b2, dec1, recon, 16, SC_FDIR_SENSOR_DIM);

 /* Per-channel reconstruction error */
 float total_err = 0.f;
 float worst_err = 0.f;
 int worst_ch = 0;
 for (int i=0;i<SC_FDIR_SENSOR_DIM;i++) {
 float e = fabsf(recon[i]-tele_n[i]);
 total_err += e;
 if (e > worst_err) { worst_err=e; worst_ch=i; }
 }
 res.reconstruction_error = total_err / SC_FDIR_SENSOR_DIM;

 /* Z-score anomaly detection */
 float mu_err = res.reconstruction_error;
 float z_score = (worst_err - mu_err) / (mu_err + EPS);
 res.anomaly_score = z_score;

 if (worst_err > fdir_threshold[worst_ch]) {
 res.fault_detected = 1;
 res.fault_component = (uint8_t)worst_ch;
 snprintf(res.fault_description, sizeof(res.fault_description),
 "Anomaly ch=%02d (%s) err=%.3f thr=%.3f",
 worst_ch, fdir_components[worst_ch],
 worst_err, fdir_threshold[worst_ch]);

 /* Recovery action selection (severity-tiered) */
 if (z_score > 10.f) res.recovery_action = 4;
 else if (z_score > 5.f) res.recovery_action = 3;
 else if (z_score > 3.f) res.recovery_action = 2;
 else res.recovery_action = 1;

 snprintf(res.fault_description + strlen(res.fault_description),
 sizeof(res.fault_description) - strlen(res.fault_description),
 " -> %s", fdir_recovery[res.recovery_action]);
 }
 return res;
}

/* --- 8. science autoencoder -------------------------------------------- */

/* Rolling latent-code dictionary for novelty scoring */
#define SCI_DICT_SIZE 64
static float sci_dict[SCI_DICT_SIZE][SC_SCI_LATENT_DIM];
static int sci_dict_ptr = 0;
static int sci_dict_n = 0;

ScienceResult science_prioritize(const float features[SC_SCI_FEATURE_DIM]) {
 ScienceResult res = {0};

 /* Encoder */
 float enc1[16], latent[SC_SCI_LATENT_DIM];
 dense_relu (sci_enc_W1, sci_enc_b1, features, enc1, SC_SCI_FEATURE_DIM, 16);
 dense_linear(sci_enc_W2, sci_enc_b2, enc1, latent, 16, SC_SCI_LATENT_DIM);

 /* Decoder & reconstruction */
 float dec1[16], recon[SC_SCI_FEATURE_DIM];
 dense_relu (sci_dec_W1, sci_dec_b1, latent, dec1, SC_SCI_LATENT_DIM, 16);
 dense_linear(sci_dec_W2, sci_dec_b2, dec1, recon, 16, SC_SCI_FEATURE_DIM);

 float rec_err = 0.f;
 for (int i=0;i<SC_SCI_FEATURE_DIM;i++) {
 float d = features[i]-recon[i]; rec_err += d*d;
 }
 rec_err = sqrtf(rec_err / SC_SCI_FEATURE_DIM);

 /* Novelty: min distance to dictionary */
 float min_dist = 1e30f;
 for (int k=0;k<sci_dict_n;k++) {
 float dist = 0.f;
 for (int i=0;i<SC_SCI_LATENT_DIM;i++) {
 float d = latent[i]-sci_dict[k][i]; dist+=d*d;
 }
 dist = sqrtf(dist);
 if (dist < min_dist) min_dist = dist;
 }
 if (sci_dict_n==0) min_dist = 1.f;
 res.novelty = tanhf(min_dist * 0.5f);

 /* Update dictionary */
 memcpy(sci_dict[sci_dict_ptr], latent, SC_SCI_LATENT_DIM*sizeof(float));
 sci_dict_ptr = (sci_dict_ptr+1) % SCI_DICT_SIZE;
 if (sci_dict_n < SCI_DICT_SIZE) sci_dict_n++;

 /* Learned scientific value regressor */
 float val[1];
 dense_linear(sci_val_W, sci_val_b, latent, val, SC_SCI_LATENT_DIM, 1);
 res.scientific_value = sigmoid(val[0]);

 /* Priority = weighted combination */
 res.priority_score = 0.5f*res.novelty + 0.3f*res.scientific_value
 + 0.2f*tanhf(rec_err);

 res.recommend_downlink = (res.priority_score > 0.6f);
 res.recommend_onboard_process = (res.novelty > 0.7f);
 return res;
}

/* --- 9. adaptive compression ------------------------------------------- */

/* Compute 8 statistics for a byte block */
static void block_stats(const uint8_t *blk, int n, float stats[8]) {
 /* mean, variance, min, max, zero_frac, hi_frac, energy, entropy */
 float sum=0, sumsq=0;
 int mn=255, mx=0, nz=0, nhi=0;
 uint8_t hist[16]={0};
 for (int i=0;i<n;i++) {
 sum+=blk[i]; sumsq+=blk[i]*(float)blk[i];
 if (blk[i]<mn) mn=blk[i];
 if (blk[i]>mx) mx=blk[i];
 if (blk[i]==0) nz++;
 if (blk[i]>200) nhi++;
 hist[blk[i]>>4]++;
 }
 float m = sum/n;
 stats[0] = m/255.f;
 stats[1] = sqrtf(fmaxf(0.f,sumsq/n-m*m))/128.f;
 stats[2] = (float)mn/255.f;
 stats[3] = (float)mx/255.f;
 stats[4] = (float)nz/n;
 stats[5] = (float)nhi/n;
 stats[6] = sumsq/(n*255.f*255.f);
 /* Shannon entropy of 16-bin histogram */
 float ent=0.f;
 for (int i=0;i<16;i++) {
 float p=(float)hist[i]/n;
 if (p>0) ent -= p*log2f(p);
 }
 stats[7] = ent/4.f; /* normalise to [0,1] */
}

CompressResult adaptive_compress(const uint8_t *data, size_t n_bytes,
 float quality_target)
{
 CompressResult res = {0};
 if (n_bytes == 0) return res;

 size_t n_blocks = (n_bytes + SC_COMPRESS_BLOCK - 1) / SC_COMPRESS_BLOCK;
 float total_cmp = 0.f;
 uint32_t total_out = 0;

 for (size_t b = 0; b < n_blocks; b++) {
 size_t boff = b * SC_COMPRESS_BLOCK;
 int blen = (int)((boff+SC_COMPRESS_BLOCK <= n_bytes)
 ? SC_COMPRESS_BLOCK
 : n_bytes - boff);
 float stats[8];
 block_stats(data + boff, blen, stats);

 /* MLP predicts bits/sample */
 float h1[16], h2[8], bps[1];
 dense_relu (cmp_W1, cmp_b1, stats, h1, 8, 16);
 dense_relu (cmp_W2, cmp_b2, h1, h2, 16, 8);
 dense_linear(cmp_W3, cmp_b3, h2, bps, 8, 1);
 float pred_bps = fmaxf(0.5f, fminf(8.f, bps[0]));

 /* Quality target adjusts quantisation level */
 float target_bps = pred_bps * quality_target;
 uint8_t qlevel = (uint8_t)fmaxf(0, fminf(7, 8.f - target_bps));

 /* Simple simulation of output size */
 float ratio = (8.f - (float)qlevel) / 8.f;
 uint32_t out_b = (uint32_t)ceilf(blen * ratio);
 total_out += out_b;
 total_cmp += ratio;
 }

 res.compression_ratio = (float)n_bytes / (float)(total_out + 1);
 res.compressed_bytes = total_out;
 res.quality_level = (uint8_t)(quality_target * 7.f);
 /* PSNR estimate: assumes quantisation noise */
 float qstep = 1.f / (float)(1 << res.quality_level);
 res.psnr_estimate = 20.f * log10f(1.f / (qstep * 0.289f + EPS));
 return res;
}

/* --- 10. IMU bias GRU -------------------------------------------------- */
IMUBiasResult imu_bias_network(SpacecraftAI *ctx,
 const float raw_gyro[3], const float raw_accel[3])
{
 IMUBiasResult res = {0};

 /* Compose 6-D input vector */
 float x[6];
 for (int i=0;i<3;i++) x[i] = raw_gyro[i] / 2.f; /* approx range ±2 rad/s */
 for (int i=0;i<3;i++) x[3+i] = raw_accel[i] / 20.f; /* approx range ±20 m/s² */

 /* GRU step: hidden state lives in ctx->gru_h */
 gru_cell(gru_Wz, gru_Uz, gru_bz,
 gru_Wr, gru_Ur, gru_br,
 gru_Wh, gru_Uh, gru_bh,
 x, ctx->gru_h,
 6, SC_IMU_HIDDEN);

 /* Output projection: hidden -> 6 (gyro_bias x3 + accel_bias x3) */
 float out[6];
 dense_linear(gru_out_W, gru_out_b, ctx->gru_h, out, SC_IMU_HIDDEN, 6);

 /* Scale to physical units */
 for (int i=0;i<3;i++) {
 res.bias_gyro[i] = out[i] * 0.005f; /* up to ±5 mrad/s */
 res.bias_accel[i] = out[3+i] * 0.05f; /* up to ±50 mm/s² */
 }

 /* Scale factors: 1 + small correction */
 for (int i=0;i<6;i++) res.scale_factor[i] = 1.f + out[i]*0.001f;

 /* Uncertainty from GRU output magnitude */
 for (int i=0;i<3;i++) {
 res.uncertainty[i] = fabsf(out[i]) * 0.002f + 1e-5f;
 res.uncertainty[3+i] = fabsf(out[3+i]) * 0.01f + 1e-4f;
 }

 return res;
}
