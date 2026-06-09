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
 * spacecraft_ai_math.c -- shared numeric utilities
 * Activations, dense layers, 2D convolutions, GRU cell,
 * quaternion and NxN matrix operations.
 */
#include "spacecraft_ai.h"
#include "../flight_compliance.h"

/* --- activations ------------------------------------------------------- */

float relu(float x) { return x > 0.f ? x : 0.f; }
float sigmoid(float x) { return 1.f / (1.f + expf(-x)); }

void softmax(float *v, int n) {
 float mx = v[0];
 for (int i = 1; i < n; i++) if (v[i] > mx) mx = v[i];
 float s = 0.f;
 for (int i = 0; i < n; i++) { v[i] = expf(v[i] - mx); s += v[i]; }
 for (int i = 0; i < n; i++) v[i] /= (s + EPS);
}

/* --- vector utilities -------------------------------------------------- */

float vec3_dot(const float a[3], const float b[3]) {
 return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void vec3_cross(const float a[3], const float b[3], float out[3]) {
 out[0] = a[1]*b[2] - a[2]*b[1];
 out[1] = a[2]*b[0] - a[0]*b[2];
 out[2] = a[0]*b[1] - a[1]*b[0];
}

float vec3_norm(const float v[3]) {
 return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void vec3_normalize(float v[3]) {
 float n = vec3_norm(v) + EPS;
 v[0] /= n; v[1] /= n; v[2] /= n;
}

/* General n-vector dot */
float vecN_dot(const float *a, const float *b, int n) {
 float s = 0.f;
 for (int i = 0; i < n; i++) s += a[i]*b[i];
 return s;
}

/* --- dense layers ------------------------------------------------------ */
void dense_relu(const float *W, const float *b,
 const float *x, float *out,
 int in_dim, int out_dim) {
 for (int i = 0; i < out_dim; i++) {
 float acc = b[i];
 for (int j = 0; j < in_dim; j++)
 acc += W[i * in_dim + j] * x[j];
 out[i] = relu(acc);
 }
}

void dense_sigmoid(const float *W, const float *b,
 const float *x, float *out,
 int in_dim, int out_dim) {
 for (int i = 0; i < out_dim; i++) {
 float acc = b[i];
 for (int j = 0; j < in_dim; j++)
 acc += W[i * in_dim + j] * x[j];
 out[i] = sigmoid(acc);
 }
}

void dense_linear(const float *W, const float *b,
 const float *x, float *out,
 int in_dim, int out_dim) {
 for (int i = 0; i < out_dim; i++) {
 float acc = b[i];
 for (int j = 0; j < in_dim; j++)
 acc += W[i * in_dim + j] * x[j];
 out[i] = acc;
 }
}

/* --- 2D conv (VALID, stride 1, ReLU) ----------------------------------- */
void conv2d_relu(const float *in, int in_h, int in_w, int in_c,
 const float *K, int kh, int kw, int out_c,
 const float *b,
 float *out)
{
 int oh = in_h - kh + 1;
 int ow = in_w - kw + 1;
 for (int y = 0; y < oh; y++)
 for (int x = 0; x < ow; x++)
 for (int oc = 0; oc < out_c; oc++) {
 float acc = b[oc];
 for (int ky = 0; ky < kh; ky++)
 for (int kx = 0; kx < kw; kx++)
 for (int ic = 0; ic < in_c; ic++) {
 int in_idx = ((y+ky)*in_w + (x+kx))*in_c + ic;
 int k_idx = ((ky*kw + kx)*in_c + ic)*out_c + oc;
 acc += in[in_idx] * K[k_idx];
 }
 out[(y*ow + x)*out_c + oc] = relu(acc);
 }
}

/* 2x2 max-pool */
void maxpool2x2(const float *in, int h, int w, int c, float *out) {
 int oh = h/2, ow = w/2;
 for (int y = 0; y < oh; y++)
 for (int x = 0; x < ow; x++)
 for (int k = 0; k < c; k++) {
 float mx = -1e30f;
 for (int dy = 0; dy < 2; dy++)
 for (int dx = 0; dx < 2; dx++) {
 float v = in[((2*y+dy)*w + (2*x+dx))*c + k];
 if (v > mx) mx = v;
 }
 out[(y*ow + x)*c + k] = mx;
 }
}

/* Global average pool */
void global_avg_pool(const float *in, int h, int w, int c, float *out) {
 memset(out, 0, c * sizeof(float));
 float inv = 1.f / (float)(h * w);
 for (int y = 0; y < h; y++)
 for (int x = 0; x < w; x++)
 for (int k = 0; k < c; k++)
 out[k] += in[(y*w + x)*c + k] * inv;
}

/* --- GRU cell ---------------------------------------------------------- */
void gru_cell(const float *Wz, const float *Uz, const float *bz, /* update */
 const float *Wr, const float *Ur, const float *br, /* reset */
 const float *Wh, const float *Uh, const float *bh, /* hidden */
 const float *x, float *h,
 int in_dim, int hidden_dim)
{
 float z[SC_IMU_HIDDEN], r[SC_IMU_HIDDEN], h_tilde[SC_IMU_HIDDEN];

 for (int i = 0; i < hidden_dim; i++) {
 float zv = bz[i], rv = br[i];
 for (int j = 0; j < in_dim; j++) { zv += Wz[i*in_dim+j]*x[j];
 rv += Wr[i*in_dim+j]*x[j]; }
 for (int j = 0; j < hidden_dim;j++) { zv += Uz[i*hidden_dim+j]*h[j];
 rv += Ur[i*hidden_dim+j]*h[j]; }
 z[i] = sigmoid(zv);
 r[i] = sigmoid(rv);
 }
 for (int i = 0; i < hidden_dim; i++) {
 float hv = bh[i];
 for (int j = 0; j < in_dim; j++) hv += Wh[i*in_dim+j]*x[j];
 for (int j = 0; j < hidden_dim;j++) hv += Uh[i*hidden_dim+j]*(r[j]*h[j]);
 h_tilde[i] = tanhf(hv);
 }
 for (int i = 0; i < hidden_dim; i++)
 h[i] = (1.f - z[i])*h[i] + z[i]*h_tilde[i];
}

/* --- quaternion math --------------------------------------------------- */

/* q_out = q1 * q2 (Hamilton product) */
void quat_mul(const Quat q1, const Quat q2, Quat out) {
 out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
 out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
 out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
 out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void quat_normalize(Quat q) {
 float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]) + EPS;
 for (int i=0;i<4;i++) q[i]/=n;
}

/* Rotate v by quaternion q */
void quat_rotate(const Quat q, const float v[3], float out[3]) {
 float qv[3] = {q[1], q[2], q[3]};
 float uv[3], uuv[3];
 vec3_cross(qv, v, uv);
 vec3_cross(qv, uv, uuv);
 for (int i=0;i<3;i++)
 out[i] = v[i] + 2.f*(q[0]*uv[i] + uuv[i]);
}

/* Quaternion from rotation vector (small-angle) */
void rotvec_to_quat(const float rv[3], Quat q) {
 float angle = vec3_norm(rv);
 if (angle < EPS) { q[0]=1;q[1]=q[2]=q[3]=0; return; }
 float s = sinf(angle*0.5f) / angle;
 q[0] = cosf(angle*0.5f);
 q[1] = rv[0]*s; q[2] = rv[1]*s; q[3] = rv[2]*s;
}

/* --- NxN matrix operations (N = SC_EKF_STATE_DIM) ---------------------- */
#define N SC_EKF_STATE_DIM

void mat_identity(float A[N][N]) {
 for (int i=0;i<N;i++) for (int j=0;j<N;j++) A[i][j] = (i==j)?1.f:0.f;
}

void mat_add(const float A[N][N], const float B[N][N], float C[N][N]) {
 for (int i=0;i<N;i++) for (int j=0;j<N;j++) C[i][j]=A[i][j]+B[i][j];
}

void mat_mul(const float A[N][N], const float B[N][N], float C[N][N]) {
 float tmp[N][N] = {0};
 for (int i=0;i<N;i++)
 for (int k=0;k<N;k++) if (A[i][k]!=0.f)
 for (int j=0;j<N;j++)
 tmp[i][j] += A[i][k]*B[k][j];
 memcpy(C, tmp, sizeof(tmp));
}

void mat_transpose(const float A[N][N], float At[N][N]) {
 for (int i=0;i<N;i++) for (int j=0;j<N;j++) At[i][j]=A[j][i];
}

/* Cholesky-based SPD inverse (in-place) */
void mat_sym_inv(float A[N][N]) {
 /* L * L' = A */
 float L[N][N] = {0};
 for (int i=0;i<N;i++) {
 for (int j=0;j<=i;j++) {
 float s = A[i][j];
 for (int k=0;k<j;k++) s -= L[i][k]*L[j][k];
 L[i][j] = (i==j) ? sqrtf(s + EPS) : s/L[j][j];
 }
 }
 /* forward-substitution for L^{-1} */
 float Linv[N][N] = {0};
 for (int i=0;i<N;i++) {
 Linv[i][i] = 1.f/L[i][i];
 for (int j=i+1;j<N;j++) {
 float s = 0.f;
 for (int k=i;k<j;k++) s += L[j][k]*Linv[k][i];
 Linv[j][i] = -s/L[j][j];
 }
 }
 /* A^{-1} = L^{-T} L^{-1} */
 for (int i=0;i<N;i++) for (int j=0;j<N;j++) {
 float s=0.f;
 for (int k=0;k<N;k++) s+=Linv[k][i]*Linv[k][j];
 A[i][j]=s;
 }
}

/* Mat-vec multiply y = A·x */
void mat_vec_mul(const float A[N][N], const float x[N], float y[N]) {
 for (int i=0;i<N;i++) {
 float s=0.f;
 for (int j=0;j<N;j++) s+=A[i][j]*x[j];
 y[i]=s;
 }
}

#undef N
