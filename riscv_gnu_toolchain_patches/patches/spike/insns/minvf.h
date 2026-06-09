// Matrix Inversion (Single Precision Float) - 3x3 inverse via cofactor/determinant method
// MINVF(md, ms1): md = inverse(ms1)
// Both operands are matrix registers
{
  int md  = insn.rd();
  int ms1 = insn.rs1();

  // Read source matrix as float precision
  float a00 = (float)STATE.matrix_regs[ms1][0][0];
  float a01 = (float)STATE.matrix_regs[ms1][0][1];
  float a02 = (float)STATE.matrix_regs[ms1][0][2];
  float a10 = (float)STATE.matrix_regs[ms1][1][0];
  float a11 = (float)STATE.matrix_regs[ms1][1][1];
  float a12 = (float)STATE.matrix_regs[ms1][1][2];
  float a20 = (float)STATE.matrix_regs[ms1][2][0];
  float a21 = (float)STATE.matrix_regs[ms1][2][1];
  float a22 = (float)STATE.matrix_regs[ms1][2][2];

  // Cofactors
  float c00 = a11 * a22 - a12 * a21;
  float c01 = a12 * a20 - a10 * a22;
  float c02 = a10 * a21 - a11 * a20;
  float c10 = a02 * a21 - a01 * a22;
  float c11 = a00 * a22 - a02 * a20;
  float c12 = a01 * a20 - a00 * a21;
  float c20 = a01 * a12 - a02 * a11;
  float c21 = a02 * a10 - a00 * a12;
  float c22 = a00 * a11 - a01 * a10;

  // Determinant (Laplace expansion along first row)
  float det = a00 * c00 + a01 * c01 + a02 * c02;

  // Inverse = adjugate / determinant
  float inv_det = 1.0f / det;
  STATE.matrix_regs[md][0][0] = (double)(c00 * inv_det);
  STATE.matrix_regs[md][0][1] = (double)(c10 * inv_det);
  STATE.matrix_regs[md][0][2] = (double)(c20 * inv_det);
  STATE.matrix_regs[md][1][0] = (double)(c01 * inv_det);
  STATE.matrix_regs[md][1][1] = (double)(c11 * inv_det);
  STATE.matrix_regs[md][1][2] = (double)(c21 * inv_det);
  STATE.matrix_regs[md][2][0] = (double)(c02 * inv_det);
  STATE.matrix_regs[md][2][1] = (double)(c12 * inv_det);
  STATE.matrix_regs[md][2][2] = (double)(c22 * inv_det);
}
