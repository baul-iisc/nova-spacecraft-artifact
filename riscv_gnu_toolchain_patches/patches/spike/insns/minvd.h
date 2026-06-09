// Matrix Inversion (Double Precision) - 3x3 inverse via cofactor/determinant method
// MINVD(md, ms1): md = inverse(ms1)
// Both operands are matrix registers
{
  int md  = insn.rd();
  int ms1 = insn.rs1();

  // Aliases for readability
  double a00 = STATE.matrix_regs[ms1][0][0];
  double a01 = STATE.matrix_regs[ms1][0][1];
  double a02 = STATE.matrix_regs[ms1][0][2];
  double a10 = STATE.matrix_regs[ms1][1][0];
  double a11 = STATE.matrix_regs[ms1][1][1];
  double a12 = STATE.matrix_regs[ms1][1][2];
  double a20 = STATE.matrix_regs[ms1][2][0];
  double a21 = STATE.matrix_regs[ms1][2][1];
  double a22 = STATE.matrix_regs[ms1][2][2];

  // Cofactors
  double c00 = a11 * a22 - a12 * a21;
  double c01 = a12 * a20 - a10 * a22;
  double c02 = a10 * a21 - a11 * a20;
  double c10 = a02 * a21 - a01 * a22;
  double c11 = a00 * a22 - a02 * a20;
  double c12 = a01 * a20 - a00 * a21;
  double c20 = a01 * a12 - a02 * a11;
  double c21 = a02 * a10 - a00 * a12;
  double c22 = a00 * a11 - a01 * a10;

  // Determinant (Laplace expansion along first row)
  double det = a00 * c00 + a01 * c01 + a02 * c02;

  // Inverse = adjugate / determinant
  // adjugate = transpose of cofactor matrix
  double inv_det = 1.0 / det;
  STATE.matrix_regs[md][0][0] = c00 * inv_det;
  STATE.matrix_regs[md][0][1] = c10 * inv_det;
  STATE.matrix_regs[md][0][2] = c20 * inv_det;
  STATE.matrix_regs[md][1][0] = c01 * inv_det;
  STATE.matrix_regs[md][1][1] = c11 * inv_det;
  STATE.matrix_regs[md][1][2] = c21 * inv_det;
  STATE.matrix_regs[md][2][0] = c02 * inv_det;
  STATE.matrix_regs[md][2][1] = c12 * inv_det;
  STATE.matrix_regs[md][2][2] = c22 * inv_det;
}
