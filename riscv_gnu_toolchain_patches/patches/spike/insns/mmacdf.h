// Matrix Multiply-Accumulate (Double Precision Float)
// MMACDF(md, ms1, ms2): md += ms1 * ms2 (3x3 output-stationary MAC, double precision)
// All three operands are matrix registers (encoded in rd, rs1, rs2 fields)
{
  int md  = insn.rd();
  int ms1 = insn.rs1();
  int ms2 = insn.rs2();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      double sum = STATE.matrix_regs[md][i][j];  // accumulator (output-stationary)
      for (int k = 0; k < 3; k++) {
        sum += STATE.matrix_regs[ms1][i][k] * STATE.matrix_regs[ms2][j][k];
      }
      STATE.matrix_regs[md][i][j] = sum;
    }
  }
}
