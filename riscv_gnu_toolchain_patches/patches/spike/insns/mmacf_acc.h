// Matrix Multiply-Accumulate Float with Accumulator
// MMACF_ACC(md, ms1, ms2): md += ms1 * ms2 (3x3 output-stationary MAC, float precision)
// All three operands are matrix registers (encoded in rd, rs1, rs2 fields)
{
  int md  = insn.rd();
  int ms1 = insn.rs1();
  int ms2 = insn.rs2();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = (float)STATE.matrix_regs[md][i][j];  // accumulator
      for (int k = 0; k < 3; k++) {
        sum += (float)STATE.matrix_regs[ms1][i][k] * (float)STATE.matrix_regs[ms2][j][k];
      }
      STATE.matrix_regs[md][i][j] = (double)sum;
    }
  }
}
