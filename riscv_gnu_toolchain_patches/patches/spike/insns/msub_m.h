// Matrix Multiply-Accumulate (Double Precision)
// NOTE: MATCH_MSUB_M == MATCH_MMACD == 0x66002077 (encoding collision)
// msub_m dispatches FIRST in the linear insn_list.h, so this handler
// runs when the test code emits MMACD instructions.
// Implementation: md += ms1 * ms2^T (3x3 output-stationary MAC, double precision)
{
  int md  = insn.rd();
  int ms1 = insn.rs1();
  int ms2 = insn.rs2();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      double sum = STATE.matrix_regs[md][i][j];
      for (int k = 0; k < 3; k++) {
        sum += STATE.matrix_regs[ms1][i][k] * STATE.matrix_regs[ms2][j][k];
      }
      STATE.matrix_regs[md][i][j] = sum;
    }
  }
}
