// Matrix Subtract (Double Precision Float)
// MSUBFD(md, ms1, ms2): md[i][j] = ms1[i][j] - ms2[i][j] for all 3x3 elements
// All three operands are matrix registers
{
  int md  = insn.rd();
  int ms1 = insn.rs1();
  int ms2 = insn.rs2();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      STATE.matrix_regs[md][i][j] = STATE.matrix_regs[ms1][i][j] - STATE.matrix_regs[ms2][i][j];
    }
  }
}
