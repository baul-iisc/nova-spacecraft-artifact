// Accumulator Store - copy accumulator ms1 to matrix register md
// MACC_STORE(md, ms1): matrix_regs[md] = matrix_regs[ms1]
{
  int md  = insn.rd();
  int ms1 = insn.rs1();
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      STATE.matrix_regs[md][i][j] = STATE.matrix_regs[ms1][i][j];
}
