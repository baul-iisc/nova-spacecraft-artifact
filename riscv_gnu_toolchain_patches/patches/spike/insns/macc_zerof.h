// Accumulator Zero (Float) - clear 3x3 matrix accumulator register
// MACC_ZEROF(md): Zero all elements of matrix register md
{
  int md = insn.rd();
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      STATE.matrix_regs[md][i][j] = 0.0;
}
