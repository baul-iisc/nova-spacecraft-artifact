// Matrix Move - broadcast scalar to all matrix elements (Double Precision)
// MMV_XMU(md, rs1): Broadcast double-reinterpreted bits of integer rs1 to all 3x3 elements
// MZERO(md) = MMV_XMU(md, X0) -> broadcasts 0.0 to all elements
{
  int md = insn.rd();
  uint64_t bits = RS1;  // integer register value (0 for MZERO via x0)
  double val;
  memcpy(&val, &bits, sizeof(double));
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      STATE.matrix_regs[md][i][j] = val;
    }
  }
}
