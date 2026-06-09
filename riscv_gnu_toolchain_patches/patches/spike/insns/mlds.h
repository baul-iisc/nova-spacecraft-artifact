// Matrix Load with Stride (Double Precision)
// MLDS(md, rs1, rs2): Load 3x3 doubles from memory into matrix register md
// rs1 = base address (integer register), rs2 = row stride in bytes (integer register)
// md = matrix register index (encoded in rd field)
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  int md = insn.rd();
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      uint64_t bits = MMU.load<uint64_t>(addr + row * stride + col * 8);
      double val;
      memcpy(&val, &bits, sizeof(double));
      STATE.matrix_regs[md][row][col] = val;
    }
  }
  // Do NOT write to integer rd — this is a matrix register operation
}
