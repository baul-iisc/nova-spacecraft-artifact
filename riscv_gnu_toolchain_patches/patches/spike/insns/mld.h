// Matrix Load (Double Precision) - Load 3x3 matrix (9 x double) from contiguous memory
// MLD(md, rs1): rs1 = base address, md = destination matrix register
// Memory layout: row-major, 9 contiguous doubles (72 bytes)
{
  reg_t addr = RS1;
  int md = insn.rd();
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      uint64_t bits = MMU.load<uint64_t>(addr + (row * 3 + col) * 8);
      double val;
      memcpy(&val, &bits, sizeof(double));
      STATE.matrix_regs[md][row][col] = val;
    }
  }
  // Do NOT write to integer rd — this is a matrix register operation
}
