// Matrix Store (Double Precision) - Store 3x3 matrix (9 x double) to contiguous memory
// MSD(md, rs1): rs1 = base address, md = source matrix register
// Memory layout: row-major, 9 contiguous doubles (72 bytes)
{
  reg_t addr = RS1;
  int md = insn.rd();
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      double val = STATE.matrix_regs[md][row][col];
      uint64_t bits;
      memcpy(&bits, &val, sizeof(uint64_t));
      MMU.store<uint64_t>(addr + (row * 3 + col) * 8, bits);
    }
  }
}
