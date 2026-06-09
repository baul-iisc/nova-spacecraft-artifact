// Matrix Store with Stride (Single Precision)
// MSS(md, rs1, rs2): Store matrix register md as 3x3 floats to memory
// md = matrix register index (encoded in rd field)
// rs1 = base address (integer register), rs2 = row stride in bytes (integer register)
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  int md = insn.rd();
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      float val = (float)STATE.matrix_regs[md][row][col];
      uint32_t bits;
      memcpy(&bits, &val, sizeof(uint32_t));
      MMU.store<uint32_t>(addr + row * stride + col * 4, bits);
    }
  }
}
