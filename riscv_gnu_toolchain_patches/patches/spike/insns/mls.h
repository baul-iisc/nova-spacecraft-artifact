// Matrix Load with Stride (Single Precision)
// MLS(md, rs1, rs2): Load 3x3 floats from memory into matrix register md
// rs1 = base address (integer register), rs2 = row stride in bytes (integer register)
// md = matrix register index (encoded in rd field)
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  int md = insn.rd();
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      uint32_t bits = MMU.load<uint32_t>(addr + row * stride + col * 4);
      float val;
      memcpy(&val, &bits, sizeof(float));
      STATE.matrix_regs[md][row][col] = (double)val;
    }
  }
  // Do NOT write to integer rd — this is a matrix register operation
}
