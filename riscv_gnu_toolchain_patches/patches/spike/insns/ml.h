// Matrix Load (Single Precision) - Load 3x3 matrix (9 x int32) from memory
{
  reg_t addr = RS1;
  for (int i = 0; i < 9; i++) {
    WRITE_REG(insn.rd(), MMU.load<uint32_t>(addr + i * 4));
  }
  // In simulation, store the base address in rd for tracking
  WRITE_RD(addr);
}
