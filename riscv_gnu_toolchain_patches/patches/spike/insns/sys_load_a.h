// Systolic Array Load A (Single Precision)
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  for (int i = 0; i < 9; i++) {
    MMU.load<uint32_t>(addr + i * 4);
  }
  WRITE_RD(addr);
}
