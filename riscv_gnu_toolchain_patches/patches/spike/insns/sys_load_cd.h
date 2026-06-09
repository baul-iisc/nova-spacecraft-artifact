// Systolic Array Load C (Double Precision)
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  for (int i = 0; i < 9; i++) {
    MMU.load<uint64_t>(addr + i * 8);
  }
  WRITE_RD(addr);
}
