// Systolic Array Store C (Double Precision)
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  for (int i = 0; i < 9; i++) {
    MMU.store<uint64_t>(addr + i * 8, 0);
  }
}
