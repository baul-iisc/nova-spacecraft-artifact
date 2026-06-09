// Systolic Array Store C (Single Precision)
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  for (int i = 0; i < 9; i++) {
    MMU.store<uint32_t>(addr + i * 4, 0);
  }
}
