// Matrix Store (Single Precision) - Store 3x3 matrix (9 x int32) to memory
{
  reg_t addr = RS1;
  reg_t val = RD;
  for (int i = 0; i < 9; i++) {
    MMU.store<uint32_t>(addr + i * 4, (uint32_t)(val >> (i * 4)));
  }
}
