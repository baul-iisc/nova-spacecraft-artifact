// Matrix-Vector Store - store vector from matrix register
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  for (int i = 0; i < 3; i++) {
    MMU.store<uint64_t>(addr + i * stride, 0);
  }
}
