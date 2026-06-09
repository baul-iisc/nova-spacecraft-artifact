// Matrix-Vector Load - load vector into matrix register
{
  reg_t addr = RS1;
  reg_t stride = RS2;
  for (int i = 0; i < 3; i++) {
    MMU.load<uint64_t>(addr + i * stride);
  }
  WRITE_RD(addr);
}
