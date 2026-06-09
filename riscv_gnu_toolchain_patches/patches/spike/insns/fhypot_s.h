// NOVA CORDIC Pipeline - Single precision hypotenuse (binary: hypot(rs1, rs2))
require_either_extension('F', EXT_ZFINX);
require_fp;
softfloat_roundingMode = RM;
float a = to_f(FRS1_F);
float b = to_f(FRS2_F);
float result = hypotf(a, b);
uint32_t bits;
memcpy(&bits, &result, sizeof(bits));
WRITE_FRD_F(f32(bits));
set_fp_exceptions;
