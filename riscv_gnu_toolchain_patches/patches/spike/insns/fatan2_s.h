// NOVA CORDIC - Single precision arctangent2 (binary: atan2(rs1, rs2))
require_either_extension('F', EXT_ZFINX);
require_fp;
softfloat_roundingMode = RM;
float y = to_f(FRS1_F);
float x = to_f(FRS2_F);
float result = atan2f(y, x);
uint32_t bits;
memcpy(&bits, &result, sizeof(bits));
WRITE_FRD_F(f32(bits));
set_fp_exceptions;
