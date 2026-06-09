// NOVA CORDIC - Double precision arctangent2 (binary: atan2(rs1, rs2))
require_either_extension('D', EXT_ZDINX);
require_fp;
softfloat_roundingMode = RM;
double y = to_f(FRS1_D);
double x = to_f(FRS2_D);
double result = atan2(y, x);
uint64_t bits;
memcpy(&bits, &result, sizeof(bits));
WRITE_FRD_D(f64(bits));
set_fp_exceptions;
