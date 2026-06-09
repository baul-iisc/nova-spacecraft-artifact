// NOVA CORDIC Pipeline - Double precision hypotenuse (binary: hypot(rs1, rs2))
require_either_extension('D', EXT_ZDINX);
require_fp;
softfloat_roundingMode = RM;
double a = to_f(FRS1_D);
double b = to_f(FRS2_D);
double result = hypot(a, b);
uint64_t bits;
memcpy(&bits, &result, sizeof(bits));
WRITE_FRD_D(f64(bits));
set_fp_exceptions;
