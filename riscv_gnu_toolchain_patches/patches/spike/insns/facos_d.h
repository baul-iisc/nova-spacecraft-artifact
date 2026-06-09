// NOVA CORDIC - Double precision arccosine
require_either_extension('D', EXT_ZDINX);
require_fp;
softfloat_roundingMode = RM;
double arg = to_f(FRS1_D);
double result = acos(arg);
uint64_t bits;
memcpy(&bits, &result, sizeof(bits));
WRITE_FRD_D(f64(bits));
set_fp_exceptions;
