// NOVA CORDIC Pipeline - Single precision hyperbolic cosine
require_either_extension('F', EXT_ZFINX);
require_fp;
softfloat_roundingMode = RM;
float arg = to_f(FRS1_F);
float result = coshf(arg);
uint32_t bits;
memcpy(&bits, &result, sizeof(bits));
WRITE_FRD_F(f32(bits));
set_fp_exceptions;
