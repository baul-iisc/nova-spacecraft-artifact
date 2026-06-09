#include <quadmath.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float128_t f128_tan(float128_t a) {
    _Float128 a_quad = float128_to__Float128(a);
    _Float128 result = tanq(a_quad);
    return _Float128_to_float128(result);
}

