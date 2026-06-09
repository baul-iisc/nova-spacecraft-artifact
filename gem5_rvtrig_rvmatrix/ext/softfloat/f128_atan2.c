#include <quadmath.h>
#include <quadmath.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float128_t f128_atan2(float128_t y, float128_t x) {
    _Float128 y_quad = float128_to__Float128(y);
    _Float128 x_quad = float128_to__Float128(x);
    _Float128 result = atan2q(y_quad, x_quad);
    return _Float128_to_float128(result);
}

