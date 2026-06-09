#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float16_t f16_atan2(float16_t y, float16_t x) {
    float y_float = float16_to_float(y);
    float x_float = float16_to_float(x);
    float result = atan2f(y_float, x_float);
    return float_to_float16(result);
}

