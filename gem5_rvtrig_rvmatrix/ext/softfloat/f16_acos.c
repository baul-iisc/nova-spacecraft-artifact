#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float16_t f16_acos(float16_t a) {
    float a_float = float16_to_float(a);
    float result = acosf(a_float);
    return float_to_float16(result);
}

