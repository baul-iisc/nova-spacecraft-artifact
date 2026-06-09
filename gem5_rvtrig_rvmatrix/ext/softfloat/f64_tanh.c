#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float64_t f64_tanh(float64_t a) {
    double a_double = float64_to_double(a);
    double result = tanh(a_double);
    return double_to_float64(result);
}
