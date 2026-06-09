#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float64_t f64_hypot(float64_t a, float64_t b) {
    double a_double = float64_to_double(a);
    double b_double = float64_to_double(b);
    double result = hypot(a_double, b_double);
    return double_to_float64(result);
}
