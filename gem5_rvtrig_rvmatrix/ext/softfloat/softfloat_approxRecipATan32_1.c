/*============================================================================

This C source file is part of the SoftFloat IEEE Floating-Point Arithmetic
Package, Release 3d, by John R. Hauser.

=============================================================================*/

#include <stdint.h>
#include "platform.h"
#include "internals.h"
#include "specialize.h"
#include "softfloat.h"
#include "primitives.h"
#include "primitives_trig.h"
#ifndef softfloat_approxRecipATan32_1

extern const uint16_t softfloat_approxRecipATan_1k0s[];
extern const uint16_t softfloat_approxRecipATan_1k1s[];

uint32_t softfloat_approxRecipATan32_1( uint32_t a )
{
    int index;
    uint16_t eps, r0;
    uint32_t sigma0;
    uint_fast32_t r;
    uint32_t sqrSigma0;

    index = a>>27 & 0xF;
    eps = (uint16_t) (a>>11);
    r0 = softfloat_approxRecipATan_1k0s[index]
             - ((softfloat_approxRecipATan_1k1s[index] * (uint_fast32_t) eps)>>20);
    sigma0 = ~(uint_fast32_t) ((r0 * (uint_fast64_t) a)>>7);
    r = ((uint_fast32_t) r0<<16) + ((r0 * (uint_fast64_t) sigma0)>>24);
    sqrSigma0 = ((uint_fast64_t) sigma0 * sigma0)>>32;
    r += ((uint32_t) r * (uint_fast64_t) sqrSigma0)>>48;
    return r;
}

#endif
