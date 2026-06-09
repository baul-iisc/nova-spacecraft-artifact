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
#ifndef softfloat_approxRecipCos64_1

extern const uint16_t softfloat_approxRecipCos_1k0s[];
extern const uint16_t softfloat_approxRecipCos_1k1s[];

uint64_t softfloat_approxRecipCos64_1( uint64_t a )
{
    int index;
    uint16_t eps, r0;
    uint64_t sigma0;
    uint_fast64_t r;
    uint64_t sqrSigma0;

    index = a>>59 & 0xF;
    eps = (uint16_t) (a>>43);
    r0 = softfloat_approxRecipCos_1k0s[index]
             - ((softfloat_approxRecipCos_1k1s[index] * (uint_fast64_t) eps)>>20);
    sigma0 = ~(uint_fast64_t) ((r0 * (uint_fast64_t) a)>>7);
    r = ((uint_fast64_t) r0<<48) + ((r0 * (uint_fast64_t) sigma0)>>24);
    sqrSigma0 = ((uint_fast64_t) sigma0 * sigma0)>>32;
    r += ((uint64_t) r * (uint_fast64_t) sqrSigma0)>>48;
    return r;
}

#endif
