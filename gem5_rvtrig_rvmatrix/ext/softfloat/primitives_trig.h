/*============================================================================

This C header file is part of the SoftFloat IEEE Floating-Point Arithmetic
Package, Release 3d, by John R. Hauser.

Copyright 2011, 2012, 2013, 2014, 2015, 2016, 2017 The Regents of the
University of California.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions, and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions, and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. Neither the name of the University nor the names of its contributors may
    be used to endorse or promote products derived from this software without
    specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS "AS IS", AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE
DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#ifndef primitives_trig_h
#define primitives_trig_h 1

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint16_t softfloat_approxRecipSin_1k0s[16];
extern const uint16_t softfloat_approxRecipSin_1k1s[16];

extern const uint16_t softfloat_approxRecipCos_1k0s[16];
extern const uint16_t softfloat_approxRecipCos_1k1s[16];

extern const uint16_t softfloat_approxRecipTan_1k0s[16];
extern const uint16_t softfloat_approxRecipTan_1k1s[16];

extern const uint16_t softfloat_approxRecipATan_1k0s[16];
extern const uint16_t softfloat_approxRecipATan_1k1s[16];

#ifndef softfloat_approxRecipSin32_1
uint32_t softfloat_approxRecipSin32_1( uint32_t a );
#endif

#ifndef softfloat_approxRecipCos32_1
uint32_t softfloat_approxRecipCos32_1( uint32_t a );
#endif

#ifndef softfloat_approxRecipTan32_1
uint32_t softfloat_approxRecipTan32_1( uint32_t a );
#endif

#ifndef softfloat_approxRecipATan32_1
uint32_t softfloat_approxRecipATan32_1( uint32_t a );
#endif

#ifndef softfloat_approxRecipSin64_1
uint64_t softfloat_approxRecipSin64_1( uint64_t a );
#endif

#ifndef softfloat_approxRecipCos64_1
uint64_t softfloat_approxRecipCos64_1( uint64_t a );
#endif

#ifndef softfloat_approxRecipTan64_1
uint64_t softfloat_approxRecipTan64_1( uint64_t a );
#endif

#ifndef softfloat_approxRecipATan64_1
uint64_t softfloat_approxRecipATan64_1( uint64_t a );
#endif

#ifdef __cplusplus
}
#endif

#endif
