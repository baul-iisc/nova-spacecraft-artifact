/*
 * Copyright (c) 2010, 2017-2018, 2020, 2022 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU__OP_CLASS_HH__
#define __CPU__OP_CLASS_HH__

#include "enums/OpClass.hh"

namespace gem5
{

/*
 * Do a bunch of wonky stuff to maintain backward compatability so I
 * don't have to change code in a zillion places.
 */
using enums::OpClass;
using enums::No_OpClass;

static const OpClass IntAluOp = enums::IntAlu;
static const OpClass IntMultOp = enums::IntMult;
static const OpClass IntDivOp = enums::IntDiv;
static const OpClass FloatAddOp = enums::FloatAdd;
static const OpClass FloatCmpOp = enums::FloatCmp;
static const OpClass FloatCvtOp = enums::FloatCvt;
static const OpClass FloatMultOp = enums::FloatMult;
static const OpClass FloatMultAccOp = enums::FloatMultAcc;
static const OpClass FloatDivOp = enums::FloatDiv;
static const OpClass FloatMiscOp = enums::FloatMisc;
static const OpClass FloatSqrtOp = enums::FloatSqrt;
static const OpClass FloatSinOp = enums::FloatSin;
static const OpClass FloatCosOp = enums::FloatCos;
static const OpClass FloatTanOp = enums::FloatTan;
static const OpClass FloataTanOp = enums::FloataTan;
static const OpClass FloataSinOp = enums::FloataSin;
static const OpClass FloataCosOp = enums::FloataCos;
static const OpClass FloataTan2Op = enums::FloataTan2;
static const OpClass FloatExpOp = enums::FloatExp;
static const OpClass FloatLogOp = enums::FloatLog;
static const OpClass FloatSinhOp = enums::FloatSinh;
static const OpClass FloatCoshOp = enums::FloatCosh;
static const OpClass FloatTanhOp = enums::FloatTanh;
static const OpClass FloatHypotOp = enums::FloatHypot;
static const OpClass SimdAddOp = enums::SimdAdd;
static const OpClass SimdAddAccOp = enums::SimdAddAcc;
static const OpClass SimdAluOp = enums::SimdAlu;
static const OpClass SimdCmpOp = enums::SimdCmp;
static const OpClass SimdCvtOp = enums::SimdCvt;
static const OpClass SimdMiscOp = enums::SimdMisc;
static const OpClass SimdMultOp = enums::SimdMult;
static const OpClass SimdMultAccOp = enums::SimdMultAcc;
static const OpClass SimdMatMultAccOp = enums::SimdMatMultAcc;
static const OpClass SimdShiftOp = enums::SimdShift;
static const OpClass SimdShiftAccOp = enums::SimdShiftAcc;
static const OpClass SimdDivOp = enums::SimdDiv;
static const OpClass SimdSqrtOp = enums::SimdSqrt;
static const OpClass SimdReduceAddOp = enums::SimdReduceAdd;
static const OpClass SimdReduceAluOp = enums::SimdReduceAlu;
static const OpClass SimdReduceCmpOp = enums::SimdReduceCmp;
static const OpClass SimdFloatAddOp = enums::SimdFloatAdd;
static const OpClass SimdFloatAluOp = enums::SimdFloatAlu;
static const OpClass SimdFloatCmpOp = enums::SimdFloatCmp;
static const OpClass SimdFloatCvtOp = enums::SimdFloatCvt;
static const OpClass SimdFloatDivOp = enums::SimdFloatDiv;
static const OpClass SimdFloatMiscOp = enums::SimdFloatMisc;
static const OpClass SimdFloatMultOp = enums::SimdFloatMult;
static const OpClass SimdFloatMultAccOp = enums::SimdFloatMultAcc;
static const OpClass SimdFloatMatMultAccOp = enums::SimdFloatMatMultAcc;
static const OpClass SimdFloatSqrtOp = enums::SimdFloatSqrt;
static const OpClass SimdFloatSinOp = enums::SimdFloatSin;
static const OpClass SimdFloatCosOp = enums::SimdFloatCos;
static const OpClass SimdFloataSinOp = enums::SimdFloataSin;
static const OpClass SimdFloataCosOp = enums::SimdFloataCos;
static const OpClass SimdFloatTanOp = enums::SimdFloatTan;
static const OpClass SimdFloataTanOp = enums::SimdFloataTan;
static const OpClass SimdFloataTan2Op = enums::SimdFloataTan2;
static const OpClass SimdFloatReduceCmpOp = enums::SimdFloatReduceCmp;
static const OpClass SimdFloatReduceAddOp = enums::SimdFloatReduceAdd;
static const OpClass SimdAesOp = enums::SimdAes;
static const OpClass SimdAesMixOp = enums::SimdAesMix;
static const OpClass SimdSha1HashOp = enums::SimdSha1Hash;
static const OpClass SimdSha1Hash2Op = enums::SimdSha1Hash2;
static const OpClass SimdSha256HashOp = enums::SimdSha256Hash;
static const OpClass SimdSha256Hash2Op = enums::SimdSha256Hash2;
static const OpClass SimdShaSigma2Op = enums::SimdShaSigma2;
static const OpClass SimdShaSigma3Op = enums::SimdShaSigma3;
static const OpClass SimdPredAluOp = enums::SimdPredAlu;
static const OpClass MatrixOp = enums::Matrix;
static const OpClass MatrixMovOp = enums::MatrixMov;
static const OpClass MatrixOPOp = enums::MatrixOP;
static const OpClass MemReadOp = enums::MemRead;
static const OpClass MemWriteOp = enums::MemWrite;
static const OpClass FloatMemReadOp = enums::FloatMemRead;
static const OpClass FloatMemWriteOp = enums::FloatMemWrite;
static const OpClass SimdUnitStrideLoadOp = enums::SimdUnitStrideLoad;
static const OpClass SimdUnitStrideStoreOp = enums::SimdUnitStrideStore;
static const OpClass SimdUnitStrideMaskLoadOp
             = enums::SimdUnitStrideMaskLoad;
static const OpClass SimdUnitStrideMaskStoreOp
             = enums::SimdUnitStrideMaskStore;
static const OpClass SimdStridedLoadOp = enums::SimdStridedLoad;
static const OpClass SimdStridedStoreOp = enums::SimdStridedStore;
static const OpClass SimdIndexedLoadOp = enums::SimdIndexedLoad;
static const OpClass SimdIndexedStoreOp = enums::SimdIndexedStore;
static const OpClass SimdUnitStrideFaultOnlyFirstLoadOp
             = enums::SimdUnitStrideFaultOnlyFirstLoad;
static const OpClass SimdWholeRegisterLoadOp
             = enums::SimdWholeRegisterLoad;
static const OpClass SimdWholeRegisterStoreOp
             = enums::SimdWholeRegisterStore;
static const OpClass IprAccessOp = enums::IprAccess;
static const OpClass InstPrefetchOp = enums::InstPrefetch;
static const OpClass SimdUnitStrideSegmentedLoadOp = enums::SimdUnitStrideSegmentedLoad;
static const OpClass SimdUnitStrideSegmentedStoreOp
             = enums::SimdUnitStrideSegmentedStore;
static const OpClass SimdExtOp = enums::SimdExt;
static const OpClass SimdFloatExtOp = enums::SimdFloatExt;
static const OpClass SimdConfigOp = enums::SimdConfig;

// SpaceGPU Operations (PhD Research: Spacecraft Visualization)
static const OpClass GpuRenderOp = enums::GpuRender;
static const OpClass GpuClearOp = enums::GpuClear;
static const OpClass GpuDrawPointsOp = enums::GpuDrawPoints;
static const OpClass GpuDrawLinesOp = enums::GpuDrawLines;
static const OpClass GpuDrawTerrainOp = enums::GpuDrawTerrain;
static const OpClass GpuDrawStarsOp = enums::GpuDrawStars;
static const OpClass GpuComputeOp = enums::GpuCompute;
static const OpClass GpuReduceOp = enums::GpuReduce;
static const OpClass GpuThermalMapOp = enums::GpuThermalMap;
static const OpClass GpuHazardMapOp = enums::GpuHazardMap;
static const OpClass GpuDepthMapOp = enums::GpuDepthMap;
static const OpClass GpuBlendOp = enums::GpuBlend;

static const OpClass Num_OpClasses = enums::Num_OpClass;

} // namespace gem5

#endif // __CPU__OP_CLASS_HH__
