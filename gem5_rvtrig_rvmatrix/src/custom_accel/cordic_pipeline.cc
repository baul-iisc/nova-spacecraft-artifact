/*
 * NOVA Processor — CORDIC Pipeline Accelerator Implementation
 * PhD Research: Chandraboul
 *
 * Implements the complete CORDIC algorithm with:
 *   - Circular mode (sin, cos, tan, asin, acos, atan, atan2)
 *   - Hyperbolic mode (exp, ln, sinh, cosh, tanh, sqrt)
 *   - Magnitude (hypot) via circular vectoring
 *   - Configurable pipeline depth (default 32 iterations)
 *   - Full pipeline with pre/post-processing stages
 */

#include "custom_accel/cordic_pipeline.hh"

#include "base/trace.hh"
#include "debug/CORDICPipeline.hh"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gem5
{

/* ===================================================================
 * Constructor / Destructor
 * =================================================================== */

CORDICPipeline::CORDICPipeline(const Params &p)
    : ClockedObject(p),
      numIterations(p.num_iterations),
      pipelineDepth(p.num_iterations + 2),
      numCores(p.num_cores),
      pipelineFull(false),
      mmioBase(p.mmio_base),
      mmioSize(p.mmio_size),
      regCtrl(0), regStatus(0), regCoreId(0), regOpType(0), regPrec(0),
      regInputA(0), regInputB(0),
      mmioPort(name() + ".mmio_port", this),
      tickEvent([this]{ tick(); }, name() + ".tickEvent"),
      stats(this)
{
    /* Clamp iterations to valid range */
    if (numIterations < 1)  numIterations = 1;
    if (numIterations > MAX_ITERATIONS) numIterations = MAX_ITERATIONS;
    pipelineDepth = numIterations + 2;

    /* Allocate pipeline stages */
    stages.resize(pipelineDepth);

    /* Per-core buffers */
    resultBuffers.resize(numCores, 0.0);
    resultValid.resize(numCores, false);
    waitCycles.resize(numCores, 0);

    /* Initialize lookup tables */
    initLookupTables();

    /* Initialize per-operation latency table */
    initOpLatencies();

    DPRINTF(CORDICPipeline,
            "CORDICPipeline created: %d iterations, depth=%d, cores=%d, "
            "MMIO base=0x%lx\n",
            numIterations, pipelineDepth, numCores, mmioBase);
}

CORDICPipeline::~CORDICPipeline() = default;

/* ===================================================================
 * Lookup Table Initialization
 * =================================================================== */

void
CORDICPipeline::initLookupTables()
{
    /*
     * Circular angles: atan(2^-i) for i = 0, 1, ..., MAX_ITERATIONS-1
     * These are the elementary rotation angles for circular CORDIC.
     */
    for (int i = 0; i < MAX_ITERATIONS; i++) {
        circularAngles[i] = std::atan(std::ldexp(1.0, -i));
    }

    /*
     * Hyperbolic angles: atanh(2^-i) for i = 1, 2, ..., MAX_ITERATIONS
     * Note: hyperbolic CORDIC starts from i=1 (atanh(1) is undefined).
     * Index 0 is unused so hyperbolicAngles[i] corresponds to i.
     */
    hyperbolicAngles[0] = 0.0; /* unused */
    for (int i = 1; i < MAX_ITERATIONS; i++) {
        double val = std::ldexp(1.0, -i);
        if (std::abs(val) < 1.0) {
            hyperbolicAngles[i] = std::atanh(val);
        } else {
            hyperbolicAngles[i] = 0.0;
        }
    }

    /*
     * Circular gain: K_n = product_{i=0}^{n-1} sqrt(1 + 2^{-2i})
     * Its inverse is used to initialise x0 for sin/cos.
     */
    circularGain = 1.0;
    for (int i = 0; i < numIterations; i++) {
        circularGain *= std::sqrt(1.0 + std::ldexp(1.0, -2 * i));
    }
    circularGainInv = 1.0 / circularGain;

    /*
     * Hyperbolic gain: K_h = product_{i=1}^{n} sqrt(1 - 2^{-2i})
     * Must account for repeated iterations at i = 4, 13, 40, ...
     * (3k+1 rule for hyperbolic CORDIC convergence).
     */
    hyperbolicGain = 1.0;
    int repeatNext = 4;
    for (int i = 1; i <= numIterations; i++) {
        double factor = std::sqrt(1.0 - std::ldexp(1.0, -2 * i));
        hyperbolicGain *= factor;
        if (i == repeatNext) {
            hyperbolicGain *= factor; /* repeat iteration */
            repeatNext = 3 * repeatNext + 1;
        }
    }
    hyperbolicGainInv = 1.0 / hyperbolicGain;

    DPRINTF(CORDICPipeline,
            "CORDIC tables initialized: K_circ=%.10f (1/K=%.10f), "
            "K_hyp=%.10f (1/K=%.10f)\n",
            circularGain, circularGainInv,
            hyperbolicGain, hyperbolicGainInv);
}

/* ===================================================================
 * Per-Operation Latency Table
 * =================================================================== */

void
CORDICPipeline::initOpLatencies()
{
    /*
     * All operations go through the full pipeline, so latency is
     * pipelineDepth = numIterations + 2.  However, some operations
     * require additional pre/post steps (e.g., tan = sin/cos compute
     * + a division), so we add extra cycles for those.
     *
     * Default latencies (in cycles):
     */
    int base = pipelineDepth;

    opLatencyTable[OP_SIN]   = base;        /* straight rotation */
    opLatencyTable[OP_COS]   = base;
    opLatencyTable[OP_TAN]   = base + 4;    /* sin + cos + division */
    opLatencyTable[OP_ASIN]  = base + 2;    /* extra range reduction */
    opLatencyTable[OP_ACOS]  = base + 2;
    opLatencyTable[OP_ATAN]  = base;        /* vectoring mode */
    opLatencyTable[OP_ATAN2] = base;        /* vectoring mode */
    opLatencyTable[OP_EXP]   = base + 2;    /* hyperbolic + add */
    opLatencyTable[OP_LOG]   = base + 2;    /* hyperbolic vectoring + scale */
    opLatencyTable[OP_SINH]  = base;        /* hyperbolic rotation */
    opLatencyTable[OP_COSH]  = base;
    opLatencyTable[OP_TANH]  = base + 4;    /* sinh + cosh + division */
    opLatencyTable[OP_HYPOT] = base;        /* circular vectoring */
    opLatencyTable[OP_SQRT]  = base + 2;    /* hyperbolic vectoring + scale */
}

/* ===================================================================
 * Port / Address Range
 * =================================================================== */

Port &
CORDICPipeline::getPort(const std::string &name, PortID idx)
{
    if (name == "mmio_port")
        return mmioPort;
    return ClockedObject::getPort(name, idx);
}

AddrRangeList
CORDICPipeline::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(mmioBase, mmioBase + mmioSize));
    return ranges;
}

/* ===================================================================
 * Operation → Mode Mapping
 * =================================================================== */

CORDICPipeline::CORDICMode
CORDICPipeline::opToMode(CORDICOp op) const
{
    switch (op) {
        case OP_SIN:
        case OP_COS:
        case OP_TAN:
            return MODE_CIRCULAR_ROTATION;

        case OP_ASIN:
        case OP_ACOS:
        case OP_ATAN:
        case OP_ATAN2:
        case OP_HYPOT:
            return MODE_CIRCULAR_VECTORING;

        case OP_EXP:
        case OP_SINH:
        case OP_COSH:
        case OP_TANH:
            return MODE_HYPERBOLIC_ROTATION;

        case OP_LOG:
        case OP_SQRT:
            return MODE_HYPERBOLIC_VECTORING;

        default:
            return MODE_CIRCULAR_ROTATION;
    }
}

/* ===================================================================
 * Pipeline Interface
 * =================================================================== */

int
CORDICPipeline::submitOperation(CORDICOp op, Precision prec,
                                 double inputA, double inputB, int coreId)
{
    if (pipelineFull) {
        stats.pipelineStalls++;
        DPRINTF(CORDICPipeline, "Pipeline STALL: full, cannot accept op=%s "
                "from core %d\n", cordicOpName(op), coreId);
        return -1;
    }

    /* Create a new pipeline entry at stage 0 */
    PipelineStage &s0 = stages[0];
    s0.valid       = true;
    s0.op          = op;
    s0.mode        = opToMode(op);
    s0.prec        = prec;
    s0.coreId      = coreId;
    s0.enterTime   = curTick();
    s0.iteration   = 0;
    s0.resultReady = false;
    s0.result      = 0.0;

    /* Store raw inputs for pre-processing */
    s0.x = inputA;
    s0.y = inputB;
    s0.z = 0.0;

    /* Pre-process: range reduction and CORDIC register setup */
    preProcess(s0);

    stats.totalOperations++;
    stats.perOpCount[op]++;
    if (coreId < numCores) {
        stats.perCoreRequests[coreId]++;
    }

    resultValid[std::min(coreId, numCores - 1)] = false;

    DPRINTF(CORDICPipeline,
            "Submit: op=%s prec=%s core=%d inputA=%.6f inputB=%.6f "
            "-> x0=%.6f y0=%.6f z0=%.6f\n",
            cordicOpName(op), prec == PREC_F32 ? "f32" : "f64",
            coreId, inputA, inputB, s0.x, s0.y, s0.z);

    /* Schedule the tick event if not already scheduled */
    if (!tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + clockPeriod());
    }

    return getOpLatency(op);
}

bool
CORDICPipeline::resultReady(int coreId) const
{
    if (coreId >= 0 && coreId < numCores)
        return resultValid[coreId];
    return false;
}

double
CORDICPipeline::getResult(int coreId) const
{
    if (coreId >= 0 && coreId < numCores)
        return resultBuffers[coreId];
    return 0.0;
}

int
CORDICPipeline::getOpLatency(CORDICOp op) const
{
    if (op >= 0 && op < NUM_OPS)
        return opLatencyTable[op];
    return pipelineDepth;
}

/* ===================================================================
 * CORDIC Pre-Processing
 *
 * Sets up the initial x, y, z registers based on operation type.
 * Performs range reduction as needed.
 * =================================================================== */

void
CORDICPipeline::preProcess(PipelineStage &s)
{
    double a = s.x;  /* primary input */
    double b = s.y;  /* secondary input (for binary ops) */

    switch (s.op) {
        /* ---- Circular Rotation Mode: sin, cos, tan ---- */
        case OP_SIN:
        case OP_COS:
        case OP_TAN: {
            /*
             * Range reduction: map angle to [-π/2, π/2].
             * For CORDIC convergence, the angle must be in this range.
             */
            double angle = std::fmod(a, 2.0 * M_PI);
            if (angle > M_PI)  angle -= 2.0 * M_PI;
            if (angle < -M_PI) angle += 2.0 * M_PI;

            /* Track quadrant for sign correction */
            int quadrant = 0;
            if (angle > M_PI / 2.0) {
                angle = M_PI - angle;
                quadrant = 1;
            } else if (angle < -M_PI / 2.0) {
                angle = -M_PI - angle;
                quadrant = 2;
            }

            /* For rotation mode: x0 = 1/K, y0 = 0, z0 = angle */
            s.x = circularGainInv;
            s.y = 0.0;
            s.z = angle;
            /* Store quadrant in iteration field for post-processing */
            s.iteration = quadrant;
            break;
        }

        /* ---- Circular Vectoring Mode: atan, atan2 ---- */
        case OP_ATAN: {
            /* atan(a) = atan2(a, 1) */
            s.x = 1.0;
            s.y = a;
            s.z = 0.0;
            s.iteration = 0;
            break;
        }

        case OP_ATAN2: {
            /* atan2(y, x) — inputs: a=y, b=x */
            double x_in = b;
            double y_in = a;

            /*
             * Vectoring mode requires x > 0.
             * If x < 0, negate both and add/subtract π later.
             */
            int signAdjust = 0;
            if (x_in < 0) {
                x_in = -x_in;
                y_in = -y_in;
                signAdjust = (a >= 0) ? 1 : -1;
            }

            s.x = x_in;
            s.y = y_in;
            s.z = 0.0;
            s.iteration = signAdjust;
            break;
        }

        /* ---- Circular Vectoring: asin, acos ---- */
        case OP_ASIN: {
            /*
             * asin(a) using CORDIC vectoring:
             * Initialize with x0 = sqrt(1 - a²), y0 = a
             * Then vectoring gives atan(y0/x0) = atan(a/sqrt(1-a²)) = asin(a)
             */
            double clamped = std::max(-1.0, std::min(1.0, a));
            s.x = std::sqrt(1.0 - clamped * clamped);
            s.y = clamped;
            s.z = 0.0;
            s.iteration = 0;
            break;
        }

        case OP_ACOS: {
            /*
             * acos(a) = π/2 - asin(a)
             * Initialize same as asin, then subtract from π/2 in post.
             */
            double clamped = std::max(-1.0, std::min(1.0, a));
            s.x = std::sqrt(1.0 - clamped * clamped);
            s.y = clamped;
            s.z = 0.0;
            s.iteration = 0;
            break;
        }

        /* ---- Circular Vectoring: magnitude/hypot ---- */
        case OP_HYPOT: {
            /*
             * hypot(a, b) = sqrt(a² + b²)
             * Vectoring mode: x_n = K * sqrt(x0² + y0²)
             * So set x0 = a, y0 = b, result = x_n / K
             */
            s.x = std::abs(a);
            s.y = std::abs(b);
            s.z = 0.0;
            s.iteration = 0;
            break;
        }

        /* ---- Hyperbolic Rotation: exp, sinh, cosh, tanh ---- */
        case OP_EXP: {
            /*
             * exp(a) = cosh(a) + sinh(a)
             * Hyperbolic rotation with x0 = 1/Kh, y0 = 0, z0 = a
             * Result: x_n = cosh(a), y_n = sinh(a)
             *
             * Range reduction: use exp(a) = 2^k * exp(r) where r ∈ [-0.5, 0.5]
             */
            double z_val = a;
            int k = 0;
            if (std::abs(z_val) > 0.5) {
                /* Range reduction: a = k*ln(2) + r */
                k = (int)std::round(z_val / M_LN2);
                z_val = a - k * M_LN2;
            }

            s.x = hyperbolicGainInv;
            s.y = 0.0;
            s.z = z_val;
            s.iteration = k;  /* store scaling factor */
            break;
        }

        case OP_SINH: {
            /* sinh(a): hyperbolic rotation, result = y_n */
            double z_val = a;
            int k = 0;
            if (std::abs(z_val) > 0.5) {
                k = (int)std::round(z_val / M_LN2);
                z_val = a - k * M_LN2;
            }
            s.x = hyperbolicGainInv;
            s.y = 0.0;
            s.z = z_val;
            s.iteration = k;
            break;
        }

        case OP_COSH: {
            /* cosh(a): hyperbolic rotation, result = x_n */
            double z_val = a;
            int k = 0;
            if (std::abs(z_val) > 0.5) {
                k = (int)std::round(z_val / M_LN2);
                z_val = a - k * M_LN2;
            }
            s.x = hyperbolicGainInv;
            s.y = 0.0;
            s.z = z_val;
            s.iteration = k;
            break;
        }

        case OP_TANH: {
            /*
             * tanh(a) = sinh(a)/cosh(a)
             * Compute both via hyperbolic rotation, then divide in post.
             */
            s.x = hyperbolicGainInv;
            s.y = 0.0;
            s.z = a;
            s.iteration = 0;
            break;
        }

        /* ---- Hyperbolic Vectoring: log, sqrt ---- */
        case OP_LOG: {
            /*
             * ln(a) using CORDIC:
             * x0 = a + 1, y0 = a - 1
             * Vectoring gives z_n = atanh((a-1)/(a+1))
             * ln(a) = 2 * z_n
             *
             * Range reduction: a = m * 2^e, compute ln(m) + e*ln(2)
             */
            double val = std::abs(a);
            int exponent = 0;
            if (val > 0.0) {
                /* Normalize to [0.5, 2.0] range for CORDIC convergence */
                val = std::frexp(val, &exponent);
                val *= 2.0;  /* now val ∈ [1.0, 2.0) */
                exponent--;
            }

            s.x = val + 1.0;
            s.y = val - 1.0;
            s.z = 0.0;
            s.iteration = exponent;  /* store exponent for reconstruction */
            break;
        }

        case OP_SQRT: {
            /*
             * sqrt(a) using CORDIC hyperbolic vectoring:
             * x0 = a + 0.25, y0 = a - 0.25
             * Vectoring: x_n = Kh * sqrt(x0² - y0²) = Kh * sqrt(a)
             * So sqrt(a) = x_n / Kh
             *
             * For better convergence, normalize input to [0.25, 1]
             */
            double val = std::abs(a);
            int shift = 0;

            if (val > 0.0) {
                /* Normalize: factor out powers of 4 */
                while (val > 4.0) {
                    val /= 4.0;
                    shift++;
                }
                while (val < 0.25) {
                    val *= 4.0;
                    shift--;
                }
            }

            s.x = val + 0.25;
            s.y = val - 0.25;
            s.z = 0.0;
            s.iteration = shift;  /* power-of-4 scale factor */
            break;
        }

        default:
            s.x = a;
            s.y = b;
            s.z = 0.0;
            s.iteration = 0;
            break;
    }
}

/* ===================================================================
 * CORDIC Micro-Rotation Iteration
 *
 * Core of the algorithm.  One iteration per pipeline stage.
 * =================================================================== */

void
CORDICPipeline::cordicIteration(PipelineStage &s, int i)
{
    double x = s.x;
    double y = s.y;
    double z = s.z;
    double x_new, y_new, z_new;

    double shift = std::ldexp(1.0, -i);  /* 2^{-i} */

    if (s.mode == MODE_CIRCULAR_ROTATION ||
        s.mode == MODE_CIRCULAR_VECTORING) {
        /*
         * Circular CORDIC:
         *   x_{i+1} = x_i - d_i * y_i * 2^{-i}
         *   y_{i+1} = y_i + d_i * x_i * 2^{-i}
         *   z_{i+1} = z_i - d_i * atan(2^{-i})
         *
         * Rotation mode: d_i = sign(z_i)  (drive z → 0)
         * Vectoring mode: d_i = -sign(y_i) (drive y → 0)
         */
        int d;
        if (s.mode == MODE_CIRCULAR_ROTATION) {
            d = (z >= 0.0) ? 1 : -1;
        } else {
            d = (y < 0.0) ? 1 : -1;
        }

        x_new = x - d * y * shift;
        y_new = y + d * x * shift;
        z_new = z - d * circularAngles[i];

    } else {
        /*
         * Hyperbolic CORDIC (starts at i=1):
         *   x_{i+1} = x_i + d_i * y_i * 2^{-i}
         *   y_{i+1} = y_i + d_i * x_i * 2^{-i}
         *   z_{i+1} = z_i - d_i * atanh(2^{-i})
         *
         * Rotation mode: d_i = sign(z_i)
         * Vectoring mode: d_i = -sign(y_i)
         *
         * Iterations 4, 13, 40, 121, ... (3k+1) must be repeated.
         */
        int idx = std::max(1, i);  /* hyperbolic uses i>=1 */
        shift  = std::ldexp(1.0, -idx);

        int d;
        if (s.mode == MODE_HYPERBOLIC_ROTATION) {
            d = (z >= 0.0) ? 1 : -1;
        } else {
            d = (y < 0.0) ? 1 : -1;
        }

        x_new = x + d * y * shift;
        y_new = y + d * x * shift;
        z_new = z - d * hyperbolicAngles[idx];
    }

    s.x = x_new;
    s.y = y_new;
    s.z = z_new;
}

/* ===================================================================
 * CORDIC Post-Processing
 *
 * Extract the final result from the CORDIC registers after all
 * iterations complete.  Apply gain compensation and mode-specific
 * corrections.
 * =================================================================== */

void
CORDICPipeline::postProcess(PipelineStage &s)
{
    double result = 0.0;
    int extra = s.iteration;  /* stored during pre-processing */

    switch (s.op) {
        case OP_SIN: {
            /* After rotation: y_n ≈ sin(angle), x_n ≈ cos(angle) */
            result = s.y;
            /* Quadrant sign correction */
            if (extra == 1 || extra == 2)
                result = -result;
            break;
        }

        case OP_COS: {
            result = s.x;
            if (extra == 1)
                result = -result;
            else if (extra == 2)
                result = -result;
            break;
        }

        case OP_TAN: {
            /*
             * tan = sin/cos = y_n / x_n
             * No divide-by-zero since cos is never exactly 0 in CORDIC range.
             */
            if (std::abs(s.x) > 1e-300)
                result = s.y / s.x;
            else
                result = (s.y >= 0) ? 1e300 : -1e300;
            break;
        }

        case OP_ATAN: {
            /* After vectoring: z_n ≈ atan(y0/x0) = atan(a) */
            result = s.z;
            break;
        }

        case OP_ATAN2: {
            result = s.z;
            /* If original x was negative, adjust by ±π */
            if (extra == 1)
                result = M_PI + result;
            else if (extra == -1)
                result = -M_PI + result;
            break;
        }

        case OP_ASIN: {
            /* Vectoring gives atan(y0/x0) = asin(a) */
            result = s.z;
            break;
        }

        case OP_ACOS: {
            /* acos(a) = π/2 - asin(a) */
            result = M_PI / 2.0 - s.z;
            break;
        }

        case OP_HYPOT: {
            /* Vectoring: x_n = K * sqrt(x0² + y0²) */
            result = s.x * circularGainInv;
            break;
        }

        case OP_EXP: {
            /* exp(a) = cosh + sinh = x_n + y_n */
            result = s.x + s.y;
            /* Undo range reduction: multiply by 2^k */
            if (extra != 0)
                result = std::ldexp(result, extra);
            break;
        }

        case OP_SINH: {
            result = s.y;
            /* Reconstruct: sinh(a) with range reduction */
            if (extra != 0) {
                double cosh_val = s.x;
                double sinh_val = s.y;
                double scale = std::ldexp(1.0, std::abs(extra));
                /* sinh(a) = (exp(a) - exp(-a))/2 after scaling */
                double exp_val = cosh_val + sinh_val;
                double exp_neg = cosh_val - sinh_val;
                if (extra > 0)
                    result = (exp_val * scale - exp_neg / scale) / 2.0;
                else
                    result = (exp_val / scale - exp_neg * scale) / 2.0;
            }
            break;
        }

        case OP_COSH: {
            result = s.x;
            if (extra != 0) {
                double cosh_val = s.x;
                double sinh_val = s.y;
                double scale = std::ldexp(1.0, std::abs(extra));
                double exp_val = cosh_val + sinh_val;
                double exp_neg = cosh_val - sinh_val;
                if (extra > 0)
                    result = (exp_val * scale + exp_neg / scale) / 2.0;
                else
                    result = (exp_val / scale + exp_neg * scale) / 2.0;
            }
            break;
        }

        case OP_TANH: {
            /* tanh = sinh/cosh = y_n / x_n */
            if (std::abs(s.x) > 1e-300)
                result = s.y / s.x;
            else
                result = (s.y >= 0) ? 1.0 : -1.0;
            break;
        }

        case OP_LOG: {
            /*
             * Vectoring: z_n = atanh((a-1)/(a+1))
             * ln(a) = 2 * z_n  (for the normalized mantissa)
             * Add exponent correction: + extra * ln(2)
             */
            result = 2.0 * s.z;
            if (extra != 0)
                result += extra * M_LN2;
            break;
        }

        case OP_SQRT: {
            /*
             * Vectoring: x_n = Kh * sqrt(x0² - y0²)
             *           = Kh * sqrt((a+0.25)² - (a-0.25)²)
             *           = Kh * sqrt(a)
             * So sqrt(a) = x_n / Kh = x_n * hyperbolicGainInv
             *
             * Undo range reduction: multiply by 2^shift
             */
            result = s.x * hyperbolicGainInv;
            if (extra != 0)
                result = std::ldexp(result, extra);
            break;
        }

        default:
            result = 0.0;
            break;
    }

    /* Clamp f32 precision if needed */
    if (s.prec == PREC_F32) {
        result = (double)(float)result;
    }

    s.result = result;
    s.resultReady = true;

    DPRINTF(CORDICPipeline,
            "PostProcess: op=%s core=%d result=%.10f "
            "(x=%.6f y=%.6f z=%.6f)\n",
            cordicOpName(s.op), s.coreId, result, s.x, s.y, s.z);
}

/* ===================================================================
 * Pipeline Tick — Advance All Stages
 * =================================================================== */

void
CORDICPipeline::tick()
{
    bool anyActive = false;

    /*
     * Process pipeline in reverse order (last stage first) to avoid
     * overwriting data that has not yet been read.
     */

    /* Stage N+1 (last): post-processing and result write-back */
    int lastStage = pipelineDepth - 1;
    if (stages[lastStage].valid) {
        PipelineStage &s = stages[lastStage];
        postProcess(s);

        /* Write result to core's buffer */
        int coreId = std::min(s.coreId, numCores - 1);
        resultBuffers[coreId] = s.result;
        resultValid[coreId] = true;

        Tick latency = curTick() - s.enterTime;
        stats.totalLatencyCycles += latency / clockPeriod();
        stats.opLatencyDist.sample(latency / clockPeriod());
        if (coreId < numCores) {
            stats.perCoreWaitCycles[coreId] += latency / clockPeriod();
        }

        s.valid = false;
        anyActive = true;
    }

    /* Middle stages: CORDIC iterations (shift pipeline down) */
    for (int stage = lastStage - 1; stage >= 1; stage--) {
        if (stages[stage].valid && !stages[stage + 1].valid) {
            /* Perform CORDIC iteration for this stage */
            int iterIdx = stage - 1;  /* stage 1 → iteration 0, etc. */

            /* Handle hyperbolic repeated iterations */
            if (stages[stage].mode == MODE_HYPERBOLIC_ROTATION ||
                stages[stage].mode == MODE_HYPERBOLIC_VECTORING) {
                int hIdx = iterIdx + 1;  /* hyperbolic starts at i=1 */
                cordicIteration(stages[stage], hIdx);

                /* Check for repeated iterations (4, 13, 40, 121, ...) */
                static const int repeats[] = {4, 13, 40, 121, 364};
                for (int r = 0; r < 5; r++) {
                    if (hIdx == repeats[r]) {
                        cordicIteration(stages[stage], hIdx);
                        break;
                    }
                }
            } else {
                cordicIteration(stages[stage], iterIdx);
            }

            /* Advance to next stage */
            stages[stage + 1] = stages[stage];
            stages[stage].valid = false;
            anyActive = true;
        } else if (stages[stage].valid) {
            anyActive = true;
        }
    }

    /* Stage 0: pre-processing already done during submit, just advance */
    if (stages[0].valid && !stages[1].valid) {
        stages[1] = stages[0];
        stages[0].valid = false;
        anyActive = true;
    } else if (stages[0].valid) {
        anyActive = true;
    }

    /* Update pipeline full status */
    pipelineFull = stages[0].valid;

    /* Track utilization */
    if (anyActive) {
        stats.pipelineActiveCycles++;
    } else {
        stats.pipelineIdleCycles++;
    }

    /* Reschedule tick if pipeline is active */
    if (anyActive && !tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + clockPeriod());
    }
}

/* ===================================================================
 * MMIO Handler
 * =================================================================== */

Tick
CORDICPipeline::handleMMIO(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - mmioBase;

    if (pkt->isRead()) {
        uint64_t data = 0;

        switch (offset) {
            case REG_CTRL:
                data = regCtrl;
                break;
            case REG_STATUS:
                data = regStatus;
                if (pipelineFull)
                    data |= STATUS_FULL;
                for (auto &s : stages) {
                    if (s.valid) { data |= STATUS_BUSY; break; }
                }
                break;
            case REG_CORE_ID:
                data = regCoreId;
                break;
            case REG_RESULT:
                if (regCoreId < (uint64_t)numCores) {
                    double val = resultBuffers[regCoreId];
                    std::memcpy(&data, &val, sizeof(double));
                }
                break;
            case REG_WAIT_CYCLES:
                if (regCoreId < (uint64_t)numCores)
                    data = waitCycles[regCoreId];
                break;
            case REG_OP_TYPE:
                data = regOpType;
                break;
            case REG_PRECISION:
                data = regPrec;
                break;
            case REG_NUM_STAGES:
                data = numIterations;
                break;
            case REG_PIPE_STATUS: {
                /* Count occupied stages */
                int occupied = 0;
                for (auto &s : stages)
                    if (s.valid) occupied++;
                data = occupied;
                break;
            }
        }

        pkt->setData((uint8_t*)&data);
        DPRINTF(CORDICPipeline, "MMIO Read: offset=0x%lx, data=0x%lx\n",
                offset, data);
    }
    else if (pkt->isWrite()) {
        uint64_t data;
        pkt->writeData((uint8_t*)&data);

        switch (offset) {
            case REG_CTRL:
                regCtrl = data;
                if (data & CTRL_START) {
                    if (regCoreId < (uint64_t)numCores &&
                        regOpType < NUM_OPS) {
                        CORDICOp op = static_cast<CORDICOp>(regOpType);
                        Precision prec = (regPrec == 0) ?
                                         PREC_F32 : PREC_F64;
                        submitOperation(op, prec, regInputA, regInputB,
                                        regCoreId);
                    }
                    regCtrl = 0;
                }
                break;
            case REG_CORE_ID:
                regCoreId = data;
                break;
            case REG_INPUT_A:
                std::memcpy(&regInputA, &data, sizeof(double));
                break;
            case REG_INPUT_B:
                std::memcpy(&regInputB, &data, sizeof(double));
                break;
            case REG_OP_TYPE:
                regOpType = data;
                break;
            case REG_PRECISION:
                regPrec = data;
                break;
        }
    }

    return clockPeriod();
}

/* ===================================================================
 * Statistics
 * =================================================================== */

CORDICPipeline::CORDICPipelineStats::CORDICPipelineStats(
    CORDICPipeline *parent)
    : statistics::Group(parent),
      ADD_STAT(totalOperations, statistics::units::Count::get(),
               "Total CORDIC operations"),
      ADD_STAT(pipelineStalls, statistics::units::Count::get(),
               "Pipeline stall events (full)"),
      ADD_STAT(totalLatencyCycles, statistics::units::Cycle::get(),
               "Total operation latency cycles"),
      ADD_STAT(contentionEvents, statistics::units::Count::get(),
               "Contention events"),
      ADD_STAT(perOpCount, statistics::units::Count::get(),
               "Per-operation type count"),
      ADD_STAT(perCoreRequests, statistics::units::Count::get(),
               "Per-core request count"),
      ADD_STAT(perCoreWaitCycles, statistics::units::Cycle::get(),
               "Per-core wait cycles"),
      ADD_STAT(opLatencyDist, statistics::units::Cycle::get(),
               "Operation latency distribution"),
      ADD_STAT(pipelineActiveCycles, statistics::units::Cycle::get(),
               "Pipeline active cycles"),
      ADD_STAT(pipelineIdleCycles, statistics::units::Cycle::get(),
               "Pipeline idle cycles")
{
    perOpCount.init(CORDICPipeline::NUM_OPS);
    for (int i = 0; i < CORDICPipeline::NUM_OPS; i++) {
        perOpCount.subname(i, cordicOpName(
            static_cast<CORDICPipeline::CORDICOp>(i)));
    }

    perCoreRequests.init(parent->numCores);
    perCoreWaitCycles.init(parent->numCores);

    opLatencyDist
        .init(0, 128, 4)
        .flags(statistics::nozero);
}

} // namespace gem5
