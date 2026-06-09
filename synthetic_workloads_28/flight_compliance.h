#ifndef FLIGHT_COMPLIANCE_H
#define FLIGHT_COMPLIANCE_H

/* Flight-style compliance shim.
 *
 * Two build modes, selected by the FLIGHT_HOST_IO preprocessor symbol:
 *
 *  -DFLIGHT_HOST_IO          gem5 / FPGA / host evaluation build.
 *                            heap APIs fall through to libc malloc/free
 *                            (unlimited heap, real reclamation), printf
 *                            / fprintf / puts go to libc stdio so the
 *                            simulator can capture and validate workload
 *                            output, and host filesystem I/O works.
 *                            FLIGHT_SAFE_HALT calls exit() so simulations
 *                            terminate cleanly.
 *
 *  (FLIGHT_HOST_IO undefined) flight build for an embedded RTOS target.
 *                            heap APIs use a deterministic static pool,
 *                            stdio is replaced by a no-op telemetry stub
 *                            (real flight software emits CCSDS HK packets,
 *                            not host stdio), file I/O wrappers are no-ops,
 *                            and FLIGHT_SAFE_HALT is a watchdog-friendly
 *                            spin that the platform fault manager catches.
 *
 * Workloads are written once and compile cleanly under both modes, so the
 * synthetic suite is a flight-grade reference and the gem5 evaluation
 * produces correct numerical output at the same time.
 *
 * IMPORTANT: include this header AFTER all libc headers (<stdio.h>,
 * <stdlib.h>, <math.h>, etc.).  The malloc/printf overrides at the
 * bottom of this file are macros, so any libc declaration of those
 * names that is processed afterwards would be corrupted.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

/* In host evaluation mode the shim itself uses libc stdio/stdlib, so pull
 * those headers in here BEFORE any of the malloc/printf macro overrides
 * at the bottom of this file. */
#ifdef FLIGHT_HOST_IO
#include <stdio.h>
#include <stdlib.h>
#endif

/* ------------------------------------------------------------------------
 * Heap APIs
 *   FLIGHT_HOST_IO  : pass through to libc malloc/calloc/realloc/free
 *                     (gem5/FPGA evaluation needs real reclamation).
 *   else            : deterministic static-pool allocator suitable for
 *                     init-time allocation in embedded flight software.
 * ------------------------------------------------------------------------ */
#ifdef FLIGHT_HOST_IO

static void *flight_malloc_impl(size_t size)        { return malloc(size); }
static void *flight_calloc_impl(size_t n, size_t s) { return calloc(n, s); }
static void *flight_realloc_impl(void *p, size_t s) { return realloc(p, s); }
static void  flight_free_impl(void *p)              { free(p); }

#else /* flight build: static-pool allocator */

#ifndef FLIGHT_POOL_BYTES
#define FLIGHT_POOL_BYTES (64U * 1024U * 1024U)
#endif

#ifndef FLIGHT_POOL_MAX_ALLOCS
#define FLIGHT_POOL_MAX_ALLOCS 8192U
#endif

typedef struct {
    void *ptr;
    size_t size;
    uint8_t active;
} flight_alloc_entry_t;

static uint8_t flight_pool[FLIGHT_POOL_BYTES];
static size_t flight_pool_offset = 0U;
static flight_alloc_entry_t flight_alloc_table[FLIGHT_POOL_MAX_ALLOCS];
static size_t flight_alloc_count = 0U;

static size_t flight_align8(size_t n)
{
    return (n + 7U) & ~(size_t)7U;
}

static void *flight_malloc_impl(size_t size)
{
    size_t need;
    void *p;
    if (size == 0U) {
        return NULL;
    }
    need = flight_align8(size);
    if ((flight_pool_offset + need) > FLIGHT_POOL_BYTES) {
        return NULL;
    }
    if (flight_alloc_count >= FLIGHT_POOL_MAX_ALLOCS) {
        return NULL;
    }
    p = (void *)&flight_pool[flight_pool_offset];
    flight_pool_offset += need;
    flight_alloc_table[flight_alloc_count].ptr = p;
    flight_alloc_table[flight_alloc_count].size = size;
    flight_alloc_table[flight_alloc_count].active = 1U;
    flight_alloc_count++;
    return p;
}

static void *flight_calloc_impl(size_t n, size_t size)
{
    size_t total;
    void *p;
    if ((n == 0U) || (size == 0U)) {
        return NULL;
    }
    total = n * size;
    if ((n != 0U) && ((total / n) != size)) {
        return NULL;
    }
    p = flight_malloc_impl(total);
    if (p != NULL) {
        (void)memset(p, 0, total);
    }
    return p;
}

static size_t flight_find_size(const void *ptr)
{
    size_t i;
    for (i = 0U; i < flight_alloc_count; i++) {
        if ((flight_alloc_table[i].active == 1U) && (flight_alloc_table[i].ptr == ptr)) {
            return flight_alloc_table[i].size;
        }
    }
    return 0U;
}

static void flight_free_impl(void *ptr)
{
    size_t i;
    if (ptr == NULL) {
        return;
    }
    for (i = 0U; i < flight_alloc_count; i++) {
        if ((flight_alloc_table[i].active == 1U) && (flight_alloc_table[i].ptr == ptr)) {
            flight_alloc_table[i].active = 0U;
            return;
        }
    }
}

static void *flight_realloc_impl(void *ptr, size_t size)
{
    void *pnew;
    size_t old_size;
    size_t copy_size;
    if (ptr == NULL) {
        return flight_malloc_impl(size);
    }
    if (size == 0U) {
        flight_free_impl(ptr);
        return NULL;
    }
    old_size = flight_find_size(ptr);
    pnew = flight_malloc_impl(size);
    if (pnew == NULL) {
        return NULL;
    }
    copy_size = (old_size < size) ? old_size : size;
    if (copy_size > 0U) {
        (void)memcpy(pnew, ptr, copy_size);
    }
    flight_free_impl(ptr);
    return pnew;
}

#endif /* FLIGHT_HOST_IO ? libc-passthrough : static-pool */

/* ------------------------------------------------------------------------
 * Telemetry / log stub
 *   FLIGHT_LOG(fmt, ...) routes through this function.
 *
 *   - host-IO build : forward to vprintf so workload telemetry reaches
 *                     gem5's captured stdout for output validation.
 *   - flight build  : no-op (real flight code emits structured CCSDS HK
 *                     packets, not host stdio).
 * ------------------------------------------------------------------------ */
static void flight_log_stub(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
#ifdef FLIGHT_HOST_IO
    if (fmt != NULL) {
        (void)vprintf(fmt, ap);
    }
#else
    (void)fmt;
#endif
    va_end(ap);
}

/* ------------------------------------------------------------------------
 * Host-filesystem I/O shim
 * ------------------------------------------------------------------------ */
#ifdef FLIGHT_HOST_IO
typedef FILE flight_file_t;
#define FLIGHT_FOPEN(path, mode)        fopen((path), (mode))
#define FLIGHT_FCLOSE(fp)               ((void)fclose((fp)))
#define FLIGHT_FREAD(buf, sz, n, fp)    fread((buf), (sz), (n), (fp))
#define FLIGHT_FWRITE(buf, sz, n, fp)   fwrite((buf), (sz), (n), (fp))
#else
typedef void flight_file_t;
#define FLIGHT_FOPEN(path, mode)        ((flight_file_t *)0)
#define FLIGHT_FCLOSE(fp)               ((void)(fp))
#define FLIGHT_FREAD(buf, sz, n, fp)    ((void)(buf),(void)(fp),(size_t)0U)
#define FLIGHT_FWRITE(buf, sz, n, fp)   ((void)(buf),(void)(fp),(size_t)((sz)*(n)))
#endif

/* ------------------------------------------------------------------------
 * Safe-halt hook
 * ------------------------------------------------------------------------ */
#ifndef FLIGHT_SAFE_HALT
#ifdef FLIGHT_HOST_IO
static void flight_safe_halt_impl(int code) { exit(code); }
#else
static void flight_safe_halt_impl(int code)
{
    (void)code;
    /* Watchdog reset will fire here on real hardware. */
    for (;;) {
        /* spin */
    }
}
#endif
#define FLIGHT_SAFE_HALT(code) flight_safe_halt_impl((int)(code))
#endif

/* ------------------------------------------------------------------------
 * libc-overriding macros (workload-facing).
 *
 * In flight mode these route the workload's malloc/printf calls through
 * the shim's static pool and no-op log stub.  In host-IO mode we leave
 * printf / fprintf / puts UN-redirected so the workload's output reaches
 * gem5 stdout for validation; malloc/free still go through the shim
 * functions but those functions themselves pass through to libc (above),
 * so the net effect is identical to the original libc-only behavior.
 * ------------------------------------------------------------------------ */
#define FLIGHT_LOG(...) flight_log_stub(__VA_ARGS__)
#define malloc(sz)     flight_malloc_impl((size_t)(sz))
#define calloc(n, sz)  flight_calloc_impl((size_t)(n), (size_t)(sz))
#define realloc(p, sz) flight_realloc_impl((p), (size_t)(sz))
#define free(p)        flight_free_impl((p))

#ifndef FLIGHT_HOST_IO
#define printf(...)            FLIGHT_LOG(__VA_ARGS__)
#define fprintf(stream, ...)   FLIGHT_LOG(__VA_ARGS__)
#define puts(s)                FLIGHT_LOG("%s", (s))
#endif

#endif
