/*
 * ftrace.c - LD_PRELOAD shim that logs libftd2xx traffic.
 *
 * Interposes the D2XX functions libscu3dcontrol.so uses, forwards each call
 * to the real libftd2xx.so and logs arguments, results and every byte
 * written and read. Use it to capture ground truth for the SCU wire
 * protocol from SmarAct's own library:
 *
 *   SCU_FTRACE=/tmp/scu.trace LD_PRELOAD=$PWD/libftrace.so \
 *       scu --lib /opt/smaract/lib/libscu3dcontrol.so status
 *
 * Environment:
 *   SCU_FTRACE          log file (default: stderr); appended to
 *   SCU_FTRACE_FTD2XX   path of the real libftd2xx.so, only needed if it
 *                       cannot be found via the normal symbol lookup
 *   SCU_FTRACE_STATUS   set to 1 to also log FT_GetStatus (noisy)
 *
 * Log lines (one per call; times in seconds since the first call):
 *   <t> W h=<handle> n=<req> -> st=<status> done=<n> | <hex bytes>
 *   <t> R h=<handle> n=<req> -> st=<status> got=<n> | <hex bytes>
 *   <t> <FT_Function> <args> -> st=<status>
 *
 * Read-only: it never alters or injects traffic.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef void *FT_HANDLE;
typedef unsigned long FT_STATUS;   /* ULONG in the Linux D2XX headers */
typedef unsigned int DWORD;

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static FILE *out;
static void *real_lib;
static int log_status;
static struct timespec t0;

static void init_log(void)
{
    if (out)
        return;
    const char *path = getenv("SCU_FTRACE");
    out = path ? fopen(path, "a") : NULL;
    if (!out)
        out = stderr;
    setvbuf(out, NULL, _IOLBF, 0);
    const char *s = getenv("SCU_FTRACE_STATUS");
    log_status = s && *s == '1';
    clock_gettime(CLOCK_MONOTONIC, &t0);
    fprintf(out, "# scu ftrace start\n");
}

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) * 1e-9;
}

static void *real(const char *name)
{
    void *f = dlsym(RTLD_NEXT, name);
    if (f)
        return f;
    if (!real_lib) {
        const char *p = getenv("SCU_FTRACE_FTD2XX");
        real_lib = dlopen(p ? p : "libftd2xx.so", RTLD_NOW | RTLD_GLOBAL);
    }
    f = real_lib ? dlsym(real_lib, name) : NULL;
    if (!f) {
        fprintf(stderr, "ftrace: cannot resolve %s (set SCU_FTRACE_FTD2XX)\n", name);
        abort();
    }
    return f;
}

#define REAL(name, type) \
    static __typeof__(type) fn; \
    if (!fn) fn = (type)real(#name)

static void logf_(const char *fmt, ...)
{
    va_list ap;
    pthread_mutex_lock(&lock);
    init_log();
    fprintf(out, "%.6f ", now());
    va_start(ap, fmt);
    vfprintf(out, fmt, ap);
    va_end(ap);
    fputc('\n', out);
    pthread_mutex_unlock(&lock);
}

static void log_bytes(char dir, FT_HANDLE h, DWORD req, FT_STATUS st,
                      DWORD n, const unsigned char *b)
{
    pthread_mutex_lock(&lock);
    init_log();
    fprintf(out, "%.6f %c h=%p n=%u -> st=%lu %s=%u |", now(), dir, h, req, st,
            dir == 'W' ? "done" : "got", n);
    for (DWORD i = 0; i < n; i++)
        fprintf(out, " %02x", b[i]);
    fputc('\n', out);
    pthread_mutex_unlock(&lock);
}

FT_STATUS FT_Write(FT_HANDLE h, void *buf, DWORD n, DWORD *done)
{
    REAL(FT_Write, FT_STATUS (*)(FT_HANDLE, void *, DWORD, DWORD *));
    DWORD d = 0;
    FT_STATUS st = fn(h, buf, n, &d);
    if (done)
        *done = d;
    /* log what was offered if the write failed outright */
    log_bytes('W', h, n, st, st ? n : d, buf);
    return st;
}

FT_STATUS FT_Read(FT_HANDLE h, void *buf, DWORD n, DWORD *got)
{
    REAL(FT_Read, FT_STATUS (*)(FT_HANDLE, void *, DWORD, DWORD *));
    DWORD g = 0;
    FT_STATUS st = fn(h, buf, n, &g);
    if (got)
        *got = g;
    log_bytes('R', h, n, st, g, buf);
    return st;
}

FT_STATUS FT_Open(int dev, FT_HANDLE *ph)
{
    REAL(FT_Open, FT_STATUS (*)(int, FT_HANDLE *));
    FT_STATUS st = fn(dev, ph);
    logf_("FT_Open dev=%d -> st=%lu h=%p", dev, st, ph ? *ph : NULL);
    return st;
}

FT_STATUS FT_Close(FT_HANDLE h)
{
    REAL(FT_Close, FT_STATUS (*)(FT_HANDLE));
    FT_STATUS st = fn(h);
    logf_("FT_Close h=%p -> st=%lu", h, st);
    return st;
}

FT_STATUS FT_ListDevices(void *a1, void *a2, DWORD flags)
{
    REAL(FT_ListDevices, FT_STATUS (*)(void *, void *, DWORD));
    FT_STATUS st = fn(a1, a2, flags);
    if (flags == 0x80000000u && a1)   /* FT_LIST_NUMBER_ONLY */
        logf_("FT_ListDevices NUMBER_ONLY -> st=%lu count=%u", st, *(DWORD *)a1);
    else
        logf_("FT_ListDevices flags=0x%08x -> st=%lu", flags, st);
    return st;
}

FT_STATUS FT_GetDeviceInfo(FT_HANDLE h, DWORD *type, DWORD *id, char *serial,
                           char *desc, void *dummy)
{
    REAL(FT_GetDeviceInfo, FT_STATUS (*)(FT_HANDLE, DWORD *, DWORD *, char *, char *, void *));
    FT_STATUS st = fn(h, type, id, serial, desc, dummy);
    logf_("FT_GetDeviceInfo h=%p -> st=%lu type=%u id=%04x:%04x serial='%s' desc='%s'",
          h, st, type ? *type : 0, id ? *id >> 16 : 0, id ? *id & 0xffff : 0,
          serial ? serial : "", desc ? desc : "");
    return st;
}

FT_STATUS FT_SetDivisor(FT_HANDLE h, unsigned short div)
{
    REAL(FT_SetDivisor, FT_STATUS (*)(FT_HANDLE, unsigned short));
    FT_STATUS st = fn(h, div);
    logf_("FT_SetDivisor h=%p div=%u (%.0f baud) -> st=%lu", h, div,
          div ? 3000000.0 / div : 0.0, st);
    return st;
}

FT_STATUS FT_SetDataCharacteristics(FT_HANDLE h, unsigned char bits,
                                    unsigned char stop, unsigned char parity)
{
    REAL(FT_SetDataCharacteristics, FT_STATUS (*)(FT_HANDLE, unsigned char, unsigned char, unsigned char));
    FT_STATUS st = fn(h, bits, stop, parity);
    logf_("FT_SetDataCharacteristics h=%p bits=%u stop=%u parity=%u -> st=%lu",
          h, bits, stop, parity, st);
    return st;
}

FT_STATUS FT_SetFlowControl(FT_HANDLE h, unsigned short flow,
                            unsigned char xon, unsigned char xoff)
{
    REAL(FT_SetFlowControl, FT_STATUS (*)(FT_HANDLE, unsigned short, unsigned char, unsigned char));
    FT_STATUS st = fn(h, flow, xon, xoff);
    logf_("FT_SetFlowControl h=%p flow=0x%04x -> st=%lu", h, flow, st);
    return st;
}

FT_STATUS FT_SetTimeouts(FT_HANDLE h, unsigned long rd, unsigned long wr)
{
    REAL(FT_SetTimeouts, FT_STATUS (*)(FT_HANDLE, unsigned long, unsigned long));
    FT_STATUS st = fn(h, rd, wr);
    logf_("FT_SetTimeouts h=%p read=%lu write=%lu -> st=%lu", h, rd, wr, st);
    return st;
}

FT_STATUS FT_SetLatencyTimer(FT_HANDLE h, unsigned char ms)
{
    REAL(FT_SetLatencyTimer, FT_STATUS (*)(FT_HANDLE, unsigned char));
    FT_STATUS st = fn(h, ms);
    logf_("FT_SetLatencyTimer h=%p ms=%u -> st=%lu", h, ms, st);
    return st;
}

FT_STATUS FT_Purge(FT_HANDLE h, unsigned long mask)
{
    REAL(FT_Purge, FT_STATUS (*)(FT_HANDLE, unsigned long));
    FT_STATUS st = fn(h, mask);
    logf_("FT_Purge h=%p mask=%lu -> st=%lu", h, mask, st);
    return st;
}

FT_STATUS FT_CyclePort(FT_HANDLE h)
{
    REAL(FT_CyclePort, FT_STATUS (*)(FT_HANDLE));
    FT_STATUS st = fn(h);
    logf_("FT_CyclePort h=%p -> st=%lu", h, st);
    return st;
}

FT_STATUS FT_GetStatus(FT_HANDLE h, DWORD *rx, DWORD *tx, DWORD *ev)
{
    REAL(FT_GetStatus, FT_STATUS (*)(FT_HANDLE, DWORD *, DWORD *, DWORD *));
    FT_STATUS st = fn(h, rx, tx, ev);
    if (log_status)
        logf_("FT_GetStatus h=%p -> st=%lu rx=%u", h, st, rx ? *rx : 0);
    return st;
}
