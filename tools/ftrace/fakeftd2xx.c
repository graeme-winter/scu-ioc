/*
 * fakeftd2xx.c - minimal stand-in for FTDI's libftd2xx.so, for testing.
 *
 * Presents one device whose bytes go to a serial port or pty named by
 * SCU_FAKE_FTDI_PORT (e.g. the scu-wire-sim pty). This lets SmarAct's
 * real libscu3dcontrol.so run against a protocol simulator, which is how
 * the wire-protocol map is checked without hardware. Never for production.
 *
 *   SCU_FAKE_FTDI_PORT=/tmp/scu-wire  description from SCU_FAKE_FTDI_DESC
 *   (default "SmarAct SCU-3D"), serial from SCU_FAKE_FTDI_SERIAL.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>

typedef unsigned long FT_STATUS;
typedef unsigned int DWORD;
enum { FT_OK = 0, FT_INVALID_HANDLE = 1, FT_DEVICE_NOT_FOUND = 2,
       FT_DEVICE_NOT_OPENED = 3, FT_IO_ERROR = 4 };

struct dev { int fd; unsigned long rd_ms, wr_ms; };

static const char *env(const char *k, const char *d)
{
    const char *v = getenv(k);
    return v && *v ? v : d;
}

FT_STATUS FT_ListDevices(void *a1, void *a2, DWORD flags)
{
    (void)a2;
    if (flags == 0x80000000u && a1) { *(DWORD *)a1 = getenv("SCU_FAKE_FTDI_PORT") ? 1 : 0; return FT_OK; }
    return FT_OK;
}

FT_STATUS FT_Open(int n, void **ph)
{
    if (n != 0 || !getenv("SCU_FAKE_FTDI_PORT")) return FT_DEVICE_NOT_FOUND;
    int fd = open(getenv("SCU_FAKE_FTDI_PORT"), O_RDWR | O_NOCTTY);
    if (fd < 0) return FT_DEVICE_NOT_OPENED;
    struct termios t;
    if (tcgetattr(fd, &t) == 0) { cfmakeraw(&t); tcsetattr(fd, TCSANOW, &t); }
    struct dev *d = calloc(1, sizeof *d);
    d->fd = fd; d->rd_ms = 1000; d->wr_ms = 1000;
    *ph = d;
    return FT_OK;
}

FT_STATUS FT_Close(void *h)
{
    struct dev *d = h;
    if (!d) return FT_INVALID_HANDLE;
    close(d->fd); free(d);
    return FT_OK;
}

FT_STATUS FT_GetDeviceInfo(void *h, DWORD *type, DWORD *id, char *serial, char *desc, void *x)
{
    (void)h; (void)x;
    if (type) *type = 5;                       /* FT_DEVICE_232R */
    if (id) *id = 0x04036001;
    if (serial) strcpy(serial, env("SCU_FAKE_FTDI_SERIAL", "FAKE0001"));
    if (desc) strcpy(desc, env("SCU_FAKE_FTDI_DESC", "SmarAct SCU-3D"));
    return FT_OK;
}

static long ms_left(struct timespec *end)
{
    struct timespec n;
    clock_gettime(CLOCK_MONOTONIC, &n);
    return (end->tv_sec - n.tv_sec) * 1000 + (end->tv_nsec - n.tv_nsec) / 1000000;
}

FT_STATUS FT_Read(void *h, void *buf, DWORD n, DWORD *got)
{
    struct dev *d = h;
    if (!d) return FT_INVALID_HANDLE;
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &end);
    end.tv_sec += d->rd_ms / 1000; end.tv_nsec += (d->rd_ms % 1000) * 1000000L;
    if (end.tv_nsec >= 1000000000L) { end.tv_sec++; end.tv_nsec -= 1000000000L; }
    DWORD k = 0;
    while (k < n) {
        long left = ms_left(&end);
        if (left <= 0) break;
        struct pollfd p = { d->fd, POLLIN, 0 };
        int r = poll(&p, 1, (int)left);
        if (r <= 0) break;
        ssize_t m = read(d->fd, (char *)buf + k, n - k);
        if (m < 0 && errno == EINTR) continue;
        if (m <= 0) return FT_IO_ERROR;
        k += (DWORD)m;
    }
    *got = k;                                  /* short read = timeout, as D2XX */
    return FT_OK;
}

FT_STATUS FT_Write(void *h, void *buf, DWORD n, DWORD *done)
{
    struct dev *d = h;
    if (!d) return FT_INVALID_HANDLE;
    DWORD k = 0;
    while (k < n) {
        ssize_t m = write(d->fd, (char *)buf + k, n - k);
        if (m < 0 && errno == EINTR) continue;
        if (m <= 0) return FT_IO_ERROR;
        k += (DWORD)m;
    }
    *done = k;
    return FT_OK;
}

FT_STATUS FT_SetTimeouts(void *h, unsigned long rd, unsigned long wr)
{
    struct dev *d = h;
    if (!d) return FT_INVALID_HANDLE;
    d->rd_ms = rd; d->wr_ms = wr;
    return FT_OK;
}

FT_STATUS FT_Purge(void *h, unsigned long mask)
{
    struct dev *d = h;
    if (!d) return FT_INVALID_HANDLE;
    if (mask & 1) tcflush(d->fd, TCIFLUSH);
    return FT_OK;
}

FT_STATUS FT_GetStatus(void *h, DWORD *rx, DWORD *tx, DWORD *ev)
{
    struct dev *d = h;
    int q = 0;
    if (!d) return FT_INVALID_HANDLE;
    ioctl(d->fd, FIONREAD, &q);
    if (rx) *rx = (DWORD)q;
    if (tx) *tx = 0;
    if (ev) *ev = 0;
    return FT_OK;
}

FT_STATUS FT_SetDivisor(void *h, unsigned short v) { (void)h; (void)v; return FT_OK; }
FT_STATUS FT_SetDataCharacteristics(void *h, unsigned char a, unsigned char b, unsigned char c) { (void)h; (void)a; (void)b; (void)c; return FT_OK; }
FT_STATUS FT_SetFlowControl(void *h, unsigned short a, unsigned char b, unsigned char c) { (void)h; (void)a; (void)b; (void)c; return FT_OK; }
FT_STATUS FT_SetLatencyTimer(void *h, unsigned char v) { (void)h; (void)v; return FT_OK; }
FT_STATUS FT_CyclePort(void *h) { (void)h; return FT_OK; }
