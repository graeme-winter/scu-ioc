CFLAGS ?= -O2 -Wall -Wextra
all: libftrace.so
libftrace.so: ftrace.c
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $< -ldl -lpthread
# Fake D2XX library that forwards to a serial port / pty (testing only).
libfakeftd2xx.so: fakeftd2xx.c
	$(CC) $(CFLAGS) -shared -fPIC -Wl,-soname,libftd2xx.so -o $@ $<
clean:
	rm -f libftrace.so libfakeftd2xx.so
.PHONY: all clean
