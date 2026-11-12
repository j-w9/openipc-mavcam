# Cross-compile with e.g.  make CC=arm-openipc-linux-gnueabihf-gcc
CC      ?= cc
CFLAGS  ?= -O2 -g
override CFLAGS += -D_FILE_OFFSET_BITS=64 -std=gnu99 -Wall -Wextra -Wno-address-of-packed-member -Ithird_party
override LDLIBS += -lm -lpthread

SRCS := src/mavcam.c src/params.c src/ftp.c src/worker.c src/link.c src/streamer.c src/http.c
OBJS := $(SRCS:.c=.o)

mavcam: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

src/%.o: src/%.c $(wildcard src/*.h)
	$(CC) $(CFLAGS) -c -o $@ $<

tests/test_link: tests/test_link.c src/link.c src/link.h
	$(CC) $(CFLAGS) -o $@ tests/test_link.c src/link.c

test: mavcam tests/test_link
	tests/test_link
	python3 tests/test_mavcam.py
	python3 tests/test_features.py
	python3 tests/test_serial.py
	python3 tests/test_lossy.py

clean:
	rm -f mavcam $(OBJS) tests/test_link

.PHONY: test clean
