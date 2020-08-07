# The most simplistic Makefile

CC = mipsel-linux-gnu-gcc-9
CFLAGS += -static

xxd: xxd.c
	$(CC) $(CFLAGS) $(LDFLAGS) -DUNIX -o xxd xxd.c

clean:
	rm -f xxd xxd.o
