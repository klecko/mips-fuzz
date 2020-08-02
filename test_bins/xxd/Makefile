# The most simplistic Makefile

CFLAGS += -m32 -static

xxd: xxd.c
	$(CC) $(CFLAGS) $(LDFLAGS) -DUNIX -o xxd xxd.c

clean:
	rm -f xxd xxd.o
