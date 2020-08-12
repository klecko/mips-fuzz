#ifndef _COMMON_H
#define _COMMON_H

#include <cstdio>
#include <cstdlib>

#define DEBUG        0
#define TIMETRACE    0
#define GUEST_OUTPUT 0
#define SINGLE_RUN   0

// Type used for guest virtual addresses
typedef uint32_t vaddr_t;

// Type used for indexing guest virtual address
typedef vaddr_t vsize_t;

#if DEBUG == 1
#define dbgprintf(...) printf(__VA_ARGS__)
#else
#define dbgprintf(...) ((void)0)
#endif

#if GUEST_OUTPUT == 1
#define guestprintf(...) printf(__VA_ARGS__)
#else
#define guestprintf(...) ((void)0)
#endif

template <class... Args>
void die(const char* fmt, Args... args){
	fprintf(stderr, fmt, args...);
	exit(EXIT_FAILURE);
}

#endif