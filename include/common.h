#ifndef _COMMON_H
#define _COMMON_H

#include <cstdio>
#include <cstdlib>

// Type used for guest virtual addresses
typedef uint32_t vaddr_t;

// Type used for indexing guest virtual address
typedef vaddr_t vsize_t;

#define DEBUG 0

#if DEBUG == 1
#define dbgprintf(...) printf(__VA_ARGS__)
#else
#define dbgprintf(...) ((void)0)
#endif

template <class... Args>
void die(const char* fmt, Args... args){
	fprintf(stderr, fmt, args...);
	exit(EXIT_FAILURE);
}

#endif