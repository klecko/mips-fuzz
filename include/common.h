#ifndef _COMMON_H
#define _COMMON_H

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <assert.h>

#define DEBUG 0

// Type used for guest virtual addresses
typedef uint32_t vaddr_t;

// Type used for indexing guest virtual address
typedef vaddr_t vsize_t;

// Data structure used for measuring coverage
typedef std::vector<uint8_t> cov_t;

#if DEBUG == 1
#define dbgprintf(...) printf(__VA_ARGS__)
#else
#define dbgprintf(...) ((void)0)
#endif

#define die(...) do { \
	fprintf(stderr, __VA_ARGS__); \
	exit(EXIT_FAILURE);           \
} while(0)

#endif
