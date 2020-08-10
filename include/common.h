#ifndef _COMMON_H
#define _COMMON_H

#include <cstdio>
#include <cstdlib>

// Type used for guest virtual addresses
typedef uint32_t vaddr_t;

// Type used for indexing guest virtual address
typedef vaddr_t vsize_t;

/* #define die(...)                       \
	do {                               \
		fprintf(stderr, __VA_ARGS__);  \
		exit(EXIT_FAILURE);            \
	} while (0) */
template <class... Args>
void die(const char* fmt, Args... args){
	fprintf(stderr, fmt, args...);
	exit(EXIT_FAILURE);
}

#endif