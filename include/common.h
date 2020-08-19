#ifndef _COMMON_H
#define _COMMON_H

#include <cstdio>
#include <cstdlib>
#include <vector>

#define DEBUG        0
#define TIMETRACE    0
#define GUEST_OUTPUT 0
#define SINGLE_RUN   0

// Type used for guest virtual addresses
typedef uint32_t vaddr_t;

// Type used for indexing guest virtual address
typedef vaddr_t vsize_t;

// Structure used for measuring coverage in a single run
// Take advantage of the bitmap, which is nice for checking if an address has
// already been visited but is terrible for hashing, and the vector, which is 
// nice for hashing but is bad for checking if an address has already been
// visited
struct cov_t {
	std::vector<bool> bitmap;
	std::vector<std::pair<vaddr_t, vaddr_t>> vec;
	bool operator==(const cov_t& other) const {
		return vec == other.vec;
	}
};

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