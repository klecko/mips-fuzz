#ifndef _COMMON_H
#define _COMMON_H

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <atomic>
#include <x86intrin.h> // _rdtsc()

#define DEBUG 0
#define TIMETRACE 0

// Type used for guest virtual addresses
typedef uint32_t vaddr_t;

// Type used for indexing guest virtual address
typedef vaddr_t vsize_t;

// Type returned by _rdtsc() for measuring cpu cycles
typedef unsigned long long cycle_t;


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

// STATS
struct Stats {
	uint64_t cases {0};
	uint64_t instr {0};
	cycle_t  total_cycles {0};
	cycle_t  reset_cycles {0};
	cycle_t  run_cycles {0};
	cycle_t  run_inst_cycles {0};
	cycle_t  inst_handl_cycles {0};
	cycle_t  fetch_inst_cycles {0};
	cycle_t  jump_cycles {0};
	cycle_t  bp_cycles {0};
	cycle_t  timeout_cycles {0};
};

extern Stats stats;

#if TIMETRACE == 0
#undef _rdtsc
#define _rdtsc() (0)
#endif

#endif