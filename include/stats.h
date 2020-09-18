#ifndef _STATS_H
#define _STATS_H

#include <cstdint>
#include <atomic>
#include <x86intrin.h> // _rdtsc()

/* Timetracing:
 *   - 0 means no timetracing
 *   - 1 means timetracing of things that happen once per fuzz case:
 *       coverage report, input mutating, run.
 *   - 2 means timetracing of things that happen a lot of times per fuzz case.
 *       In the JIT that's jit cache lookup, vm exit handler and vm time.
 *       In the interpreter that's breakpoint handling, instruction fetch, jump
 *       time and instruction running.
 */
#define TIMETRACE 1

// Type returned by _rdtsc() for measuring cpu cycles
typedef unsigned long long cycle_t;

// STATS
struct Stats {
	uint64_t cases {0};
	uint64_t instr {0};
	uint64_t cov {0};
	uint64_t crashes {0};
	uint64_t timeouts {0};
	uint64_t ooms {0};
	cycle_t  total_cycles {0};
	cycle_t  reset_cycles {0};
	cycle_t  run_cycles {0};
	cycle_t  cov_cycles {0};
	cycle_t  mut_cycles {0};
	cycle_t  inst_handl_cycles {0};
	cycle_t  fetch_inst_cycles {0};
	cycle_t  jump_cycles {0};
	cycle_t  bp_cycles {0};
	cycle_t  jit_cache_cycles {0};
	cycle_t  vm_exit_cycles {0};
	cycle_t  vm_cycles {0};
	std::atomic_flag lock = ATOMIC_FLAG_INIT;

	void update(const Stats& stats){
		while (lock.test_and_set());
		cases             += stats.cases;
		instr             += stats.instr;
		cov               += stats.cov;
		crashes           += stats.crashes;
		timeouts          += stats.timeouts;
		ooms              += stats.ooms;
		total_cycles      += stats.total_cycles;
		reset_cycles      += stats.reset_cycles;
		run_cycles        += stats.run_cycles;
		cov_cycles        += stats.cov_cycles;
		mut_cycles        += stats.mut_cycles;
		inst_handl_cycles += stats.inst_handl_cycles;
		fetch_inst_cycles += stats.fetch_inst_cycles;
		jump_cycles       += stats.jump_cycles;
		bp_cycles         += stats.bp_cycles;
		jit_cache_cycles  += stats.jit_cache_cycles;
		vm_exit_cycles    += stats.vm_exit_cycles;
		vm_cycles         += stats.vm_cycles;
		lock.clear();
	}
};

#if TIMETRACE == 0
#define rdtsc1() (0)
#define rdtsc2() (0)

#elif TIMETRACE == 1
#define rdtsc1() _rdtsc()
#define rdtsc2() (0)

#elif TIMETRACE >= 2
#define rdtsc1() _rdtsc()
#define rdtsc2() _rdtsc()
#endif


#endif