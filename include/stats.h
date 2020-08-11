#ifndef _STATS_H
#define _STATS_H

#include <cstdint>
#include <atomic>
#include <x86intrin.h> // _rdtsc()

// Type returned by _rdtsc() for measuring cpu cycles
typedef unsigned long long cycle_t;

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
	std::atomic_flag lock = ATOMIC_FLAG_INIT;

	void update(const Stats& stats){
		while (lock.test_and_set());
		cases             += stats.cases;
		instr             += stats.instr;
		total_cycles      += stats.total_cycles;
		reset_cycles      += stats.reset_cycles;
		run_cycles        += stats.run_cycles;
		run_inst_cycles   += stats.run_inst_cycles;
		inst_handl_cycles += stats.inst_handl_cycles;
		fetch_inst_cycles += stats.fetch_inst_cycles;
		jump_cycles       += stats.jump_cycles;
		bp_cycles         += stats.bp_cycles;
		timeout_cycles    += stats.timeout_cycles;
		lock.clear();
	}
};

#if TIMETRACE == 0
#undef _rdtsc
#define _rdtsc() (0)
#endif

#endif