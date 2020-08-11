#include <iostream>
#include <thread>
#include <atomic>
#include "common.h"
#include "emulator.h"
#include "corpus.h"
#include "stats.h"

using namespace std;

/* TODO
Adapt the elf parser to my code
Faster breakpoints
Measure perf
Dividir emulator.cpp en varios archivos: instr.cpp y syscalls.cpp
*/

void worker(int id, Emulator runner, const Emulator& parent, Corpus& corpus,
            Stats& stats)
{
	cycle_t cycles_init, cycles;
	while (true){
		Stats local_stats;
		cycles_init = __rdtsc(); // total_cycles, __rdtsc() to avoid noping macro

		// Run some time saving stats in local_stats
		while (__rdtsc() - cycles_init < 5000000){
			cycles = _rdtsc(); // run_cycles
			try {
				const string& input = corpus.get_new_input(id);
				runner.run(input, local_stats);
			} catch (const Fault& f) {
				cout << "[PC: 0x" << hex << runner.get_pc() << "] " << f << endl;
				// future: Save input as a crash
			} catch (const RunTimeout&) {
				cout << "TIMEOUT" << endl;
			}
			local_stats.run_cycles += _rdtsc() - cycles;
			local_stats.cases += 1;

			cycles = _rdtsc(); // reset_cycles
			runner.reset(parent);
			local_stats.reset_cycles += _rdtsc() - cycles;
		}
		local_stats.total_cycles = __rdtsc() - cycles_init;

		// Update global stats
		stats.update(local_stats);
	}
}

void print_stats(int nthreads, Stats& stats){
	uint32_t elapsed = 0;
	double minstrps, fcps, reset_time, run_time, run_inst_time, inst_handl_time,
	       fetch_inst_time, jump_time, bp_time, timeout_time;
	uint64_t cases;
	const char* s = (nthreads > 1 ? "THREADS" : "THREAD");
	while (true){
		this_thread::sleep_for(chrono::seconds(1));
		elapsed++;
		cases           = stats.cases;
		minstrps        = (double)stats.instr / (elapsed * 1000000);
		fcps            = (double)cases / elapsed;
		reset_time      = (double)stats.reset_cycles / stats.total_cycles;
		run_time        = (double)stats.run_cycles / stats.total_cycles;
		run_inst_time   = (double)stats.run_inst_cycles / stats.total_cycles;
		inst_handl_time = (double)stats.inst_handl_cycles / stats.total_cycles;
		fetch_inst_time = (double)stats.fetch_inst_cycles / stats.total_cycles;
		jump_time       = (double)stats.jump_cycles / stats.total_cycles;
		bp_time         = (double)stats.bp_cycles / stats.total_cycles;
		timeout_time    = (double)stats.timeout_cycles / stats.total_cycles;
		printf("[%d %s] cases: %lu, minstr/s: %.3f, fcps: %.3f, "
		       "reset: %.3f, run: %.3f, run_inst: %.3f, inst_handl: %.3f, "
		       "fetch_inst: %.3f, jump: %.3f, bp: %.3f, timeout: %.3f\n",
		        nthreads, s, cases, minstrps, fcps, reset_time, run_time,
			    run_inst_time, inst_handl_time, fetch_inst_time, jump_time,
			    bp_time, timeout_time);
	}
}

int main(){
	const int num_threads = 8;
	cout << "Threads: " << num_threads << endl;
	Stats stats;
	Corpus corpus(num_threads, "../corpus");
	Emulator emu(
		8 * 1024 * 1024,         // memory
		"../test_bins/xxd/xxd",  // path to elf
		{"xxd", "input_file"}    // argv
	);

	// Run until open before forking
	uint64_t insts = emu.run_until(0x429e6c);
	cout << "Executed " << insts << " instructions before forking" << endl;

	// Create worker threads
	vector<thread> threads;
	for (int i = 0; i < num_threads; i++)
		threads.push_back(thread(worker, i, emu.fork(), ref(emu), ref(corpus),
		                         ref(stats)));
	
	// Create stats thread
	threads.push_back(thread(print_stats, num_threads, ref(stats)));

	// Wait for every thread to finish
	for (thread& t : threads)
		t.join();
}