#include <iostream>
#include <thread>
#include "common.h"
#include "emulator.h"
#include "corpus.h"
#include "stats.h"

using namespace std;

/* TODO
Adapt the elf parser to my code
*/

void worker(int id, Emulator runner, const Emulator& parent, Corpus& corpus,
            Stats& stats)
{
	Rng rng;
	cycle_t cycles_init, cycles;
	cov_t cov  = { vector<bool>(runner.memsize(), 0) };
	while (true){
		Stats local_stats;
		cycles_init = _rdtsc(); // total_cycles, _rdtsc() to avoid noping macro

		// Run some time saving stats in local_stats
		while (_rdtsc() - cycles_init < 50000000){
			// Get new mutated input
			const string& input = corpus.get_new_input(id, rng);

			// Clear coverage
			for (const vaddr_t& addr : cov.vec)
				cov.bitmap[addr] = 0;
			cov.vec.clear();

			cycles = rdtsc(); // run_cycles
			try {
				runner.run(input, cov, local_stats);
			} catch (const Fault& f) {
				// Crash. Corpus will handle it if it is a new one
				local_stats.crashes++;
				corpus.report_crash(runner.get_pc(), f);

			} catch (const RunTimeout&) {
				// Timeout
				cout << "TIMEOUT" << endl;
				local_stats.timeouts++;
			}
			if (SINGLE_RUN)
				die("end\n");
			local_stats.run_cycles += rdtsc() - cycles;

			local_stats.cases++;
			corpus.report_cov(id, cov);

			cycles = rdtsc(); // reset_cycles
			runner.reset(parent);
			local_stats.reset_cycles += rdtsc() - cycles;
		}
		//  _rdtsc() to avoid noping macro
		local_stats.total_cycles = _rdtsc() - cycles_init;

		// Update global stats
		stats.update(local_stats);
	}
}

void print_stats(Stats& stats, const Corpus& corpus){
	// Each second get data from stats and print it
	uint32_t elapsed = 0;
	double minstrps, fcps, reset_time, run_time, run_inst_time, inst_handl_time,
	       fetch_inst_time, jump_time, bp_time, timeout_time;
	uint64_t cases, uniq_crashes, crashes, timeouts, cov;
	while (true){
		this_thread::sleep_for(chrono::seconds(1));
		elapsed++;
		cases           = stats.cases;
		minstrps        = (double)stats.instr / (elapsed * 1000000);
		fcps            = (double)cases / elapsed;
		cov             = corpus.cov_size();
		uniq_crashes    = corpus.uniq_crashes_size();
		crashes         = stats.crashes;
		timeouts        = stats.timeouts;
		reset_time      = (double)stats.reset_cycles / stats.total_cycles;
		run_time        = (double)stats.run_cycles / stats.total_cycles;
		run_inst_time   = (double)stats.run_inst_cycles / stats.total_cycles;
		inst_handl_time = (double)stats.inst_handl_cycles / stats.total_cycles;
		fetch_inst_time = (double)stats.fetch_inst_cycles / stats.total_cycles;
		jump_time       = (double)stats.jump_cycles / stats.total_cycles;
		bp_time         = (double)stats.bp_cycles / stats.total_cycles;
		timeout_time    = (double)stats.timeout_cycles / stats.total_cycles;
		printf("[%u secs] cases: %lu, minstr/s: %.3f, fcps: %.3f, cov: %lu, "
		       "uniq crashes: %lu, crashes: %lu, timeouts: %lu\n",
			   elapsed, cases, minstrps, fcps, cov, uniq_crashes, crashes,
			   timeouts);

		if (!TIMETRACE) continue;
		printf("\treset: %.3f, run: %.3f, run_inst: %.3f, inst_handl: %.3f, "
				"fetch_inst: %.3f, jump: %.3f, bp: %.3f, timeout: %.3f\n",
				reset_time, run_time, run_inst_time, inst_handl_time,
				fetch_inst_time, jump_time, bp_time, timeout_time);
	}
}

int main(){
	//srand(time(NULL));
	const int num_threads = 8;
	cout << "Threads: " << num_threads << endl;
	Stats stats;
	Corpus corpus(num_threads, "../corpus");
	Emulator emu(
		8 * 1024 * 1024,           // memory
		"../test_bins/test/test",  // path to elf
		{"test", "input_file"}     // argv
	);

	// Run until open before forking
	try {
		uint64_t insts = emu.run_until(0x423e8c);
		cout << "Executed " << insts << " instructions before forking" << endl;
	} catch (const Fault& f) {
		cout << "Unexpected fault runing before forking" << endl;
		cout << "[PC: 0x" << hex << emu.get_pc() << "] " << f << endl;
		return -1;
	}


	// Create worker threads
	vector<thread> threads;
	for (int i = 0; i < num_threads; i++)
		threads.push_back(thread(worker, i, emu.fork(), ref(emu), ref(corpus),
		                         ref(stats)));
	
	// Create stats thread
	threads.push_back(thread(print_stats, ref(stats), ref(corpus)));

	// Wait for every thread to finish
	for (thread& t : threads)
		t.join();
}