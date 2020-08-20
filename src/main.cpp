#include <iostream>
#include <thread>
#include <cstring>
#include <sched.h>
#include "common.h"
#include "emulator.h"
#include "corpus.h"
#include "stats.h"

using namespace std;

/* TODO
Adapt the elf parser to my code
*/


void print_stats(Stats& stats, const Corpus& corpus){
	// Each second get data from stats and print it
	uint32_t elapsed = 0;
	double minstrps, fcps, reset_time, run_time, run_inst_time, inst_handl_time,
	       fetch_inst_time, jump_time, bp_time, timeout_time, corpus_sz;
	uint64_t cases, uniq_crashes, crashes, timeouts, cov, corpus_n;
	while (true){
		this_thread::sleep_for(chrono::seconds(1));
		elapsed++;
		cases           = stats.cases;
		minstrps        = (double)stats.instr / (elapsed * 1000000);
		fcps            = (double)cases / elapsed;
		cov             = corpus.cov_size();
		corpus_n        = corpus.size();
		corpus_sz       = (double)corpus.memsize() / 1024;
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
		       "corpus: %lu/%.3fKB, uniq crashes: %lu, crashes: %lu, "
			   "timeouts: %lu\n",
			   elapsed, cases, minstrps, fcps, cov, corpus_n, corpus_sz,
			   uniq_crashes, crashes, timeouts);

		if (!TIMETRACE) continue;
		printf("\treset: %.3f, run: %.3f, run_inst: %.3f, inst_handl: %.3f, "
				"fetch_inst: %.3f, jump: %.3f, bp: %.3f, timeout: %.3f\n",
				reset_time, run_time, run_inst_time, inst_handl_time,
				fetch_inst_time, jump_time, bp_time, timeout_time);
	}
}

void worker(int id, Emulator runner, const Emulator& parent, Corpus& corpus,
            Stats& stats)
{
	Rng rng;
	cycle_t cycles_init, cycles;
	cov_t cov = { vector<bool>(runner.memsize(), 0) };
	uint32_t hash;
	while (true){
		Stats local_stats;
		cycles_init = _rdtsc(); // total_cycles, _rdtsc() to avoid noping macro

		// Run some time saving stats in local_stats
		while (_rdtsc() - cycles_init < 50000000){
			// Get new mutated input
			const string& input = corpus.get_new_input(id, rng);

			// Clear coverage
			for (const auto& jump : cov.vec){
				hash = branch_hash(jump.first, jump.second) % cov.bitmap.size();
				cov.bitmap[hash] = 0;
			}
			cov.vec.clear();

			cycles = rdtsc(); // run_cycles
			try {
				runner.run(input, cov, local_stats);
			} catch (const Fault& f) {
				// Crash. Corpus will handle it if it is a new one
				local_stats.crashes++;
				corpus.report_crash(id, runner.get_prev_pc(), f);

			} catch (const RunTimeout&) {
				// Timeout
				cout << "TIMEOUT" << endl;
				local_stats.timeouts++;
			}
			local_stats.run_cycles += rdtsc() - cycles;

			local_stats.cases++;
			corpus.report_cov(id, cov);

			if (SINGLE_RUN)
				die("end\n");

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

int main(){
	const int num_threads = (DEBUG ? 1 : thread::hardware_concurrency());
	cout << "Threads: " << num_threads << endl;

	// Create crash folder
	struct stat st;
	if (stat("./crashes", &st) == -1){
		// Assume folder doesn't exist. Create it
		if (mkdir("./crashes", S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH) == -1){
			cerr << "Creating crashes folder: " << strerror(errno) << endl;
			return -1;
		}
	} else if (!(st.st_mode & S_IFDIR)) {
		// It exists. Check it is a folder
		cerr << "./crashes is not a folder" << endl;
		return -1;
	}

	// Create shared objects
	Stats stats;
	Corpus corpus(num_threads, "../corpus");
	Emulator emu(
		8 * 1024 * 1024,                // memory
		"../test_bins/readelf",         // path to elf
		{"readelf", "-l", "input_file"} // argv
	);

	// Run until open before forking
	// test:    0x00423e8c | 0x41d6e4
	// xxd:     0x00429e6c
	// readelf: 0x004c081c
	try {
		uint64_t insts = emu.run_until(0x004c081c);
		cout << "Executed " << insts << " instructions before forking" << endl;
	} catch (const Fault& f) {
		cout << "Unexpected fault runing before forking" << endl;
		cout << "[PC: 0x" << hex << emu.get_prev_pc() << "] " << f << endl;
		cout << emu << endl;
		return -1;
	}

	// Create worker threads and assign each to one core
	cpu_set_t cpu;
	vector<thread> threads;
	for (int i = 0; i < num_threads; i++){
		thread t = thread(worker, i, emu.fork(), ref(emu), ref(corpus),
		                  ref(stats));
		CPU_ZERO(&cpu);
		CPU_SET(i, &cpu);
		if (pthread_setaffinity_np(t.native_handle(), sizeof(cpu), &cpu) != 0)
			perror("pthread_setaffinity_np");
		threads.push_back(move(t));
	}

	// Create stats thread
	threads.push_back(thread(print_stats, ref(stats), ref(corpus)));

	// Wait for every thread to finish
	for (thread& t : threads)
		t.join();
}
