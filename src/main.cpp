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
	double minstrps, fcps, reset_time, run_time, cov_time, inst_handl_time,
	       fetch_inst_time, jump_time, bp_time, corpus_sz,
	       jit_cache_time, vm_exit_time;
	uint64_t cases, uniq_crashes, crashes, timeouts, cov, corpus_n;
	while (true){
		this_thread::sleep_for(chrono::seconds(1));
		elapsed++;
		cases           = stats.cases;
		minstrps        = (double)stats.instr / ((uint64_t)elapsed * 1000000);
		fcps            = (double)cases / elapsed;
		cov             = corpus.get_cov();
		corpus_n        = corpus.size();
		corpus_sz       = (double)corpus.memsize() / 1024;
		uniq_crashes    = corpus.get_uniq_crashes_size();
		crashes         = stats.crashes;
		timeouts        = stats.timeouts;
		reset_time      = (double)stats.reset_cycles / stats.total_cycles;
		run_time        = (double)stats.run_cycles / stats.total_cycles;
		cov_time        = (double)stats.cov_cycles / stats.total_cycles;
		inst_handl_time = (double)stats.inst_handl_cycles / stats.total_cycles;
		fetch_inst_time = (double)stats.fetch_inst_cycles / stats.total_cycles;
		jump_time       = (double)stats.jump_cycles / stats.total_cycles;
		bp_time         = (double)stats.bp_cycles / stats.total_cycles;
		jit_cache_time  = (double)stats.jit_cache_cycles / stats.total_cycles;
		vm_exit_time    = (double)stats.vm_exit_cycles / stats.total_cycles;
		printf("[%u secs] cases: %lu, minstrps: %.3f, fcps: %.3f, cov: %lu, "
		       "corpus: %lu/%.3fKB, uniq crashes: %lu, crashes: %lu, "
		       "timeouts: %lu\n",
		       elapsed, cases, minstrps, fcps, cov, corpus_n, corpus_sz,
		       uniq_crashes, crashes, timeouts);

		if (TIMETRACE >= 1)
			printf("\treset: %.3f, run: %.3f, cov: %.3f\n",
			       reset_time, run_time, cov_time);

		if (TIMETRACE >= 2)
			printf("\tinst_handl: %.3f, fetch_inst: %.3f, jump: %.3f, bp: %.3f\n"
			       "\tjit cache: %.3f, vm exit: %.3f\n",
			       inst_handl_time, fetch_inst_time, jump_time, bp_time,
			       jit_cache_time, vm_exit_time);
	}
}

void worker(int id, Emulator runner, const Emulator& parent, Corpus& corpus,
            JIT::jit_cache_t& jit_cache, Stats& stats)
{
	// Custom RNG: avoids locks and simpler
	Rng rng;

	// Timetracing
	cycle_t cycles_init, cycles;

	// Recorded coverage in current run. It is reported to the corpus after
	// each run
	std::vector<uint8_t> cov;
	size_t covsize = corpus.get_cov_map_size();

	// Main loop
	while (true){
		Stats local_stats;
		cycles_init = _rdtsc(); // total_cycles, _rdtsc() to avoid macros

		// Run some time saving stats in local_stats
		while (_rdtsc() - cycles_init < 50000000){
			// Get new mutated input
			const string& input = corpus.get_new_input(id, rng);

			// Clear coverage
			cycles = rdtsc1(); // cov_cycles1
			cov.assign(covsize, 0);
			local_stats.cov_cycles += rdtsc1() - cycles;

			try {
				//runner.run_interpreter(input, cov, local_stats);
				runner.run_jit(input, cov, jit_cache, local_stats);
			} catch (const Fault& f) {
				// Crash. Corpus will handle it if it is a new one
				local_stats.crashes++;
				corpus.report_crash(id, runner.get_prev_pc(), f);

			} catch (const RunTimeout&) {
				// Timeout
				cout << "TIMEOUT" << endl;
				local_stats.timeouts++;
			}

			local_stats.cases++;

			cycles = rdtsc1(); // cov_cycles2
			corpus.report_cov(id, cov);
			local_stats.cov_cycles += rdtsc1() - cycles;

			cycles = rdtsc1(); // reset_cycles
			runner.reset(parent);
			local_stats.reset_cycles += rdtsc1() - cycles;

			if (SINGLE_RUN)
				die("end\n");
		}
		local_stats.total_cycles = _rdtsc() - cycles_init;

		// Update global stats
		stats.update(local_stats);
	}
}

void create_folder(const char* name){
	// If folder doesn't exist, create it. If it exists, check it is an
	// actual folder
	struct stat st;
	if (stat(name, &st) == -1){
		if (mkdir(name, S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH) == -1)
			die("Creating dir %s: %s\n", name, strerror(errno));
	} else if (!(st.st_mode & S_IFDIR))
		die("%s is not a folder\n", name);
}

int main(){
	const int num_threads = (DEBUG ? 1 : thread::hardware_concurrency());
	cout << "Threads: " << num_threads << endl;

	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	// Create folders
	create_folder("./crashes");
	create_folder("./jitcache");

	// Create shared objects
	Stats stats;
	Corpus corpus(num_threads, "../corpus");
	Emulator emu(
		8 * 1024 * 1024,                // memory
		"../test_bins/readelf",         // path to elf
		{"readelf", "-e", "input_file"} // argv
	);
	JIT::jit_cache_t jit_cache(emu.memsize());

	// Run until open before forking
	// test:    0x00423e8c | 0x41d6e4 | 0x00423e7c
	// xxd:     0x00429e6c
	// readelf: 0x004c081c
	try {
		uint64_t insts = emu.run_until(0x004c081c);//0x00423ec8);
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
		                  ref(jit_cache), ref(stats));
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
