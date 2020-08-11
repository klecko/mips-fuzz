#include <iostream>
#include <thread>
#include <atomic>
#include "common.h"
#include "emulator.h"
#include "corpus.h"

using namespace std;

/* TODO
Adapt the elf parser to my code
Faster breakpoints
Measure perf
Dividir emulator.cpp en varios archivos: instr.cpp y syscalls.cpp
*/

Stats stats;

void worker(Emulator runner, const Emulator& parent, Corpus& corpus){
	cycle_t total_cycles, cycles;
	for (int i = 0; i < 10000; i++){
		total_cycles = _rdtsc();
		cycles = _rdtsc();
		try {
			const string& input = corpus.get_new_input();
			runner.run(input);
		} catch (const Fault& f) {
			cout << "[PC: 0x" << hex << runner.get_pc() << "] " << f << endl;
			// future: Save input as a crash
		} catch (const RunTimeout&) {
			cout << "TIMEOUT" << endl;
		}
		stats.cases += 1;
		stats.run_cycles += _rdtsc() - cycles;

		cycles = _rdtsc();
		runner.reset(parent);
		stats.reset_cycles += _rdtsc() - cycles;

		stats.total_cycles += _rdtsc() - total_cycles;
	}
}

void print_stats(){
	uint32_t elapsed = 0;
	double minstrps, fcps, reset_time, run_time, run_inst_time, inst_handl_time,
	       fetch_inst_time, jump_time, bp_time, timeout_time;
	uint64_t cases;
	while (true){
		this_thread::sleep_for(chrono::seconds(1));
		elapsed++;
		/* cases          = stats.cases.load();
		fcps            = (double)cases/elapsed;
		reset_time      = (double)stats.reset_cycles.load() / stats.total_cycles;
		bp_time         = (double)stats.bp_cycles.load() / stats.total_cycles;
		run_time        = (double)stats.run_cycles.load() / stats.total_cycles;
		input_time      = (double)stats.input_cycles.load() / stats.total_cycles; */
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
		printf("[STATS] cases: %lu, minstr/s: %.3f, fcps: %.3f, reset: %.3f, run: %.3f, run_inst: %.3f, inst_handl: %.3f, fetch_inst: %.3f, jump: %.3f, bp: %.3f, timeout: %.3f\n", 
		       cases, minstrps, fcps, reset_time, run_time, run_inst_time, inst_handl_time, fetch_inst_time, jump_time, bp_time, timeout_time);
	}
}

int main(){
	srand(time(NULL));
	Corpus corpus("../corpus");
	Emulator emu(8 * 1024 * 1024, "../test_bins/xxd/xxd",
	             {"xxd", "input_file"});
	// future: run emu til main

	//Emulator runner = emu.fork();
	thread t(worker, emu.fork(), ref(emu), ref(corpus));

	thread stats_t(print_stats);
	t.join();
	stats_t.join();
	//worker(emu.fork(), emu, corpus);
}