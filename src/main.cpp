#include <iostream>
#include <thread>
#include <atomic>
#include <common.h>
#include <emulator.h>
#include <corpus.h>

using namespace std;

/* TODO
Adapt the elf parser to my code
Faster breakpoints
Measure perf
Dividir emulator.cpp en varios archivos: instr.cpp y syscalls.cpp
*/

struct Stats {
	atomic_uint64_t cases {0};
};

void worker(Emulator runner, const Emulator& parent, Corpus& corpus, 
            Stats& stats)
{
	for (int i = 0; i < 10000; i++){
		try {
			const string& input = corpus.get_new_input();
			runner.run(input);
		} catch (const Fault& f) {
			cout << "[PC: 0x" << hex << runner.get_pc() << "] " << f << endl;
			// future: Save input as a crash
		}
		stats.cases += 1;
		runner.reset(parent);
	}
}

void print_stats(Stats& stats){
	while (true){
		printf("[STATS] %lu cases\n", stats.cases.load());
		this_thread::sleep_for(chrono::seconds(1));
	}
}

int main(){
	srand(time(NULL));
	Corpus corpus("../corpus");
	Stats stats;
	Emulator emu(8 * 1024 * 1024, "../test_bins/xxd/xxd",
	             {"xxd", "input_file"});
	// future: run emu til main

	//Emulator runner = emu.fork();
	thread t(worker, emu.fork(), ref(emu), ref(corpus), ref(stats));

	thread stats_t(print_stats, ref(stats));
	t.join();
	stats_t.join();
	//worker(emu.fork(), emu, corpus);
}