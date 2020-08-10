#include <iostream>
#include <emulator.h>
#include <mmu.h>
#include <common.h>
#include <corpus.h>

using namespace std;

/* TODO
Adapt the elf parser to my code
*/

int main(){
	Corpus corpus("../corpus");
	Emulator emu(8 * 1024 * 1024, "../test_bins/xxd/xxd",
	             {"xxd", "input_file"});

	Emulator runner = emu.fork();
	
	for (int i = 0; i < 1; i++){
		try {
			const string& input = corpus.get_new_input();
			runner.run(input);
		} catch (const Fault& f) {
			cout << "[PC: 0x" << hex << runner.get_pc() << "] " << f << endl;
			// Save input as a crash
		}
		runner.reset(emu);
	}
}