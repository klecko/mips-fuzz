#include <iostream>
#include <emulator.h>
#include <mmu.h>
#include <common.h>

using namespace std;

/* TODO
Adapt the elf parser to my code
*/

int main(){
	Emulator emu(8 * 1024 * 1024);
	try {
		emu.load_elf("../test_bins/xxd/xxd", {"hola", "adios"});
	} catch (const Fault& f) {
		cout << "Unexepected fault loading elf: " << f << endl;
		exit(EXIT_FAILURE);
	}
	Emulator runner = emu.fork();
	
	for (int i = 0; i < 1; i++){
		try {
			runner.run();
		} catch (const Fault& f) {
			cout << "[PC: 0x" << hex << runner.get_pc() << "] " << f << endl;
		}
		runner.reset(emu);
	}
}