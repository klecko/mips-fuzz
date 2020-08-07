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
	emu.load_elf("../test_bins/xxd/xxd");
	emu.fork();
	try {
		
	} catch (const Fault& f) {
		cout << f << endl;
	}
}