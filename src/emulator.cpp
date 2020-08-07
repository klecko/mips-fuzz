#include <cstring>
#include "emulator.h"
#include "elf_parser.hpp"

using namespace std;

/* Emulator::Emulator(): {
	memset(regs, 0, sizeof(regs));
	hi = 0;
	lo = 0;
	pc = 0;
} */

Emulator::Emulator(size_t mem_size): mmu(mem_size) {
	memset(regs, 0, sizeof(regs));
	hi = 0;
	lo = 0;
	pc = 0;
}

Emulator Emulator::fork(){
	Emulator new_emu(*this);
	new_emu.mmu = mmu.fork();
	return new_emu;
}

void Emulator::load_elf(const char* pathname){
	Elf_parser elf(pathname);
	mmu.load_elf(elf.get_segments());

	pc = elf.get_entry();
	printf("Entry 0x%X\n", pc);
}

void Emulator::run_emu(){

}

void Emulator::run(){

}