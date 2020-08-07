#include <cstring>
#include "emulator.h"
#include "elf_parser.hpp"
#include "common.h"

using namespace std;

// Instruction handlers indexed by opcode
const inst_handler Emulator::table[] = {
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented,
	&Emulator::inst_unimplemented
};

void Emulator::inst_unimplemented(uint32_t inst){
	die("Unimplemented instruction: 0x%X\n", inst);
}

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

void Emulator::reset(const Emulator& other){
	mmu.reset(other.mmu);
	memcpy(regs, other.regs, sizeof(regs));
	hi = other.hi;
	lo = other.lo;
	pc = other.pc;
}

void Emulator::load_elf(const char* pathname){
	Elf_parser elf(pathname);
	mmu.load_elf(elf.get_segments());

	pc = elf.get_entry();
	printf("Entry 0x%X\n", pc);
}

void Emulator::run_inst(){
	uint32_t inst   = mmu.read<uint32_t>(pc);
	uint8_t  opcode = (inst >> 26) & 0b111111;
	printf("[0x%X] Opcode: 0x%X\n", pc, opcode);

	(this->*table[opcode])(inst);

	pc += 4;
}

// Possibilities: Clean exit, timeout, [exception (fault)]
void Emulator::run(){
	for (int i= 0; i < 10; i++)
		run_inst();
}