#include <cstring>
#include <iomanip>
#include <assert.h>
#include "emulator.h"
#include "elf_parser.hpp"
#include "common.h"

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
	condition   = false;
	jump_offset = 0;
}

void Emulator::set_reg(uint8_t reg, uint32_t val){
	assert(0 <= reg && reg <= 31);
	if (reg != 0)
		regs[reg] = val;
}

uint32_t Emulator::get_reg(uint8_t reg){
	assert(0 <= reg && reg <= 31);
	return (reg ? regs[reg] : 0);
}

void Emulator::set_pc(addr_t addr){
	pc = addr;
}

uint32_t Emulator::get_pc(){
	return pc;
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
	condition   = other.condition;
	jump_offset = other.jump_offset;
}

void Emulator::load_elf(const char* pathname, const vector<string>& argv){
	Elf_parser elf(pathname);
	mmu.load_elf(elf.get_segments());

	pc = elf.get_entry();

	// Allocate the stack
	regs[29] = mmu.alloc(2 * 1024 * 1024) + 2 * 1024 * 1024;

	// Load args into memory
	vector<addr_t> argv_vm;
	addr_t arg_vm;
	for (const string& arg : argv){
		arg_vm = mmu.alloc(arg.size()+1);
		mmu.write_mem(arg_vm, arg.c_str(), arg.size()+1);
		argv_vm.push_back(arg_vm);
	}
	argv_vm.push_back(0); // Argv last element must be NULL

	// Set up argv and argc. Maybe I have to set up envp too?
	for (auto it = argv_vm.rbegin(); it != argv_vm.rend(); ++it){
		regs[29] -= 4;
		mmu.write<addr_t>(regs[29], *it);
	}
	regs[29] -= 4;
	mmu.write<uint32_t>(regs[29], argv.size());

	printf("Entry 0x%X\n", pc);
}

// Possibilities: Clean exit, timeout, [exception (fault)]
void Emulator::run(){
	for (int i= 0; i < 10; i++){
		run_inst();
		cout << *this << endl;
	}
}


// Instruction handlers indexed by opcode
const inst_handler Emulator::inst_handlers[] = {
	&Emulator::inst_R, // 000 000
	&Emulator::inst_RI, // 000 001
	&Emulator::inst_unimplemented, // 000 010
	&Emulator::inst_unimplemented, // 000 011
	&Emulator::inst_unimplemented, // 000 100
	&Emulator::inst_unimplemented, // 000 101
	&Emulator::inst_unimplemented, // 000 110
	&Emulator::inst_unimplemented, // 000 111
	&Emulator::inst_unimplemented, // 001 000
	&Emulator::inst_addiu,         // 001 001
	&Emulator::inst_unimplemented, // 001 010
	&Emulator::inst_unimplemented, // 001 011
	&Emulator::inst_unimplemented, // 001 100
	&Emulator::inst_unimplemented, // 001 101
	&Emulator::inst_unimplemented, // 001 110
	&Emulator::inst_lui,           // 001 111
	&Emulator::inst_unimplemented, // 010 000
	&Emulator::inst_unimplemented, // 010 001
	&Emulator::inst_unimplemented, // 010 010
	&Emulator::inst_unimplemented, // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_unimplemented, // 011 000
	&Emulator::inst_unimplemented, // 011 001
	&Emulator::inst_unimplemented, // 011 010
	&Emulator::inst_unimplemented, // 011 011
	&Emulator::inst_unimplemented, // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_unimplemented, // 011 111
	&Emulator::inst_unimplemented, // 100 000
	&Emulator::inst_unimplemented, // 100 001
	&Emulator::inst_unimplemented, // 100 010
	&Emulator::inst_lw,            // 100 011
	&Emulator::inst_unimplemented, // 100 100
	&Emulator::inst_unimplemented, // 100 101
	&Emulator::inst_unimplemented, // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_unimplemented, // 101 010
	&Emulator::inst_unimplemented, // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_unimplemented, // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_unimplemented, // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_unimplemented, // 110 011
	&Emulator::inst_unimplemented, // 110 100
	&Emulator::inst_unimplemented, // 110 101
	&Emulator::inst_unimplemented, // 110 110
	&Emulator::inst_unimplemented, // 110 111
	&Emulator::inst_unimplemented, // 111 000
	&Emulator::inst_unimplemented, // 111 001
	&Emulator::inst_unimplemented, // 111 010
	&Emulator::inst_unimplemented, // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_unimplemented, // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

// Type R instruction handlers indexed by functor
const inst_handler Emulator::inst_handlers_R[] = {
	&Emulator::inst_nop,           // 000 000
	&Emulator::inst_unimplemented, // 000 001
	&Emulator::inst_unimplemented, // 000 010
	&Emulator::inst_unimplemented, // 000 011
	&Emulator::inst_unimplemented, // 000 100
	&Emulator::inst_unimplemented, // 000 101
	&Emulator::inst_unimplemented, // 000 110
	&Emulator::inst_unimplemented, // 000 111
	&Emulator::inst_unimplemented, // 001 000
	&Emulator::inst_unimplemented, // 001 001
	&Emulator::inst_unimplemented, // 001 010
	&Emulator::inst_unimplemented, // 001 011
	&Emulator::inst_unimplemented, // 001 100
	&Emulator::inst_unimplemented, // 001 101
	&Emulator::inst_unimplemented, // 001 110
	&Emulator::inst_unimplemented, // 001 111
	&Emulator::inst_unimplemented, // 010 000
	&Emulator::inst_unimplemented, // 010 001
	&Emulator::inst_unimplemented, // 010 010
	&Emulator::inst_unimplemented, // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_unimplemented, // 011 000
	&Emulator::inst_unimplemented, // 011 001
	&Emulator::inst_unimplemented, // 011 010
	&Emulator::inst_unimplemented, // 011 011
	&Emulator::inst_unimplemented, // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_unimplemented, // 011 111
	&Emulator::inst_unimplemented, // 100 000
	&Emulator::inst_unimplemented, // 100 001
	&Emulator::inst_unimplemented, // 100 010
	&Emulator::inst_unimplemented, // 100 011
	&Emulator::inst_unimplemented, // 100 100
	&Emulator::inst_or,            // 100 101
	&Emulator::inst_unimplemented, // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_unimplemented, // 101 010
	&Emulator::inst_unimplemented, // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_unimplemented, // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_unimplemented, // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_unimplemented, // 110 011
	&Emulator::inst_unimplemented, // 110 100
	&Emulator::inst_unimplemented, // 110 101
	&Emulator::inst_unimplemented, // 110 110
	&Emulator::inst_unimplemented, // 110 111
	&Emulator::inst_unimplemented, // 111 000
	&Emulator::inst_unimplemented, // 111 001
	&Emulator::inst_unimplemented, // 111 010
	&Emulator::inst_unimplemented, // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_unimplemented, // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

// Type RI instruction handlers indexed by functor
const inst_handler Emulator::inst_handlers_RI[] = {
	&Emulator::inst_unimplemented, // 00 000
	&Emulator::inst_unimplemented, // 00 001
	&Emulator::inst_unimplemented, // 00 010
	&Emulator::inst_unimplemented, // 00 011
	&Emulator::inst_unimplemented, // 00 100
	&Emulator::inst_unimplemented, // 00 101
	&Emulator::inst_unimplemented, // 00 110
	&Emulator::inst_unimplemented, // 00 111
	&Emulator::inst_unimplemented, // 01 000
	&Emulator::inst_unimplemented, // 01 001
	&Emulator::inst_unimplemented, // 01 010
	&Emulator::inst_unimplemented, // 01 011
	&Emulator::inst_unimplemented, // 01 100
	&Emulator::inst_unimplemented, // 01 101
	&Emulator::inst_unimplemented, // 01 110
	&Emulator::inst_unimplemented, // 01 111
	&Emulator::inst_unimplemented, // 10 000
	&Emulator::inst_bgezal,        // 10 001
	&Emulator::inst_unimplemented, // 10 010
	&Emulator::inst_unimplemented, // 10 011
	&Emulator::inst_unimplemented, // 10 100
	&Emulator::inst_unimplemented, // 10 101
	&Emulator::inst_unimplemented, // 10 110
	&Emulator::inst_unimplemented, // 10 111
	&Emulator::inst_unimplemented, // 11 000
	&Emulator::inst_unimplemented, // 11 001
	&Emulator::inst_unimplemented, // 11 010
	&Emulator::inst_unimplemented, // 11 011
	&Emulator::inst_unimplemented, // 11 100
	&Emulator::inst_unimplemented, // 11 101
	&Emulator::inst_unimplemented, // 11 110
	&Emulator::inst_unimplemented, // 11 111
};

void Emulator::run_inst(){
	uint32_t inst   = mmu.read<uint32_t>(pc);
	uint8_t  opcode = (inst >> 26) & 0b111111;
	printf("[0x%X] Opcode: 0x%X, inst: 0x%X\n", pc, opcode, inst);

	// If needed, take the branch after fetching the current instruction
	// Otherwise, just increment PC so it points to the next instruction
	if (condition){
		pc += jump_offset;
		condition = false;
		jump_offset = 0;
	} else 
		pc += 4;

	// Handle current instruction
	(this->*inst_handlers[opcode])(inst);
	
}

void Emulator::inst_test(uint32_t inst){
	cout << "Test instruction" << endl;
}

void Emulator::inst_unimplemented(uint32_t inst){
	die("Unimplemented instruction: 0x%X\n", inst);
}

void Emulator::inst_R(uint32_t inst){
	// Get the functor and call the appropiate handler
	(this->*inst_handlers_R[inst & 0b00111111])(inst);
}

void Emulator::inst_RI(uint32_t inst){
	// Get the functor and call the appropiate handler
	(this->*inst_handlers_RI[(inst >> 16) & 0b00011111])(inst);
}

void Emulator::inst_or(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) | get_reg(inst.t));
}

void Emulator::inst_bgezal(uint32_t val){
	inst_RI_t inst(val);
	regs[31]    = pc + 4; // Instruction after the delay slot
	condition   = (inst.s == 0 || regs[inst.s] >= 0);
	jump_offset = inst.C << 2;
}

void Emulator::inst_nop(uint32_t val){ }

void Emulator::inst_lui(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, inst.C << 16);
}

void Emulator::inst_addiu(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) + (int16_t)inst.C);
}

void Emulator::inst_lw(uint32_t val){
	inst_I_t inst(val);
	addr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint32_t>(addr));
}

ostream& operator<<(ostream& os, const Emulator& emu){
	os << hex << setfill('0');
	os << "PC: " << emu.pc << endl;
	for (int i = 0; i < 32; i++){
		os << "$" << setw(2) << dec << i << hex << ": " << setw(8)
		   << emu.regs[i] << "\t";
		if ((i+1)%8 == 0)
			os << endl;
	}
	os << "condition: " << emu.condition
	   << "\tjump offset: " << emu.jump_offset << endl;
	os << dec;
	return os;
}