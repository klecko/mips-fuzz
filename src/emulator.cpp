#include <cstring>
#include <iomanip>
#include <assert.h>
#include "emulator.h"
#include "elf_parser.hpp"
#include "common.h"

using namespace std;

enum Reg {
	zero, at, v0, v1, a0, a1, a2, a3,
	t0,   t1, t2, t3, t4, t5, t6, t7,
	s0,   s1, s2, s3, s4, s5, s6, s7,
	t8,   t9, k0, k1, gp, sp, fp, ra,
};

const std::unordered_map<vaddr_t, breakpoint_t> Emulator::breakpoints = {
	{0x0042b1a0, &Emulator::sbrk_bp}
};

/* Emulator::Emulator(): {
	memset(regs, 0, sizeof(regs));
	hi = 0;
	lo = 0;
	pc = 0;
} */

Emulator::Emulator(vsize_t mem_size): mmu(mem_size) {
	memset(regs, 0, sizeof(regs));
	hi = 0;
	lo = 0;
	pc = 0;
	condition = false;
	jump_addr = 0;
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

void Emulator::set_pc(vaddr_t addr){
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
	condition = other.condition;
	jump_addr = other.jump_addr;
}

void Emulator::load_elf(const char* pathname, const vector<string>& argv){
	Elf_parser elf(pathname);
	mmu.load_elf(elf.get_segments());

	// Set entry
	pc = elf.get_entry();
	printf("Entry 0x%X\n", pc);

	// Allocate the stack
	regs[Reg::sp] = mmu.alloc_stack(2 * 1024 * 1024);
	printf("Allocated stack at 0x%X\n", regs[Reg::sp]);

	// Load args into the stack
	vector<vaddr_t> argv_vm;
	for (const string& arg : argv){
		regs[Reg::sp] -= arg.size()+1;
		mmu.write_mem(regs[Reg::sp], arg.c_str(), arg.size()+1);
		argv_vm.push_back(regs[Reg::sp]);
	}
	argv_vm.push_back(0); // Argv last element must be NULL

	// Align sp
	regs[Reg::sp] = regs[Reg::sp] - 0xF & ~0xF;

	// Set up auxp
	regs[Reg::sp] -= 4;
	mmu.write<vaddr_t>(regs[Reg::sp], 0);

	// Set up envp
	regs[Reg::sp] -= 4;
	mmu.write<vaddr_t>(regs[Reg::sp], 0);

	// Set up argv and argc
	for (auto it = argv_vm.rbegin(); it != argv_vm.rend(); ++it){
		regs[Reg::sp] -= 4;
		mmu.write<vaddr_t>(regs[Reg::sp], *it);
	}
	regs[Reg::sp] -= 4;
	mmu.write<uint32_t>(regs[Reg::sp], argv.size());
}

// Possibilities: Clean exit, timeout, [exception (fault)]
void Emulator::run(){
	for (int i= 0; i < 1000; i++){
		run_inst();
		cout << *this << endl;
		if (pc == 0x4014a4){
			cout << "MAIN" << endl;
			break;
		}
	}
}

void Emulator::sbrk_bp(){
	int32_t increment = regs[Reg::a0];
	if (increment < 0)
		die("sbrk neg size: %d\n", increment);

	vaddr_t addr = mmu.alloc(increment);
	regs[Reg::v0] = addr;
	pc = regs[Reg::ra];
	printf("sbrk(%d) --> 0x%X\n", increment, addr);
}

/* uint32_t Emulator::sys_brk(vaddr_t addr){
	vaddr_t brk = mmu.get_brk();

	// Attempt to get current brk
	if (!addr)
		return brk;

	// Attempt to change brk
	mmu.set_brk(addr);

	// If we got here change was successful
	return addr;
} */

void Emulator::handle_syscall(uint32_t syscall){
	switch (syscall){
		case 4024: // getuid
		case 4047: // getgid
		case 4049: // geteuid
		case 4050: // getegid
			regs[Reg::v0] = 0;
			regs[Reg::a3] = 0;
			break;
		default:
			die("Unimplemented syscall: %d\n", syscall);
	}
}


// Instruction handlers indexed by opcode
const inst_handler_t Emulator::inst_handlers[] = {
	&Emulator::inst_R,             // 000 000
	&Emulator::inst_RI,            // 000 001
	&Emulator::inst_unimplemented, // 000 010
	&Emulator::inst_unimplemented, // 000 011
	&Emulator::inst_beq,           // 000 100
	&Emulator::inst_bne,           // 000 101
	&Emulator::inst_blez,          // 000 110
	&Emulator::inst_unimplemented, // 000 111
	&Emulator::inst_unimplemented, // 001 000
	&Emulator::inst_addiu,         // 001 001
	&Emulator::inst_slti,          // 001 010
	&Emulator::inst_sltiu,         // 001 011
	&Emulator::inst_andi,          // 001 100
	&Emulator::inst_ori,           // 001 101
	&Emulator::inst_xori,          // 001 110
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
	&Emulator::inst_special2,      // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_special3,      // 011 111
	&Emulator::inst_unimplemented, // 100 000
	&Emulator::inst_unimplemented, // 100 001
	&Emulator::inst_unimplemented, // 100 010
	&Emulator::inst_lw,            // 100 011
	&Emulator::inst_unimplemented, // 100 100
	&Emulator::inst_lhu,           // 100 101
	&Emulator::inst_unimplemented, // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_unimplemented, // 101 010
	&Emulator::inst_sw,            // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_unimplemented, // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_unimplemented, // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_pref,          // 110 011
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
const inst_handler_t Emulator::inst_handlers_R[] = {
	&Emulator::inst_sll,           // 000 000
	&Emulator::inst_unimplemented, // 000 001
	&Emulator::inst_unimplemented, // 000 010
	&Emulator::inst_unimplemented, // 000 011
	&Emulator::inst_unimplemented, // 000 100
	&Emulator::inst_unimplemented, // 000 101
	&Emulator::inst_unimplemented, // 000 110
	&Emulator::inst_unimplemented, // 000 111
	&Emulator::inst_jr,            // 001 000
	&Emulator::inst_jalr,          // 001 001
	&Emulator::inst_movz,          // 001 010
	&Emulator::inst_movn,          // 001 011
	&Emulator::inst_syscall,       // 001 100
	&Emulator::inst_unimplemented, // 001 101
	&Emulator::inst_unimplemented, // 001 110
	&Emulator::inst_unimplemented, // 001 111
	&Emulator::inst_unimplemented, // 010 000
	&Emulator::inst_unimplemented, // 010 001
	&Emulator::inst_mflo,          // 010 010
	&Emulator::inst_unimplemented, // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_unimplemented, // 011 000
	&Emulator::inst_unimplemented, // 011 001
	&Emulator::inst_unimplemented, // 011 010
	&Emulator::inst_divu,          // 011 011
	&Emulator::inst_unimplemented, // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_unimplemented, // 011 111
	&Emulator::inst_unimplemented, // 100 000
	&Emulator::inst_addu,          // 100 001
	&Emulator::inst_unimplemented, // 100 010
	&Emulator::inst_subu,          // 100 011
	&Emulator::inst_and,           // 100 100
	&Emulator::inst_or,            // 100 101
	&Emulator::inst_xor,           // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_unimplemented, // 101 010
	&Emulator::inst_sltu,          // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_unimplemented, // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_unimplemented, // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_unimplemented, // 110 011
	&Emulator::inst_teq,           // 110 100
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
const inst_handler_t Emulator::inst_handlers_RI[] = {
	&Emulator::inst_bltz,          // 00 000
	&Emulator::inst_bgez,          // 00 001
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

// Type special2 instructions indexed by functor
const inst_handler_t Emulator::inst_handlers_special2[] = {
	&Emulator::inst_unimplemented, // 000 000
	&Emulator::inst_unimplemented, // 000 001
	&Emulator::inst_mul,           // 000 010
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

const inst_handler_t Emulator::inst_handlers_special3[] = {
	&Emulator::inst_unimplemented, // 000 000
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
	&Emulator::inst_rdhwr,         // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_unimplemented, // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

void Emulator::run_inst(){
	// Handle breakpoint. It may change pc
	if (breakpoints.count(pc))
		(this->*breakpoints.at(pc))();

	uint32_t inst   = mmu.read<uint32_t>(pc);
	uint8_t  opcode = (inst >> 26) & 0b111111;
	printf("[0x%X] Opcode: 0x%X, inst: 0x%X\n", pc, opcode, inst);

	// If needed, take the branch after fetching the current instruction
	// Otherwise, just increment PC so it points to the next instruction
	if (condition){
		pc = jump_addr;
		condition = false;
		jump_addr = 0;
	} else 
		pc += 4;

	// Handle current instruction if it isn't a NOP
	if (inst)
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

void Emulator::inst_special2(uint32_t inst){
	(this->*inst_handlers_special2[inst & 0b00111111])(inst);
}

void Emulator::inst_special3(uint32_t inst){
	(this->*inst_handlers_special3[inst & 0b00111111])(inst);
}

void Emulator::inst_or(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) | get_reg(inst.t));
}

void Emulator::inst_bgezal(uint32_t val){
	inst_RI_t inst(val);
	regs[Reg::ra]  = pc + 4; // Instruction after the delay slot
	condition = (inst.s == 0 || regs[inst.s] >= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
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
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint32_t>(addr));
}

void Emulator::inst_and(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) & get_reg(inst.t));
}

void Emulator::inst_sw(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint32_t>(addr, get_reg(inst.t));
}

void Emulator::inst_jalr(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, pc+4); // Instruction after the delay slot
	condition = true;
	jump_addr = get_reg(inst.s);
}

void Emulator::inst_beq(uint32_t val){
	inst_I_t inst(val);
	condition = (get_reg(inst.s) == get_reg(inst.t));
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_addu(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.t) + get_reg(inst.s));
}

void Emulator::inst_sll(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.t) << inst.S);
}

void Emulator::inst_bne(uint32_t val){
	inst_I_t inst(val);
	condition = (get_reg(inst.s) != get_reg(inst.t));
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_jr(uint32_t val){
	inst_R_t inst(val);
	condition = true;
	jump_addr = get_reg(inst.s);
}

void Emulator::inst_lhu(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint16_t>(addr));
}

void Emulator::inst_syscall(uint32_t val){
	handle_syscall(regs[Reg::v0]);
}

void Emulator::inst_xor(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) ^ get_reg(inst.t));
}

void Emulator::inst_sltu(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) < get_reg(inst.t));
}

void Emulator::inst_sltiu(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) < (uint32_t)(int16_t)inst.C);
}

void Emulator::inst_movn(uint32_t val){
	inst_R_t inst(val);
	if (get_reg(inst.t) != 0)
		set_reg(inst.d, get_reg(inst.s));
}

void Emulator::inst_movz(uint32_t val){
	inst_R_t inst(val);
	if (get_reg(inst.t) == 0)
		set_reg(inst.d, get_reg(inst.s));
}

void Emulator::inst_subu(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) - get_reg(inst.t));
}

void Emulator::inst_teq(uint32_t val){
	inst_R_t inst(val);
	if (get_reg(inst.s) == get_reg(inst.t))
		die("TRAP\n");
}

void Emulator::inst_divu(uint32_t val){
	inst_R_t inst(val);
	uint32_t div  = get_reg(inst.s);
	uint32_t quot = get_reg(inst.t);
	hi = div / quot;
	lo = div % quot;
}

void Emulator::inst_mflo(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, lo);
}

void Emulator::inst_mul(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, (int32_t)get_reg(inst.s) * (int32_t)get_reg(inst.t));
}

void Emulator::inst_bltz(uint32_t val){
	inst_RI_t inst(val);
	condition = ((int32_t)get_reg(inst.s) < 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_blez(uint32_t val){
	inst_I_t inst(val);
	condition = ((int32_t)get_reg(inst.s) <= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_rdhwr(uint32_t val){
	inst_R_t inst(val);
	uint32_t hwr = 0;
	switch (inst.d){
		case 29: // 0b11101, User Local Register
			hwr = mmu.get_tls();
			printf("tls = 0x%X\n", hwr);
			break;

		default:
			die("Unimplemented rdhwr: %d\n", inst.d);
	}
	
	set_reg(inst.t, hwr);
}

void Emulator::inst_bgez(uint32_t val){
	inst_RI_t inst(val);
	condition = ((int32_t)get_reg(inst.s) >= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_slti(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, (int32_t)get_reg(inst.s) < (int32_t)(int16_t)inst.C);
}

void Emulator::inst_andi(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) & inst.C);
}

void Emulator::inst_ori(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) | inst.C);
}

void Emulator::inst_xori(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) ^ inst.C);
}

void Emulator::inst_pref(uint32_t val){ }

const char* regs_map[] = {
	"00",  "at", "v0", "v1",
	"a0", "a1", "a2", "a3",
	"t0", "t1", "t2", "t3", 
	"t4", "t5", "t6", "t7", 
	"s0", "s1", "s2", "s3", 
	"s4", "s5", "s6", "s7", 
	"t8", "t9", "k0", "k1", 
	"gp", "sp", "fp", "ra"
};
ostream& operator<<(ostream& os, const Emulator& emu){
	os << hex << setfill('0');
	os << "PC:  " << setw(8) << emu.pc << endl;
	for (int i = 0; i < 32; i++){
		os << "$" << regs_map[i] << ": " << setw(8) << emu.regs[i] << "\t";
		if ((i+1)%8 == 0)
			os << endl;
	}
	os << "condition: " << emu.condition << "\t"
	   << "jump addr: " << emu.jump_addr << endl;
	os << dec;
	return os;
}