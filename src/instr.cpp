#include <iostream>
#include <cstring> // memcpy
#include <cmath>   // isnan
#include "emulator.h"
#include "inst_decoding.h"

using namespace std;

// Instruction handlers indexed by opcode
const inst_handler_t Emulator::inst_handlers[] = {
	&Emulator::inst_R,             // 000 000
	&Emulator::inst_RI,            // 000 001
	&Emulator::inst_j,             // 000 010
	&Emulator::inst_jal,           // 000 011
	&Emulator::inst_beq,           // 000 100
	&Emulator::inst_bne,           // 000 101
	&Emulator::inst_blez,          // 000 110
	&Emulator::inst_bgtz,          // 000 111
	&Emulator::inst_unimplemented, // 001 000
	&Emulator::inst_addiu,         // 001 001
	&Emulator::inst_slti,          // 001 010
	&Emulator::inst_sltiu,         // 001 011
	&Emulator::inst_andi,          // 001 100
	&Emulator::inst_ori,           // 001 101
	&Emulator::inst_xori,          // 001 110
	&Emulator::inst_lui,           // 001 111
	&Emulator::inst_unimplemented, // 010 000
	&Emulator::inst_COP1,          // 010 001
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
	&Emulator::inst_lb,            // 100 000
	&Emulator::inst_lh,            // 100 001
	&Emulator::inst_lwl,           // 100 010
	&Emulator::inst_lw,            // 100 011
	&Emulator::inst_lbu,           // 100 100
	&Emulator::inst_lhu,           // 100 101
	&Emulator::inst_lwr,           // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_sb,            // 101 000
	&Emulator::inst_sh,            // 101 001
	&Emulator::inst_swl,           // 101 010
	&Emulator::inst_sw,            // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_swr,           // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_ll,            // 110 000
	&Emulator::inst_lwc1,          // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_pref,          // 110 011
	&Emulator::inst_unimplemented, // 110 100
	&Emulator::inst_ldc1,          // 110 101
	&Emulator::inst_unimplemented, // 110 110
	&Emulator::inst_unimplemented, // 110 111
	&Emulator::inst_sc,            // 111 000
	&Emulator::inst_swc1,          // 111 001
	&Emulator::inst_unimplemented, // 111 010
	&Emulator::inst_unimplemented, // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_sdc1,          // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

// Type R instruction handlers indexed by functor
const inst_handler_t Emulator::inst_handlers_R[] = {
	&Emulator::inst_sll,           // 000 000
	&Emulator::inst_unimplemented, // 000 001
	&Emulator::inst_srl,           // 000 010
	&Emulator::inst_sra,           // 000 011
	&Emulator::inst_sllv,          // 000 100
	&Emulator::inst_unimplemented, // 000 101
	&Emulator::inst_srlv,          // 000 110
	&Emulator::inst_srav,          // 000 111
	&Emulator::inst_jr,            // 001 000
	&Emulator::inst_jalr,          // 001 001
	&Emulator::inst_movz,          // 001 010
	&Emulator::inst_movn,          // 001 011
	&Emulator::inst_syscall,       // 001 100
	&Emulator::inst_unimplemented, // 001 101
	&Emulator::inst_unimplemented, // 001 110
	&Emulator::inst_sync,          // 001 111
	&Emulator::inst_mfhi,          // 010 000
	&Emulator::inst_mthi,          // 010 001
	&Emulator::inst_mflo,          // 010 010
	&Emulator::inst_mtlo,          // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_mult,          // 011 000
	&Emulator::inst_multu,         // 011 001
	&Emulator::inst_div,           // 011 010
	&Emulator::inst_divu,          // 011 011
	&Emulator::inst_unimplemented, // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_unimplemented, // 011 111
	&Emulator::inst_add,           // 100 000
	&Emulator::inst_addu,          // 100 001
	&Emulator::inst_sub,           // 100 010
	&Emulator::inst_subu,          // 100 011
	&Emulator::inst_and,           // 100 100
	&Emulator::inst_or,            // 100 101
	&Emulator::inst_xor,           // 100 110
	&Emulator::inst_nor,           // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_slt,           // 101 010
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
	&Emulator::inst_clz,           // 100 000
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
	&Emulator::inst_ext,           // 000 000
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
	&Emulator::inst_bshfl,         // 100 000
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

// Type COP1 instructions indexed by rs/fmt field
const inst_handler_t Emulator::inst_handlers_COP1[] = {
	&Emulator::inst_mfc1,          // 00 000
	&Emulator::inst_unimplemented, // 00 001
	&Emulator::inst_cfc1,          // 00 010
	&Emulator::inst_mfhc1,         // 00 011
	&Emulator::inst_mtc1,          // 00 100
	&Emulator::inst_unimplemented, // 00 101
	&Emulator::inst_unimplemented, // 00 110
	&Emulator::inst_mthc1,         // 00 111
	&Emulator::inst_bc1,           // 01 000
	&Emulator::inst_unimplemented, // 01 001
	&Emulator::inst_unimplemented, // 01 010
	&Emulator::inst_unimplemented, // 01 011
	&Emulator::inst_unimplemented, // 01 100
	&Emulator::inst_unimplemented, // 01 101
	&Emulator::inst_unimplemented, // 01 110
	&Emulator::inst_unimplemented, // 01 111
	&Emulator::inst_fmt_s,         // 10 000
	&Emulator::inst_fmt_d,         // 10 001
	&Emulator::inst_unimplemented, // 10 010
	&Emulator::inst_unimplemented, // 10 011
	&Emulator::inst_fmt_w,         // 10 100
	&Emulator::inst_fmt_l,         // 10 101
	&Emulator::inst_fmt_ps,        // 10 110
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

void Emulator::inst_test(uint32_t inst){
	cout << "Test instruction" << endl;
}

void Emulator::inst_unimplemented(uint32_t inst){
	die("Unimplemented instruction at 0x%X: 0x%X\n", prev_pc, inst);
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

void Emulator::inst_COP1(uint32_t inst){
	(this->*inst_handlers_COP1[(inst >> 21) & 0b00011111])(inst);
}

void Emulator::inst_or(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) | get_reg(inst.t));
}

void Emulator::inst_bgezal(uint32_t val){
	inst_RI_t inst(val);
	regs[Reg::ra]  = pc + 4; // Instruction after the delay slot
	condition = ((int32_t)get_reg(inst.s) >= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
	rec_cov   = true;
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
	rec_cov   = true;
}

void Emulator::inst_beq(uint32_t val){
	inst_I_t inst(val);
	condition = (get_reg(inst.s) == get_reg(inst.t));
	jump_addr = pc + ((int16_t)inst.C << 2);
	rec_cov   = true;
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
	rec_cov   = true;
}

void Emulator::inst_jr(uint32_t val){
	inst_R_t inst(val);
	condition = true;
	jump_addr = get_reg(inst.s);
	rec_cov   = true;
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

void Emulator::inst_div(uint32_t val){
	inst_R_t inst(val);
	int32_t dividend  = get_reg(inst.s);
	int32_t divisor   = get_reg(inst.t);
	hi = dividend % divisor; // module
	lo = dividend / divisor; // quotient
}


void Emulator::inst_divu(uint32_t val){
	inst_R_t inst(val);
	uint32_t dividend  = get_reg(inst.s);
	uint32_t divisor   = get_reg(inst.t);
	hi = dividend % divisor; // module
	lo = dividend / divisor; // quotient
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
	rec_cov   = true;
}

void Emulator::inst_blez(uint32_t val){
	inst_I_t inst(val);
	condition = ((int32_t)get_reg(inst.s) <= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
	rec_cov   = true;
}

void Emulator::inst_rdhwr(uint32_t val){
	inst_R_t inst(val);
	handle_rdhwr(inst.d, inst.t);
}

void Emulator::inst_bgez(uint32_t val){
	inst_RI_t inst(val);
	condition = ((int32_t)get_reg(inst.s) >= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
	rec_cov   = true;
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

void Emulator::inst_jal(uint32_t val){
	inst_J_t inst(val);
	regs[Reg::ra] = pc + 4; // Instruction after the delay slot
	condition = true;       // 28 bits from A, 4 bits from pc
	jump_addr = (inst.A << 2) | (pc & 0xF0000000);
	rec_cov   = true;
}

void Emulator::inst_lb(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, (int32_t)mmu.read<int8_t>(addr));
}

void Emulator::inst_nor(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, ~(get_reg(inst.s) | get_reg(inst.t)));
}

void Emulator::inst_bshfl(uint32_t val){
	switch ((val >> 6) & 0b11111){
		case 0b00010: // wsbh
			inst_wsbh(val);
			break;

		case 0b10000: // seb
			inst_seb(val);
			break;

		case 0b11000: // seh
			inst_seh(val);
			break;

		default:
			die("Unimplemented bshfl instruction at 0x%X: 0x%X\n",prev_pc, val);
	}
}

void Emulator::inst_seh(uint32_t val){
	inst_R_t inst(val);
	uint16_t value = get_reg(inst.t);
	set_reg(inst.d, (int32_t)(int16_t)value);
}

void Emulator::inst_seb(uint32_t val){
	inst_R_t inst(val);
	uint8_t value = get_reg(inst.t);
	set_reg(inst.d, (int32_t)(int8_t)value);
}

void Emulator::inst_wsbh(uint32_t val){
	inst_R_t inst(val);
	uint32_t value = get_reg(inst.t);
	value = ((value & 0xFF000000) >> 8) | ((value & 0x00FF0000) << 8) |
	        ((value & 0x0000FF00) >> 8) | ((value & 0x000000FF) << 8);
	set_reg(inst.d, value);
}

void Emulator::inst_srl(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.t) >> inst.S);
}

void Emulator::inst_lh(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, (int32_t)mmu.read<int16_t>(addr));
}

void Emulator::inst_lbu(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint8_t>(addr));
}


void Emulator::inst_lwl(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	uint32_t w   = mmu.read<uint32_t>(addr & ~3);
	memcpy((char*)(&reg)+3-offset, &w, offset+1);
	set_reg(inst.t, reg);
}

void Emulator::inst_lwr(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	uint32_t w   = mmu.read<uint32_t>(addr & ~3);
	memcpy(&reg, (char*)(&w)+offset, 4-offset);
	set_reg(inst.t, reg);
}

void Emulator::inst_sb(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint8_t>(addr, get_reg(inst.t));
}

void Emulator::inst_sh(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint16_t>(addr, get_reg(inst.t));
}

void Emulator::inst_swl(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	// Don't check uninitialized memory here
	uint32_t w   = mmu.read<uint32_t>(addr & ~3, false);
	memcpy(&w, (char*)(&reg)+3-offset, offset+1);
	mmu.write<uint32_t>(addr & ~3, w);
}

void Emulator::inst_swr(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	// Don't check uninitialized memory here
	uint32_t w   = mmu.read<uint32_t>(addr & ~3, false);
	memcpy((char*)(&w)+offset, &reg, 4-offset);
	mmu.write<uint32_t>(addr & ~3, w);
}

void Emulator::inst_sllv(uint32_t val){
	inst_R_t inst(val);
	uint8_t shift = get_reg(inst.s) & 0b00011111;
	set_reg(inst.d, get_reg(inst.t) << shift);
}

void Emulator::inst_srlv(uint32_t val){
	inst_R_t inst(val);
	uint8_t shift = get_reg(inst.s) & 0b00011111;
	set_reg(inst.d, get_reg(inst.t) >> shift);
}

void Emulator::inst_slt(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, (int32_t)get_reg(inst.s) < (int32_t)get_reg(inst.t));
}

void Emulator::inst_sub(uint32_t val){
	inst_R_t inst(val);
	int32_t result;
	bool overflow = __builtin_sub_overflow(
		(int32_t)get_reg(inst.s),
		(int32_t)get_reg(inst.t),
		&result
	);
	if (overflow)
		die("sub overflow\n");
	set_reg(inst.d, result);
	die("sub\n");
}

void Emulator::inst_add(uint32_t val){
	inst_R_t inst(val);
	int32_t result;
	bool overflow = __builtin_add_overflow(
		(int32_t)get_reg(inst.s),
		(int32_t)get_reg(inst.t),
		&result
	);
	if (overflow)
		die("Add overflow\n");
	set_reg(inst.d, result);
	die("add\n");
}

void Emulator::inst_j(uint32_t val){
	inst_J_t inst(val);
	condition = true;       // 28 bits from A, 4 bits from pc
	jump_addr = (inst.A << 2) | (pc & 0xF0000000);
	rec_cov   = true;
}

void Emulator::inst_ll(uint32_t val){
	// Forget about atomics for now
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint32_t>(addr));
}

void Emulator::inst_sc(uint32_t val){
	// Forget about atomics for now
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint32_t>(addr, get_reg(inst.t));
	set_reg(inst.t, 1);
}

void Emulator::inst_sync(uint32_t val){
	// Forget about atomics for now
}

void Emulator::inst_bgtz(uint32_t val){
	inst_I_t inst(val);
	condition = ((int32_t)get_reg(inst.s) > 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
	rec_cov   = true;
}

void Emulator::inst_mult(uint32_t val){
	inst_R_t inst(val);
	uint64_t result = (int32_t)get_reg(inst.s) * (int32_t)get_reg(inst.t);
	lo = result & 0xFFFFFFFF;
	hi = (result >> 32) & 0xFFFFFFFF;
}

void Emulator::inst_multu(uint32_t val){
	inst_R_t inst(val);
	uint64_t result = (uint64_t)get_reg(inst.s) * get_reg(inst.t);
	lo = result & 0xFFFFFFFF;
	hi = (result >> 32) & 0xFFFFFFFF;
}

void Emulator::inst_mfhi(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, hi);
}

void Emulator::inst_mthi(uint32_t val){
	inst_R_t inst(val);
	hi = get_reg(inst.s);
}

void Emulator::inst_mtlo(uint32_t val){
	inst_R_t inst(val);
	lo = get_reg(inst.s);
}

void Emulator::inst_ext(uint32_t val){
	inst_R_t inst(val);
	uint32_t lsb  = inst.S;
	uint32_t size = inst.d + 1;
	uint32_t mask = (1 << size) - 1;
	set_reg(inst.t, (get_reg(inst.s) >> lsb) & mask);
}

void Emulator::inst_sra(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, (int32_t)get_reg(inst.t) >> inst.S);
}

void Emulator::inst_srav(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, (int32_t)get_reg(inst.t) >> (get_reg(inst.s) & 0b00011111));
}

void Emulator::inst_clz(uint32_t val){
	inst_R_t inst(val);
	uint32_t value = get_reg(inst.s);
	uint32_t result = 0;
	
	// Count leading zeros
	while ( (value & (1<<(31-result))) == 0 && result<32 ) result++;
	set_reg(inst.d, result);	
}

void Emulator::inst_lwc1(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	sets_reg(inst.t, mmu.read<float>(addr));
}

void Emulator::inst_swc1(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<float>(addr, gets_reg(inst.t));
}

void Emulator::inst_ldc1(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	setd_reg(inst.t, mmu.read<double>(addr));
}

void Emulator::inst_sdc1(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<double>(addr, getd_reg(inst.t));
}

void Emulator::inst_fmt_s(uint32_t val){
	inst_F_t inst(val);
	if ((inst.funct & 0b110000) == 0b110000){
		// C.cond.s
		inst_c_cond_s(val);
	} else switch (inst.funct){
		case 0b000010: // mul.s
			sets_reg(inst.d, gets_reg(inst.s) * gets_reg(inst.t));
			break;
		case 0b000011: // div.s
			sets_reg(inst.d, gets_reg(inst.s) / gets_reg(inst.t));
			break;
		case 0b100001: // cvt.d.s
			setd_reg(inst.d, (double)gets_reg(inst.s));
			break;
		default:
			die("Unimplemented fmt s at 0x%X\n", prev_pc);
	}
}

void Emulator::inst_fmt_d(uint32_t val){
	inst_F_t inst(val);
	if ((inst.funct & 0b110000) == 0b110000){
		// C.cond.d
		inst_c_cond_d(val);
	} else switch (inst.funct){
		case 0b000010: // mul.d
			setd_reg(inst.d, getd_reg(inst.s) * getd_reg(inst.t));
			break;
		case 0b000011: // div.d
			setd_reg(inst.d, getd_reg(inst.s) / getd_reg(inst.t));
			break;
		default:
			die("Unimplemented fmt s at 0x%X\n", prev_pc);
	}
}

void Emulator::inst_fmt_w(uint32_t val){
	die("Unimplemented fmt w at 0x%X\n", prev_pc);
}

void Emulator::inst_fmt_l(uint32_t val){
	die("Unimplemented fmt l at 0x%X\n", prev_pc);
}

void Emulator::inst_fmt_ps(uint32_t val){
	die("Unimplemented fmt ps at 0x%X\n", prev_pc);
}

void Emulator::inst_mfc1(uint32_t val){
	inst_F_t inst(val);
	set_reg(inst.t, gets_reg(inst.s));
}

void Emulator::inst_mfhc1(uint32_t val){
	inst_F_t inst(val);
	double value = getd_reg(inst.s);
	// Get upper 32 bits
	set_reg(inst.t, *(uint32_t*)((float*)&value+1));
}

void Emulator::inst_mtc1(uint32_t val){
	inst_F_t inst(val);
	sets_reg(inst.s, get_reg(inst.t));
}

void Emulator::inst_mthc1(uint32_t val){
	inst_F_t inst(val);
	sets_reg(inst.s+1, get_reg(inst.t));
}

void Emulator::inst_c_cond_s(uint32_t val){
	inst_F_t inst(val);
	//uint8_t cond = inst.funct & 0b001111;
	die("Unimplemented c.cond.s at 0x%X\n", prev_pc);
}

void Emulator::inst_c_cond_d(uint32_t val){
	inst_F_t inst(val);
	uint8_t cond = inst.funct & 0b001111;
	uint8_t cc   = (inst.d >> 2) & 0b111;
	double  val1 = getd_reg(inst.s);
	double  val2 = getd_reg(inst.t);
	switch (cond){
		case 0b0001: // un
			set_cc(cc, isnan(val1) || isnan(val1));
			break;
		case 0b0111: // ule
			/* set_cc(cc, isnan(val1) || isnan(val1) || (val1<val2) ||
			           (val1==val2)); */
			set_cc(cc, !(val1>val2));
			break;
		default:
			die("Unimplemented c.cond.d at 0x%X: %X\n", prev_pc, val);
	}
}

void Emulator::inst_bc1(uint32_t val){
	inst_I_t inst(val);
	bool jump_if_true = inst.t & 1;
	uint8_t cc = (inst.t >> 2) & 0b111;
	condition  = (jump_if_true ? get_cc(cc) : !get_cc(cc));
	jump_addr  = pc + ((int16_t)inst.C << 2);
	rec_cov    = true;
}

void Emulator::inst_cfc1(uint32_t val){
	inst_F_t inst(val);
	uint32_t value;
	switch (inst.s){
		case 31: // FCSR. Qemu uses all fields 0 except CCs
			value = 0;
			// Set CCs
			value |= (get_cc(0) << 23);
			for (int i = 1; i < 8; i++)
				value |= (get_cc(i) << (24+i));
			break;

		default:
			die("Unimplemented cfc1 at 0x%X\n", prev_pc);
	}
	set_reg(inst.t, value);
}

void Emulator::inst_break(uint32_t val){
	die("Break?\n");
}