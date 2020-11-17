#include "jitter.h"
#include "inst_decoding.h"

using namespace std;
using namespace JIT;

// When enabled, a function call will finish the compilation so that
// the called function is in another compilation unit.
// When disabled, the function will be inlined in the current compilation unit.
// Disabling this usually gived a bit more runtime speed, with the cost of a
// greatly increased compilation time.
#define END_COMPILING_ON_CALLS 1

// Helpers for FPU instructions
#define as_u32(value) builder.CreateBitCast(value, int32_ty)
#define as_float(value) builder.CreateBitCast(value, float_ty)

// Instruction handlers indexed by opcode
const Jitter::inst_handler_t Jitter::inst_handlers[] = {
	&Jitter::inst_R,             // 000 000
	&Jitter::inst_RI,            // 000 001
	&Jitter::inst_j,             // 000 010
	&Jitter::inst_jal,           // 000 011
	&Jitter::inst_beq,           // 000 100
	&Jitter::inst_bne,           // 000 101
	&Jitter::inst_blez,          // 000 110
	&Jitter::inst_bgtz,          // 000 111
	&Jitter::inst_unimplemented, // 001 000
	&Jitter::inst_addiu,         // 001 001
	&Jitter::inst_slti,          // 001 010
	&Jitter::inst_sltiu,         // 001 011
	&Jitter::inst_andi,          // 001 100
	&Jitter::inst_ori,           // 001 101
	&Jitter::inst_xori,          // 001 110
	&Jitter::inst_lui,           // 001 111
	&Jitter::inst_unimplemented, // 010 000
	&Jitter::inst_COP1,          // 010 001
	&Jitter::inst_unimplemented, // 010 010
	&Jitter::inst_unimplemented, // 010 011
	&Jitter::inst_unimplemented, // 010 100
	&Jitter::inst_unimplemented, // 010 101
	&Jitter::inst_unimplemented, // 010 110
	&Jitter::inst_unimplemented, // 010 111
	&Jitter::inst_unimplemented, // 011 000
	&Jitter::inst_unimplemented, // 011 001
	&Jitter::inst_unimplemented, // 011 010
	&Jitter::inst_unimplemented, // 011 011
	&Jitter::inst_special2,      // 011 100
	&Jitter::inst_unimplemented, // 011 101
	&Jitter::inst_unimplemented, // 011 110
	&Jitter::inst_special3,      // 011 111
	&Jitter::inst_lb,            // 100 000
	&Jitter::inst_lh,            // 100 001
	&Jitter::inst_lwl,           // 100 010
	&Jitter::inst_lw,            // 100 011
	&Jitter::inst_lbu,           // 100 100
	&Jitter::inst_lhu,           // 100 101
	&Jitter::inst_lwr,           // 100 110
	&Jitter::inst_unimplemented, // 100 111
	&Jitter::inst_sb,            // 101 000
	&Jitter::inst_sh,            // 101 001
	&Jitter::inst_swl,           // 101 010
	&Jitter::inst_sw,            // 101 011
	&Jitter::inst_unimplemented, // 101 100
	&Jitter::inst_unimplemented, // 101 101
	&Jitter::inst_swr,           // 101 110
	&Jitter::inst_unimplemented, // 101 111
	&Jitter::inst_ll,            // 110 000
	&Jitter::inst_lwc1,          // 110 001
	&Jitter::inst_unimplemented, // 110 010
	&Jitter::inst_pref,          // 110 011
	&Jitter::inst_unimplemented, // 110 100
	&Jitter::inst_ldc1,          // 110 101
	&Jitter::inst_unimplemented, // 110 110
	&Jitter::inst_unimplemented, // 110 111
	&Jitter::inst_sc,            // 111 000
	&Jitter::inst_swc1,          // 111 001
	&Jitter::inst_unimplemented, // 111 010
	&Jitter::inst_unimplemented, // 111 011
	&Jitter::inst_unimplemented, // 111 100
	&Jitter::inst_sdc1,          // 111 101
	&Jitter::inst_unimplemented, // 111 110
	&Jitter::inst_unimplemented, // 111 111
};

// Type R instruction handlers indexed by functor
const Jitter::inst_handler_t Jitter::inst_handlers_R[] = {
	&Jitter::inst_sll,           // 000 000
	&Jitter::inst_unimplemented, // 000 001
	&Jitter::inst_srl,           // 000 010
	&Jitter::inst_sra,           // 000 011
	&Jitter::inst_sllv,          // 000 100
	&Jitter::inst_unimplemented, // 000 101
	&Jitter::inst_srlv,          // 000 110
	&Jitter::inst_srav,          // 000 111
	&Jitter::inst_jr,            // 001 000
	&Jitter::inst_jalr,          // 001 001
	&Jitter::inst_movz,          // 001 010
	&Jitter::inst_movn,          // 001 011
	&Jitter::inst_syscall,       // 001 100
	&Jitter::inst_break,         // 001 101
	&Jitter::inst_unimplemented, // 001 110
	&Jitter::inst_sync,          // 001 111
	&Jitter::inst_mfhi,          // 010 000
	&Jitter::inst_mthi,          // 010 001
	&Jitter::inst_mflo,          // 010 010
	&Jitter::inst_mtlo,          // 010 011
	&Jitter::inst_unimplemented, // 010 100
	&Jitter::inst_unimplemented, // 010 101
	&Jitter::inst_unimplemented, // 010 110
	&Jitter::inst_unimplemented, // 010 111
	&Jitter::inst_mult,          // 011 000
	&Jitter::inst_multu,         // 011 001
	&Jitter::inst_div,           // 011 010
	&Jitter::inst_divu,          // 011 011
	&Jitter::inst_unimplemented, // 011 100
	&Jitter::inst_unimplemented, // 011 101
	&Jitter::inst_unimplemented, // 011 110
	&Jitter::inst_unimplemented, // 011 111
	&Jitter::inst_add,           // 100 000
	&Jitter::inst_addu,          // 100 001
	&Jitter::inst_sub,           // 100 010
	&Jitter::inst_subu,          // 100 011
	&Jitter::inst_and,           // 100 100
	&Jitter::inst_or,            // 100 101
	&Jitter::inst_xor,           // 100 110
	&Jitter::inst_nor,           // 100 111
	&Jitter::inst_unimplemented, // 101 000
	&Jitter::inst_unimplemented, // 101 001
	&Jitter::inst_slt,           // 101 010
	&Jitter::inst_sltu,          // 101 011
	&Jitter::inst_unimplemented, // 101 100
	&Jitter::inst_unimplemented, // 101 101
	&Jitter::inst_unimplemented, // 101 110
	&Jitter::inst_unimplemented, // 101 111
	&Jitter::inst_unimplemented, // 110 000
	&Jitter::inst_unimplemented, // 110 001
	&Jitter::inst_unimplemented, // 110 010
	&Jitter::inst_unimplemented, // 110 011
	&Jitter::inst_teq,           // 110 100
	&Jitter::inst_unimplemented, // 110 101
	&Jitter::inst_unimplemented, // 110 110
	&Jitter::inst_unimplemented, // 110 111
	&Jitter::inst_unimplemented, // 111 000
	&Jitter::inst_unimplemented, // 111 001
	&Jitter::inst_unimplemented, // 111 010
	&Jitter::inst_unimplemented, // 111 011
	&Jitter::inst_unimplemented, // 111 100
	&Jitter::inst_unimplemented, // 111 101
	&Jitter::inst_unimplemented, // 111 110
	&Jitter::inst_unimplemented, // 111 111
};

// Type RI instruction handlers indexed by functor
const Jitter::inst_handler_t Jitter::inst_handlers_RI[] = {
	&Jitter::inst_bltz,          // 00 000
	&Jitter::inst_bgez,          // 00 001
	&Jitter::inst_unimplemented, // 00 010
	&Jitter::inst_unimplemented, // 00 011
	&Jitter::inst_unimplemented, // 00 100
	&Jitter::inst_unimplemented, // 00 101
	&Jitter::inst_unimplemented, // 00 110
	&Jitter::inst_unimplemented, // 00 111
	&Jitter::inst_unimplemented, // 01 000
	&Jitter::inst_unimplemented, // 01 001
	&Jitter::inst_unimplemented, // 01 010
	&Jitter::inst_unimplemented, // 01 011
	&Jitter::inst_unimplemented, // 01 100
	&Jitter::inst_unimplemented, // 01 101
	&Jitter::inst_unimplemented, // 01 110
	&Jitter::inst_unimplemented, // 01 111
	&Jitter::inst_unimplemented, // 10 000
	&Jitter::inst_bgezal,        // 10 001
	&Jitter::inst_unimplemented, // 10 010
	&Jitter::inst_unimplemented, // 10 011
	&Jitter::inst_unimplemented, // 10 100
	&Jitter::inst_unimplemented, // 10 101
	&Jitter::inst_unimplemented, // 10 110
	&Jitter::inst_unimplemented, // 10 111
	&Jitter::inst_unimplemented, // 11 000
	&Jitter::inst_unimplemented, // 11 001
	&Jitter::inst_unimplemented, // 11 010
	&Jitter::inst_unimplemented, // 11 011
	&Jitter::inst_unimplemented, // 11 100
	&Jitter::inst_unimplemented, // 11 101
	&Jitter::inst_unimplemented, // 11 110
	&Jitter::inst_unimplemented, // 11 111
};

// Type special2 instructions indexed by functor
const Jitter::inst_handler_t Jitter::inst_handlers_special2[] = {
	&Jitter::inst_unimplemented, // 000 000
	&Jitter::inst_unimplemented, // 000 001
	&Jitter::inst_mul,           // 000 010
	&Jitter::inst_unimplemented, // 000 011
	&Jitter::inst_unimplemented, // 000 100
	&Jitter::inst_unimplemented, // 000 101
	&Jitter::inst_unimplemented, // 000 110
	&Jitter::inst_unimplemented, // 000 111
	&Jitter::inst_unimplemented, // 001 000
	&Jitter::inst_unimplemented, // 001 001
	&Jitter::inst_unimplemented, // 001 010
	&Jitter::inst_unimplemented, // 001 011
	&Jitter::inst_unimplemented, // 001 100
	&Jitter::inst_unimplemented, // 001 101
	&Jitter::inst_unimplemented, // 001 110
	&Jitter::inst_unimplemented, // 001 111
	&Jitter::inst_unimplemented, // 010 000
	&Jitter::inst_unimplemented, // 010 001
	&Jitter::inst_unimplemented, // 010 010
	&Jitter::inst_unimplemented, // 010 011
	&Jitter::inst_unimplemented, // 010 100
	&Jitter::inst_unimplemented, // 010 101
	&Jitter::inst_unimplemented, // 010 110
	&Jitter::inst_unimplemented, // 010 111
	&Jitter::inst_unimplemented, // 011 000
	&Jitter::inst_unimplemented, // 011 001
	&Jitter::inst_unimplemented, // 011 010
	&Jitter::inst_unimplemented, // 011 011
	&Jitter::inst_unimplemented, // 011 100
	&Jitter::inst_unimplemented, // 011 101
	&Jitter::inst_unimplemented, // 011 110
	&Jitter::inst_unimplemented, // 011 111
	&Jitter::inst_clz,           // 100 000
	&Jitter::inst_unimplemented, // 100 001
	&Jitter::inst_unimplemented, // 100 010
	&Jitter::inst_unimplemented, // 100 011
	&Jitter::inst_unimplemented, // 100 100
	&Jitter::inst_unimplemented, // 100 101
	&Jitter::inst_unimplemented, // 100 110
	&Jitter::inst_unimplemented, // 100 111
	&Jitter::inst_unimplemented, // 101 000
	&Jitter::inst_unimplemented, // 101 001
	&Jitter::inst_unimplemented, // 101 010
	&Jitter::inst_unimplemented, // 101 011
	&Jitter::inst_unimplemented, // 101 100
	&Jitter::inst_unimplemented, // 101 101
	&Jitter::inst_unimplemented, // 101 110
	&Jitter::inst_unimplemented, // 101 111
	&Jitter::inst_unimplemented, // 110 000
	&Jitter::inst_unimplemented, // 110 001
	&Jitter::inst_unimplemented, // 110 010
	&Jitter::inst_unimplemented, // 110 011
	&Jitter::inst_unimplemented, // 110 100
	&Jitter::inst_unimplemented, // 110 101
	&Jitter::inst_unimplemented, // 110 110
	&Jitter::inst_unimplemented, // 110 111
	&Jitter::inst_unimplemented, // 111 000
	&Jitter::inst_unimplemented, // 111 001
	&Jitter::inst_unimplemented, // 111 010
	&Jitter::inst_unimplemented, // 111 011
	&Jitter::inst_unimplemented, // 111 100
	&Jitter::inst_unimplemented, // 111 101
	&Jitter::inst_unimplemented, // 111 110
	&Jitter::inst_unimplemented, // 111 111
};

const Jitter::inst_handler_t Jitter::inst_handlers_special3[] = {
	&Jitter::inst_ext,           // 000 000
	&Jitter::inst_unimplemented, // 000 001
	&Jitter::inst_unimplemented, // 000 010
	&Jitter::inst_unimplemented, // 000 011
	&Jitter::inst_ins,           // 000 100
	&Jitter::inst_unimplemented, // 000 101
	&Jitter::inst_unimplemented, // 000 110
	&Jitter::inst_unimplemented, // 000 111
	&Jitter::inst_unimplemented, // 001 000
	&Jitter::inst_unimplemented, // 001 001
	&Jitter::inst_unimplemented, // 001 010
	&Jitter::inst_unimplemented, // 001 011
	&Jitter::inst_unimplemented, // 001 100
	&Jitter::inst_unimplemented, // 001 101
	&Jitter::inst_unimplemented, // 001 110
	&Jitter::inst_unimplemented, // 001 111
	&Jitter::inst_unimplemented, // 010 000
	&Jitter::inst_unimplemented, // 010 001
	&Jitter::inst_unimplemented, // 010 010
	&Jitter::inst_unimplemented, // 010 011
	&Jitter::inst_unimplemented, // 010 100
	&Jitter::inst_unimplemented, // 010 101
	&Jitter::inst_unimplemented, // 010 110
	&Jitter::inst_unimplemented, // 010 111
	&Jitter::inst_unimplemented, // 011 000
	&Jitter::inst_unimplemented, // 011 001
	&Jitter::inst_unimplemented, // 011 010
	&Jitter::inst_unimplemented, // 011 011
	&Jitter::inst_unimplemented, // 011 100
	&Jitter::inst_unimplemented, // 011 101
	&Jitter::inst_unimplemented, // 011 110
	&Jitter::inst_unimplemented, // 011 111
	&Jitter::inst_bshfl,         // 100 000
	&Jitter::inst_unimplemented, // 100 001
	&Jitter::inst_unimplemented, // 100 010
	&Jitter::inst_unimplemented, // 100 011
	&Jitter::inst_unimplemented, // 100 100
	&Jitter::inst_unimplemented, // 100 101
	&Jitter::inst_unimplemented, // 100 110
	&Jitter::inst_unimplemented, // 100 111
	&Jitter::inst_unimplemented, // 101 000
	&Jitter::inst_unimplemented, // 101 001
	&Jitter::inst_unimplemented, // 101 010
	&Jitter::inst_unimplemented, // 101 011
	&Jitter::inst_unimplemented, // 101 100
	&Jitter::inst_unimplemented, // 101 101
	&Jitter::inst_unimplemented, // 101 110
	&Jitter::inst_unimplemented, // 101 111
	&Jitter::inst_unimplemented, // 110 000
	&Jitter::inst_unimplemented, // 110 001
	&Jitter::inst_unimplemented, // 110 010
	&Jitter::inst_unimplemented, // 110 011
	&Jitter::inst_unimplemented, // 110 100
	&Jitter::inst_unimplemented, // 110 101
	&Jitter::inst_unimplemented, // 110 110
	&Jitter::inst_unimplemented, // 110 111
	&Jitter::inst_unimplemented, // 111 000
	&Jitter::inst_unimplemented, // 111 001
	&Jitter::inst_unimplemented, // 111 010
	&Jitter::inst_rdhwr,         // 111 011
	&Jitter::inst_unimplemented, // 111 100
	&Jitter::inst_unimplemented, // 111 101
	&Jitter::inst_unimplemented, // 111 110
	&Jitter::inst_unimplemented, // 111 111
};

// Type COP1 instructions indexed by rs/fmt field
const Jitter::inst_handler_t Jitter::inst_handlers_COP1[] = {
	&Jitter::inst_mfc1,          // 00 000
	&Jitter::inst_unimplemented, // 00 001
	&Jitter::inst_cfc1,          // 00 010
	&Jitter::inst_mfhc1,         // 00 011
	&Jitter::inst_mtc1,          // 00 100
	&Jitter::inst_unimplemented, // 00 101
	&Jitter::inst_ctc1,          // 00 110
	&Jitter::inst_mthc1,         // 00 111
	&Jitter::inst_bc1,           // 01 000
	&Jitter::inst_unimplemented, // 01 001
	&Jitter::inst_unimplemented, // 01 010
	&Jitter::inst_unimplemented, // 01 011
	&Jitter::inst_unimplemented, // 01 100
	&Jitter::inst_unimplemented, // 01 101
	&Jitter::inst_unimplemented, // 01 110
	&Jitter::inst_unimplemented, // 01 111
	&Jitter::inst_fmt_s,         // 10 000
	&Jitter::inst_fmt_d,         // 10 001
	&Jitter::inst_unimplemented, // 10 010
	&Jitter::inst_unimplemented, // 10 011
	&Jitter::inst_fmt_w,         // 10 100
	&Jitter::inst_fmt_l,         // 10 101
	&Jitter::inst_fmt_ps,        // 10 110
	&Jitter::inst_unimplemented, // 10 111
	&Jitter::inst_unimplemented, // 11 000
	&Jitter::inst_unimplemented, // 11 001
	&Jitter::inst_unimplemented, // 11 010
	&Jitter::inst_unimplemented, // 11 011
	&Jitter::inst_unimplemented, // 11 100
	&Jitter::inst_unimplemented, // 11 101
	&Jitter::inst_unimplemented, // 11 110
	&Jitter::inst_unimplemented, // 11 111
};

bool Jitter::inst_test(vaddr_t pc, uint32_t inst){
	cout << "Test instruction" << endl;
	return false;
}

bool Jitter::inst_unimplemented(vaddr_t pc, uint32_t inst){
	die("Unimplemented instruction 0x%X at 0x%X\n", inst, pc-4);
	return false;
}

bool Jitter::inst_R(vaddr_t pc, uint32_t inst){
	// Get the functor and call the appropiate handler
	return (this->*inst_handlers_R[inst & 0b00111111])(pc, inst);
}

bool Jitter::inst_RI(vaddr_t pc, uint32_t inst){
	// Get the functor and call the appropiate handler
	return (this->*inst_handlers_RI[(inst >> 16) & 0b00011111])(pc, inst);
}

bool Jitter::inst_special2(vaddr_t pc, uint32_t inst){
	return (this->*inst_handlers_special2[inst & 0b00111111])(pc, inst);
}

bool Jitter::inst_special3(vaddr_t pc, uint32_t inst){
	return (this->*inst_handlers_special3[inst & 0b00111111])(pc, inst);
}

bool Jitter::inst_COP1(vaddr_t pc, uint32_t inst){
	return (this->*inst_handlers_COP1[(inst >> 21) & 0b00011111])(pc, inst);
}

bool Jitter::inst_or(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateOr(get_reg(inst.s), get_reg(inst.t)));
	return false;
}

bool Jitter::inst_bgezal(vaddr_t pc, uint32_t val){
	// Create cmp, set return address and handle delay slot
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);
	llvm::Value* cmp = builder.CreateICmpSGE(
		get_reg(inst.s),
		builder.getInt32(0),
		"cmp"
	);
	set_reg(Reg::ra, pc+4);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	if (END_COMPILING_ON_CALLS){
		// This jump is a call: finish compilation
		llvm::Value* reenter_pc = builder.CreateSelect(
			cmp,
			builder.getInt32(jump_addr),
			builder.getInt32(pc+4)
		);
		vm_exit(ExitInfo::ExitReason::Call, reenter_pc);
	} else {
		// Create both blocks, branch and finish compilation
		llvm::BasicBlock* true_block  = create_block(jump_addr);
		llvm::BasicBlock* false_block = create_block(pc+4);

		builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	}
	return true;
}

bool Jitter::inst_nop(vaddr_t pc, uint32_t val){
	return false;
}

bool Jitter::inst_lui(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, inst.C << 16);
	return false;
}

bool Jitter::inst_addiu(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* v = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	set_reg(inst.t, builder.CreateAdd(get_reg(inst.s), v));
	return false;
}

bool Jitter::inst_lw(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	set_reg(inst.t, read_mem(addr, 4, pc));
	return false;
}

bool Jitter::inst_and(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateAnd(get_reg(inst.s), get_reg(inst.t)));
	return false;
}

bool Jitter::inst_sw(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	write_mem(addr, get_reg(inst.t), 4, pc);
	return false;
}

bool Jitter::inst_jalr(vaddr_t pc, uint32_t val){
	// Calculate jump addr and set return address
	inst_R_t inst(val);
	llvm::Value* jump_addr = get_reg(inst.s);
	set_reg(inst.d, pc+4);

	// Handle delay slot
	handle_inst(pc);

	// Record coverage
	if (add_coverage(builder.getInt32(pc), jump_addr))
		return true;

	// Generate indirect branch and finish compilation
	vm_exit(ExitInfo::ExitReason::IndirectBranch, jump_addr);
	return true;
}

bool Jitter::inst_beq(vaddr_t pc, uint32_t val){
	// Create cmp, handle delay slot
	inst_I_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);
	llvm::Value* cmp = builder.CreateICmpEQ(
		get_reg(inst.s),
		get_reg(inst.t),
		"cmp"
	);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	// Create both blocks, branch and finish compilation
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_addu(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateAdd(get_reg(inst.t), get_reg(inst.s)));
	return false;
}

bool Jitter::inst_sll(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* shamt = builder.getInt32(inst.S);
	set_reg(inst.d, builder.CreateShl(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_bne(vaddr_t pc, uint32_t val){
	// Create cmp, handle delay slot
	inst_I_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);
	llvm::Value* cmp = builder.CreateICmpNE(
		get_reg(inst.s),
		get_reg(inst.t),
		"cmp"
	);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	// Create both blocks, branch and finish compilation
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_jr(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* jump_addr = get_reg(inst.s);

	// Handle delay slot first
	handle_inst(pc);

	// Record coverage
	if (add_coverage(builder.getInt32(pc), jump_addr))
		return true;

	// Generate indirect branch and finish compilation
	vm_exit(ExitInfo::ExitReason::IndirectBranch, jump_addr);
	return true;
}

bool Jitter::inst_lhu(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* value = read_mem(addr, 2, pc);
	set_reg(inst.t, value);
	return false;
}

bool Jitter::inst_syscall(vaddr_t pc, uint32_t val){
	if (!INTEGRATED_CALLS){
		// Generate vm exit and handle syscall from outside the VM
		llvm::Value* reenter_pc = builder.getInt32(pc);
		vm_exit(ExitInfo::ExitReason::Syscall, reenter_pc);
		return true;
	} else {
		// Call handle_syscall from the JIT, no need to vm exit.
		// If TMP_REGS, register saving is done at the end by gen_save_regs
		llvm::Value* emu_ptr = &function->arg_begin()[3];
		llvm::FunctionType* handle_syscall_ty = llvm::FunctionType::get(
			int1_ty,
			{
				int8ptr_ty, // pointer to emulator
				int32_ty,   // syscall number
			},
			false
		);
		llvm::FunctionCallee handle_syscall_func =
			module.getOrInsertFunction("handle_syscall", handle_syscall_ty);

		// Function handle_syscall returns whether we should exit or not
		// Getting reg v0 should be free because most times it will be a
		// constant
		llvm::Value* exit =
			builder.CreateCall(handle_syscall_func, {emu_ptr, get_reg(Reg::v0)});
		llvm::BasicBlock* continue_path =
			llvm::BasicBlock::Create(context, "continue", function);
		builder.Insert(llvm::BranchInst::Create(end_path, continue_path, exit));

		builder.SetInsertPoint(continue_path);
		if (TMP_REGS){
			// Load registers (unconditionally)
			load_reg(Reg::v0);
			load_reg(Reg::a3);
		}
		return false;
	}
}

bool Jitter::inst_xor(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateXor(get_reg(inst.s), get_reg(inst.t)));
	return false;
}

bool Jitter::inst_sltu(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpULT(get_reg(inst.s), get_reg(inst.t));
	set_reg(inst.d, cmp);
	return false;
}

bool Jitter::inst_sltiu(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = builder.getInt32((int16_t)inst.C);
	llvm::Value* cmp = builder.CreateICmpULT(get_reg(inst.s), value);
	set_reg(inst.t, cmp);
	return false;
}

bool Jitter::inst_movn(vaddr_t pc, uint32_t val){
	// Hope this shit gets optimized: `rd = (rt != 0 ? rs : rd)`
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpNE(
		get_reg(inst.t),
		builder.getInt32(0)
	);
	llvm::Value* sel = builder.CreateSelect(
		cmp,
		get_reg(inst.s),
		get_reg(inst.d)
	);
	set_reg(inst.d, sel);
	return false;
}

bool Jitter::inst_movz(vaddr_t pc, uint32_t val){
	// Hope this shit gets optimized: `rd = (rt == 0 ? rs : rd)`
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpEQ(
		get_reg(inst.t),
		builder.getInt32(0)
	);
	llvm::Value* sel = builder.CreateSelect(
		cmp,
		get_reg(inst.s),
		get_reg(inst.d)
	);
	set_reg(inst.d, sel);
	return false;
}

bool Jitter::inst_subu(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateSub(get_reg(inst.s), get_reg(inst.t)));
	return false;
}

bool Jitter::inst_teq(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::BasicBlock* trap_path =
		llvm::BasicBlock::Create(context, "trap_path", function);
	llvm::BasicBlock* normal_path =
		llvm::BasicBlock::Create(context, "normal_path", function);
	llvm::Value* cmp = builder.CreateICmpEQ(get_reg(inst.s), get_reg(inst.t));
	builder.Insert(llvm::BranchInst::Create(trap_path, normal_path, cmp));
	
	builder.SetInsertPoint(trap_path);
	llvm::Value* reenter_pc = builder.getInt32(pc);
	vm_exit(ExitInfo::ExitReason::Exception, reenter_pc);

	builder.SetInsertPoint(normal_path);
	return false;
}

bool Jitter::inst_div(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* dividend = get_reg(inst.s);
	llvm::Value* divisor  = get_reg(inst.t);
	set_reg(Reg::hi, builder.CreateSRem(dividend, divisor));
	set_reg(Reg::lo, builder.CreateSDiv(dividend, divisor));
	return false;
}

bool Jitter::inst_divu(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* dividend = get_reg(inst.s);
	llvm::Value* divisor  = get_reg(inst.t);
	set_reg(Reg::hi, builder.CreateURem(dividend, divisor));
	set_reg(Reg::lo, builder.CreateUDiv(dividend, divisor));
	return false;
}

bool Jitter::inst_mflo(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(Reg::lo));
	return false;
}

bool Jitter::inst_mul(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateMul(get_reg(inst.s), get_reg(inst.t)));
	return false;
}

bool Jitter::inst_bltz(vaddr_t pc, uint32_t val){
	// Create cmp, handle delay slot
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);
	llvm::Value* cmp = builder.CreateICmpSLT(
		get_reg(inst.s),
		llvm::ConstantInt::get(int32_ty, 0, true),
		"cmp"
	);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	// Create both blocks, branch and finish compilation
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_blez(vaddr_t pc, uint32_t val){
	// Create cmp, handle delay slot
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);
	llvm::Value* cmp = builder.CreateICmpSLE(
		get_reg(inst.s),
		llvm::ConstantInt::get(int32_ty, 0, true),
		"cmp"
	);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	// Create both blocks, branch and finish compilation
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_rdhwr(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	if (!INTEGRATED_CALLS){
		// Generate vm exit and handle rdhwr from outside the VM
		llvm::Value* reenter_pc = builder.getInt32(pc);
		vm_exit(
			ExitInfo::ExitReason::Rdhwr,
			reenter_pc,
			builder.getInt32(inst.d),
			builder.getInt32(inst.t)
		);
		return true;
	} else {
		// Call handle_rdhwr from the JIT, no need to vm exit
		llvm::Value* emu_ptr = &function->arg_begin()[3];
		llvm::FunctionType* handle_rdhwr_ty = llvm::FunctionType::get(
			void_ty,
			{
				int8ptr_ty, // pointer to emulator
				int8_ty,    // hwr
				int8_ty     // reg
			},
			false
		);
		llvm::FunctionCallee handle_rdhwr_func =
			module.getOrInsertFunction("handle_rdhwr", handle_rdhwr_ty);
		builder.CreateCall(handle_rdhwr_func, {
			emu_ptr,
			builder.getInt8(inst.d),
			builder.getInt8(inst.t)
		});
		if (TMP_REGS)
			load_reg(inst.t);
		return false;
	}
}

bool Jitter::inst_bgez(vaddr_t pc, uint32_t val){
	// Create cmp, handle delay slot
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);
	llvm::Value* cmp = builder.CreateICmpSGE(
		get_reg(inst.s),
		builder.getInt32(0),
		"cmp"
	);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	// Create both blocks, branch and finish compilation
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_slti(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* cmp   = builder.CreateICmpSLT(get_reg(inst.s), value);
	set_reg(inst.t, cmp);
	return false;
}

bool Jitter::inst_andi(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = builder.getInt32(inst.C);
	set_reg(inst.t, builder.CreateAnd(get_reg(inst.s), value));
	return false;
}

bool Jitter::inst_ori(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = builder.getInt32(inst.C);
	set_reg(inst.t, builder.CreateOr(get_reg(inst.s), value));
	return false;
}

bool Jitter::inst_xori(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = builder.getInt32(inst.C);
	set_reg(inst.t, builder.CreateXor(get_reg(inst.s), value));
	return false;
}

bool Jitter::inst_pref(vaddr_t pc, uint32_t val){
	return false;
}

bool Jitter::inst_jal(vaddr_t pc, uint32_t val){
	// Calculate jump addr and set return address
	inst_J_t inst(val);
	vaddr_t jump_addr = (inst.A << 2) | (pc & 0xF0000000);
	set_reg(Reg::ra, pc+4);

	// Handle delay slot
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr))
		return true;

	if (END_COMPILING_ON_CALLS){
		// This jump is a call: finish compilation
		vm_exit(ExitInfo::ExitReason::Call, builder.getInt32(jump_addr));
	} else {
		// Create block, branch and finish compilation
		llvm::BasicBlock* true_block  = create_block(jump_addr);

		builder.Insert(llvm::BranchInst::Create(true_block));
	}
	return true;
}

bool Jitter::inst_lb(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* value = read_mem(addr, 1, pc); // type: i8
	llvm::Value* value_sext = builder.CreateSExt(value, int32_ty); // type: i32
	set_reg(inst.t, value_sext);
	return false;
}

bool Jitter::inst_nor(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateNot(builder.CreateOr(get_reg(inst.s),
	                                                   get_reg(inst.t))));
	return false;
}

bool Jitter::inst_bshfl(vaddr_t pc, uint32_t val){
	switch ((val >> 6) & 0b11111){
		case 0b00010: // wsbh
			return inst_wsbh(pc, val);
		case 0b10000: // seb
			return inst_seb(pc, val);
		case 0b11000: // seh
			return inst_seh(pc, val);
		default:
			die("Unimplemented bshfl instruction at 0x%X: 0x%X\n", pc-4, val);
	}
	return false;
}

bool Jitter::inst_seh(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* reg = get_reg(inst.t);
	set_reg(inst.d, 
	        builder.CreateSExt(builder.CreateTrunc(reg, int16_ty), int32_ty));
	return false;
}

bool Jitter::inst_seb(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* reg = get_reg(inst.t);
	set_reg(inst.d, 
	        builder.CreateSExt(builder.CreateTrunc(reg, int8_ty), int32_ty));
	return false;
}

bool Jitter::inst_wsbh(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* v     = get_reg(inst.t);
	llvm::Value* int8  = builder.getInt32(8);
	llvm::Value* mask1 = builder.getInt32(0xFF000000);
	llvm::Value* mask2 = builder.getInt32(0x00FF0000);
	llvm::Value* mask3 = builder.getInt32(0x0000FF00);
	llvm::Value* mask4 = builder.getInt32(0x000000FF);
	llvm::Value* tmp1  = builder.CreateOr(
			builder.CreateLShr(builder.CreateAnd(v, mask1), int8),
			builder.CreateShl (builder.CreateAnd(v, mask2), int8)
	);
	llvm::Value* tmp2  = builder.CreateOr(
			builder.CreateLShr(builder.CreateAnd(v, mask3), int8),
			builder.CreateShl (builder.CreateAnd(v, mask4), int8)
	);
	llvm::Value* res   = builder.CreateOr(tmp1, tmp2);
	set_reg(inst.d, res);
	return false;
}

bool Jitter::inst_srl(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* shamt = builder.getInt32(inst.S);
	set_reg(inst.d, builder.CreateLShr(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_lh(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* value = read_mem(addr, 2, pc); // type: i16
	llvm::Value* value_sext = builder.CreateSExt(value, int32_ty); // type: i32
	set_reg(inst.t, value_sext);
	return false;
}

bool Jitter::inst_lbu(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* value = read_mem(addr, 1, pc);
	set_reg(inst.t, value);
	return false;
}

bool Jitter::inst_lwl(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* offset = builder.CreateAnd(addr, builder.getInt32(3));
	llvm::Value* addr_align = builder.CreateSub(addr, offset);

	// This is the same as in `read_mem`
	if (DETAILED_FAULT)
		builder.CreateStore(builder.getInt32(pc-4), p_fault_pc);
	check_bounds_mem(addr_align, 4);
	uint8_t perm = Mmu::PERM_READ;
	if (CHECK_UNINIT)
		perm |= Mmu::PERM_INIT;
	check_perms_mem(addr_align, 4, perm);

	// Perform memcpy
	llvm::Value* p_mem = get_pmemory(addr_align);
	llvm::Value* p_reg = get_preg(inst.t);
	builder.CreateMemCpy(
		builder.CreateInBoundsGEP(
			builder.CreateBitCast(p_reg, int8ptr_ty),
			builder.CreateSub(builder.getInt32(3), offset)),
		4,     // dst align
		p_mem, // src
		1,     // src align
		builder.CreateAdd(offset, builder.getInt32(1))
	);
	return false;
}

bool Jitter::inst_lwr(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* offset = builder.CreateAnd(addr, builder.getInt32(3));
	llvm::Value* addr_align = builder.CreateSub(addr, offset);

	// This is the same as in `read_mem`. We use addr_align despite reading from
	// addr because we don't know the length of the read at compile time, so we
	// just check everything on the whole aligned word.
	if (DETAILED_FAULT)
		builder.CreateStore(builder.getInt32(pc-4), p_fault_pc);
	check_bounds_mem(addr_align, 4);
	uint8_t perm = Mmu::PERM_READ;
	if (CHECK_UNINIT)
		perm |= Mmu::PERM_INIT;
	check_perms_mem(addr_align, 4, perm);

	// Perform memcpy
	llvm::Value* p_mem = get_pmemory(addr);
	llvm::Value* p_reg = get_preg(inst.t);
	builder.CreateMemCpy(
		p_reg, // dst
		4,     // dst align
		p_mem, // src
		1,     // src align
		builder.CreateSub(builder.getInt32(4), offset)
	);
	return false;
}

bool Jitter::inst_sb(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	write_mem(addr, get_reg(inst.t), 1, pc);
	return false;
}

bool Jitter::inst_sh(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	write_mem(addr, get_reg(inst.t), 2, pc);
	return false;
}

bool Jitter::inst_swl(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* offset = builder.CreateAnd(addr, builder.getInt32(3));
	llvm::Value* addr_align = builder.CreateSub(addr, offset);

	// This is the same as in `write_mem`
	if (DETAILED_FAULT)
		builder.CreateStore(builder.getInt32(pc-4), p_fault_pc);
	check_bounds_mem(addr_align, 4);
	check_perms_mem (addr_align, 4, Mmu::PERM_WRITE);
	if (CHECK_UNINIT)
		set_init(addr_align, 4);
	set_dirty_mem(addr_align); // we're writing to addr_align

	// Perform memcpy
	llvm::Value* p_mem = get_pmemory(addr_align);
	llvm::Value* p_reg = get_preg(inst.t);
	builder.CreateMemCpy(
		p_mem, // dst
		4,     // dst align
		builder.CreateInBoundsGEP(
			builder.CreateBitCast(p_reg, int8ptr_ty),
			builder.CreateSub(builder.getInt32(3), offset)
		),     // src
		1,     // src align
		builder.CreateAdd(offset, builder.getInt32(1))
	);
	return false;
}

bool Jitter::inst_swr(vaddr_t pc, uint32_t val){
	// Locurote. Traduced from Emulator::inst_swr
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* offset = builder.CreateAnd(addr, builder.getInt32(3));
	llvm::Value* addr_align = builder.CreateSub(addr, offset);

	// Instead of reading the reg, reading the value from memory, moving part of
	// the reg to the value and writing back to memory, just change the value in
	// memory. Some repeated code but I think it's worth it
	// TODO: NOT SETTING DIRTY??

	// This is the same as in `write_mem`. We use addr_align despite writing to
	// addr because we don't know the length of the write at compile time, so we
	// just check everything on the whole aligned word.
	if (DETAILED_FAULT)
		builder.CreateStore(builder.getInt32(pc-4), p_fault_pc);
	check_bounds_mem(addr_align, 4);
	check_perms_mem (addr_align, 4, Mmu::PERM_WRITE);
	if (CHECK_UNINIT)
		set_init(addr_align, 4);
	set_dirty_mem(addr); // we're writing to addr

	// Perform memcpy
	llvm::Value* p_mem = get_pmemory(addr);
	llvm::Value* p_reg = get_preg(inst.t);
 	builder.CreateMemCpy(
		p_mem, // dst
		1,     // dst align
		p_reg, // src
		4,     // src align
		builder.CreateSub(builder.getInt32(4), offset)
	);
	return false;
}

bool Jitter::inst_sllv(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* mask  = builder.getInt32(0b00011111);
	llvm::Value* shamt = builder.CreateAnd(get_reg(inst.s), mask);
	set_reg(inst.d, builder.CreateShl(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_srlv(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* mask  = builder.getInt32(0b00011111);
	llvm::Value* shamt = builder.CreateAnd(get_reg(inst.s), mask);
	set_reg(inst.d, builder.CreateLShr(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_slt(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpSLT(
		get_reg(inst.s),
		get_reg(inst.t)
	);
	set_reg(inst.d, cmp);
	return false;
}

bool Jitter::inst_sub(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_add(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_j(vaddr_t pc, uint32_t val){
	// Calculate jump address and handle delay slot
	inst_J_t inst(val);
	vaddr_t jump_addr = (inst.A << 2) | (pc & 0xF0000000);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr))
		return true;

	// This jump is a call: finish compilation.
	if (END_COMPILING_ON_CALLS){
		// It seems to be used as a tail call or as a call with no return
		vm_exit(ExitInfo::ExitReason::Call, builder.getInt32(jump_addr));
	} else {
		// Create block, branch and finish compilation
		llvm::BasicBlock* true_block = create_block(jump_addr);

		builder.Insert(llvm::BranchInst::Create(true_block));
	}

	return true;
}

bool Jitter::inst_ll(vaddr_t pc, uint32_t val){
	// Forget about atomics for now
	inst_lw(pc, val);
	return false;
}

bool Jitter::inst_sc(vaddr_t pc, uint32_t val){
	// Forget about atomics for now
	inst_sw(pc, val);
	inst_I_t inst(val);
	set_reg(inst.t, 1);
	return false;
}

bool Jitter::inst_sync(vaddr_t pc, uint32_t val){
	// Forget about atomics for now
	return false;
}

bool Jitter::inst_bgtz(vaddr_t pc, uint32_t val){
	// Create cmp, handle delay slot
	inst_I_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);
	llvm::Value* cmp = builder.CreateICmpSGT(
		get_reg(inst.s),
		builder.getInt32(0),
		"cmp"
	);
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	// Create both blocks, branch and finish compilation
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_mult(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* reg1 = builder.CreateSExt(get_reg(inst.s), int64_ty);
	llvm::Value* reg2 = builder.CreateSExt(get_reg(inst.t), int64_ty);
	llvm::Value* res  = builder.CreateMul(reg1, reg2);
	
	llvm::Value* i32 = llvm::ConstantInt::get(int64_ty, 32);
	set_reg(Reg::lo, builder.CreateTrunc(res, int32_ty));
	set_reg(Reg::hi, builder.CreateTrunc(builder.CreateLShr(res, i32), int32_ty));
	return false;
}

bool Jitter::inst_multu(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* reg1 = builder.CreateZExt(get_reg(inst.s), int64_ty);
	llvm::Value* reg2 = builder.CreateZExt(get_reg(inst.t), int64_ty);
	llvm::Value* res  = builder.CreateMul(reg1, reg2);
	
	llvm::Value* i32 = llvm::ConstantInt::get(int64_ty, 32);
	set_reg(Reg::lo, builder.CreateTrunc(res, int32_ty));
	set_reg(Reg::hi, builder.CreateTrunc(builder.CreateLShr(res, i32), int32_ty));
	return false;
}

bool Jitter::inst_mfhi(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(Reg::hi));
	return false;
}

bool Jitter::inst_mthi(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(Reg::hi, get_reg(inst.s));
	return false;
}

bool Jitter::inst_mtlo(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(Reg::lo, get_reg(inst.s));
	return false;
}

bool Jitter::inst_ext(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	uint32_t size = inst.d + 1;
	llvm::Value* lsb  = builder.getInt32(inst.S);
	llvm::Value* mask = builder.getInt32((1 << size) - 1);
	set_reg(inst.t, 
	        builder.CreateAnd(builder.CreateLShr(get_reg(inst.s), lsb), mask));
	return false;
}

bool Jitter::inst_ins(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	uint32_t size = inst.d - inst.S + 1;
	llvm::Value* lsb  = builder.getInt32(inst.S);
	llvm::Value* mask = builder.getInt32((1 << size)-1);
	
	// Get bits to insert, place them at the correct position
	llvm::Value* ins  = builder.CreateAnd(get_reg(inst.s), mask);
	llvm::Value* ins2 = builder.CreateShl(ins, lsb);

	// Insert bits into reg (and & or)
	llvm::Value* reg    = get_reg(inst.t);
	llvm::Value* result = builder.CreateAnd(builder.CreateOr(reg, ins2), ins2);
	set_reg(inst.t, result);
	
	return false;
}

bool Jitter::inst_sra(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* shamt = builder.getInt32(inst.S);
	set_reg(inst.d, builder.CreateAShr(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_srav(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* mask  = builder.getInt32(0b00011111);
	llvm::Value* shamt = builder.CreateAnd(get_reg(inst.s), mask);
	set_reg(inst.d, builder.CreateAShr(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_clz(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Function* ctlz_i32 = llvm::Intrinsic::getDeclaration(
		p_module.get(),
		llvm::Intrinsic::ctlz,
	    {int32_ty}
	);
	llvm::ConstantInt* fals3 = llvm::ConstantInt::getFalse(context);
	set_reg(inst.d, builder.CreateCall(ctlz_i32, {get_reg(inst.s), fals3}));
	return false;
}

bool Jitter::inst_lwc1(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* v    = as_float(read_mem(addr, 4, pc));
	sets_reg(inst.t, v);
	return false;
}

bool Jitter::inst_swc1(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* v    = as_u32(gets_reg(inst.t));
	write_mem(addr, v, 4, pc);
	return false;
}

bool Jitter::inst_ldc1(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* v    = builder.CreateBitCast(read_mem(addr, 8, pc), double_ty);
	setd_reg(inst.t, v);
	return false;
}

bool Jitter::inst_sdc1(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* v    = builder.CreateBitCast(getd_reg(inst.t), int64_ty);
	write_mem(addr, v, 8, pc);
	return false;
}

bool Jitter::inst_fmt_s(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	if ((inst.funct & 0b110000) == 0b110000){
		// C.cond.s
		inst_c_cond_s(pc, val);
	} else switch (inst.funct){
		case 0b000000: // add.s
			sets_reg(inst.d,
			         builder.CreateFAdd(gets_reg(inst.s), gets_reg(inst.t)));
			break;
		case 0b000001: // sub.s
			sets_reg(inst.d,
			         builder.CreateFSub(gets_reg(inst.s), gets_reg(inst.t)));
			break; // Once, this break didn't exist. This comment is here just
			// to pay respect for those hours spent debugging. F.
		case 0b000010: // mul.s
			sets_reg(inst.d,
			         builder.CreateFMul(gets_reg(inst.s), gets_reg(inst.t)));
			break;
		case 0b000011: // div.s
			sets_reg(inst.d,
			         builder.CreateFDiv(gets_reg(inst.s), gets_reg(inst.t)));
			break;
		case 0b000110: // mov.s
			sets_reg(inst.d, gets_reg(inst.s));
			break;
		case 0b001101: // trunc.w.s
			// Convert float to u32, then use `as_float` to use `sets_reg`
			sets_reg(inst.d,
			         as_float(builder.CreateFPToUI(gets_reg(inst.s),int32_ty)));
			break;
		case 0b010011: { // movn.s
			// Hope this shit gets optimized
			llvm::Value* cmp = builder.CreateICmpNE(
				get_reg(inst.t),
				builder.getInt32(0)
			);
			sets_reg(inst.d, builder.CreateSelect(
				cmp,
				gets_reg(inst.s),
				gets_reg(inst.d)
			));
			break;
		}
		case 0b100001: // cvt.d.s
			setd_reg(inst.d, builder.CreateFPExt(gets_reg(inst.s), double_ty));
			break;
		default:
			die("Unimplemented fmt s at 0x%X\n", pc-4);
	}
	return false;
}

bool Jitter::inst_fmt_d(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	if ((inst.funct & 0b110000) == 0b110000){
		// C.cond.d
		inst_c_cond_d(pc, val);
	} else switch (inst.funct){
		case 0b000000: // add.d
			setd_reg(inst.d,
			         builder.CreateFAdd(getd_reg(inst.s), getd_reg(inst.t)));
			break;
		case 0b000001: // sub.d
			setd_reg(inst.d,
			         builder.CreateFSub(getd_reg(inst.s), getd_reg(inst.t)));
			break;
		case 0b000010: // mul.d
			setd_reg(inst.d,
			         builder.CreateFMul(getd_reg(inst.s), getd_reg(inst.t)));
			break;
		case 0b000011: // div.d
			setd_reg(inst.d,
			         builder.CreateFDiv(getd_reg(inst.s), getd_reg(inst.t)));
			break;
		case 0b000110: // mov.d
			setd_reg(inst.d, getd_reg(inst.s));
			break;
		case 0b001101: // trunc.w.d
			// Convert double to u32, then use `as_float` to use `sets_reg`
			sets_reg(inst.d,
			         as_float(builder.CreateFPToUI(getd_reg(inst.s), int32_ty)));
			break;
		case 0b010011:{ // movn.s
			// Hope this shit gets optimized
			llvm::Value* cmp = builder.CreateICmpNE(
				get_reg(inst.t),
				builder.getInt32(0)
			);
			setd_reg(inst.d, builder.CreateSelect(
				cmp,
				getd_reg(inst.s),
				getd_reg(inst.d)
			));
			break;
		}
		case 0b100000: // cvt.s.d
			sets_reg(inst.d, builder.CreateFPTrunc(getd_reg(inst.s), float_ty));
			break;
		default:
			die("Unimplemented fmt d at 0x%X\n", pc-4);
	}
	return false;
}

bool Jitter::inst_fmt_w(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	if ((inst.funct & 0b110000) == 0b110000){
		// C.cond.w doesn't exist
		die("NANI fmt w\n");
	} else switch (inst.funct){
		case 0b100000: // cvt.s.w
			// There's an u32 in FPR s, and we must convert it to float
			sets_reg(inst.d, 
			         builder.CreateUIToFP(as_u32(gets_reg(inst.s)), float_ty));
			break;
		case 0b100001: // cvt.d.w
			// There's an u32 in FPR s, and we must convert it to double
			setd_reg(inst.d, 
			         builder.CreateUIToFP(as_u32(gets_reg(inst.s)), double_ty));
			break;
		default:
			die("Unimplemented fmt w at 0x%X: %d\n", pc-4, inst.funct);
	}
	return false;
}

bool Jitter::inst_fmt_l(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_fmt_ps(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_mfc1(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	// Get lower 32 bits of the 64bits fpu register
	set_reg(inst.t, as_u32(gets_reg(inst.s)));
	return false;
}

bool Jitter::inst_mfhc1(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	// Get upper 32 bits of the 64bits fpu register
	set_reg(inst.t, as_u32(gets_reg(inst.s+1)));
	return false;
}

bool Jitter::inst_mtc1(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	sets_reg(inst.s, as_float(get_reg(inst.t)));
	return false;
}

bool Jitter::inst_mthc1(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	sets_reg(inst.s+1, as_float(get_reg(inst.t)));
	return false;
}

bool Jitter::inst_c_cond_s(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	uint8_t cond = inst.funct & 0b001111;
	uint8_t cc   = (inst.d >> 2) & 0b111;
	llvm::Value* val1 = gets_reg(inst.s);
	llvm::Value* val2 = gets_reg(inst.t);
	
	// LLVM IR has the 'fcmp' instruction which performs exactly what we want.
	// God bless LLVM
	switch (cond){
		case 0b0001: // un
			set_cc(cc, builder.CreateFCmpUNO(val1, val2));
			break;
		case 0b0010: // eq
			set_cc(cc, builder.CreateFCmpOEQ(val1, val2));
			break;
		case 0b0111: // ule
			set_cc(cc, builder.CreateFCmpULE(val1, val2));
			break;
		case 0b1100: // lt
			set_cc(cc, builder.CreateFCmpOLT(val1, val2));
			break;
		case 0b1110: // le
			set_cc(cc, builder.CreateFCmpOLE(val1, val2));
			break;
		default:
			die("Unimplemented c.cond.d at 0x%X: %X\n", pc-4, val);
	}

	return false;
}

bool Jitter::inst_c_cond_d(vaddr_t pc, uint32_t val){
	inst_F_t inst(val);
	uint8_t cond = inst.funct & 0b001111;
	uint8_t cc   = (inst.d >> 2) & 0b111;
	llvm::Value* val1 = getd_reg(inst.s);
	llvm::Value* val2 = getd_reg(inst.t);
	
	// LLVM IR has the 'fcmp' instruction which performs exactly what we want.
	// God bless LLVM
	switch (cond){
		case 0b0001: // un
			set_cc(cc, builder.CreateFCmpUNO(val1, val2));
			break;
		case 0b0010: // eq
			set_cc(cc, builder.CreateFCmpOEQ(val1, val2));
			break;
		case 0b0111: // ule
			set_cc(cc, builder.CreateFCmpULE(val1, val2));
			break;
		case 0b1100: // lt
			set_cc(cc, builder.CreateFCmpOLT(val1, val2));
			break;
		case 0b1110: // le
			set_cc(cc, builder.CreateFCmpOLE(val1, val2));
			break;
		default:
			die("Unimplemented c.cond.d at 0x%X: %X\n", pc-4, val);
	}

	return false;
}

bool Jitter::inst_bc1(vaddr_t pc, uint32_t val){
	// Create cmp, handle delay slot
	inst_I_t inst(val);
	bool jump_if_true = inst.t & 1;
	uint8_t cc = (inst.t >> 2) & 0b111;
	vaddr_t jump_addr = pc + ((int16_t)inst.C * 4);

	llvm::Value* cond = get_cc(cc);
	llvm::Value* cmp  = (jump_if_true ? cond : builder.CreateNot(cond));
	handle_inst(pc);

	// Record coverage
	if (add_coverage(pc, jump_addr, pc+4, cmp))
		return true;

	// Create both blocks, branch and finish compilation
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_cfc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	return false;
}

bool Jitter::inst_ctc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	return false;
}

bool Jitter::inst_break(vaddr_t pc, uint32_t val){
	llvm::Value* reenter_pc = builder.getInt32(pc);
	vm_exit(ExitInfo::ExitReason::Exception, reenter_pc);
	return true;
}