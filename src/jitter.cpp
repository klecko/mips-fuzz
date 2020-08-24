#include <iostream>
#include <sstream>
#include <fstream>
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Verifier.h"
#include "llvm/ExecutionEngine/ExecutionEngine.h"
#include "llvm/ExecutionEngine/SectionMemoryManager.h"
#include "llvm/ExecutionEngine/Orc/CompileUtils.h"
#include "llvm/Support/TargetSelect.h"
#include "jitter.h"
#include "inst_decoding.h"

using namespace std;

/*struct jit_state {
	uint32_t* regs;
	float*    fpregs;
	uint8_t*  memory;
};*/

Jitter::Jitter(vaddr_t pc, const Mmu& _mmu):
	mmu(_mmu), module("module", context), builder(context)
{
	// PC in hex
	ostringstream oss;
	oss << hex << pc;
	string pc_s(oss.str());

	// Get types
	llvm::Type* void_ty      = llvm::Type::getVoidTy(context);
	llvm::Type* p_int32_ty   = llvm::Type::getInt32PtrTy(context);
	llvm::Type* p_float_ty   = llvm::Type::getFloatPtrTy(context);
	llvm::Type* p_int8_ty    = llvm::Type::getInt8PtrTy(context);
	llvm::Type* jit_state_ty = llvm::StructType::get(
		context,
		{
			p_int32_ty,
			p_float_ty,
			p_int8_ty,
		}
	);
	llvm::Type* p_jit_state_ty = llvm::PointerType::get(jit_state_ty, 0);

	// Create function
	llvm::FunctionType* function_type = llvm::FunctionType::get(
		void_ty,          // ret
		{p_jit_state_ty}, // args
		false             // varargs
	);
	function = llvm::Function::Create(
		function_type,
		llvm::Function::ExternalLinkage,
		"func_" + pc_s,
		module
	);

	// Create basic blocks recursively
	llvm::BasicBlock* bb = create_block(pc);

	// JIT module
	if (llvm::verifyModule(module, &llvm::errs())){
		llvm::errs() << module;
		die("bad module\n");
	}

	code = compile(module);

	llvm::errs() << module;
}

string Jitter::compile(llvm::Module& module){
	// Write LLVM to disk
	error_code err;
	llvm::raw_fd_ostream os("module.ll", err, llvm::sys::fs::F_None);
	os << module;
	os.close();

	// Compile LLVM
	system("opt module.ll -O3 -S -o module.ll.opt");
	system("llc module.ll.opt -relocation-model=pic -filetype obj -O3 -o module.o");
	system("ld -shared module.o -o module");
	system("objcopy module --dump-section .text=module.text");

	// Read compiled LLVM
	ostringstream buf;
	ifstream is("module.text");
	buf << is.rdbuf();
	return buf.str();
}

llvm::Value* Jitter::get_preg(uint8_t reg){
	llvm::Type* int32_ty = llvm::Type::getInt32Ty(context);
	llvm::Value* state = &function->arg_begin()[0];
	llvm::Value* pp_regs = builder.CreateInBoundsGEP(
		state,
		{
			llvm::ConstantInt::get(int32_ty, 0), // *state
			llvm::ConstantInt::get(int32_ty, 0), // state->regs
		},
		"pp_regs"
	);
	llvm::Value* p_regs = builder.CreateLoad(pp_regs, "p_regs");
	llvm::Value* p_reg = builder.CreateInBoundsGEP(
		p_regs,
		{ llvm::ConstantInt::get(int32_ty, reg) },
		"p_reg" + to_string(reg)
	);
	return p_reg;
}

llvm::Value* Jitter::get_reg(uint8_t reg){
	llvm::Value* reg_val;
	if (reg == 0)
		reg_val = llvm::ConstantInt::get(context, llvm::APInt(32, 0));
	else {
		llvm::Value* p_reg = get_preg(reg);
		reg_val = builder.CreateLoad(p_reg, "reg" + to_string(reg));
	}
	return reg_val;
}

void Jitter::set_reg(uint8_t reg, llvm::Value* val){
	if (reg != 0){
		llvm::Value* p_reg = get_preg(reg);
		builder.CreateStore(val, p_reg);
	}
}

void Jitter::set_reg(uint8_t reg, uint32_t val){
	llvm::Value* v = llvm::ConstantInt::get(context, llvm::APInt(32, val));
	set_reg(reg, v);
}

llvm::BasicBlock* Jitter::create_block(vaddr_t pc){
	// Check if block was already created
	if (basic_blocks.count(pc))
		return basic_blocks[pc];

	// PC in hex
	ostringstream oss;
	oss << hex << pc;
	string pc_s(oss.str());

	// Save previous basic block
	llvm::BasicBlock* prev_block = builder.GetInsertBlock();

	// Create new block and register it
	llvm::BasicBlock* block = 
		llvm::BasicBlock::Create(context, "block_" + pc_s, function);
	basic_blocks[pc] = block;

	builder.SetInsertPoint(block);

	bool end = false;
	while (!end){
		end = handle_inst(pc);
		pc += 4;
	}

	// Set builder to previous basic block and return new block
	builder.SetInsertPoint(prev_block);
	return block;
}

bool Jitter::handle_inst(vaddr_t pc){
	// Fetch current instruction
	uint32_t inst   = mmu.read_inst(pc);
	uint8_t  opcode = (inst >> 26) & 0b111111;

 	if (inst)
		return (this->*inst_handlers[opcode])(pc+4, inst);

	return false;
}

string Jitter::get_code(){
	return code;
}




bool Jitter::inst_test(vaddr_t pc, uint32_t inst){
	cout << "Test instruction" << endl;
	return false;
}

bool Jitter::inst_unimplemented(vaddr_t pc, uint32_t inst){
	die("Unimplemented instruction 0x%X\n", inst);
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
	// Handle delay slot first
	handle_inst(pc);

	// Set return address
	set_reg(Reg::ra, pc+4);

	// Create both blocks and finish compilation
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpSGE(
		get_reg(inst.s),
		llvm::ConstantInt::get(context, llvm::APInt(32, 0)),
		"cmp"
	);
	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
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
	llvm::Value* v  =
		llvm::ConstantInt::get(context, llvm::APInt(32, (int16_t)inst.C, true));
	set_reg(inst.t, builder.CreateAdd(get_reg(inst.s), v));
	return false;
}

bool Jitter::inst_lw(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	return false;
	//llvm::errs() << module;
	//exit(0);
}

bool Jitter::inst_and(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateAnd(get_reg(inst.s), get_reg(inst.t)));
	return false;
}

bool Jitter::inst_sw(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	return false;
}

bool Jitter::inst_jalr(vaddr_t pc, uint32_t val){
	// Handle delay slot first
	handle_inst(pc);

	// Set return address
	set_reg(Reg::ra, pc+4);

	inst_RI_t inst(val);
	builder.CreateRetVoid();
	return true;
}

bool Jitter::inst_beq(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_addu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sll(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_bne(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_jr(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_lhu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_syscall(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_xor(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sltu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sltiu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_movn(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_movz(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_subu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_teq(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_div(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_divu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mflo(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mul(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_bltz(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_blez(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_rdhwr(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_bgez(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_slti(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_andi(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_ori(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_xori(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_pref(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_jal(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_lb(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_nor(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_bshfl(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_seh(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_seb(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_wsbh(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_srl(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_lh(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_lbu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_lwl(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_lwr(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sb(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sh(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_swl(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_swr(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sllv(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_srlv(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_slt(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sub(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_add(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_j(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_ll(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sc(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sync(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_bgtz(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mult(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_multu(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mfhi(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mthi(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mtlo(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_ext(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sra(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_clz(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_lwc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_swc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_ldc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_sdc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_fmt_s(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_fmt_d(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_fmt_w(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_fmt_l(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_fmt_ps(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mfc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mfhc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mtc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_mthc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_c_cond_s(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_c_cond_d(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_bc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}

bool Jitter::inst_cfc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s\n", __func__);
	llvm::errs() << module;
	exit(0);
}


// Instruction handlers indexed by opcode
const inst_handler_jit_t Jitter::inst_handlers[] = {
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
const inst_handler_jit_t Jitter::inst_handlers_R[] = {
	&Jitter::inst_sll,           // 000 000
	&Jitter::inst_unimplemented, // 000 001
	&Jitter::inst_srl,           // 000 010
	&Jitter::inst_sra,           // 000 011
	&Jitter::inst_sllv,          // 000 100
	&Jitter::inst_unimplemented, // 000 101
	&Jitter::inst_srlv,          // 000 110
	&Jitter::inst_unimplemented, // 000 111
	&Jitter::inst_jr,            // 001 000
	&Jitter::inst_jalr,          // 001 001
	&Jitter::inst_movz,          // 001 010
	&Jitter::inst_movn,          // 001 011
	&Jitter::inst_syscall,       // 001 100
	&Jitter::inst_unimplemented, // 001 101
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
const inst_handler_jit_t Jitter::inst_handlers_RI[] = {
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
const inst_handler_jit_t Jitter::inst_handlers_special2[] = {
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

const inst_handler_jit_t Jitter::inst_handlers_special3[] = {
	&Jitter::inst_ext,           // 000 000
	&Jitter::inst_unimplemented, // 000 001
	&Jitter::inst_unimplemented, // 000 010
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
const inst_handler_jit_t Jitter::inst_handlers_COP1[] = {
	&Jitter::inst_mfc1,          // 00 000
	&Jitter::inst_unimplemented, // 00 001
	&Jitter::inst_cfc1,          // 00 010
	&Jitter::inst_mfhc1,         // 00 011
	&Jitter::inst_mtc1,          // 00 100
	&Jitter::inst_unimplemented, // 00 101
	&Jitter::inst_unimplemented, // 00 110
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