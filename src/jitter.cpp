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

llvm::Type* void_ty;
llvm::Type* int8_ty;
llvm::Type* int16_ty;
llvm::Type* int32_ty;
llvm::Type* int64_ty;
llvm::Type* p_int32_ty;
llvm::Type* p_float_ty;
llvm::Type* p_int8_ty;

/*struct jit_state {
	uint32_t* regs;
	float*    fpregs;
	uint8_t*  memory;
};*/

Jitter::Jitter(vaddr_t pc, const Mmu& _mmu):
	mmu(_mmu), module("module", context), builder(context)
{
	printf("starting jit 0x%X\n", pc);
	// PC in hex
	ostringstream oss;
	oss << hex << pc;
	string pc_s(oss.str());

	// Get types
	void_ty      = llvm::Type::getVoidTy(context);
	int8_ty      = llvm::Type::getInt8Ty(context);
	int16_ty     = llvm::Type::getInt16Ty(context);
	int32_ty     = llvm::Type::getInt32Ty(context);
	int64_ty     = llvm::Type::getInt64Ty(context);
	p_int32_ty   = llvm::Type::getInt32PtrTy(context);
	p_float_ty   = llvm::Type::getFloatPtrTy(context);
	p_int8_ty    = llvm::Type::getInt8PtrTy(context);

	llvm::Type* jit_state_ty = llvm::StructType::get(context,
		{
			p_int32_ty, // regs
			p_float_ty, // fpregs
			p_int8_ty,  // memory
			p_int8_ty,  // perms
		}
	);
	llvm::Type* p_jit_state_ty = llvm::PointerType::get(jit_state_ty, 0);

	llvm::Type* exit_info_ty = llvm::StructType::get(context,
		{
			int32_ty, // reason
			int32_ty, // pc
			int32_ty, // info1
			int32_ty, // info2
		}
	);
	llvm::Type* p_exit_info_ty = llvm::PointerType::get(exit_info_ty, 0);

	// Create function
	llvm::FunctionType* function_type = llvm::FunctionType::get(
		void_ty,          // ret
		{p_jit_state_ty, p_exit_info_ty}, // args
		false             // varargs
	);
	function = llvm::Function::Create(
		function_type,
		llvm::Function::ExternalLinkage,
		"func_" + pc_s,
		module
	);

	// Create entry basic block
	llvm::BasicBlock* entry_block = llvm::BasicBlock::Create(context, "entry", function);

	// Create basic blocks recursively
	llvm::BasicBlock* bb = create_block(pc);

	// Jump from entry to `bb`. LLVM doesn't allow jumping to the first basic
	// block of a function. This way `bb` is the second basic block and we can
	// jump to it
	llvm::BranchInst::Create(bb, entry_block);

	// JIT module
	if (llvm::verifyModule(module, &llvm::errs())){
		llvm::errs() << module;
		die("bad module\n");
	}

	code = compile(module);
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
		"p_reg" + to_string(reg) + "_"
	);
	return p_reg;
}

llvm::Value* Jitter::get_reg(uint8_t reg){
	assert(0 <= reg && reg <= 33);
	llvm::Value* reg_val;
	if (reg == 0)
		reg_val = llvm::ConstantInt::get(int32_ty, 0);
	else {
		llvm::Value* p_reg = get_preg(reg);
		reg_val = builder.CreateLoad(p_reg, "reg" + to_string(reg) + "_");
	}
	return reg_val;
}

void Jitter::set_reg(uint8_t reg, llvm::Value* val){
	// WARNING: we may need to zero extend val
	assert(0 <= reg && reg <= 33);
	llvm::Value* val_zext = builder.CreateZExt(val, int32_ty);
	if (reg != 0){
		llvm::Value* p_reg = get_preg(reg);
		builder.CreateStore(val_zext, p_reg);
	}
}

void Jitter::set_reg(uint8_t reg, uint32_t val){
	llvm::Value* v = llvm::ConstantInt::get(int32_ty, val);
	set_reg(reg, v);
}

llvm::Value* Jitter::get_pmemory(llvm::Value* addr){
	llvm::Value* state = &function->arg_begin()[0];
	llvm::Value* pp_memory = builder.CreateInBoundsGEP(
		state,
		{
			llvm::ConstantInt::get(int32_ty, 0), // *state
			llvm::ConstantInt::get(int32_ty, 2), // state->memory
		},
		"pp_memory"
	);
	llvm::Value* p_memory = builder.CreateLoad(pp_memory, "p_memory");
	llvm::Value* p_value = builder.CreateInBoundsGEP(
		p_memory,
		{ addr },
		"p_value"
	);
	return p_value;
}

llvm::Type* get_int_ptr_ty(vsize_t sz, llvm::LLVMContext& context){
	llvm::Type* type;
	switch (sz){
		case 1:
			type = llvm::Type::getInt8PtrTy(context);
			break;
		case 2:
			type = llvm::Type::getInt16PtrTy(context);
			break;
		case 4:
			type = llvm::Type::getInt32PtrTy(context);
			break;
		default:
			die("unsupported ptr size: %d\n", sz);
	}
	return type;
}

void Jitter::gen_vm_exit(exit_info::ExitReason reason, llvm::Value* reenter_pc,
                         llvm::Value* info1, llvm::Value* info2)
{
	llvm::Value* p_exit_info = &function->arg_begin()[1];
	llvm::Value* p_exit_reason = builder.CreateInBoundsGEP(
		p_exit_info,
		{
			llvm::ConstantInt::get(int32_ty, 0),
			llvm::ConstantInt::get(int32_ty, 0),
		},
		"p_exit_reason"
	);
	llvm::Value* p_pc = builder.CreateInBoundsGEP(
		p_exit_info,
		{
			llvm::ConstantInt::get(int32_ty, 0),
			llvm::ConstantInt::get(int32_ty, 1),
		},
		"p_pc"
	);
	llvm::Value* p_info1 = builder.CreateInBoundsGEP(
		p_exit_info,
		{
			llvm::ConstantInt::get(int32_ty, 0),
			llvm::ConstantInt::get(int32_ty, 2),
		},
		"p_info1_"
	);
	llvm::Value* p_info2 = builder.CreateInBoundsGEP(
		p_exit_info,
		{
			llvm::ConstantInt::get(int32_ty, 0),
			llvm::ConstantInt::get(int32_ty, 3),
		},
		"p_info2_"
	);
	builder.CreateStore(llvm::ConstantInt::get(int32_ty, reason), p_exit_reason);
	builder.CreateStore(reenter_pc, p_pc);
	builder.CreateStore(info1 ? info1 : llvm::ConstantInt::get(int32_ty, 0), p_info1);
	builder.CreateStore(info2 ? info2 : llvm::ConstantInt::get(int32_ty, 0), p_info2);
	builder.CreateRetVoid();
}

void Jitter::check_bounds_mem(llvm::Value* addr, vsize_t len, uint8_t perm,
                              vaddr_t pc)
{
	// Get fault type in case we have go the fault path
	Fault::Type fault_type;
	switch (perm){
		case Mmu::PERM_READ:
			fault_type = Fault::Type::OutOfBoundsRead;
			break;
		case Mmu::PERM_WRITE:
			fault_type = Fault::Type::OutOfBoundsWrite;
			break;
		case Mmu::PERM_EXEC:
			fault_type = Fault::Type::OutOfBoundsExec;
			break;
		default:
			die("Wrong permissions check_bounds\n");
	}

	// Create both paths, compare and branch
	llvm::BasicBlock* fault_path =
		llvm::BasicBlock::Create(context, "fault_bounds", function);
	llvm::BasicBlock* nofault_path =
		llvm::BasicBlock::Create(context, "nofault_bounds", function);
	llvm::Value* memsize = llvm::ConstantInt::get(int32_ty, mmu.size());
	llvm::Value* last_addr = builder.CreateAdd(
		addr,
		llvm::ConstantInt::get(int32_ty, len),
		"last_addr"
	);
	llvm::Value* cmp = builder.CreateICmpUGT(last_addr, memsize, "cmp_bounds");
	builder.Insert(llvm::BranchInst::Create(fault_path, nofault_path, cmp));

	// Fault path
	builder.SetInsertPoint(fault_path);
	gen_vm_exit(
		exit_info::ExitReason::Fault,
		llvm::ConstantInt::get(int32_ty, pc-4),
		llvm::ConstantInt::get(int32_ty, fault_type),
		addr
	);

	// Continue building nofault path
	builder.SetInsertPoint(nofault_path);
}

void Jitter::check_perms_mem(llvm::Value* addr, vsize_t len, uint8_t perm,
                             vaddr_t pc)
{
	llvm::Type* mask_ty = llvm::Type::getIntNTy(context, len*8);
	llvm::Type* ptr_ty  = get_int_ptr_ty(len, context);

	// Get fault type in case we have go the fault path
	Fault::Type fault_type;
	switch (perm){
		case Mmu::PERM_READ:
			fault_type = Fault::Type::Read;
			break;
		case Mmu::PERM_WRITE:
			fault_type = Fault::Type::Write;
			break;
		case Mmu::PERM_EXEC:
			fault_type = Fault::Type::Exec;
			break;
		default:
			die("Wrong permissions check_perms\n");
	}

	// Compute permission mask for given len
	uint32_t perms_mask = 0;
	for (int i = 0; i < len; i++)
		perms_mask |= (perm << (i*8));
	llvm::Value* perms_mask_val = llvm::ConstantInt::get(mask_ty, perms_mask);

	// Create both paths
	llvm::BasicBlock* fault_path =
		llvm::BasicBlock::Create(context, "fault_perms", function);
	llvm::BasicBlock* nofault_path =
		llvm::BasicBlock::Create(context, "nofault_perms", function);

	// Get pointer to permissions value, cast it and load its value
	llvm::Value* state = &function->arg_begin()[0];
	llvm::Value* pp_perms = builder.CreateInBoundsGEP(
		state,
		{
			llvm::ConstantInt::get(int32_ty, 0), // *state
			llvm::ConstantInt::get(int32_ty, 3), // state->perms
		},
		"pp_perms"
	);
	llvm::Value* p_perms = builder.CreateLoad(pp_perms, "p_perms");
	llvm::Value* p_perm  = builder.CreateInBoundsGEP(
		p_perms,
		{ addr },
		"p_perm"
	);
	llvm::Value* p_perm_cast =
		builder.CreateBitCast(p_perm, ptr_ty, "p_perm_cast");
	llvm::Value* perm_val = builder.CreateLoad(p_perm_cast, "perm");

	// And it with perm_masks, check if they're equal and branch
	llvm::Value* result = builder.CreateAnd(perm_val, perms_mask_val);
	llvm::Value* cmp = builder.CreateICmpNE(result, perms_mask_val, "cmp_perms");
	builder.Insert(llvm::BranchInst::Create(fault_path, nofault_path, cmp));

	// Fault path
	builder.SetInsertPoint(fault_path);
	gen_vm_exit(
		exit_info::ExitReason::Fault,
		llvm::ConstantInt::get(int32_ty, pc-4),
		llvm::ConstantInt::get(int32_ty, fault_type),
		addr
	);

	// Continue building nofault path
	builder.SetInsertPoint(nofault_path);
}

void Jitter::check_alignment_mem(llvm::Value* addr, vsize_t len, uint8_t perm,
                                 vaddr_t pc)
{
	// Get fault type in case we have go the fault path
	Fault::Type fault_type;
	switch (perm){
		case Mmu::PERM_READ:
			fault_type = Fault::Type::MisalignedRead;
			break;
		case Mmu::PERM_WRITE:
			fault_type = Fault::Type::MisalignedWrite;
			break;
		case Mmu::PERM_EXEC:
			fault_type = Fault::Type::MisalignedExec;
			break;
		default:
			die("Wrong permissions check_alignment\n");
	}

	// Create both paths
	llvm::BasicBlock* fault_path =
		llvm::BasicBlock::Create(context, "fault_align", function);
	llvm::BasicBlock* nofault_path =
		llvm::BasicBlock::Create(context, "nofault_align", function);

	// Check alignment and branch
	llvm::Value* addr_masked = builder.CreateAnd(
		addr,
		llvm::ConstantInt::get(int32_ty, len-1)
	);
	llvm::Value* cmp = builder.CreateICmpNE(
		addr_masked,
		llvm::ConstantInt::get(int32_ty, 0),
		"cmp_align"
	);
	builder.Insert(llvm::BranchInst::Create(fault_path, nofault_path, cmp));

	// Fault path
	builder.SetInsertPoint(fault_path);
	gen_vm_exit(
		exit_info::ExitReason::Fault,
		llvm::ConstantInt::get(int32_ty, pc-4),
		llvm::ConstantInt::get(int32_ty, fault_type),
		addr
	);

	// Continue building nofault path
	builder.SetInsertPoint(nofault_path);
}

llvm::Value* Jitter::read_mem(llvm::Value* addr, vsize_t len, vaddr_t pc){
	// Check out of bounds
	check_bounds_mem(addr, len, Mmu::PERM_READ, pc);

	// Check permissions
	check_perms_mem(addr, len, Mmu::PERM_READ, pc);

	// Check alignment
	check_alignment_mem(addr, len, Mmu::PERM_READ, pc);

	// Get pointer to memory position, cast it from i8* to iN* and read from it
	llvm::Value* p_value      = get_pmemory(addr);
	llvm::Type*  cast_type    = get_int_ptr_ty(len, context);
	llvm::Value* p_value_cast = builder.CreateBitCast(p_value, cast_type);
	return builder.CreateLoad(p_value_cast);
}

void Jitter::write_mem(llvm::Value* addr, llvm::Value* value, vsize_t len,
                       vaddr_t pc)
{
	// Check out of bounds
	check_bounds_mem(addr, len, Mmu::PERM_WRITE, pc);

	// Check permissions
	check_perms_mem(addr, len, Mmu::PERM_WRITE, pc);

	// Check alignment
	check_alignment_mem(addr, len, Mmu::PERM_WRITE, pc);

	// Get pointer to memory position, cast it from i8* to iN*, cast value from
	// ?? to iN, and write value to pointer
	llvm::Value* p_value      = get_pmemory(addr);
	llvm::Type*  cast_type_p  = get_int_ptr_ty(len, context);
	llvm::Type*  cast_type_v  = llvm::Type::getIntNTy(context, len*8);
	llvm::Value* p_value_cast = builder.CreateBitCast(p_value, cast_type_p);
	llvm::Value* value_cast   = builder.CreateTrunc(value, cast_type_v);
	builder.CreateStore(value_cast, p_value_cast);
}

llvm::BasicBlock* Jitter::create_block(vaddr_t pc){
	// Check if block was already created
	if (basic_blocks.count(pc))
		return basic_blocks[pc];

	//printf("jitting 0x%X\n", pc);

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
	// Handle delay slot first
	handle_inst(pc);

	// Set return address
	set_reg(Reg::ra, pc+4);

	// Create both blocks, branch and finish compilation
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpSGE(
		get_reg(inst.s),
		llvm::ConstantInt::get(int32_ty, 0),
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
	inst_R_t inst(val);

	// Handle delay slot first
	handle_inst(pc);

	// Set return address
	set_reg(inst.d, pc+4);

	// Generate indirect branch and finish compilation
	gen_vm_exit(exit_info::ExitReason::IndirectBranch, get_reg(inst.s));
	return true;
}

bool Jitter::inst_beq(vaddr_t pc, uint32_t val){
	// Handle delay slot first
	handle_inst(pc);

	// Create both blocks, branch and finish compilation
	inst_I_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpEQ(
		get_reg(inst.s),
		get_reg(inst.t),
		"cmp"
	);
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
	llvm::Value* shamt = llvm::ConstantInt::get(int32_ty, inst.S);
	set_reg(inst.d, builder.CreateShl(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_bne(vaddr_t pc, uint32_t val){
	// Handle delay slot first
	handle_inst(pc);

	// Create both blocks, branch and finish compilation
	inst_I_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpNE(
		get_reg(inst.s),
		get_reg(inst.t),
		"cmp"
	);
	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_jr(vaddr_t pc, uint32_t val){
	// Handle delay slot first
	handle_inst(pc);

	// Generate indirect branch and finish compilation
	inst_R_t inst(val);
	gen_vm_exit(exit_info::ExitReason::IndirectBranch, get_reg(inst.s));
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
	llvm::Value* reenter_pc = llvm::ConstantInt::get(int32_ty, pc);
	gen_vm_exit(exit_info::ExitReason::Syscall, reenter_pc);
	return true;
}

bool Jitter::inst_xor(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, builder.CreateXor(get_reg(inst.s), get_reg(inst.t)));
	return false;
}

bool Jitter::inst_sltu(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpULT(get_reg(inst.s), get_reg(inst.t));
	/* llvm::Value* sel = builder.CreateSelect(
		cmp,
		llvm::ConstantInt::get(context, llvm::APInt(32, 1)),
		llvm::ConstantInt::get(int32_ty, 0)
	);
	set_reg(inst.d, sel); */
	set_reg(inst.d, cmp);
	return false;
}

bool Jitter::inst_sltiu(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C);
	llvm::Value* cmp = builder.CreateICmpULT(get_reg(inst.s), value);
	/* llvm::Value* sel = builder.CreateSelect(
		cmp,
		llvm::ConstantInt::get(context, llvm::APInt(32, 1)),
		llvm::ConstantInt::get(int32_ty, 0)
	);
	set_reg(inst.t, sel); */
	set_reg(inst.t, cmp);
	return false;
}

bool Jitter::inst_movn(vaddr_t pc, uint32_t val){
	// Hope this shit gets optimized
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpNE(
		get_reg(inst.t),
		llvm::ConstantInt::get(int32_ty, 0)
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
	// Hope this shit gets optimized
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpEQ(
		get_reg(inst.t),
		llvm::ConstantInt::get(int32_ty, 0)
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
	llvm::Value* reenter_pc = llvm::ConstantInt::get(int32_ty, pc);
	gen_vm_exit(exit_info::ExitReason::Exception, reenter_pc);

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
	// Handle delay slot first
	handle_inst(pc);

	// Create both blocks, branch and finish compilation
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpSLT(
		get_reg(inst.s),
		llvm::ConstantInt::get(int32_ty, 0, true),
		"cmp"
	);
	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_blez(vaddr_t pc, uint32_t val){
	// Handle delay slot first
	handle_inst(pc);

	// Create both blocks, branch and finish compilation
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpSLE(
		get_reg(inst.s),
		llvm::ConstantInt::get(int32_ty, 0, true),
		"cmp"
	);
	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_rdhwr(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* reenter_pc = llvm::ConstantInt::get(int32_ty, pc);
	gen_vm_exit(
		exit_info::ExitReason::Rdhwr,
		reenter_pc, 
		llvm::ConstantInt::get(int32_ty, inst.d), 
		llvm::ConstantInt::get(int32_ty, inst.t)
	);
	return true;
}

bool Jitter::inst_bgez(vaddr_t pc, uint32_t val){
	// Handle delay slot first
	handle_inst(pc);

	// Create both blocks, branch and finish compilation
	inst_RI_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpSGE(
		get_reg(inst.s),
		llvm::ConstantInt::get(int32_ty, 0),
		"cmp"
	);
	builder.Insert(llvm::BranchInst::Create(true_block, false_block, cmp));
	return true;
}

bool Jitter::inst_slti(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* cmp   = builder.CreateICmpSLT(get_reg(inst.s), value);
	/* llvm::Value* sel = builder.CreateSelect(
		cmp,
		llvm::ConstantInt::get(context, llvm::APInt(32, 1)),
		llvm::ConstantInt::get(int32_ty, 0)
	);
	set_reg(inst.t, sel); */
	set_reg(inst.t, cmp);
	return false;
}

bool Jitter::inst_andi(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = llvm::ConstantInt::get(int32_ty, inst.C);
	set_reg(inst.t, builder.CreateAnd(get_reg(inst.s), value));
	return false;
}

bool Jitter::inst_ori(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = llvm::ConstantInt::get(int32_ty, inst.C);
	set_reg(inst.t, builder.CreateOr(get_reg(inst.s), value));
	return false;
}

bool Jitter::inst_xori(vaddr_t pc, uint32_t val){
	inst_I_t inst(val);
	llvm::Value* value = llvm::ConstantInt::get(int32_ty, inst.C);
	set_reg(inst.t, builder.CreateXor(get_reg(inst.s), value));
	return false;
}

bool Jitter::inst_pref(vaddr_t pc, uint32_t val){
	return false;
}

bool Jitter::inst_jal(vaddr_t pc, uint32_t val){
	// Handle delay slot first
	handle_inst(pc);

	// Save return address
	set_reg(Reg::ra, pc+4);

	// Create block, branch and finish compilation
	inst_J_t inst(val);
	vaddr_t jump_addr = (inst.A << 2) | (pc & 0xF0000000);
	llvm::BasicBlock* true_block = create_block(jump_addr);

	builder.Insert(llvm::BranchInst::Create(true_block));
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
			break;
		case 0b10000: // seb
			return inst_seb(pc, val);
			break;
		case 0b11000: // seh
			return inst_seh(pc, val);
			break;
		default:
			die("Unimplemented bshfl instruction at 0x%X: 0x%X\n", pc-4, val);
	}
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
	llvm::Value* int8  = llvm::ConstantInt::get(int32_ty, 8);
	llvm::Value* mask1 = llvm::ConstantInt::get(int32_ty, 0xFF000000);
	llvm::Value* mask2 = llvm::ConstantInt::get(int32_ty, 0x00FF0000);
	llvm::Value* mask3 = llvm::ConstantInt::get(int32_ty, 0x0000FF00);
	llvm::Value* mask4 = llvm::ConstantInt::get(int32_ty, 0x000000FF);
	llvm::Value* res = builder.CreateOr({
		builder.CreateLShr(builder.CreateAnd(v, mask1), int8),
		builder.CreateShl (builder.CreateAnd(v, mask2), int8),
		builder.CreateLShr(builder.CreateAnd(v, mask3), int8),
		builder.CreateShl (builder.CreateAnd(v, mask4), int8)
	});
	set_reg(inst.d, res);
	return false;
}

bool Jitter::inst_srl(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* shamt = llvm::ConstantInt::get(int32_ty, inst.S);
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
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	return false;
}

bool Jitter::inst_lwr(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
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
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	return false;
}

bool Jitter::inst_swr(vaddr_t pc, uint32_t val){
	// Locurote. Traduced from Emulator::inst_swr
/* 	inst_I_t inst(val);
	llvm::Value* offs = llvm::ConstantInt::get(int32_ty, (int16_t)inst.C, true);
	llvm::Value* addr = builder.CreateAdd(get_reg(inst.s), offs, "addr");
	llvm::Value* offset =
		builder.CreateAnd(addr, llvm::ConstantInt::get(int32_ty, 3)); */

	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	return false;
}

bool Jitter::inst_sllv(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* mask  = llvm::ConstantInt::get(int32_ty, 0b00011111);
	llvm::Value* shamt = builder.CreateAnd(get_reg(inst.s), mask);
	set_reg(inst.d, builder.CreateShl(get_reg(inst.d), shamt));
	return false;
}

bool Jitter::inst_srlv(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* mask  = llvm::ConstantInt::get(int32_ty, 0b00011111);
	llvm::Value* shamt = builder.CreateAnd(get_reg(inst.s), mask);
	set_reg(inst.d, builder.CreateLShr(get_reg(inst.d), shamt));
	return false;
}

bool Jitter::inst_slt(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* cmp = builder.CreateICmpSLT(
		get_reg(inst.s),
		get_reg(inst.t)
	);
	/* llvm::Value* sel = builder.CreateSelect(
		cmp,
		llvm::ConstantInt::get(context, llvm::APInt(32, 1)),
		llvm::ConstantInt::get(int32_ty, 0)
	);
	set_reg(inst.t, sel); */
	set_reg(inst.t, cmp);
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
	// Handle delay slot first
	handle_inst(pc);

	// Create block, branch and finish compilation
	inst_J_t inst(val);
	vaddr_t jump_addr = (inst.A << 2) | (pc & 0xF0000000);
	llvm::BasicBlock* true_block = create_block(jump_addr);

	builder.Insert(llvm::BranchInst::Create(true_block));
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
	// Handle delay slot first
	handle_inst(pc);

	// Create both blocks, branch and finish compilation
	inst_I_t inst(val);
	vaddr_t jump_addr = pc + ((int16_t)inst.C << 2);
	llvm::BasicBlock* true_block  = create_block(jump_addr);
	llvm::BasicBlock* false_block = create_block(pc+4);

	llvm::Value* cmp = builder.CreateICmpSGT(
		get_reg(inst.s),
		llvm::ConstantInt::get(int32_ty, 0),
		"cmp"
	);
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
	set_reg(Reg::hi, get_reg(inst.d));
	return false;
}

bool Jitter::inst_mtlo(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	set_reg(Reg::lo, get_reg(inst.d));
	return false;
}

bool Jitter::inst_ext(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* lsb  = llvm::ConstantInt::get(int32_ty, inst.S);
	llvm::Value* size = llvm::ConstantInt::get(int32_ty, inst.d + 1);
	llvm::Value* mask = llvm::ConstantInt::get(int32_ty, (1 << (inst.d+1)) - 1);
	set_reg(inst.t, 
	        builder.CreateAnd(builder.CreateLShr(get_reg(inst.s), lsb), mask));
	return false;
}

bool Jitter::inst_ins(vaddr_t pc, uint32_t val){
	inst_R_t inst(val);
	llvm::Value* lsb  = llvm::ConstantInt::get(int32_ty, inst.S);
	llvm::Value* size = llvm::ConstantInt::get(int32_ty, inst.d - inst.S + 1);
	llvm::Value* mask = llvm::ConstantInt::get(int32_ty, (1 << (inst.d-inst.S+1))-1);
	
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
	llvm::Value* shamt = llvm::ConstantInt::get(int32_ty, inst.S);
	set_reg(inst.d, builder.CreateLShr(get_reg(inst.t), shamt));
	return false;
}

bool Jitter::inst_clz(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_lwc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_swc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_ldc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	return false;
}

bool Jitter::inst_sdc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	return false;
}

bool Jitter::inst_fmt_s(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_fmt_d(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_fmt_w(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
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
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_mfhc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_mtc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_mthc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_c_cond_s(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_c_cond_d(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_bc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_cfc1(vaddr_t pc, uint32_t val){
	printf("unimplemented %s at 0x%X\n", __func__, pc-4);
	exit(0);
}

bool Jitter::inst_break(vaddr_t pc, uint32_t val){
	llvm::Value* reenter_pc = llvm::ConstantInt::get(int32_ty, pc);
	gen_vm_exit(exit_info::ExitReason::Exception, reenter_pc);
	return true;
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

const char* exit_reason_map[] = {
	"syscall", "fault", "indirect branch", "rdhwr", "exception",
};
ostream& operator<<(ostream& os, const exit_info& exit_inf){
	assert(exit_inf.reason < sizeof(exit_reason_map)/sizeof(const char*));
	os << "Exit reason: " << exit_reason_map[exit_inf.reason]
	   << "; reenter pc: 0x" << hex << exit_inf.reenter_pc;
	if (exit_inf.reason == exit_info::ExitReason::Fault)
		os << "; Fault: " << Fault((Fault::Type)exit_inf.info1, exit_inf.info2);
	return os;
}