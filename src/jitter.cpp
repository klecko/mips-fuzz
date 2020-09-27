#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <unordered_set>
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Object/ObjectFile.h"
#include "llvm/LinkAllPasses.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/TargetRegistry.h"
#include "jitter.h"

#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include "llvm/Analysis/TargetTransformInfo.h"

// Attempt to give a unique coverage id to each branch.
// When disabled, branch hashes are used as coverage ids, same as the
// interpreter. When enabled, branch hashes are only used for indirect branches.
// The rest of the branches are given a constant unique coverage id when
// jitting, slightly improving performance and reducing collisions.
// On readelf with a cov map size of 64KB, it reduces collisions from 2.5%
// to 1.5%
// TODO: think how to reduce collisions even more
//       (somehow unique ids to indirect branches)
#define UNIQUE_COV_ID_ATTEMPT 1

using namespace std;
using namespace JIT;

uint32_t load_next_cov_id(){
	uint32_t result;
	ifstream is("./jitcache/next_cov_id");
	if (!is.good())
		result = 0;
	else
		is >> result;
	is.close();
	return result;
}
uint32_t Jitter::next_cov_id = load_next_cov_id();
unordered_map<vaddr_t, std::pair<uint32_t, uint32_t>> Jitter::cov_ids;
std::mutex Jitter::mutex;

struct jit_init {
	jit_init(){
		llvm::InitializeNativeTarget();
		llvm::InitializeNativeTargetAsmPrinter();
		llvm::InitializeNativeTargetAsmParser();
	}
};

Jitter::Jitter(vaddr_t pc, const Mmu& mmu, size_t cov_map_size,
               const Breakpoints& breakpoints, const vector<linkage_t>& linkages,
			   const EmuOptions& options):
	mmu(mmu), cov_map_size(cov_map_size), breakpoints(breakpoints),
	linkages(linkages), options(options), p_context(new llvm::LLVMContext),
	p_module(new llvm::Module("module", *p_context)),
	context(*p_context), module(*p_module), builder(context),
	must_be_loaded{false}, must_be_saved{false}
{
	// Lock global jitter mutex
	std::lock_guard<std::mutex> lock(mutex);

	static jit_init init; // thread_local?

	assert(__builtin_popcount(cov_map_size) == 1);

	// Get function name
	ostringstream oss;
	oss << hex << pc;
	string pc_s(oss.str());
	string func_name = "func_" + pc_s;

	// Try to load from disk if it was jitted in previous runs
	if (load_from_disk(func_name))
		return;

	// We couldn't load it from disk. Time to JIT
	printf("starting jit 0x%X\n", pc);

	// Get basic types
	void_ty      = llvm::Type::getVoidTy(context);
	int1_ty      = llvm::Type::getInt1Ty(context);
	int8_ty      = llvm::Type::getInt8Ty(context);
	int16_ty     = llvm::Type::getInt16Ty(context);
	int32_ty     = llvm::Type::getInt32Ty(context);
	int64_ty     = llvm::Type::getInt64Ty(context);
	float_ty     = llvm::Type::getFloatTy(context);
	double_ty    = llvm::Type::getDoubleTy(context);
	int1ptr_ty   = llvm::Type::getInt1PtrTy(context);
	int8ptr_ty   = llvm::Type::getInt8PtrTy(context);
	int16ptr_ty  = llvm::Type::getInt16PtrTy(context);
	int32ptr_ty  = llvm::Type::getInt32PtrTy(context);
	int64ptr_ty  = llvm::Type::getInt64PtrTy(context);
	floatptr_ty  = llvm::Type::getFloatPtrTy(context);
	doubleptr_ty = llvm::Type::getDoublePtrTy(context);

	// Get function arguments types
	llvm::Type* vm_state_ty = llvm::StructType::get(context,
		{
			int32ptr_ty, // regs
			int8ptr_ty,  // memory
			int8ptr_ty,  // perms
			int32ptr_ty, // dirty_vec
			int32ptr_ty, // p_dirty_size
			int8ptr_ty,  // dirty_map
			floatptr_ty, // fpregs
			int8ptr_ty,  // ccs
		}
	);
	llvm::Type* p_vm_state_ty = llvm::PointerType::get(vm_state_ty, 0);

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
		int32_ty,           // ret: number of instructions executed
		{
			p_vm_state_ty,  // p_vm_state
			p_exit_info_ty, // p_exit_info
			int8ptr_ty,     // cov_map
			int8ptr_ty,     // p_emu
			int32ptr_ty,    // regs_dump
		},
		false               // varargs
	);
	function = llvm::Function::Create(
		function_type,
		llvm::GlobalValue::LinkageTypes::ExternalLinkage,
		func_name,
		module
	);

	// Create entry basic block. Allocate stack for counting number of
	// instructions, which will be returned by the function. With optimization
	// passes this turns into returning a constant, so it's free.
	llvm::BasicBlock* entry_block =
		llvm::BasicBlock::Create(context, "entry", function);
	builder.SetInsertPoint(entry_block);
	p_instr = builder.CreateAlloca(int32_ty);
	builder.CreateStore(builder.getInt32(0), p_instr);

	// Load state
	state = {
		get_state_field(0, "p_regs"),
		get_state_field(1, "p_memory"),
		get_state_field(2, "p_perms"),
		get_state_field(3, "p_dirty_vec"),
		get_state_field(4, "p_dirty_size"),
		get_state_field(5, "p_dirty_map"),
		get_state_field(6, "p_fpregs"),
		get_state_field(7, "p_ccs"),
	};

	if (TMP_REGS){
		// Allocate stack for each reg
		for (int i = 0; i < NUM_REGS; i++)
			p_regs[i] = builder.CreateAlloca(int32_ty);
	}

	// Create fault path. Whenever a fault occurs, this fast vm exit will be
	// taken. We exit with the fault pc as reenter_pc, so interpreter can then
	// re-run last instruction to get more details about the fault. Having the
	// fault pc has a small perf cost because each write_mem and read_mem has
	// to set the fault pc just in case there's a fault.
	// The other option is not saving the fault pc and running the interpreter
	// from the beginning when there's a fault, but that means each crash run
	// would be run twice.
	// Anyways, having this fast vm exit is much better than generating
	// a vm exit for every possible fault in every memory access.
	p_fault_pc = builder.CreateAlloca(int32_ty);
	fault_path = llvm::BasicBlock::Create(context, "fault_path", function);
	builder.SetInsertPoint(fault_path);
	vm_exit(ExitInfo::ExitReason::Fault, builder.CreateLoad(p_fault_pc));

	// Create end path, which will vm exit with RunFinished when handle_syscall
	// returns true
	if (INTEGRATED_CALLS){
		end_path = llvm::BasicBlock::Create(context, "end_path", function);
		builder.SetInsertPoint(end_path);
		vm_exit(ExitInfo::ExitReason::RunFinished);
	}

	// Create basic blocks recursively
	llvm::BasicBlock* bb = create_block(pc);


	builder.SetInsertPoint(entry_block);

	if (TMP_REGS){
		// Load each necessary register
		load_regs();
	}

	// Jump from entry to `bb`. LLVM doesn't allow jumping to the first basic
	// block of a function. This way `bb` is the second basic block and we can
	// jump to it
	builder.CreateBr(bb);

	if (TMP_REGS){
		// Save registers in each vm exit
		gen_save_regs();
	}

	// JIT module
	if (llvm::verifyModule(module)){
		llvm::outs() << module;
		llvm::verifyModule(module, &llvm::errs());
		die("bad module\n");
	}
	compile();
}

void Jitter::link_stuff(llvm::ExecutionEngine* ee){
	for (const linkage_t& linkage : linkages){
		ee->addGlobalMapping(linkage.name, (uint64_t)linkage.func);
	}
}

bool Jitter::load_from_disk(const string& name){
	// Check if file exists
	string cache_name = "jitcache/" + name;
	if (access(cache_name.c_str(), R_OK) == -1)
		return false;

	//cout << "Loading cached " << name << endl;

	// Read file into memory buffer
	auto buffer = llvm::MemoryBuffer::getFile(cache_name);
	if (!buffer){
		cout << "Error reading cache " << name << ": "
		     << buffer.getError().message() << endl;
		return false;
	}

	// Create object file from memory buffer
	auto obj = llvm::object::ObjectFile::createObjectFile(
		buffer.get()->getMemBufferRef());
	if (!obj){
		cout << "Error creating obj for cache " << name << endl;
		return false;
	}

	// Create execution engine, add object file and get pointer to function.
	// Unlike in compile(), here we are not jitting because we are adding an
	// object which is already compiled
	llvm::ExecutionEngine* ee = llvm::EngineBuilder(move(p_module))
		.setEngineKind(llvm::EngineKind::JIT)
		.create();
	ee->addObjectFile(move(obj.get()));
	link_stuff(ee);
	result = (jit_block_t)ee->getFunctionAddress(name);
	if (result == NULL){
		cout << "Error getting function address for cache " << name << endl;
		return false;
	}
	return true;
}

void add_opt_passes(llvm::legacy::PassManagerBase &passes,
                  llvm::legacy::FunctionPassManager &fnPasses,
                  llvm::TargetMachine *machine)
{
	llvm::PassManagerBuilder builder;
	builder.OptLevel = 3;
	builder.SizeLevel = 0;
	builder.Inliner = llvm::createFunctionInliningPass(3, 0, false);
	builder.LoopVectorize = true;
	builder.SLPVectorize = true;
	machine->adjustPassManager(builder);

	builder.populateFunctionPassManager(fnPasses);
	builder.populateModulePassManager(passes);
}

void add_link_passes(llvm::legacy::PassManagerBase &passes){
	llvm::PassManagerBuilder builder;
	builder.VerifyInput = true;
	builder.Inliner = llvm::createFunctionInliningPass(3, 0, false);
	builder.populateLTOPassManager(passes);
}

void optimize_module(llvm::TargetMachine *machine, llvm::Module *module) {
	module->setTargetTriple(machine->getTargetTriple().str());
	module->setDataLayout(machine->createDataLayout());

	llvm::legacy::PassManager passes;
	llvm::TargetIRAnalysis ir_analysis = machine->getTargetIRAnalysis();
	passes.add(new llvm::TargetLibraryInfoWrapperPass(machine->getTargetTriple()));
	passes.add(llvm::createTargetTransformInfoWrapperPass(ir_analysis));

	llvm::legacy::FunctionPassManager fnPasses(module);
	fnPasses.add(llvm::createTargetTransformInfoWrapperPass(ir_analysis));

	add_opt_passes(passes, fnPasses, machine);
	add_link_passes(passes);

	fnPasses.doInitialization();
	for (llvm::Function &func : *module)
		fnPasses.run(func);
	fnPasses.doFinalization();

	passes.add(llvm::createVerifierPass());
	passes.run(*module);
}

void Jitter::compile(){
	// Get target machine
	string error;
	string triple = llvm::sys::getDefaultTargetTriple();
	const llvm::Target* target =
		llvm::TargetRegistry::lookupTarget(triple, error);
	if (!target)
		die("err target: %s\n", error.c_str());

	llvm::SubtargetFeatures subtarget_features;
	llvm::StringMap<bool> feature_map;
	if (llvm::sys::getHostCPUFeatures(feature_map))
		for (auto &feature : feature_map)
			subtarget_features.AddFeature(feature.first(), feature.second);
	string features = subtarget_features.getString();

	llvm::TargetMachine* machine = target->createTargetMachine(
		triple,                               // triple str
		llvm::sys::getHostCPUName(),          // cpu name str
		features,                             // features str
		llvm::TargetOptions(),                // target options
		llvm::Optional<llvm::Reloc::Model>(), // relocation model
		llvm::None,                           // no idea
		llvm::CodeGenOpt::Default,            // codegen optimization
		true                                  // JIT
		// It took me two whole days to figure out why relocation types
		// were wrong when emiting object files to the disk cache were wrong.
		// JIT is false by default. Beware, kids.
	);

	// Optimize IR
	optimize_module(machine, p_module.get());

	// Compile
	llvm::ExecutionEngine* ee = llvm::EngineBuilder(move(p_module))
		.setEngineKind(llvm::EngineKind::JIT)
		.create(machine);
	link_stuff(ee);
	result = (jit_block_t)ee->getFunctionAddress(function->getName());

	// Dump to disk
	string cache_name = ("jitcache/" + function->getName()).str();
	error_code ec;
	llvm::raw_fd_ostream os(cache_name, ec);
	if (ec)
		die("error opening cache for writing: %s\n", ec.message().c_str());

	llvm::legacy::PassManager pass_dumper;
	machine->addPassesToEmitFile(pass_dumper, os, nullptr,
	                             llvm::TargetMachine::CGFT_ObjectFile);
	pass_dumper.run(module);
	os.flush();
	if (ec)
		die("error dumping objfile to disk: %s\n", ec.message().c_str());
}

jit_block_t Jitter::get_result(){
	return result;
}

llvm::Value* Jitter::get_state_field(uint8_t field, const string& name){
	// Get pointer to field and load it
	llvm::Value* p_value = builder.CreateInBoundsGEP(
		&function->arg_begin()[0],
		{
			builder.getInt32(0),     // *state
			builder.getInt32(field), // state->field
		},
		"p_" + name
	);
	llvm::Value* value = builder.CreateLoad(p_value, name);
	return value;
}

llvm::Value* Jitter::get_preg(uint8_t reg){
	// Get pointer to regs[reg]
	assert(1 <= reg && reg <= 33); // 0 is reg zero
	llvm::Value* p_reg;
	if (TMP_REGS){
		p_reg = p_regs[reg];
	} else {
		p_reg = builder.CreateInBoundsGEP(
			state.p_regs,
			builder.getInt32(reg),
			"p_reg" + to_string(reg) + "_"
		);
	}
	return p_reg;
}

llvm::Value* Jitter::gets_preg(uint8_t reg){
	assert(0 <= reg && reg <= 31);
	llvm::Value* p_fpreg = builder.CreateInBoundsGEP(
		state.p_fpregs,
		builder.getInt32(reg),
		"p_fpreg" + to_string(reg) + "_"
	);
	return p_fpreg;
}

llvm::Value* Jitter::getd_preg(uint8_t reg){
	assert(0 <= reg && reg <= 31 && (reg%2 == 0));
	llvm::Value* p_fpreg = gets_preg(reg);
	return builder.CreateBitCast(p_fpreg, doubleptr_ty);
}

llvm::Value* Jitter::get_reg(uint8_t reg){
	// Get pointer to reg and load it
	assert(0 <= reg && reg <= 33);
	llvm::Value* reg_val;
	if (reg == 0)
		reg_val = builder.getInt32(0);
	else {
		llvm::Value* p_reg = get_preg(reg);
		reg_val = builder.CreateLoad(p_reg, "reg" + to_string(reg) + "_");
		must_be_loaded[reg] = true;
	}
	return reg_val;
}

void Jitter::set_reg(uint8_t reg, llvm::Value* val){
	// Get pointer to reg and write zero extended val
	assert(0 <= reg && reg <= 33);
	llvm::Value* val_zext = builder.CreateZExt(val, int32_ty);
	if (reg != 0){
		llvm::Value* p_reg = get_preg(reg);
		builder.CreateStore(val_zext, p_reg);
		must_be_saved[builder.GetInsertBlock()][reg] = true;
	}
}

void Jitter::set_reg(uint8_t reg, uint32_t val){
	set_reg(reg, builder.getInt32(val));
}

llvm::Value* Jitter::gets_reg(uint8_t reg){
	llvm::Value* p_fpreg = gets_preg(reg);
	return builder.CreateLoad(p_fpreg);
}

void Jitter::sets_reg(uint8_t reg, llvm::Value* val){
	llvm::Value* p_fpreg = gets_preg(reg);
	builder.CreateStore(val, p_fpreg);
}

void Jitter::sets_reg(uint8_t reg, float val){
	sets_reg(reg, llvm::ConstantFP::get(context, llvm::APFloat(val)));
}

llvm::Value* Jitter::getd_reg(uint8_t reg){
	llvm::Value* p_fpreg = getd_preg(reg);
	return builder.CreateLoad(p_fpreg);
}

void Jitter::setd_reg(uint8_t reg, llvm::Value* val){
	llvm::Value* p_fpreg = getd_preg(reg);
	builder.CreateStore(val, p_fpreg);
}

void Jitter::setd_reg(uint8_t reg, double val){
	setd_reg(reg, llvm::ConstantFP::get(context, llvm::APFloat(val)));
}

llvm::Value* Jitter::get_cc(uint8_t cc){
	assert(0 <= cc && cc <= 7);
	llvm::Value* ccs = builder.CreateLoad(state.p_ccs);
	llvm::Value* cc_val = builder.CreateTrunc(
		builder.CreateLShr(ccs, builder.getInt8(cc)),
		int1_ty
	);
	return cc_val;
}

void Jitter::set_cc(uint8_t cc, llvm::Value* val){
	assert(0 <= cc && cc <= 7);
	llvm::Value* ccs = builder.CreateLoad(state.p_ccs);
	llvm::Value* bit = builder.getInt8(1 << cc);
	llvm::Value* cmp = builder.CreateICmpEQ(val, builder.getTrue());
	llvm::Value* new_ccs = builder.CreateSelect(
		cmp,
		builder.CreateOr(ccs, bit),
		builder.CreateAnd(ccs, builder.CreateNot(bit))
	);
	builder.CreateStore(new_ccs, state.p_ccs);
}

void Jitter::set_cc(uint8_t cc, bool val){
	set_cc(cc, builder.getInt1(val));
}

llvm::Value* Jitter::get_pmemory(llvm::Value* addr){
	// Get pointer to memory[addr]
	llvm::Value* p_value = builder.CreateInBoundsGEP(
		state.p_memory,
		addr,
		"p_value"
	);
	return p_value;
}

void Jitter::vm_exit(ExitInfo::ExitReason reason, llvm::Value* reenter_pc,
                     llvm::Value* info1, llvm::Value* info2)
{
	// Get pointer to exit info fields, write to them and return
	llvm::Value* p_exit_info = &function->arg_begin()[1];
	llvm::Value* p_exit_reason = builder.CreateInBoundsGEP(
		p_exit_info,
		{
			builder.getInt32(0),
			builder.getInt32(0),
		},
		"p_exit_reason"
	);
	builder.CreateStore(builder.getInt32(reason), p_exit_reason);

	// Store the rest of the parameters only if they are provided
	if (reenter_pc){
		llvm::Value* p_pc = builder.CreateInBoundsGEP(
			p_exit_info,
			{
				builder.getInt32(0),
				builder.getInt32(1),
			},
			"p_pc"
		);
		builder.CreateStore(reenter_pc, p_pc);
	}

	if (info1){
		llvm::Value* p_info1 = builder.CreateInBoundsGEP(
			p_exit_info,
			{
				builder.getInt32(0),
				builder.getInt32(2),
			},
			"p_info1_"
		);
		builder.CreateStore(info1, p_info1);
	}

	if (info2){
		llvm::Value* p_info2 = builder.CreateInBoundsGEP(
			p_exit_info,
			{
				builder.getInt32(0),
				builder.getInt32(3),
			},
			"p_info2_"
		);
		builder.CreateStore(info2, p_info2);
	}

	builder.CreateRet(builder.CreateLoad(p_instr, "instr"));
}

uint32_t Jitter::get_new_cov_id(size_t cov_map_size){
	if (next_cov_id >= cov_map_size)
		die("Out of coverage ids: %u/%lu\n", next_cov_id, cov_map_size);

	//cout << "Returning cov id " << next_cov_id << endl;
	uint32_t ret = next_cov_id++;

	// Save to disk
	ofstream os("./jitcache/next_cov_id");
	os << next_cov_id << endl;
	os.close();

	return ret;
}

llvm::Value* Jitter::add_coverage(vaddr_t pc, vaddr_t jump1, vaddr_t jump2,
                                  llvm::Value* cmp)
{
	// Conditional branches version
	if (UNIQUE_COV_ID_ATTEMPT){
		// Get coverage ids for this branch
		if (!cov_ids.count(pc)){
			cov_ids[pc].first  = get_new_cov_id(cov_map_size);
			cov_ids[pc].second = get_new_cov_id(cov_map_size);
		}

		// Select coverage id depending on the branch result (taking it or not)
		llvm::Value* cov_id = builder.CreateSelect(
			cmp,
			builder.getInt32(cov_ids[pc].first),
			builder.getInt32(cov_ids[pc].second)
		);
		return add_coverage(cov_id);

	} else {
		// Branch hash
		llvm::Value* jmp_address = builder.CreateSelect(
			cmp,
			builder.getInt32(jump1),
			builder.getInt32(jump2)
		);
		return add_coverage(builder.getInt32(pc), jmp_address);
	}
}

llvm::Value* Jitter::add_coverage(vaddr_t pc, vaddr_t jump){
	// Unconditional branches version
	if (UNIQUE_COV_ID_ATTEMPT){
		// Get coverage id for this branch
		if (!cov_ids.count(pc))
			cov_ids[pc].first = get_new_cov_id(cov_map_size);
		return add_coverage(builder.getInt32(cov_ids[pc].first));

	} else {
		// Branch hash
		return add_coverage(builder.getInt32(pc), builder.getInt32(jump));
	}
}

llvm::Value* Jitter::add_coverage(llvm::Value* from, llvm::Value* to){
	// Indirect branches version
	// Compute branch hash. When `from` and `to` are constants, this will be
	// translated to a constant (for example in bal). Particularly, `from` is
	// always constant
	llvm::Value* tmp1 = builder.CreateShl (from, builder.getInt32(6));
	llvm::Value* tmp2 = builder.CreateLShr(from, builder.getInt32(2));
	llvm::Value* tmp3 = builder.CreateAdd(builder.CreateAdd(to, tmp1), tmp2);
	llvm::Value* tmp4 = builder.CreateXor(from, tmp3);
	llvm::Value* branch_hash =
		builder.CreateAnd(tmp4, builder.getInt32(cov_map_size-1));

	// Use branch hash as coverage id
	return add_coverage(branch_hash);
}

llvm::Value* Jitter::add_coverage(llvm::Value* cov_id){
	// Get quotient and rest
	llvm::Value* cov_id_q    = builder.CreateUDiv(cov_id, builder.getInt32(8));
	llvm::Value* cov_id_r    = builder.CreateTrunc(
		builder.CreateURem(cov_id, builder.getInt32(8)),
		int8_ty
	);

	// Set the corresponding bit in cov_map
	llvm::Value* bit         = builder.CreateShl(builder.getInt8(1), cov_id_r);
	llvm::Value* cov_map     = &function->arg_begin()[2];
	llvm::Value* p_cov_value = builder.CreateInBoundsGEP(cov_map, cov_id_q);
	llvm::Value* cov_value   = builder.CreateLoad(p_cov_value);
	llvm::Value* new_cov_value = builder.CreateOr(cov_value, bit);
	builder.CreateStore(new_cov_value, p_cov_value);
	return cov_id;
}

void Jitter::check_bounds_mem(llvm::Value* addr, vsize_t len){
	// Create nofault path, compare and branch
	llvm::BasicBlock* nofault_path =
		llvm::BasicBlock::Create(context, "nofault_bounds", function);
	llvm::Value* memsize = builder.getInt32(mmu.size());
	llvm::Value* last_addr = builder.CreateAdd(
		addr,
		builder.getInt32(len),
		"last_addr"
	);
	llvm::Value* cmp = builder.CreateICmpUGE(last_addr, memsize, "cmp_bounds");
	builder.CreateCondBr(cmp, fault_path, nofault_path);

	// Continue building nofault path
	builder.SetInsertPoint(nofault_path);
}

void Jitter::check_perms_mem(llvm::Value* addr, vsize_t len, uint8_t perm){
	llvm::Type* mask_ty = llvm::Type::getIntNTy(context, len*8);
	llvm::Type* ptr_ty  = llvm::Type::getIntNPtrTy(context, len*8);

	// Compute permission mask for given len
	uint64_t perms_mask = 0;
	for (vsize_t i = 0; i < len; i++)
		perms_mask |= (perm << (i*8));
	llvm::Value* perms_mask_val = llvm::ConstantInt::get(mask_ty, perms_mask);

	// Create nofault path
	llvm::BasicBlock* nofault_path =
		llvm::BasicBlock::Create(context, "nofault_perms", function);

	// Get pointer to permissions value, cast it and load its value
	llvm::Value* p_perm  = builder.CreateInBoundsGEP(
		state.p_perms,
		addr,
		"p_perm"
	);
	llvm::Value* p_perm_cast =
		builder.CreateBitCast(p_perm, ptr_ty, "p_perm_cast");
	llvm::Value* perm_val = builder.CreateLoad(p_perm_cast, "perm");

	// And perm value with perm_masks, check if they're equal and branch
	llvm::Value* result = builder.CreateAnd(perm_val, perms_mask_val);
	llvm::Value* cmp = builder.CreateICmpNE(result, perms_mask_val, "cmp_perms");
	builder.CreateCondBr(cmp, fault_path, nofault_path);

	// Continue building nofault path
	builder.SetInsertPoint(nofault_path);
}

void Jitter::check_alignment_mem(llvm::Value* addr, vsize_t len){
	// Create nofault paths
	llvm::BasicBlock* nofault_path =
		llvm::BasicBlock::Create(context, "nofault_align", function);

	// Check alignment and branch
	llvm::Value* addr_masked = builder.CreateAnd(
		addr,
		builder.getInt32(len-1)
	);
	llvm::Value* cmp = builder.CreateICmpNE(
		addr_masked,
		builder.getInt32(0),
		"cmp_align"
	);
	builder.CreateCondBr(cmp, fault_path, nofault_path);

	// Continue building nofault path
	builder.SetInsertPoint(nofault_path);
}

void Jitter::set_dirty_mem(llvm::Value* addr, vsize_t len){
	assert(len > 0);
	/* Pseudocode:
	 *
	 * block_begin = addr/DIRTY_BLOCK_SIZE
	 * block_end   = (addr+len+DIRTY_BLOCK_SIZE-1)/DIRTY_BLOCK_SIZE
	 * for (block = block_begin; block < block_end; block++){
	 *     if (!dirty_map[block]){
	 *         dirty_vec[dirty_size++] = block
	 *         dirty_map[block] = true
	 *     }
	 * }
	 *
	 * Actually it doesn't check the condition first, so it always performs
	 * at least one iteration, which is consistent with having len > 0.
	 *
	 * Maybe having a for loop is innecessary, most times it will run only
	 * one iteration and it will be optimized.
	*/

	// Basic blocks used
	llvm::BasicBlock* prev_bb = builder.GetInsertBlock();
	llvm::BasicBlock* for_body =
		llvm::BasicBlock::Create(context, "dirty_for_body", function);
	llvm::BasicBlock* for_register_dirty =
		llvm::BasicBlock::Create(context, "dirty_for_register_dirty", function);
	llvm::BasicBlock* for_cond =
		llvm::BasicBlock::Create(context, "dirty_for_cond", function);
	llvm::BasicBlock* for_end =
		llvm::BasicBlock::Create(context, "dirty_for_end", function);

	// Computations before entering for body
	llvm::Value* dirty_block_sz = builder.getInt32(Mmu::DIRTY_BLOCK_SIZE);
	llvm::Value* addr_end    = builder.CreateAdd(addr, builder.getInt32(len));
	llvm::Value* block_begin = builder.CreateUDiv(addr, dirty_block_sz,
	                                              "block_begin");
	llvm::Value* block_end   = builder.CreateUDiv(
		builder.CreateAdd(
			addr_end, 
			builder.CreateSub(dirty_block_sz, builder.getInt32(1))
		),
		dirty_block_sz,
		"block_end"
	);
	builder.CreateBr(for_body);

	// For body. Load dirty value and branch
	builder.SetInsertPoint(for_body);
	llvm::PHINode* block = builder.CreatePHI(int32_ty, 2, "block");
	block->addIncoming(block_begin, prev_bb);
	llvm::Value* p_dirty_value =
		builder.CreateInBoundsGEP(state.p_dirty_map, block, "p_dirty_value");
	llvm::Value* dirty_value = builder.CreateLoad(p_dirty_value, "dirty_value");
	llvm::Value* cmp_dirty = 
		builder.CreateICmpEQ(dirty_value, builder.getInt8(0), "cmp_dirty");
	builder.CreateCondBr(cmp_dirty, for_register_dirty, for_cond);

	// For register dirty. Block was not dirty: set it as dirty in the map and
	// add it to the vector.
	builder.SetInsertPoint(for_register_dirty);
	builder.CreateStore(builder.getInt8(1), p_dirty_value);
	llvm::Value* dirty_size = builder.CreateLoad(state.p_dirty_size, "dirty_size");
	builder.CreateStore(block, 
		builder.CreateInBoundsGEP(state.p_dirty_vec, dirty_size));
	builder.CreateStore(builder.CreateAdd(dirty_size, builder.getInt32(1)),
	                    state.p_dirty_size);
	builder.CreateBr(for_cond);

	// For cond, increment block and check end condition.
	builder.SetInsertPoint(for_cond);
	llvm::Value* block_inc = builder.CreateAdd(block, builder.getInt32(1),
	                                           "block_inc");
	block->addIncoming(block_inc, for_cond);
	llvm::Value* cmp_exit  = builder.CreateICmpUGE(block_inc, block_end,
	                                               "cmp_exit");
	builder.CreateCondBr(cmp_exit, for_end, for_body);

	// For end. Continue with the store
	builder.SetInsertPoint(for_end);
}

llvm::Value* Jitter::read_mem(llvm::Value* addr, vsize_t len, vaddr_t pc){
	assert(len != 0);

	// Store pc just in case there's a fault
	builder.CreateStore(builder.getInt32(pc-4), p_fault_pc);

	// Check out of bounds
	check_bounds_mem(addr, len);

	// Check permissions
	check_perms_mem(addr, len, Mmu::PERM_READ);

	// Check alignment
	check_alignment_mem(addr, len);

	// Get pointer to memory position, cast it from i8* to iN* and read from it
	llvm::Value* p_value      = get_pmemory(addr);
	llvm::Type*  cast_type    = llvm::Type::getIntNPtrTy(context, len*8);
	llvm::Value* p_value_cast = builder.CreateBitCast(p_value, cast_type);
	return builder.CreateLoad(p_value_cast);
}

void Jitter::write_mem(llvm::Value* addr, llvm::Value* value, vsize_t len,
                       vaddr_t pc)
{
	assert(len != 0);

	// Store pc just in case there's a fault
	builder.CreateStore(builder.getInt32(pc-4), p_fault_pc);

	// Check out of bounds
	check_bounds_mem(addr, len);

	// Check permissions
	check_perms_mem(addr, len, Mmu::PERM_WRITE);

	// Check alignment
	check_alignment_mem(addr, len);

	// Set dirty
	set_dirty_mem(addr, len);

	// Get pointer to memory position, cast it from i8* to iN*, cast value from
	// ?? to iN, and write value to pointer
	llvm::Value* p_value      = get_pmemory(addr);
	llvm::Type*  cast_type_p  = llvm::Type::getIntNPtrTy(context, len*8);
	llvm::Type*  cast_type_v  = llvm::Type::getIntNTy(context, len*8);
	llvm::Value* p_value_cast = builder.CreateBitCast(p_value, cast_type_p);
	llvm::Value* value_cast   = builder.CreateTrunc(value, cast_type_v);
	builder.CreateStore(value_cast, p_value_cast);
}

void Jitter::load_reg(uint8_t reg){
	llvm::Value* p_reg = builder.CreateInBoundsGEP(
		state.p_regs,
		builder.getInt32(reg)
	);
	builder.CreateStore(builder.CreateLoad(p_reg), p_regs[reg]);
}

void Jitter::save_reg(uint8_t reg){
	llvm::Value* p_reg = builder.CreateInBoundsGEP(
		state.p_regs,
		builder.getInt32(reg)
	);
	builder.CreateStore(builder.CreateLoad(p_regs[reg]), p_reg);
}

void Jitter::load_regs(){
	for (int i = 0; i < NUM_REGS; i++)
		if (must_be_loaded[i])
			load_reg(i);
}

void Jitter::save_regs(){
	for (int i = 0; i < NUM_REGS; i++)
		if (must_be_saved[builder.GetInsertBlock()][i])
			save_reg(i);
}

// Add dirty registers of `block` to `succ` and all of its successors
// If block == succ, add dirty registers of `block` to all of its successors
void Jitter::join_must_be_saved(llvm::BasicBlock* block, llvm::BasicBlock* succ){
	// Join block dirty registers to succ dirty registers
	for (int i = 0; i < NUM_REGS; i++)
		if (must_be_saved[block][i])
			must_be_saved[succ][i] = true;

	// Recursion
	for (llvm::BasicBlock* new_succ : llvm::successors(succ)){
		if (new_succ != block)
			join_must_be_saved(block, new_succ);
	}
}

void Jitter::update_must_be_saved(llvm::BasicBlock* block, handled_t& handled){
	join_must_be_saved(block, block);
	for (llvm::BasicBlock* succ : llvm::successors(block)){
		if (handled.insert(succ).second)
			update_must_be_saved(succ, handled);
	}
}


void Jitter::gen_save_regs(){
	handled_t handled;
	update_must_be_saved(&function->getEntryBlock(), handled);

	for (llvm::BasicBlock& block : *function){
		// Look for vm exits and save registers before them
		llvm::Instruction* terminator = block.getTerminator();
		if (llvm::isa<llvm::ReturnInst>(terminator)){
			builder.SetInsertPoint(terminator->getPrevNode());
			save_regs();
		}

		if (INTEGRATED_CALLS){
			// Calls from JIT code to emulator code must save some regs.
			// Look for every call instruction, and save needed regs
			for (llvm::Instruction& inst : block){
				if (!llvm::isa<llvm::CallInst>(inst))
					continue;
				llvm::CallInst& call = llvm::cast<llvm::CallInst>(inst);
				if (call.getCalledFunction()->getName() == "handle_syscall"){
					// Handle syscall: save some registers
					builder.SetInsertPoint(inst.getPrevNode());
					for (auto r : {Reg::a0, Reg::a1, Reg::a2, Reg::a3, Reg::sp})
						if (must_be_saved[&block][r])
							save_reg(r);
				}
			}
		}
	}
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

	// Generate code for every instruction in the block
	bool end = false;
	while (!end){
		// If there's a breakpoint in current instruction, generate vm exit.
		// Breakpoint handler will be run. It MUST change pc. Otherwise, we'll
		// enter an infinite loop of vm exits and breakpoint handlers.
		// Breakpoints that don't change pc can be implemented calling handlers
		// directly from JIT code, without generating vm exits
		if (breakpoints.has_bp(pc)){
			vm_exit(ExitInfo::ExitReason::Breakpoint, builder.getInt32(pc));
			break;
		}

		end = handle_inst(pc);

		pc += 4;
	}

	// Set builder to previous basic block and return new block
	builder.SetInsertPoint(prev_block);
	return block;
}

bool Jitter::handle_inst(vaddr_t pc){
	// Log state
	if (options.dump_pc || options.dump_regs){
		llvm::Value* num_inst = builder.CreateLoad(p_instr);
		llvm::Value* i_dump = builder.CreateMul(num_inst, builder.getInt32(35));
		llvm::Value* regs_dump = &function->arg_begin()[3];
		llvm::Value* reg_dump  = builder.CreateInBoundsGEP(regs_dump, i_dump);
		if (options.dump_regs){
			for (int i = 0; i < NUM_REGS; i++){
				llvm::Value* p_reg =
					builder.CreateInBoundsGEP(reg_dump, builder.getInt32(i));
				builder.CreateStore(get_reg(i), p_reg);
			}
		}
		llvm::Value* p_reg =
			builder.CreateInBoundsGEP(reg_dump, builder.getInt32(NUM_REGS));
		builder.CreateStore(builder.getInt32(pc), p_reg);
	}

	// Increment instruction count
	llvm::Value* instr     = builder.CreateLoad(p_instr, "instr");
	llvm::Value* instr_inc = builder.CreateAdd(instr, builder.getInt32(1));
	builder.CreateStore(instr_inc, p_instr);

	// Fetch current instruction
	uint32_t inst   = mmu.read_inst(pc);
	uint8_t  opcode = (inst >> 26) & 0b111111;

	// Call instruction handler
 	if (inst)
		return (this->*inst_handlers[opcode])(pc+4, inst);

	return false;
}

const char* exit_reason_map[] = {
	"syscall", "fault", "indirect branch", "rdhwr", "exception", "breakpoint",
};
ostream& JIT::operator<<(ostream& os, const ExitInfo& exit_inf){
	if (exit_inf.reason >= sizeof(exit_reason_map)/sizeof(const char*)){
		os << "Unknown exit reason " << exit_inf.reason;
		return os;
	}

	os << "Exit reason: " << exit_reason_map[exit_inf.reason]
	   << "; reenter pc: 0x" << hex << exit_inf.reenter_pc << dec;
	if (exit_inf.reason == ExitInfo::ExitReason::Fault)
		os << "; Fault: " << Fault((Fault::Type)exit_inf.info1, exit_inf.info2);
	return os;
}
