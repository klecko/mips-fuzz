#ifndef _JITTER_H
#define _JITTER_H

#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <mutex>
#include <fstream>
#include "llvm/IR/IRBuilder.h"
#include "llvm/ExecutionEngine/ExecutionEngine.h"
#include "common.h"
#include "mmu.h"
#include "breakpoints.h"

#define TMP_REGS 0
#define INTEGRATED_CALLS 0
#define DETAILED_FAULT 1

// Jitter will access pc has a hacky register, so it will use NUM_REGS+1
const int NUM_REGS = 34;
const int JIT_NUM_REGS = NUM_REGS + 1;
const int NUM_FP_REGS = 32;
enum Reg {
	zero, at, v0, v1, a0, a1, a2, a3,
	t0,   t1, t2, t3, t4, t5, t6, t7,
	s0,   s1, s2, s3, s4, s5, s6, s7,
	t8,   t9, k0, k1, gp, sp, fp, ra,
	hi,   lo, pc
};

struct EmuOptions;

namespace JIT {

struct linkage_t {
	std::string name;
	void*       func;
};

// Struct that will be modified by the jitted code when returning to notify
// exit information
struct ExitInfo {
	enum ExitReason: uint32_t {
		Syscall = 0,
		Fault,
		IndirectBranch,
		Call,
		Rdhwr,
		Exception, // trap or break
		Breakpoint,
		CheckRepCovId, // debugging coverage ids
		RunFinished,
	};
	ExitReason reason;
	vaddr_t    reenter_pc;
	uint32_t   info1;
	uint32_t   info2;
};
std::ostream& operator<<(std::ostream& os, const ExitInfo& exit_inf);

// Struct that will be accessed by the jitted code to modify the VM
struct VmState {
	uint32_t* regs;
	uint8_t*  memory;
	uint8_t*  perms;
	vaddr_t*  dirty_vec;
	vsize_t*  p_dirty_size;
	uint8_t*  dirty_map;
	float*    fpregs;
	uint8_t*  ccs;
};

// JIT block: signature of the function returned by Jitter
typedef uint32_t (*jit_block_t)(
	VmState*  p_vm_state,
	ExitInfo* p_exit_info,
	uint8_t*  cov_map,
	Emulator* p_emu // pointer to emu so we can call its methods
);

// Type used for storing already compiled JIT blocks.
// We use a vector instead of an unordered_map because we need to access it
// as fast as we can, but memory cost increases. Also, vector is thread-safe
// as size is not being changed
typedef std::vector<std::atomic<jit_block_t>> jit_cache_t;
//typedef std::unordered_map<vaddr_t, std::atomic<jit_block_t>> jit_cache_t;
//#include "sparsehash/dense_hash_map"
//typedef google::dense_hash_map<vaddr_t, JIT::jit_block_t> jit_cache_t;

const jit_block_t JIT_BLOCK_COMPILING = (jit_block_t)1;

class CovIdProvider {
	private:
		std::atomic_flag lock;
		std::atomic<uint32_t> next_cov_id;
		std::map<std::pair<vaddr_t, size_t>, uint32_t> cov_ids;
		uint32_t get_next(){
			return next_cov_id++;
		}
	public:
		CovIdProvider(){
			std::ifstream is("./jitcache/next_cov_id");
			if (!is.good())
				next_cov_id = 0;
			else {
				uint32_t v;
				is >> v;
				next_cov_id = v;
			}
			is.close();
		}
		void save(){
			while (lock.test_and_set());
			std::ofstream os("./jitcache/next_cov_id");
			os << next_cov_id;
			os.close();
			lock.clear();
		}
		void check_max(uint32_t max_cov_id){
			if (next_cov_id >= max_cov_id)
				die("Out of cov ids: %u/%u\n", next_cov_id.load(), max_cov_id);
		}
		uint32_t get(vaddr_t pc, size_t i, uint32_t max_cov_id){
			check_max(max_cov_id);
			auto key = std::make_pair(pc, i);
			if (!cov_ids.count(key))
				cov_ids[key] = get_next();
			return cov_ids[key];
		}
};

class Jitter {
public:
	Jitter(vaddr_t pc, const Mmu& mmu, size_t cov_map_size,
	       const Breakpoints& breakpoints,
	       const std::vector<linkage_t>& linkages,
		   const EmuOptions& options);
	jit_block_t get_result();

private:
	// Jitter instruction handler
	typedef bool (Jitter::*inst_handler_t)(vaddr_t, uint32_t);

	const Mmu& mmu;
	size_t cov_map_size;
	const Breakpoints& breakpoints;
	std::vector<linkage_t> linkages;
	const EmuOptions& options;

	static CovIdProvider cov_id_provider;

	std::unique_ptr<llvm::LLVMContext> p_context;
	std::unique_ptr<llvm::Module>      p_module;
	llvm::LLVMContext& context;
	llvm::Module&      module;
	llvm::IRBuilder<>  builder;
	llvm::Function*    function;
	llvm::Value*       p_instr;

	llvm::Value*       p_fault_pc;
	llvm::BasicBlock*  fault_path;
	llvm::BasicBlock*  end_path;

	struct {
		llvm::Value* p_regs;
		llvm::Value* p_memory;
		llvm::Value* p_perms;
		llvm::Value* p_dirty_vec;
		llvm::Value* p_dirty_size;
		llvm::Value* p_dirty_map;
		llvm::Value* p_fpregs;
		llvm::Value* p_ccs;
	} state;

	llvm::Value* p_regs[34];
	std::bitset<34> must_be_loaded;
	std::unordered_map<llvm::BasicBlock*, std::bitset<34>> must_be_saved;

	typedef std::unordered_set<llvm::BasicBlock*> handled_t;
	void load_reg(uint8_t reg);
	void save_reg(uint8_t reg);
	void load_regs();
	void save_regs();
	void join_must_be_saved(llvm::BasicBlock* from, llvm::BasicBlock* succ);
	void update_must_be_saved(llvm::BasicBlock* block, handled_t& handled);
	void gen_save_regs();
	void link_stuff(llvm::ExecutionEngine* ee);

	// Map of created basic blocks. A basic block is not created if it
	// is registered here. This way we avoid things like infinite recursion
	// in loops
	std::unordered_map<vaddr_t, llvm::BasicBlock*> basic_blocks;

	// Resulting jit block
	jit_block_t result;

	// Basic data types
	llvm::Type* void_ty;
	llvm::Type* int1_ty;
	llvm::Type* int8_ty;
	llvm::Type* int16_ty;
	llvm::Type* int32_ty;
	llvm::Type* int64_ty;
	llvm::Type* float_ty;
	llvm::Type* double_ty;
	llvm::Type* int1ptr_ty;
	llvm::Type* int8ptr_ty;
	llvm::Type* int32ptr_ty;
	llvm::Type* int16ptr_ty;
	llvm::Type* int64ptr_ty;
	llvm::Type* floatptr_ty;
	llvm::Type* doubleptr_ty;


	// Attempt to load JIT code associated to `name` from disk cache
	bool load_from_disk(const std::string& name);

	// Read a field from vm_state struct
	llvm::Value* get_state_field(uint8_t field, const std::string& name);

	// Get pointer to general purpose register 
	llvm::Value* get_preg(uint8_t reg);

	// Get pointer to single precission FPU register
	llvm::Value* gets_preg(uint8_t reg);

	// Get pointer to double precission FPU register
	llvm::Value* getd_preg(uint8_t reg);

	// Get or set a general purpose register
	llvm::Value* get_reg(uint8_t reg);
	void set_reg(uint8_t reg, llvm::Value* val);
	void set_reg(uint8_t reg, uint32_t val);

	// Get or set a single precission FPU register
	llvm::Value* gets_reg(uint8_t reg);
	void sets_reg(uint8_t reg, llvm::Value* val);
	void sets_reg(uint8_t reg, float val);

	// Get or set a single double FPU register
	llvm::Value* getd_reg(uint8_t reg);
	void setd_reg(uint8_t reg, llvm::Value* val);
	void setd_reg(uint8_t reg, double val);

	// Get or set a FPU condition code
	llvm::Value* get_cc(uint8_t cc);
	void set_cc(uint8_t cc, llvm::Value* val);
	void set_cc(uint8_t cc, bool val);

	// Get pointer to virtual memory address `addr`
	llvm::Value* get_pmemory(llvm::Value* addr);

	// Generate vm exit with given information
	void vm_exit(ExitInfo::ExitReason reason, llvm::Value* reenter_pc=NULL,
	             llvm::Value* info1=NULL, llvm::Value* info2=NULL);

	// Methods for reporting code coverage. They return whether they generated
	// a vm exit or not.
	// For conditional branches. It gets unused coverage ids
	bool add_coverage(vaddr_t pc, vaddr_t jump1, vaddr_t jump2,
	                  llvm::Value* cmp);

	// For unconditional branches. It gets unused coverage id
	bool add_coverage(vaddr_t pc, vaddr_t jump);

	// For indirect branches. It generates a hash based on `from` and `to`
	// which is used as coverage id
	bool add_coverage(llvm::Value* from, llvm::Value* to);

	// General method
	bool add_coverage(llvm::Value* cov_id, llvm::Value* from, llvm::Value* to);

	// Generation of memory access checks. We also generate a fault vm exit
	// so faults are notified at runtime if any check fails.

	// Generate bounds checks for memory access
	void check_bounds_mem(llvm::Value* addr, vsize_t len);

	// Generate perms checks for memory access
	void check_perms_mem(llvm::Value* addr, vsize_t len, uint8_t perm);

	// Generate alignment checks for memory access
	void check_alignment_mem(llvm::Value* addr, vsize_t len);

	// Set memory region from `addr` to `addr+len` as initialized
	void set_init(llvm::Value* addr, vsize_t len);

	// Set block corresponding to `addr` as dirty
	void set_dirty_mem(llvm::Value* addr);

	// Reads a value from memory. Checks bounds, perms and alignment
	llvm::Value* read_mem(llvm::Value* addr, vsize_t len, vaddr_t pc,
	                      bool chk_uninit=CHECK_UNINIT);

	// Writes a value to memory. Checks bounds, perms and alignment
	void write_mem(llvm::Value* addr, llvm::Value* value, vsize_t len,
	               vaddr_t pc);

	// Lift code at `pc`. It calls instruction handlers, which eventually
	// create other blocks until every path leads to a vm exit
	llvm::BasicBlock* create_block(vaddr_t pc);

	// Handle instruction at `pc`. It calls the proper instruction handler,
	// which will lift code for that instruction
	bool handle_inst(vaddr_t pc);

	// Compile module: optimize IR, compile it and dump it to disk
	void compile();

	// Instruction handlers
	static const inst_handler_t inst_handlers[];
	static const inst_handler_t inst_handlers_R[];
	static const inst_handler_t inst_handlers_RI[];
	static const inst_handler_t inst_handlers_special2[];
	static const inst_handler_t inst_handlers_special3[];
	static const inst_handler_t inst_handlers_COP1[];

	bool inst_R(vaddr_t, uint32_t);
	bool inst_RI(vaddr_t, uint32_t);
	bool inst_special2(vaddr_t, uint32_t);
	bool inst_special3(vaddr_t, uint32_t);
	bool inst_COP1(vaddr_t, uint32_t);

	bool inst_test(vaddr_t, uint32_t);
	bool inst_unimplemented(vaddr_t, uint32_t);
	bool inst_or(vaddr_t, uint32_t);
	bool inst_bgezal(vaddr_t, uint32_t);
	bool inst_nop(vaddr_t, uint32_t);
	bool inst_lui(vaddr_t, uint32_t);
	bool inst_addiu(vaddr_t, uint32_t);
	bool inst_lw(vaddr_t, uint32_t);
	bool inst_and(vaddr_t, uint32_t);
	bool inst_sw(vaddr_t, uint32_t);
	bool inst_jalr(vaddr_t, uint32_t);
	bool inst_beq(vaddr_t, uint32_t);
	bool inst_addu(vaddr_t, uint32_t);
	bool inst_sll(vaddr_t, uint32_t);
	bool inst_bne(vaddr_t, uint32_t);
	bool inst_jr(vaddr_t, uint32_t);
	bool inst_lhu(vaddr_t, uint32_t);
	bool inst_syscall(vaddr_t, uint32_t);
	bool inst_xor(vaddr_t, uint32_t);
	bool inst_sltu(vaddr_t, uint32_t);
	bool inst_sltiu(vaddr_t, uint32_t);
	bool inst_movn(vaddr_t, uint32_t);
	bool inst_movz(vaddr_t, uint32_t);
	bool inst_subu(vaddr_t, uint32_t);
	bool inst_teq(vaddr_t, uint32_t);
	bool inst_div(vaddr_t, uint32_t);
	bool inst_divu(vaddr_t, uint32_t);
	bool inst_mflo(vaddr_t, uint32_t);
	bool inst_mul(vaddr_t, uint32_t);
	bool inst_bltz(vaddr_t, uint32_t);
	bool inst_blez(vaddr_t, uint32_t);
	bool inst_rdhwr(vaddr_t, uint32_t);
	bool inst_bgez(vaddr_t, uint32_t);
	bool inst_slti(vaddr_t, uint32_t);
	bool inst_andi(vaddr_t, uint32_t);
	bool inst_ori(vaddr_t, uint32_t);
	bool inst_xori(vaddr_t, uint32_t);
	bool inst_pref(vaddr_t, uint32_t);
	bool inst_jal(vaddr_t, uint32_t);
	bool inst_lb(vaddr_t, uint32_t);
	bool inst_nor(vaddr_t, uint32_t);
	bool inst_bshfl(vaddr_t, uint32_t);
	bool inst_seh(vaddr_t, uint32_t);
	bool inst_seb(vaddr_t, uint32_t);
	bool inst_wsbh(vaddr_t, uint32_t);
	bool inst_srl(vaddr_t, uint32_t);
	bool inst_lh(vaddr_t, uint32_t);
	bool inst_lbu(vaddr_t, uint32_t);
	bool inst_lwl(vaddr_t, uint32_t);
	bool inst_lwr(vaddr_t, uint32_t);
	bool inst_sb(vaddr_t, uint32_t);
	bool inst_sh(vaddr_t, uint32_t);
	bool inst_swl(vaddr_t, uint32_t);
	bool inst_swr(vaddr_t, uint32_t);
	bool inst_sllv(vaddr_t, uint32_t);
	bool inst_srlv(vaddr_t, uint32_t);
	bool inst_slt(vaddr_t, uint32_t);
	bool inst_sub(vaddr_t, uint32_t);
	bool inst_add(vaddr_t, uint32_t);
	bool inst_j(vaddr_t, uint32_t);
	bool inst_ll(vaddr_t, uint32_t);
	bool inst_sc(vaddr_t, uint32_t);
	bool inst_sync(vaddr_t, uint32_t);
	bool inst_bgtz(vaddr_t, uint32_t);
	bool inst_mult(vaddr_t, uint32_t);
	bool inst_multu(vaddr_t, uint32_t);
	bool inst_mfhi(vaddr_t, uint32_t);
	bool inst_mthi(vaddr_t, uint32_t);
	bool inst_mtlo(vaddr_t, uint32_t);
	bool inst_ext(vaddr_t, uint32_t);
	bool inst_ins(vaddr_t, uint32_t);
	bool inst_sra(vaddr_t, uint32_t);
	bool inst_srav(vaddr_t, uint32_t);
	bool inst_clz(vaddr_t, uint32_t);
	bool inst_lwc1(vaddr_t, uint32_t);
	bool inst_swc1(vaddr_t, uint32_t);
	bool inst_ldc1(vaddr_t, uint32_t);
	bool inst_sdc1(vaddr_t, uint32_t);
	bool inst_fmt_s(vaddr_t, uint32_t);
	bool inst_fmt_d(vaddr_t, uint32_t);
	bool inst_fmt_w(vaddr_t, uint32_t);
	bool inst_fmt_l(vaddr_t, uint32_t);
	bool inst_fmt_ps(vaddr_t, uint32_t);
	bool inst_mfc1(vaddr_t, uint32_t);
	bool inst_mfhc1(vaddr_t, uint32_t);
	bool inst_mtc1(vaddr_t, uint32_t);
	bool inst_mthc1(vaddr_t, uint32_t);
	bool inst_c_cond_s(vaddr_t, uint32_t);
	bool inst_c_cond_d(vaddr_t, uint32_t);
	bool inst_bc1(vaddr_t, uint32_t);
	bool inst_cfc1(vaddr_t, uint32_t);
	bool inst_ctc1(vaddr_t, uint32_t);
	bool inst_break(vaddr_t, uint32_t);
};

} // JIT namespace

// This has to be after the JIT namespace :(
struct EmuOptions {
	JIT::jit_cache_t* jit_cache = NULL;
	bool guest_output           = false;
	bool coverage               = true;
	bool dump_pc                = false;
	bool dump_regs              = false;
	bool check_repeated_cov_id  = false;
};

#endif
