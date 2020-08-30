#ifndef _JITTER_H
#define _JITTER_H

#include <unordered_map>
#include "llvm/IR/IRBuilder.h"
#include "common.h"
#include "mmu.h"
#include "fault.h"

enum Reg {
	zero, at, v0, v1, a0, a1, a2, a3,
	t0,   t1, t2, t3, t4, t5, t6, t7,
	s0,   s1, s2, s3, s4, s5, s6, s7,
	t8,   t9, k0, k1, gp, sp, fp, ra,
	hi,   lo,
};

struct jit_state {
	uint32_t* regs;
	float*    fpregs;
	uint8_t*  memory;
	uint8_t*  perms;
};

struct exit_info {
	enum ExitReason: uint32_t {
		Syscall = 0,
		Fault,
		IndirectBranch,
		Rdhwr,
		Exception, // trap or breakpoint
	};
	ExitReason reason;
	vaddr_t    reenter_pc;
	uint32_t   info1;
	uint32_t   info2;
	friend std::ostream& operator<<(std::ostream& os, const exit_info& exit_inf);
};

typedef void (*jit_block_t)(jit_state*, exit_info*, uint32_t*, uint32_t arr[][35]);
typedef std::unordered_map<vaddr_t, jit_block_t> jit_cache_t;

class Jitter;
typedef bool (Jitter::*const inst_handler_jit_t)(vaddr_t, uint32_t);

class Jitter {
	private:
		const Mmu& mmu;

		std::unique_ptr<llvm::LLVMContext> p_context;
		std::unique_ptr<llvm::Module>      p_module;
		llvm::LLVMContext& context;
		llvm::Module&      module;
		llvm::IRBuilder<>  builder;
		llvm::Function*    function;
		std::unordered_map<vaddr_t, llvm::BasicBlock*> basic_blocks;
		jit_block_t        result;

		llvm::Value* get_preg(uint8_t reg);
		llvm::Value* get_reg(uint8_t reg);
		void set_reg(uint8_t reg, llvm::Value* val);
		void set_reg(uint8_t reg, uint32_t val);

		llvm::Value* get_pmemory(llvm::Value* addr);

		void gen_vm_exit(exit_info::ExitReason reason, llvm::Value* reenter_pc,
		                llvm::Value* info1=NULL, llvm::Value* info2=NULL);

		void check_bounds_mem(llvm::Value* addr, vsize_t len, uint8_t perm,
		                      vaddr_t pc);

		void check_perms_mem(llvm::Value* addr, vsize_t len, uint8_t perm,
		                     vaddr_t pc);

		void check_alignment_mem(llvm::Value* addr, vsize_t len, uint8_t perm, 
		                         vaddr_t pc);

		// Reads a value from memory. Checks bounds, perms and alignment
		llvm::Value* read_mem(llvm::Value* addr, vsize_t len, vaddr_t pc);

		// Writes a value to memory. Checks bounds, perms and alignment
		void write_mem(llvm::Value* addr, llvm::Value* value, vsize_t len,
		               vaddr_t pc);

		llvm::BasicBlock* create_block(vaddr_t pc);
		bool handle_inst(vaddr_t pc);
		void compile();

	public:
		Jitter(vaddr_t pc, const Mmu& mmu);
		jit_block_t get_result();

	private: // Instruction handlers
		static const inst_handler_jit_t inst_handlers[];
		static const inst_handler_jit_t inst_handlers_R[];
		static const inst_handler_jit_t inst_handlers_RI[];
		static const inst_handler_jit_t inst_handlers_special2[];
		static const inst_handler_jit_t inst_handlers_special3[];
		static const inst_handler_jit_t inst_handlers_COP1[];

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
		bool inst_break(vaddr_t, uint32_t);
};


#endif