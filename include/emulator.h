#ifndef _EMULATOR_H
#define _EMULATOR_H

#include <unordered_map>
#include "mmu.h"

// Implement prefetch properly
// Fix sbrk

class Emulator;
typedef void (Emulator::*const inst_handler_t)(uint32_t);
typedef void (Emulator::*breakpoint_t)();

enum Reg {
	zero, at, v0, v1, a0, a1, a2, a3,
	t0,   t1, t2, t3, t4, t5, t6, t7,
	s0,   s1, s2, s3, s4, s5, s6, s7,
	t8,   t9, k0, k1, gp, sp, fp, ra,
};

struct guest_iovec {
	vaddr_t iov_base;
	vsize_t iov_len;
};

struct guest_uname {
	char sysname[65];
	char nodename[65];
	char release[65];
	char version[65];
	char machine[65];
	//char domainname[65];
};

class Emulator {
	private:
		// Memory
		Mmu mmu;

		// Registers
		uint32_t  regs[32];
		uint32_t  hi, lo;
		vaddr_t   pc;

		// Handling of branches. If condition is true, pc must be updated to
		// jump_addr in next cycle
		bool      condition;
		vaddr_t   jump_addr;

		vaddr_t   tls;

		// Breakpoints hash table
		static const std::unordered_map<vaddr_t, breakpoint_t> breakpoints;

		// Breakpoints
		void test_bp();
		void sbrk_bp();
		void malloc_bp();   // __libc_malloc
		void free_bp();     // __free
		void realloc_bp();  // __libc_realloc
		void memalign_bp(); // __libc_memalign
		void valloc_bp();   // __libc_valloc
		void pvalloc_bp();  // pvalloc
		void calloc_bp();   // __calloc

		// Syscalls
		uint32_t sys_brk(vaddr_t new_brk, uint32_t& error);
		uint32_t sys_openat(int32_t dirfd, vaddr_t pathname_addr, int32_t flags,
		                    uint32_t& error);
		uint32_t sys_writev(int32_t fd, vaddr_t iov_addr, int32_t iovcnt,
		                    uint32_t& error);
		vaddr_t sys_mmap2(vaddr_t addr, vsize_t length, uint32_t prot,
		                  uint32_t flags, uint32_t fd, uint32_t pgoffset,
		                  uint32_t& error);
		uint32_t sys_uname(vaddr_t addr, uint32_t& error);
		uint32_t sys_readlink(vaddr_t pathname_addr, vaddr_t buf_addr,
		                      vaddr_t bufsize, uint32_t& error);

		void handle_syscall(uint32_t syscall);

		void run_inst();


	public:
		//Emulator();

		Emulator(vsize_t mem_size);

		void set_reg(uint8_t reg, uint32_t val);
		uint32_t get_reg(uint8_t reg);

		void set_pc(vaddr_t addr);
		uint32_t get_pc();

		// Push an element to the stack
		template<class T>
		void push_stack(T val);

		// Forks the emulator and returns the child
		Emulator fork();

		// Resets the emulator to the parent it was previously forked from
		void reset(const Emulator& other);

		// Load elf into memory, allocate stack and load argv into the stack
		void load_elf(const char* pathname, const std::vector<std::string>& argv);

		void run();

		friend std::ostream& operator<<(std::ostream& os, const Emulator& emu);

	private: // Instruction handlers
		static const inst_handler_t inst_handlers[];
		static const inst_handler_t inst_handlers_R[];
		static const inst_handler_t inst_handlers_RI[];
		static const inst_handler_t inst_handlers_special2[];
		static const inst_handler_t inst_handlers_special3[];

		void inst_R(uint32_t);
		void inst_RI(uint32_t);
		void inst_special2(uint32_t);
		void inst_special3(uint32_t);

		void inst_test(uint32_t);
		void inst_unimplemented(uint32_t);
		void inst_or(uint32_t);
		void inst_bgezal(uint32_t);
		void inst_nop(uint32_t);
		void inst_lui(uint32_t);
		void inst_addiu(uint32_t);
		void inst_lw(uint32_t);
		void inst_and(uint32_t);
		void inst_sw(uint32_t);
		void inst_jalr(uint32_t);
		void inst_beq(uint32_t);
		void inst_addu(uint32_t);
		void inst_sll(uint32_t);
		void inst_bne(uint32_t);
		void inst_jr(uint32_t);
		void inst_lhu(uint32_t);
		void inst_syscall(uint32_t);
		void inst_xor(uint32_t);
		void inst_sltu(uint32_t);
		void inst_sltiu(uint32_t);
		void inst_movn(uint32_t);
		void inst_movz(uint32_t);
		void inst_subu(uint32_t);
		void inst_teq(uint32_t);
		void inst_divu(uint32_t);
		void inst_mflo(uint32_t);
		void inst_mul(uint32_t);
		void inst_bltz(uint32_t);
		void inst_blez(uint32_t);
		void inst_rdhwr(uint32_t);
		void inst_bgez(uint32_t);
		void inst_slti(uint32_t);
		void inst_andi(uint32_t);
		void inst_ori(uint32_t);
		void inst_xori(uint32_t);
		void inst_pref(uint32_t);
		void inst_jal(uint32_t);
		void inst_lb(uint32_t);
		void inst_nor(uint32_t);
		void inst_bshfl(uint32_t);
		void inst_seh(uint32_t);
		void inst_srl(uint32_t);
		void inst_lh(uint32_t);
		void inst_lbu(uint32_t);
		void inst_lwl(uint32_t);
		void inst_lwr(uint32_t);
		void inst_sb(uint32_t);
		void inst_sh(uint32_t);
		void inst_swl(uint32_t);
		void inst_swr(uint32_t);
		void inst_sllv(uint32_t);
		void inst_slt(uint32_t);
		void inst_sub(uint32_t);
		void inst_add(uint32_t);
		void inst_j(uint32_t);
		void inst_ll(uint32_t);
		void inst_sc(uint32_t);
		void inst_sync(uint32_t);
		void inst_bgtz(uint32_t);
		void inst_mult(uint32_t);
		void inst_multu(uint32_t);
		void inst_mfhi(uint32_t);
		void inst_mthi(uint32_t);
		void inst_mtlo(uint32_t);
		void inst_ext(uint32_t);
};

template<class T>
void Emulator::push_stack(T val){
	regs[Reg::sp] -= sizeof(T);
	mmu.write<T>(regs[Reg::sp], val);
}

struct inst_R_t {
	uint8_t s;
	uint8_t t;
	uint8_t d;
	uint8_t S;
	
	inst_R_t(uint32_t inst){
		s = (inst >> 21) & 0b00011111;
		t = (inst >> 16) & 0b00011111;
		d = (inst >> 11) & 0b00011111;
		S = (inst >> 6)  & 0b00011111;
	}
};

struct inst_RI_t {
	uint8_t  s;
	uint16_t C;
	
	inst_RI_t(uint32_t inst){
		s = (inst >> 21) & 0b00011111;
		C = inst & 0b11111111'11111111;
	}
};

struct inst_I_t {
	uint8_t  s;
	uint8_t  t;
	uint16_t C;

	inst_I_t(uint32_t inst){
		s = (inst >> 21) & 0b00011111;
		t = (inst >> 16) & 0b00011111;
		C = inst & 0b11111111'11111111;
	}
};

struct inst_J_t {
	uint32_t A;

	inst_J_t(uint32_t inst){
		A = inst & 0b11'11111111'11111111'11111111;
	}
};

#endif