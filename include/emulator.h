#ifndef _EMULATOR_H
#define _EMULATOR_H

#include "mmu.h"

// Implement prefetch properly

class Emulator;
typedef void (Emulator::*const inst_handler)(uint32_t);

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

		uint32_t sys_brk(vaddr_t addr);
		void handle_syscall(uint32_t syscall);

		void run_inst();

	public:
		//Emulator();

		Emulator(vsize_t mem_size);

		void set_reg(uint8_t reg, uint32_t val);
		uint32_t get_reg(uint8_t reg);

		void set_pc(vaddr_t addr);
		uint32_t get_pc();

		// Forks the emulator and returns the child
		Emulator fork();

		// Resets the emulator to the parent it was previously forked from
		void reset(const Emulator& other);

		// Load elf into memory, allocate stack and load argv into the stack
		void load_elf(const char* pathname, const std::vector<std::string>& argv);

		void run();

		friend std::ostream& operator<<(std::ostream& os, const Emulator& emu);

	private: // Instruction handlers
		static const inst_handler inst_handlers[];
		static const inst_handler inst_handlers_R[];
		static const inst_handler inst_handlers_RI[];
		static const inst_handler inst_handlers_special2[];
		static const inst_handler inst_handlers_special3[];

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
};

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

#endif