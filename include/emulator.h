#ifndef _EMULATOR_H
#define _EMULATOR_H

#include "mmu.h"

class Emulator;
typedef void (Emulator::*const inst_handler)(uint32_t);

class Emulator {
	private:
		Mmu mmu;

		uint32_t  regs[32];
		uint32_t  hi, lo;
		addr_t    pc;

		bool      condition;
		int32_t   jump_offset;

		void handle_syscall(uint32_t syscall);

		void run_inst();

	public:
		//Emulator();

		Emulator(size_t mem_size);

		void set_reg(uint8_t reg, uint32_t val);
		uint32_t get_reg(uint8_t reg);

		void set_pc(addr_t addr);
		uint32_t get_pc();

		Emulator fork();

		void reset(const Emulator& other);

		void load_elf(const char* pathname, const std::vector<std::string>& argv);

		void run();

		friend std::ostream& operator<<(std::ostream& os, const Emulator& emu);

	private: // Instruction handlers
		static const inst_handler inst_handlers[];
		static const inst_handler inst_handlers_R[];
		static const inst_handler inst_handlers_RI[];

		void inst_R(uint32_t);
		void inst_RI(uint32_t);

		void inst_test(uint32_t);
		void inst_unimplemented(uint32_t);
		void inst_or(uint32_t);
		void inst_bgezal(uint32_t);
		void inst_nop(uint32_t);
		void inst_lui(uint32_t);
		void inst_addiu(uint32_t);
		void inst_lw(uint32_t);
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