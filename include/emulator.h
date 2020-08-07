#ifndef _EMULATOR_H
#define _EMULATOR_H

#include "mmu.h"

class Emulator;
typedef void (Emulator::*const inst_handler)(uint32_t);

class Emulator {
	private:
		Mmu mmu;

		uint32_t regs[32];
		uint32_t hi, lo;
		addr_t   pc;

		void handle_syscall(uint32_t syscall);

		void run_inst();

	public:
		//Emulator();

		Emulator(size_t mem_size);

		Emulator fork();

		void reset(const Emulator& other);

		void load_elf(const char* pathname);

		void run();

	private: // Instruction handlers
		static const inst_handler table[];

		void inst_R(uint32_t);
		void inst_RI(uint32_t);

		void inst_unimplemented(uint32_t);
};

#endif