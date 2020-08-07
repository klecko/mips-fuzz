#ifndef _EMULATOR_H
#define _EMULATOR_H

#include "mmu.h"

class Emulator {
	private:
		Mmu mmu;

		uint32_t regs[32];
		uint32_t hi, lo;
		uint32_t pc;

		void run_emu();

	public:
		//Emulator();

		Emulator(size_t mem_size);

		Emulator fork();

		void load_elf(const char* pathname);

		void run();
};

#endif