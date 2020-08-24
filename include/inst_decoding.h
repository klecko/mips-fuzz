#ifndef _INST_DECODING_H
#define _INST_DECODING_H

#include <cstdint>

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
		C = inst & 0xFFFF;
	}
};

struct inst_I_t {
	uint8_t  s;
	uint8_t  t;
	uint16_t C;

	inst_I_t(uint32_t inst){
		s = (inst >> 21) & 0b00011111;
		t = (inst >> 16) & 0b00011111;
		C = inst & 0xFFFF;
	}
};

struct inst_J_t {
	uint32_t A;

	inst_J_t(uint32_t inst){ // 26 bits
		A = inst & 0x3FFFFFF;
	}
};

// Used for FPU
struct inst_F_t {
	uint8_t t;
	uint8_t s;
	uint8_t d;
	uint8_t funct;

	inst_F_t(uint32_t inst){
		t     = (inst >> 16) & 0b00011111;
		s     = (inst >> 11) & 0b00011111;
		d     = (inst >> 6)  & 0b00011111;
		funct = inst & 0b00111111;
	}
};

#endif