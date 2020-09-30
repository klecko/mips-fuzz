// Jitter will use hacky registers, so it will use NUM_REGS+3
const int NUM_REGS = 32;
enum Reg {
	zero, at, v0, v1, a0, a1, a2, a3,
	t0,   t1, t2, t3, t4, t5, t6, t7,
	s0,   s1, s2, s3, s4, s5, s6, s7,
	t8,   t9, k0, k1, gp, sp, fp, ra,
	hi,   lo, pc // hacky registers
};

struct EmuOptions {
	JIT::jit_cache_t* jit_cache = NULL;
	bool guest_output = false;
	bool coverage     = true;
	bool dump_pc      = false;
	bool dump_regs    = false;
	bool check_repeated_cov_id = false;
};