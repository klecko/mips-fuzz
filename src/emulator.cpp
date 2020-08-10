#include <cstring>
#include <iomanip>
#include <assert.h>
#include <unistd.h>
#include <string>
#include "emulator.h"
#include "elf_parser.hpp"
#include "common.h"

using namespace std;

const std::unordered_map<vaddr_t, breakpoint_t> Emulator::breakpoints = {
	//{0x0042b1a0, &Emulator::sbrk_bp},
	{0x00423f00, &Emulator::malloc_bp},
	{0x00424634, &Emulator::free_bp},
	{0x004248d8, &Emulator::realloc_bp},
	{0x00424d20, &Emulator::memalign_bp},
	{0x00424d3c, &Emulator::valloc_bp},
	{0x00424db8, &Emulator::pvalloc_bp},
	{0x00424e64, &Emulator::calloc_bp},
	//{0x00403ea4, &Emulator::test_bp},
};

Emulator::Emulator(vsize_t mem_size, const string& filepath,
                   const vector<string>& argv): mmu(mem_size)
{
	memset(regs, 0, sizeof(regs));
	hi = 0;
	lo = 0;
	pc = 0;
	condition = false;
	jump_addr = 0;
	tls       = 0;
	running   = false;
	input     = NULL;
	input_sz  = 0;
	load_elf(filepath, argv);
}

void Emulator::set_reg(uint8_t reg, uint32_t val){
	assert(0 <= reg && reg <= 31);
	if (reg != 0)
		regs[reg] = val;
}

uint32_t Emulator::get_reg(uint8_t reg){
	assert(0 <= reg && reg <= 31);
	return (reg ? regs[reg] : 0);
}

void Emulator::set_pc(vaddr_t addr){
	pc = addr;
}

uint32_t Emulator::get_pc(){
	return pc;
}

Emulator Emulator::fork(){
	Emulator new_emu(*this);
	new_emu.mmu = mmu.fork();
	return new_emu;
}

void Emulator::reset(const Emulator& other){
	mmu.reset(other.mmu);
	memcpy(regs, other.regs, sizeof(regs));
	hi         = other.hi;
	lo         = other.lo;
	pc         = other.pc;
	condition  = other.condition;
	jump_addr  = other.jump_addr;
	tls        = other.tls;
	running    = other.running;
	input      = other.input;
	input_sz   = other.input_sz;
	open_files = other.open_files;
}


void Emulator::load_elf(const string& filepath, const vector<string>& argv){
	// Stack layout described in
	// http://articles.manugarg.com/aboutelfauxiliaryvectors.html
	// I add random numbers at the bottom of the stack for auxv
	Elf_parser elf(filepath);
	vaddr_t load_addr;
	mmu.load_elf(elf.get_segments(load_addr));

	// Set entry
	pc = elf.get_entry();
	printf("Entry 0x%X\n", pc);

	// Allocate the stack
	regs[Reg::sp] = mmu.alloc_stack(2 * 1024 * 1024);
	printf("Allocated stack at 0x%X\n", regs[Reg::sp]);

	push_stack(0);

	// Generate random bytes for auxv. Note seed is not initialized
	regs[Reg::sp] -= 16;
	vaddr_t random_bytes = regs[Reg::sp];
	for (int i = 0; i < 16; i++)
		mmu.write<uint8_t>(random_bytes + i, rand());

	// Load ascii args into the stack
	vector<vaddr_t> argv_vm;
	for (const string& arg : argv){
		regs[Reg::sp] -= arg.size()+1;
		mmu.write_mem(regs[Reg::sp], arg.c_str(), arg.size()+1);
		argv_vm.push_back(regs[Reg::sp]);
	}
	argv_vm.push_back(0); // Argv last element must be NULL

	// Align sp
	regs[Reg::sp] = regs[Reg::sp] - 0x3 & ~0x3;

	// Set up auxp. We don't use push_stack or mmu.write because of
	// misalignment checks
	phinfo_t phinfo = elf.get_phinfo();
	Elf32_auxv_t auxv[] = {
		{AT_RANDOM, random_bytes},               // Address of 16 random bytes
		{AT_EXECFN, argv_vm[0]},                 // Filename of the program
	/* 	{AT_PHDR,   load_addr + phinfo.e_phoff}, // Pointer to program headers
		{AT_PHENT,  phinfo.e_phentsize},         // Size of each entry
		{AT_PHNUM,  phinfo.e_phnum},   */           // Number of entries
		{AT_NULL,   0},                          // Auxv end
	};
	regs[Reg::sp] -= sizeof(auxv);
	mmu.write_mem(regs[Reg::sp], auxv, sizeof(auxv));

	// Set up envp
	push_stack(0);

	// Set up argv and argc
	for (auto it = argv_vm.rbegin(); it != argv_vm.rend(); ++it)
		push_stack<vaddr_t>(*it);
	push_stack<uint32_t>(argv.size());
}

// Possibilities: Clean exit, timeout, [exception (fault)]
bool Emulator::run(const string& input){
	// Save provided input. Internal representation is as const char* and not
	// as string so we don't have to perform any copy.
	this->input    = input.c_str();
	this->input_sz = input.size() + 1;

	// Perform execution recording number of executed instructions
	uint64_t instr_exec = 0;
	bool timeout = false;
	running = true;
	while (running && !timeout){
		run_inst();
		instr_exec++;
		if (instr_exec == INSTR_TIMEOUT)
			timeout = true;
		//cout << *this << endl;
	}

	return timeout;
}

void Emulator::test_bp(){
	die("test bp\n");
}

void Emulator::sbrk_bp(){
	int32_t increment = regs[Reg::a0];
	if (increment < 0)
		die("sbrk neg size: %d\n", increment);

	// Perform allocation. Warning: this can be done only once
	vaddr_t addr = mmu.alloc(increment);

	// Special allocation: zeroed and marked as initialized
	for (int i = 0; i < increment; i++)
		mmu.write<uint8_t>(addr + i, 0);

	regs[Reg::v0] = addr;
	pc = regs[Reg::ra];
	printf("sbrk(%d) --> 0x%X\n", increment, addr);
}

void Emulator::malloc_bp(){
	vsize_t size = regs[Reg::a0];
	vaddr_t addr = (size > 0 ? mmu.alloc(size) : 0);
	regs[Reg::v0] = addr;
	pc = regs[Reg::ra];
	printf("malloc(%u) --> 0x%X\n", size, addr);
}

void Emulator::free_bp(){
	//die("free_bp\n");
	vsize_t addr = regs[Reg::a0];
	pc = regs[Reg::ra];
	printf("free(0x%X)\n", addr);
}

void Emulator::realloc_bp(){
	die("realloc_bp\n");
}

void Emulator::memalign_bp(){
	die("memalign_bp\n");
}

void Emulator::valloc_bp(){
	die("valloc_bp\n");
}

void Emulator::pvalloc_bp(){
	die("pvalloc_bp\n");
}

void Emulator::calloc_bp(){
	die("calloc_bp\n");
}

uint32_t Emulator::sys_brk(vaddr_t new_brk, uint32_t& error){
	vaddr_t brk = mmu.get_brk();

	// Attempt to get current brk
	if (!new_brk)
		return brk;

	// Attempt to change brk
	error = !mmu.set_brk(new_brk);
	brk   = (error ? brk : new_brk);
	printf("brk(0x%X) --> 0x%X\n", new_brk, brk);
	return brk;
}

uint32_t Emulator::sys_openat(int32_t dirfd, vaddr_t pathname_addr, int32_t flags,
                              uint32_t& error)
{
	string pathname = mmu.read_string(pathname_addr);
	cout << "Trying to open " << pathname << " as " << flags << endl;

	// Find unused fd
	uint32_t fd = 3;
	while (open_files.count(fd))
		fd++;

	// Create input file
	if (pathname == "input_file"){
		if (flags == O_RDWR || flags == O_WRONLY)
			die("opening input file with write permissions");
		
		// God, forgive me for this casting.
		File input_file(fd, flags, (char*)input, input_sz);
		open_files[fd] = input_file;
		error = 0;
		return fd;
	}

	die("Unimplemented openat\n");
	return 0;
}

uint32_t Emulator::sys_writev(int32_t fd, vaddr_t iov_addr, int32_t iovcnt,
                              uint32_t& error)
{
	die("writev\n");
	if (fd != STDOUT_FILENO)
		die("sys_writev trying to write to fd %d\n", fd);

	// Return value
	uint32_t bytes_written;

	// Read iovec structs from guest memory
	guest_iovec iov[iovcnt];
	mmu.read_mem(&iov, iov_addr, iovcnt*sizeof(guest_iovec));

	// For each iovec struct, read its content and print it
	vaddr_t iov_base;
	vsize_t iov_len;
	for (int i = 0; i < iovcnt; i++){
		iov_base = iov[i].iov_base;
		iov_len  = iov[i].iov_len;
		char buf[iov_len + 1];
		mmu.read_mem(buf, iov_base, iov_len);
		buf[iov_len] = 0;
		printf("output: %s\n", buf);
		bytes_written += iov_len;
	}

	error = 0;
	return bytes_written;
}

vaddr_t Emulator::sys_mmap2(vaddr_t addr, vsize_t length, uint32_t prot,
                            uint32_t flags, uint32_t fd, uint32_t pgoffset,
                            uint32_t& error)
{
	die("mmap2(0x%X, 0x%X, 0x%X, 0x%X, %d, 0x%X)\n", addr, length, prot,
	    flags, fd, pgoffset);
	return 0;
}

uint32_t Emulator::sys_uname(vaddr_t addr, uint32_t& error){
	guest_uname uname = {
		"Linux",    // sysname
		"Baby emu", // nodename
		"666",      // release
		"666",      // version
		"mips"      // machine
	};
	mmu.write_mem(addr, &uname, sizeof(uname));
	error = 0;
	return 0;
}

uint32_t Emulator::sys_readlink(vaddr_t pathname_addr, vaddr_t buf_addr,
		                        vaddr_t bufsize, uint32_t& error)
{
	string pathname = mmu.read_string(pathname_addr);
	if (pathname == "/proc/self/exe"){
		char s[] = "/home/klecko/my_fuzzer";
		if (bufsize < sizeof(s)){
			die("error readlink\n");
			error = 1;
			return -1;
		}
		mmu.write_mem(buf_addr, s, sizeof(s));
		error = 0;
		return sizeof(s);
	}
	
	cout << pathname <<  endl;
	die("Unimplemented readlink\n");
	return 0;
}

uint32_t Emulator::sys_read(uint32_t fd, vaddr_t buf_addr, vsize_t count,
                            uint32_t& error)
{
	// Stdin, stdout and stderr
	switch (fd){
		case STDIN_FILENO:
			die("reading from stdin\n");
			break;

		case STDOUT_FILENO:
			die("reading from stdout?\n");
			break;

		case STDERR_FILENO:
			die("reading from stderr?\n");
			break;
	}

	// Check if file is open
	if (!open_files.count(fd))
		die("reading from not used fd\n");

	// Check if file is readable
	File& f = open_files[fd];
	if (!f.is_readable())
		die("reading from non readable fd\n");

	// Read
	char* cursor = f.get_cursor();
	count = f.move_cursor(count);
	mmu.write_mem(buf_addr, cursor, count);

	error = 0;
	return count;
}

uint32_t Emulator::sys_write(uint32_t fd, vaddr_t buf_addr, vsize_t count,
                             uint32_t& error)
{
	// Stdin, stdout and stderr
	switch (fd){
		case STDIN_FILENO:
			die("writing to stdin?\n");
			break;

		case STDOUT_FILENO:
		case STDERR_FILENO:
			char buf[count + 1];
			mmu.read_mem(buf, buf_addr, count);
			buf[count] = 0;
			printf("output:\n%s\n", buf);
			error = 0;
			return count;
	}

	// Check if file is open
	if (!open_files.count(fd))
		die("writing to not used fd\n");

	// Check if file is writable
	File& f = open_files[fd];
	if (!f.is_writable())
		die("writing to non writable fd\n");

	// Write
	char* cursor = f.get_cursor();
	count = f.move_cursor(count);
	mmu.read_mem(cursor, buf_addr, count);

	error = 0;
	return count;
}

uint32_t Emulator::sys_fstat64(uint32_t fd, vaddr_t statbuf_addr,
                               uint32_t& error)
{
	error = 1;
	return -1;
}

uint32_t Emulator::sys_close(uint32_t fd, uint32_t& error){
	if (fd == STDIN_FILENO || fd == STDOUT_FILENO || fd == STDERR_FILENO)
		goto end;

	// Check if file is open
	if (!open_files.count(fd))
		die("closing to not used fd %u\n", fd);
	
	open_files.erase(open_files.find(fd));

end:
	error = 0;
	return 0;
}

void Emulator::handle_syscall(uint32_t syscall){
	switch (syscall){
		case 4001: // exit
		case 4246: // exit_group
			running = false;
			die("EXIT\n");
			break;

		case 4003: // read
			regs[Reg::v0] = sys_read(
				regs[Reg::a0],
				regs[Reg::a1],
				regs[Reg::a2],
				regs[Reg::a3]
			);
			break;
		
		case 4004: // write
			regs[Reg::v0] = sys_write(
				regs[Reg::a0],
				regs[Reg::a1],
				regs[Reg::a2],
				regs[Reg::a3]
			);
			break;


		case 4006: // close
			regs[Reg::v0] = sys_close(regs[Reg::a0], regs[Reg::a3]);
			break;

		case 4045: // brk
			regs[Reg::v0] = sys_brk(regs[Reg::a0], regs[Reg::a3]);
			break;

		case 4024: // getuid
		case 4047: // getgid
		case 4049: // geteuid
		case 4050: // getegid
			regs[Reg::v0] = 0;
			regs[Reg::a3] = 0;
			break;

		case 4085: // readlink
			regs[Reg::v0] = sys_readlink(
				regs[Reg::a0],
				regs[Reg::a1],
				regs[Reg::a2],
				regs[Reg::a3]
			);
			break;

		case 4122: // uname
			regs[Reg::v0] = sys_uname(regs[Reg::a0], regs[Reg::a3]);
			break;

		case 4146: // writev
			regs[Reg::v0] = sys_writev(
				regs[Reg::a0],
				regs[Reg::a1],
				regs[Reg::a2],
				regs[Reg::a3]
			);
			break;

		case 4210: // mmap2
			regs[Reg::v0] = sys_mmap2(
				regs[Reg::a0],
				regs[Reg::a1],
				regs[Reg::a2],
				regs[Reg::a3],
				mmu.read<uint32_t>(regs[Reg::sp]+16),
				mmu.read<uint32_t>(regs[Reg::sp]+20),
				regs[Reg::a3]
			);
			break;

		case 4215: // fstat64
			regs[Reg::v0] = 
				sys_fstat64(regs[Reg::a0], regs[Reg::a1], regs[Reg::a3]);
			break;

		case 4283: // set_thread_area
			//die("set_thread_area(0x%X)\n", regs[Reg::a0]);
			tls = regs[Reg::a0];
			printf("set tls --> 0x%X\n", tls);
			regs[Reg::v0] = 0;
			regs[Reg::a3] = 0;
			break;

		case 4288: // openat
			regs[Reg::v0] = sys_openat(
				regs[Reg::a0],
				regs[Reg::a1],
				regs[Reg::a2],
				regs[Reg::a3]
			);
			//die("openat\n");
			break;

		default:
			die("Unimplemented syscall: %d\n", syscall);
	}
}


// Instruction handlers indexed by opcode
const inst_handler_t Emulator::inst_handlers[] = {
	&Emulator::inst_R,             // 000 000
	&Emulator::inst_RI,            // 000 001
	&Emulator::inst_j,             // 000 010
	&Emulator::inst_jal,           // 000 011
	&Emulator::inst_beq,           // 000 100
	&Emulator::inst_bne,           // 000 101
	&Emulator::inst_blez,          // 000 110
	&Emulator::inst_bgtz,          // 000 111
	&Emulator::inst_unimplemented, // 001 000
	&Emulator::inst_addiu,         // 001 001
	&Emulator::inst_slti,          // 001 010
	&Emulator::inst_sltiu,         // 001 011
	&Emulator::inst_andi,          // 001 100
	&Emulator::inst_ori,           // 001 101
	&Emulator::inst_xori,          // 001 110
	&Emulator::inst_lui,           // 001 111
	&Emulator::inst_unimplemented, // 010 000
	&Emulator::inst_unimplemented, // 010 001
	&Emulator::inst_unimplemented, // 010 010
	&Emulator::inst_unimplemented, // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_unimplemented, // 011 000
	&Emulator::inst_unimplemented, // 011 001
	&Emulator::inst_unimplemented, // 011 010
	&Emulator::inst_unimplemented, // 011 011
	&Emulator::inst_special2,      // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_special3,      // 011 111
	&Emulator::inst_lb,            // 100 000
	&Emulator::inst_lh,            // 100 001
	&Emulator::inst_lwl,           // 100 010
	&Emulator::inst_lw,            // 100 011
	&Emulator::inst_lbu,           // 100 100
	&Emulator::inst_lhu,           // 100 101
	&Emulator::inst_lwr,           // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_sb,            // 101 000
	&Emulator::inst_sh,            // 101 001
	&Emulator::inst_swl,           // 101 010
	&Emulator::inst_sw,            // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_swr,           // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_ll,            // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_pref,          // 110 011
	&Emulator::inst_unimplemented, // 110 100
	&Emulator::inst_unimplemented, // 110 101
	&Emulator::inst_unimplemented, // 110 110
	&Emulator::inst_unimplemented, // 110 111
	&Emulator::inst_sc,            // 111 000
	&Emulator::inst_unimplemented, // 111 001
	&Emulator::inst_unimplemented, // 111 010
	&Emulator::inst_unimplemented, // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_sdc1,          // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

// Type R instruction handlers indexed by functor
const inst_handler_t Emulator::inst_handlers_R[] = {
	&Emulator::inst_sll,           // 000 000
	&Emulator::inst_unimplemented, // 000 001
	&Emulator::inst_srl,           // 000 010
	&Emulator::inst_sra,           // 000 011
	&Emulator::inst_sllv,          // 000 100
	&Emulator::inst_unimplemented, // 000 101
	&Emulator::inst_unimplemented, // 000 110
	&Emulator::inst_unimplemented, // 000 111
	&Emulator::inst_jr,            // 001 000
	&Emulator::inst_jalr,          // 001 001
	&Emulator::inst_movz,          // 001 010
	&Emulator::inst_movn,          // 001 011
	&Emulator::inst_syscall,       // 001 100
	&Emulator::inst_unimplemented, // 001 101
	&Emulator::inst_unimplemented, // 001 110
	&Emulator::inst_sync,          // 001 111
	&Emulator::inst_mfhi,          // 010 000
	&Emulator::inst_mthi,          // 010 001
	&Emulator::inst_mflo,          // 010 010
	&Emulator::inst_mtlo,          // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_mult,          // 011 000
	&Emulator::inst_multu,         // 011 001
	&Emulator::inst_div,           // 011 010
	&Emulator::inst_divu,          // 011 011
	&Emulator::inst_unimplemented, // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_unimplemented, // 011 111
	&Emulator::inst_add,           // 100 000
	&Emulator::inst_addu,          // 100 001
	&Emulator::inst_sub,           // 100 010
	&Emulator::inst_subu,          // 100 011
	&Emulator::inst_and,           // 100 100
	&Emulator::inst_or,            // 100 101
	&Emulator::inst_xor,           // 100 110
	&Emulator::inst_nor,           // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_slt,           // 101 010
	&Emulator::inst_sltu,          // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_unimplemented, // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_unimplemented, // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_unimplemented, // 110 011
	&Emulator::inst_teq,           // 110 100
	&Emulator::inst_unimplemented, // 110 101
	&Emulator::inst_unimplemented, // 110 110
	&Emulator::inst_unimplemented, // 110 111
	&Emulator::inst_unimplemented, // 111 000
	&Emulator::inst_unimplemented, // 111 001
	&Emulator::inst_unimplemented, // 111 010
	&Emulator::inst_unimplemented, // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_unimplemented, // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

// Type RI instruction handlers indexed by functor
const inst_handler_t Emulator::inst_handlers_RI[] = {
	&Emulator::inst_bltz,          // 00 000
	&Emulator::inst_bgez,          // 00 001
	&Emulator::inst_unimplemented, // 00 010
	&Emulator::inst_unimplemented, // 00 011
	&Emulator::inst_unimplemented, // 00 100
	&Emulator::inst_unimplemented, // 00 101
	&Emulator::inst_unimplemented, // 00 110
	&Emulator::inst_unimplemented, // 00 111
	&Emulator::inst_unimplemented, // 01 000
	&Emulator::inst_unimplemented, // 01 001
	&Emulator::inst_unimplemented, // 01 010
	&Emulator::inst_unimplemented, // 01 011
	&Emulator::inst_unimplemented, // 01 100
	&Emulator::inst_unimplemented, // 01 101
	&Emulator::inst_unimplemented, // 01 110
	&Emulator::inst_unimplemented, // 01 111
	&Emulator::inst_unimplemented, // 10 000
	&Emulator::inst_bgezal,        // 10 001
	&Emulator::inst_unimplemented, // 10 010
	&Emulator::inst_unimplemented, // 10 011
	&Emulator::inst_unimplemented, // 10 100
	&Emulator::inst_unimplemented, // 10 101
	&Emulator::inst_unimplemented, // 10 110
	&Emulator::inst_unimplemented, // 10 111
	&Emulator::inst_unimplemented, // 11 000
	&Emulator::inst_unimplemented, // 11 001
	&Emulator::inst_unimplemented, // 11 010
	&Emulator::inst_unimplemented, // 11 011
	&Emulator::inst_unimplemented, // 11 100
	&Emulator::inst_unimplemented, // 11 101
	&Emulator::inst_unimplemented, // 11 110
	&Emulator::inst_unimplemented, // 11 111
};

// Type special2 instructions indexed by functor
const inst_handler_t Emulator::inst_handlers_special2[] = {
	&Emulator::inst_unimplemented, // 000 000
	&Emulator::inst_unimplemented, // 000 001
	&Emulator::inst_mul,           // 000 010
	&Emulator::inst_unimplemented, // 000 011
	&Emulator::inst_unimplemented, // 000 100
	&Emulator::inst_unimplemented, // 000 101
	&Emulator::inst_unimplemented, // 000 110
	&Emulator::inst_unimplemented, // 000 111
	&Emulator::inst_unimplemented, // 001 000
	&Emulator::inst_unimplemented, // 001 001
	&Emulator::inst_unimplemented, // 001 010
	&Emulator::inst_unimplemented, // 001 011
	&Emulator::inst_unimplemented, // 001 100
	&Emulator::inst_unimplemented, // 001 101
	&Emulator::inst_unimplemented, // 001 110
	&Emulator::inst_unimplemented, // 001 111
	&Emulator::inst_unimplemented, // 010 000
	&Emulator::inst_unimplemented, // 010 001
	&Emulator::inst_unimplemented, // 010 010
	&Emulator::inst_unimplemented, // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_unimplemented, // 011 000
	&Emulator::inst_unimplemented, // 011 001
	&Emulator::inst_unimplemented, // 011 010
	&Emulator::inst_unimplemented, // 011 011
	&Emulator::inst_unimplemented, // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_unimplemented, // 011 111
	&Emulator::inst_unimplemented, // 100 000
	&Emulator::inst_unimplemented, // 100 001
	&Emulator::inst_unimplemented, // 100 010
	&Emulator::inst_unimplemented, // 100 011
	&Emulator::inst_unimplemented, // 100 100
	&Emulator::inst_unimplemented, // 100 101
	&Emulator::inst_unimplemented, // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_unimplemented, // 101 010
	&Emulator::inst_unimplemented, // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_unimplemented, // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_unimplemented, // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_unimplemented, // 110 011
	&Emulator::inst_unimplemented, // 110 100
	&Emulator::inst_unimplemented, // 110 101
	&Emulator::inst_unimplemented, // 110 110
	&Emulator::inst_unimplemented, // 110 111
	&Emulator::inst_unimplemented, // 111 000
	&Emulator::inst_unimplemented, // 111 001
	&Emulator::inst_unimplemented, // 111 010
	&Emulator::inst_unimplemented, // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_unimplemented, // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

const inst_handler_t Emulator::inst_handlers_special3[] = {
	&Emulator::inst_ext,           // 000 000
	&Emulator::inst_unimplemented, // 000 001
	&Emulator::inst_unimplemented, // 000 010
	&Emulator::inst_unimplemented, // 000 011
	&Emulator::inst_unimplemented, // 000 100
	&Emulator::inst_unimplemented, // 000 101
	&Emulator::inst_unimplemented, // 000 110
	&Emulator::inst_unimplemented, // 000 111
	&Emulator::inst_unimplemented, // 001 000
	&Emulator::inst_unimplemented, // 001 001
	&Emulator::inst_unimplemented, // 001 010
	&Emulator::inst_unimplemented, // 001 011
	&Emulator::inst_unimplemented, // 001 100
	&Emulator::inst_unimplemented, // 001 101
	&Emulator::inst_unimplemented, // 001 110
	&Emulator::inst_unimplemented, // 001 111
	&Emulator::inst_unimplemented, // 010 000
	&Emulator::inst_unimplemented, // 010 001
	&Emulator::inst_unimplemented, // 010 010
	&Emulator::inst_unimplemented, // 010 011
	&Emulator::inst_unimplemented, // 010 100
	&Emulator::inst_unimplemented, // 010 101
	&Emulator::inst_unimplemented, // 010 110
	&Emulator::inst_unimplemented, // 010 111
	&Emulator::inst_unimplemented, // 011 000
	&Emulator::inst_unimplemented, // 011 001
	&Emulator::inst_unimplemented, // 011 010
	&Emulator::inst_unimplemented, // 011 011
	&Emulator::inst_unimplemented, // 011 100
	&Emulator::inst_unimplemented, // 011 101
	&Emulator::inst_unimplemented, // 011 110
	&Emulator::inst_unimplemented, // 011 111
	&Emulator::inst_bshfl,         // 100 000
	&Emulator::inst_unimplemented, // 100 001
	&Emulator::inst_unimplemented, // 100 010
	&Emulator::inst_unimplemented, // 100 011
	&Emulator::inst_unimplemented, // 100 100
	&Emulator::inst_unimplemented, // 100 101
	&Emulator::inst_unimplemented, // 100 110
	&Emulator::inst_unimplemented, // 100 111
	&Emulator::inst_unimplemented, // 101 000
	&Emulator::inst_unimplemented, // 101 001
	&Emulator::inst_unimplemented, // 101 010
	&Emulator::inst_unimplemented, // 101 011
	&Emulator::inst_unimplemented, // 101 100
	&Emulator::inst_unimplemented, // 101 101
	&Emulator::inst_unimplemented, // 101 110
	&Emulator::inst_unimplemented, // 101 111
	&Emulator::inst_unimplemented, // 110 000
	&Emulator::inst_unimplemented, // 110 001
	&Emulator::inst_unimplemented, // 110 010
	&Emulator::inst_unimplemented, // 110 011
	&Emulator::inst_unimplemented, // 110 100
	&Emulator::inst_unimplemented, // 110 101
	&Emulator::inst_unimplemented, // 110 110
	&Emulator::inst_unimplemented, // 110 111
	&Emulator::inst_unimplemented, // 111 000
	&Emulator::inst_unimplemented, // 111 001
	&Emulator::inst_unimplemented, // 111 010
	&Emulator::inst_rdhwr,         // 111 011
	&Emulator::inst_unimplemented, // 111 100
	&Emulator::inst_unimplemented, // 111 101
	&Emulator::inst_unimplemented, // 111 110
	&Emulator::inst_unimplemented, // 111 111
};

void Emulator::run_inst(){
	// Handle breakpoint. It may change pc
	if (breakpoints.count(pc))
		(this->*breakpoints.at(pc))();

	uint32_t inst   = mmu.read<uint32_t>(pc);
	uint8_t  opcode = (inst >> 26) & 0b111111;
	//printf("[0x%X] Opcode: 0x%X, inst: 0x%X\n", pc, opcode, inst);

	// If needed, take the branch after fetching the current instruction
	// Otherwise, just increment PC so it points to the next instruction
	if (condition){
		pc = jump_addr;
		condition = false;
		jump_addr = 0;
	} else 
		pc += 4;

	// Handle current instruction if it isn't a NOP
	if (inst)
		(this->*inst_handlers[opcode])(inst);
}

void Emulator::inst_test(uint32_t inst){
	cout << "Test instruction" << endl;
}

void Emulator::inst_unimplemented(uint32_t inst){
	die("Unimplemented instruction: 0x%X\n", inst);
}

void Emulator::inst_R(uint32_t inst){
	// Get the functor and call the appropiate handler
	(this->*inst_handlers_R[inst & 0b00111111])(inst);
}

void Emulator::inst_RI(uint32_t inst){
	// Get the functor and call the appropiate handler
	(this->*inst_handlers_RI[(inst >> 16) & 0b00011111])(inst);
}

void Emulator::inst_special2(uint32_t inst){
	(this->*inst_handlers_special2[inst & 0b00111111])(inst);
}

void Emulator::inst_special3(uint32_t inst){
	(this->*inst_handlers_special3[inst & 0b00111111])(inst);
}

void Emulator::inst_or(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) | get_reg(inst.t));
}

void Emulator::inst_bgezal(uint32_t val){
	inst_RI_t inst(val);
	regs[Reg::ra]  = pc + 4; // Instruction after the delay slot
	condition = (inst.s == 0 || regs[inst.s] >= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_nop(uint32_t val){ }

void Emulator::inst_lui(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, inst.C << 16);
}

void Emulator::inst_addiu(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) + (int16_t)inst.C);
}

void Emulator::inst_lw(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint32_t>(addr));
}

void Emulator::inst_and(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) & get_reg(inst.t));
}

void Emulator::inst_sw(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint32_t>(addr, get_reg(inst.t));
}

void Emulator::inst_jalr(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, pc+4); // Instruction after the delay slot
	condition = true;
	jump_addr = get_reg(inst.s);
}

void Emulator::inst_beq(uint32_t val){
	inst_I_t inst(val);
	condition = (get_reg(inst.s) == get_reg(inst.t));
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_addu(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.t) + get_reg(inst.s));
}

void Emulator::inst_sll(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.t) << inst.S);
}

void Emulator::inst_bne(uint32_t val){
	inst_I_t inst(val);
	condition = (get_reg(inst.s) != get_reg(inst.t));
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_jr(uint32_t val){
	inst_R_t inst(val);
	condition = true;
	jump_addr = get_reg(inst.s);
}

void Emulator::inst_lhu(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint16_t>(addr));
}

void Emulator::inst_syscall(uint32_t val){
	handle_syscall(regs[Reg::v0]);
}

void Emulator::inst_xor(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) ^ get_reg(inst.t));
}

void Emulator::inst_sltu(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) < get_reg(inst.t));
}

void Emulator::inst_sltiu(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) < (uint32_t)(int16_t)inst.C);
}

void Emulator::inst_movn(uint32_t val){
	inst_R_t inst(val);
	if (get_reg(inst.t) != 0)
		set_reg(inst.d, get_reg(inst.s));
}

void Emulator::inst_movz(uint32_t val){
	inst_R_t inst(val);
	if (get_reg(inst.t) == 0)
		set_reg(inst.d, get_reg(inst.s));
}

void Emulator::inst_subu(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.s) - get_reg(inst.t));
}

void Emulator::inst_teq(uint32_t val){
	inst_R_t inst(val);
	if (get_reg(inst.s) == get_reg(inst.t))
		die("TRAP\n");
}

void Emulator::inst_div(uint32_t val){
	inst_R_t inst(val);
	int32_t dividend  = get_reg(inst.s);
	int32_t divisor   = get_reg(inst.t);
	hi = dividend % divisor; // module
	lo = dividend / divisor; // quotient
}


void Emulator::inst_divu(uint32_t val){
	inst_R_t inst(val);
	uint32_t dividend  = get_reg(inst.s);
	uint32_t divisor   = get_reg(inst.t);
	hi = dividend % divisor; // module
	lo = dividend / divisor; // quotient
}

void Emulator::inst_mflo(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, lo);
}

void Emulator::inst_mul(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, (int32_t)get_reg(inst.s) * (int32_t)get_reg(inst.t));
}

void Emulator::inst_bltz(uint32_t val){
	inst_RI_t inst(val);
	condition = ((int32_t)get_reg(inst.s) < 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_blez(uint32_t val){
	inst_I_t inst(val);
	condition = ((int32_t)get_reg(inst.s) <= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_rdhwr(uint32_t val){
	inst_R_t inst(val);
	uint32_t hwr = 0;
	switch (inst.d){
		case 29: // 0b11101, User Local Register
			if (!tls)
				die("not set tls\n");
			hwr = tls;
			//printf("get tls --> 0x%X\n", tls);
			break;

		default:
			die("Unimplemented rdhwr: %d\n", inst.d);
	}
	
	set_reg(inst.t, hwr);
}

void Emulator::inst_bgez(uint32_t val){
	inst_RI_t inst(val);
	condition = ((int32_t)get_reg(inst.s) >= 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_slti(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, (int32_t)get_reg(inst.s) < (int32_t)(int16_t)inst.C);
}

void Emulator::inst_andi(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) & inst.C);
}

void Emulator::inst_ori(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) | inst.C);
}

void Emulator::inst_xori(uint32_t val){
	inst_I_t inst(val);
	set_reg(inst.t, get_reg(inst.s) ^ inst.C);
}

void Emulator::inst_pref(uint32_t val){ }

void Emulator::inst_jal(uint32_t val){
	inst_J_t inst(val);
	regs[Reg::ra] = pc + 4; // Instruction after the delay slot
	condition = true;       // 28 bits from A, 4 bits from pc
	jump_addr = (inst.A << 2) | (pc & 0xF0000000);
}

void Emulator::inst_lb(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, (int32_t)mmu.read<int8_t>(addr));
}

void Emulator::inst_nor(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, ~(get_reg(inst.d) | get_reg(inst.t)));
}

void Emulator::inst_bshfl(uint32_t val){
	switch ((val >> 6) & 0b11111){
		case 0b10000: // seb
			inst_seb(val);
			break;

		case 0b11000: // seh
			inst_seh(val);
			break;

		default:
			die("Unimplemented bshfl instruction: 0x%X\n", val);
	}
}

void Emulator::inst_seh(uint32_t val){
	inst_R_t inst(val);
	uint16_t value = get_reg(inst.t);
	set_reg(inst.d, (int32_t)(int16_t)value);
}

void Emulator::inst_seb(uint32_t val){
	inst_R_t inst(val);
	uint16_t value = get_reg(inst.t);
	set_reg(inst.d, (int32_t)(int8_t)value);
}

void Emulator::inst_srl(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, get_reg(inst.t) >> inst.S);
}

void Emulator::inst_lh(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, (int32_t)mmu.read<int16_t>(addr));
}

void Emulator::inst_lbu(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint8_t>(addr));
}


void Emulator::inst_lwl(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	uint32_t w   = mmu.read<uint32_t>(addr & ~3);
	memcpy((char*)(&reg)+3-offset, &w, offset+1);
	set_reg(inst.t, reg);
}

void Emulator::inst_lwr(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	uint32_t w   = mmu.read<uint32_t>(addr & ~3);
	memcpy(&reg, (char*)(&w)+offset, 4-offset);
	set_reg(inst.t, reg);
}

void Emulator::inst_sb(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint8_t>(addr, get_reg(inst.t));
}

void Emulator::inst_sh(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint16_t>(addr, get_reg(inst.t));
}

void Emulator::inst_swl(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	uint32_t w   = mmu.read<uint32_t>(addr & ~3);
	memcpy(&w, (char*)(&reg)+3-offset, offset+1);
	mmu.write<uint32_t>(addr & ~3, w);
	die("swl\n");
}

void Emulator::inst_swr(uint32_t val){
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	vaddr_t offset = addr & 3; // offset into aligned word

	// Locurote. Look at the manual
	uint32_t reg = get_reg(inst.t);
	uint32_t w   = mmu.read<uint32_t>(addr & ~3);
	memcpy((char*)(&w)+offset, &reg, 4-offset);
	mmu.write<uint32_t>(addr & ~3, w);
	die("swr\n");
}

void Emulator::inst_sllv(uint32_t val){
	inst_R_t inst(val);
	uint8_t shift = get_reg(inst.s) & 0b00011111;
	set_reg(inst.d, get_reg(inst.t) << shift);
}

void Emulator::inst_slt(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, (int32_t)get_reg(inst.s) < (int32_t)get_reg(inst.t));
}

void Emulator::inst_sub(uint32_t val){
	inst_R_t inst(val);
	int32_t result;
	bool overflow = __builtin_sub_overflow(
		(int32_t)get_reg(inst.s),
		(int32_t)get_reg(inst.t),
		&result
	);
	if (overflow)
		die("sub overflow\n");
	set_reg(inst.d, result);
	die("sub\n");
}

void Emulator::inst_add(uint32_t val){
	inst_R_t inst(val);
	int32_t result;
	bool overflow = __builtin_add_overflow(
		(int32_t)get_reg(inst.s),
		(int32_t)get_reg(inst.t),
		&result
	);
	if (overflow)
		die("Add overflow\n");
	set_reg(inst.d, result);
	die("add\n");
}

void Emulator::inst_j(uint32_t val){
	inst_J_t inst(val);
	condition = true;       // 28 bits from A, 4 bits from pc
	jump_addr = (inst.A << 2) | (pc & 0xF0000000);
}

void Emulator::inst_ll(uint32_t val){
	// Forget about atomics for now
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	set_reg(inst.t, mmu.read<uint32_t>(addr));
}

void Emulator::inst_sc(uint32_t val){
	// Forget about atomics for now
	inst_I_t inst(val);
	vaddr_t addr = get_reg(inst.s) + (int16_t)inst.C;
	mmu.write<uint32_t>(addr, get_reg(inst.t));
	set_reg(inst.t, 1);
}

void Emulator::inst_sync(uint32_t val){
	// Forget about atomics for now
}

void Emulator::inst_bgtz(uint32_t val){
	inst_I_t inst(val);
	condition = ((int32_t)get_reg(inst.s) > 0);
	jump_addr = pc + ((int16_t)inst.C << 2);
}

void Emulator::inst_mult(uint32_t val){
	inst_R_t inst(val);
	uint64_t result = (int32_t)get_reg(inst.s) * (int32_t)get_reg(inst.t);
	lo = result & 0xFFFFFFFF;
	hi = (result >> 32) & 0xFFFFFFFF;
}

void Emulator::inst_multu(uint32_t val){
	inst_R_t inst(val);
	uint64_t result = (uint64_t)get_reg(inst.s) * get_reg(inst.t);
	lo = result & 0xFFFFFFFF;
	hi = (result >> 32) & 0xFFFFFFFF;
}

void Emulator::inst_mfhi(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, hi);
}

void Emulator::inst_mthi(uint32_t val){
	inst_R_t inst(val);
	hi = get_reg(inst.s);
}

void Emulator::inst_mtlo(uint32_t val){
	inst_R_t inst(val);
	lo = get_reg(inst.s);
}

void Emulator::inst_ext(uint32_t val){
	inst_R_t inst(val);
	uint32_t lsb  = inst.S;
	uint32_t size = inst.d + 1;
	uint32_t mask = (1 << size) - 1;
	set_reg(inst.t, (get_reg(inst.s) >> lsb) & mask);
}

void Emulator::inst_sra(uint32_t val){
	inst_R_t inst(val);
	set_reg(inst.d, (int32_t)get_reg(inst.t) >> inst.S);
}

void Emulator::inst_sdc1(uint32_t val){
	// Forget about floats for now
}

const char* regs_map[] = {
	"00",  "at", "v0", "v1",
	"a0", "a1", "a2", "a3",
	"t0", "t1", "t2", "t3", 
	"t4", "t5", "t6", "t7", 
	"s0", "s1", "s2", "s3", 
	"s4", "s5", "s6", "s7", 
	"t8", "t9", "k0", "k1", 
	"gp", "sp", "fp", "ra"
};
ostream& operator<<(ostream& os, const Emulator& emu){
	os << hex << setfill('0');
	os << "PC:  " << setw(8) << emu.pc << endl;
	for (int i = 0; i < 32; i++){
		os << "$" << regs_map[i] << ": " << setw(8) << emu.regs[i] << "\t";
		if ((i+1)%8 == 0)
			os << endl;
	}
	os << "condition: " << emu.condition << "\t"
	   << "jump addr: " << emu.jump_addr << endl;
	os << dec;
	return os;
}