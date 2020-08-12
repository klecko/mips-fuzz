#include <cstring>
#include <iomanip>
#include <assert.h>
#include <unistd.h> // *_FILENO definitions
#include <linux/limits.h> // PATH_MAX
#include <string>
#include "common.h"
#include "emulator.h"
#include "elf_parser.hpp"

using namespace std;

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
	breakpoints_bitmap.resize(mem_size);

	// Breakpoints can only be set in text, which is between load address and 
	// initial brk (always?). Divide by 4 because each instruction is 4 bytes
	breakpoints.resize((mmu.get_brk()-load_addr)/4);
	set_bp(0x00423f00, &Emulator::malloc_bp);
	set_bp(0x00424634, &Emulator::free_bp);
	set_bp(0x004248d8, &Emulator::realloc_bp);
	set_bp(0x00424d20, &Emulator::memalign_bp);
	set_bp(0x00424d3c, &Emulator::valloc_bp);
	set_bp(0x00424db8, &Emulator::pvalloc_bp);
	set_bp(0x00424e64, &Emulator::calloc_bp);
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
	elfpath    = other.elfpath;
}


void Emulator::load_elf(const string& filepath, const vector<string>& argv){
	// Stack layout described in
	// http://articles.manugarg.com/aboutelfauxiliaryvectors.html
	// I add random numbers at the bottom of the stack for auxv
	cout << "Loading " << filepath << endl;
	Elf_parser elf(filepath);
	mmu.load_elf(elf.get_segments(load_addr));

	// Save absolute file path. Ugly conversions here
	char abspath[PATH_MAX];
	if (!realpath(filepath.c_str(), abspath))
		die("error realpath: %s\n", strerror(errno));
	elfpath.assign(abspath);

	// Set entry
	pc = elf.get_entry();
	dbgprintf("Entry 0x%X\n", pc);

	// Allocate the stack
	regs[Reg::sp] = mmu.alloc_stack(256 * 1024);
	dbgprintf("Allocated stack at 0x%X\n", regs[Reg::sp]);

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

	// Set up auxp. They aren't necessary, I did them tried to solve something
	// unrelated.
	phinfo_t phinfo = elf.get_phinfo();
	Elf32_auxv_t auxv[] = {
		{AT_RANDOM, random_bytes},               // Address of 16 random bytes
		{AT_EXECFN, argv_vm[0]},                 // Filename of the program
	 	{AT_PHDR,   load_addr + phinfo.e_phoff}, // Pointer to program headers
		{AT_PHENT,  phinfo.e_phentsize},         // Size of each entry
		{AT_PHNUM,  phinfo.e_phnum},             // Number of entries
		{AT_NULL,   0},                          // Auxv end
	};
	// We don't use push_stack or mmu.write because of misalignment checks.
	regs[Reg::sp] -= sizeof(auxv);
	mmu.write_mem(regs[Reg::sp], auxv, sizeof(auxv));

	// Set up envp
	push_stack(0);

	// Set up argv and argc
	for (auto it = argv_vm.rbegin(); it != argv_vm.rend(); ++it)
		push_stack<vaddr_t>(*it);
	push_stack<uint32_t>(argv.size());
}

void Emulator::set_bp(vaddr_t addr, breakpoint_t bp){
	size_t i = (addr - load_addr)/4;
	assert(i < breakpoints.size());
	breakpoints[i] = bp;
	breakpoints_bitmap[addr] = true;
}

breakpoint_t Emulator::get_bp(vaddr_t addr){
	size_t i = (addr - load_addr)/4;
	assert(i < breakpoints.size());
	return breakpoints[i];
}

void Emulator::run_inst(Stats& local_stats){
	// Handle breakpoint. It may change pc
	cycle_t cycles = rdtsc();
	if (breakpoints_bitmap[pc])
		(this->*get_bp(pc))();
	local_stats.bp_cycles += _rdtsc() - cycles;

	cycles = rdtsc();
	uint32_t inst   = mmu.read_inst(pc);
	uint8_t  opcode = (inst >> 26) & 0b111111;
	local_stats.fetch_inst_cycles += rdtsc() - cycles;
	//dbgprintf("[0x%X] Opcode: 0x%X, inst: 0x%X\n", pc, opcode, inst);

	// If needed, take the branch after fetching the current instruction
	// Otherwise, just increment PC so it points to the next instruction
	cycles = rdtsc();
	if (condition){
		pc = jump_addr;
		condition = false;
	} else 
		pc += 4;
	local_stats.jump_cycles += rdtsc() - cycles;

	// Handle current instruction if it isn't a NOP
	cycles = rdtsc();
	if (inst)
		(this->*inst_handlers[opcode])(inst);
	local_stats.inst_handl_cycles += rdtsc() - cycles;
}

void Emulator::run(const string& input, Stats& local_stats){
	// Save provided input. Internal representation is as const char* and not
	// as string so we don't have to perform any copy.
	this->input    = input.c_str();
	this->input_sz = input.size();

	// Perform execution recording number of executed instructions
	uint64_t instr_exec = 0;
	running = true;
	cycle_t cycles;
	while (running){
		cycles = rdtsc();
		run_inst(local_stats);
		local_stats.run_inst_cycles += rdtsc() - cycles;
		local_stats.instr += 1;

		cycles = rdtsc();
		instr_exec += 1;
		if (instr_exec >= INSTR_TIMEOUT)
			throw RunTimeout();
		local_stats.timeout_cycles += rdtsc() - cycles;
		//cout << *this << endl;
	}
}

uint64_t Emulator::run_until(vaddr_t pc){
	Stats dummy;
	while (this->pc != pc){
		run_inst(dummy);
		dummy.instr++;
	}
	return dummy.instr;
}

void Emulator::test_bp(){
	die("test bp\n");
}

void Emulator::malloc_bp(){
	vsize_t size = regs[Reg::a0];
	vaddr_t addr = (size > 0 ? mmu.alloc(size) : 0);
	regs[Reg::v0] = addr;
	pc = regs[Reg::ra];
	dbgprintf("malloc(%u) --> 0x%X\n", size, addr);
}

void Emulator::free_bp(){
	//die("free_bp\n");
	vsize_t addr = regs[Reg::a0];
	pc = regs[Reg::ra];
	dbgprintf("free(0x%X)\n", addr);
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
	dbgprintf("brk(0x%X) --> 0x%X\n", new_brk, brk);
	return brk;
}

uint32_t Emulator::sys_openat(int32_t dirfd, vaddr_t pathname_addr, int32_t flags,
                              uint32_t& error)
{
	/* cout << *this << endl;
	die(""); */
	string pathname = mmu.read_string(pathname_addr);

	// Find unused fd
	uint32_t fd = 3;
	while (open_files.count(fd))
		fd++;

	// Create input file
	if (pathname == "input_file"){
		if (flags == O_RDWR || flags == O_WRONLY)
			die("opening input file with write permissions");
		
		// God, forgive me for this casting.
		File input_file(flags, (char*)input, input_sz);
		open_files[fd] = move(input_file);
	} else
		die("Unimplemented openat\n");

	dbgprintf("openat(%d, %s, %d) --> %d\n", dirfd, pathname.c_str(), flags, fd);
	error = 0;
	return fd;
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
		guestprintf("output: %s\n", buf);
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
	dbgprintf("uname(0x%X) --> 0\n", addr);
	error = 0;
	return 0;
}

uint32_t Emulator::sys_readlink(vaddr_t path_addr, vaddr_t buf_addr,
		                        vsize_t bufsize, uint32_t& error)
{
	uint32_t written = 0;
	string path = mmu.read_string(path_addr);
	if (path == "/proc/self/exe"){
		if (bufsize < elfpath.size())
			die("error readlink\n");
		mmu.write_mem(buf_addr, elfpath.c_str(), elfpath.size());
		written = elfpath.size();
	} else
		die("Unimplemented readlink\n");
	
	dbgprintf("readlink(%s, 0x%X, %d) --> %d (%s)\n", path.c_str(), buf_addr, 
	          bufsize, written, elfpath.c_str());
	error = 0;
	return written;
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
	vsize_t real_count = f.move_cursor(count);
	mmu.write_mem(buf_addr, cursor, real_count);

	dbgprintf("read(%d, 0x%X, %d) --> %d\n", fd, buf_addr, count, real_count);
	error = 0;
	return real_count;
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
			guestprintf("[OUTPUT] %s\n", buf);
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
	vsize_t real_count = f.move_cursor(count);
	mmu.read_mem(cursor, buf_addr, real_count);

	dbgprintf("write(%d, 0x%X, %d) --> %d\n", fd, buf_addr, count, real_count);
	error = 0;
	return real_count;
}

uint32_t Emulator::sys_fstat64(uint32_t fd, vaddr_t statbuf_addr,
                               uint32_t& error)
{
	dbgprintf("fstat64(%d, 0x%X) --> -1\n", fd, statbuf_addr);
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
	dbgprintf("close(%d)\n", fd);
	error = 0;
	return 0;
}

void Emulator::handle_syscall(uint32_t syscall){
	switch (syscall){
		case 4001: // exit
		case 4246: // exit_group
			running = false;
			dbgprintf("EXIT %d\n", regs[Reg::a0]);
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
			tls = regs[Reg::a0];
			dbgprintf("set tls --> 0x%X\n", tls);
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
	for (const auto& f : emu.open_files)
		cout << "file fd " << f.first << ": " << f.second << endl;
	os << dec;
	return os;
}