#include <iostream>
#include <cstring>
#include <iomanip>
#include "mmu.h"
#include "common.h"

using namespace std;

Mmu::Mmu(vsize_t mem_size){
	memory     = new uint8_t[mem_size];
	memory_len = mem_size;
	perms      = new uint8_t[mem_size];
	next_alloc = 0;
	stack      = 0;
	brk        = 0;
	min_brk    = 0;
	max_brk    = 0;
	dirty_vec  = new vaddr_t[memory_len/DIRTY_BLOCK_SIZE + 1];
	dirty_size = 0;
	dirty_map.resize(memory_len/DIRTY_BLOCK_SIZE + 1, false);
	memset(memory, 0, mem_size);
	memset(perms, 0, mem_size);
}

Mmu::Mmu(const Mmu& other){
	memory_len = other.memory_len;
	memory     = new uint8_t[memory_len];
	perms      = new uint8_t[memory_len];
	next_alloc = other.next_alloc;
	stack      = other.stack;
	brk        = other.brk;
	min_brk    = other.min_brk;
	max_brk    = other.max_brk;
	dirty_vec  = new vaddr_t[memory_len/DIRTY_BLOCK_SIZE + 1];
	dirty_size = other.dirty_size;
	dirty_map  = vector<uint8_t>(other.dirty_map);
	cur_allocs = other.cur_allocs;
	memcpy(memory, other.memory, memory_len);
	memcpy(perms, other.perms, memory_len);
	memcpy(dirty_vec, other.dirty_vec, dirty_size);
}

Mmu::Mmu(Mmu&& other) noexcept : Mmu(){
	swap(*this, other);
}

Mmu::~Mmu(){
	delete[] memory;
	delete[] perms;
	delete[] dirty_vec;
}

Mmu& Mmu::operator=(Mmu other){
	swap(*this, other);
	return *this;
}

void swap(Mmu& first, Mmu& second){
	using std::swap;
	swap(first.memory, second.memory);
	swap(first.memory_len, second.memory_len);
	swap(first.perms, second.perms);
	swap(first.next_alloc, second.next_alloc);
	swap(first.stack, second.stack);
	swap(first.brk, second.brk);
	swap(first.min_brk, second.min_brk);
	swap(first.max_brk, second.max_brk);
	swap(first.dirty_vec, second.dirty_vec);
	swap(first.dirty_size, second.dirty_size);
	swap(first.dirty_map, second.dirty_map);
	swap(first.cur_allocs, second.cur_allocs);
}

uint8_t* Mmu::get_memory(){
	return memory;
}

uint8_t* Mmu::get_perms(){
	return perms;
}

vaddr_t* Mmu::get_dirty_vec(){
	return dirty_vec;
}

vsize_t* Mmu::get_pdirty_size(){
	return &dirty_size;
}

uint8_t* Mmu::get_dirty_map(){
	return dirty_map.data();
}

vsize_t Mmu::size() const {
	return memory_len;
}

vaddr_t Mmu::get_brk() const {
	return brk;
}

bool Mmu::set_brk(vaddr_t new_brk){
	// Checks
	if (new_brk < min_brk)
		die("attempt to set brk before min_brk to 0x%X, current is 0x%X and "
		    "min is 0x%X\n", new_brk, brk, min_brk);
	if (new_brk > max_brk)
		die("attempt to set brk past max_brk to 0x%X, current is 0x%X and max "
		    "is 0x%X\n", new_brk, brk, max_brk);

	if (new_brk >= brk){
		// Allocating memory. Zero out the new memory and set perms
		memset(memory + brk, 0, new_brk - brk);
		set_perms(brk, new_brk - brk, PERM_READ|PERM_WRITE|PERM_INIT);
	} else {
		// Reducing brk. Set perms
		set_perms(new_brk, brk - new_brk, 0);
	}

	// Set new brk and return success
	brk = new_brk;
	return true;
}

void Mmu::check_bounds(vaddr_t addr, vsize_t len, uint8_t perm) const {
	if (addr + len > memory_len) // Allow overflow here (negative len)
		switch (perm){
			case PERM_READ:
				throw Fault(Fault::Type::OutOfBoundsRead, addr);
			case PERM_WRITE:
				throw Fault(Fault::Type::OutOfBoundsWrite, addr);
			case PERM_EXEC:
				throw Fault(Fault::Type::OutOfBoundsExec, addr);
			default:
				die("wrong perm in check_bounds: %d\n", perm);
		}
}

void Mmu::check_alignment(vaddr_t addr, vsize_t align, uint8_t perm) const {
	if (addr % align != 0)
		switch (perm){
			case PERM_READ:
				throw Fault(Fault::Type::MisalignedRead, addr);
			case PERM_WRITE:
				throw Fault(Fault::Type::MisalignedWrite, addr);
			case PERM_EXEC:
				throw Fault(Fault::Type::MisalignedExec, addr);
			default:
				die("wrong perm in check_alignment: %d\n", perm);
		}
}

void Mmu::set_dirty(vaddr_t addr, vsize_t len){
	// Set dirty those blocks that arent dirty
	vsize_t block_begin = addr/DIRTY_BLOCK_SIZE;
	vsize_t block_end   = (addr+len+DIRTY_BLOCK_SIZE-1)/DIRTY_BLOCK_SIZE;
	for (vsize_t block = block_begin; block < block_end; block++){
		if (!dirty_map[block]){
			dirty_vec[dirty_size++] = block;
			dirty_map[block] = true;
		}
	}
}

void Mmu::set_init(vaddr_t addr, vsize_t len){
	vaddr_t addr_end = addr + len;
	for (vaddr_t v = addr; v < addr_end; v++)
		perms[v] |= PERM_INIT;
}

void Mmu::set_perms(vaddr_t addr, vsize_t len, uint8_t perm){
	// Set permissions
	memset(perms+addr, perm, len);

 	// Set whole region as dirty
	set_dirty(addr, len);
}

/*
Asking for PERM_READ:
	OutOfBounds if it is not in bounds of the guest memory space
	Read fault if it hasnt PERM_READ (unallocated memory)
	Uninit fault if it has PERM_READ but hasnt PERM_INIT (uninitialized memory)
	Correct if it has both PERM_READ and PERM_INIT
*/
void Mmu::check_perms(vaddr_t addr, vsize_t len, uint8_t perm) const {
	// Check permission for each address
	vaddr_t addr_end = addr + len;
	for (; addr < addr_end; addr++){
		if ((perms[addr] & perm) != perm){ 
			// Permission error. Determine which
			if (perm == PERM_WRITE)
				throw Fault(Fault::Type::Write, addr);
			else if (perm == PERM_EXEC)
				throw Fault(Fault::Type::Exec, addr);
			else if (perms[addr] & PERM_READ)
				{}
				//throw Fault(Fault::Type::Uninit, addr);
			else if (!(perms[addr] & PERM_READ))
				throw Fault(Fault::Type::Read, addr);
			else
				die("what\n");
		}
	}
}

void Mmu::read_mem(void* dst, vaddr_t src, vsize_t len, bool chk_uninit) const {
	uint8_t perm = PERM_READ;

	// Check out of bounds
	check_bounds(src, len, perm);
	
	// Check perms. Checking for reading implies checking for initialized memory
	// by default, but it can be disabled
	if (chk_uninit)
		perm |= PERM_INIT;
	check_perms(src, len, perm);

	// Copy memory
	memcpy(dst, memory+src, len);
}


void Mmu::write_mem(vaddr_t dst, const void* src, vsize_t len){
	// Check out of bounds
	check_bounds(dst, len, PERM_WRITE);

	// Check perms
	check_perms(dst, len, PERM_WRITE);

	// Memory has been initialized, update perms
	set_init(dst, len);

	// Update dirty blocks
	set_dirty(dst, len);

	// Copy memory
	memcpy(memory+dst, src, len);
}

void Mmu::copy_mem(vaddr_t dst, vaddr_t src, vsize_t len){
	// Check src bounds and perms for reading
	check_bounds(src, len, PERM_READ);
	check_perms (src, len, PERM_READ);

	// Check dst bounds and perms for writing
	check_bounds(dst, len, PERM_WRITE);
	check_perms (dst, len, PERM_WRITE);

	// Memory has been initialized, update perms
	set_init(dst, len);

	// Update dirty blocks
	set_dirty(dst, len);

	// Copy memory
	memcpy(memory+dst, memory+src, len);
}

void Mmu::set_mem(vaddr_t dst, uint8_t c, vsize_t len){
	// Check out of bounds
	check_bounds(dst, len, PERM_WRITE);

	// Check perms
	check_perms(dst, len, PERM_WRITE);

	// Memory has been initialized, update perms
	set_init(dst, len);

	// Update dirty blocks
	set_dirty(dst, len);

	// Set memory
	memset(memory+dst, c, len);
}

uint32_t Mmu::read_inst(vaddr_t addr) const {
	// Check out of bounds
	check_bounds(addr, 4, PERM_EXEC);

	// Check perms
	check_perms(addr, 4, PERM_EXEC);

	// Check alignment
	check_alignment(addr, 4, PERM_EXEC);

	// Read and return instruction
	return *(uint32_t*)(memory+addr);
}

string Mmu::read_string(vaddr_t addr) const {
	string result = "";
	char c = read<char>(addr++);
	while (c){
		result += c;
		c = read<char>(addr++);
	}
	return result;
}

vsize_t Mmu::get_alloc_size(vaddr_t addr){
	return cur_allocs[addr].size;
}

vaddr_t Mmu::alloc(vsize_t size){
	// Fast path for empty allocations
	if (size == 0)
		return 0;

	// Check out of memory and overflow
	if (next_alloc + size > stack){
		dbgprintf("Out of memory allocating 0x%X bytes\n", size);
		throw RunOOM();
	}
	if (next_alloc + size < next_alloc){
		dbgprintf("Overflow allocating 0x%X bytes\n", size);
		throw RunOOM();
	}

	vsize_t aligned_size  = (size + 0xF) & ~0xF;
	vaddr_t current_alloc = next_alloc;

	// Memory is by default readable and writable, but not initialized
	set_perms(current_alloc, size, PERM_READ|PERM_WRITE);

	// Functions like strcmp expect to read OOB...
	set_perms(current_alloc + size, aligned_size - size, PERM_READ);

	// Update next allocation
	next_alloc += aligned_size;

	// Set this addr as allocated
	cur_allocs[current_alloc] = {AllocState::Allocated, size};

	//dbgprintf("alloc(0x%X) --> 0x%X\n", size, current_alloc);
	return current_alloc;
}

void Mmu::free(vaddr_t addr){
	// Fast path for null address
	if (addr == 0)
		return;

	Allocation& allocation = cur_allocs[addr];
	switch (allocation.state){
		case AllocState::Allocated:
			break;
		case AllocState::Freed:
			throw Fault(Fault::Type::DoubleFree, addr);
		case AllocState::Unused:
			throw Fault(Fault::Type::NotAllocatedFree, addr);
		default:
			die("Unknown alloc state for 0x%X: %d\n", addr, allocation.state);
	}

	// Update allocation
	allocation.state     = AllocState::Freed;
	allocation.size      = 0;

	// Update perms
	vsize_t aligned_size = (allocation.size + 0xF) & ~0xF;
	set_perms(addr, aligned_size, NO_PERM);
}

vaddr_t Mmu::alloc_stack(vsize_t size){
	// Check if there is a stack already
	if (stack)
		die("Attempt to allocate a second stack\n");

	// Check out of memory and alignment
	if (memory_len - size < next_alloc)
		die("Out of memory allocating a stack of 0x%X bytes\n", size);
	if (size % 0x1000 != 0)
		die("Attempt to allocate an unaligned stack of size 0x%X\n", size);

	stack = memory_len - size;
	set_perms(stack, size, PERM_READ|PERM_WRITE);

	return memory_len;
}

Mmu Mmu::fork() const {
	// Create copy and reset its dirty blocks
	Mmu new_mmu(*this);
	new_mmu.dirty_size = 0;
	new_mmu.dirty_map.assign(new_mmu.dirty_map.size(), false);
	return new_mmu;
}

void Mmu::reset(const Mmu& other){
	// Reset memory and perms for every dirty block
	for (vsize_t i = 0; i < dirty_size; i++){
		vaddr_t block = dirty_vec[i];
		vaddr_t addr = block*DIRTY_BLOCK_SIZE;
		memcpy(memory + addr, other.memory + addr, DIRTY_BLOCK_SIZE);
		memcpy(perms  + addr, other.perms  + addr, DIRTY_BLOCK_SIZE);
		dirty_map[block] = false;
	}

	dirty_size = 0;
	next_alloc = other.next_alloc;
	stack      = other.stack;
	brk        = other.brk;
	min_brk    = other.min_brk;
	max_brk    = other.max_brk;
	cur_allocs = other.cur_allocs;
}

uint8_t parse_perm(const string& flag){
	uint8_t perm = Mmu::PERM_INIT;
	if (flag.find("R") != string::npos)
		perm |= Mmu::PERM_READ;
	if (flag.find("W") != string::npos)
		perm |= Mmu::PERM_WRITE;
	if (flag.find("E") != string::npos)
		perm |= Mmu::PERM_EXEC;
	return perm;
}

void Mmu::load_elf(const vector<segment_t>& segments){
	for (const segment_t& segm : segments){
		if (segm.type == "LOAD"){
			dbgprintf("Loading at 0x%X\n", segm.virtaddr);

			if (segm.virtaddr + segm.memsize > memory_len)
				die("Not enough space for loading elf (trying to load at 0x%X, "
					"max addr is 0x%X)\n", segm.virtaddr, memory_len-1);

			// Copy data into memory
			memcpy(memory+segm.virtaddr, segm.data, segm.filesize);

			// Set padding
			if (segm.memsize > segm.filesize)
				memset(
					memory + segm.virtaddr + segm.filesize,
					0,
					segm.memsize - segm.filesize
				);

			// Set permissions
			uint8_t perm = parse_perm(segm.flags);
			set_perms(segm.virtaddr, segm.memsize, perm);

			// Update brk beyond any segm we load
			vaddr_t segm_next_page = 
				(segm.virtaddr + segm.memsize + 0xFFF) & ~0xFFF;
			brk = max(brk, segm_next_page);
		}
	}
	// Set next_alloc past brk area
	max_brk    = brk + MAX_BRK_SZ;
	min_brk    = brk;
	next_alloc = max_brk;
	dbgprintf("initial brk: 0x%X, max brk: 0x%X, initial alloc: 0x%X\n",
	        brk, max_brk, next_alloc);
}

ostream& operator<<(ostream& os, const Mmu& mmu){
	os << hex;
	for (vaddr_t i = 0x08048000; i < mmu.memory_len; i++){
		if (i % 0x10 == 0)
			os << endl << "0x" << setw(2) << setfill('0') << i << ": ";
		os << setw(2) << setfill('0') << (uint64_t)mmu.memory[i] << " ";
	}
	os << dec;
	return os;
}
