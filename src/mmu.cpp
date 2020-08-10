#include <iostream>
#include <cstring>
#include <iomanip>
#include <sys/mman.h>
#include "mmu.h"
#include "common.h"
#include "elf_parser.hpp"

#define DIRTY_BLOCK_SIZE 64

using namespace std;

Mmu::Mmu(){
	memory     = NULL;
	memory_len = 0;
	perms      = NULL;
	next_alloc = 0;
	stack      = 0;
	brk        = 0;
	max_brk    = 0;
} 

Mmu::Mmu(vsize_t mem_size){
	memory     = new uint8_t[mem_size];
	memory_len = mem_size;
	perms      = new uint8_t[mem_size];
	next_alloc = 0;
	stack      = 0;
	brk        = 0;
	max_brk    = 0;
	dirty_bitmap.resize(memory_len/DIRTY_BLOCK_SIZE + 1, false);
	memset(memory, 0, mem_size);
	memset(perms, 0, mem_size);
}

Mmu::Mmu(const Mmu& other){
	memory_len   = other.memory_len;
	memory       = new uint8_t[memory_len];
	perms        = new uint8_t[memory_len];
	next_alloc   = other.next_alloc;
	stack        = other.stack;
	brk          = other.brk;
	max_brk      = other.max_brk;
	dirty_blocks = vector<vaddr_t>(other.dirty_blocks);
	dirty_bitmap = vector<bool>(other.dirty_bitmap);
	memcpy(memory, other.memory, memory_len);
	memcpy(perms, other.perms, memory_len);
}

Mmu::Mmu(Mmu&& other) noexcept : Mmu(){
	swap(*this, other);
}

Mmu::~Mmu(){
	delete[] memory;
	delete[] perms;
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
	swap(first.max_brk, second.max_brk);
	swap(first.dirty_blocks, second.dirty_blocks);
	swap(first.dirty_bitmap, second.dirty_bitmap);
}

vaddr_t Mmu::get_brk(){
	return brk;
}

bool Mmu::set_brk(vaddr_t new_brk){
	// Checks
	if (new_brk < brk)
		die("attempt to reduce brk to 0x%X, current is 0x%X\n", new_brk, brk);
	if (new_brk > max_brk)
		die("attempt to set brk past max_brk to 0x%X, current is 0x%X and max "
		    "is 0x%X\n", new_brk, brk, max_brk);

	// Zero out the new memory and set perms
	memset(memory + brk, 0, new_brk - brk);
	set_perms(brk, new_brk - brk, PERM_READ|PERM_WRITE|PERM_INIT);

	// Set new brk and return success
	brk = new_brk;
	return true;
}

void Mmu::check_bounds(vaddr_t addr, vsize_t len){
	if (addr + len > memory_len)
		throw Fault(Fault::Type::OutOfBounds, addr);
}

void Mmu::set_dirty(vaddr_t addr, vsize_t len){
	// Check out of bounds
	check_bounds(addr, len);

	// Set dirty those blocks that arent dirty
	vsize_t block_begin = addr/DIRTY_BLOCK_SIZE;
	vsize_t block_end   = (addr+len)/DIRTY_BLOCK_SIZE + 1;
	for (vsize_t block = block_begin; block < block_end; block++){
		if (!dirty_bitmap[block]){
			dirty_blocks.push_back(block);
			dirty_bitmap[block] = true;
		}
	} 
}

void Mmu::set_perms(vaddr_t addr, vsize_t len, uint8_t perm){
	// Check out of bounds
	check_bounds(addr, len);

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
void Mmu::check_perms(vaddr_t addr, vsize_t len, uint8_t perm){
	// Checking for reading implies checking for initialized memory
	if (perm == PERM_READ)
		perm |= PERM_INIT;

	vaddr_t addr_end = addr + len;
	for (; addr < addr_end; addr++){
		if ((perms[addr] & perm) != perm){ 
			// Permission error. Determine which
			if (addr == 0x49a4a8)
				printf("%X %X\n", perm, perms[addr]);
			if (perm == PERM_WRITE)
				throw Fault(Fault::Type::Write, addr);
			else if (perm == PERM_EXEC)
				throw Fault(Fault::Type::Exec, addr);
			else if (perms[addr] & PERM_READ)
				throw Fault(Fault::Type::Uninit, addr);
			else if (!(perms[addr] & PERM_READ))
				throw Fault(Fault::Type::Read, addr);
			else
				die("what\n");
		}
	}
}

void Mmu::read_mem(void* dst, vaddr_t src, vsize_t len){
	// Check out of bounds
	check_bounds(src, len);
	
	// Check perms
	check_perms(src, len, PERM_READ);

	// Copy memory
	memcpy(dst, memory+src, len);
}

void Mmu::write_mem(vaddr_t dst, const void* src, vsize_t len){
	// Check out of bounds
	check_bounds(dst, len);

	// Check perms
	check_perms(dst, len, PERM_WRITE);

	// Memory has been initialized, update perms
	for (int addr = dst; addr < dst+len; addr++)
		perms[addr] |= PERM_INIT;

	// Update dirty blocks
	set_dirty(dst, len);

	// Copy memory
	memcpy(memory+dst, src, len);
}

string Mmu::read_string(vaddr_t addr){
	string result = "";
	char c = read<char>(addr++);
	while (c){
		result += c;
		c = read<char>(addr++);
	}
	return result;
}

vaddr_t Mmu::alloc(vsize_t size){
	// Check out of memory
	if (next_alloc + size > stack)
		die("Out of memory allocating 0x%X bytes\n", size);

	vsize_t aligned_size  = size + 0xF & ~0xF;
	vaddr_t current_alloc = next_alloc;

	// Memory is by default readable and writable, but not initialized
	set_perms(current_alloc, size, PERM_READ|PERM_WRITE);

	// Update next allocation
	next_alloc += aligned_size;

	printf("alloc(0x%X) --> 0x%X\n", size, current_alloc);
	return current_alloc;
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

Mmu Mmu::fork(){
	// Create copy and reset dirty blocks
	Mmu new_mmu(*this);
	new_mmu.dirty_blocks.clear();
	new_mmu.dirty_bitmap.assign(new_mmu.dirty_bitmap.size(), false);
	return new_mmu;
}

void Mmu::reset(const Mmu& other){
	// Reset memory and perms for every dirty block
	for (vsize_t block : dirty_blocks){
		vaddr_t addr = block*DIRTY_BLOCK_SIZE;
		memcpy(memory + addr, other.memory + addr, DIRTY_BLOCK_SIZE);
		memcpy(perms  + addr, other.perms  + addr, DIRTY_BLOCK_SIZE);
		dirty_bitmap[block] = false;
	}

	dirty_blocks.clear();
	next_alloc = other.next_alloc;
}

uint8_t parse_perm(const string& flag){
	uint8_t perm = Mmu::PERM_INIT;
	if (flag.find("R") != string::npos)
		perm |= Mmu::PERM_READ;
	if (flag.find("W") != string::npos)
		perm |= Mmu::PERM_WRITE;
	if (flag.find("X") != string::npos)
		perm |= Mmu::PERM_EXEC;
	return perm;
}

void Mmu::load_elf(const vector<segment_t>& segments){
	for (const segment_t& s : segments){
		if (s.segment_type == "LOAD"){
			printf("Loading at 0x%X\n", s.segment_virtaddr);

			if (s.segment_virtaddr + s.segment_memsize > memory_len)
				die("Not enough space for loading elf (trying to load at 0x%X, "
					"max addr is 0x%X)\n", s.segment_virtaddr, memory_len-1);

			// Copy data into memory
			memcpy(memory+s.segment_virtaddr, s.data, s.segment_filesize);

			// Set padding
			if (s.segment_memsize > s.segment_filesize)
				memset(memory + s.segment_virtaddr + s.segment_filesize,
					0, s.segment_memsize - s.segment_filesize);

			// Set permissions
			uint8_t perm = parse_perm(s.segment_flags);
			set_perms(s.segment_virtaddr, s.segment_memsize, perm);

			// Update brk beyond any segment we load
			vaddr_t segm_next_page = 
				s.segment_virtaddr + s.segment_memsize + 0xFFF & ~0xFFF;
			brk = max(brk, segm_next_page);
		}
	}
	// Set next_alloc past brk area
	max_brk    = brk + MAX_BRK_SZ;
	next_alloc = max_brk;
	printf("initial brk: 0x%X, max brk: 0x%X, initial alloc: 0x%X\n",
	        brk, max_brk, next_alloc);
}

ostream& operator<<(ostream& os, const Mmu& mmu){
	os << hex;
	for (int i = 0x08048000; i < mmu.memory_len; i++){
		if (i % 0x10 == 0)
			os << endl << "0x" << setw(2) << setfill('0') << i << ": ";
		os << setw(2) << setfill('0') << (uint64_t)mmu.memory[i] << " ";
	}
	os << dec;
	return os;
}

ostream& operator<<(ostream& os, const Fault& f){
	switch (f.type){
		case Fault::Type::Read:
			os << "Read";
			break;
		case Fault::Type::Write:
			os << "Write";
			break;
		case Fault::Type::Exec:
			os << "Exec";
			break;
		case Fault::Type::Uninit:
			os << "Uninit";
			break;
		case Fault::Type::OutOfBounds:
			os << "OutOfBounds";
			break;
		case Fault::Type::MisalignedRead:
			os << "MisalignedRead";
			break;
		case Fault::Type::MisalignedWrite:
			os << "MisalignedWrite";
			break;
		default:
			os << "UnimplementedFault";
	}
	os << " Fault, fault address = 0x" << hex << f.fault_addr << dec;
	return os;
}