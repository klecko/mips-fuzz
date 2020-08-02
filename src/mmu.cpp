#include <iostream>
#include <cstring>
#include <iomanip>
#include <sys/mman.h>
#include "mmu.h"
#include "common.h"

#define DIRTY_BLOCK_SIZE 64

using namespace std;

Mmu::Mmu(size_t mem_size){
	memory     = new uint8_t[mem_size];
	memory_len = mem_size;
	perms      = new uint8_t[mem_size];
	next_alloc = 0;
	dirty_bitmap.resize(memory_len/DIRTY_BLOCK_SIZE + 1, false);
	memset(memory, 0, mem_size);
	memset(perms, 0, mem_size);
}

Mmu::Mmu(const Mmu& other){
	memory_len = other.memory_len;
	memory     = new uint8_t[memory_len];
	perms      = new uint8_t[memory_len];
	memcpy(memory, other.memory, memory_len);
	memcpy(perms, other.perms, memory_len);
	next_alloc   = other.next_alloc;
	dirty_blocks = vector<addr_t>(other.dirty_blocks);
	dirty_bitmap = vector<bool>(other.dirty_bitmap);
}

Mmu::~Mmu(){
	delete[] memory;
	delete[] perms;
}

void Mmu::check_bounds(addr_t addr, size_t len){
	if (addr + len > memory_len)
		throw Fault(Fault::Type::OutOfBounds, addr);
		/*die("Out of bounds: accessing 0x%lX with len 0x%lX (size is "
		    "0x%lX)", addr, len, memory_len);*/
}

void Mmu::set_dirty(addr_t addr, size_t len){
	// Check out of bounds
	check_bounds(addr, len);

	// Set dirty those blocks that arent dirty
	size_t block_begin = addr/DIRTY_BLOCK_SIZE;
	size_t block_end   = (addr+len)/DIRTY_BLOCK_SIZE + 1;
	for (size_t block = block_begin; block < block_end; block++){
		if (!dirty_bitmap[block]){
			dirty_blocks.push_back(block);
			dirty_bitmap[block] = true;
		}
	} 
}

void Mmu::set_perms(addr_t addr, size_t len, uint8_t perm){
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
void Mmu::check_perms(addr_t addr, size_t len, uint8_t perm){
	// Checking for reading implies checking for initialized memory
	if (perm == PERM_READ)
		perm |= PERM_INIT;

	addr_t addr_end = addr + len;
	for (; addr < addr_end; addr++){
		if ((perms[addr] & perm) != perm){ 
			// Permission error. Determine which
			if (perm == PERM_WRITE)
				throw Fault(Fault::Type::Write, addr);
				//die("Write fault accessing 0x%lX\n", addr);
			else if (perm == PERM_EXEC)
				throw Fault(Fault::Type::Exec, addr);
				//die("Exec fault accessing 0x%lX\n", addr);
			else if (perms[addr] & PERM_READ)
				throw Fault(Fault::Type::Uninit, addr);
				//die("Uninit fault accessing 0x%lX\n", addr);
			else
				throw Fault(Fault::Type::Read, addr);
				//die("Read fault accessing 0x%lX\n", addr);
		}
	}
}

void Mmu::read_mem(void* dst, addr_t src, size_t len){
	// Check out of bounds
	check_bounds(src, len);
	
	// Check perms
	check_perms(src, len, PERM_READ);

	// Copy memory
	memcpy(dst, memory+src, len);
}

void Mmu::write_mem(addr_t dst, void* src, size_t len){
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

addr_t Mmu::alloc(size_t size){
	// Check out of memory
	if (next_alloc + size > memory_len)
		die("Out of memory allocating 0x%lX bytes", size);

	size_t aligned_size  = (size + 0xF) & (!0xF);
	addr_t current_alloc = next_alloc;

	// Memory is by default readable and writable, but not initialized
	set_perms(current_alloc, size, PERM_READ|PERM_WRITE);

	// Update next allocation
	next_alloc += aligned_size;

	return current_alloc;
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
	for (size_t block : dirty_blocks){
		addr_t addr = block*DIRTY_BLOCK_SIZE;
		memcpy(memory+addr, other.memory+addr, DIRTY_BLOCK_SIZE);
		memcpy(perms+addr, other.perms+addr, DIRTY_BLOCK_SIZE);
		dirty_bitmap[block] = false;
	}

	dirty_blocks.clear();
	next_alloc = other.next_alloc;
}

void Mmu::load_elf(const char* pathname){
	// TODO
}

ostream& operator<<(ostream& os, const Mmu& mmu){
	os << hex;
	for (int i = 0; i < mmu.memory_len; i++){
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
	}
	os << " Fault, fault address = 0x" << hex << f.fault_addr << dec;
	return os;
}