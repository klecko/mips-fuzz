#ifndef _MMU_H
#define _MMU_H

#include <stdint.h>
#include <cstdlib>
#include <bitset>
#include <vector>

/*
It might be better registering separately dirty memory and dirty perms?

Change almost every die with a exception
*/

// Type used for guest virtual addresses
typedef size_t addr_t;

enum FaultType{
	Read,
	Write,
	Exec,
	Uninit,
	Miss
};

struct Fault: public std::exception{
	FaultType type;
	addr_t    addr;
};

class Mmu {
	public:
		static const uint8_t PERM_READ  = (1 << 0); // Can be read
		static const uint8_t PERM_WRITE = (1 << 1); // Can be written
		static const uint8_t PERM_EXEC  = (1 << 2); // Can be executed
		static const uint8_t PERM_INIT  = (1 << 3); // Has been initialized

	private:
		Mmu& operator=(const Mmu& other){};

		// Guest virtual memory
		uint8_t* memory;
		size_t   memory_len;

		// Byte-level permissions for guest virtual memory
		uint8_t* perms;

		// Next allocation returned by `alloc`
		addr_t   next_alloc;

		// Holds the index of every dirty block
		std::vector<addr_t> dirty_blocks;

		// Bitmap for every block, true if dirty
		std::vector<bool>   dirty_bitmap;

		// Sets region from `addr` to `addr+len` as dirty
		void set_dirty(addr_t addr, size_t len);

		// Set permissions `perm` from `addr` to `addr+len`
		// Updates dirty
		void set_perms(addr_t addr, size_t len, uint8_t perm);

		// Checks that all bytes from `addr` to `addr+len` have `perm`
		// permission. `perm` should be PERM_READ, PERM_WRITE or PERM_EXEC.
		// It will throw an exception is permissions are not fulfilled.
		// Doesn't check out of bounds
		void check_perms(addr_t addr, size_t len, uint8_t perm);

		// Read `len` bytes from virtual addr `src` into `dst` checking perms
		void read_mem(void* dst, addr_t src, size_t len);

		// Write `len` bytes from `src` into virtual addr `dst` checking perms
		void write_mem(addr_t dst, void* src, size_t len);

	public:
		Mmu(size_t mem_size);

		Mmu(const Mmu& other);

		~Mmu();

		// Allocates a block of `size` bytes. Default perms are RW
		addr_t alloc(size_t size);

		// Reads a value from memory
		template <class T>
		T read(addr_t addr);

		// Writes a value to memory
		template <class T>
		void write(addr_t addr, T value);

		// Forks the Mmu and returns the child
		Mmu fork();

		// Resets the Mmu to the parent which has previously been forked from
		void reset(const Mmu& other);

		void load_elf(const char* pathname);

		// Hexdumps the memory
		friend std::ostream& operator<<(std::ostream& os, const Mmu& mmu);
};

template<class T>
T Mmu::read(addr_t addr){
	T result;
	read_mem(&result, addr, sizeof(T));
	return result;
}

template<class T>
void Mmu::write(addr_t addr, T value){
	write_mem(addr, &value, sizeof(T));
}

#endif