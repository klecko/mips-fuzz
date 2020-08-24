#ifndef _JIT_CACHE_H
#define _JIT_CACHE_H

#include <unordered_map>
#include <atomic>
#include <sys/mman.h>
#include "common.h"

class JitCache {
	private:
		uint8_t* mem;
		size_t size;
		size_t max_size;
		std::unordered_map<vaddr_t, jit_block_t> cache;
		std::atomic_flag lock;

	public:
		JitCache(size_t sz){
			mem = (uint8_t*)mmap(NULL, sz, PROT_READ|PROT_WRITE|PROT_EXEC,
								 MAP_ANONYMOUS|MAP_PRIVATE, -1, 0);
			if (mem == MAP_FAILED)
				die("jit cache mmap failed\n");

			size = 0;
			max_size = sz;
			lock.clear();
		}

		void add(vaddr_t pc, std::string code){
			while (lock.test_and_set());

			// Test OOM
			if (size + code.size() > max_size)
				die("OOM jit\n");

			// Copy code, add its address to the map and update size
			memcpy(mem + size, code.c_str(), code.size());
			cache[pc] = (jit_block_t)(mem + size);
			size += code.size();

			lock.clear();
		}

		bool is_cached(vaddr_t pc){
			return cache.count(pc);
		}

		jit_block_t get(vaddr_t pc){
			return cache.at(pc);
		}
};

#endif