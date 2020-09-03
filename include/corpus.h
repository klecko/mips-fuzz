#ifndef _CORPUS_H
#define _CORPUS_H

#include <unordered_map>
#include <vector>
#include <string>
#include <atomic>
#include <x86intrin.h>
#include "common.h"
#include "fault.h"

// Used for mutating inputs. We don't use glibc rand() because it uses locks
// in order to be thread safe. Instead, we implement a simpler algorithm, and
// each thread will have its own rng.
class Rng {
	private:
		uint64_t state;
	public:
		Rng(){
			state = _rdtsc();
		}
		uint64_t rnd(){
			// xorshif64*
			state ^= state >> 12;
			state ^= state << 25;
			state ^= state >> 27;
			return state * 2685821657736338717LL;
		}
};

// Type used for storing unique crashes. May be changed by a hash table
typedef std::vector<std::pair<vaddr_t, Fault>> crashes_t;

class Corpus {
	private:
		static const int MUTATED_BYTES = 2;

		// Corpus and its lock
		std::vector<std::string> corpus;
		std::atomic_flag lock_corpus;

		// Vector with one mutated input for each thread
		std::vector<std::string> mutated_inputs;

		// Data structure for storing unique crashes and its lock
		crashes_t uniq_crashes;
		std::atomic_flag lock_uniq_crashes;

		// Mutate input in `mutated_inputs[id]`
		void mutate_input(int id, Rng& rng);

		// Add input to corpus
		void add_input(const std::string& new_input);

	public:
		Corpus(int nthreads, const std::string& path);

		size_t size() const;
		size_t memsize() const;
		size_t uniq_crashes_size() const;

		// Get a new mutated input, which will be a constant reference to
		// `mutated_inputs[id]`
		const std::string& get_new_input(int id, Rng& rng);

		// Report new coverage: save corresponding input into corpus
		void report_new_cov(int id);

		// Report a crash. If it is new, save it and write its associated input 
		// to disk
		void report_crash(int id, vaddr_t pc, const Fault& fault);

};

#endif
