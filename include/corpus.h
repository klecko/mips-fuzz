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



// Coverage hasher
struct cov_hasher {
	std::size_t operator()(const cov_t& c) const{
		std::size_t hash = c.vec.size();
		for (const vaddr_t& addr : c.vec){
			hash ^= addr + (hash << 6) + (hash >> 2);
		}
		return hash;
	}
};

// Type used for storing recorded coverage across runs
typedef std::unordered_map<cov_t, bool, cov_hasher> recorded_cov_t;

// Type used for storing unique crashes. May be changed by a hash table
typedef std::vector<std::pair<vaddr_t, Fault>> crashes_t;

class Corpus {
	private:
		static const int MUTATED_BYTES = 32;

		// Corpus and its lock
		std::vector<std::string> corpus;
		std::atomic_flag lock_corpus;

		// Vector with one mutated input for each thread
		std::vector<std::string> mutated_inputs;

		// Data structure for storing recorded coverage across runs and its lock
		recorded_cov_t recorded_cov;
		std::atomic_flag lock_recorded_cov;

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
		size_t cov_size() const;
		size_t uniq_crashes_size() const;

		// Get a new mutated input, which will be a constant reference to
		// `mutated_inputs[id]`
		const std::string& get_new_input(int id, Rng& rng);

		// Report coverage. If it is new, save it and add the corresponding
		// input to the corpus
		void report_cov(int id, const cov_t& cov);

		// Report a crash. If it is new, save it and write its associated input 
		// to disk
		void report_crash(int id, vaddr_t pc, const Fault& fault);

};

#endif
