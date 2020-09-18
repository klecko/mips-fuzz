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
		uint64_t rnd(uint64_t min, uint64_t max){
			assert(max >= min);
			return min + (rnd() % (max-min+1));
		}
		uint64_t rnd_exp(uint64_t min, uint64_t max){
			//std::cout << max << " " << min << std::endl;
			uint64_t x = rnd(min, max);
			return rnd(min, x);
		}
};

// Type used for storing unique crashes. May be changed by a hash table
typedef std::vector<std::pair<vaddr_t, Fault>> crashes_t;

class Corpus {
public:
	static const int COVERAGE_MAP_SIZE = 64*1024;
	static const int MIN_MUTATIONS     = 1;
	static const int MAX_MUTATIONS     = 5;

	Corpus(int nthreads, const std::string& path);

	size_t size() const;
	size_t memsize() const;

	size_t get_cov() const;
	size_t get_uniq_crashes_size() const;

	// Get a new mutated input, which will be a constant reference to
	// `mutated_inputs[id]`
	const std::string& get_new_input(int id, Rng& rng);

	// Report coverage. If it has any new branch, save corresponding input
	// into corpus
	void report_cov(int id, const cov_t& cov);

	// Report a crash. If it is new, save it and write its associated input 
	// to disk
	void report_crash(int id, vaddr_t pc, const Fault& fault);

private:
	// Corpus and its lock
	std::vector<std::string> corpus;
	std::atomic_flag lock_corpus;

	// Vector with one mutated input for each thread
	std::vector<std::string> mutated_inputs;

	// Data structure for storing unique crashes and its lock
	crashes_t uniq_crashes;
	std::atomic_flag lock_uniq_crashes;

	// Recorded coverage across runs
	std::atomic<size_t> cov_n;
	std::vector<std::atomic_flag> recorded_cov;

	// Max input size, used in expand mutation
	size_t max_input_size;

	// Add input to corpus
	void add_input(const std::string& new_input);

	// Mutate input in `mutated_inputs[id]`
	void mutate_input(int id, Rng& rng);

	// Mutation strategies
	typedef void (Corpus::*mutation_strat_t)(std::string& input, Rng& rng);
	static const std::vector<mutation_strat_t> mut_strats;
	void shrink(std::string& input, Rng& rng);
	void expand(std::string& input, Rng& rng);
	void bit(std::string& input, Rng& rng);
	void dec_byte(std::string& input, Rng& rng);
	void inc_byte(std::string& input, Rng& rng);
	void neg_byte(std::string& input, Rng& rng);
	void add_sub(std::string& input, Rng& rng);
	void set(std::string& input, Rng& rng);
	void swap(std::string& input, Rng& rng);
	void copy(std::string& input, Rng& rng);
	void inter_splice(std::string& input, Rng& rng);
	void insert_rand(std::string& input, Rng& rng);
	void overwrite_rand(std::string& input, Rng& rng);
	void byte_repeat_overwrite(std::string& input, Rng& rng);
	void byte_repeat_insert(std::string& input, Rng& rng);
	void magic_overwrite(std::string& input, Rng& rng);
	void magic_insert(std::string& input, Rng& rng);
	void random_overwrite(std::string& input, Rng& rng);
	void random_insert(std::string& input, Rng& rng);
	void splice_overwrite(std::string& input, Rng& rng);
	void splice_insert(std::string& input, Rng& rng);
};

#endif
