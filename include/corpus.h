#ifndef _CORPUS_H
#define _CORPUS_H

#include <vector>
#include <string>
#include <atomic>
#include <x86intrin.h>

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

class Corpus {
	private:
		static const int MUTATED_BYTES = 4;

		// Corpus
		std::vector<std::string> corpus;

		// Vector with one mutated input for each thread
		std::vector<std::string> mutated_inputs;

		// Lock for adding elements to `corpus`
		std::atomic_flag lock;

		// Mutate input in mutated_inputs[id]
		void mutate_input(int id, Rng& rng);

	public:
		Corpus(int nthreads, const std::string& pathname);

		size_t size();

		// Get a new mutated input
		const std::string& get_new_input(int id, Rng& rng);

		// Add input to corpus
		void add_input(const std::string& new_input);

		// Report coverage
		void last_input_was_nice_thanks(int id);

};

#endif