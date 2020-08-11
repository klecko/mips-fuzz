#ifndef _CORPUS_H
#define _CORPUS_H

#include <vector>
#include <string>
#include <atomic>

class Corpus {
	private:
		static const int MUTATED_BYTES = 1;

		// Corpus
		std::vector<std::string> corpus;

		// Vector with one mutated input for each thread
		std::vector<std::string> mutated_inputs;

		// Lock for adding elements to `corpus`
		std::atomic_flag lock;

		// Mutate input in mutated_inputs[id]
		void mutate_input(int id);

	public:
		Corpus(int nthreads, const std::string& pathname);

		size_t size();

		// Get a new mutated input
		const std::string& get_new_input(int id);

		// Add input to corpus
		void add_input(const std::string& new_input);

		// Report coverage
		void last_input_was_nice_thanks(int id);

};

#endif