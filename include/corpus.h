#ifndef _CORPUS_H
#define _CORPUS_H

#include <vector>
#include <string>

class Corpus {
	private:
		static const int MUTATED_BYTES = 1;

		std::vector<std::string> inputs;
		std::string mutated_input;

		void mutate_input();

	public:
		Corpus(const std::string& pathname);

		// Get a new mutated input
		const std::string& get_new_input();

		// Add input to corpus
		void add_input(const std::string& new_input);

		// Report coverage
		void last_input_was_nice_thanks();
};

#endif