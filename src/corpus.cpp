#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring> // strerror
#include <dirent.h>
#include "corpus.h"
#include "common.h"

using namespace std;

Corpus::Corpus(int nthreads, const string& pathname){
	// One element for each thread
	mutated_inputs.resize(nthreads);

	// Try to open the directory
	DIR* dir = opendir(pathname.c_str());
	if (!dir)
		die("Error opening directory %s: %s\n", pathname.c_str(), 
		     strerror(errno));
	
	// Iterate the directory
	struct dirent* ent;
	while (ent = readdir(dir)){
		if (ent->d_type != DT_REG) 
			continue;
		// For each regular file, introduce its content into `corpus`
		ifstream ifs(pathname + "/" + ent->d_name);
		ostringstream ss;
		ss << ifs.rdbuf();
		corpus.push_back(ss.str());
		cout << "Read file '" << ent->d_name << "', size: " << ss.str().size() << endl;
	}
	closedir(dir);

	lock.clear();
	
	cout << "Total files read: " << corpus.size() << endl;
}

size_t Corpus::size(){
	return corpus.size();
}

void Corpus::mutate_input(int id){
	string& input = mutated_inputs[id];
	for (int i = 0; i < MUTATED_BYTES; i++)
		input[rand() % input.size()] = rand() % 0xFF;
}

const std::string& Corpus::get_new_input(int id){
	// Get a random input and mutate it
	size_t i = rand() % (corpus.size());
	mutated_inputs[id] = corpus[i]; // performs copy
	mutate_input(id);
	return mutated_inputs[id];
}

void Corpus::add_input(const std::string& new_input){
	while (lock.test_and_set());
	corpus.push_back(new_input);
	lock.clear();
}

void Corpus::last_input_was_nice_thanks(int id){
	add_input(mutated_inputs[id]);
}