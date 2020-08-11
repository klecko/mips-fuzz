#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring> // strerror
#include <dirent.h>
#include "corpus.h"
#include "common.h"

using namespace std;

Corpus::Corpus(const string& pathname){
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
		// For each regular file, introduce its content into `inputs`
		ifstream ifs(pathname + "/" + ent->d_name);
		ostringstream ss;
		ss << ifs.rdbuf();
		inputs.push_back(ss.str());
		cout << "Read " << ent->d_name << ", size: " << ss.str().size() << endl;
	}
	closedir(dir);
	
	cout << "Total files read: " << inputs.size() << endl;
}

size_t Corpus::size(){
	return inputs.size();
}

void Corpus::mutate_input(){
	for (int i = 0; i < MUTATED_BYTES; i++)
		mutated_input[rand() % mutated_input.size()] = rand() % 0xFF;
}

const std::string& Corpus::get_new_input(){
	// Get a random input and mutate it
	size_t i = rand() % (inputs.size());
	mutated_input = inputs[i];
	mutate_input();
	return mutated_input;
}

void Corpus::add_input(const std::string& new_input){
	inputs.push_back(new_input);
}

void Corpus::last_input_was_nice_thanks(){
	inputs.push_back(mutated_input);
}