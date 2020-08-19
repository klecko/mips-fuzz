#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring> // strerror
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "corpus.h"
#include "common.h"

using namespace std;

Corpus::Corpus(int nthreads, const string& path){
	// One element for each thread
	mutated_inputs.resize(nthreads);

	// Try to open the directory
	DIR* dir = opendir(path.c_str());
	if (!dir)
		die("Error opening directory %s: %s\n", path.c_str(),
		     strerror(errno));

	// Iterate the directory
	struct dirent* ent;
	struct stat st;
	string filepath;
	while (ent = readdir(dir)){
		filepath = path + "/" + ent->d_name;

		// Check file type. If readdir fails to provide it, fallback
		// to stat
		if (ent->d_type == DT_UNKNOWN){
			stat(filepath.c_str(), &st);
			if (!S_ISREG(st.st_mode))
				continue;
		} else if (ent->d_type != DT_REG)
			continue;

		// For each regular file, introduce its content into `corpus`
		ifstream ifs(filepath);
		ostringstream ss;
		ss << ifs.rdbuf();
		corpus.push_back(ss.str());
		cout << "Read file '" << ent->d_name << "', size: " << ss.str().size() << endl;
	}
	closedir(dir);

	lock_corpus.clear();
	lock_recorded_cov.clear();
	lock_uniq_crashes.clear();

	if (corpus.size() == 0)
		die("Empty corpus\n");
	cout << "Total files read: " << corpus.size() << endl;
}

size_t Corpus::size() const {
	return corpus.size();
}

size_t Corpus::memsize() const {
	size_t sz = 0;
	for (const string& s : corpus)
		sz += s.size();
	return sz;
}

size_t Corpus::cov_size() const {
	return recorded_cov.size();
}

size_t Corpus::uniq_crashes_size() const {
	return uniq_crashes.size();
}

void Corpus::mutate_input(int id, Rng& rng){
	string& input = mutated_inputs[id];
	for (int i = 0; i < MUTATED_BYTES; i++)
		input[rng.rnd() % input.size()] = rng.rnd() % 0xFF;
}

void Corpus::add_input(const std::string& new_input){
	while (lock_corpus.test_and_set());
	corpus.push_back(new_input);
	lock_corpus.clear();
}

const std::string& Corpus::get_new_input(int id, Rng& rng){
	// Copy a random input and mutate it. Is a lock necessary here? shared_mutex
	mutated_inputs[id] = corpus[rng.rnd() % corpus.size()];
	mutate_input(id, rng);
	return mutated_inputs[id];
}

void Corpus::report_cov(int id, const cov_t& cov){
	while (lock_recorded_cov.test_and_set());
	if (!recorded_cov.count(cov)){
		// New coverage, add input to corpus and save coverage
		add_input(mutated_inputs[id]);
		recorded_cov[cov] = true;
	}
	lock_recorded_cov.clear();
}

void Corpus::report_crash(int id, vaddr_t pc, const Fault& fault){
	auto crash = make_pair(pc, fault);
	while (lock_uniq_crashes.test_and_set());

	// Look for the crash
	auto it = uniq_crashes.begin();
	while (it != uniq_crashes.end() && *it != crash) ++it;

	// If it is a new crash, save it and write its associaded input to disk
	if (it == uniq_crashes.end()){
		uniq_crashes.push_back(move(crash));
		cout << "[PC: 0x" << hex << pc << "] " << fault << endl;

		ostringstream filename;
		filename << fault.type_str() << "_0x" << hex << pc << "_0x"
		         << fault.fault_addr;
		ofstream ofs("./crashes/" + filename.str());
		ofs << mutated_inputs[id];
		ofs.close();
	}
	lock_uniq_crashes.clear();
}
