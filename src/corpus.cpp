#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring> // strerror
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "corpus.h"
#include "common.h"
#include "magic_values.h"

using namespace std;

Corpus::Corpus(int nthreads, const string& path):
	lock_corpus(false), mutated_inputs(nthreads), lock_uniq_crashes(false),
	cov_n(0), recorded_cov(COVERAGE_MAP_SIZE)
{
	// Try to open the directory
	DIR* dir = opendir(path.c_str());
	if (!dir)
		die("Error opening directory %s: %s\n", path.c_str(),
		     strerror(errno));

	// Iterate the directory
	struct dirent* ent;
	struct stat st;
	string filepath, input;
	size_t max_sz = 0;
	while ((ent = readdir(dir))){
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
		input = ss.str();
		corpus.push_back(input);

		// Record the size of the largest initial file
		max_sz = max(max_sz, input.size());

		cout << "Read file '" << ent->d_name << "', size: "
		     << input.size() << endl;
	}
	closedir(dir);

	// Set max_input_size to X times the size of the largest initial file
	// This will be the maximum size of inputs produced by mutations
	max_input_size = 10*max_sz;

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

size_t Corpus::get_cov() const {
	return cov_n;
}

size_t Corpus::get_uniq_crashes_size() const {
	return uniq_crashes.size();
}

const std::string& Corpus::get_new_input(int id, Rng& rng){
	// Copy a random input and mutate it
	while (lock_corpus.test_and_set());
	mutated_inputs[id] = corpus[rng.rnd() % corpus.size()];
	lock_corpus.clear();
	mutate_input(id, rng);
	return mutated_inputs[id];
}

void Corpus::report_cov(int id, const cov_t& cov){
	assert(cov.size() == recorded_cov.size());
	size_t cov_size = cov.size();

	// If there's any new coverage, record it and increment coverage counter
	bool new_cov = false;
	for (size_t i = 0; i < cov_size; i++)
		if (cov[i] && (!recorded_cov[i].test_and_set())){
			cov_n++;
			new_cov |= true;
		}

	// If there was new coverage, add associated input to corpus
	if (new_cov)
		add_input(mutated_inputs[id]);
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

void Corpus::add_input(const string& new_input){
	while (lock_corpus.test_and_set());
	corpus.push_back(new_input);
	lock_corpus.clear();
}

// MUTATION STUFF
const vector<Corpus::mutation_strat_t> Corpus::mut_strats = {
	&Corpus::shrink,
	&Corpus::expand,
	&Corpus::bit,
	&Corpus::dec_byte,
	&Corpus::inc_byte,
	&Corpus::neg_byte,
	&Corpus::add_sub,
	&Corpus::set,
	&Corpus::swap,
	&Corpus::copy,
	&Corpus::inter_splice,
	&Corpus::insert_rand,
	&Corpus::overwrite_rand,
	&Corpus::byte_repeat_overwrite,
	&Corpus::byte_repeat_insert,
	&Corpus::magic_overwrite,
	&Corpus::magic_insert,
	&Corpus::random_overwrite,
	&Corpus::random_insert,
	&Corpus::splice_overwrite,
	&Corpus::splice_insert,
};

void Corpus::mutate_input(int id, Rng& rng){
	string& input = mutated_inputs[id];
	mutation_strat_t mut_strat;
	for (size_t i = 0; i < rng.rnd(MIN_MUTATIONS, MAX_MUTATIONS); i++){
		mut_strat = mut_strats[rng.rnd(0, mut_strats.size()-1)];
		(this->*mut_strat)(input, rng);
	}
	assert(input.size() <= max_input_size);
}

/* POR QUÉ AL INSERTAR COGE EL TAMAÑO COMO SI FUERA A OVERWRITEAR?
   ESO HACE QUE EL TAMAÑO DE LO QUE INSERTAS SEA MÁS PEQUEÑO CONFORME MÁS
   CERCA DEL FINAL LO INSERTES */

size_t rand_offset(const string& input, Rng& rng, bool plus_one=false){
	// Special case when input is empty. Some mutations may want to insert at
	// index 0. In that case, `plus_one` must be true
	if (input.empty()){
		assert(plus_one);
		return 0;
	}
	return rng.rnd_exp(0, input.size() - (!plus_one));
}

void Corpus::shrink(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Offset to remove data at
	size_t offset = rand_offset(input, rng);

	// Maximum number of bytes we'll remove.
	// 15/16 chance of removing at most 16 bytes
	//  1/16 chance of removing a random amount of bytes to the end of the input
	size_t max_remove = input.size() - offset;
	size_t rnd = rng.rnd(1, 16);
	max_remove = (rnd == 1 ? max_remove : min((size_t)16, max_remove));

	// Number of bytes we'll remove
	size_t to_remove = rng.rnd_exp(1, max_remove);

	// Remove bytes
	input.erase(offset, to_remove);
}

void Corpus::expand(string& input, Rng& rng){
	// Check size
	if (input.size() >= max_input_size)
		return;

	// Offset to insert data at
	size_t offset = rand_offset(input, rng, true);

	// Maximum number of bytes we'll insert. Same as in `shrink`
	size_t max_expand = max_input_size - input.size();
	size_t rnd = rng.rnd(1, 16);
	max_expand = (rnd == 1 ? max_expand : min((size_t)16, max_expand));

	// Number of bytes we'll insert
	size_t to_expand = rng.rnd_exp(1, max_expand);

	// Insert bytes
	input.insert(offset, to_expand, '\x00');
}

void Corpus::bit(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Flip random bit at random offset
	size_t offset  = rand_offset(input, rng);
	uint8_t bit    = rng.rnd(0, 7);
	input[offset] ^= (1 << bit);
}

void Corpus::inc_byte(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Increment byte at random offset
	size_t offset = rand_offset(input, rng);
	input[offset]++;
}

void Corpus::dec_byte(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Decrement byte at random offset
	size_t offset = rand_offset(input, rng);
	input[offset]--;
}

void Corpus::neg_byte(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Negate byte at random offset
	size_t offset = rand_offset(input, rng);
	input[offset] = ~input[offset];
}

void Corpus::add_sub(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Offset of the integer we'll modify
	size_t offset = rand_offset(input, rng);

	// Remaining bytes
	size_t remain = input.size() - offset;

	// Get the size of the integer we'll modify
	size_t int_size;
	if (remain >= 8)
		int_size = 1 << rng.rnd(0, 3); // size 1, 2, 4 or 8
	else if (remain >= 4)
		int_size = 1 << rng.rnd(0, 2); // size 1, 2 or 4
	else if (remain >= 2)
		int_size = 1 << rng.rnd(0, 1); // size 1 or 2
	else
		int_size = 1;                  // size 1

	// Helper macros
	#define __builtin_bswap8(n) n
	#define bswap(sz, n) __builtin_bswap##sz(n)
	#define mut(sz) do { \
		/* Get the number we'll add, which can be negative */                  \
		int32_t delta = rng.rnd(1, range*2) - range;                           \
		                                                                       \
		/* Read bytes, interpret them as an integer with random endianness, */ \
		/* add delta and store back those bytes */                             \
		uint##sz##_t n = *(uint##sz##_t*)(input.c_str()+offset);               \
		bool swap_endianness = rng.rnd(0, 1) && (sz != 8);                     \
		if (swap_endianness)                                                   \
		    n = bswap(sz, n);                                                  \
		n += delta;                                                            \
		if (swap_endianness)                                                   \
		    n = bswap(sz, n);                                                  \
		*(uint##sz##_t*)(input.c_str()+offset) = n;                            \
	} while (0)

	// Get maximum number to add or substract depending on int size and mutate
	size_t range;
	if (int_size == 1){
		range = 16;
		mut(8);
	} else if (int_size == 2){
		range = 4096;
		mut(16);
	} else if (int_size == 4){
		range = 1024 * 1024;
		mut(32);
	} else {
		range = 256 * 1024 * 1024;
		mut(64);
	}

	#undef mut
}

void Corpus::set(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Get offset, len and value to memset
	size_t  offset = rand_offset(input, rng);
	size_t  len    = rng.rnd_exp(1, input.size() - offset);
	uint8_t c      = rng.rnd(0, 255);

	// Replace
	input.replace(offset, len, len, c);
}

void Corpus::swap(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Get random offsets and their remaining bytes
	size_t offset1 = rand_offset(input, rng);
	size_t offset2 = rand_offset(input, rng);
	size_t offset1_remaining = input.size() - offset1;
	size_t offset2_remaining = input.size() - offset2;

	// Get random length
	size_t len = rng.rnd_exp(1, min(offset1_remaining, offset2_remaining));

	// Save input[offset1 : offset1+len] into tmp
	string tmp = input.substr(offset1, len);

	// Set input[offset1 : offset1+len] = input[offset2 : offset2+len]
	input.replace(offset1, len, input, offset2, len);

	// Set input[offset2 : offset2+len] = tmp
	input.replace(offset2, len, tmp);
}

void Corpus::copy(string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Get random offsets and their remaining bytes
	size_t src = rand_offset(input, rng);
	size_t dst = rand_offset(input, rng);
	size_t src_remaining = input.size() - src;
	size_t dst_remaining = input.size() - dst;

	// Get random length
	size_t len = rng.rnd_exp(1, min(src_remaining, dst_remaining));

	// Replace
	input.replace(dst, len, input, src, len);
}

void Corpus::inter_splice(std::string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Check size
	if (input.size() >= max_input_size)
		return;

	// Get random offsets and a random length
	size_t src = rand_offset(input, rng);
	size_t dst = rand_offset(input, rng, true);
	size_t src_remaining = input.size() - src;
	size_t max_insert    = max_input_size - input.size();
	size_t len = rng.rnd_exp(1, min(src_remaining, max_insert));

	// Insert
	input.insert(dst, input, src, len);
}

inline void fill_rand_bytes(string& s, Rng& rng){
	size_t len = s.size();
	for (size_t i = 0; i < len; i++)
		s[i] = rng.rnd(0x00, 0xFF);
}

void Corpus::insert_rand(std::string& input, Rng& rng){
	// Check size
	if (input.size() >= max_input_size)
		return;

	// Get one or two random bytes and insert them at a random offset
	size_t max_insert = max_input_size - input.size();
	size_t offset = rand_offset(input, rng, true);
	string bytes(min(rng.rnd(1,2), max_insert), '\x00');
	fill_rand_bytes(bytes, rng);
	input.insert(offset, bytes);
}

void Corpus::overwrite_rand(std::string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Overwrite one or two input bytes with random bytes
	size_t offset = rand_offset(input, rng);
	size_t len    = (input.size()-offset > 1 ? rng.rnd(1, 2) : 1);
	string bytes(len, '\x00');
	fill_rand_bytes(bytes, rng);
	input.replace(offset, len, bytes);
}

void Corpus::byte_repeat_overwrite(std::string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Get random offset and amount
	size_t offset = rand_offset(input, rng);
	size_t amount = rng.rnd_exp(1, input.size() - offset);

	// Get byte to repeat and overwrite `amount` bytes with it
	char val = input[offset];
	for (size_t i = 0; i < amount; i++)
		input[offset + i + 1] = val;
}

void Corpus::byte_repeat_insert(std::string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Check size
	if (input.size() >= max_input_size)
		return;

	// Get random offset and amount
	size_t offset = rand_offset(input, rng);
	size_t max_amount = max_input_size - input.size();
	size_t amount = rng.rnd_exp(1, min(input.size() - offset, max_amount));

	// Get byte to repeat and insert it `amount` times
	char val = input[offset];
	string bytes(amount, val);
	input.insert(offset, bytes);
}

void Corpus::magic_overwrite(std::string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Get random offset and magic value
	size_t offset = rand_offset(input, rng);
	string magic_value = MAGIC_VALUES[rng.rnd(0, MAGIC_VALUES.size()-1)];

	// Truncate magic value if needed
	size_t remain = input.size() - offset;
	size_t len    = min(magic_value.size(), remain);

	// Replace bytes with magic value
	input.replace(offset, len, magic_value, 0, len);
}

void Corpus::magic_insert(std::string& input, Rng& rng){
	// Check size
	if (input.size() >= max_input_size)
		return;

	// Get random offset and magic value
	size_t offset = rand_offset(input, rng, true);
	string magic_value = MAGIC_VALUES[rng.rnd(0, MAGIC_VALUES.size()-1)];

	// Truncate magic value if needed
	size_t max_len = max_input_size - input.size();
	size_t len     = min(magic_value.size(), max_len);

	// Insert magic value
	input.insert(offset, magic_value, 0, len);
}

void Corpus::random_overwrite(std::string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Get random offset and amount to overwrite
	size_t offset = rand_offset(input, rng);
	size_t amount = rng.rnd_exp(1, input.size() - offset);

	// Get random bytes
	string bytes(amount, '\x00');
	fill_rand_bytes(bytes, rng);

	// Replace with random bytes
	input.replace(offset, amount, bytes);
}

void Corpus::random_insert(std::string& input, Rng& rng){
	// Check size
	if (input.size() >= max_input_size)
		return;

	// Get random offset and amount to insert
	size_t offset = rand_offset(input, rng, true);
	size_t max_amount = max_input_size - input.size();
	size_t amount = rng.rnd_exp(0, min(input.size() - offset, max_amount));

	// Get random bytes
	string bytes(amount, '\x00');
	fill_rand_bytes(bytes, rng);

	// Insert random bytes
	input.insert(offset, bytes);
}

void Corpus::splice_overwrite(std::string& input, Rng& rng){
	// Check empty input
	if (input.empty())
		return;

	// Get the random input we'll copy from
	const string& inp = corpus[rng.rnd(0, corpus.size()-1)];

	// Get dst and src offsets and lengths, making sure we don't exceed
	// max_input_size when replacing
	size_t dst_off = rand_offset(input, rng);
	size_t dst_len = rng.rnd_exp(1, input.size() - dst_off);
	size_t src_off = rand_offset(inp, rng);
	size_t max_src_len = max_input_size - input.size() + dst_len;
	size_t src_len = rng.rnd_exp(1, min(inp.size() - src_off, max_src_len));

	// Replace
	input.replace(dst_off, dst_len, inp, src_off, src_len);
}

void Corpus::splice_insert(std::string& input, Rng& rng){
	// Check size
	if (input.size() >= max_input_size)
		return;

	// Get the random input we'll copy from
	const string& inp = corpus[rng.rnd(0, corpus.size()-1)];

	// Get dst and src offsets and src length, making sure we don't exceed
	// max_input_size when inserting
	size_t dst_off = rand_offset(input, rng, true);
	size_t src_off = rand_offset(inp, rng);
	size_t max_src_len = max_input_size - input.size();
	size_t src_len = rng.rnd_exp(1, min(inp.size() - src_off, max_src_len));

	// Insert
	input.insert(dst_off, inp, src_off, src_len);
}
