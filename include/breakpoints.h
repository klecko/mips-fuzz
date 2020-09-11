#ifndef _BREAKPOINTS_H
#define _BREAKPOINTS_H

#include <unordered_map>
#include <vector>
#include "elf_parser.hpp"
#include "common.h"

class Emulator;
typedef void (Emulator::*bp_handler_t)();

struct breakpoint_t {
	std::string name;
	bp_handler_t handl;
	bool changes_pc;
};

class Breakpoints {
public:
	Breakpoints(vsize_t size, const std::vector<symbol_t>& symbols):
		bitmap(size), symbols(symbols) {}

	// Return constant references to data members. TODO: remove these and
	// add iterator
	const std::unordered_map<vaddr_t, breakpoint_t>& get_map() const {
		return map;
	}
	const std::vector<bool>& get_bitmap() const {
		return bitmap;
	}

	// Set a breakpoint at a given address
	void set_bp_addr(const std::string& name, vaddr_t addr,
	                 bp_handler_t handl, bool changes_pc)
	{
		assert(addr < bitmap.size());
		map[addr] = {name, handl, changes_pc};
		bitmap[addr] = true;
	}

	// Set abreakpoint at a given symbol
	void set_bp_sym (const std::string& sym_name, bp_handler_t handl,
	                 bool changes_pc)
	{
		// Resolve symbol and call set_bp_addr
		auto it = symbols.begin();
		while (it != symbols.end() && it->name != sym_name) ++it;
		if (it == symbols.end())
			die("Not found symbol %s\n", sym_name.c_str());

		set_bp_addr(sym_name + "_bp", it->value, handl, changes_pc);
		dbgprintf("set breakpoint %s at 0x%X\n", sym_name.c_str(), it->value);
	}

	// Check if there's a breakpoint at a given address
	bool has_bp(vaddr_t addr) const {
		assert(addr < bitmap.size());
		return bitmap[addr];
	}

	// Get a breakpoint handler
	bp_handler_t get_bp(vaddr_t addr) const {
		return (has_bp(addr) ? map.at(addr).handl : NULL);
	}

	// Get a breakpoint
	breakpoint_t get(vaddr_t addr) const {
		assert(has_bp(addr));
		return map.at(addr);
	}

private:
	std::unordered_map<vaddr_t, breakpoint_t> map;
	std::vector<bool> bitmap;
	std::vector<symbol_t> symbols;
};

#endif