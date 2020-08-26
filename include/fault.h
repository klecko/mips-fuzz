#ifndef _FAULT_H
#define _FAULT_H

#include <iostream>
#include "common.h"

// Fault: everything that will end the execution abruptly and will be considered
// a crash
struct Fault: public std::exception {
	enum Type: uint32_t {
		Read,
		Write,
		Exec,
		Uninit,
		OutOfBoundsRead,
		OutOfBoundsWrite,
		OutOfBoundsExec,
		MisalignedRead,
		MisalignedWrite,
		MisalignedExec,
		NoFault = 0xFFFFFFFF
	};

	Fault::Type type;
	vaddr_t     fault_addr;

	Fault(){}
	Fault(Fault::Type type, vaddr_t fault_addr):
		type(type), fault_addr(fault_addr) {}
	std::string type_str() const;

	friend std::ostream& operator<<(std::ostream& os, const Fault& f);
	bool operator==(const Fault& other) const { return type == other.type && fault_addr == other.fault_addr; }
};

inline std::string Fault::type_str() const {
	switch (type){
		case Fault::Type::Read:
			return "Read";
		case Fault::Type::Write:
			return "Write";
		case Fault::Type::Exec:
			return "Exec";
		case Fault::Type::Uninit:
			return "Uninit";
		case Fault::Type::OutOfBoundsRead:
			return "OutOfBoundsRead";
		case Fault::Type::OutOfBoundsWrite:
			return "OutOfBoundsWrite";
		case Fault::Type::OutOfBoundsExec:
			return "OutOfBoundsExec";
		case Fault::Type::MisalignedRead:
			return "MisalignedRead";
		case Fault::Type::MisalignedWrite:
			return "MisalignedWrite";
		case Fault::Type::MisalignedExec:
			return "MisalignedExec";
		default:
			return "Unimplemented (" + std::to_string(type) + ")";
	}
}

inline std::ostream& operator<<(std::ostream& os, const Fault& f){
	os << f.type_str() << " Fault, fault address = 0x" << std::hex
	   << f.fault_addr << std::dec;
	return os;
}

#endif