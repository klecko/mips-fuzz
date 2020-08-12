#ifndef _FAULT_H
#define _FAULT_H

#include <iostream>
#include "common.h"

// Fault: everything that will end the execution abruptly and will be considered
// a crash
class Fault : public std::exception {
	public:
		enum Type {
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
		};

	private:
		Fault::Type type;
		vaddr_t     fault_addr;

	public:
		Fault(Fault::Type type, vaddr_t fault_addr):
			type(type), fault_addr(fault_addr)
			{}

		friend std::ostream& operator<<(std::ostream& os, const Fault& f);
		bool operator==(const Fault& other) const { return type == other.type && fault_addr == other.fault_addr; }
};

inline std::ostream& operator<<(std::ostream& os, const Fault& f){
	switch (f.type){
		case Fault::Type::Read:
			os << "Read";
			break;
		case Fault::Type::Write:
			os << "Write";
			break;
		case Fault::Type::Exec:
			os << "Exec";
			break;
		case Fault::Type::Uninit:
			os << "Uninit";
			break;
		case Fault::Type::OutOfBoundsRead:
			os << "OutOfBoundsRead";
			break;
		case Fault::Type::OutOfBoundsWrite:
			os << "OutOfBoundsWrite";
			break;
		case Fault::Type::OutOfBoundsExec:
			os << "OutOfBoundsExec";
			break;
		case Fault::Type::MisalignedRead:
			os << "MisalignedRead";
			break;
		case Fault::Type::MisalignedWrite:
			os << "MisalignedWrite";
			break;
		case Fault::Type::MisalignedExec:
			os << "MisalignedExec";
			break;
		default:
			os << "UnimplementedFault";
	}
	os << " Fault, fault address = 0x" << std::hex << f.fault_addr << std::dec;
	return os;
}

#endif