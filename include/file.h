#ifndef _FILE_H
#define _FILE_H

#include <iostream>
#include <stdint.h>
#include "common.h"
#include "guest.h"
#include "mmu.h"

class File;
struct file_ops {
	void (File::*stat)(guest_stat& st);
	void (File::*stat64)(guest_stat64& st);
	uint32_t (File::*read)(vaddr_t buf_addr, vsize_t len, Mmu& mmu);
	uint32_t (File::*write)(vaddr_t buf_addr, vsize_t len, Mmu& mmu, bool output);
};

// Interface for stat. Fstat should use the corresponding methods in File class
void stat_regular_file(guest_stat& st, vsize_t size);
void stat_stdout_file(guest_stat& st);
void stat_random_file(guest_stat& st);
void stat_urandom_file(guest_stat& st);
void stat64_regular_file(guest_stat64& st, vsize_t size);
void stat64_stdout_file(guest_stat64& st);
void stat64_random_file(guest_stat64& st);
void stat64_urandom_file(guest_stat64& st);

class File {
public:
	File(uint32_t flags = 0, char* buf = NULL, size_t size = 0);

	// Get file cursor
	char*   get_cursor();

	// Attempt to move the cursor. Returns the real increment performed
	int64_t move_cursor(int64_t increment);

	// Get and set offset. It can be set past `size`
	size_t  get_offset();
	void    set_offset(size_t new_offset);

	// Get size
	size_t  get_size();

	// Flags
	bool    is_readable();
	bool    is_writable();

	// Stat, read and write
	void stat_unimplemented(guest_stat& st);
	void stat_regular(guest_stat& st);
	void stat_stdout(guest_stat& st);
	void stat_random(guest_stat& st);
	void stat_urandom(guest_stat& st);
	void stat64_unimplemented(guest_stat64& st);
	void stat64_regular(guest_stat64& st);
	void stat64_stdout(guest_stat64& st);
	void stat64_random(guest_stat64& st);
	void stat64_urandom(guest_stat64& st);
	uint32_t read_forbidden(vaddr_t buf_addr, vsize_t len, Mmu& mmu);
	uint32_t read_regular(vaddr_t buf_addr, vsize_t len, Mmu& mmu);
	uint32_t read_random(vaddr_t buf_addr, vsize_t len, Mmu& mmu);
	uint32_t write_forbidden(vaddr_t buf_addr, vsize_t len, Mmu& mmu, bool output);
	uint32_t write_stdout(vaddr_t buf_addr, vsize_t len, Mmu& mmu, bool output);

	void stat(guest_stat& st);
	void stat64(guest_stat64& st);
	uint32_t read (vaddr_t buf_addr, vsize_t len, Mmu& mmu);
	uint32_t write(vaddr_t buf_addr, vsize_t len, Mmu& mmu, bool output);

	friend std::ostream& operator<<(std::ostream& os, const File& emu);

protected:
	// Modified by inherited classes. I've done it this way to achieve
	// polymorphism without dynamic memory allocations
	file_ops fops;

private:
	// Flags used when opened
	uint32_t flags;

	// Pointer to file content
	char*    buf;

	// File size
	size_t   size;

	// Cursor offset
	size_t   offset;
};

class FileStdin : public File {
public:
	FileStdin();
};

class FileStdout : public File {
public:
	FileStdout();
};

class FileStderr : public File {
public:
	FileStderr();
};

class FileRandom : public File {
public:
	FileRandom();
};

class FileUrandom : public File {
public:
	FileUrandom();
};

#endif