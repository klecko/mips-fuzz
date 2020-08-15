#ifndef _FILE_H
#define _FILE_H

#include <iostream>
#include <stdint.h>
#include "common.h"
#include "guest.h"

// Fills `st` as a regular file with a given size
void guest_stat_default(struct guest_stat64& st, size_t sz);

void guest_stat_stdout(struct guest_stat64& st);


class File {
	private:
		// Flags used when opened
		uint32_t flags;

		// Pointer to file content
		char*    buf;

		// File size
		size_t   size;

		// Cursor offset
		size_t   offset;

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

		// Stat
		void    stat(guest_stat64& st);

		// Flags
		bool    is_readable();
		bool    is_writable();

		friend std::ostream& operator<<(std::ostream& os, const File& emu);
};

#endif