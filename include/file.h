#ifndef _FILE_H
#define _FILE_H

#include <iostream>
#include <stdint.h>
#include "common.h"

class File {
	private:
		uint32_t fd;
		uint32_t flags;
		char*    buf;
		size_t   size;
		size_t   offset;

	public:
		File(uint32_t fd = 0, uint32_t flags = 0, char* buf = NULL, 
		     size_t size = 0);

		// Get file cursor
		char*   get_cursor();

		// Attempt to move the cursor. Returns the real increment performed
		int32_t move_cursor(int32_t increment);

		// Flags
		bool    is_readable();
		bool    is_writable();

		friend std::ostream& operator<<(std::ostream& os, const File& emu);
};

#endif