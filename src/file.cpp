#include <iostream>
#include <fcntl.h> // O_* definitions
#include "file.h"

using namespace std;

File::File(uint32_t flags, char* buf, size_t size){
	this->flags = flags;
	this->buf = buf;
	this->size = size;
	this->offset = 0;
}

char* File::get_cursor(){
	return buf + offset;
}

int32_t File::move_cursor(int32_t increment){
	// Don't handle negatives for now
	if (increment < 0)
		die("negative move cursor\n");

	// Reduce increment if there is not enough space available
	vsize_t ret = (offset+increment < size ? increment : size-offset);

	// Update offset
	offset += ret;
	return ret;
}

bool File::is_readable(){
	return ((flags & O_ACCMODE) == O_RDONLY) || ((flags & O_ACCMODE) == O_RDWR);
}

bool File::is_writable(){
	return ((flags & O_ACCMODE) == O_WRONLY) || (flags == O_RDWR);
}

ostream& operator<<(ostream& os, const File& f){
	os << "flags: " << f.flags << ", offset: " << f.offset << ", size: "
	   << f.size << ", buf: " << hex << (void*)f.buf << dec << endl;
	return os;
}