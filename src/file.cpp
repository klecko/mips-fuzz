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

int64_t File::move_cursor(int64_t increment){
	// Don't handle negatives for now
	if (increment < 0)
		die("negative move cursor\n");

	// Reduce increment if there is not enough space available
	int64_t ret = (offset+increment < size ? increment : size-offset);

	// Update offset
	offset += ret;
	return ret;
}

size_t File::get_offset(){
	return offset;
}

size_t File::set_offset(size_t new_offset){
	// This is legal, but disallow for now
	if (offset > size)
		die("seting offset past end\n");

	offset = new_offset;
	return offset;
}

size_t File::get_size(){
	return size;
}

void File::stat(guest_stat64& st){
	guest_stat_default(st, size);
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

void guest_stat_default(struct guest_stat64& st, size_t sz){
	st.st_dev        = 2052;
	st.st_ino        = 11349843;
	st.st_mode       = 0100664; // regular file with default permissions
	st.st_nlink      = 1;
	st.st_uid        = 0;
	st.st_gid        = 0;
	st.st_rdev       = 0;
	st.st_size       = sz;
	st.st_atime      = 0;
	st.st_atime_nsec = 0;
	st.st_mtime      = 0;
	st.st_mtime_nsec = 0;
	st.st_ctime      = 0;
	st.st_ctime_nsec = 0;
	st.st_blksize    = 4096;
	st.st_blocks     = (sz/512) + 1;
}

void guest_stat_stdout(struct guest_stat64& st){
	st.st_dev        = 22;
	st.st_ino        = 3;
	st.st_mode       = 020620; // stdout
	st.st_nlink      = 1;
	st.st_uid        = 0;
	st.st_gid        = 0;
	st.st_rdev       = 34815;
	st.st_size       = 0;
	st.st_atime      = 0;
	st.st_atime_nsec = 0;
	st.st_mtime      = 0;
	st.st_mtime_nsec = 0;
	st.st_ctime      = 0;
	st.st_ctime_nsec = 0;
	st.st_blksize    = 1024;
	st.st_blocks     = 0;
}