#include <iostream>
#include <fcntl.h> // O_* definitions
#include "file.h"

using namespace std;

#define stat_regular_file_common \
	st.st_dev        = 2052;     \
	st.st_ino        = 11349843; \
	st.st_mode       = 0100664;  \
	st.st_nlink      = 1;        \
	st.st_uid        = 0;        \
	st.st_gid        = 0;        \
	st.st_rdev       = 0;        \
	st.st_size       = size;     \
	st.st_atime      = 0;        \
	st.st_atime_nsec = 0;        \
	st.st_mtime      = 0;        \
	st.st_mtime_nsec = 0;        \
	st.st_ctime      = 0;        \
	st.st_ctime_nsec = 0;        \
	st.st_blksize    = 4096;     \
	st.st_blocks     = (size/512) + 1;

#define stat_stdout_file_common \
	st.st_dev        = 22;      \
	st.st_ino        = 3;       \
	st.st_mode       = 020620;  \
	st.st_nlink      = 1;       \
	st.st_uid        = 0;       \
	st.st_gid        = 0;       \
	st.st_rdev       = 34815;   \
	st.st_size       = 0;       \
	st.st_atime      = 0;       \
	st.st_atime_nsec = 0;       \
	st.st_mtime      = 0;       \
	st.st_mtime_nsec = 0;       \
	st.st_ctime      = 0;       \
	st.st_ctime_nsec = 0;       \
	st.st_blksize    = 1024;    \
	st.st_blocks     = 0;

#define stat_random_file_common \
	st.st_dev        = 6; \
	st.st_ino        = 10; \
	st.st_mode       = 020666; \
	st.st_nlink      = 1; \
	st.st_uid        = 0; \
	st.st_gid        = 0; \
	st.st_rdev       = 264; \
	st.st_size       = 0; \
	st.st_atime      = 0; \
	st.st_atime_nsec = 0; \
	st.st_mtime      = 0; \
	st.st_mtime_nsec = 0; \
	st.st_ctime      = 0; \
	st.st_ctime_nsec = 0; \
	st.st_blksize    = 4096; \
	st.st_blocks     = 0;

#define stat_urandom_file_common \
	st.st_dev        = 6; \
	st.st_ino        = 11; \
	st.st_mode       = 020666; \
	st.st_nlink      = 1; \
	st.st_uid        = 0; \
	st.st_gid        = 0; \
	st.st_rdev       = 265; \
	st.st_size       = 0; \
	st.st_atime      = 0; \
	st.st_atime_nsec = 0; \
	st.st_mtime      = 0; \
	st.st_mtime_nsec = 0; \
	st.st_ctime      = 0; \
	st.st_ctime_nsec = 0; \
	st.st_blksize    = 4096; \
	st.st_blocks     = 0;

void stat_regular_file(guest_stat& st, vsize_t size){
	stat_regular_file_common
}

void stat_stdout_file(guest_stat& st){
	stat_stdout_file_common
}

void stat_random_file(guest_stat& st){
	stat_random_file_common
}

void stat_urandom_file(guest_stat& st){
	stat_urandom_file_common
}

void stat64_regular_file(guest_stat64& st, vsize_t size){
	stat_regular_file_common
}

void stat64_stdout_file(guest_stat64& st){
	stat_stdout_file_common
}

void stat64_random_file(guest_stat64& st){
	stat_random_file_common
}

void stat64_urandom_file(guest_stat64& st){
	stat_urandom_file_common
}

file_ops fops_regular = {
	&File::stat_regular,
	&File::stat64_regular,
	&File::read_regular,
	&File::write_forbidden
};

file_ops fops_stdin = {
	&File::stat_unimplemented,
	&File::stat64_unimplemented,
	&File::read_forbidden,
	&File::write_forbidden
};

file_ops fops_stdout = {
	&File::stat_stdout,
	&File::stat64_stdout,
	&File::read_forbidden,
	&File::write_stdout
};

file_ops fops_stderr = {
	&File::stat_unimplemented,
	&File::stat64_unimplemented,
	&File::read_forbidden,
	&File::write_stdout
};

file_ops fops_random = {
	&File::stat_random,
	&File::stat64_random,
	&File::read_random,
	&File::write_forbidden
};

file_ops fops_urandom = {
	&File::stat_urandom,
	&File::stat64_urandom,
	&File::read_random,
	&File::write_forbidden
};

File::File(uint32_t flags, char* buf, size_t size):
	fops(fops_regular), flags(flags), buf(buf), size(size), offset(0){};

char* File::get_cursor(){
	return buf + offset;
}

int64_t File::move_cursor(int64_t increment){
	// Don't handle negatives for now
	if (increment < 0)
		die("negative move cursor\n");

	if (offset >= size){
		// Offset is currently past end.
		return 0;
	}

	// Reduce increment if there is not enough space available
	int64_t ret = (offset+increment < size ? increment : size-offset);

	// Update offset
	offset += ret;
	return ret;
}


size_t File::get_offset(){
	return offset;
}

void File::set_offset(size_t new_offset){
	if (offset < 0)
		die("attempt to set file negative offset\n");
	offset = new_offset;
}

size_t File::get_size(){
	return size;
}

bool File::is_readable(){
	return ((flags & O_ACCMODE) == O_RDONLY) || ((flags & O_ACCMODE) == O_RDWR);
}

bool File::is_writable(){
	return ((flags & O_ACCMODE) == O_WRONLY) || (flags == O_RDWR);
}


void File::stat_unimplemented(guest_stat& st){
	die("unimplemented fstat\n");
}

void File::stat_regular(guest_stat& st){
	stat_regular_file(st, size);
}

void File::stat_stdout(guest_stat& st){
	stat_stdout_file(st);
}

void File::stat_random(guest_stat& st){
	stat_random_file(st);
}

void File::stat_urandom(guest_stat& st){
	stat_urandom_file(st);
}

void File::stat64_unimplemented(guest_stat64& st){
	die("unimplemented fstat\n");
}

void File::stat64_regular(guest_stat64& st){
	stat64_regular_file(st, size);
}

void File::stat64_stdout(guest_stat64& st){
	stat64_stdout_file(st);
}

void File::stat64_random(guest_stat64& st){
	stat64_random_file(st);
}

void File::stat64_urandom(guest_stat64& st){
	stat64_urandom_file(st);
}

uint32_t File::read_forbidden(vaddr_t buf_addr, vsize_t len, Mmu& mmu){
	die("forbidden read\n");
}

uint32_t File::read_regular(vaddr_t buf_addr, vsize_t len, Mmu& mmu){
	// Check if file is readable
	if (!is_readable())
		die("reading from non readable fd\n");

	// Read from file (write into memory)
	char* cursor = get_cursor();
	vsize_t real_len = move_cursor(len);
	mmu.write_mem(buf_addr, cursor, real_len);

	return real_len;
}

uint32_t File::read_random(vaddr_t buf_addr, vsize_t len, Mmu& mmu){
	for (vsize_t i = 0; i < len; i++)
		mmu.write<uint8_t>(buf_addr + i, i & 0xFF);
	return len;
}

uint32_t File::write_forbidden(vaddr_t buf_addr, vsize_t len, Mmu& mmu,
                               bool output)
{
	die("forbidden write\n");
}

uint32_t File::write_stdout(vaddr_t buf_addr, vsize_t len, Mmu& mmu,
                            bool output)
{
	char buf[len + 1];
	mmu.read_mem(buf, buf_addr, len);
	buf[len] = 0;
	if (output)
		printf("%s", buf);
	return len;
}

void File::stat(guest_stat& st){
	(this->*fops.stat)(st);
}

void File::stat64(guest_stat64& st){
	(this->*fops.stat64)(st);
}

uint32_t File::read(vaddr_t buf_addr, vsize_t len, Mmu& mmu){
	return (this->*fops.read)(buf_addr, len, mmu);
}

uint32_t File::write(vaddr_t buf_addr, vsize_t len, Mmu& mmu, bool output){
	return (this->*fops.write)(buf_addr, len, mmu, output);
}

ostream& operator<<(ostream& os, const File& f){
	os << "flags: " << f.flags << ", offset: " << f.offset << ", size: "
	   << f.size << ", buf: " << hex << (void*)f.buf << dec << endl;
	return os;
}

FileStdin::FileStdin() : File(O_RDONLY) { fops = fops_stdin; };

FileStdout::FileStdout() : File(O_WRONLY) { fops = fops_stdout; };

FileStderr::FileStderr() : File(O_WRONLY) { fops = fops_stdout; };

FileRandom::FileRandom() : File(O_RDONLY) { fops = fops_random; };

FileUrandom::FileUrandom() : File(O_RDONLY) { fops = fops_urandom; };