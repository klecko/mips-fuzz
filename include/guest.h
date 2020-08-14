#ifndef _GUEST_H
#define _GUEST_H

#include <cstdint>
#include "common.h"

struct guest_iovec {
	vaddr_t iov_base;
	vsize_t iov_len;
};

struct guest_uname {
	char sysname[65];
	char nodename[65];
	char release[65];
	char version[65];
	char machine[65];
	//char domainname[65];
};

// /usr/mipsel-linux-gnu/include/bits/stat.h
#undef st_atime
#undef st_mtime
#undef st_ctime

struct guest_stat64 {
    uint32_t st_dev;
    uint32_t st_pad0[3]; /* Reserved for st_dev expansion  */

    uint64_t st_ino;

    uint32_t st_mode;
    uint32_t st_nlink;

    uint32_t st_uid;
    uint32_t st_gid;

    uint32_t st_rdev;
    uint32_t st_pad1[3]; /* Reserved for st_rdev expansion  */

    int64_t  st_size;

    /*
     * Actually this should be timestruc_t st_atime, st_mtime and st_ctime
     * but we don't have it under Linux.
     */
    int32_t  st_atime;
    uint32_t st_atime_nsec;  /* Reserved for st_atime expansion  */

    int32_t  st_mtime;
    uint32_t st_mtime_nsec;  /* Reserved for st_mtime expansion  */

    int32_t  st_ctime;
    uint32_t st_ctime_nsec;  /* Reserved for st_ctime expansion  */

    uint32_t st_blksize;
    uint32_t st_pad2;

    int64_t  st_blocks;
};

#endif