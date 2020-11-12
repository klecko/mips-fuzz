#ifndef _GUEST_H
#define _GUEST_H

#include <cstdint>
#include "common.h"

#define G_MAP_SHARED          0x01
#define G_MAP_PRIVATE         0x02
#define G_MAP_SHARED_VALIDATE 0x03
#define G_MAP_TYPE            0x0F
#define G_MAP_FIXED           0x10
#define G_MAP_ANONYMOUS       0x0800
#define G_MAP_ANON            G_MAP_ANONYMOUS


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

struct guest_stat {
	uint32_t st_dev;
	uint32_t st_pad1[3];

	uint32_t st_ino;

	uint32_t st_mode;
	uint32_t st_nlink;
	uint32_t st_uid;
	uint32_t st_gid;
	uint32_t st_rdev;
	uint32_t st_pad2[2];
	uint32_t st_size;
	uint32_t st_pad3;

	uint32_t st_atime;
	uint32_t st_atime_nsec;
	uint32_t st_mtime;
	uint32_t st_mtime_nsec;
	uint32_t st_ctime;
	uint32_t st_ctime_nsec;

	uint32_t st_blksize;
	uint32_t st_blocks;

	uint32_t st_pad5[14];
};

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

	uint32_t st_pad4[14];
};

struct guest_pollfd {
	int   fd;
	short events;
	short revents;
};

// /usr/mipsel-linux-gnu/include/linux/sysinfo.h
struct guest_sysinfo {
	int32_t uptime;             /* Seconds since boot */
	uint32_t loads[3];  /* 1, 5, and 15 minute load averages */
	uint32_t totalram;  /* Total usable main memory size */
	uint32_t freeram;   /* Available memory size */
	uint32_t sharedram; /* Amount of shared memory */
	uint32_t bufferram; /* Memory used by buffers */
	uint32_t totalswap; /* Total swap space size */
	uint32_t freeswap;  /* Swap space still available */
	uint16_t procs;    /* Number of current processes */
	char _f[22];             /* Pads structure to 64 bytes */
};

inline struct guest_sysinfo default_guest_sysinfo(){
	struct guest_sysinfo s = {
		28738,                 // uptime
		{47744, 50240, 53984}, // loads
		3961286656,            // totalram
		832942080,             // freeram
		278626304,             // sharedram
		286662656,             // bufferram
		4294959104,            // totalswap
		4287619072,            // freeswap
		1257,                  // procs
		0,                     // high
		0,                     // free
		1                      // mem_unit
	};
	return s;
}

#endif