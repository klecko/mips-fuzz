#ifndef _COMMON_H
#define _COMMON_H

#define die(...)                       \
	do {                               \
		fprintf(stderr, __VA_ARGS__);  \
		exit(EXIT_FAILURE);            \
	} while (0)

#endif