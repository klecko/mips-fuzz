#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

void error(const char* msg){
	perror(msg);
	exit(EXIT_FAILURE);
}

int main(int argc, char** argv){
	if (argc < 2){
		fprintf(stderr, "usage: %s file\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	int fd = open(argv[1], O_RDONLY);
	if (fd == -1)
		error("open");

	char buf[256];
	read(fd, buf, sizeof(buf));
	printf("%s\n", buf);

	// This would be impossible without coverage
	if (buf[0] == 'A' && buf[1] == 'B' && buf[2] == 'C' && buf[3] == 'D' &&
	    buf[4] == 'E' && buf[5] == 'F')
			*(int*)(0x13371337) = 0x12345678;

	close(fd);
}
