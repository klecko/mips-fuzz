#define _GNU_SOURCE
#include <sys/mman.h>
#include <stdio.h>

int main(int argc){
	char* p = mmap(NULL, 0x1000, PROT_WRITE|PROT_EXEC, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
	*p = 0xC3;
	if (mprotect(p, 0x1000, PROT_EXEC) == -1)
		printf("error\n");
	
	((void (*)())p)();
	printf("%p\n", p);
}