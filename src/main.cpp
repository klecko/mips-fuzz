#include <iostream>
#include <mmu.h>
#include <common.h>

using namespace std;

int main(){
	Mmu mmu(32);
	addr_t addr = mmu.alloc(30);
	mmu.write(addr+8, 0x69);

	Mmu copy = mmu.fork();
	copy.write(addr, 5);
	copy.write(addr+8, 0x12);

	cout << mmu << endl;
	cout << copy << endl;

	copy.reset(mmu);
	cout << mmu << endl;
	cout << copy << endl;

	printf("%X", mmu.read<int>(addr));
	printf("%X", copy.read<int>(addr));
	
}