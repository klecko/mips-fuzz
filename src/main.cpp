#include <iostream>
#include <mmu.h>
#include <common.h>

using namespace std;

int main(){
	Mmu mmu(1024 * 1024 * 1024);
	
	try {
		mmu.load_elf("../test_bins/xxd/xxd");
	} catch (const Fault& f) {
		cout << f << endl;
	}
}