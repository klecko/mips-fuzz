// MIT License

// Copyright (c) 2018 finixbit

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef H_ELF_PARSER
#define H_ELF_PARSER

#include <iostream>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>    /* O_RDONLY */
#include <sys/stat.h> /* For the size of the file. , fstat */
#include <sys/mman.h> /* mmap, MAP_PRIVATE */
#include <vector>
#include <elf.h>      // Elf64_Shdr
#include <fcntl.h>
#include "common.h"

#define BITS_32

#ifdef BITS_32 // 32 bits
#define Elf_Ehdr Elf32_Ehdr
#define Elf_Shdr Elf32_Shdr
#define Elf_Phdr Elf32_Phdr
#define Elf_Sym  Elf32_Sym
#define Elf_Rela Elf32_Rela
#define Elf32_Word Elf32_Word
#define Elf32_Half Elf32_Half
#define Elf32_Off  Elf32_Off
#define ELF_ST_TYPE ELF32_ST_TYPE
#define ELF_ST_BIND ELF32_ST_BIND
#define ELF_ST_VISIBILITY ELF32_ST_VISIBILITY
#define ELF_R_TYPE ELF32_R_TYPE
#define ELF_R_SYM ELF32_R_SYM
#define ELFCLASS ELFCLASS32
#endif

struct phinfo_t {
    vaddr_t  e_phoff;      /* Program header table file offset */
    uint16_t e_phentsize;  /* Program header table entry size */
    uint16_t e_phnum;      /* Program header table entry count */
};

struct section_t {
    uint16_t    index; 
    vaddr_t     offset;
    vaddr_t     addr;
    std::string name;
    std::string type; 
    vsize_t     size;
    vsize_t     ent_size; 
    uint32_t    addr_align;
};

struct segment_t {
    std::string type;
    std::string flags;
    vaddr_t     offset;
    vaddr_t     virtaddr;
    vaddr_t     physaddr;
    vsize_t     filesize;
    vsize_t     memsize;
    uint32_t    align;
    void*       data;
};

struct symbol_t {
    std::string index;
    vaddr_t     value;
    uint32_t    num;
    vsize_t     size;
    std::string type;
    std::string bind;
    std::string visibility;
    std::string name;
    std::string section;      
};

struct relocation_t {
    vaddr_t     offset;
    uint32_t    info;
    vaddr_t     symbol_value;
    std::string type;
    std::string symbol_name;
    std::string section_name;
    vaddr_t     plt_address;
};


class Elf_parser {
    public:
        Elf_parser (const std::string &elf_path);
        long get_entry();
        std::string get_path();
        std::string get_abs_path();
        phinfo_t get_phinfo();
        std::vector<section_t> get_sections();
        std::vector<segment_t> get_segments(Elf32_Addr& load_addr);
        std::vector<symbol_t> get_symbols();
        std::vector<relocation_t> get_relocations();
        uint8_t *get_memory_map();
        
    private:
        void load_memory_map();

        std::string get_section_type(uint32_t tt);

        std::string get_segment_type(Elf32_Word seg_type);
        std::string get_segment_flags(Elf32_Word seg_flags);

        std::string get_symbol_type(uint8_t sym_type);
        std::string get_symbol_bind(uint8_t sym_bind);
        std::string get_symbol_visibility(uint8_t sym_vis);
        std::string get_symbol_index(Elf32_Half sym_idx);

        std::string get_relocation_type(Elf32_Word rela_type);
        vaddr_t     get_rel_symbol_value(Elf32_Word sym_idx,
                                         const std::vector<symbol_t>& syms);
        std::string get_rel_symbol_name(Elf32_Word sym_idx,
                                        const std::vector<symbol_t>& syms);

        std::string m_elf_path;
        std::string m_elf_abs_path;
        uint8_t*    m_mmap_program;
};

#endif