cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_COMPILER g++)

project(my_fuzzer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall \
                     -I/usr/lib/llvm-9/include -D_GNU_SOURCE \
                     -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS \
                     -D__STDC_LIMIT_MACROS")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
set(SOURCE_FILES src/main.cpp src/mmu.cpp src/elf_parser.cpp src/emulator.cpp
                 src/corpus.cpp src/file.cpp src/instr.cpp src/jitter.cpp)
include_directories(include)

add_executable(my_fuzzer ${SOURCE_FILES})
target_link_libraries(my_fuzzer "LLVM-9")
