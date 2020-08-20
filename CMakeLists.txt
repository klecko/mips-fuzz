cmake_minimum_required(VERSION 3.10)
set (CMAKE_CXX_STANDARD 11)

project(my_fuzzer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
set(SOURCE_FILES src/main.cpp src/mmu.cpp src/elf_parser.cpp src/emulator.cpp
                 src/corpus.cpp src/file.cpp src/instr.cpp)
include_directories(include)

add_executable(my_fuzzer ${SOURCE_FILES})
