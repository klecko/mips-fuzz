cmake_minimum_required(VERSION 3.10)

project(my_fuzzer)

set(SOURCE_FILES src/main.cpp src/mmu.cpp src/elf_parser.cpp src/emulator.cpp)
include_directories(include)

add_executable(my_fuzzer ${SOURCE_FILES})
