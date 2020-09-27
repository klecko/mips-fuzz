cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

if (ADVISOR)
    message("Set to compile for Intel Advisor")
    set(CMAKE_CXX_COMPILER icpc)
else()
    set(CMAKE_CXX_COMPILER g++)
endif()

project(mips-fuzz)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -msse4 \
                     -I/usr/lib/llvm-9/include -D_GNU_SOURCE \
                     -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS \
                     -D__STDC_LIMIT_MACROS")
if (ADVISOR)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -g \
                         -qopt-report=5 -simd -qopenmp")
endif()

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
set(SOURCE_FILES src/main.cpp src/mmu.cpp src/elf_parser.cpp src/emulator.cpp
                 src/corpus.cpp src/file.cpp src/instr.cpp src/jitter.cpp
                 src/jitter_instr.cpp)
include_directories(include)

add_executable(mips-fuzz ${SOURCE_FILES})
target_link_libraries(mips-fuzz "LLVM-9")
