# mips-fuzz

This project has the goal of fuzzing closed-source MIPS binaries, and it is greatly inspired by [gamozolabs fuzz week](https://gamozolabs.github.io/2020/07/12/fuzz_week_2020.html). I've done it with the purpose of learning about emulation and fuzzing. It is mostly useless as MIPS binaries have very low interest, but the same could be done for x86 (just with a lot more work)

## What
It emulates MIPS32 architecture, having full control over what's being executed and its environment. This allows snapshot fuzzing, code coverage, breakpoints and a custom dynamic memory allocator with byte-level permission checks to detect memory corruption bugs.

This was thought to be fully scalable. No IPC, no bottlenecking in kernel (kernel time is under 1%, no syscalls per fuzzcase) and almost no locks. It also should be thread-safe.

Regarding the emulation, there's an interpreter and a LLVM-based JIT. The interpreter is quite slow. The JIT has some very slow runs where it compiles code, but following runs are very fast. By default fuzz cases are run with the JIT because of its speed, although it depends on the interpreter for crash reporting. The interpreter itself is better for crash reproduction, tracing, or when you don't want to wait for the JIT to compile all the code.

## Performance
The most used fuzzer for closed-source binaries is probably AFL++ in QEMU mode. I've done some tests, each of them lasting two minutes and measuring **fuzz cases per second** (as AFL++ doesn't provide million instructions per second). Both mips-fuzz and AFL++ have the same binary, initial corpus and [reset address](https://github.com/AFLplusplus/AFLplusplus/tree/stable/qemu_mode#3-bonus-feature-1-deferred-initialization). Fuzzed binaries were `readelf`, `stegdetect`, and a simple and small `test` binary.


#### readelf
|               | 1 core | 8 cores |
|---------------|--------|---------|
| **mips-fuzz** |   527  |   2468  |
| **AFL++**     |   361  |   1950  |

#### stegdetect
|               | 1 core | 8 cores |
|---------------|--------|---------|
| **mips-fuzz** |   450  |   1930  |
| **AFL++**     |    90  |    720  |

#### test
|               | 1 core | 8 cores |
|---------------|--------|---------|
| **mips-fuzz** |   61k  |   305k  |
| **AFL++**     |  1.8k  |   8.7k  |

The comparison is not completely fair because of a lot of factors, such as different memory permission checks, different mutators, scalability. In general mips-fuzz seems to be faster, and much more when fuzzing small binaries, where AFL++ bottlenecks in `fork`.