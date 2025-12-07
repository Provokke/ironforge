#!/usr/bin/env bash
# Runs the full test suite under ASan+UBSan (with leak checking) and under TSan.
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build-asan -G Ninja -DCMAKE_BUILD_TYPE=Debug -DIFG_SANITIZE=address,undefined
cmake --build build-asan -j2 --target ironforge_tests
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 ./build-asan/ironforge_tests
cmake -S . -B build-tsan -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DIFG_SANITIZE=thread
cmake --build build-tsan -j2 --target ironforge_tests
TSAN_OPTIONS=halt_on_error=1 ./build-tsan/ironforge_tests
