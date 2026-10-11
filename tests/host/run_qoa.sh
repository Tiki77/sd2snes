#!/usr/bin/env bash
# Conformance test for the streaming QOA decoder (src/qoa.c), compiled against the REAL
# source. qoa_cli.c checks it bit-exact against files encoded and decoded by the reference
# implementation (qoa.h, phoboslab/qoa), under every request size, plus truncated and
# corrupted input.
#
# src/qoa.c is deliberately dependency-free (no config.h, no ff.h), so like src/strutil.c
# it compiles straight from src/.
set -u
cd "$(dirname "$0")"
CC="${CC:-cc}"
. ./sanitizers.sh   # ASAN_OPTIONS/UBSAN_OPTIONS + san_report(); see the file
mkdir -p build

$CC -O1 -Wall -Wextra -fsanitize=address,undefined -I ../../src \
    qoa_cli.c ../../src/qoa.c -o build/qoa_cli || exit 1
exec ./build/qoa_cli
