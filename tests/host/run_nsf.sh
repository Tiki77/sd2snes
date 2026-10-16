#!/usr/bin/env bash
# Host gate for the NSF player core (src/nsf.c + src/nsfapu.c), compiled from
# the REAL sources with ASan/UBSan.
#
#   1. tone: tiny NSFs built by the test hold one APU channel on a fixed period;
#      the output frequency has to match the period formula.  Always runs.
#   2. CPU: nestest in automation mode, every instruction compared with the
#      log (registers + cycle count, documented and undocumented opcodes).
#   3. APU: blargg's apu_test singles (length counters, frame IRQ flag and its
#      timing, DMC), result read from $6000.
#   4. corpus: every song of every .nsf renders 20 s without an error, and
#      the same as the build that steps the DEX/BNE delay loops one
#      instruction at a time (-DNSF_NO_IDLE_SKIP).  So does a synthetic sample
#      player (nsf_cli loops), whose PLAY never returns: it must play, while a
#      PLAY stuck in a JMP to itself must stop as hung.  Always runs.
#
# 2-4 need files that are not part of this repository: NES_TESTS (a directory
# holding nestest.nes, nestest.log and nes-test-roms/apu_test/rom_singles/) and
# NSF_CORPUS (a directory of .nsf).  Missing ones are SKIPPED unless
# NSF_TESTS_REQUIRED=1, which turns a skip into a failure.
set -u
cd "$(dirname "$0")"
CC="${CC:-cc}"
. ./sanitizers.sh
mkdir -p build
NES_TESTS="${NES_TESTS:-../../../nes/tests}"
NSF_CORPUS="${NSF_CORPUS:-../../../nsf/tests/corpus}"
REQ="${NSF_TESTS_REQUIRED:-0}"

$CC -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -I ../../src \
    nsf_cli.c ../../src/nsf.c ../../src/nsfapu.c -o build/nsf_cli || exit 1
$CC -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -DNSF_NO_IDLE_SKIP -I ../../src \
    nsf_cli.c ../../src/nsf.c ../../src/nsfapu.c -o build/nsf_cli_step || exit 1

fail=0
skip() { echo "SKIP: $1"; [ "$REQ" = 1 ] && fail=1; }

./build/nsf_cli tone || fail=1

a="$(./build/nsf_cli loops)" || fail=1
b="$(./build/nsf_cli_step loops)" || fail=1
echo "$a"
[ "$a" = "$b" ] || { echo "FAIL: loops differ from the stepped build: $b"; fail=1; }

if [ -f "$NES_TESTS/nestest.nes" ] && [ -f "$NES_TESTS/nestest.log" ]; then
  ./build/nsf_cli nestest "$NES_TESTS/nestest.nes" "$NES_TESTS/nestest.log" || fail=1
else
  skip "nestest ($NES_TESTS)"
fi

apu="$NES_TESTS/nes-test-roms/apu_test/rom_singles"
if ls "$apu"/*.nes >/dev/null 2>&1; then
  for t in "$apu"/*.nes; do
    out="$(./build/nsf_cli apu "$t")"; rc=$?
    echo "$out" | head -1
    [ $rc -eq 0 ] || { echo "$out"; fail=1; }
  done
else
  skip "apu_test ($apu)"
fi

if ls "$NSF_CORPUS"/*.nsf >/dev/null 2>&1; then
  n=0
  for f in "$NSF_CORPUS"/*.nsf; do
    songs="$(./build/nsf_cli info "$f" | sed -n 's/^songs \([0-9]*\).*/\1/p')"
    [ -n "$songs" ] || { echo "FAIL: $f does not parse"; fail=1; continue; }
    for s in $(seq 1 "$songs"); do
      ./build/nsf_cli render "$f" "$s" 20 build/nsf_a.wav >/dev/null || { echo "FAIL: $(basename "$f") song $s"; fail=1; }
      ./build/nsf_cli_step render "$f" "$s" 20 build/nsf_b.wav >/dev/null
      cmp -s build/nsf_a.wav build/nsf_b.wav || { echo "FAIL: $(basename "$f") song $s differs from the stepped build"; fail=1; }
      n=$((n + 1))
    done
  done
  echo "corpus: $n songs rendered"
else
  skip "NSF corpus ($NSF_CORPUS)"
fi

[ $fail -eq 0 ] && echo "run_nsf: OK" || echo "run_nsf: FAILED"
exit $fail
