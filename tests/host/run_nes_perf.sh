#!/usr/bin/env bash
# Gate of the NES renderer performance telemetry (snes/nes/nes_perf.a65).
#
# Runs the REAL routines (bytes of misc/nes_snes.bin) under the 65816 interpreter
# (m65816.c) and compares every observable with a model written from the spec in
# utils/nes_perf_decode.py: scanline clock, decode accumulators, NMI entry bins,
# NMI end, the 48-slot stream (through the host assembler), NMI preemption of a
# sample and the cycle budget.  See nes_perf_gate.py.
#
# The binary under test is the MEASUREMENT build, `make -C snes/nes perf` ->
# misc/nes_snes_perf.{bin,map}; the release binary (misc/nes_snes.bin) carries no
# telemetry at all (the gate refuses it: no nes_pf_* symbols).
#
# Two kinds of check.  The routines are driven one by one against a model written
# from the spec (clock, decode, entry, NMI end, 48-slot stream, preemption, freeze
# saturation, cycle budget).  Then the INTEGRATION run (`sys`): the real main loop
# and the real NMI run under the interpreter clock against a bridge model, and the
# counters the renderer keeps must equal the ones the ground truth of the run
# implies -- which is what proves the probes sit at their call sites.
#
# SKIP: the measurement .bin is not versioned and the release build does not make it
# (`make -C snes/nes perf` does), so a tree without it skips and exits 0.
# NES_PERF_REQUIRED=1 turns the skip into a failure (NES_BIN_REQUIRED, which guards the
# release binary of run_nes_chr.sh, does not apply here).
set -u
cd "$(dirname "$0")"
CC="${CC:-cc}"
PY="${PYTHON:-python3}"
REPO_ROOT="$(cd ../.. && pwd)"
NES_BIN="${NES_SNES_PERF_BIN:-$REPO_ROOT/misc/nes_snes_perf.bin}"
NES_MAP="${NES_SNES_PERF_MAP:-${NES_BIN%.bin}.map}"
BUILD="${NES_PERF_BUILD:-build}"      # separate dir per run when several gates run at once
mkdir -p "$BUILD"
skip() {
  if [ "${NES_PERF_REQUIRED:-0}" != "0" ]; then echo "FAIL: $1 (NES_PERF_REQUIRED is set)" >&2; exit 1; fi
  echo "SKIP  nes_perf: $1"; exit 0
}
[ -f "$NES_BIN" ] || skip "$NES_BIN missing"
[ -f "$NES_MAP" ] || skip "$NES_MAP missing"
grep -q ' nes_pf_nmi_end$' "$NES_MAP" || skip "$NES_BIN is not a measurement build (make -C snes/nes perf)"
$CC -O1 -g -fsanitize=address,undefined -I. -Ishim nes_perf_cli.c m65816.c -o "$BUILD/nes_perf_cli" || exit 1
. ./sanitizers.sh
$PY nes_perf_gate.py "$BUILD/nes_perf_cli" "$NES_BIN" "$NES_MAP"
