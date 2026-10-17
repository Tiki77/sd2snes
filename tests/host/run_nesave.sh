#!/usr/bin/env bash
# Conformance test for the NES battery (src/nesave.c), compiled against the REAL source over a fake
# PSRAM and a fake card.  Copies go into build/nesave/ so the shim headers win over the REAL firmware
# headers that sit next to src/nesave.c (quoted #includes search the including file's directory first).
#
# NESAVE_MUTANT=<name> applies one deliberate defect to the copy; the test must FAIL on every one of
# them (run_nesave_mutants.sh does the loop).  Without it this is the plain gate.
set -u
cd "$(dirname "$0")"
CC="${CC:-cc}"
. ./sanitizers.sh
B="${NESAVE_BUILD:-build/nesave}"      # the mutant loop uses its own dir: both may run at once
mkdir -p "$B"
find "$B" -type f -delete

cp ../../src/nesave.c "$B/nesave_under_test.c" || exit 1
cp ../../src/nesave.h "$B/nesave.h"            || exit 1

M="$B/nesave_under_test.c"
case "${NESAVE_MUTANT:-}" in
  "") ;;
  nocap)       sed -i.bak 's/while(done < max) {/while(1) {/; s/if(got > max - done) got = (UINT)(max - done);//' "$M" ;;
  nodetach)    sed -i.bak '/^void nesave_detach/,/^}/ s/sv_size = 0;//' "$M" ;;
  noflushdiff) sed -i.bak 's/if(crc != sv_crc_old || sv_diff)/if(crc != sv_crc_old)/' "$M" ;;
  noflush)     sed -i.bak '/^void nesave_flush_inreset/,/^}/ s/if(!sv_size) return;/return;/' "$M" ;;
  nobaseline)  sed -i.bak 's/  if(!sv_hw_err) sv_crc_old = sv_scan(NESAVE_WINDOW_ADDR, sv_size, 0, 0);//' "$M" ;;
  nostable)    sed -i.bak 's/sv_same >= NESAVE_STABLE_PASSES/1/' "$M" ;;
  noreliable)  sed -i.bak 's/if(!sv_reliable() \&\& !sv_hw_err) sv_bail = 1;//' "$M" ;;
  noratelimit) sed -i.bak 's/if((int32_t)(now - sv_next) < 0) return;//' "$M" ;;
  fullsize)    sed -i.bak 's/if(is_nes20 \&\& (flags10 >> 4)) {/if(0) {/' "$M" ;;
  nolastchunk) sed -i.bak 's/if(n > NESAVE_CHUNK) n = NESAVE_CHUNK;/if(n > NESAVE_CHUNK) n = NESAVE_CHUNK; if(sv_offset + n >= sv_size) { sv_offset = 0; sv_crc = 0; return; }/' "$M" ;;
  noautosaveoff) sed -i.bak 's/if(!CFG.enable_autosave) {/if(0) {/' "$M" ;;
  retryfail)   sed -i.bak 's/    sv_same = 0;                  \/\* retry/    \/\* retry/' "$M" ;;
  nofpgacheck) sed -i.bak 's/  if(fpga_test() != FPGA_TEST_TOKEN) return;//' "$M" ;;
  nowatchreset) sed -i.bak 's/sv_crc = sv_scan(NESAVE_WINDOW_ADDR + sv_offset, n, sv_crc, 1);/sv_crc = sv_scan(NESAVE_WINDOW_ADDR + sv_offset, n, sv_crc, 0);/' "$M" ;;
  m_didnot)    sed -i.bak "s/if(sv_didnotsave > 50) sv_write(filename);//" "$M" ;;
  m_keepdiff)  perl -0pi -e 's/    sv_diff = 0;/    /' "$M" ;;
  m_unrel)     perl -0pi -e 's/(if\(sv_bail\) \{\n)\s*sv_offset = 0;\n\s*sv_crc = 0;\n/$1/' "$M" ;;
  m_nobaseup)  sed -i.bak "s/  sv_crc_old = sv_crc;//" "$M" ;;
  m_flushbase) sed -i.bak "s/    if(!sv_failed) sv_crc_old = crc;//" "$M" ;;
  m_didfail)   sed -i.bak "s/    sv_didnotsave = 25;/    sv_didnotsave = 0;/" "$M" ;;
  unbounded)   sed -i.bak '/^static uint32_t sv_scan/,/^}/ s/FPGA_WAIT_RDY_TO(sv_hw_err)/FPGA_WAIT_RDY()/' "$M" ;;
  unbounded_setaddr) sed -i.bak '/^static void sv_setaddr/,/^}/ s/FPGA_WAIT_RDY_TO(sv_hw_err)/FPGA_WAIT_RDY()/' "$M" ;;
  unbounded_write) sed -i.bak '/^static void __attribute__((noinline)) sv_save_file/,/^}/ s/FPGA_WAIT_RDY_TO(sv_hw_err)/FPGA_WAIT_RDY()/' "$M" ;;
  unbounded_put) sed -i.bak '/^static void sv_put/,/^}/ s/FPGA_WAIT_RDY_TO(sv_hw_err)/FPGA_WAIT_RDY()/' "$M" ;;
  unbounded_rel) sed -i.bak '/^static uint8_t sv_reliable/,/^}/ s/FPGA_WAIT_RDY_TO(sv_hw_err)/FPGA_WAIT_RDY()/' "$M" ;;
  nodead)      sed -i.bak "s/    sv_dead = 1;//" "$M" ;;
  noprobe)     sed -i.bak '/^static void __attribute__((noinline)) sv_save_file/,/^}/{/^  sv_setaddr(NESAVE_WINDOW_ADDR);$/d;}' "$M" ;;
  noattachdisarm) sed -i.bak '/FPGA not answering, save disarmed/{n;s/    sv_size = 0;//;}' "$M" ;;
  nowriteerr)  sed -i.bak 's/sv_failed = (file_res || sv_hw_err) ? 1 : 0;/sv_failed = file_res ? 1 : 0;/' "$M" ;;
  *) echo "unknown mutant $NESAVE_MUTANT" >&2; exit 2 ;;
esac
if [ -n "${NESAVE_MUTANT:-}" ] && cmp -s ../../src/nesave.c "$M"; then
  echo "mutant $NESAVE_MUTANT did not change the source" >&2; exit 2
fi

$CC -O1 -Wall -Wextra -fsanitize=address,undefined -I shim_nesave -I "$B" \
    nesave_cli.c "$M" -o "$B/nesave_cli" || exit 1
exec "$B/nesave_cli"
