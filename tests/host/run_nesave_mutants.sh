#!/usr/bin/env bash
# Mutation check for run_nesave.sh: every deliberate defect must make the gate fail.
set -u
cd "$(dirname "$0")"
export NESAVE_BUILD=build/nesave_mut
bash ./run_nesave.sh >/dev/null 2>&1 || { echo "baseline gate FAILS -- fix that first"; exit 1; }
bad=0
for m in nocap nodetach noflushdiff noflush nobaseline nostable noreliable noratelimit fullsize nolastchunk noautosaveoff retryfail nofpgacheck nowatchreset \
         m_didnot m_keepdiff m_unrel m_nobaseup m_flushbase m_didfail \
         unbounded unbounded_setaddr unbounded_write unbounded_put unbounded_rel nodead noprobe noattachdisarm nowriteerr; do
  out=$(NESAVE_MUTANT=$m bash ./run_nesave.sh 2>&1); rc=$?
  # "killed" = the test ran and reported a FAIL line; a mutant that does not build or does not
  # change the source proves nothing
  if [ $rc -eq 0 ]; then
    echo "SURVIVED: $m (the gate does not notice it)"; bad=1
  elif printf '%s\n' "$out" | grep -q '^FAIL'; then
    echo "killed:   $m"
  else
    echo "INVALID:  $m (did not build / did not apply)"; bad=1
  fi
done
exit $bad
