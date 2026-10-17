#!/usr/bin/env python3
"""The NES core is built three times (fpga_nes, fpga_nes_mmc3, fpga_nes_k), each with the
mapper families of the others compiled out by NES_NO_* macros of its own main.qsf.  Three
places have to agree, and a mismatch is silent on the console (a mapper sent to a core that
lacks it loads as NROM and runs garbage, no error):

  - verilog/sd2snes_nes/mmu.v       which mapper numbers each h_* arm of MultiMapper serves
  - the three main.qsf              which arms each core keeps (the NES_NO_* macros)
  - src/nes.c  nes_maps_{s,b,k}     which file the firmware loads for each mapper

Also checks that the two variant main.qsf are the parent's with only the paths, the macros and
the SEED changed (they are copies: a file added to the parent and not to a variant is a build
break found late). Run standalone (python3 tests/test_nes_core_routing.py) or via pytest.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
V = ROOT / "verilog"
CORES = {
    "S": V / "sd2snes_nes",
    "B": V / "sd2snes_nes_mmc3",
    "K": V / "sd2snes_nes_k",
}
FW_TABLE = {"S": "nes_maps_s", "B": "nes_maps_b", "K": "nes_maps_k"}
# mmu.v arm -> macro that removes it
ARM_MACRO = {"mmc1": "MMC1", "mmc3": "MMC3", "m28": "M28", "m69": "M69",
             "disc": "DISC", "vrc": "VRC", "m1k": "M1K"}
# internal submapper aliases (nes.c NES_MAP_ALIAS_*): the firmware routes the REAL number
ALIAS = {0xF4: 4, 0xF1: 32, 0xF8: 78}
# decoded by the RTL on purpose but kept out of every firmware table: 153 needs its battery
# RAM filled on the first run (the firmware hands the cart RAM over zeroed), and the screens
# of the 157 (Datach) games have no trustworthy emulator reference and were not proven on a board
RTL_ONLY = {153, 157}


def mmu_arms():
    """{arm: set(real mapper numbers)} from the `wire h_<arm> = (mnum == N) || ...;` lines."""
    src = (CORES["S"] / "mmu.v").read_text()
    src = re.sub(r"//[^\n]*", "", src)
    out = {}
    for m in re.finditer(r"wire\s+h_(\w+)\s*=([^;]*);", src):
        arm, body = m.group(1), m.group(2)
        if arm not in ARM_MACRO or "mnum" not in body:
            continue
        nums = set()
        for k in re.finditer(r"mnum\s*==\s*8'([dh])(\w+)", body):
            n = int(k.group(2), 10 if k.group(1) == "d" else 16)
            nums.add(ALIAS.get(n, n))
        out[arm] = nums
    return out


def qsf_macros(core):
    txt = (CORES[core] / "main.qsf").read_text()
    return set(re.findall(r'VERILOG_MACRO\s+"NES_NO_(\w+)=', txt))


def fw_table(name):
    src = (ROOT / "src" / "nes.c").read_text()
    m = re.search(r"static const uint8_t %s\[\]\s*=\s*\{(.*?)\};" % name, src, re.S)
    assert m, "table %s not found in src/nes.c" % name
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    return [int(x) for x in re.findall(r"\b\d+\b", body)]


def test_arms_are_all_known():
    arms = mmu_arms()
    assert set(arms) == set(ARM_MACRO), (set(arms) ^ set(ARM_MACRO))


def test_each_core_serves_what_the_firmware_routes_to_it():
    arms = mmu_arms()
    fw_all = []
    for core in "SBK":
        removed = qsf_macros(core)
        kept = {a for a, mac in ARM_MACRO.items() if mac not in removed}
        served = set()
        for a in kept:
            served |= arms[a]
        fw = fw_table(FW_TABLE[core])
        assert len(fw) == len(set(fw)), "duplicate mapper in %s" % FW_TABLE[core]
        fw_all += fw
        # NROM (mapper 0) is the fallback arm of every core, so it is in `served` of all of
        # them only through h_m28; the firmware lists it in S alone
        assert set(fw) <= served, "%s: firmware routes %s to a core without the arm" % (
            core, sorted(set(fw) - served))
        # every mapper a core serves through a KEPT arm must be routed to that core, except
        # the ones another core carries too (none today: the arms are disjoint)
        assert served - RTL_ONLY <= set(fw), "%s: the core serves %s, the firmware never sends them" % (
            core, sorted(served - RTL_ONLY - set(fw)))
    assert len(fw_all) == len(set(fw_all)), "a mapper is routed to two cores"


def test_arms_partition_between_the_cores():
    """Every arm is kept by exactly one core: two would waste area, none would drop a mapper."""
    for arm, mac in ARM_MACRO.items():
        keepers = [c for c in "SBK" if mac not in qsf_macros(c)]
        assert len(keepers) == 1, "arm %s kept by %s" % (arm, keepers)


def _norm(core):
    txt = (CORES[core] / "main.qsf").read_text()
    out = []
    for ln in txt.splitlines():
        if ln.startswith("#"):
            continue
        if re.match(r"set_global_assignment -name (SEED|SEARCH_PATH) ", ln):
            continue
        if "VERILOG_MACRO" in ln and "NES_NO_" in ln:
            continue
        out.append(ln.replace("../sd2snes_nes/", ""))
    return out


def test_variant_qsf_follow_the_parent():
    parent = _norm("S")
    for core in "BK":
        assert _norm(core) == parent, "main.qsf of %s drifted from sd2snes_nes/main.qsf" % core


def test_every_core_pins_a_seed():
    for core in list("SBK") + ["M5"]:
        txt = (CORES.get(core, V / "sd2snes_nes_mmc5") / "main.qsf").read_text()
        assert re.search(r"^set_global_assignment -name SEED \d+$", txt, re.M), core


def test_mmc5_has_its_own_core_and_a_title_list():
    """Mapper 5 lives only in fpga_nes_mmc5 and only for titles proven by CRC32 of the PRG."""
    src = (ROOT / "src" / "nes.c").read_text()
    m = re.search(r"static const uint8_t \*nes_core_for\(uint8_t mapper\)\s*\{(.*?)\n\}", src, re.S)
    assert m, "nes_core_for not found in src/nes.c"
    assert re.search(r"if\(mapper == 5\) return FPGA_NES_MMC5;", m.group(1)), \
        "nes_core_for does not route mapper 5 to FPGA_NES_MMC5"
    for core in "SBK":
        assert 5 not in fw_table(FW_TABLE[core]), "mapper 5 routed to %s" % core
    lst = re.search(r"\}\s*nes_mmc5_ok\[\]\s*=\s*\{(.*?)\};", src, re.S)
    assert lst and re.search(r"0x[0-9A-Fa-f]{8}UL", lst.group(1)), "nes_mmc5_ok[] is empty"
    assert re.search(r"nes_mmc5_validated\(", src.split("static uint8_t nes_mmc5_validated")[1]), \
        "nes_mmc5_validated is never called"
    assert (V / "sd2snes_nes_mmc5" / "main.qsf").exists()


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS", name)
            except AssertionError as e:
                fails += 1
                print("FAIL", name, "--", e)
    print("%d failure(s)" % fails)
    sys.exit(1 if fails else 0)
