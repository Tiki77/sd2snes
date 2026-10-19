#!/usr/bin/env python3
"""Check the bank literals of snes/header.a65 against the link.

header.a65 assembles to a fixed address (it is the .ips), where snescom refuses the bank
of an extern, so the bank bytes it publishes for the firmware and the in-game menu are
literals: $C1 for hx_pad and for the five words of MENU_GOPT_PTRS.  A label moving to
another bank would leave the literal pointing at whatever now sits at that address, with
a clean build.  This reads the objects' maps (utils/mkmap.sh: "C11020 strtab_lo") and fails
when one of them is not where the header says.

Usage: hdrcheck.py <object.map>...
"""
import re
import sys

EXPECT = {  # label -> the bank header.a65 publishes for it
    "hx_pad": 0xC1,
    "menu_enttab_cfg": 0xC1,
    "strtab_lo": 0xC1,
    "strtab_hi": 0xC1,
    "strpool_bank": 0xC1,
    "strtab_nlang_latin": 0xC1,
}

def main() -> int:
    found = {}
    for path in sys.argv[1:]:
        for line in open(path, errors="replace"):
            m = re.match(r"\s*([0-9A-Fa-f]{6})\s+(\w+)\s*$", line)
            if m and m.group(2) in EXPECT:
                found[m.group(2)] = int(m.group(1), 16) >> 16
    bad = []
    for label, bank in EXPECT.items():
        if label not in found:
            bad.append(f"{label}: in none of the maps")
        elif found[label] != bank:
            bad.append(f"{label}: bank ${found[label]:02X}, header.a65 says ${bank:02X}")
    if bad:
        print("hdrcheck: snes/header.a65 publishes a wrong bank:\n  " + "\n  ".join(bad))
        return 1
    print("hdrcheck: header.a65 banks ok")
    return 0

if __name__ == "__main__":
    sys.exit(main())
