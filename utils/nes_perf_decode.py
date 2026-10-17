#!/usr/bin/env python3
"""nes_perf_decode.py -- decoder of the NES renderer performance telemetry.

The renderer (snes/nes/nes_perf.a65) closes a report every 48 slots and sends it
through the one-byte debug register $2BDF (NDBG +31).  Each slot is held for
HOLD NMIs:

    bit7 = 1 only on slot 0      bit6 = slot parity      bits5..0 = 6 data bits

The 48 groups of 6 bits (MSB first) are the 36 bytes of the report:

    0        version<<4 | HOLD        1       window number (mod 256)
    2..21    20 saturated 8-bit counters (G8, order below)
    22       decode wall maximum / 4 lines (saturated)
    23..28   dsum/4, dnmi/4, drn/4 (16 bits LE each)
    29..34   older stale bad reent wovr qovr (delta since the previous report)
    35       checksum: c = 0x5A; for i in 0..34: c = rol8(c) ^ rep[i]

All counters are PER WINDOW (reset at every report).  One window = 48*HOLD NMIs.

Units: 'lines' are scanlines (262 per frame); a 'frame' is 262 lines.
"""
import sys

VER = 1
SLOTS = 48
REPORT_BYTES = 36
CHK_SEED = 0x5A
FRAME_LINES = 262

G8 = ["decn", "h0", "h1", "h2", "h3", "h4", "h5", "skip", "late", "ldisp", "apply",
      "e0", "e1", "e2", "emax", "ndur", "nend", "novr", "age1", "age2"]
DELTAS = ["older", "stale", "bad", "reent", "wovr", "qovr"]
# decode wall histogram bins (lines): [0,131) [131,262) [262,393) [393,524) [524,786) [786,inf)
HIST_EDGES = [131, 262, 393, 524, 786]
HIST_LABELS = ["<0.5f", "<1f", "<1.5f", "<2f", "<3f", ">=3f"]


def rol8(c):
    c = (c << 1) & 0x1FF
    return ((c & 0xFF) | (c >> 8)) & 0xFF


def checksum(rep):
    c = CHK_SEED
    for b in rep[:REPORT_BYTES - 1]:
        c = rol8(c) ^ b
    return c


def seal(rep):
    """rep: list of 36 ints with rep[35] ignored -> the same list with the checksum."""
    rep = list(rep[:REPORT_BYTES])
    rep[REPORT_BYTES - 1] = checksum(rep)
    return rep


def encode_slots(rep):
    """36 bytes -> the 48 published slot bytes (what $2BDF shows, in order)."""
    bits = []
    for b in rep:
        bits += [(b >> k) & 1 for k in range(7, -1, -1)]
    out = []
    for s in range(SLOTS):
        v = 0
        for k in range(6):
            v = (v << 1) | bits[6 * s + k]
        out.append(v | (0x40 if s & 1 else 0) | (0x80 if s == 0 else 0))
    return out


def decode_slots(slots):
    """48 slot bytes -> 36 report bytes (no validation)."""
    bits = []
    for s in slots:
        bits += [((s & 0x3F) >> k) & 1 for k in range(5, -1, -1)]
    rep = []
    for i in range(REPORT_BYTES):
        v = 0
        for k in range(8):
            v = (v << 1) | bits[8 * i + k]
        rep.append(v)
    return rep


class Assembler:
    """Feed the byte seen at $2BDF at successive instants (feed(b)); returns a
    report (dict) whenever a complete, checksum-valid cycle has been seen.

    Slots change when the byte changes (the parity bit alternates, so two
    consecutive slots never carry the same byte).  A change whose parity is not
    the expected one means a slot was missed: the cycle in progress is dropped
    and counted in `gaps`.  A bad checksum counts in `bad`.
    """

    def __init__(self):
        self.last = None
        self.buf = None
        self.gaps = 0
        self.bad = 0
        self.ok = 0
        self.changes = 0

    def feed(self, b):
        b &= 0xFF
        if b == self.last:
            return None
        self.last = b
        self.changes += 1
        if b & 0x80:                      # slot 0: start a cycle
            if self.buf is not None:      # the previous cycle never completed
                self.gaps += 1
            if (b & 0x40) != 0:           # slot 0 has parity 0
                self.buf = None
                self.gaps += 1
                return None
            self.buf = [b]
            return None
        if self.buf is None:
            return None
        if ((b >> 6) & 1) != (len(self.buf) & 1):
            self.buf = None               # a slot went by unseen
            self.gaps += 1
            return None
        self.buf.append(b)
        if len(self.buf) < SLOTS:
            return None
        rep = decode_slots(self.buf)
        self.buf = None
        if rep[REPORT_BYTES - 1] != checksum(rep) or (rep[0] >> 4) != VER:
            self.bad += 1
            return None
        self.ok += 1
        return parse_report(rep)


def parse_report(rep):
    r = {"ver": rep[0] >> 4, "hold": rep[0] & 15, "wseq": rep[1]}
    for i, n in enumerate(G8):
        r[n] = rep[2 + i]
    r["dmax"] = rep[22] * 4
    r["dsum"] = (rep[23] | rep[24] << 8) * 4
    r["dnmi"] = (rep[25] | rep[26] << 8) * 4
    r["drn"] = (rep[27] | rep[28] << 8) * 4
    for i, n in enumerate(DELTAS):
        r[n] = rep[29 + i]
    r["nmis"] = SLOTS * r["hold"]
    r["raw"] = list(rep)
    return r


def derive(r):
    """Per-window numbers a human reads (all means over decodes)."""
    n = r["decn"]
    d = {"nmis": r["nmis"], "decodes": n,
         "decodes_per_nmi": n / r["nmis"] if r["nmis"] else 0.0,
         "idle_nmis": r["nmis"] - r["apply"] - r["late"]}
    if n:
        d["wall_mean_f"] = r["dsum"] / n / FRAME_LINES
        d["nmi_in_decode_f"] = r["dnmi"] / n / FRAME_LINES
        d["cpu_mean_f"] = (r["dsum"] - r["dnmi"]) / n / FRAME_LINES
        d["drain_mean_f"] = r["drn"] / n / FRAME_LINES
        d["wall_max_f"] = r["dmax"] / FRAME_LINES
        d["over1f_pct"] = 100.0 * sum(r["h%d" % k] for k in range(2, 6)) / n
    return d


def fmt(r):
    d = derive(r)
    hist = " ".join("%s=%d" % (HIST_LABELS[k], r["h%d" % k]) for k in range(6))
    lines = [
        "win %3d  nmis=%d  decodes=%d (%.2f/nmi)  applies=%d late=%d(disp %d) idle=%d" % (
            r["wseq"], r["nmis"], r["decn"], d["decodes_per_nmi"], r["apply"], r["late"],
            r["ldisp"], d["idle_nmis"]),
        "  decode wall   hist: %s" % hist,
    ]
    if r["decn"]:
        lines.append("  decode wall   mean %.3f f  max %.3f f  (cpu %.3f + nmi %.3f; drain %.3f)  >1f %.1f%%" % (
            d["wall_mean_f"], d["wall_max_f"], d["cpu_mean_f"], d["nmi_in_decode_f"],
            d["drain_mean_f"], d["over1f_pct"]))
    lines += [
        "  transport     skip=%d older=%d stale=%d bad=%d age>=1:%d age>=2:%d" % (
            r["skip"], r["older"], r["stale"], r["bad"], r["age1"], r["age2"]),
        "  nmi           entry L: 0-1:%d 2-3:%d 4-7:%d max %d lines | dur max %d lines  end max %d (L) | ended in display %d | reent %d" % (
            r["e0"], r["e1"], r["e2"], r["emax"], r["ndur"], r["nend"], r["novr"], r["reent"]),
        "  chr overruns  wovr=%d qovr=%d" % (r["wovr"], r["qovr"]),
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    # self-test: encode a report, decode it through the assembler with a hold
    import random
    rnd = random.Random(1)
    asm = Assembler()
    for t in range(200):
        rep = seal([VER << 4 | 3, t & 255] + [rnd.randrange(256) for _ in range(REPORT_BYTES - 2)])
        got = None
        for s in encode_slots(rep):
            for _ in range(rnd.randrange(1, 4)):
                got = asm.feed(s) or got
        assert got and got["raw"] == rep, t
    # a missed slot drops the cycle
    rep = seal([VER << 4 | 3, 7] + [0] * (REPORT_BYTES - 2))
    slots = encode_slots(rep)
    del slots[10]
    a2 = Assembler(); got = None
    for s in slots + encode_slots(rep):       # the cycle after the loss is clean
        got = a2.feed(s) or got
    assert got and got["raw"] == rep and a2.gaps == 1 and a2.ok == 1
    print("nes_perf_decode self-test OK")
