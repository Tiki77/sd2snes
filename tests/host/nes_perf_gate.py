#!/usr/bin/env python3
"""nes_perf_gate.py -- gate of the NES renderer performance telemetry.

Runs the REAL routines of snes/nes/nes_perf.a65 (bytes of misc/nes_snes.bin) in
the 65816 interpreter (nes_perf_cli) and compares every observable with a MODEL
written from the specification in utils/nes_perf_decode.py, never from the
assembly:

  sample     (ent, vm = (V9-225) mod 262), every scanline 0..261 (the 9-bit V is
             what separates 256..261 from 0..5), n0 = nmil
  decode     begin/mid/end -> the 24 accumulators (histogram edges, saturation,
             maximum, skip clamp, age)
  entry      the late-guard bins of the apply for every scanline
  nmi end    duration, end line, display overrun, ent, the `have` latch
  stream     hundreds of NMIs with decodes in between: the model closes the
             window the way the spec says and the bytes the assembly PUBLISHED
             at $2BDF, fed to the host assembler, must decode to the same report
  preempt    an NMI injected at every instruction of the sample must not change
             the answer (the retry on `ent`)
  cost       cycles of every routine (budget asserts)

Usage: nes_perf_gate.py <nes_perf_cli> <nes_snes.bin> <nes_snes.map>
"""
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "utils"))
import nes_perf_decode as dec  # noqa: E402

CLI, ROM, MAP = sys.argv[1:4]
HOLD = int(os.environ.get("NES_PF_HOLD", "3"))
SLOTS = dec.SLOTS
ACC = ["DECN", "H0", "H1", "H2", "H3", "H4", "H5", "SKIP", "LATE", "LDISP", "APPLY", "E0", "E1", "E2",
       "EMAX", "NDUR", "NEND", "NOVR", "AGE1", "AGE2", "DMAX", "DSUM", "DNMI", "DRN"]
IX = {n: i for i, n in enumerate(ACC)}
DBG = ["nes_dbg_older", "nes_dbg_stale", "nes_dbg_badrun", "nes_dbg_badop", "nes_dbg_reent",
       "nes_dbg_wovr", "nes_dbg_qovr"]

fails = []
checks = 0


def check(cond, what):
    global checks
    checks += 1
    if not cond:
        if len(fails) < 40:
            fails.append(what)
        elif len(fails) == 40:
            fails.append("... (more)")


class Script:
    """Accumulates CLI commands and one handler per output record."""

    def __init__(self):
        self.lines = ["boot"]
        self.handlers = []

    NOUT = {"call": 1, "cyc": 1, "g8": 1, "g16": 1, "acc": 1, "rep": 2, "stepcall": 1, "resume": 2}

    def cmd(self, line, handler=None, nout=None):
        self.lines.append(line)
        n = self.NOUT.get(line.split()[0], 0)
        if n:
            self.handlers.append((handler or (lambda o: None), n))

    def run(self):
        p = subprocess.run([CLI, ROM, MAP], input="\n".join(self.lines) + "\n", text=True,
                           capture_output=True)
        if p.returncode != 0:
            print(p.stderr)
            sys.exit("nes_perf_cli died (rc=%d)" % p.returncode)
        out = p.stdout.splitlines()
        i = 0
        for h, n in self.handlers:
            h(out[i:i + n])
            i += n
        if i != len(out):
            sys.exit("output/handler mismatch: %d lines for %d handlers" % (len(out), i))


def kv(line):
    return dict(t.split("=") for t in line.split()[1:] if "=" in t)


def T(ent, v9):
    return (ent * 262 + ((v9 - 225) % 262)) & 0xFFFF


def inst(t):
    """(ent, V9) whose clock T equals t (mod 65536).  ent*262 only reaches even
    residues, so vm takes the parity of t; ent is fixed mod 32768."""
    t &= 0xFFFF
    for vm in range(262):
        diff = (t - vm) & 0xFFFF
        if diff % 2 == 0:
            e = ((diff // 2) * pow(131, -1, 32768)) % 32768
            return e, (vm + 225) % 262
    raise AssertionError


# ---------------------------------------------------------------- model of the accumulators
class Model:
    def __init__(self):
        self.acc = [0] * 24
        self.t0 = self.n0 = self.t1 = 0
        self.ent = 0
        self.nmil = 0
        self.have = 0
        self.ens = 0
        self.prev = [0] * 6
        self.wseq = 0
        self.hold = 0
        self.slot = SLOTS - 1
        self.slots = None
        self.pub = 0

    def inc(self, n):
        i = IX[n]
        self.acc[i] = min(0xFFFF, self.acc[i] + 1)

    def derived(self):
        """DECN and APPLY have no accumulator: the freeze sums the histogram / entry bins."""
        self.acc[IX["DECN"]] = sum(self.acc[IX["H%d" % k]] for k in range(6)) & 0xFFFF
        self.acc[IX["APPLY"]] = sum(self.acc[IX[n]] for n in ("E0", "E1", "E2")) & 0xFFFF

    def add(self, n, v):
        i = IX[n]
        self.acc[i] = min(0xFFFF, self.acc[i] + (v & 0xFFFF))

    def mx(self, n, v):
        i = IX[n]
        if (v & 0xFFFF) >= self.acc[i]:
            self.acc[i] = v & 0xFFFF

    # --- main side
    def begin(self, ent, v9):
        self.t0 = self.t1 = T(ent, v9)
        self.n0 = self.nmil

    def mid(self, ent, v9):
        t = T(ent, v9)
        self.add("DRN", (t - self.t1) & 0xFFFF)
        self.t1 = t

    def end(self, ent, v9, age, skip):
        wall = (T(ent, v9) - self.t0) & 0xFFFF
        self.add("DSUM", wall)
        self.mx("DMAX", wall)
        self.add("DNMI", (self.nmil - self.n0) & 0xFFFF)
        self.add("SKIP", 0 if skip >= 0x8000 else skip)
        if age >= 1:
            self.inc("AGE1")
        if age >= 2:
            self.inc("AGE2")
        b = 5
        for k, e in enumerate(dec.HIST_EDGES):
            if wall < e:
                b = k
                break
        self.inc("H%d" % b)

    # --- NMI side
    def entry(self, v9):
        L = (v9 - 225) % 262
        self.have, self.ens = 1, L
        self.inc("E0" if L < 2 else "E1" if L < 4 else "E2")
        self.mx("EMAX", L)

    def late(self, v9):
        L = (v9 - 225) % 262
        self.have, self.ens = 1, L
        self.inc("LATE")
        if L >= 37:
            self.inc("LDISP")
        self.mx("EMAX", L)

    def nmi_end(self, v9, dbgvals):
        """returns (pub_byte, report_or_None)"""
        if not self.have:
            self.ens = 0                      # no apply: assumed to have entered at V=225
        le = (v9 - 225) % 262
        d = (le - self.ens) % 262
        self.nmil = (self.nmil + d) & 0xFFFF
        self.mx("NDUR", d)
        self.mx("NEND", le)
        if le >= 38:
            self.inc("NOVR")
        self.have = 0
        self.ent = (self.ent + 1) & 0xFFFF
        rep = None
        self.hold += 1
        if self.hold >= HOLD:
            self.slot += 1
            if self.slot >= SLOTS:
                rep = self.freeze(dbgvals)
                self.slot = 0
            self.pub = self.slots[self.slot]
            self.hold = 0
        return self.pub, rep

    def freeze(self, dbg):
        o, st, br, bo, re_, wo, qo = dbg
        cur = [o, st, (br + bo) & 0xFFFF, re_, wo, qo]
        sat = lambda x: min(255, x)
        self.derived()
        rep = [0x10 | HOLD, self.wseq & 0xFF]
        rep += [sat(self.acc[i]) for i in range(20)]
        rep.append(sat(self.acc[IX["DMAX"]] >> 2))
        for n in ("DSUM", "DNMI", "DRN"):
            v = self.acc[IX[n]] >> 2
            rep += [v & 0xFF, v >> 8]
        for k in range(6):
            rep.append(sat((cur[k] - self.prev[k]) & 0xFFFF))
            self.prev[k] = cur[k]
        rep.append(0)
        rep = dec.seal(rep)
        self.wseq = (self.wseq + 1) & 0xFFFF
        self.acc = [0] * 24
        self.slots = dec.encode_slots(rep)
        return rep


def poke_acc(s, vals):
    for i, v in enumerate(vals):
        if v:
            s.cmd("p16 nes_pf_acc+%d %d" % (2 * i, v))


# ---------------------------------------------------------------- scenarios
def sc_clock():
    """The sample (ent, vm): every scanline 0..261 (the 9-bit V is what separates
    256..261 from 0..5) for a spread of ent values; nmil is copied as n0."""
    s = Script()
    s.cmd("init")
    ents = [0, 1, 2, 3, 127, 128, 255, 256, 257, 1000, 4000, 65535]
    for e in ents:
        for v in range(262):
            s.cmd("p16 nes_pf_ent %d" % e)
            s.cmd("p16 nes_pf_nmil %d" % ((e * 7 + v) & 0xFFFF))
            s.cmd("v %d" % v)
            s.cmd("call nes_pf_dec_begin")
            vm = (v - 225) % 262
            for sym, want in (("nes_pf_s0", e), ("nes_pf_s0+2", vm), ("nes_pf_s1", e), ("nes_pf_s1+2", vm),
                              ("nes_pf_n0", (e * 7 + v) & 0xFFFF)):
                s.cmd("g16 " + sym, lambda o, sym=sym, want=want, e=e, v=v: check(
                    int(o[0].split("=")[1]) == want, "sample %s ent=%d V=%d got %s want %d" % (sym, e, v, o[0], want)))
    s.run()


def sc_decode(rnd):
    s = Script()
    for trial in range(500):
        m = Model()
        pre = [0] * 24
        if trial % 7 == 0:       # saturation: accumulators one step from the ceiling
            pre = [rnd.choice([0, 0xFFFF, 0xFFFE, 0xFFF0]) for _ in range(24)]
        m.acc = list(pre)
        s.cmd("init")
        poke_acc(s, pre)
        e0, v0 = rnd.randrange(65536), rnd.randrange(262)
        wall_target = rnd.choice([0, 1, 130, 131, 132, 261, 262, 263, 392, 393, 523, 524, 785, 786,
                                  rnd.randrange(3000), rnd.randrange(20000)])
        nm0 = rnd.randrange(65536)
        nm1 = (nm0 + rnd.randrange(0, 400)) & 0xFFFF
        t0 = T(e0, v0)
        mid_off = rnd.randrange(0, max(1, wall_target + 1))
        em, vm_ = inst(t0 + mid_off)
        ee, ve = inst(t0 + wall_target)
        age = rnd.choice([0, 0, 1, 2, 3, 9])
        skip = rnd.choice([0, 0, 1, 2, 5, 0x7FFF, 0x8000, 0xFFFF])
        s.cmd("p16 nes_pf_ent %d" % e0)
        s.cmd("p16 nes_pf_nmil %d" % nm0)
        s.cmd("v %d" % v0)
        s.cmd("call nes_pf_dec_begin a=0xABCD x=0x1111 y=0x2222")
        m.nmil = nm0
        m.begin(e0, v0)
        s.cmd("p16 nes_pf_ent %d" % em)
        s.cmd("v %d" % vm_)
        s.cmd("call nes_pf_dec_mid")
        m.mid(em, vm_)
        s.cmd("p16 nes_pf_ent %d" % ee)
        s.cmd("p16 nes_pf_nmil %d" % nm1)
        s.cmd("v %d" % ve)
        m.nmil = nm1
        m.end(ee, ve, age, skip)

        s.cmd("call nes_pf_dec_end a=%d x=%d" % (age, skip))
        want = list(m.acc)

        def hacc(o, want=want, trial=trial, wall_target=wall_target):
            got = [int(x) for x in o[0].split()[1:]]
            for i, (g, w) in enumerate(zip(got, want)):
                check(g == w, "decode trial %d (wall=%d) acc[%s]=%d want %d" % (trial, wall_target, ACC[i], g, w))
        s.cmd("acc", hacc)
    s.run()


def sc_entry():
    s = Script()
    for v9 in range(262):
        s.cmd("init")
        s.cmd("p16 nes_pf_nv %d" % v9)
        m = Model()
        ok = 225 <= v9 <= 232
        s.cmd("call %s" % ("nes_pf_entry" if ok else "nes_pf_late"))
        (m.entry if ok else m.late)(v9)

        def h(o, m=m, v9=v9):
            got = [int(x) for x in o[0].split()[1:]]
            for i, (g, w) in enumerate(zip(got, m.acc)):
                check(g == w, "entry V=%d acc[%s]=%d want %d" % (v9, ACC[i], g, w))
        s.cmd("acc", h)
        s.cmd("g16 nes_pf_ens", lambda o, m=m, v9=v9: check(int(o[0].split("=")[1]) == m.ens,
                                                              "entry V=%d ens" % v9))
    s.run()


def sc_stream(rnd):
    """The model closes the window; the published bytes go through the host
    assembler; the dump of the report in WRAM is compared as well."""
    s = Script()
    m = Model()
    m.slots = None
    asm = dec.Assembler()
    decoded = []
    nm = 3 * SLOTS * HOLD + 7
    dbg = [rnd.randrange(65536) for _ in range(7)]      # whatever the older instruments had counted
    for i in range(7):
        s.cmd("p16 %s %d" % (DBG[i], dbg[i]))
    s.cmd("init")                                       # seeds the deltas from them
    m.prev = [dbg[0], dbg[1], (dbg[2] + dbg[3]) & 0xFFFF, dbg[4], dbg[5], dbg[6]]
    reps_expected = []
    reps_wram = []
    pubs = []
    ent = 0
    for k in range(nm):
        # main side: a decode now and then
        if rnd.random() < 0.55:
            e0, v0 = ent, rnd.randrange(262)
            t0 = T(e0, v0)
            wall = rnd.choice([40, 100, 200, 300, 500, 900, 2500, 100, 100, 250])
            em, vmm = inst(t0 + wall // 3)
            ee, ve = inst(t0 + wall)
            s.cmd("p16 nes_pf_ent %d" % e0)
            s.cmd("v %d" % v0)
            s.cmd("call nes_pf_dec_begin")
            m.begin(e0, v0)
            s.cmd("p16 nes_pf_ent %d" % em)
            s.cmd("v %d" % vmm)
            s.cmd("call nes_pf_dec_mid")
            m.mid(em, vmm)
            s.cmd("p16 nes_pf_ent %d" % ee)
            s.cmd("v %d" % ve)
            age, skip = rnd.choice([0, 0, 1, 2]), rnd.choice([0, 0, 0, 1, 3])
            s.cmd("call nes_pf_dec_end a=%d x=%d" % (age, skip))
            m.end(ee, ve, age, skip)
            s.cmd("p16 nes_pf_ent %d" % ent)       # back to the NMI-side counter
        # NMI side: apply or not, late or not
        r = rnd.random()
        if r < 0.6:
            v9 = rnd.randrange(225, 233)
            s.cmd("p16 nes_pf_nv %d" % v9)
            s.cmd("call nes_pf_entry")
            m.entry(v9)
        elif r < 0.75:
            v9 = rnd.choice([233, 240, 255, 256, 261, 0, 1, 50, 224])
            s.cmd("p16 nes_pf_nv %d" % v9)
            s.cmd("call nes_pf_late")
            m.late(v9)
        # the dbg counters of the older instruments drift
        for i in range(7):
            if rnd.random() < 0.15:
                dbg[i] = (dbg[i] + rnd.choice([1, 1, 2, 7, 300, 0xFFF0])) & 0xFFFF
                s.cmd("p16 %s %d" % (DBG[i], dbg[i]))
        vend = rnd.choice([232, 240, 250, 255, 256, 259, 0, 1, 2, 5, 30, 150, 224]) if m.have else rnd.choice([225, 226, 229, 232, 255, 259, 0, 1, 2, 40, 224, rnd.randrange(262)])  # V=0/1/2 -> Le 37/38/39: the display-overrun edge
        s.cmd("v %d" % vend)
        pub, rep = m.nmi_end(vend, dbg)
        ent = m.ent
        if rep is not None:
            reps_expected.append(rep)
        exp_pub = pub
        was_freeze = rep is not None

        def hn(o, exp_pub=exp_pub):
            pass
        s.cmd("call nes_pf_nmi_end a=0x5555 x=0x6666 y=0x7777", hn)

        def hp(o, exp_pub=exp_pub, k=k):
            check(int(o[1].split()[1]) == exp_pub, "stream NMI %d: published %s want %d" % (k, o[1].split()[1], exp_pub))
            pubs.append(int(o[1].split()[1]))
        s.cmd("rep", hp, nout=2)
        s.cmd("g16 nes_pf_ent", lambda o, e=m.ent: check(int(o[0].split("=")[1]) == e, "ent counter"))
        s.cmd("g16 nes_pf_nmil", lambda o, e=m.nmil: check(int(o[0].split("=")[1]) == e, "nmil counter"))
        if was_freeze:
            def hw(o):
                reps_wram.append([int(x) for x in o[0].split()[1:]])
            s.cmd("rep", hw, nout=2)
    s.run()
    check(len(reps_expected) == len(reps_wram) and len(reps_wram) >= 3, "number of windows")
    for i, (a, b) in enumerate(zip(reps_expected, reps_wram)):
        check(a == b, "window %d report in WRAM differs:\n   want %s\n   got  %s" % (i, a, b))
    # what the host sees: replay the published bytes (each held, so duplicates)
    for p in pubs:
        r = asm.feed(p)
        if r:
            decoded.append(r["raw"])
    check(decoded == reps_expected[:len(decoded)] and len(decoded) >= len(reps_expected) - 1,
          "host assembler decoded %d of %d windows (or differently)" % (len(decoded), len(reps_expected)))


def sc_preempt():
    """An NMI at every instruction boundary of nes_pf_dec_begin (its sample)."""
    s = Script()
    res = {}
    cases = ((10, 230), (500, 261), (7, 0), (65535, 100))
    for step in range(1, 60):
        for (e, v) in cases:
            s.cmd("init")
            s.cmd("p16 nes_pf_ent %d" % e)
            s.cmd("v %d" % v)
            s.cmd("stepcall nes_pf_dec_begin %d" % step, lambda o, k=(step, e, v): res.setdefault(k, {}).update(
                started=(o[0] == "stepcall suspended")))
            s.cmd("inject nes_pf_nmi_end a=1 x=2 y=3")
            s.cmd("resume 100000", lambda o, k=(step, e, v): res.setdefault(k, {}).update(done=(o[0] == "resume done")), nout=2)
            s.cmd("g16 nes_pf_s0", lambda o, k=(step, e, v): res[k].update(ent=int(o[0].split("=")[1])))
            s.cmd("g16 nes_pf_s0+2", lambda o, k=(step, e, v): res[k].update(vm=int(o[0].split("=")[1])))
            s.cmd("g16 nes_pf_s1", lambda o, k=(step, e, v): res[k].update(ent1=int(o[0].split("=")[1])))
    s.run()
    seen = {}
    for (step, e, v), r in sorted(res.items()):
        if not r.get("started"):
            continue                              # the routine had already returned
        vm = (v - 225) % 262
        check(r["done"], "preempt@%d: the call did not finish" % step)
        # before the 2nd read of `ent` an NMI forces the retry (sample with the NEW ent);
        # after it the pair was already consistent (sample with the old one)
        check((r["ent"], r["vm"]) in ((e + 1 & 0xFFFF, vm), (e, vm)) and r["ent1"] == r["ent"],
              "preempt@%d ent=%d V=%d got ent=%d vm=%d (s1 ent %d) want ent %d or %d vm %d" % (
                  step, e, v, r["ent"], r["vm"], r["ent1"], (e + 1) & 0xFFFF, e, vm))
        seen.setdefault((e, v), []).append((step, r["ent"] == ((e + 1) & 0xFFFF)))
    check(len(seen) == len(cases), "preempt: a case never started")
    for key, lst in seen.items():
        lst.sort()
        flags = [f for _, f in lst]
        # monotone: NEW sample up to some step, OLD after it, and the retry region is non-trivial
        k = flags.index(False) if False in flags else len(flags)
        check(all(flags[:k]) and not any(flags[k:]) and k >= 8,
              "preempt %s: retry region is not a prefix of >= 8 steps: %s" % (key, flags))


def sc_freeze_sat(rnd):
    """Saturation at the report: every accumulator at 254..258 (and the /4 fields
    around 1020..1031), the deltas of the older instruments at the same edges.
    A byte field that lets 256 or 257 through is a corrupt report, and only a
    value sitting EXACTLY on the edge tells."""
    s = Script()
    edges = [0, 1, 254, 255, 256, 257, 258, 1019, 1020, 1023, 1024, 1027, 1028, 65535]
    for idx in range(24):
        for val in edges:
            if idx in (IX["DECN"], IX["APPLY"]):
                continue               # derived at the freeze: no accumulator of their own
            m = Model()
            s.cmd("init")
            s.cmd("p16 nes_pf_slot %d" % (SLOTS - 1))
            s.cmd("p16 nes_pf_hold %d" % (HOLD - 1))
            s.cmd("p16 nes_pf_acc+%d %d" % (2 * idx, val))
            m.acc[idx] = val
            m.slot, m.hold = SLOTS - 1, HOLD - 1
            s.cmd("v 240")
            rep = m.nmi_end(240, [0] * 7)[1]
            s.cmd("call nes_pf_nmi_end")

            def h(o, rep=rep, idx=idx, val=val):
                got = [int(x) for x in o[0].split()[1:]]
                check(got == rep, "freeze acc[%s]=%d:\n   want %s\n   got  %s" % (ACC[idx], val, rep, got))
            s.cmd("rep", h, nout=2)
    for k in range(6):                  # deltas of the older instruments
        for d in (254, 255, 256, 257, 258):
            m = Model()
            for name in DBG:                 # init seeds the deltas from whatever the last case left
                s.cmd("p16 %s 0" % name)
            s.cmd("init")
            vals = [0] * 7
            idx = [0, 1, 2, 4, 5, 6][k]
            vals[idx] = d
            s.cmd("p16 %s %d" % (DBG[idx], d))
            s.cmd("p16 nes_pf_slot %d" % (SLOTS - 1))
            s.cmd("p16 nes_pf_hold %d" % (HOLD - 1))
            m.slot, m.hold = SLOTS - 1, HOLD - 1
            s.cmd("v 240")
            rep = m.nmi_end(240, vals)[1]
            s.cmd("call nes_pf_nmi_end")

            def h(o, rep=rep, k=k, d=d):
                got = [int(x) for x in o[0].split()[1:]]
                check(got == rep, "freeze delta %d=%d:\n   want %s\n   got  %s" % (k, d, rep, got))
            s.cmd("rep", h, nout=2)
    s.run()


def run_sys(frames, events):
    """The INTEGRATION run.  Returns the dict of what the CLI printed."""
    p = subprocess.run([CLI, ROM, MAP], input="sys %d %s\n" % (frames, " ".join(events)),
                       text=True, capture_output=True)
    if p.returncode != 0:
        print(p.stdout[-2000:])
        print(p.stderr)
        sys.exit("nes_perf_cli died in the integration run (rc=%d)" % p.returncode)
    r = {"err": []}
    for ln in p.stdout.splitlines():
        if ln.startswith("act "):
            r["act"] = [int(x) for x in ln.split()[1:]]
        elif ln.startswith("exp "):
            r["exp"] = [int(x) for x in ln.split()[1:]]
        elif ln.startswith("err "):
            r["err"].append(ln[4:])
        elif ln.startswith("sys "):
            r["sys"] = {}
            for t in ln.split()[1:]:
                k, v = t.split("=")
                r["sys"][k] = tuple(int(x) for x in v.split("/")) if "/" in v else \
                    tuple(int(x) for x in v.split(",")) if "," in v else int(v)
    return r


def sc_sys():
    """Real poll loop + real NMI.  The renderer's counters must equal the ones the
    run's ground truth implies; a probe missing from its call site, or fed the
    wrong register, shows up here and nowhere else."""
    r = run_sys(120, ["ev=20:reent", "ev=50:late", "ev=80:disp"])
    s = r["sys"]
    for e in r["err"]:
        check(False, "integration: " + e)
    act, exp = r["act"], r["exp"]
    for i, n in enumerate(ACC):
        if n in ("DECN", "APPLY"):
            continue               # derived at the freeze
        check(act[i] == exp[i], "integration acc[%s]: renderer kept %d, ground truth says %d" % (n, act[i], exp[i]))
    check(s["ent"][0] == s["ent"][1], "integration: ent counter %d, vector entries %d" % s["ent"])
    check(s["nmil"][0] == s["nmil"][1], "integration: nmil %d, model %d" % s["nmil"])
    check(s["full"] == s["decodes"] and s["decodes"] > 0, "integration: %d decodes, %d with all 3 probes" % (s["decodes"], s["full"]))
    check(s["probes"] == (s["decodes"],) * 3 or s["probes"][0] >= s["decodes"], "integration: probe entries %s for %d decodes" % (s["probes"], s["decodes"]))
    # the scenario must actually exercise what it claims to (a vacuous run proves nothing)
    check(s["reents"] == 1 and s["reent_ctr"] == 1, "integration: reentry NMIs %d (counter %d), want 1" % (s["reents"], s["reent_ctr"]))
    check(s["lates"] >= 2, "integration: late NMIs %d, want >= 2" % s["lates"])
    check(s["applies"] >= 25 and s["decodes"] >= 30, "integration: applies %d decodes %d" % (s["applies"], s["decodes"]))
    check(s["wseq"] == 1, "integration: a second window closed (wseq %d); the run must stay inside one" % s["wseq"])
    check(exp[IX["LDISP"]] == 1 and exp[IX["EMAX"]] >= 37, "integration: no display-active entry (LDISP %d EMAX %d)" % (exp[IX["LDISP"]], exp[IX["EMAX"]]))
    check(exp[IX["LATE"]] == 2, "integration: LATE %d, want 2" % exp[IX["LATE"]])
    check(sum(1 for k in range(6) if exp[IX["H%d" % k]]) >= 3, "integration: decode histogram covers < 3 bins")
    for n in ("SKIP", "AGE1", "DNMI", "DRN", "DSUM", "NDUR", "NEND", "E0"):
        check(exp[IX[n]] > 0, "integration: ground truth has %s = 0 (the run does not exercise it)" % n)
    check(sum(exp[IX["E%d" % k]] for k in range(3)) == s["applies"], "integration: bins vs applies")
    print("  integration: %d NMIs (%d reentry, %d late), %d decodes, ent=%d, nmil=%d" % (
        s["nmis"], s["reents"], s["lates"], s["decodes"], s["ent"][0], s["nmil"][0]))


COST_BUDGET = {"nes_pf_dec_begin": 400, "nes_pf_dec_mid": 300, "nes_pf_dec_end": 800,
               "nes_pf_entry": 200, "nes_pf_late": 200, "nes_pf_nmi_end": 450, "nes_pf_reent": 40}


def sc_cost():
    s = Script()
    s.cmd("init")
    costs = {}
    s.cmd("p16 nes_pf_ent 5")
    s.cmd("v 240")
    for name in ("nes_pf_dec_begin", "nes_pf_dec_mid", "nes_pf_dec_end"):
        def h(o, name=name):
            costs[name] = float(o[0].split("cycles=")[1].split()[0])
        s.cmd("cyc %s a=0 x=0" % name, h)
    s.cmd("p16 nes_pf_nv 227")
    for name in ("nes_pf_entry", "nes_pf_late", "nes_pf_nmi_end", "nes_pf_reent"):
        def h(o, name=name):
            costs[name] = float(o[0].split("cycles=")[1].split()[0])
        s.cmd("cyc %s" % name, h)
    s.run()
    for k, v in costs.items():
        check(v <= COST_BUDGET[k], "cost of %s = %.0f cycles > budget %d" % (k, v, COST_BUDGET[k]))
    print("cost (cycles at 8 mc, the figure the renderer profile uses; 1 frame = 44671):")
    for k, v in costs.items():
        print("  %-18s %6.0f" % (k, v))
    per_dec = costs["nes_pf_dec_begin"] + costs["nes_pf_dec_mid"] + costs["nes_pf_dec_end"]
    per_nmi = costs["nes_pf_entry"] + costs["nes_pf_nmi_end"]
    print("  per decoded frame (begin+mid+end)   %6.0f  = %.2f%% of a frame" % (per_dec, 100 * per_dec / 44671))
    print("  per NMI (entry+nmi_end, w/o freeze) %6.0f  = %.2f%% of a frame" % (per_nmi, 100 * per_nmi / 44671))
    return costs


def sc_freeze_cost(rnd):
    s = Script()
    s.cmd("init")
    out = []
    s.cmd("p16 nes_pf_slot %d" % (SLOTS - 1))
    s.cmd("p16 nes_pf_hold %d" % (HOLD - 1))
    s.cmd("v 240")
    s.cmd("cyc nes_pf_nmi_end", lambda o: out.append(float(o[0].split("cycles=")[1].split()[0])))
    s.run()
    print("  nmi_end that freezes a window       %6.0f  = %.2f%% of a frame" % (out[0], 100 * out[0] / 44671))
    check(out[0] <= 4500, "freeze cost %.0f > 4500" % out[0])


def main():
    rnd = random.Random(20261014)
    only = os.environ.get("NES_PERF_ONLY", "").split(",")      # development aid: run a subset
    for name, fn in (("clock", sc_clock), ("decode", lambda: sc_decode(rnd)), ("entry", sc_entry),
                     ("stream", lambda: sc_stream(rnd)), ("preempt", sc_preempt),
                     ("freeze", lambda: sc_freeze_sat(rnd)), ("system", sc_sys)):
        if only != [""] and name not in only:
            continue
        before = len(fails)
        fn()
        print("%-8s %s" % (name, "PASS" if len(fails) == before else "FAIL"))
    before = len(fails)
    if only == [""] or "cost" in only:
        sc_cost()
        sc_freeze_cost(rnd)
        print("cost     %s" % ("PASS" if len(fails) == before else "FAIL"))
    print("%d checks, %d failures" % (checks, len(fails)))
    for f in fails:
        print("  FAIL:", f)
    sys.exit(1 if fails else 0)


main()
