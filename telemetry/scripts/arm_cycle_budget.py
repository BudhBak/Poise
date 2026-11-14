"""Static worst-case cycle budget of one 500 Hz step() on the RA4M1 (Cortex-M4F, 48 MHz).

Compiles firmware/sim/arm_probe.cpp with the Arduino renesas_uno 1.6.0 flags (-Os -fno-builtin, cortex-m4,
fpv4-sp-d16, hard float), disassembles it, and bounds the cycles of probe_v2 (v2 HapticEngine), probe_db
(dual-band BMFLC) and probe_worst (every v3 feature enabled):

  * every instruction is costed with the Cortex-M4 TRM upper bounds below (branches always taken);
  * loops are the non-trivial strongly connected components of each function's control-flow graph; every
    loop in innovation_engine.h / admittance.h is bounded at 16 iterations by construction (BM_N = FO_N = 16),
    so each loop instruction is charged 16 times;
  * all paths are summed (every if-branch charged), so the bound covers any InnovationConfig;
  * calls into the object are followed recursively; library calls are charged conservative constants;
  * any double-precision helper (__aeabi_d*) or unknown call is reported as a FAIL (software emulation);
  * x2 bound for one flash wait state at 48 MHz (no cache credit).
Budget: 250 us = 12,000 cycles at 48 MHz. Exits 1 if the bound is exceeded or a forbidden call appears.

ELF mode (--elf src.ino.elf): the same costing applied to the linked firmware image built by arduino-cli.
Roots: controlIsr() (the 500 Hz control ISR, HapticEngine::step inlined), the LED-matrix refresh chain
gpt_counter_overflow_isr -> ArduinoLEDMatrix::turnOnLedISR (10 kHz, priority 12) and the millis() chain
agt_int_isr -> timer_micros_callback (1 kHz, priority 8). Direct calls are resolved from "bl <sym>"
operands; an indirect "blx rN" is listed and its known callee is analysed as a separate root. The control
ISR bound must fit HW_MAX_ISR_US_BUDGET (250 us); the preemption load of the two lower-priority chains is
reported as a CPU fraction (they never delay the control ISR, which runs at NVIC priority 1).

Usage: python arm_cycle_budget.py [--gcc <arm-none-eabi-g++>] [--json out.json] [--elf src.ino.elf]
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PROBE = REPO / "firmware" / "sim" / "arm_probe.cpp"
FLAGS = ["-c", "-Os", "-g0", "-mcpu=cortex-m4", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16", "-std=gnu++17",
         "-fno-rtti", "-fno-exceptions", "-fno-builtin", "-fsigned-char", "-ffunction-sections", "-fdata-sections",
         "-Wall", "-Wextra"]
F_CPU = 48e6
BUDGET_CYCLES = 250e-6 * F_CPU
LIB_COST = {"sqrtf": 40, "fmodf": 250, "cosf": 400, "sinf": 400, "fabsf": 4, "floorf": 40}
FORBIDDEN = re.compile(r"__aeabi_d|__aeabi_f2d|__aeabi_d2f|__adddf3|__muldf3|__divdf3")


def find_gcc(explicit: str | None) -> Path:
    cands = [explicit] if explicit else []
    cands += [os.environ.get("ARM_GXX", "")]
    roots = [Path(os.environ.get("LOCALAPPDATA", "")) / "Arduino15", Path.home() / ".arduino15",
             Path.home() / "Library" / "Arduino15", Path(os.environ.get("TEMP", "/tmp")) / "sgA" / "data"]
    for r in roots:
        cands += [str(p) for p in (r / "packages" / "arduino" / "tools" / "arm-none-eabi-gcc").glob("*/bin/arm-none-eabi-g++*")]
    for c in cands:
        if c and Path(c).exists():
            return Path(c)
    raise SystemExit("arm-none-eabi-g++ not found (install the Arduino renesas_uno core or pass --gcc)")


def portable(gxx: Path) -> str:
    """Toolchain id without the local install prefix (keeps user names out of committed JSON)."""
    return "/".join(gxx.parts[-3:])        # e.g. 7-2017q4/bin/arm-none-eabi-g++.exe


def cost(mn: str, ops: str) -> int:
    m = mn.split(".")[0].lower()
    if m in ("vdiv", "vsqrt"):
        return 14
    if m in ("vmla", "vmls", "vnmla", "vnmls", "vfma", "vfms", "vfnma", "vfnms"):
        return 3
    if m in ("vldr", "vstr", "ldr", "ldrb", "ldrh", "ldrsb", "ldrsh", "str", "strb", "strh", "ldrd", "strd"):
        return 2 if m not in ("ldrd", "strd") else 3
    if m in ("vldmia", "vldmdb", "vstmia", "vstmdb", "vpush", "vpop"):
        return 1 + ops.count("s") + ops.count("d") * 2
    if m in ("push", "pop", "ldmia", "stmia", "ldm", "stm", "stmdb", "ldmdb"):
        return 1 + max(1, ops.count(",") + 1)
    if m in ("b", "bx", "cbz", "cbnz") or re.fullmatch(r"b(eq|ne|cs|cc|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|hs|lo)", m):
        return 3
    if m in ("bl", "blx"):
        return 4
    if m in ("udiv", "sdiv"):
        return 12
    return 1


def parse_elf(objdump: str) -> dict[str, list[list]]:
    """Linked image: call/tail-call targets come from the "<sym>" operand (no relocations in an ELF)."""
    funcs: dict[str, list] = {}
    cur = None
    for line in objdump.splitlines():
        mh = re.match(r"^([0-9a-f]+) <(.+)>:$", line)
        if mh:
            cur = mh.group(2)
            funcs[cur] = []
            continue
        mi = re.match(r"^\s+([0-9a-f]+):\s+([a-z][a-z0-9.]*)\s*(.*)$", line)
        if mi and cur:
            m, o = mi.group(2), mi.group(3)
            rel = None
            mt = re.search(r"[0-9a-f]+ <([^>]+?)>", o)
            if mt and (m.split(".")[0] in ("bl", "blx") or BRANCH.match(m)) and "+0x" not in mt.group(1):
                if mt.group(1) != cur:
                    rel = mt.group(1)
            if m.split(".")[0] == "blx" and not mt:
                rel = "<indirect>"
            funcs[cur].append([int(mi.group(1), 16), m, o, rel])
    return funcs


def parse(objdump: str) -> dict[str, list[list]]:
    """Functions -> [addr, mnemonic, operands, reloc_symbol]; relocation lines (objdump -r) name call targets."""
    funcs: dict[str, list] = {}
    cur = None
    for line in objdump.splitlines():
        mh = re.match(r"^([0-9a-f]+) <(.+)>:$", line)
        if mh:
            cur = mh.group(2)
            funcs[cur] = []
            continue
        mr = re.match(r"^\s+[0-9a-f]+:\s+(R_ARM_\w+)\s+(.+)$", line)
        if mr:
            if cur and funcs[cur] and mr.group(1) in ("R_ARM_THM_CALL", "R_ARM_THM_JUMP24", "R_ARM_THM_JUMP19"):
                funcs[cur][-1][3] = mr.group(2).strip()
            continue
        mi = re.match(r"^\s+([0-9a-f]+):\s+([a-z][a-z0-9.]*)\s*(.*)$", line)
        if mi and cur:
            funcs[cur].append([int(mi.group(1), 16), mi.group(2), mi.group(3), None])
    return funcs


BRANCH = re.compile(r"^(b|b(eq|ne|cs|cc|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|hs|lo)|cbz|cbnz)(\.[nw])?$")


def loop_blocks(ins) -> tuple[set[int], int]:
    """Instruction indices inside real loops (non-trivial SCCs of the CFG), and the number of loops."""
    n = len(ins)
    addr = {a: i for i, (a, _, _, _) in enumerate(ins)}
    succ = [[] for _ in range(n)]
    for i, (a, m, o, _) in enumerate(ins):
        mb = BRANCH.match(m)
        tgt = None
        if mb:
            mt = re.match(r"([0-9a-f]+) <", o)
            if mt and int(mt.group(1), 16) in addr:
                tgt = addr[int(mt.group(1), 16)]
        is_ret = ((m.startswith("bx") and "lr" in o) or (m.startswith("pop") and "pc" in o)
                  or (m.startswith("ldm") and "pc" in o))
        if tgt is not None:
            succ[i].append(tgt)
            if m.split(".")[0] not in ("b",):
                if i + 1 < n: succ[i].append(i + 1)
        elif not is_ret and i + 1 < n:
            succ[i].append(i + 1)
    pred = [[] for _ in range(n)]
    for i in range(n):
        for j in succ[i]:
            pred[j].append(i)

    def reach(start, edges):
        seen, todo = {start}, [start]
        while todo:
            v = todo.pop()
            for w in edges[v]:
                if w not in seen:
                    seen.add(w); todo.append(w)
        return seen

    res = []
    for i in range(n):
        for t in succ[i]:
            if t <= i:                                 # back edge candidate (latch i -> header t)
                body = reach(t, succ) & reach(i, pred)
                if i in body and t in body:
                    res.append(body)
    inside = set()
    for c in res:
        inside |= c
    return inside, len(res)


def analyse(funcs, name, memo, notes) -> float:
    if name in memo:
        return memo[name]
    memo[name] = 0.0                      # recursion guard
    ins = funcs[name]
    base = [cost(m, o) for _, m, o, _ in ins]
    for i, (_, m, o, rel) in enumerate(ins):
        if m.split(".")[0] in ("bl", "blx") or (BRANCH.match(m) and rel):
            c = rel or ""
            if not c:
                continue
            key = c.split("(")[0]
            if c == "<indirect>":
                notes["indirect"].append(f"{name} @0x{ins[i][0]:x}"); continue
            if FORBIDDEN.search(c):
                notes["forbidden"].append(f"{name} -> {c}"); base[i] += 1000
            elif key in LIB_COST:
                base[i] += LIB_COST[key]; notes["lib"].append(f"{key} ({LIB_COST[key]} cyc)")
            elif c in funcs:
                base[i] += analyse(funcs, c, memo, notes)
            else:
                key = c.split("(")[0]
                if key in LIB_COST:
                    base[i] += LIB_COST[key]; notes["lib"].append(f"{key} ({LIB_COST[key]} cyc)")
                else:
                    notes["forbidden"].append(f"{name} -> {c} (unknown)"); base[i] += 1000
    inside, nloops = loop_blocks(ins)
    total = float(sum(c * (16 if i in inside else 1) for i, c in enumerate(base)))
    notes["loops"].append(f"{name}: {nloops} loop(s), {len(ins)} instructions")
    memo[name] = total
    return total


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gcc")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--elf", type=Path, help="linked firmware image (arduino-cli build path/src.ino.elf)")
    args = ap.parse_args(argv)
    gxx = find_gcc(args.gcc)
    objdump = gxx.with_name(gxx.name.replace("g++", "objdump"))
    if args.elf:
        return elf_main(args, objdump)
    if args.json is None:
        args.json = REPO / "telemetry" / "results" / "innovation" / "arm_cycles.json"
    with tempfile.TemporaryDirectory() as td:
        obj = Path(td) / "probe.o"
        cp = subprocess.run([str(gxx), *FLAGS, "-I", str(REPO / "firmware" / "src"), str(PROBE), "-o", str(obj)],
                            capture_output=True, text=True)
        warnings = [l for l in (cp.stdout + cp.stderr).splitlines() if "warning" in l]
        if cp.returncode != 0:
            print(cp.stdout + cp.stderr)
            return 1
        dis = subprocess.run([str(objdump), "-dr", "-C", "--no-show-raw-insn", str(obj)], capture_output=True, text=True).stdout
    funcs = parse(dis)
    out, ok = {"gcc": portable(gxx), "flags": FLAGS, "compiler_warnings": warnings, "probes": {}}, not warnings
    print(f"arm-none-eabi-g++: {gxx}\ncompiler warnings: {len(warnings)}")
    for w in warnings:
        print("  " + w)
    for probe in ("probe_v2", "probe_db", "probe_worst"):
        notes = {"forbidden": [], "lib": [], "loops": [], "indirect": []}
        cyc = analyse(funcs, probe, {}, notes)
        bound = 2 * cyc
        us = bound / F_CPU * 1e6
        passed = bound <= BUDGET_CYCLES and not notes["forbidden"]
        ok &= passed
        out["probes"][probe] = {"cycles_zero_wait": cyc, "cycles_bound_x2": bound, "us_bound": us,
                                "pass": passed, **{k: sorted(set(v)) for k, v in notes.items()}}
        print(f"  [{'PASS' if passed else 'FAIL'}] {probe}: {cyc:.0f} cycles (0 wait), bound x2 = {bound:.0f} cycles = "
              f"{us:.1f} us of the 250 us budget; library calls: {sorted(set(notes['lib'])) or 'none'}; "
              f"forbidden: {notes['forbidden'] or 'none'}")
        for l in sorted(set(notes["loops"])):
            print(f"        {l}")
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(out, indent=2))
    return 0 if ok else 1


ELF_ROOTS = [  # (label, [functions charged together], rate Hz, NVIC priority)
    ("control_isr", ["controlIsr()"], 500.0, 1),
    ("led_matrix_isr", ["gpt_counter_overflow_isr", "ArduinoLEDMatrix::turnOnLedISR(st_timer_callback_args*)"], 10000.0, 12),
    ("millis_agt0_isr", ["agt_int_isr", "timer_micros_callback(st_timer_callback_args*)"], 1000.0, 8),
]
ENTRY_EXIT_CYCLES = 12 + 12 + 17 + 17     # exception entry + exit (TRM), plus lazy FP stacking of s0-s15/FPSCR both ways


def elf_main(args, objdump: Path) -> int:
    if args.json is None:
        args.json = REPO / "telemetry" / "results" / "fabric" / "isr_cycles.json"
    dis = subprocess.run([str(objdump), "-d", "-C", "--no-show-raw-insn", str(args.elf)],
                         capture_output=True, text=True).stdout
    funcs = parse_elf(dis)
    out, ok = {"elf": args.elf.name, "f_cpu": F_CPU, "entry_exit_cycles": ENTRY_EXIT_CYCLES, "roots": {}}, True
    load = 0.0
    for label, names, rate, prio in ELF_ROOTS:
        missing = [n for n in names if n not in funcs]
        if missing:
            print(f"  [FAIL] {label}: symbol(s) not found in ELF: {missing}")
            ok = False
            continue
        notes = {"forbidden": [], "lib": [], "loops": [], "indirect": []}
        cyc = sum(analyse(funcs, n, {}, notes) for n in names) + ENTRY_EXIT_CYCLES * len(names[:1])
        bound = 2 * cyc
        us = bound / F_CPU * 1e6
        frac = bound / F_CPU * rate
        entry = {"functions": names, "rate_hz": rate, "nvic_priority": prio, "cycles_zero_wait": cyc,
                 "cycles_bound_x2": bound, "us_bound": us, "cpu_fraction_bound": frac,
                 **{k: sorted(set(v)) for k, v in notes.items()}}
        if label == "control_isr":
            entry["pass"] = bound <= BUDGET_CYCLES and not notes["forbidden"]
            ok &= entry["pass"]
            tag = "PASS" if entry["pass"] else "FAIL"
        else:
            load += frac
            tag = "INFO"
        out["roots"][label] = entry
        print(f"  [{tag}] {label} ({rate:.0f} Hz, prio {prio}): {cyc:.0f} cycles (0 wait), bound x2 = {bound:.0f} "
              f"cycles = {us:.1f} us, CPU bound {100 * frac:.2f} %; lib: {sorted(set(notes['lib'])) or 'none'}; "
              f"forbidden: {notes['forbidden'] or 'none'}; indirect: {sorted(set(notes['indirect'])) or 'none'}")
        for l in sorted(set(notes["loops"])):
            if not l.endswith(" 0 loop(s)") and " 0 loop(s)," not in l:
                print(f"        {l}")
    out["lower_priority_cpu_fraction_bound"] = load
    print(f"  lower-priority ISR load (bound): {100 * load:.2f} % of the CPU")
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(out, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
