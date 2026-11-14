"""SymbioGrip desktop human-in-the-loop bench (trackpad / mouse / pen).

WHAT THIS IS: a real human steers a simulated stylus through the simulated wire-loop corridor while the
UNMODIFIED firmware HapticEngine (firmware/src/admittance.h, via libhaptic) closes the loop on the SAME plant
used by the SIL benchmark (firmware/sim/plant.h). The pointer is the hand's equilibrium point; the hand is a
spring-damper coupling to the stylus, so the engine's damping is felt as lag/drag of the stylus, not as force.

WHAT THIS IS NOT: hardware-in-the-loop. No Arduino, motor, encoder or FSR is involved. Grip is not measured:
it comes from pen pressure (PointerEvent.pressure) when the device reports it, otherwise from holding the
SPACE bar ("squeeze"). Data from this bench are labelled 'desktop_hitl' and must never be reported as
hardware results.

Protocol: N blocks; each block runs A, B and C once in an order drawn from OS entropy (secrets.SystemRandom).
Blind mode (default) hides mode, damping, difficulty, grip and contact counts during and after trials.
Trials start when the stylus leaves the start gate and end at the finish line (or 30 s timeout).
Validity rule (pre-registered, applied by evaluator.py): MT >= 1.0 s and mean speed <= 0.25 m/s.

Output: telemetry/data_hil/<session>/trial_mode_<M>_run_<block>_hitl.csv (same schema as the SIL CSVs),
        manifest.json (evaluator-compatible) and session.json (protocol, order, validity, environment).

Usage:
  python hil_trackpad_bench.py [--blocks 5] [--port 8765] [--session NAME] [--practice]
  python hil_trackpad_bench.py --selftest      # headless scripted trial per mode + HTTP probe; writes to a temp dir
"""
from __future__ import annotations

import argparse
import json
import math
import platform
import secrets
import sys
import tempfile
import threading
import time
import urllib.request
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import haptic_native as hn  # noqa: E402

REPO = SCRIPT_DIR.parents[1]
CFG = hn.config()
DT = CFG["control_dt"]
TRACK_LEN, CORRIDOR_R, A_WIRE, LAMBDA = 0.180, 0.00135, 0.0023, 0.060   # must match plant.h PlantParams
START_GATE_X = 0.004          # trial clock starts when stylus passes 4 mm
TIMEOUT_S = 30.0
LOG_DIV = 5                   # 100 Hz rows, as in SIL
HAND = dict(kx=110.0, bx=5.5, ky=180.0, by=5.5)   # pointer-to-stylus coupling (limb impedance proxy)
GRIP_REST_N, GRIP_SQUEEZE_N = 2.0, 7.0
FSR_SCALE_N = 1.0             # FSR model F = S * a / (1 - a)  <=>  a = F / (S + F)   (config.h)
VALID_MIN_MT, VALID_MAX_SPEED = 1.0, 0.25
HEADER = ("t_ms,mode,grip_adc,enc_counts,vel_mps,damping_Nspm,difficulty,tremor_rms,duty,energy_mJ,contacts,"
          "pos_m,y_m,lat_err_m,grip_true_N,motor_force_N,motor_current_A,pc_beta")


def grip_to_adc(force_n: float) -> int:
    a = force_n / (FSR_SCALE_N + force_n)
    return max(0, min(16383, int(round(a * 16383))))


class Trial:
    """One trial: engine + plant stepped at 500 Hz against the latest pointer sample."""

    def __init__(self, mode: str, block: int, out_dir: Path | None):
        self.mode, self.block = mode, block
        self.plant = hn.Plant()
        self.engine = hn.Engine()
        self.engine.set_mode(mode, self.plant.encoder())
        self.tick = 0
        self.started = False
        self.t0_tick = 0
        self.done = False
        self.timed_out = False
        self.rows: list[str] = []
        self.out_dir = out_dir
        self.grip_n = GRIP_REST_N

    def step(self, px: float, py: float, grip_n: float):
        if self.done:
            return
        self.grip_n += 0.1 * (grip_n - self.grip_n)          # ~20 ms first-order hand-grip response
        cmd = self.engine.step(grip_to_adc(self.grip_n), self.plant.encoder())
        self.plant.advance(x_eq=px, y_eq=py, cmd=cmd, **HAND)
        st = self.plant.state()
        if not self.started and st["x"] >= START_GATE_X:
            self.started, self.t0_tick = True, self.tick
        if self.started:
            k = self.tick - self.t0_tick
            if k % LOG_DIV == 0 or st["finished"]:
                e = self.engine.state()
                self.rows.append(
                    f"{(k + 1) * 2},{self.mode},{grip_to_adc(self.grip_n)},{self.plant.encoder()},{e['velocity']:.5f},"
                    f"{e['damping']:.3f},{e['difficulty']:.4f},{e['tremor_rms']:.4f},{cmd.duty if not cmd.coast else 0.0:.4f},"
                    f"{e['energy_j'] * 1000:.4f},{int(st['contacts'])},{st['x']:.6f},{st['y']:.7f},{st['lat_err']:.7f},"
                    f"{self.grip_n:.4f},{st['motor_force']:.4f},{st['motor_current']:.4f},{e['beta']:.3f}")
            if st["finished"] or (self.tick - self.t0_tick) * DT > TIMEOUT_S:
                self.done, self.timed_out = True, not st["finished"]
        self.tick += 1

    def summary(self) -> dict:
        st = self.plant.state()
        mt = (self.tick - self.t0_tick) * DT if self.started else 0.0
        speed = (st["x"] - START_GATE_X) / mt if mt > 0 else 0.0
        return {"mode": self.mode, "block": self.block, "completed": bool(st["finished"]), "timed_out": self.timed_out,
                "mt_s": round(mt, 3), "mean_speed_mps": round(speed, 4), "contacts": int(st["contacts"]),
                "valid": bool(st["finished"] and mt >= VALID_MIN_MT and speed <= VALID_MAX_SPEED),
                "pc_activations": int(self.engine.state()["pc_activations"])}

    def save(self) -> Path | None:
        if self.out_dir is None or not self.rows:
            return None
        p = self.out_dir / f"trial_mode_{self.mode}_run_{self.block}_hitl.csv"
        p.write_text(HEADER + "\n" + "\n".join(self.rows) + "\n", encoding="utf-8")
        return p


class Session:
    def __init__(self, blocks: int, out_dir: Path | None, blind: bool, practice: bool):
        rng = secrets.SystemRandom()
        self.order = []
        for b in range(1, blocks + 1):
            modes = list("ABC")
            rng.shuffle(modes)
            self.order += [(m, b) for m in modes]
        if practice:
            self.order = [("A", 0), ("B", 0), ("C", 0)]
        self.blind, self.practice = blind, practice
        self.out_dir = out_dir
        self.idx = 0
        self.trial: Trial | None = None
        self.summaries: list[dict] = []
        self.pointer = (0.0, 0.0)
        self.grip = GRIP_REST_N
        self.lock = threading.Lock()
        self.running = True
        self.started_utc = datetime.now(timezone.utc).isoformat()
        if out_dir:
            out_dir.mkdir(parents=True, exist_ok=True)
            self._write_manifest()

    # ---- control ----
    def arm_next(self) -> bool:
        with self.lock:
            if self.idx >= len(self.order):
                return False
            m, b = self.order[self.idx]
            self.trial = Trial(m, b, None if self.practice else self.out_dir)
            return True

    def finish_trial(self):
        t = self.trial
        s = t.summary()
        s["file"] = (p.name if (p := t.save()) else None)
        self.summaries.append(s)
        self.idx += 1
        self.trial = None
        if self.out_dir:
            self._write_manifest()

    def loop(self):
        """Real-time 500 Hz stepping with catch-up (the pointer sample is held between browser updates)."""
        sim_t, wall0 = 0.0, time.perf_counter()
        while self.running:
            now = time.perf_counter() - wall0
            n = 0
            with self.lock:
                while sim_t < now and n < 50:
                    if self.trial is not None and not self.trial.done:
                        self.trial.step(self.pointer[0], self.pointer[1], self.grip)
                        if self.trial.done:
                            self.finish_trial()
                    sim_t += DT
                    n += 1
                if n == 50:
                    sim_t = now   # fell behind (e.g. OS stall): drop time rather than replay stale input
            time.sleep(0.001)

    # ---- views ----
    def view(self) -> dict:
        with self.lock:
            v = {"blind": self.blind, "practice": self.practice, "index": self.idx, "total": len(self.order),
                 "armed": self.trial is not None, "geometry": {"len": TRACK_LEN, "r": CORRIDOR_R, "a": A_WIRE, "lambda": LAMBDA}}
            if self.trial is not None:
                st = self.trial.plant.state()
                v.update({"x": st["x"], "y": st["y"], "started": self.trial.started})
                if not self.blind:
                    e = self.trial.engine.state()
                    v.update({"mode": self.trial.mode, "damping": e["damping"], "difficulty": e["difficulty"],
                              "grip_n": self.trial.grip_n, "contacts": int(st["contacts"]), "contact_now": bool(st["contact_now"])})
            if self.summaries and not self.blind:
                v["last"] = self.summaries[-1]
            v["finished_all"] = self.idx >= len(self.order)
            return v

    def _write_manifest(self):
        man = {"generator": "telemetry/scripts/hil_trackpad_bench.py", "data_kind": "desktop_hitl (NOT hardware)",
               "counts_per_meter": CFG["counts_per_meter"],
               "plant": {"track_len_m": TRACK_LEN, "corridor_half_width_m": CORRIDOR_R,
                         "wire_amplitude_m": A_WIRE, "wire_wavelength_m": LAMBDA},
               "control_hz": 500, "log_hz": 500 // LOG_DIV, "trials": self.summaries}
        (self.out_dir / "manifest.json").write_text(json.dumps(man, indent=2))
        ses = {"started_utc": self.started_utc, "blind": self.blind, "order_source": "secrets.SystemRandom (OS entropy)",
               "order": [{"mode": m, "block": b} for m, b in self.order], "hand_coupling": HAND,
               "grip_input": {"rest_n": GRIP_REST_N, "squeeze_n": GRIP_SQUEEZE_N, "source": "pen pressure or SPACE key"},
               "validity_rule": {"min_mt_s": VALID_MIN_MT, "max_mean_speed_mps": VALID_MAX_SPEED},
               "evaluate_with": f"python telemetry/scripts/evaluator.py --data <this dir> --min-mt {VALID_MIN_MT} --max-speed {VALID_MAX_SPEED}",
               "environment": {"python": platform.python_version(), "os": platform.platform()},
               "summaries": self.summaries}
        (self.out_dir / "session.json").write_text(json.dumps(ses, indent=2))


PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SymbioGrip Desktop Bench</title><style>
:root{--bg:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--wire:#2a78d6;--wall:#e4e3df;--stylus:#eb6834;--hit:#e34948}
@media (prefers-color-scheme:dark){:root{--bg:#1a1a19;--ink:#f0efec;--ink2:#c3c2b7;--wire:#3987e5;--wall:#3a3a38;--stylus:#d95926;--hit:#e66767}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px system-ui,sans-serif}
header{padding:12px 16px;display:flex;gap:16px;align-items:center;flex-wrap:wrap}
button{font:inherit;padding:6px 14px;border-radius:6px;border:1px solid var(--ink2);background:transparent;color:var(--ink);cursor:pointer}
canvas{display:block;width:100%;touch-action:none;cursor:crosshair}#info{color:var(--ink2)}
</style></head><body><header><strong>SymbioGrip desktop bench</strong><button id="go">Start next trial</button>
<span id="info">Loading…</span></header><canvas id="c"></canvas><script>
const cv=document.getElementById('c'),cx=cv.getContext('2d'),info=document.getElementById('info');
let V=null,px=0,py=0,pressure=null,space=false,scale=1,ox=40,oy=0;
function css(n){return getComputedStyle(document.documentElement).getPropertyValue(n)}
function resize(){cv.width=cv.clientWidth*devicePixelRatio;cv.height=Math.min(innerHeight-60,420)*devicePixelRatio;cv.style.height=(cv.height/devicePixelRatio)+'px'}
addEventListener('resize',resize);resize();
addEventListener('keydown',e=>{if(e.code==='Space'){space=true;e.preventDefault()}});
addEventListener('keyup',e=>{if(e.code==='Space')space=false});
cv.addEventListener('pointermove',e=>{const r=cv.getBoundingClientRect();const X=(e.clientX-r.left)*devicePixelRatio,Y=(e.clientY-r.top)*devicePixelRatio;
 px=(X-ox)/scale;py=-(Y-oy)/scale;if(e.pointerType==='pen'&&e.pressure>0)pressure=e.pressure});
document.getElementById('go').onclick=()=>fetch('/arm',{method:'POST'});
async function tick(){
 const grip=pressure!==null?2+5*pressure:(space?7:2);
 try{const r=await fetch('/input',{method:'POST',body:JSON.stringify({x:px,y:py,grip})});V=await r.json()}catch(e){}
 draw();requestAnimationFrame(tick)}
function draw(){if(!V)return;const g=V.geometry;scale=(cv.width-2*ox)/g.len;oy=cv.height/2;
 cx.fillStyle=css('--bg');cx.fillRect(0,0,cv.width,cv.height);
 const N=400;cx.lineWidth=2*g.r*scale;cx.strokeStyle=css('--wall');cx.lineCap='round';cx.beginPath();
 for(let i=0;i<=N;i++){const x=g.len*i/N,y=g.a*Math.sin(2*Math.PI*x/g.lambda);i?cx.lineTo(ox+x*scale,oy-y*scale):cx.moveTo(ox+x*scale,oy-y*scale)}cx.stroke();
 cx.lineWidth=2;cx.strokeStyle=css('--wire');cx.stroke();
 cx.strokeStyle=css('--ink2');cx.lineWidth=1;cx.setLineDash([4,4]);cx.beginPath();cx.moveTo(ox+g.len*scale,0);cx.lineTo(ox+g.len*scale,cv.height);cx.stroke();cx.setLineDash([]);
 if(V.armed){const hit=!V.blind&&V.contact_now;cx.fillStyle=hit?css('--hit'):css('--stylus');cx.beginPath();cx.arc(ox+V.x*scale,oy-V.y*scale,5*devicePixelRatio,0,7);cx.fill()}
 let t=`Trial ${Math.min(V.index+1,V.total)} of ${V.total}`+(V.practice?' (practice, not saved)':'');
 if(V.finished_all)t='Session complete. You can close this tab.';
 else if(!V.armed)t+=' — press "Start next trial", then follow the blue wire left to right.';
 else if(!V.started)t+=' — move right to begin.';
 if(!V.blind&&V.armed)t+=` | mode ${V.mode} | damping ${V.damping.toFixed(1)} N·s/m | difficulty ${V.difficulty.toFixed(2)} | grip ${V.grip_n.toFixed(1)} N | contacts ${V.contacts}`;
 t+=pressure!==null?' | grip: pen pressure':' | grip: hold SPACE to squeeze';info.textContent=t}
tick();
</script></body></html>"""


def make_handler(sess: Session):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code: int, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif self.path == "/state":
                self._send(200, json.dumps(sess.view()).encode(), "application/json")
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(n) if n else b""
            if self.path == "/input":
                try:
                    d = json.loads(body)
                    x, y, g = float(d["x"]), float(d["y"]), float(d["grip"])
                    if all(math.isfinite(v) for v in (x, y, g)):
                        with sess.lock:
                            sess.pointer = (max(-0.05, min(0.25, x)), max(-0.05, min(0.05, y)))
                            sess.grip = max(0.0, min(20.0, g))
                except (ValueError, KeyError, TypeError):
                    pass
                self._send(200, json.dumps(sess.view()).encode(), "application/json")
            elif self.path == "/arm":
                ok = sess.trial is None and sess.arm_next()
                self._send(200, json.dumps({"armed": ok}).encode(), "application/json")
            else:
                self._send(404, b"not found", "text/plain")
    return H


def selftest() -> int:
    """Headless: one scripted trial per mode through the real engine/plant, then an HTTP probe."""
    ok = True
    with tempfile.TemporaryDirectory() as td:
        out = Path(td)
        sess = Session(1, out, blind=True, practice=False)
        k_wave = 2 * math.pi / LAMBDA
        for _ in range(3):
            sess.arm_next()
            t = sess.trial
            for k in range(int(20 / DT)):
                x = min(0.19, 0.05 * k * DT)                     # pointer glides at 50 mm/s along the wire
                y = A_WIRE * math.sin(k_wave * x)
                t.step(x, y, GRIP_REST_N + (3.0 if (k // 500) % 2 else 0.0))
                if t.done:
                    break
            sess.finish_trial()
        for s in sess.summaries:
            good = s["completed"] and s["file"] and (out / s["file"]).exists()
            print(f"  [{'PASS' if good else 'FAIL'}] scripted trial mode {s['mode']}: completed={s['completed']}, "
                  f"MT={s['mt_s']} s, contacts={s['contacts']}, valid={s['valid']}, file={s['file']}")
            ok &= bool(good)
        files = sorted(p.name for p in out.iterdir())
        good = {"manifest.json", "session.json"} <= set(files)
        print(f"  [{'PASS' if good else 'FAIL'}] manifest.json + session.json written ({len(files)} files)")
        ok &= good
        hdr = (out / sess.summaries[0]["file"]).read_text().splitlines()[0]
        print(f"  [{'PASS' if hdr == HEADER else 'FAIL'}] CSV header matches SIL schema")
        ok &= hdr == HEADER

    sess = Session(1, None, blind=True, practice=True)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(sess))
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=sess.loop, daemon=True).start()
    try:
        page = urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5).read().decode()
        urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{port}/arm", method="POST"), timeout=5).read()
        st = json.loads(urllib.request.urlopen(urllib.request.Request(
            f"http://127.0.0.1:{port}/input", data=b'{"x":0.02,"y":0,"grip":2}', method="POST"), timeout=5).read())
        time.sleep(0.3)
        st = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/state", timeout=5).read())
        good = "<canvas" in page and st["armed"] and "mode" not in st and "damping" not in st and st["x"] > 0.001
        print(f"  [{'PASS' if good else 'FAIL'}] HTTP probe: page served, trial armed, stylus moved to x={st['x'] * 1000:.1f} mm, "
              f"blind view hides mode/damping: {'mode' not in st and 'damping' not in st}")
        ok &= bool(good)
    finally:
        sess.running = False
        srv.shutdown()
    print(f"selftest {'PASSED' if ok else 'FAILED'}")
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--blocks", type=int, default=5)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--session", default=None, help="session folder name (default: UTC timestamp)")
    ap.add_argument("--unblind", action="store_true", help="show mode/damping/contacts (pilot debugging only)")
    ap.add_argument("--practice", action="store_true", help="3 unsaved familiarisation trials, unblinded")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()
    name = args.session or datetime.now(timezone.utc).strftime("session_%Y%m%dT%H%M%SZ")
    out = None if args.practice else REPO / "telemetry" / "data_hil" / name
    sess = Session(args.blocks, out, blind=not (args.unblind or args.practice), practice=args.practice)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(sess))
    threading.Thread(target=sess.loop, daemon=True).start()
    print(f"SymbioGrip desktop bench on http://127.0.0.1:{args.port}/  ({len(sess.order)} trials, "
          f"{'blind' if sess.blind else 'UNBLINDED'}; output: {out or 'not saved'})  Ctrl+C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sess.running = False
    return 0


if __name__ == "__main__":
    sys.exit(main())
