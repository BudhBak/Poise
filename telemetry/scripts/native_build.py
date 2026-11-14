"""Cross-platform native build for the SymbioGrip simulation targets.

Targets (written to firmware/sim/build/):
  * bench : test_bench executable (multi-rate SIL benchmark)
  * lib   : libhaptic shared library (.dll / .dylib / .so) for ctypes (desktop bench, verification)
  * lab   : innovation_lab executable (v3 R&D candidates, telemetry/scripts/analyze_innovations.py)

Toolchain discovery order:
  Windows : MSVC (located with vswhere, environment from vcvars64.bat) -> g++ (MinGW/MSYS2) -> clang++
  macOS   : clang++ -> g++
  Linux   : g++ -> clang++

Usage:  python native_build.py [--force] [bench|lib|lab|all]
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_DIR = REPO_ROOT / "firmware" / "sim"
SRC_DIR = REPO_ROOT / "firmware" / "src"
BUILD_DIR = SIM_DIR / "build"

DEPENDENCIES = [SRC_DIR / "admittance.h", SRC_DIR / "config.h", SIM_DIR / "plant.h", SIM_DIR / "operator_model.h",
                SIM_DIR / "innovation_engine.h"]
SOURCES = {"bench": SIM_DIR / "test_bench.cpp", "lib": SIM_DIR / "hil_bridge.cpp", "lab": SIM_DIR / "innovation_lab.cpp"}


def lib_filename() -> str:
    if sys.platform.startswith("win"):
        return "libhaptic.dll"
    if sys.platform == "darwin":
        return "libhaptic.dylib"
    return "libhaptic.so"


def output_path(target: str) -> Path:
    if target == "lib":
        return BUILD_DIR / lib_filename()
    stem = "innovation_lab" if target == "lab" else "test_bench"
    return BUILD_DIR / (stem + ".exe" if sys.platform.startswith("win") else stem)


def _find_vcvars() -> Path | None:
    installer = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "Microsoft Visual Studio" / "Installer"
    vswhere = installer / "vswhere.exe"
    if not vswhere.exists():
        return None
    try:
        out = subprocess.run(
            [str(vswhere), "-products", "*", "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
             "-property", "installationPath", "-sort"],
            capture_output=True, text=True, check=True,
        ).stdout.splitlines()
    except (OSError, subprocess.CalledProcessError):
        return None
    for root in (line.strip() for line in out):
        if not root:
            continue
        cand = Path(root) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
        if cand.exists():
            return cand
    return None


def find_toolchain() -> tuple[str, str]:
    """Return (kind, locator). kind in {'msvc','gnu','clang'}."""
    order: list[str]
    if sys.platform.startswith("win"):
        order = ["msvc", "gnu", "clang"]
    elif sys.platform == "darwin":
        order = ["clang", "gnu"]
    else:
        order = ["gnu", "clang"]
    for kind in order:
        if kind == "msvc":
            vc = _find_vcvars()
            if vc:
                return "msvc", str(vc)
        elif kind == "gnu" and shutil.which("g++"):
            return "gnu", shutil.which("g++")
        elif kind == "clang" and shutil.which("clang++"):
            return "clang", shutil.which("clang++")
    raise RuntimeError(
        "No C++17 compiler found. Install one of: Visual Studio 2019+ with 'Desktop development with C++' "
        "(Windows), MSYS2 mingw-w64 g++ (Windows), Xcode command line tools (macOS), or g++ (Linux)."
    )


def _stale(target: str) -> bool:
    out = output_path(target)
    if not out.exists():
        return True
    newest = max(p.stat().st_mtime for p in DEPENDENCIES + [SOURCES[target]])
    return out.stat().st_mtime < newest


def build(target: str, force: bool = False, verbose: bool = True) -> Path:
    if target not in SOURCES:
        raise ValueError(target)
    out = output_path(target)
    if not force and not _stale(target):
        return out
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    kind, loc = find_toolchain()
    src = SOURCES[target]
    if kind == "msvc":
        obj_dir = BUILD_DIR / f"obj_{target}"
        obj_dir.mkdir(exist_ok=True)
        link = "/LD" if target == "lib" else ""
        obj_file = obj_dir / (src.stem + ".obj")
        cl = f'cl /nologo /O2 /EHsc /std:c++17 /W3 {link} "{src}" /Fe:"{out}" /Fo:"{obj_file}"'
        env = dict(os.environ)
        installer = Path(env.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "Microsoft Visual Studio" / "Installer"
        system32 = Path(env.get("SystemRoot", r"C:\Windows")) / "System32"
        env["PATH"] = os.pathsep.join([str(installer), str(system32), env.get("PATH", "")])
        bat = BUILD_DIR / f"build_{target}.bat"
        bat.write_text(f'@echo off\r\ncall "{loc}" >nul\r\nif errorlevel 1 exit /b 1\r\n{cl}\r\n', encoding="ascii")
        proc = subprocess.run(["cmd", "/d", "/c", str(bat)], capture_output=True, text=True, env=env)
    else:
        args = [loc, "-O2", "-std=c++17", "-Wall", "-Wextra", str(src), "-o", str(out)]
        if target == "lib":
            if sys.platform == "darwin":
                args.insert(1, "-dynamiclib")
            else:
                args[1:1] = ["-shared", "-fPIC"]
            if sys.platform.startswith("win"):
                args += ["-static-libgcc", "-static-libstdc++"]
        proc = subprocess.run(args, capture_output=True, text=True)
    if proc.returncode != 0 or not out.exists():
        sys.stderr.write(proc.stdout + proc.stderr)
        raise RuntimeError(f"build of {target} failed with exit code {proc.returncode} ({kind})")
    if verbose:
        print(f"[native_build] built {target} with {kind}: {out}")
    return out


def main(argv: list[str]) -> int:
    if "-h" in argv or "--help" in argv:
        print(__doc__)
        return 0
    force = "--force" in argv
    targets = [a for a in argv if not a.startswith("--")] or ["all"]
    if targets == ["all"]:
        targets = ["bench", "lib", "lab"]
    for t in targets:
        p = build(t, force=force)
        print(p)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
