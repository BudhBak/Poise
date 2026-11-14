"""ctypes wrapper around firmware/sim/hil_bridge.cpp (auto-built by native_build.py).

Exposes the unmodified firmware HapticEngine and the shared Plant to Python.
"""
from __future__ import annotations

import ctypes as C
from dataclasses import dataclass

import native_build

ENGINE_FIELDS = ("position", "velocity", "damping", "difficulty", "tremor_rms", "grip_n",
                 "energy_j", "beta", "force_cmd", "pc_activations", "fault")
PLANT_FIELDS = ("x", "vx", "y", "vy", "lat_err", "motor_force", "motor_current", "contacts",
                "contact_now", "finished", "motor_work_j")
CONFIG_FIELDS = ("counts_per_meter", "control_dt", "damping_min", "damping_max", "damping_fixed",
                 "tremor_floor", "tremor_span", "motor_kt_eff", "motor_r", "supply_v", "i_max",
                 "vel_deadband", "pc_beta_max", "b_friction_est")

_lib = None


def lib() -> C.CDLL:
    global _lib
    if _lib is None:
        path = native_build.build("lib", verbose=False)
        L = C.CDLL(str(path))
        vp, f, i32, u16, d = C.c_void_p, C.c_float, C.c_int32, C.c_uint16, C.POINTER(C.c_double)
        sig = {
            "hb_abi_version": ([], C.c_int),
            "hb_engine_create": ([], vp),
            "hb_engine_create_custom": ([f, f, f, f, f], vp),
            "hb_engine_destroy": ([vp], None),
            "hb_engine_set_mode": ([vp, C.c_char, i32], None),
            "hb_engine_reset": ([vp, i32], None),
            "hb_engine_set_pc_enabled": ([vp, i32], None),
            "hb_engine_step": ([vp, u16, i32, C.POINTER(i32)], f),
            "hb_engine_state": ([vp, d], None),
            "hb_plant_create": ([], vp),
            "hb_plant_destroy": ([vp], None),
            "hb_plant_reset": ([vp], None),
            "hb_plant_advance": ([vp, f, f, f, f, f, f, f, f, f, i32], None),
            "hb_plant_state": ([vp, d], None),
            "hb_plant_encoder": ([vp], i32),
            "hb_plant_wire_y": ([vp, f], f),
            "hb_plant_prescribe": ([vp, C.c_double, f, i32], None),
            "hb_config": ([d], None),
        }
        for name, (args, res) in sig.items():
            fn = getattr(L, name)
            fn.argtypes = args
            fn.restype = res
        if L.hb_abi_version() != 2:
            raise RuntimeError("libhaptic ABI mismatch; delete firmware/sim/build and rebuild")
        _lib = L
    return _lib


def config() -> dict:
    buf = (C.c_double * len(CONFIG_FIELDS))()
    lib().hb_config(buf)
    return dict(zip(CONFIG_FIELDS, list(buf)))


@dataclass
class Command:
    duty: float
    coast: bool


class Engine:
    def __init__(self, actuator: tuple[float, float, float, float, float] | None = None):
        self._h = lib().hb_engine_create_custom(*actuator) if actuator else lib().hb_engine_create()
        self._buf = (C.c_double * len(ENGINE_FIELDS))()
        self._coast = C.c_int32(1)

    def close(self):
        if self._h:
            lib().hb_engine_destroy(self._h)
            self._h = None

    __del__ = close

    def set_mode(self, mode: str, counts: int) -> None:
        lib().hb_engine_set_mode(self._h, mode.encode("ascii"), int(counts))

    def reset(self, counts: int) -> None:
        lib().hb_engine_reset(self._h, int(counts))

    def set_pc_enabled(self, on: bool) -> None:
        lib().hb_engine_set_pc_enabled(self._h, 1 if on else 0)

    def step(self, grip_adc: int, counts: int) -> Command:
        duty = lib().hb_engine_step(self._h, int(grip_adc), int(counts), C.byref(self._coast))
        return Command(float(duty), bool(self._coast.value))

    def state(self) -> dict:
        lib().hb_engine_state(self._h, self._buf)
        return dict(zip(ENGINE_FIELDS, list(self._buf)))


class Plant:
    def __init__(self):
        self._h = lib().hb_plant_create()
        self._buf = (C.c_double * len(PLANT_FIELDS))()

    def close(self):
        if self._h:
            lib().hb_plant_destroy(self._h)
            self._h = None

    __del__ = close

    def reset(self) -> None:
        lib().hb_plant_reset(self._h)

    def advance(self, fx=0.0, x_eq=0.0, kx=0.0, bx=0.0, fy=0.0, y_eq=0.0, ky=0.0, by=0.0,
                cmd: Command = Command(0.0, True)) -> None:
        lib().hb_plant_advance(self._h, fx, x_eq, kx, bx, fy, y_eq, ky, by, cmd.duty, 1 if cmd.coast else 0)

    def prescribe(self, x_target: float, cmd: Command) -> None:
        lib().hb_plant_prescribe(self._h, float(x_target), cmd.duty, 1 if cmd.coast else 0)

    def encoder(self) -> int:
        return int(lib().hb_plant_encoder(self._h))

    def wire_y(self, x: float) -> float:
        return float(lib().hb_plant_wire_y(self._h, x))

    def state(self) -> dict:
        lib().hb_plant_state(self._h, self._buf)
        return dict(zip(PLANT_FIELDS, list(self._buf)))
