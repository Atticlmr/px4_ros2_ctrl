#!/usr/bin/env python3
# Copyright 2026 px4_ros2_ctrl contributors

import argparse
import ctypes
import os
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ACADOS_ROOT = PACKAGE_ROOT / "third_party" / "acados"
CASADI_LIB = PACKAGE_ROOT / "third_party" / "casadi" / "build" / "lib"
DEFAULT_JSON = PACKAGE_ROOT / "generated" / "body_rate_nmpc" / "body_rate_nmpc.json"


def configure_runtime():
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(ACADOS_ROOT))
    sys.path.insert(0, str(ACADOS_ROOT / "interfaces" / "acados_template"))
    sys.path.insert(0, str(CASADI_LIB))
    for lib_name in ("libqpOASES_e.so", "libblasfeo.so", "libhpipm.so", "libacados.so"):
        path = ACADOS_ROOT / "lib" / lib_name
        if path.exists():
            ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)


def main():
    parser = argparse.ArgumentParser(description="Smoke-test the generated body-rate NMPC solver.")
    parser.add_argument("--solver-json", type=Path, default=DEFAULT_JSON)
    args = parser.parse_args()

    configure_runtime()
    from acados_template import AcadosOcpSolver

    solver = AcadosOcpSolver(None, json_file=str(args.solver_json), build=False, generate=False)
    x0 = np.array([0.0, 0.0, -1.8, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    xref = np.array([0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    uref = np.array([0.0, 0.0, 0.0, 0.60])

    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)
    for stage in range(solver.N):
        solver.set(stage, "yref", np.concatenate([xref, uref]))
        solver.set(stage, "x", x0)
        solver.set(stage, "u", uref)
    solver.set(solver.N, "yref", xref)
    solver.set(solver.N, "x", xref)

    status = solver.solve()
    if status != 0:
        raise RuntimeError(f"acados solve failed with status {status}")
    u0 = solver.get(0, "u")
    print(f"body_rate_nmpc smoke test ok: u0={u0}")


if __name__ == "__main__":
    main()
