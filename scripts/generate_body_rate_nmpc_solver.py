#!/usr/bin/env python3
# Copyright 2026 px4_ros2_ctrl contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ACADOS_ROOT = PACKAGE_ROOT / "third_party" / "acados"
CASADI_LIB = PACKAGE_ROOT / "third_party" / "casadi" / "build" / "lib"
DEFAULT_OUTPUT_DIR = PACKAGE_ROOT / "generated" / "body_rate_nmpc"


def configure_python_paths():
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(ACADOS_ROOT))
    sys.path.insert(0, str(ACADOS_ROOT / "interfaces" / "acados_template"))
    sys.path.insert(0, str(CASADI_LIB))


def quaternion_to_rotation_body_to_ned(ca, q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
        ca.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)),
        ca.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)),
    )


def export_quadrotor_body_rate_model():
    import casadi as ca
    from acados_template import AcadosModel

    model = AcadosModel()
    model.name = "body_rate_nmpc"

    position = ca.SX.sym("p", 3)
    velocity = ca.SX.sym("v", 3)
    q = ca.SX.sym("q", 4)
    x = ca.vertcat(position, velocity, q)

    body_rates = ca.SX.sym("w", 3)
    thrust = ca.SX.sym("thrust")
    u = ca.vertcat(body_rates, thrust)

    xdot = ca.SX.sym("xdot", 10)

    mass = 2.0
    gravity = 9.80665
    hover_thrust = 0.60

    q_norm = q / ca.sqrt(ca.dot(q, q) + 1e-9)
    rotation_body_to_ned = quaternion_to_rotation_body_to_ned(ca, q_norm)

    force_body_frd = ca.vertcat(0.0, 0.0, -mass * gravity * thrust / hover_thrust)
    acceleration_ned = ca.vertcat(0.0, 0.0, gravity) + rotation_body_to_ned @ (force_body_frd / mass)

    wx, wy, wz = body_rates[0], body_rates[1], body_rates[2]
    qdot = 0.5 * ca.vertcat(
        -q[1] * wx - q[2] * wy - q[3] * wz,
        q[0] * wx + q[2] * wz - q[3] * wy,
        q[0] * wy - q[1] * wz + q[3] * wx,
        q[0] * wz + q[1] * wy - q[2] * wx,
    )

    f_expl = ca.vertcat(velocity, acceleration_ned, qdot)

    model.x = x
    model.xdot = xdot
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    model.x_labels = ["n", "e", "d", "vn", "ve", "vd", "qw", "qx", "qy", "qz"]
    model.u_labels = ["roll_rate", "pitch_rate", "yaw_rate", "thrust"]
    model.t_label = "t"
    return model


def build_ocp(output_dir: Path, horizon_steps: int, horizon_time: float):
    import casadi as ca
    from acados_template import AcadosOcp

    model = export_quadrotor_body_rate_model()
    ocp = AcadosOcp()
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()

    ocp.solver_options.N_horizon = horizon_steps
    ocp.solver_options.tf = horizon_time

    q_weights = np.array([18.0, 18.0, 24.0, 4.0, 4.0, 6.0, 8.0, 4.0, 4.0, 3.0])
    r_weights = np.array([0.08, 0.08, 0.10, 0.35])
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.cost.yref = np.zeros(nx + nu)
    ocp.cost.W = np.diag(np.concatenate([q_weights, r_weights]))

    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.yref_e = np.zeros(nx)
    ocp.cost.W_e = np.diag(q_weights * 2.0)

    ocp.constraints.x0 = np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.lbu = np.array([-3.0, -3.0, -1.5, 0.10])
    ocp.constraints.ubu = np.array([3.0, 3.0, 1.5, 0.90])

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.regularize_method = "CONVEXIFY"

    output_dir.mkdir(parents=True, exist_ok=True)
    ocp.code_gen_opts.code_export_directory = str(output_dir)
    ocp.code_gen_opts.json_file = str(output_dir / "body_rate_nmpc.json")
    return ocp


def main():
    parser = argparse.ArgumentParser(description="Generate the acados solver for the body-rate NMPC demo.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--horizon-steps", type=int, default=20)
    parser.add_argument("--horizon-time", type=float, default=1.0)
    args = parser.parse_args()

    configure_python_paths()
    from acados_template import AcadosOcpSolver

    ocp = build_ocp(args.output_dir.resolve(), args.horizon_steps, args.horizon_time)
    solver = AcadosOcpSolver(ocp, json_file=ocp.code_gen_opts.json_file, build=True, generate=True)
    status = solver.solve()
    if status != 0:
        raise RuntimeError(f"Initial acados solve failed with status {status}")
    print(f"Generated BODY_RATE NMPC solver: {ocp.code_gen_opts.json_file}")


if __name__ == "__main__":
    main()
