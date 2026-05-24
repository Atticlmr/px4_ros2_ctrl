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

import ctypes
import math
import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import VehicleOdometry, VehicleRatesSetpoint
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


SCRIPT_PATH = Path(__file__).resolve()
SOURCE_ROOT_CANDIDATES = [
    SCRIPT_PATH.parents[1],
    SCRIPT_PATH.parents[3] / "src" / "px4_ros2_ctrl" if len(SCRIPT_PATH.parents) > 3 else SCRIPT_PATH.parents[1],
    Path("/home/li/Desktop/ws_ros2/src/px4_ros2_ctrl"),
]


def find_source_root():
    for candidate in SOURCE_ROOT_CANDIDATES:
        if (candidate / "third_party" / "acados" / "lib" / "libacados.so").exists():
            return candidate
    return SOURCE_ROOT_CANDIDATES[0]


PACKAGE_ROOT = find_source_root()
ACADOS_ROOT = PACKAGE_ROOT / "third_party" / "acados"
CASADI_LIB = PACKAGE_ROOT / "third_party" / "casadi" / "build" / "lib"


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


class BodyRateNmpcController(Node):
    def __init__(self):
        super().__init__("body_rate_nmpc_controller")

        default_solver_json = self.default_solver_json()
        self.declare_parameter("solver_json", default_solver_json)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("hover_thrust", 0.60)
        self.declare_parameter("target_acceptance_radius_m", 0.45)
        self.declare_parameter("target_yaw_rad", 0.0)
        self.declare_parameter("targets_ned", [0.0, 0.0, -2.0, 3.0, 0.0, -2.0, 3.0, 3.0, -2.0, 0.0, 3.0, -2.0])

        self.hover_thrust = float(self.get_parameter("hover_thrust").value)
        self.target_acceptance_radius = float(self.get_parameter("target_acceptance_radius_m").value)
        self.target_yaw = float(self.get_parameter("target_yaw_rad").value)
        self.targets = self.parse_targets(self.get_parameter("targets_ned").value)
        self.target_index = 0
        self.latest_state = None
        self.last_solution_u = np.array([0.0, 0.0, 0.0, self.hover_thrust])

        self.solver = None
        self.load_solver(Path(str(self.get_parameter("solver_json").value)))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos,
        )
        self.output_pub = self.create_publisher(VehicleRatesSetpoint, "/controller/body_rate/output", 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("BODY_RATE NMPC controller started")
        self.get_logger().info("Publishing BODY_RATE output to /controller/body_rate/output")

    def default_solver_json(self):
        try:
            share = Path(get_package_share_directory("px4_ros2_ctrl"))
            installed_json = share / "generated" / "body_rate_nmpc" / "body_rate_nmpc.json"
            if installed_json.exists():
                return str(installed_json)
        except Exception:
            pass
        return str(PACKAGE_ROOT / "generated" / "body_rate_nmpc" / "body_rate_nmpc.json")

    def parse_targets(self, flat_targets):
        values = [float(v) for v in flat_targets]
        if len(values) < 3 or len(values) % 3 != 0:
            raise ValueError("targets_ned must contain triples: n,e,d")
        return [np.array(values[i : i + 3], dtype=float) for i in range(0, len(values), 3)]

    def configure_acados_runtime(self, solver_json):
        os.environ.setdefault("ACADOS_SOURCE_DIR", str(ACADOS_ROOT))
        sys.path.insert(0, str(ACADOS_ROOT / "interfaces" / "acados_template"))
        sys.path.insert(0, str(CASADI_LIB))
        for lib_name in ("libqpOASES_e.so", "libblasfeo.so", "libhpipm.so", "libacados.so"):
            path = ACADOS_ROOT / "lib" / lib_name
            if path.exists():
                ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
        solver_lib_dir = solver_json.parent
        if solver_lib_dir.exists():
            for lib in solver_lib_dir.glob("*.so"):
                ctypes.CDLL(str(lib), mode=ctypes.RTLD_GLOBAL)

    def load_solver(self, solver_json):
        if not solver_json.exists():
            self.get_logger().error(f"NMPC solver json not found: {solver_json}")
            return
        try:
            self.configure_acados_runtime(solver_json)
            from acados_template import AcadosOcpSolver

            self.solver = AcadosOcpSolver(None, json_file=str(solver_json), build=False, generate=False)
            self.get_logger().info(f"Loaded acados solver: {solver_json}")
        except Exception as exc:
            self.solver = None
            self.get_logger().error(f"Failed to load acados solver: {exc}")

    def odometry_callback(self, msg):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        position = np.array(msg.position, dtype=float)
        velocity = np.array(msg.velocity, dtype=float)
        q = normalize_quaternion(np.array(msg.q, dtype=float))
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)) or not np.all(np.isfinite(q)):
            return
        self.latest_state = np.concatenate([position, velocity, q])

    def current_reference(self):
        target = self.targets[self.target_index]
        if self.latest_state is not None:
            distance = np.linalg.norm(target - self.latest_state[0:3])
            if distance < self.target_acceptance_radius:
                self.target_index = (self.target_index + 1) % len(self.targets)
                target = self.targets[self.target_index]
                self.get_logger().info(f"Switching NMPC target to index {self.target_index}: {target.tolist()}")
        return np.concatenate([target, np.zeros(3), yaw_to_quaternion(self.target_yaw)])

    def control_loop(self):
        if self.solver is None or self.latest_state is None:
            return

        x0 = self.latest_state.copy()
        xref = self.current_reference()
        uref = np.array([0.0, 0.0, 0.0, self.hover_thrust])

        try:
            self.solver.set(0, "lbx", x0)
            self.solver.set(0, "ubx", x0)
            for stage in range(self.solver.N):
                self.solver.set(stage, "yref", np.concatenate([xref, uref]))
                self.solver.set(stage, "x", xref)
                self.solver.set(stage, "u", self.last_solution_u)
            self.solver.set(self.solver.N, "yref", xref)
            self.solver.set(self.solver.N, "x", xref)

            status = self.solver.solve()
            if status != 0:
                self.get_logger().warn(f"acados solve returned status {status}")
                return
            u0 = np.asarray(self.solver.get(0, "u"), dtype=float)
            if not np.all(np.isfinite(u0)):
                self.get_logger().warn("acados returned non-finite control")
                return
            self.last_solution_u = u0
            self.publish_output(u0)
        except Exception as exc:
            self.get_logger().warn(f"NMPC solve failed: {exc}")

    def publish_output(self, u):
        msg = VehicleRatesSetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.roll = float(np.clip(u[0], -3.0, 3.0))
        msg.pitch = float(np.clip(u[1], -3.0, 3.0))
        msg.yaw = float(np.clip(u[2], -1.5, 1.5))
        thrust = float(np.clip(u[3], 0.10, 0.90))
        msg.thrust_body = [0.0, 0.0, -thrust]
        msg.reset_integral = False
        self.output_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BodyRateNmpcController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
