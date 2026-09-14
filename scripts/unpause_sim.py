#!/usr/bin/env python3
# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Unpause Gazebo once ros2_control holds the joints.
The world starts paused so gravity cannot pull the arms down before then.
"""

import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import ListHardwareComponents


class UnpauseSim(Node):
    def __init__(self):
        super().__init__("unpause_sim")
        self.declare_parameter("world", "empty")
        self.declare_parameter("wait_hardware", "")
        self.declare_parameter("controller_manager", "controller_manager")
        self.declare_parameter("timeout", 60.0)
        self.declare_parameter("poll_interval", 0.5)

    def run(self) -> int:
        world = str(self.get_parameter("world").value)
        raw = str(self.get_parameter("wait_hardware").value)
        wanted = {name.strip() for name in raw.split(",") if name.strip()}
        timeout = float(self.get_parameter("timeout").value)
        poll_interval = float(self.get_parameter("poll_interval").value)

        if wanted:
            self.get_logger().info(
                f"Waiting for hardware components before unpausing '{world}': {sorted(wanted)}"
            )
            if not self._wait_for_hardware(wanted, timeout, poll_interval):
                self.get_logger().warn(
                    f"Hardware components not all active after {timeout:.0f}s — unpausing anyway"
                )
        return self._unpause(world)

    def _wait_for_hardware(self, wanted: set, timeout: float, poll_interval: float) -> bool:
        cm = str(self.get_parameter("controller_manager").value).rstrip("/")
        client = self.create_client(ListHardwareComponents, f"{cm}/list_hardware_components")

        service = client.srv_name
        deadline = time.monotonic() + timeout
        next_notice = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if not client.service_is_ready():
                if time.monotonic() >= next_notice:
                    self.get_logger().warn(f"Still waiting for {service}")
                    next_notice = time.monotonic() + 5.0
                client.wait_for_service(timeout_sec=poll_interval)
                continue

            future = client.call_async(ListHardwareComponents.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=poll_interval * 4)
            response = future.result()
            if response is not None:
                active = {c.name for c in response.component if c.state.label == "active"}
                missing = wanted - active
                if not missing:
                    self.get_logger().info("All required hardware components active")
                    return True
                if time.monotonic() >= next_notice:
                    self.get_logger().warn(f"Not active yet: {sorted(missing)}")
                    next_notice = time.monotonic() + 5.0
            time.sleep(poll_interval)
        return False

    def _unpause(self, world: str) -> int:
        # gz_ros2_control offers no ROS-side pause control; the world control
        # service only exists on the Gazebo transport side, so shell out to
        # the gz CLI (same approach as start_sim.py's docker wrapping).
        cmd = [
            "gz",
            "service",
            "-s",
            f"/world/{world}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            "pause: false",
        ]
        for attempt in range(1, 4):
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0 and "true" in result.stdout.lower():
                self.get_logger().info(f"Simulation '{world}' unpaused")
                return 0
            self.get_logger().warn(
                f"Unpause attempt {attempt}/3 failed: {result.stdout.strip()} {result.stderr.strip()}"
            )
            time.sleep(1.0)
        self.get_logger().error(f"Could not unpause world '{world}'")
        return 1


def main():
    rclpy.init()
    node = UnpauseSim()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
