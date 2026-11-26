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

import getpass
import os
import shutil
import subprocess
import sys

from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node


class SimulationStarter(Node):
    def __init__(self):
        super().__init__("simulation_starter")

        # Declare ROS parameters with default values
        self.declare_parameter("world", "empty")
        self.declare_parameter("headless", False)
        self.declare_parameter("gpu", False)

        # Get parameter values
        self.world = self.get_parameter("world").get_parameter_value().string_value
        self.headless = self.get_parameter("headless").get_parameter_value().bool_value
        self.gpu = self.get_parameter("gpu").get_parameter_value().bool_value

        compose_dir = os.path.join(get_package_share_directory("duatic_gazebo"), "docker")
        self.compose_cmd = ["docker", "compose", "-p", f"sim_{self.world}"]
        self.compose_cmd += ["-f", os.path.join(compose_dir, "docker-compose.yml")]
        if self.gpu:
            self.compose_cmd += ["-f", os.path.join(compose_dir, "docker-compose.gpu.yml")]

        self.env = dict(
            os.environ,
            WORLD=self.world,
            HEADLESS="true" if self.headless else "false",
            HOST_UID=str(os.getuid()),
            HOST_GID=str(os.getgid()),
            HOST_USER=getpass.getuser(),
        )
        self.started = False

    def start_simulation(self) -> bool:
        """Start the docker container for this simulation. Returns False if it did not start."""
        if shutil.which("docker") is None:
            self.get_logger().error(
                "No docker CLI here. Run start_sim.py on a host with Docker, or launch "
                "duatic_gazebo gazebo.launch.py directly."
            )
            return False
        if not self.headless:
            if not os.environ.get("DISPLAY"):
                self.get_logger().error("DISPLAY is not set. Set it, or pass headless:=true.")
                return False
            # Admit the host user, whom the container runs as, to the X server.
            if shutil.which("xhost"):
                subprocess.run(
                    ["xhost", f"+SI:localuser:{getpass.getuser()}"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

        self.get_logger().info(
            f"Starting simulation container for world '{self.world}' "
            f"(headless: {self.headless}, gpu: {self.gpu})"
        )
        cmd = self.compose_cmd + ["up", "simulation", "--detach", "--wait"]
        self.started = True
        if subprocess.run(cmd, env=self.env).returncode != 0:
            self.get_logger().error(f"Simulation container did not start: {' '.join(cmd)}")
            return False
        return True

    def stop_simulation(self):
        """Stop the docker container for this simulation."""
        if not self.started:
            return
        cmd = self.compose_cmd + ["down", "--timeout", "0"]
        self.get_logger().info(f"Stopping simulation container: {' '.join(cmd)}")
        subprocess.run(cmd, env=self.env)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationStarter()
    exit_code = 0

    try:
        if node.start_simulation():
            rclpy.spin(node)
        else:
            exit_code = 1
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the simulation container on shutdown
        node.stop_simulation()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
