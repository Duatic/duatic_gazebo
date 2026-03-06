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

import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node


class SimulationStarter(Node):
    def __init__(self):
        super().__init__("simulation_starter")

        # Declare ROS parameters with default values
        self.declare_parameter("world", "empty")
        self.declare_parameter("headless", False)

        # Get parameter values
        self.world = self.get_parameter("world").get_parameter_value().string_value
        self.headless = self.get_parameter("headless").get_parameter_value().bool_value

        # Convert boolean to string for docker compose
        self.headless_str = "true" if self.headless else "false"

        self.get_logger().info(f"Starting simulation container for world: {self.world}")
        self.get_logger().info(f"Headless mode: {self.headless_str}")

        # Compose the Docker start command
        self.start_cmd = (
            f"WORLD={self.world} HEADLESS={self.headless_str} docker compose "
            f"-p sim_{self.world} "
            "-f " + get_package_share_directory("duatic_gazebo") + "/docker/docker-compose.yml "
            "up simulation --detach --wait"
        )

        self.get_logger().info(f"Executing: {self.start_cmd}")
        os.system(self.start_cmd)

    def stop_simulation(self):
        """Stop the docker container for this simulation."""
        stop_cmd = (
            f"WORLD={self.world} docker compose "
            f"-p sim_{self.world} "
            "-f " + get_package_share_directory("duatic_gazebo") + "/docker/docker-compose.yml "
            "down --timeout 0"
        )
        self.get_logger().info(f"Stopping simulation container: {stop_cmd}")
        os.system(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationStarter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the simulation container on shutdown
        node.stop_simulation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
