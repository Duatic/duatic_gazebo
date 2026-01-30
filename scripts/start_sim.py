#!/usr/bin/env python3
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
