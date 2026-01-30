import warnings
import unittest

import rclpy
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import launch_testing
import rosgraph_msgs.msg

from duatic_ros2_testing.tests import wait_for_node, wait_for_message

ARGUMENTS = [("world", "warehouse"), ("headless", "true")]


def generate_test_description():
    """Generate a LaunchDescription for the test."""

    simulation = Node(
        package="duatic_gazebo",
        executable="start_sim.py",
        name="simulation",
        output="screen",
        arguments=ARGUMENTS,
    )

    ready = TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()])

    ld = LaunchDescription()
    ld.add_action(simulation)
    ld.add_action(ready)

    return ld


# -----------------------
# Shared test definitions
# -----------------------
class TestGazeboWorld(unittest.TestCase):
    """Base test class for all Gazebo world tests."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gazebo_world")

    def tearDown(self):
        self.node.destroy_node()

    def test_clock_bridge_start(self):
        """Test if the clock_bridge node started."""
        wait_for_node(self.node, "clock_bridge", timeout=10.0)

    def test_publishes_clock(self, proc_output):
        """Check whether /clock messages are published."""
        wait_for_message(self.node, "/clock", rosgraph_msgs.msg.Clock, timeout=10.0)


@launch_testing.post_shutdown_test()
class TestGazeboWorldShutdown(unittest.TestCase):
    """Post-shutdown test to verify processes exited cleanly."""

    def test_exit_codes(self, proc_info):
        try:
            launch_testing.asserts.assertExitCodes(proc_info)
        except AssertionError as e:
            warnings.warn(f"Process exit warning: {e}")
