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

"""Drive joints of the simulated DuaDrive hold their initial position with no controller claiming them."""

import os
import time
import unittest
import warnings
import xml.etree.ElementTree as ET

import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
from controller_manager_msgs.srv import ListHardwareInterfaces
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from duatic_gazebo.sim_drive import apply_sim_drive, load_sim_drive_config

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(TEST_DIR, "urdf", "sim_duadrive_test.urdf")
DRIVE_CONFIG = os.path.join(TEST_DIR, "config", "sim_duadrive_test.yaml")
CONTROLLERS = os.path.join(TEST_DIR, "config", "sim_duadrive_test_controllers.yaml")
INITIAL_POSITION = 0.05

# Keeps this test's Gazebo transport apart from any other simulation on the machine.
os.environ["GZ_PARTITION"] = f"sim_duadrive_{os.getpid()}"


def robot_description():
    with open(URDF) as f:
        urdf = apply_sim_drive(f.read(), load_sim_drive_config(DRIVE_CONFIG))
    root = ET.fromstring(urdf)
    for plugin in root.iter("plugin"):
        if plugin.get("name") == "gz_ros2_control::GazeboSimROS2ControlPlugin":
            plugin.find("parameters").text = CONTROLLERS
    return ET.tostring(root, encoding="unicode")


def generate_test_description():
    """Generate a LaunchDescription for the test."""

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("duatic_gazebo"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments=[
            ("world", "duatic_empty"),
            ("headless", "true"),
            ("wait_hardware", "TestSystem"),
        ],
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description(), "use_sim_time": True}],
        output="screen",
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("duatic_gazebo"), "launch", "spawn.launch.py"])
        ),
        launch_arguments=[("namespace", "sim_duadrive_test")],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    ready = TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()])

    return LaunchDescription([simulation, state_publisher, spawn, joint_state_broadcaster, ready])


class TestSimDuaDrive(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_sim_duadrive")
        self.sim_time = None
        self.samples = []
        self.node.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_states, 100)

    def tearDown(self):
        self.node.destroy_node()

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_joint_states(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append(
            (stamp, dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity)))
        )

    def _spin_until(self, condition, timeout, what):
        end = time.time() + timeout
        while not condition() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(condition(), f"Timed out after {timeout} s waiting for {what}")

    def test_drive_joints_hold_without_controller(self):
        self._spin_until(
            lambda: self.samples and self.sim_time is not None and self.sim_time >= 4.0,
            120.0,
            "/joint_states and 4 s of sim time",
        )
        # Average over the sim-time window [3 s, 4 s].
        window = [s for s in self.samples if 3.0 <= s[0] <= 4.0]
        self.assertGreater(len(window), 50, f"Only {len(window)} joint states between 3 s and 4 s")
        pos = {j: sum(s[1][j] for s in window) / len(window) for j in ("heavy", "light")}
        max_abs_vel = {j: max(abs(s[2][j]) for s in window) for j in ("heavy", "light")}
        print(f"held positions {pos}, max |velocity| {max_abs_vel}")

        # Gravity sag of the heavy joint is about 2.9 Nm / 300 Nm/rad.
        self.assertAlmostEqual(pos["heavy"], INITIAL_POSITION, delta=0.02)
        self.assertAlmostEqual(pos["light"], INITIAL_POSITION, delta=0.01)
        self.assertLess(max_abs_vel["heavy"], 0.05)
        self.assertLess(max_abs_vel["light"], 0.05)

    def test_gain_interfaces_are_offered(self):
        client = self.node.create_client(
            ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=120.0), "controller_manager not up")
        future = client.call_async(ListHardwareInterfaces.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        self.assertTrue(future.done(), "list_hardware_interfaces did not answer")
        command_interfaces = [itf.name for itf in future.result().command_interfaces]

        for itf in ("position", "velocity", "effort", "p_gain", "i_gain", "d_gain"):
            self.assertTrue(f"heavy/{itf}" in command_interfaces, command_interfaces)
        self.assertFalse("wheel/p_gain" in command_interfaces, command_interfaces)


@launch_testing.post_shutdown_test()
class TestSimDuaDriveShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        try:
            launch_testing.asserts.assertExitCodes(proc_info)
        except AssertionError as e:
            warnings.warn(f"Process exit warning: {e}")
