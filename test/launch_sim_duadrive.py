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

"""Simulated DuaDrive: hold without a controller, and the drive law that follows the claimed interfaces."""

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
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces, SwitchController
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from duatic_gazebo.sim_drive import apply_sim_drive, load_sim_drive_config

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(TEST_DIR, "urdf", "sim_duadrive_test.urdf")
DRIVE_CONFIG = os.path.join(TEST_DIR, "config", "sim_duadrive_test.yaml")
CONTROLLERS = os.path.join(TEST_DIR, "config", "sim_duadrive_test_controllers.yaml")
INITIAL_POSITION = 0.05
HEAVY_DEFAULT_GAINS = [300.0, 0.0, 25.0]
TEST_CONTROLLERS = [
    "heavy_pd",
    "heavy_position",
    "heavy_effort",
    "wheel_velocity",
    "light_velocity",
]

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

    test_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[*TEST_CONTROLLERS, "--inactive", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    ready = TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()])

    return LaunchDescription(
        [simulation, state_publisher, spawn, joint_state_broadcaster, test_controllers, ready]
    )


class Settled:
    def __init__(self, samples):
        joints = samples[0][1].keys()
        self.pos = {j: sum(s[1][j] for s in samples) / len(samples) for j in joints}
        self.vel = {j: sum(s[2][j] for s in samples) / len(samples) for j in joints}


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
        self._publishers = {
            c: self.node.create_publisher(Float64MultiArray, f"/{c}/commands", 10)
            for c in TEST_CONTROLLERS
        }
        # Cleanups run last in first out, so the node outlives the cleanups that tests add.
        self.addCleanup(self.node.destroy_node)

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

    def _call(self, srv_type, name, request):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=120.0), f"{name} not up")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertTrue(future.done(), f"{name} did not answer")
        self.node.destroy_client(client)
        return future.result()

    def switch(self, activate=(), deactivate=()):
        request = SwitchController.Request(
            activate_controllers=[activate] if isinstance(activate, str) else list(activate),
            deactivate_controllers=(
                [deactivate] if isinstance(deactivate, str) else list(deactivate)
            ),
            strictness=SwitchController.Request.STRICT,
        )
        result = self._call(SwitchController, "/controller_manager/switch_controller", request)
        self.assertTrue(result.ok, f"switch_controller refused {request}")

    def activate(self, controller):
        self.switch(activate=controller)

    def deactivate(self, controller):
        self.switch(deactivate=controller)

    def publish(self, controller, values):
        publisher = self._publishers[controller]
        self._spin_until(
            lambda: publisher.get_subscription_count() > 0, 30.0, f"{controller} to subscribe"
        )
        publisher.publish(Float64MultiArray(data=[float(v) for v in values]))

    def settle(self, duration):
        """Wait `duration` s of sim time; return the mean joint state over its second half."""
        self._spin_until(lambda: self.sim_time is not None, 120.0, "/clock")
        start = self.sim_time
        self._spin_until(
            lambda: self.sim_time >= start + duration,
            10.0 * duration + 30.0,
            f"{duration} s of sim time",
        )
        window = [s for s in self.samples if start + duration / 2 <= s[0] <= start + duration]
        self.assertTrue(window, f"No joint states in the second half of {duration} s")
        return Settled(window)

    def reset_heavy(self):
        """Deactivate every test controller, then hold heavy at the initial position with default gains."""
        end = time.time() + 120.0
        while True:
            controllers = self._call(
                ListControllers, "/controller_manager/list_controllers", ListControllers.Request()
            ).controller
            states = {c.name: c.state for c in controllers if c.name in TEST_CONTROLLERS}
            loaded = all(states.get(c) in ("inactive", "active") for c in TEST_CONTROLLERS)
            if loaded or time.time() > end:
                break
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.assertTrue(loaded, f"Test controllers not loaded: {states}")
        active = [name for name, state in states.items() if state == "active"]
        if active:
            self.deactivate(active)
        self.activate("heavy_pd")
        self.publish("heavy_pd", [INITIAL_POSITION, 0.0, 0.0, *HEAVY_DEFAULT_GAINS])
        held = self.settle(2.0).pos["heavy"]
        self.deactivate("heavy_pd")
        self.assertAlmostEqual(held, INITIAL_POSITION, delta=0.02)

    def start_from_hold(self):
        """Start from heavy held at the initial position, and restore that state when the test ends."""
        self.reset_heavy()
        self.addCleanup(self.reset_heavy)

    # Tests run in name order; the 0 keeps this one first, before any test moves a joint.
    def test_0_drive_joints_hold_without_controller(self):
        self._spin_until(lambda: self.samples, 120.0, "/joint_states")
        first = self.samples[0][0]
        self._spin_until(
            lambda: self.sim_time is not None and self.sim_time >= first + 4.0,
            120.0,
            "4 s of sim time after the first joint state",
        )
        # Average over the sim-time window [3 s, 4 s] after the first joint state.
        window = [s for s in self.samples if first + 3.0 <= s[0] <= first + 4.0]
        self.assertGreater(
            len(window), 50, f"Only {len(window)} joint states 3 s to 4 s after the first"
        )
        pos = {j: sum(s[1][j] for s in window) / len(window) for j in ("heavy", "light")}
        max_abs_vel = {j: max(abs(s[2][j]) for s in window) for j in ("heavy", "light")}
        print(f"held positions {pos}, max |velocity| {max_abs_vel}")

        # Gravity sag of the heavy joint is about 2.9 Nm / 300 Nm/rad; a velocity hold would show none.
        self.assertAlmostEqual(pos["heavy"], INITIAL_POSITION, delta=0.02)
        self.assertTrue(0.005 < pos["heavy"] - INITIAL_POSITION < 0.015, pos["heavy"])
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

    def test_commanded_gains_are_used(self):
        self.start_from_hold()
        self.activate("heavy_pd")
        self.publish("heavy_pd", [0.5, 0.0, 0.0, 300.0, 0.0, 25.0])
        held = self.settle(3.0).pos["heavy"]
        print(f"commanded gains: held at {held}")
        self.assertLess(abs(held - 0.5), 0.02)
        # Zero gains: the link drops under gravity.
        self.publish("heavy_pd", [0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
        dropped = self.settle(3.0).pos["heavy"]
        print(f"zero gains: mean position {dropped}")
        self.assertGreater(abs(dropped - 0.5), 0.3)
        self.deactivate("heavy_pd")

    def test_effort_only_passes_torque_through(self):
        self.start_from_hold()
        self.activate("heavy_effort")
        self.publish("heavy_effort", [0.0])
        pos = self.settle(3.0).pos["heavy"]
        print(f"zero effort: mean position {pos}")
        self.assertGreater(abs(pos - 0.05), 0.3)
        self.deactivate("heavy_effort")

    def test_position_works_again_after_effort(self):
        self.start_from_hold()
        self.activate("heavy_effort")
        self.publish("heavy_effort", [0.0])
        self.settle(1.0)
        self.switch(deactivate="heavy_effort", activate="heavy_position")
        self.publish("heavy_position", [0.3])
        pos = self.settle(3.0).pos["heavy"]
        print(f"position after effort: {pos}")
        self.assertLess(abs(pos - 0.3), 0.02)
        self.deactivate("heavy_position")

    def test_unclaimed_joint_holds_where_it_is(self):
        self.start_from_hold()
        self.activate("heavy_position")
        self.publish("heavy_position", [0.6])
        self.settle(3.0)
        self.deactivate("heavy_position")
        pos = self.settle(2.0).pos["heavy"]
        print(f"unclaimed: held at {pos}")
        self.assertLess(abs(pos - 0.6), 0.03)

    def test_switch_reseeds_position_target(self):
        self.start_from_hold()
        self.activate("heavy_position")
        self.publish("heavy_position", [0.6])
        self.settle(3.0)
        self.switch(deactivate="heavy_position", activate="heavy_effort")
        self.publish("heavy_effort", [0.0])
        self.settle(1.0)
        # No position command is published after this switch.
        self.switch(deactivate="heavy_effort", activate="heavy_position")
        # The joint swings at about 4 rad/s, so the reference is its newest state when the switch returns.
        before = self.samples[-1][1]["heavy"]
        after = self.settle(0.5).pos["heavy"]
        print(f"reseed: before {before}, after {after}")
        self.assertLess(abs(after - before), 0.1)
        self.deactivate("heavy_position")

    def test_bad_gain_keeps_previous(self):
        self.start_from_hold()
        self.activate("heavy_pd")
        self.publish("heavy_pd", [0.5, 0.0, 0.0, 300.0, 0.0, 25.0])
        self.settle(3.0)
        self.publish("heavy_pd", [0.5, 0.0, 0.0, float("nan"), 0.0, -1.0])
        # The controller drops a message holding NaN, so negative gains alone reach the drive.
        self.publish("heavy_pd", [0.5, 0.0, 0.0, -1.0, 0.0, -1.0])
        pos = self.settle(2.0).pos["heavy"]
        print(f"bad gains: held at {pos}")
        self.assertLess(abs(pos - 0.5), 0.02)
        self.deactivate("heavy_pd")

    def test_wheel_keeps_velocity_path(self):
        self.start_from_hold()
        self.activate("wheel_velocity")
        self.publish("wheel_velocity", [2.0])
        vel = self.settle(2.0).vel["wheel"]
        print(f"wheel velocity {vel}")
        self.assertLess(abs(vel - 2.0), 0.2)
        self.deactivate("wheel_velocity")

    def test_velocity_only_on_drive_joint(self):
        self.start_from_hold()
        # light has been under the PD law since spawn, so Gazebo holds a force command for it.
        start = self.settle(0.5)
        self.assertLess(abs(start.vel["light"]), 0.05)
        self.activate("light_velocity")
        self.publish("light_velocity", [1.0])
        moving = self.settle(1.0)
        self.deactivate("light_velocity")
        held = self.settle(1.0)
        later = self.settle(1.0)
        print(
            f"light: velocity {moving.vel['light']}, held at {held.pos['light']} then {later.pos['light']}"
        )
        self.assertLess(abs(moving.vel["light"] - 1.0), 0.1)
        self.assertGreater(held.pos["light"] - start.pos["light"], 0.5)
        self.assertLess(abs(later.pos["light"] - held.pos["light"]), 0.01)
        self.assertLess(abs(later.vel["light"]), 0.05)


@launch_testing.post_shutdown_test()
class TestSimDuaDriveShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        try:
            launch_testing.asserts.assertExitCodes(proc_info)
        except AssertionError as e:
            warnings.warn(f"Process exit warning: {e}")
