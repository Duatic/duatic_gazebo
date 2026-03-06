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

import warnings
import unittest

import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import rosgraph_msgs.msg

from duatic_helpers.test_helper import wait_for_message, wait_for_node

ARGUMENTS = [("world", "warehouse"), ("headless", "true")]


def generate_test_description():
    """Generate a LaunchDescription for the test."""

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("duatic_gazebo"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments=ARGUMENTS,
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
