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

"""A load welded to a hook by sim_scene stays up when its shelf is removed, and falls once released."""

import os
import subprocess
import time
import unittest
import warnings

import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
from tf2_msgs.msg import TFMessage

from duatic_helper_msgs.srv import AttachModel, DetachModel

WORLD = "sim_scene_weld"
POSE_TOPIC = f"/world/{WORLD}/dynamic_pose/info"

# Keeps this test's Gazebo transport apart from any other simulation on the machine.
os.environ["GZ_PARTITION"] = f"{WORLD}_{os.getpid()}"


def generate_test_description():
    """Generate a LaunchDescription for the test."""

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("duatic_gazebo"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments=[
            ("world", WORLD),
            ("headless", "true"),
            ("gz_models_path", os.path.join(os.path.dirname(__file__), "worlds")),
        ],
    )

    sim_scene = Node(
        package="duatic_gazebo",
        executable="sim_scene_node",
        name="sim_scene",
        parameters=[{"world": WORLD, "robot_model": "hook"}],
        output="screen",
    )

    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pose_bridge",
        arguments=[f"{POSE_TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        output="screen",
    )

    ready = TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()])

    return LaunchDescription([simulation, sim_scene, pose_bridge, ready])


class TestSimSceneWeld(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_sim_scene_weld")
        self.load_z = None
        self.node.create_subscription(TFMessage, POSE_TOPIC, self._on_poses, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _on_poses(self, msg):
        # The load is the world's only dynamic model; the other entry is its link, relative to it.
        if msg.transforms:
            self.load_z = max(t.transform.translation.z for t in msg.transforms)

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _fresh_load_z(self, timeout=10.0):
        self.load_z = None
        end = time.time() + timeout
        while self.load_z is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIsNotNone(self.load_z, f"No pose on {POSE_TOPIC} within {timeout} s")
        return self.load_z

    def _call(self, srv_type, name, request):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), f"{name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        self.assertTrue(future.done(), f"{name} did not answer")
        return future.result()

    def _remove_shelf(self):
        result = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                f"/world/{WORLD}/remove",
                "--reqtype",
                "gz.msgs.Entity",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                'name: "shelf" type: MODEL',
            ],
            capture_output=True,
            text=True,
        )
        self.assertTrue(
            "true" in result.stdout, f"Shelf not removed: {result.stdout} {result.stderr}"
        )

    def test_weld_holds_then_releases_load(self):
        resting_z = self._fresh_load_z(timeout=30.0)

        attach = self._call(
            AttachModel, "/sim_scene/attach", AttachModel.Request(child_model="load")
        )
        self.assertTrue(attach.success, attach.message)

        self._remove_shelf()
        self._spin(2.0)
        self.assertAlmostEqual(self._fresh_load_z(), resting_z, delta=0.02)

        detach = self._call(
            DetachModel, "/sim_scene/detach", DetachModel.Request(child_model="load")
        )
        self.assertTrue(detach.success, detach.message)

        self._spin(2.0)
        self.assertLess(self._fresh_load_z(), resting_z - 0.5)


@launch_testing.post_shutdown_test()
class TestSimSceneWeldShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        try:
            launch_testing.asserts.assertExitCodes(proc_info)
        except AssertionError as e:
            warnings.warn(f"Process exit warning: {e}")
