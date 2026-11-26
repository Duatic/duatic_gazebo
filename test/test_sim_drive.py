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

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from duatic_gazebo.sim_drive import apply_sim_drive, load_sim_drive_config

URDF = """<?xml version="1.0"?>
<robot name="r">
  <ros2_control name="TestSystem" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
    <joint name="a"><command_interface name="position"/></joint>
    <joint name="b"><command_interface name="position"/></joint>
    <joint name="wheel"><command_interface name="velocity"/></joint>
  </ros2_control>
  <ros2_control name="Head" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
    <joint name="neck"><command_interface name="position"/></joint>
  </ros2_control>
  <link name="base"/>
  <link name="link_a">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <link name="link_b">
    <inertial>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.003"/>
    </inertial>
  </link>
  <link name="link_wheel">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="a" type="revolute">
    <parent link="base"/><child link="link_a"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="b" type="revolute">
    <parent link="link_a"/><child link="link_b"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="wheel" type="continuous">
    <parent link="link_b"/><child link="link_wheel"/><axis xyz="0 1 0"/>
  </joint>
</robot>
"""

CFG = {
    "hardware": "TestSystem",
    "joints": {
        "a": {"p_gain": 60.0, "i_gain": 0.0, "d_gain": 3.0, "armature": 0.02},
        "b": {"p_gain": 300.0, "i_gain": 0.0, "d_gain": 25.0, "armature": 0.06},
    },
}


def plugin_of(urdf, hardware):
    for ros2_control in ET.fromstring(urdf).iter("ros2_control"):
        if ros2_control.get("name") == hardware:
            return ros2_control.find("hardware/plugin").text
    raise AssertionError(hardware)


def joint_params(urdf, joint):
    for ros2_control in ET.fromstring(urdf).iter("ros2_control"):
        for element in ros2_control.findall("joint"):
            if element.get("name") == joint:
                return {p.get("name"): p.text for p in element.findall("param")}
    raise AssertionError(joint)


def inertia(urdf, link):
    for element in ET.fromstring(urdf).iter("link"):
        if element.get("name") == link:
            return {k: float(v) for k, v in element.find("inertial/inertia").attrib.items()}
    raise AssertionError(link)


def test_plugin_is_switched():
    assert plugin_of(apply_sim_drive(URDF, CFG), "TestSystem") == "duatic_gazebo/SimDuaDriveSystem"


def test_gain_params_added_only_to_listed_joints():
    out = apply_sim_drive(URDF, CFG)
    assert joint_params(out, "a") == {
        "sim_p_gain": "60.0",
        "sim_i_gain": "0.0",
        "sim_d_gain": "3.0",
    }
    assert joint_params(out, "b") == {
        "sim_p_gain": "300.0",
        "sim_i_gain": "0.0",
        "sim_d_gain": "25.0",
    }
    assert "sim_p_gain" not in joint_params(out, "wheel")


def test_armature_added_about_joint_axis():
    out = inertia(apply_sim_drive(URDF, CFG), "link_a")
    assert out["izz"] == pytest.approx(0.021)
    assert out["ixx"] == pytest.approx(0.011)
    assert out["iyy"] == pytest.approx(0.011)


def test_armature_respects_inertial_rotation():
    # The joint axis z in the link frame is the inertial frame's y axis after rpy (pi/2, 0, 0).
    out = inertia(apply_sim_drive(URDF, CFG), "link_b")
    assert out["iyy"] == pytest.approx(0.062)
    assert out["ixx"] == pytest.approx(0.001 + 0.03)
    assert out["izz"] == pytest.approx(0.003 + 0.03)


def with_link_a_inertia(values, axis="0 0 1"):
    urdf = URDF.replace(
        '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>',
        "<inertia " + " ".join(f'{k}="{v}"' for k, v in values.items()) + "/>",
        1,
    )
    return urdf.replace(
        '<child link="link_a"/><axis xyz="0 0 1"/>', f'<child link="link_a"/><axis xyz="{axis}"/>'
    )


def test_armature_keeps_inertia_valid():
    wrist = {"ixx": 1e-5, "ixy": 0, "ixz": 0, "iyy": 1e-5, "iyz": 0, "izz": 1.5e-5}
    out = inertia(apply_sim_drive(with_link_a_inertia(wrist), CFG), "link_a")
    tensor = np.array(
        [
            [out["ixx"], out["ixy"], out["ixz"]],
            [out["ixy"], out["iyy"], out["iyz"]],
            [out["ixz"], out["iyz"], out["izz"]],
        ]
    )
    small, middle, large = np.linalg.eigvalsh(tensor)
    assert small > 0.0
    assert small + middle >= large


def test_invalid_result_names_the_link():
    bad = {"ixx": 1e-3, "ixy": 0, "ixz": 0, "iyy": 1e-3, "iyz": 0, "izz": 1.0}
    cfg = {"hardware": "TestSystem", "joints": {"a": {**CFG["joints"]["a"], "armature": 0.01}}}
    with pytest.raises(ValueError, match="link_a"):
        apply_sim_drive(with_link_a_inertia(bad, axis="1 0 0"), cfg)


def test_unlisted_link_inertia_is_unchanged():
    assert inertia(apply_sim_drive(URDF, CFG), "link_wheel") == inertia(URDF, "link_wheel")


def test_unknown_joint_is_named():
    bad = {"hardware": "TestSystem", "joints": {"nope": CFG["joints"]["a"]}}
    with pytest.raises(ValueError, match="nope"):
        apply_sim_drive(URDF, bad)


def test_joint_missing_from_urdf_joints_is_named():
    urdf = URDF.replace('<joint name="a" type="revolute">', '<joint name="a_gone" type="revolute">')
    with pytest.raises(ValueError, match="'a'"):
        apply_sim_drive(urdf, CFG)


def test_joint_in_other_ros2_control_is_named():
    bad = {"hardware": "TestSystem", "joints": {"neck": CFG["joints"]["a"]}}
    with pytest.raises(ValueError, match="neck"):
        apply_sim_drive(URDF, bad)


def test_link_without_inertial_is_named():
    root = ET.fromstring(URDF)
    link = next(link for link in root.iter("link") if link.get("name") == "link_a")
    link.remove(link.find("inertial"))
    with pytest.raises(ValueError, match="link_a"):
        apply_sim_drive(ET.tostring(root, encoding="unicode"), CFG)


def test_unknown_hardware_is_named():
    with pytest.raises(ValueError, match="Missing"):
        apply_sim_drive(URDF, {"hardware": "Missing", "joints": {}})


def write_config(tmp_path, joints):
    path = tmp_path / "drive.yaml"
    lines = ["hardware: TestSystem", "joints:"]
    for name, values in joints.items():
        body = ", ".join(f"{k}: {v}" for k, v in values.items())
        lines.append(f"  {name}: {{{body}}}")
    path.write_text("\n".join(lines) + "\n")
    return str(path)


def test_config_loads(tmp_path):
    path = write_config(tmp_path, {"a": CFG["joints"]["a"]})
    assert load_sim_drive_config(path) == {
        "hardware": "TestSystem",
        "joints": {"a": CFG["joints"]["a"]},
    }


def test_missing_or_negative_value_is_rejected(tmp_path):
    good = CFG["joints"]["a"]
    no_d = {k: v for k, v in good.items() if k != "d_gain"}
    with pytest.raises(ValueError, match="no_d.*d_gain"):
        load_sim_drive_config(write_config(tmp_path, {"no_d": no_d}))
    with pytest.raises(ValueError, match="neg_p"):
        load_sim_drive_config(write_config(tmp_path, {"neg_p": {**good, "p_gain": -1.0}}))
    with pytest.raises(ValueError, match="nan_d"):
        load_sim_drive_config(write_config(tmp_path, {"nan_d": {**good, "d_gain": ".nan"}}))


def test_infinite_value_is_rejected(tmp_path):
    good = CFG["joints"]["a"]
    with pytest.raises(ValueError, match="inf_p.*p_gain"):
        load_sim_drive_config(write_config(tmp_path, {"inf_p": {**good, "p_gain": ".inf"}}))
    with pytest.raises(ValueError, match="inf_arm.*armature"):
        load_sim_drive_config(write_config(tmp_path, {"inf_arm": {**good, "armature": ".inf"}}))


def test_zero_length_axis_is_named():
    with pytest.raises(ValueError, match="'a'"):
        apply_sim_drive(
            URDF.replace(
                '<child link="link_a"/><axis xyz="0 0 1"/>',
                '<child link="link_a"/><axis xyz="0 0 0"/>',
            ),
            CFG,
        )


def test_empty_joints_switches_only_named_plugin():
    out = apply_sim_drive(URDF, {"hardware": "TestSystem", "joints": {}})
    assert plugin_of(out, "TestSystem") == "duatic_gazebo/SimDuaDriveSystem"
    assert plugin_of(out, "Head") == "gz_ros2_control/GazeboSimSystem"
    for joint in ("a", "b", "wheel", "neck"):
        assert joint_params(out, joint) == {}
    for link in ("link_a", "link_b", "link_wheel"):
        assert inertia(out, link) == inertia(URDF, link)


def test_other_ros2_control_untouched_with_joints():
    assert plugin_of(apply_sim_drive(URDF, CFG), "Head") == "gz_ros2_control/GazeboSimSystem"
