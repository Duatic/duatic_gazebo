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

import math
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PLUGIN = "duatic_gazebo/SimDuaDriveSystem"
GAINS = ("p_gain", "i_gain", "d_gain")
FIELDS = GAINS + ("armature",)


def _number(joint, key, values):
    value = values.get(key) if isinstance(values, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"joint '{joint}': '{key}' is missing or not a number")
    if math.isnan(value) or value < 0.0:
        raise ValueError(f"joint '{joint}': '{key}' must be a non-negative number, got {value}")
    return float(value)


def load_sim_drive_config(path: str) -> dict:
    """Read and validate a sim drive config: the hardware name and per-joint gains and armature."""
    with open(path) as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict) or not isinstance(raw.get("hardware"), str):
        raise ValueError(f"{path}: 'hardware' must be the name of a ros2_control element")
    joints = raw.get("joints")
    if not isinstance(joints, dict):
        raise ValueError(f"{path}: 'joints' must map joint names to values")
    return {
        "hardware": raw["hardware"],
        "joints": {
            name: {k: _number(name, k, values) for k in FIELDS} for name, values in joints.items()
        },
    }


def _rotation_from_rpy(rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _vector(text, default):
    return np.array([float(v) for v in text.split()]) if text else np.array(default, dtype=float)


def _add_armature(link, axis_in_link, armature):
    inertial = link.find("inertial")
    if inertial is None or inertial.find("inertia") is None:
        raise ValueError(f"link '{link.get('name')}' has no <inertial> to add armature to")
    origin = inertial.find("origin")
    rpy = _vector(origin.get("rpy") if origin is not None else None, (0, 0, 0))
    # Joint axis is in the link frame, the inertia tensor in the inertial frame.
    n = _rotation_from_rpy(rpy).T @ axis_in_link
    inertia = inertial.find("inertia")
    names = {
        "ixx": (0, 0),
        "ixy": (0, 1),
        "ixz": (0, 2),
        "iyy": (1, 1),
        "iyz": (1, 2),
        "izz": (2, 2),
    }
    # Thin disc: full armature about the axis, half about each perpendicular axis, valid for any link.
    delta = armature * np.outer(n, n) + 0.5 * armature * (np.eye(3) - np.outer(n, n))
    tensor = np.zeros((3, 3))
    for key, (i, j) in names.items():
        value = float(inertia.get(key, "0")) + float(delta[i, j])
        inertia.set(key, repr(value))
        tensor[i, j] = tensor[j, i] = value
    small, middle, large = np.linalg.eigvalsh(tensor)
    if small <= 0.0 or small + middle < large * (1.0 - 1e-9):
        raise ValueError(f"link '{link.get('name')}' has an invalid inertia after adding armature")


def apply_sim_drive(urdf_xml: str, config: dict) -> str:
    """Switch the named ros2_control hardware to the sim drive plugin and add per-joint gains and armature."""
    root = ET.fromstring(urdf_xml)
    hardware = config["hardware"]
    system = next((e for e in root.iter("ros2_control") if e.get("name") == hardware), None)
    plugin = system.find("hardware/plugin") if system is not None else None
    if plugin is None:
        raise ValueError(f"no <ros2_control name='{hardware}'> with a hardware plugin in the URDF")

    control_joints = {e.get("name"): e for e in system.findall("joint")}
    urdf_joints = {e.get("name"): e for e in root.findall("joint")}
    links = {e.get("name"): e for e in root.findall("link")}
    for name in config["joints"]:
        if name not in control_joints:
            raise ValueError(f"joint '{name}' is not a joint of ros2_control '{hardware}'")
        if name not in urdf_joints:
            raise ValueError(f"joint '{name}' is not a <joint> of the URDF")

    plugin.text = PLUGIN
    for name, values in config["joints"].items():
        for key in GAINS:
            param = ET.SubElement(control_joints[name], "param", name=f"sim_{key}")
            param.text = repr(float(values[key]))
        urdf_joint = urdf_joints[name]
        child = urdf_joint.find("child").get("link")
        if child not in links:
            raise ValueError(f"joint '{name}': child link '{child}' is not in the URDF")
        axis = urdf_joint.find("axis")
        axis_vec = _vector(axis.get("xyz") if axis is not None else None, (1, 0, 0))
        _add_armature(links[child], axis_vec / np.linalg.norm(axis_vec), values["armature"])
    return ET.tostring(root, encoding="unicode")
