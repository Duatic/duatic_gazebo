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

import numpy as np
import pytest
from sdf_to_map import FREE, OCCUPIED, rasterize, slice_world
import trimesh

WORLD = """<?xml version="1.0"?>
<sdf version="1.11">
  <world name="test">
    <model name="ground">
      <link name="link">
        <visual name="visual"><geometry><plane><normal>0 0 1</normal></plane></geometry></visual>
      </link>
    </model>
    <model name="box">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <pose>0 1 0 0 0 0</pose>
        <visual name="visual">
          <pose>0 0 0.5 0 0 0</pose>
          <geometry><box><size>1 2 1</size></box></geometry>
        </visual>
        <collision name="collision">
          <geometry><box><size>3 6 3</size></box></geometry>
        </collision>
      </link>
    </model>
    <include>
      <uri>model://create</uri>
      <pose>-3 0 0 0 0 1.5707963</pose>
    </include>
  </world>
</sdf>
"""

CREATE = """<?xml version="1.0"?>
<sdf version="1.11">
  <model name="create">
    <pose>100 100 0 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><mesh><uri>meshes/create.stl</uri><scale>0.001 0.002 0.001</scale></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def make_world(tmp_path):
    """Write a world with a posed box and an included create model whose STL is in millimetres.

    :param tmp_path: Directory to write the world and its ``models/`` directory into.
    :returns: Path of the world file and of the models directory.
    """
    crate_dir = tmp_path / "models" / "create"
    (crate_dir / "meshes").mkdir(parents=True)
    (crate_dir / "model.sdf").write_text(CREATE)
    mesh = trimesh.creation.box([400, 200, 1000])
    mesh.apply_translation([0, 0, 500])
    mesh.export(crate_dir / "meshes" / "create.stl")
    world = tmp_path / "test.sdf"
    world.write_text(WORLD)
    return world, tmp_path / "models"


def test_slice_world_uses_visuals_with_poses_scale_and_include_override(tmp_path):
    world, models_dir = make_world(tmp_path)
    points = slice_world(world, models_dir, 0.25).reshape(-1, 2)
    box = points[points[:, 0] > 0]
    create = points[points[:, 0] < 0]
    np.testing.assert_allclose(box.min(axis=0), [1.5, 0.0], atol=1e-6)
    np.testing.assert_allclose(box.max(axis=0), [2.5, 2.0], atol=1e-6)
    # 0.4 x 0.4 m create turned 90 degrees about z at (-3, 0), ignoring the model's own pose.
    np.testing.assert_allclose(create.min(axis=0), [-3.2, -0.2], atol=1e-6)
    np.testing.assert_allclose(create.max(axis=0), [-2.8, 0.2], atol=1e-6)


def test_slice_world_misses_geometry_above_the_plane(tmp_path):
    world, models_dir = make_world(tmp_path)
    assert len(slice_world(world, models_dir, 1.5)) == 0


def test_rasterize_frames_segments_with_margin():
    segments = np.array([[[0.02, 0.02], [0.98, 0.02]], [[0.98, 0.02], [0.98, 0.98]]])
    image, origin = rasterize(segments, 0.1)
    assert origin == (-0.5, -0.5)
    assert image.size == (20, 20)
    pixels = np.array(image)
    assert pixels[14, 5] == OCCUPIED  # (0.02, 0.02) is column 5, row 20 - 5 - 1 from the top
    assert pixels[5, 14] == OCCUPIED  # (0.98, 0.98)
    assert pixels[10, 10] == FREE


def test_rasterize_rejects_empty_slice():
    with pytest.raises(ValueError):
        rasterize(np.empty((0, 2, 2)), 0.05)
