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

"""Write an occupancy map for a Gazebo world by slicing the visual geometry its LIDAR renders."""

import argparse
import functools
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import trimesh

# Scan plane of the DuaRover's lidar_center frame above the ground.
DEFAULT_HEIGHT = 0.25
DEFAULT_RESOLUTION = 0.05
MARGIN = 0.5
OCCUPIED = 0
FREE = 254


def parse_pose(text: str | None) -> np.ndarray:
    """Convert an SDF ``<pose>`` string into a homogeneous transform.

    :param text: Six numbers ``x y z roll pitch yaw`` in metres and radians, or ``None`` for identity.
    :returns: 4x4 transform from the pose's frame to its parent frame.
    """
    if not text:
        return np.eye(4)
    x, y, z, roll, pitch, yaw = (float(v) for v in text.split())
    transform = trimesh.transformations.euler_matrix(roll, pitch, yaw, "sxyz")
    transform[:3, 3] = (x, y, z)
    return transform


def resolve_uri(uri: str, base_dir: Path, models_dir: Path) -> Path:
    """Map an SDF URI to a file or directory on disk.

    :param uri: ``model://name/...``, ``file://...``, an absolute path, or a path relative to ``base_dir``.
    :param base_dir: Directory of the model that contains the URI.
    :param models_dir: Directory holding one subdirectory per model, used for ``model://`` and bare
        model names.
    :returns: Path the URI refers to. Existence is not checked.
    """
    if uri.startswith("model://"):
        return models_dir / uri.removeprefix("model://")
    if uri.startswith("file://"):
        return Path(uri.removeprefix("file://"))
    if (models_dir / uri).is_dir():
        return models_dir / uri
    return base_dir / uri


@functools.cache
def load_mesh(path: Path) -> trimesh.Trimesh:
    """Load a mesh file in metres, flattening any scene graph it contains.

    :param path: STL, OBJ, DAE or GLB file.
    :returns: Single mesh in the file's own frame.
    :raises ValueError: If trimesh cannot read the file.
    """
    mesh = trimesh.load(path, force="mesh")
    if path.suffix.lower() == ".dae":
        unit = re.search(r'<unit[^>]*meter="([0-9.eE+-]+)"', path.read_text(errors="ignore"))
        if unit:
            mesh.apply_scale(float(unit.group(1)))
    return mesh


def geometry_to_mesh(
    geometry: ET.Element, model_dir: Path, models_dir: Path
) -> trimesh.Trimesh | None:
    """Build a mesh for one SDF ``<geometry>`` element in its visual frame.

    :param geometry: The ``<geometry>`` element.
    :param model_dir: Directory of the model, for relative mesh URIs.
    :param models_dir: Directory holding one subdirectory per model.
    :returns: New mesh the caller may modify, or ``None`` for planes and unsupported shapes, which
        cannot show up in a horizontal slice.
    """
    shape = geometry[0]
    if shape.tag == "box":
        return trimesh.creation.box([float(v) for v in shape.findtext("size").split()])
    if shape.tag == "cylinder":
        radius, length = float(shape.findtext("radius")), float(shape.findtext("length"))
        return trimesh.creation.cylinder(radius=radius, height=length)
    if shape.tag == "sphere":
        return trimesh.creation.icosphere(radius=float(shape.findtext("radius")))
    if shape.tag == "mesh":
        mesh = load_mesh(resolve_uri(shape.findtext("uri").strip(), model_dir, models_dir)).copy()
        scale = shape.findtext("scale")
        if scale:
            mesh.apply_transform(np.diag([*(float(v) for v in scale.split()), 1.0]))
        return mesh
    if shape.tag != "plane":
        print(f"Skipping unsupported geometry <{shape.tag}>", file=sys.stderr)
    return None


def collect_meshes(
    model: ET.Element,
    parent: np.ndarray,
    model_dir: Path,
    models_dir: Path,
    pose: str | None = None,
):
    """Yield every visual mesh of a model and its nested and included models, in world frame.

    :param model: The ``<model>`` element.
    :param parent: 4x4 transform from the frame the model's pose is expressed in to the world.
    :param model_dir: Directory of the model, for relative mesh URIs.
    :param models_dir: Directory holding one subdirectory per model.
    :param pose: Pose overriding the model's own ``<pose>``, as an ``<include>`` does.
    :returns: Generator of meshes transformed into the world frame.
    """
    frame = parent @ parse_pose(pose if pose is not None else model.findtext("pose"))
    for link in model.findall("link"):
        link_frame = frame @ parse_pose(link.findtext("pose"))
        for visual in link.findall("visual"):
            mesh = geometry_to_mesh(visual.find("geometry"), model_dir, models_dir)
            if mesh is not None:
                yield mesh.apply_transform(link_frame @ parse_pose(visual.findtext("pose")))
    for nested in model.findall("model"):
        yield from collect_meshes(nested, frame, model_dir, models_dir)
    yield from collect_includes(model, frame, model_dir, models_dir)


def collect_includes(parent_elem: ET.Element, parent: np.ndarray, base_dir: Path, models_dir: Path):
    """Yield the visual meshes of every ``<include>`` directly under an element, in world frame.

    :param parent_elem: ``<world>`` or ``<model>`` element containing the includes.
    :param parent: 4x4 transform from that element's frame to the world.
    :param base_dir: Directory of the file containing the element, for relative include URIs.
    :param models_dir: Directory holding one subdirectory per model.
    :returns: Generator of meshes transformed into the world frame.
    """
    for include in parent_elem.findall("include"):
        included_dir = resolve_uri(include.findtext("uri").strip(), base_dir, models_dir)
        included = ET.parse(included_dir / "model.sdf").getroot().find("model")
        yield from collect_meshes(
            included, parent, included_dir, models_dir, include.findtext("pose")
        )


def slice_world(world_path: Path, models_dir: Path, height: float) -> np.ndarray:
    """Intersect all visual geometry of a world with a horizontal plane.

    :param world_path: World ``.sdf`` file.
    :param models_dir: Directory holding one subdirectory per model.
    :param height: Plane height above the world origin in metres.
    :returns: Array of shape (n, 2, 2) holding the xy endpoints of each intersection segment, in metres.
    """
    world = ET.parse(world_path).getroot().find("world")
    meshes = [
        *collect_includes(world, np.eye(4), world_path.parent, models_dir),
        *(
            mesh
            for model in world.findall("model")
            for mesh in collect_meshes(model, np.eye(4), world_path.parent, models_dir)
        ),
    ]
    segments = [
        trimesh.intersections.mesh_plane(mesh, plane_normal=[0, 0, 1], plane_origin=[0, 0, height])
        for mesh in meshes
    ]
    return np.concatenate([s[:, :, :2] for s in segments if len(s)] or [np.empty((0, 2, 2))])


def rasterize(segments: np.ndarray, resolution: float) -> tuple[Image.Image, tuple[float, float]]:
    """Draw slice segments as occupied cells on a free map framed around them.

    :param segments: Array of shape (n, 2, 2) of xy endpoints in metres, as from :func:`slice_world`.
    :param resolution: Cell size in metres.
    :returns: Greyscale image with row 0 at the largest y, and the xy of its lower-left corner in metres.
    :raises ValueError: If there are no segments to frame the map around.
    """
    if not len(segments):
        raise ValueError("no geometry crosses the slice height")
    points = segments.reshape(-1, 2)
    origin = np.floor((points.min(axis=0) - MARGIN) / resolution) * resolution
    width, height = np.ceil((points.max(axis=0) + MARGIN - origin) / resolution).astype(int)
    image = Image.new("L", (int(width), int(height)), FREE)
    draw = ImageDraw.Draw(image)
    for start, end in (segments - origin) / resolution:
        draw.line([(start[0], height - start[1]), (end[0], height - end[1])], fill=OCCUPIED)
    return image, (round(float(origin[0]), 3), round(float(origin[1]), 3))


def write_map(
    image: Image.Image, origin: tuple[float, float], resolution: float, output_dir: Path, name: str
) -> None:
    """Write ``<name>.png`` and the matching map_server ``<name>.yaml``, overwriting existing files.

    :param image: Map image as from :func:`rasterize`.
    :param origin: xy of the image's lower-left corner in metres.
    :param resolution: Cell size in metres.
    :param output_dir: Existing directory to write into.
    :param name: Basename shared by both files.
    """
    image.save(output_dir / f"{name}.png")
    (output_dir / f"{name}.yaml").write_text(
        f"image: {name}.png\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, 0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )


def main() -> None:
    """Write a map into the package's ``maps/`` for each world given on the command line."""
    package_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("worlds", nargs="+", type=Path, help="world .sdf files")
    parser.add_argument("--height", type=float, default=DEFAULT_HEIGHT, help="slice height in m")
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION, help="m per cell")
    args = parser.parse_args()
    for world in args.worlds:
        segments = slice_world(world, package_dir / "models", args.height)
        image, origin = rasterize(segments, args.resolution)
        write_map(image, origin, args.resolution, package_dir / "maps", world.stem)
        print(f"{world.stem}: {image.width}x{image.height} cells, origin {origin}")


if __name__ == "__main__":
    main()
