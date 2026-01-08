#!/usr/bin/env python3
"""
Convert a ROS 2 xacro file to a URDF with mesh paths that are usable in Isaac Sim.

Features:
- Runs xacro to expand the .xacro into plain URDF
- Rewrites all mesh filenames from:
    file:///absolute/path/.../share/<pkg>/...  -> file://../../../<pkg>/...
- (Optional) Rewrites package:// URIs via ament_index_python if present.

Intended usage:
- Input xacro has things like: file://$(find alpha_description)/meshes/${hip_roll_mesh}
- After xacro + this script, you get:
    file://../../../alpha_description/meshes/hip_roll.dae
"""

import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from typing import Optional


def run_xacro(xacro_path: str, extra_args: Optional[list[str]] = None) -> str:
    """Run xacro and return the generated URDF as a string."""
    cmd = ["xacro", xacro_path]
    if extra_args:
        cmd.extend(extra_args)
    try:
        result = subprocess.run(
            cmd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError:
        print("ERROR: xacro command not found. Did you source your ROS 2 environment?", file=sys.stderr)
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print("ERROR: xacro failed:", file=sys.stderr)
        print(e.stderr, file=sys.stderr)
        sys.exit(e.returncode)
    return result.stdout


# -------------------- package:// handling (kept from your original) -------------------- #

def resolve_package_uri_with_ament(uri: str) -> str:
    """Resolve a package:// URI using ament_index_python to an absolute path."""
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError  # type: ignore

    if not uri.startswith("package://"):
        return uri

    rest = uri[len("package://") :]
    pkg_name, sep, rel_path = rest.partition("/")
    if not sep:
        raise ValueError(f"URI '{uri}' does not contain a '/' after package name")

    try:
        share_dir = get_package_share_directory(pkg_name)
    except PackageNotFoundError as e:
        raise RuntimeError(f"Package '{pkg_name}' not found in ament index when resolving '{uri}'") from e

    return os.path.join(share_dir, rel_path)


def resolve_package_uri_with_root(uri: str, mesh_root: str) -> str:
    """Resolve a package:// URI by mapping to <mesh_root>/<pkg_name>/..."""
    if not uri.startswith("package://"):
        return uri

    rest = uri[len("package://") :]
    pkg_name, sep, rel_path = rest.partition("/")
    if not sep:
        raise ValueError(f"URI '{uri}' does not contain a '/' after package name")

    # /mesh_root/pkg_name/<rel_path>
    return os.path.join(mesh_root, pkg_name, rel_path)


# -------------------- NEW: file:// → file://../../../pkg/... handling -------------------- #

_pkg_prefixes_cache = None


def _init_pkg_prefixes():
    """Lazy-load package prefixes from ament_index_python."""
    global _pkg_prefixes_cache
    if _pkg_prefixes_cache is None:
        from ament_index_python.packages import get_packages_with_prefixes  # type: ignore

        _pkg_prefixes_cache = get_packages_with_prefixes()
    return _pkg_prefixes_cache


def rewrite_file_uri_to_relative(filename: str, rel_prefix: str = "../../../") -> str:
    """
    Rewrite a file:// URI pointing into a ROS package share dir to a relative path:

      file:///.../share/<pkg>/meshes/hip_roll.dae
      -> file://../../../<pkg>/meshes/hip_roll.dae

    If we can't match it to any known package, we leave it unchanged.
    """
    if not filename.startswith("file://"):
        return filename

    # Strip scheme and normalize
    full_path = filename[len("file://") :]
    if not full_path.startswith("/"):
        # Not an absolute path; leave as-is
        return filename

    norm_full = os.path.normpath(full_path)

    pkg_prefixes = _init_pkg_prefixes()

    best_match = None  # (share_dir, pkg_name, rel_path)

    for pkg_name, prefix in pkg_prefixes.items():
        # Recreate share dir like get_package_share_directory() does:
        share_dir = os.path.normpath(os.path.join(prefix, "share", pkg_name))
        if norm_full.startswith(share_dir + os.sep):
            rel_path = os.path.relpath(norm_full, share_dir)  # e.g. meshes/hip_roll.dae
            # Prefer the longest matching share_dir (in case of overlays)
            if best_match is None or len(share_dir) > len(best_match[0]):
                best_match = (share_dir, pkg_name, rel_path)

    if best_match is None:
        # Could not match to any installed ROS package; leave unchanged
        return filename

    _, pkg_name, rel_path = best_match
    rel_path = rel_path.replace(os.sep, "/")  # make sure we use forward slashes

    # Build the desired relative URI:
    #   file://../../../<pkg_name>/<rel_path_inside_pkg>
    return f"file://{rel_prefix}{pkg_name}/{rel_path}"


# -------------------- Main URDF rewrite -------------------- #

def convert_urdf_paths(
    urdf_str: str,
    use_ament: bool = True,
    mesh_root: Optional[str] = None,
) -> str:
    """
    Parse URDF XML and rewrite:
      - <mesh filename="...">
      - <uri>...</uri>

    Behavior:
      - If value starts with 'file://', we try to map it to:
          file://../../../<pkg_name>/<rel_path>
        using ament_index_python to detect which package the file lives in.
      - If value starts with 'package://', we either:
          * resolve via ament_index_python to an absolute path, OR
          * if mesh_root is given, map to <mesh_root>/pkg/...
    """
    # package:// resolver (kept for completeness)
    if mesh_root:
        resolver_pkg = lambda u: resolve_package_uri_with_root(u, mesh_root)  # noqa: E731
    elif use_ament:
        try:
            # test import here so we fail early if ROS env not sourced
            from ament_index_python.packages import get_package_share_directory  # noqa: F401  # type: ignore
        except ImportError as e:  # pragma: no cover
            raise RuntimeError(
                "ament_index_python not available. Either source your ROS 2 workspace "
                "or run the script with --mesh-root to avoid needing ROS."
            ) from e
        resolver_pkg = resolve_package_uri_with_ament
    else:
        resolver_pkg = lambda u: u  # noqa: E731

    root = ET.fromstring(urdf_str)

    # 1) <mesh filename="...">
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue

        new_filename = filename

        if filename.startswith("file://"):
            # NEW behavior: rewrite file:// absolute path into relative file://../../../pkg/...
            new_filename = rewrite_file_uri_to_relative(filename)
        elif filename.startswith("package://"):
            # Old behavior retained for package:// URIs if they exist
            abs_path = resolver_pkg(filename)
            # You *could* also run that through rewrite_file_uri_to_relative if you wanted
            # everything to be file://../../../pkg/..., but for now we just store abs_path.
            new_filename = abs_path

        if new_filename != filename:
            mesh.set("filename", new_filename)

    # 2) <uri>...</uri>  (e.g., Gazebo materials)
    for uri_elem in root.iter("uri"):
        if not uri_elem.text:
            continue

        text = uri_elem.text.strip()
        new_text = text

        if text.startswith("file://"):
            new_text = rewrite_file_uri_to_relative(text)
        elif text.startswith("package://"):
            new_text = resolver_pkg(text)

        if new_text != text:
            uri_elem.text = new_text

    # Serialize back to string
    return ET.tostring(root, encoding="unicode")


def main():
    parser = argparse.ArgumentParser(description="Convert xacro to URDF for Isaac Sim (no absolute package paths).")
    parser.add_argument("xacro", help="Input xacro file")
    parser.add_argument(
        "-o",
        "--output",
        help="Output URDF file path (default: same as xacro with .urdf extension)",
    )
    parser.add_argument(
        "--mesh-root",
        help=(
            "Optional base directory for meshes; if set, package://pkg/path "
            "becomes <mesh_root>/pkg/path. If not set, uses ament_index_python "
            "to resolve package:// URIs."
        ),
    )
    parser.add_argument(
        "--xacro-arg",
        action="append",
        default=[],
        help="Extra argument to pass to xacro (can be used multiple times), e.g. --xacro-arg foo:=bar",
    )
    args = parser.parse_args()

    xacro_path = os.path.abspath(args.xacro)
    if not os.path.isfile(xacro_path):
        print(f"ERROR: xacro file '{xacro_path}' does not exist", file=sys.stderr)
        sys.exit(1)

    output_path = args.output
    if not output_path:
        base, _ = os.path.splitext(xacro_path)
        output_path = base + ".urdf"
    output_path = os.path.abspath(output_path)

    mesh_root = args.mesh_root or os.environ.get("MESH_ROOT")

    print(f"[convert] Running xacro on: {xacro_path}")
    if mesh_root:
        print(f"[convert] Using mesh_root={mesh_root} for package:// remapping")
    else:
        print("[convert] Using ament_index_python to resolve package:// URIs (if any)")
        print("[convert] file:// URIs pointing into ROS packages will be rewritten to file://../../../<pkg>/...")

    # Run xacro
    urdf_str = run_xacro(xacro_path, extra_args=args.xacro_arg)

    # Rewrite paths
    try:
        new_urdf_str = convert_urdf_paths(
            urdf_str,
            use_ament=(mesh_root is None),
            mesh_root=mesh_root,
        )
    except Exception as e:
        print(f"ERROR while rewriting URDF mesh paths: {e}", file=sys.stderr)
        sys.exit(1)

    # Write output
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(new_urdf_str)

    print(f"[convert] Wrote Isaac-ready URDF to: {output_path}")


if __name__ == "__main__":
    main()
