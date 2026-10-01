from itertools import product
import os

from duatic_gazebo.utils import join_env_paths


def test_join_env_paths():
    test_paths = ["", "p1", os.pathsep + "p2", "p3" + os.pathsep, "p4" + os.pathsep + "p5"]
    for a, b in product(test_paths, test_paths):
        joined = join_env_paths(a, b)
        context = f"join_env_paths({a!r}, {b!r}) = {joined!r}"
        assert joined.strip(os.pathsep) == joined, f"leading/trailing '{os.pathsep}' for {context}"
        assert 2 * os.pathsep not in joined, f"double '{os.pathsep}' (empty path) for {context}"
        assert all(
            [len(entry) in [0, 2] for entry in joined.split(os.pathsep)]
        ), f"entries not separated by '{os.pathsep}' for {context}"
