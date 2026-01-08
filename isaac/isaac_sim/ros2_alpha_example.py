import os
import numpy as np

# Optional (but common): set ROS domain before Isaac starts
os.environ.setdefault("ROS_DOMAIN_ID", "0")

script_path = os.path.abspath(__file__)
script_dir  = os.path.dirname(script_path)

# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import os
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# --- (optional) get path of this running script ---
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()

# --- Enable ROS 2 bridge (new + old extension names) ---
# Newer Isaac Sim uses renamed extensions like "isaacsim.ros2.bridge".
# Older versions used "omni.isaac.ros2_bridge".
try:
    enable_extension("isaacsim.ros2.bridge")
except Exception:
    enable_extension("omni.isaac.ros2_bridge")

# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)

# Load the Warehouse environment (as a USD reference)
warehouse_usd = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")

# camera
set_camera_view(
    eye=[5.0, 0.0, 1.5], target=[0.0, 0.0, 1.0], camera_prim_path="/OmniverseKit_Persp"
)

# Add YOUR robot USD (local path example; adjust to your file)
# Tip: build it relative to the script folder if you want:
# robot_usd = os.path.join(SCRIPT_DIR, "assets", "my_robot.usd")
robot_usd = os.path.join(script_dir, "../models/alpha/usd/alpha.usd")
robot_prim_path = "/World/Robot"
add_reference_to_stage(usd_path=robot_usd, prim_path=robot_prim_path)

# Create an articulation handle (assumes the robot USD root is an articulation)
robot = Articulation(prim_paths_expr=robot_prim_path, name="alpha")

# Optional: move robot a bit above ground (warehouse floor is usually at z=0)
robot.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]) / get_stage_units())

# initialize the world (this also starts physics state)
my_world.reset()

# --- run / play the simulation ---
while simulation_app.is_running():
    # step the simulation, both rendering and physics
    my_world.step(render=True)

simulation_app.close()
