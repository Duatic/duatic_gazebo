#!/usr/bin/env python3
import sys
import argparse
import os

# ----------------------------------------------------------------------
# Argument parsing that works with ./python.sh (ignores Isaac Sim args)
# ----------------------------------------------------------------------
def parse_args_isaac_safe():
    parser = argparse.ArgumentParser(
        description="Generate a robot USD from URDF, swap wheels, wire ROS2 OmniGraphs, and save.",
        add_help=True,
    )

    parser.add_argument("--urdf", type=str, help="Path to the input URDF file")
    parser.add_argument("--output_usd", type=str, help="Path to the output USD file")
    parser.add_argument(
        "--environment",
        type=str,
        help="(Optional) Path to environment USD. Leave empty if you want a reusable robot asset.",
    )

    args, _unknown = parser.parse_known_args()

    missing = []
    if not args.urdf:
        missing.append("--urdf")
    if not args.output_usd:
        missing.append("--output_usd")

    if missing:
        print(f"[ERROR] Missing required argument(s): {', '.join(missing)}")
        print(
            "Usage example:\n"
            "  ./python.sh create_usd_alpha_clean.py "
            "--urdf path/to/robot.urdf --output_usd /tmp/robot.usd"
        )
        sys.exit(1)

    return args


args = parse_args_isaac_safe()
URDF_PATH = args.urdf
OUTPUT_USD_PATH = args.output_usd
ENV_USD_PATH = args.environment

# ----------------------------------------------------------------------
# Isaac Sim / Omniverse imports
# ----------------------------------------------------------------------
from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import omni
import omni.kit.commands
import omni.usd
import omni.graph.core as og

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import get_extension_path_from_name, enable_extension
from isaacsim.sensors.camera import SingleViewDepthSensorAsset
from isaacsim.storage.native import get_assets_root_path

from pxr import UsdLux, UsdGeom, UsdPhysics, Sdf, Gf, Usd


# ======================================================================
# Enable required extensions
# ======================================================================
enable_extension("isaacsim.ros2.bridge")
kit.update()

# ActionGraph creation functions
def create_torso_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/DynaarmControl"
    robot_base = robot_root_prim

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),

                ("SubscribeJointStateDynaarm",  "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerDynaarm", "isaacsim.core.nodes.IsaacArticulationController"),

                ("GetPrimPath", "omni.graph.nodes.GetPrimPath"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateDynaarm.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerDynaarm.inputs:execIn"),

                ("GetPrimPath.outputs:path", "ArticulationControllerDynaarm.inputs:robotPath"),

                ("SubscribeJointStateDynaarm.outputs:jointNames",
                 "ArticulationControllerDynaarm.inputs:jointNames"),
                ("SubscribeJointStateDynaarm.outputs:positionCommand",
                 "ArticulationControllerDynaarm.inputs:positionCommand"),
                ("SubscribeJointStateDynaarm.outputs:velocityCommand",
                 "ArticulationControllerDynaarm.inputs:velocityCommand"),
                ("SubscribeJointStateDynaarm.outputs:effortCommand",
                 "ArticulationControllerDynaarm.inputs:effortCommand"),

            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeJointStateDynaarm.inputs:topicName", "/isaac_joint_command"),
                ("GetPrimPath.inputs:prim", robot_base),
            ],
        },
        update_usd=True,
    )

def create_joint_states_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/PublishJointStates"
    robot_base = robot_root_prim + "/base_link"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:targetPrim", robot_base),
                ("PublishJointState.inputs:topicName", "/isaac_joint_states"),
            ],
        },
        update_usd=True,
    )

print(f"[INFO] URDF extension path: {get_extension_path_from_name('isaacsim.asset.importer.urdf')}")
print(f"[INFO] Importing URDF from: {URDF_PATH}")

_, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = True
import_config.distance_scale = 1.0

_, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    get_articulation_root=True,
)

robot_path = os.path.dirname(prim_path)
print(f"[INFO] URDF imported, articulation root prim_path: {robot_path}")

stage = omni.usd.get_context().get_stage()

# Optional environment
if ENV_USD_PATH:
    add_reference_to_stage(usd_path=ENV_USD_PATH, prim_path="/World/Environment")
    print(f"[INFO] Loaded environment: {ENV_USD_PATH}")
else:
    print("[INFO] No environment specified; skipping.")


# Lighting
distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distant_light.CreateIntensityAttr(300)
print("[INFO] Added distant light")

# Graphs
create_torso_graph(robot_path)
create_joint_states_graph(robot_path)

kit.update()

# Save
omni.usd.get_context().save_as_stage(OUTPUT_USD_PATH)
print(f"[INFO] Saved robot USD → {OUTPUT_USD_PATH}")

kit.close()