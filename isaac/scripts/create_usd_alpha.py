#!/usr/bin/env python3
import sys
import argparse
import os
from pathlib import Path

script_dir = Path(__file__).resolve().parent

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


# ======================================================================
# Wheel + joint helpers
# ======================================================================
def _resolve_prim_path(robot_root: str, maybe_rel: str) -> str:
    """Accept absolute prim paths or paths relative to robot_root."""
    if maybe_rel.startswith("/"):
        return maybe_rel
    return f"{robot_root}/{maybe_rel.lstrip('/')}"


def replace_link_prim_with_usd_reference(
    stage,
    robot_root: str,
    link_rel_or_abs: str,
    usd_path: str,
    *,
    translate=(0.0, 0.0, 0.0),
    rotate_deg=(0.0, 0.0, 0.0),
    scale=(1.0, 1.0, 1.0),
    reference_prim_path_in_asset: str | None = None,
) -> bool:
    """
    Replace a link prim (at the same prim path) by referencing a USD asset into it.
    This keeps joints stable *as long as* they still point to the same prim paths or are retargeted after.
    """
    link_path = _resolve_prim_path(robot_root, link_rel_or_abs)
    link_sdf = Sdf.Path(link_path)

    old = stage.GetPrimAtPath(link_sdf)
    if not old.IsValid():
        print(f"[WARN] Link prim not found: {link_path}")
        return False

    stage.RemovePrim(link_sdf)

    xf = UsdGeom.Xform.Define(stage, link_sdf)
    prim = xf.GetPrim()

    if reference_prim_path_in_asset:
        prim.GetReferences().AddReference(usd_path, reference_prim_path_in_asset)
    else:
        prim.GetReferences().AddReference(usd_path)

    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(tuple(translate))
    api.SetRotate(tuple(rotate_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    api.SetScale(tuple(scale))

    print(f"[INFO] Replaced link prim: {link_path} -> {usd_path}")
    return True

def set_joint_angular_drive_max_force(stage, joint_prim_path: str, max_force: float,
                                      stiffness: float | None = None,
                                      damping: float | None = None):
    """
    Applies/updates an angular DriveAPI on a joint and sets max force.
    Optionally also sets stiffness/damping.
    """
    prim = stage.GetPrimAtPath(Sdf.Path(joint_prim_path))
    if not prim.IsValid():
        print(f"[WARN] Joint prim not found: {joint_prim_path}")
        return False

    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")

    if stiffness is not None:
        drive.CreateStiffnessAttr(float(stiffness))
    if damping is not None:
        drive.CreateDampingAttr(float(damping))

    # Create* updates if exists, creates if missing
    drive.CreateMaxForceAttr(float(max_force))

    print(f"[INFO] Set angular maxForce={max_force} on {joint_prim_path}")
    return True

def set_joint_body1(stage, joint_path: str, new_body1_path: str) -> bool:
    """Set body1 target on a PhysicsRevoluteJoint."""
    joint_prim = stage.GetPrimAtPath(Sdf.Path(joint_path))
    if not joint_prim.IsValid():
        print(f"[WARN] Joint not found: {joint_path}")
        return False

    joint = UsdPhysics.RevoluteJoint(joint_prim)
    joint.GetBody1Rel().SetTargets([Sdf.Path(new_body1_path)])

    print(f"[INFO] {joint_path}: body1 -> {new_body1_path}")
    return True


def retarget_wheel_joint_body1s(stage, robot_root: str, wheel_body1_by_index: dict[int, str]):
    """
    wheel_body1_by_index: {1: "<prim path for body1>", ...}
    Assumes joints at: <robot_root>/joints/joint_wheel1 .. joint_wheel4
    """
    for i, body1_path in wheel_body1_by_index.items():
        joint_path = f"{robot_root}/joints/joint_wheel{i}"
        set_joint_body1(stage, joint_path, body1_path)


def set_joint_local_rot1_xyz_deg(stage, joint_path: str, x_deg=0.0, y_deg=0.0, z_deg=0.0) -> bool:
    """
    Set physics:localRot1 on a PhysicsRevoluteJoint prim.
    Handles quatf vs quatd automatically.
    """
    prim = stage.GetPrimAtPath(Sdf.Path(joint_path))
    if not prim.IsValid():
        print(f"[WARN] Joint not found: {joint_path}")
        return False

    rot = (
        Gf.Rotation(Gf.Vec3d(1, 0, 0), float(x_deg)) *
        Gf.Rotation(Gf.Vec3d(0, 1, 0), float(y_deg)) *
        Gf.Rotation(Gf.Vec3d(0, 0, 1), float(z_deg))
    )
    qd = rot.GetQuat()  # Quatd

    attr = prim.GetAttribute("physics:localRot1")
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf)

    tname = attr.GetTypeName()

    if tname == Sdf.ValueTypeNames.Quatf:
        qf = Gf.Quatf(float(qd.GetReal()), Gf.Vec3f(*[float(v) for v in qd.GetImaginary()]))
        attr.Set(qf)
    else:
        # Quatd or other: try Quatd; fallback to Quatf if needed
        try:
            attr.Set(qd)
        except Exception:
            qf = Gf.Quatf(float(qd.GetReal()), Gf.Vec3f(*[float(v) for v in qd.GetImaginary()]))
            attr.Set(qf)

    print(f"[INFO] Set {joint_path} physics:localRot1 to XYZ deg ({x_deg}, {y_deg}, {z_deg}) as {tname}")
    return True


def setup_wheels(stage, robot_path: str):
    """
    One place that contains all wheel-related configuration and steps.
    """
    # --- Wheel configuration (edit here) ---
    wheels = {
        1: {
            "link": "wheel1",
            "usd": os.path.join(script_dir, "../models/alpha/usd/assets/omniwheel_fl.usd"),
            "t": (0.15735,  0.2509, 0.1015),
            "r": (0.0, 0.0, 0.0),
            "s": (1.0, 1.0, 1.0),
            "body1": f"{robot_path}/wheel1/omniwheel_fl/omniwheel",
            "joint_localrot1_z": -90.0,
        },
        2: {
            "link": "wheel2",
            "usd": os.path.join(script_dir, "../models/alpha/usd/assets/omniwheel_fr.usd"),
            "t": (0.15735, -0.2509, 0.1015),
            "r": (0.0, 0.0, 0.0),
            "s": (1.0, 1.0, 1.0),
            "body1": f"{robot_path}/wheel2/omniwheel_fr/omniwheel",
            "joint_localrot1_z": 90.0,
        },
        3: {
            "link": "wheel3",
            "usd": os.path.join(script_dir, "../models/alpha/usd/assets/omniwheel_rr.usd"),
            "t": (-0.6158, -0.2509, 0.1015),
            "r": (0.0, 0.0, 0.0),
            "s": (1.0, 1.0, 1.0),
            "body1": f"{robot_path}/wheel3/omniwheel_rr/omniwheel",
            "joint_localrot1_z": 90.0,
        },
        4: {
            "link": "wheel4",
            "usd": os.path.join(script_dir, "../models/alpha/usd/assets/omniwheel_rl.usd"),
            "t": (-0.6158,  0.2509, 0.1015),
            "r": (0.0, 0.0, 0.0),
            "s": (1.0, 1.0, 1.0),
            "body1": f"{robot_path}/wheel4/omniwheel_rl/omniwheel",
            "joint_localrot1_z": -90.0,
        },
    }

    # --- Replace wheel link prims ---
    for i, cfg in wheels.items():
        replace_link_prim_with_usd_reference(
            stage,
            robot_root=robot_path,
            link_rel_or_abs=cfg["link"],
            usd_path=cfg["usd"],
            translate=cfg["t"],
            rotate_deg=cfg["r"],
            scale=cfg["s"],
        )

    kit.update()

    # --- Retarget joints body1 ---
    wheel_body1_by_index = {i: cfg["body1"] for i, cfg in wheels.items()}
    retarget_wheel_joint_body1s(stage, robot_path, wheel_body1_by_index)
    kit.update()

    # --- Fix joint Local Rotation 1 ---
    for i, cfg in wheels.items():
        set_joint_local_rot1_xyz_deg(
            stage,
            f"{robot_path}/joints/joint_wheel{i}",
            z_deg=cfg["joint_localrot1_z"],
        )

    kit.update()


# ======================================================================
# Sensors & lights
# ======================================================================
def add_realsense_d455_to_robot(stage, robot_root_prim: str) -> str:
    assets_root = get_assets_root_path()
    d455_usd = assets_root + "/Isaac/Sensors/Intel/RealSense/rsd455.usd"

    d455_root_prim = f"{robot_root_prim}/camera_link"
    realsense_d455 = SingleViewDepthSensorAsset(prim_path=d455_root_prim, asset_path=d455_usd)
    realsense_d455.initialize()

    d455_prim = stage.GetPrimAtPath(d455_root_prim)
    UsdGeom.XformCommonAPI(d455_prim).SetTranslate((0.0, 0.0, 0.0))
    UsdGeom.XformCommonAPI(d455_prim).SetRotate((0.0, 0.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    camera_prim_path_depth = d455_root_prim + "/RSD455/Camera_Pseudo_Depth"
    print(f"[INFO] RealSense D455 mounted at {d455_root_prim}")
    print(f"[INFO] D455 depth camera prim: {camera_prim_path_depth}")
    return camera_prim_path_depth


def add_rtx_lidar_to_robot(stage, robot_root_prim: str) -> str:
    lidar_parent = f"{robot_root_prim}/lidar"
    lidar_path = f"{lidar_parent}/rtx_lidar"

    omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_path,
        parent="",
        config="Example_Rotary_2D",
        translation=(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )

    print(f"[INFO] RTX Lidar created at {lidar_path}")
    return lidar_path


def configure_wheel_joint_drives(stage, robot_root_prim: str):
    wheel_joint_paths = [
        Sdf.Path(f"{robot_root_prim}/joints/joint_wheel1"),
        Sdf.Path(f"{robot_root_prim}/joints/joint_wheel2"),
        Sdf.Path(f"{robot_root_prim}/joints/joint_wheel3"),
        Sdf.Path(f"{robot_root_prim}/joints/joint_wheel4"),
    ]

    for p in wheel_joint_paths:
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            print(f"[WARN] Wheel joint prim not found (adjust path!): {p}")
            continue

        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive_api.CreateStiffnessAttr(0.0)
        drive_api.CreateDampingAttr(1000.0)
        drive_api.CreateMaxForceAttr(1000.0)

        print(f"[INFO] Configured wheel joint drive on {p}")


# ======================================================================
# OmniGraph creation functions
# (kept as-is; only removed unused comments + kept naming consistent)
# ======================================================================
def create_rtf_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/PublishRTF"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("GenericPublisher", "isaacsim.ros2.bridge.ROS2Publisher"),
                ("RTF", "isaacsim.core.nodes.IsaacRealTimeFactor"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("GenericPublisher.inputs:messageName", "Float32"),
                ("GenericPublisher.inputs:messagePackage", "std_msgs"),
                ("GenericPublisher.inputs:messageSubfolder", "msg"),
                ("GenericPublisher.inputs:topicName", "/isaac/rtf"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "GenericPublisher.inputs:execIn"),
                ("Context.outputs:context", "GenericPublisher.inputs:context"),
            ],
        },
        update_usd=True,
    )

    og.Controller.connect(
        og.Controller.attribute(f"{graph_path}/RTF.outputs:rtf"),
        og.Controller.attribute(f"{graph_path}/GenericPublisher.inputs:data"),
        update_usd=True,
    )

def create_camera_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/CameraRosGraph"

    color_cam = robot_root_prim + "/camera_link/RSD455/Camera_OmniVision_OV9782_Color"
    depth_cam = robot_root_prim + "/camera_link/RSD455/Camera_Pseudo_Depth"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOneSimFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),

                ("CreateRenderProductRGB", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CreateRenderProductDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),

                ("CameraRosHelperRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraRosHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),

                ("CameraRosInfoHelperRGB", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("CameraRosInfoHelperDepth", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneSimFrame.inputs:execIn"),

                ("RunOneSimFrame.outputs:step", "CreateRenderProductRGB.inputs:execIn"),
                ("RunOneSimFrame.outputs:step", "CreateRenderProductDepth.inputs:execIn"),

                ("CreateRenderProductRGB.outputs:renderProductPath",
                 "CameraRosHelperRGB.inputs:renderProductPath"),
                ("CreateRenderProductDepth.outputs:renderProductPath",
                 "CameraRosHelperDepth.inputs:renderProductPath"),

                ("CreateRenderProductRGB.outputs:execOut", "CameraRosHelperRGB.inputs:execIn"),
                ("CreateRenderProductRGB.outputs:renderProductPath",
                 "CameraRosInfoHelperRGB.inputs:renderProductPath"),
                ("CreateRenderProductRGB.outputs:execOut", "CameraRosInfoHelperRGB.inputs:execIn"),

                ("CreateRenderProductDepth.outputs:execOut", "CameraRosHelperDepth.inputs:execIn"),
                ("CreateRenderProductDepth.outputs:renderProductPath",
                 "CameraRosInfoHelperDepth.inputs:renderProductPath"),
                ("CreateRenderProductDepth.outputs:execOut", "CameraRosInfoHelperDepth.inputs:execIn"),

                ("ROS2Context.outputs:context", "CameraRosHelperRGB.inputs:context"),
                ("ROS2Context.outputs:context", "CameraRosInfoHelperRGB.inputs:context"),
                ("ROS2Context.outputs:context", "CameraRosHelperDepth.inputs:context"),
                ("ROS2Context.outputs:context", "CameraRosInfoHelperDepth.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:useDomainIDEnvVar", True),

                ("CreateRenderProductRGB.inputs:cameraPrim", color_cam),
                ("CreateRenderProductRGB.inputs:width", 640),
                ("CreateRenderProductRGB.inputs:height", 480),

                ("CreateRenderProductDepth.inputs:cameraPrim", depth_cam),
                ("CreateRenderProductDepth.inputs:width", 640),
                ("CreateRenderProductDepth.inputs:height", 480),

                ("CameraRosHelperRGB.inputs:topicName", "/camera/rgb/image_raw"),
                ("CameraRosHelperRGB.inputs:frameId", "camera_color_optical_frame"),
                ("CameraRosHelperRGB.inputs:type", "rgb"),

                ("CameraRosHelperDepth.inputs:topicName", "/camera/depth/image_raw"),
                ("CameraRosHelperDepth.inputs:frameId", "camera_depth_optical_frame"),
                ("CameraRosHelperDepth.inputs:type", "depth"),

                ("CameraRosInfoHelperRGB.inputs:topicName", "/camera/rgb/camera_info"),
                ("CameraRosInfoHelperRGB.inputs:frameId", "camera_color_optical_frame"),

                ("CameraRosInfoHelperDepth.inputs:topicName", "/camera/depth/camera_info"),
                ("CameraRosInfoHelperDepth.inputs:frameId", "camera_depth_optical_frame"),
            ],
        },
        update_usd=True,
    )

def create_imu_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/IMUGraph"
    imu_prim = robot_root_prim + "/camera_link/RSD455/Imu_Sensor"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacSimulationGate", "isaacsim.core.nodes.IsaacSimulationGate"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("IsaacReadIMUNode", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("IsaacReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ROS2PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "IsaacSimulationGate.inputs:execIn"),

                ("IsaacSimulationGate.outputs:execOut", "IsaacReadIMUNode.inputs:execIn"),
                ("ROS2Context.outputs:context", "ROS2PublishIMU.inputs:context"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishIMU.inputs:timeStamp"),

                ("IsaacReadIMUNode.outputs:execOut", "ROS2PublishIMU.inputs:execIn"),
                ("IsaacReadIMUNode.outputs:angVel", "ROS2PublishIMU.inputs:angularVelocity"),
                ("IsaacReadIMUNode.outputs:linAcc", "ROS2PublishIMU.inputs:linearAcceleration"),
                ("IsaacReadIMUNode.outputs:orientation", "ROS2PublishIMU.inputs:orientation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ROS2PublishIMU.inputs:frameId", "camera_link"),
                ("ROS2PublishIMU.inputs:topicName", "/sensor/imu/imu"),
                ("IsaacReadIMUNode.inputs:imuPrim", imu_prim),
            ],
        },
        update_usd = True,
    )

def create_rtx_lidar_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/RtxLidarGraph"
    lidar_prim = robot_root_prim + "/lidar/rtx_lidar"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOneSimFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RtxLidarRosHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneSimFrame.inputs:execIn"),
                ("RunOneSimFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),

                ("CreateRenderProduct.outputs:renderProductPath",
                 "RtxLidarRosHelper.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:execOut", "RtxLidarRosHelper.inputs:execIn"),

                ("ROS2Context.outputs:context", "RtxLidarRosHelper.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:useDomainIDEnvVar", True),

                ("CreateRenderProduct.inputs:cameraPrim", lidar_prim),
                ("CreateRenderProduct.inputs:width", 1),
                ("CreateRenderProduct.inputs:height", 1),

                ("RtxLidarRosHelper.inputs:topicName", "/sensor/lidar/scan_raw"),
                ("RtxLidarRosHelper.inputs:frameId", "lidar"),
            ],
        },
        update_usd=True,
    )


def create_torso_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/TorsoControl"
    robot_base = robot_root_prim + "/base_link"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),

                ("SubscribeJointStateArmLeft",  "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerArmLeft", "isaacsim.core.nodes.IsaacArticulationController"),

                ("SubscribeJointStateArmRight", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerArmRight", "isaacsim.core.nodes.IsaacArticulationController"),

                ("SubscribeJointStateHip", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerHip", "isaacsim.core.nodes.IsaacArticulationController"),

                ("GetPrimPath", "omni.graph.nodes.GetPrimPath"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateArmLeft.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerArmLeft.inputs:execIn"),

                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateArmRight.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerArmRight.inputs:execIn"),

                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateHip.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerHip.inputs:execIn"),

                ("GetPrimPath.outputs:path", "ArticulationControllerArmLeft.inputs:robotPath"),
                ("GetPrimPath.outputs:path", "ArticulationControllerArmRight.inputs:robotPath"),
                ("GetPrimPath.outputs:path", "ArticulationControllerHip.inputs:robotPath"),

                ("SubscribeJointStateArmLeft.outputs:jointNames",
                 "ArticulationControllerArmLeft.inputs:jointNames"),
                ("SubscribeJointStateArmLeft.outputs:positionCommand",
                 "ArticulationControllerArmLeft.inputs:positionCommand"),
                ("SubscribeJointStateArmLeft.outputs:velocityCommand",
                 "ArticulationControllerArmLeft.inputs:velocityCommand"),
                ("SubscribeJointStateArmLeft.outputs:effortCommand",
                 "ArticulationControllerArmLeft.inputs:effortCommand"),

                ("SubscribeJointStateArmRight.outputs:jointNames",
                 "ArticulationControllerArmRight.inputs:jointNames"),
                ("SubscribeJointStateArmRight.outputs:positionCommand",
                 "ArticulationControllerArmRight.inputs:positionCommand"),
                ("SubscribeJointStateArmRight.outputs:velocityCommand",
                 "ArticulationControllerArmRight.inputs:velocityCommand"),
                ("SubscribeJointStateArmRight.outputs:effortCommand",
                 "ArticulationControllerArmRight.inputs:effortCommand"),

                ("SubscribeJointStateHip.outputs:jointNames",
                 "ArticulationControllerHip.inputs:jointNames"),
                ("SubscribeJointStateHip.outputs:positionCommand",
                 "ArticulationControllerHip.inputs:positionCommand"),
                ("SubscribeJointStateHip.outputs:velocityCommand",
                 "ArticulationControllerHip.inputs:velocityCommand"),
                ("SubscribeJointStateHip.outputs:effortCommand",
                 "ArticulationControllerHip.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeJointStateArmLeft.inputs:topicName", "/arm_left_isaac_joint_command"),
                ("SubscribeJointStateArmRight.inputs:topicName", "/arm_right_isaac_joint_command"),
                ("SubscribeJointStateHip.inputs:topicName", "/hip_isaac_joint_command"),
                ("GetPrimPath.inputs:prim", robot_base),
            ],
        },
        update_usd=True,
    )


def create_platform_graph(robot_root_prim: str):
    graph_path = f"{robot_root_prim}/ActionGraphs/PlatformControl"
    robot_base = robot_root_prim + "/base_link"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),

                ("SubscribeJointStateWheel1", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerWheel1", "isaacsim.core.nodes.IsaacArticulationController"),

                ("SubscribeJointStateWheel2", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerWheel2", "isaacsim.core.nodes.IsaacArticulationController"),

                ("SubscribeJointStateWheel3", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerWheel3", "isaacsim.core.nodes.IsaacArticulationController"),

                ("SubscribeJointStateWheel4", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerWheel4", "isaacsim.core.nodes.IsaacArticulationController"),

                ("GetPrimPath", "omni.graph.nodes.GetPrimPath"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateWheel1.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerWheel1.inputs:execIn"),

                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateWheel2.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerWheel2.inputs:execIn"),

                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateWheel3.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerWheel3.inputs:execIn"),

                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateWheel4.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerWheel4.inputs:execIn"),

                ("GetPrimPath.outputs:path", "ArticulationControllerWheel1.inputs:robotPath"),
                ("GetPrimPath.outputs:path", "ArticulationControllerWheel2.inputs:robotPath"),
                ("GetPrimPath.outputs:path", "ArticulationControllerWheel3.inputs:robotPath"),
                ("GetPrimPath.outputs:path", "ArticulationControllerWheel4.inputs:robotPath"),

                ("SubscribeJointStateWheel1.outputs:jointNames",
                 "ArticulationControllerWheel1.inputs:jointNames"),
                ("SubscribeJointStateWheel1.outputs:velocityCommand",
                 "ArticulationControllerWheel1.inputs:velocityCommand"),

                ("SubscribeJointStateWheel2.outputs:jointNames",
                 "ArticulationControllerWheel2.inputs:jointNames"),
                ("SubscribeJointStateWheel2.outputs:velocityCommand",
                 "ArticulationControllerWheel2.inputs:velocityCommand"),

                ("SubscribeJointStateWheel3.outputs:jointNames",
                 "ArticulationControllerWheel3.inputs:jointNames"),
                ("SubscribeJointStateWheel3.outputs:velocityCommand",
                 "ArticulationControllerWheel3.inputs:velocityCommand"),

                ("SubscribeJointStateWheel4.outputs:jointNames",
                 "ArticulationControllerWheel4.inputs:jointNames"),
                ("SubscribeJointStateWheel4.outputs:velocityCommand",
                 "ArticulationControllerWheel4.inputs:velocityCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeJointStateWheel1.inputs:topicName", "/wheel1_isaac_joint_command"),
                ("SubscribeJointStateWheel2.inputs:topicName", "/wheel2_isaac_joint_command"),
                ("SubscribeJointStateWheel3.inputs:topicName", "/wheel3_isaac_joint_command"),
                ("SubscribeJointStateWheel4.inputs:topicName", "/wheel4_isaac_joint_command"),
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


# ======================================================================
# Main pipeline
# ======================================================================
print(f"[INFO] URDF extension path: {get_extension_path_from_name('isaacsim.asset.importer.urdf')}")
print(f"[INFO] Importing URDF from: {URDF_PATH}")

_, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False
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

# Wheels: replace link prims + retarget joints + fix joint localRot1
setup_wheels(stage, robot_path)

# Optional environment
if ENV_USD_PATH:
    add_reference_to_stage(usd_path=ENV_USD_PATH, prim_path="/World/Environment")
    print(f"[INFO] Loaded environment: {ENV_USD_PATH}")
else:
    print("[INFO] No environment specified; skipping.")

# Sensors & lighting
add_rtx_lidar_to_robot(stage, robot_path)
add_realsense_d455_to_robot(stage, robot_path)

distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distant_light.CreateIntensityAttr(300)
print("[INFO] Added distant light")

configure_wheel_joint_drives(stage, robot_path)

# Graphs
create_camera_graph(robot_path)
create_rtx_lidar_graph(robot_path)
create_platform_graph(robot_path)
create_torso_graph(robot_path)
create_joint_states_graph(robot_path)
create_rtf_graph(robot_path)
create_imu_graph(robot_path)

HIP_MAX_FORCE = 1000.0

set_joint_angular_drive_max_force(stage, f"{robot_path}/joints/hip_roll",  HIP_MAX_FORCE)
set_joint_angular_drive_max_force(stage, f"{robot_path}/joints/hip_pitch", HIP_MAX_FORCE)

kit.update()

# Save
omni.usd.get_context().save_as_stage(OUTPUT_USD_PATH)
print(f"[INFO] Saved robot USD → {OUTPUT_USD_PATH}")

kit.close()
