# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates a simple manager-based environment for a 4-wheeled robot.

It uses a robot config from `my_robot_cfg.py` (e.g. `ALPHA_CFG`) and commands the wheels
to drive forward at a constant velocity. No height scanner / ray caster is used.

.. code-block:: bash

    # Run the script
    ./isaaclab.sh -p path/to/script/alpha_example.py --num_envs 8 --device cuda:0

"""

"""Launch Isaac Sim Simulator first."""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Example: alpharobot base environment")
parser.add_argument("--num_envs", type=int, default=4, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise


# Import robot config
from config.alpha_cfg import ALPHA_CFG

# Custom command term (constant forward drive)
def constant_base_command(env: ManagerBasedEnv) -> torch.Tensor:
    """Constant command: drive forward in +X at 1 m/s."""
    return torch.tensor([[1.0, 0.0, 0.0]], device=env.device).repeat(env.num_envs, 1)

# Scene definition
@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Example scene configuration."""

    # simple ground plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False,
    )

    # robot
    robot: ArticulationCfg = ALPHA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

# MDP settings
@configclass
class ActionsCfg:
    """Action specifications for the MDP.

    NOTE:
    - Joint names must match the *joint names* in the articulation.
    - If your joints are not named joint_wheel1..4, update the regex.
    """

    wheel_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["joint_wheel[1-4]"],
        scale=10.0,
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.10, n_max=0.10))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.02, n_max=0.02))

        velocity_commands = ObsTerm(func=constant_base_command)

        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.0, n_max=1.0))

        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()

@configclass
class EventCfg:
    """Configuration for events."""

    reset_scene = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

# Environment configuration
@configclass
class FourWheelEnvCfg(ManagerBasedEnvCfg):
    """Configuration for a basic 4-wheeled environment (no height scanner)."""

    scene: MySceneCfg = MySceneCfg(num_envs=args_cli.num_envs, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 0.005
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.device = args_cli.device

# Main loop
def main():
    env_cfg = FourWheelEnvCfg()
    env = ManagerBasedEnv(cfg=env_cfg)

    count = 0
    obs, _ = env.reset()

    while simulation_app.is_running():
        with torch.inference_mode():
            if count % 1000 == 0:
                obs, _ = env.reset()
                count = 0
                print("-" * 80)
                print("[INFO]: Resetting environment...")

            # 4 wheels -> action dim 4
            action = torch.ones((env.num_envs, 4), device=env.device) * 0.5

            obs, _ = env.step(action)
            count += 1

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
