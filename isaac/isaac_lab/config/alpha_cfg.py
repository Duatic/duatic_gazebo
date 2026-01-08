import os
from isaaclab.assets import ArticulationCfg
from isaaclab.sim import UsdFileCfg, RigidBodyPropertiesCfg, ArticulationRootPropertiesCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass

@configclass
class AlphaRobotCfg(ArticulationCfg):
    """Alpha robot spawned from a USD generated via URDF import pipeline."""

    prim_path = "{ENV_REGEX_NS}/Robot"

    script_path = os.path.abspath(__file__)
    script_dir  = os.path.dirname(script_path)

    spawn = UsdFileCfg(
        usd_path=os.path.join(script_dir, "../models/alpha/usd/alpha.usd"),
    )

    # Physics tuning
    rigid_props = RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=100.0,
        max_angular_velocity=100.0,
        max_depenetration_velocity=5.0,
    )

    articulation_props = ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=1,
        sleep_threshold=0.0,
        stabilization_threshold=0.0,
    )

    # --- Initial state ---
    init_state = ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.25),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            # Add joint names
        },
        joint_vel={
            # Wheels start stopped
            "joint_wheel1": 0.0,
            "joint_wheel2": 0.0,
            "joint_wheel3": 0.0,
            "joint_wheel4": 0.0,
        },
    )

    # --- Actuators ---
    actuators = {
        # Wheels: velocity control
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["joint_wheel[1-4]"],
            effort_limit=1000.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=1000.0,
        ),

        # Hip: position control
        "hip": ImplicitActuatorCfg(
            joint_names_expr=[
                "hip_roll",
                "hip_pitch",
            ],
            effort_limit=1000.0,
            velocity_limit=10.0,
            stiffness=200.0,
            damping=20.0,
        ),

        # Arms: position control
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*arm_left.*",
                ".*arm_right.*",
            ],
            effort_limit=200.0,
            velocity_limit=10.0,
            stiffness=150.0,
            damping=15.0,
        ),
    }

# Convenience instance
ALPHA_CFG = AlphaRobotCfg()
