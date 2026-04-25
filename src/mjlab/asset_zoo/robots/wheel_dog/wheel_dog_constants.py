"""Wheel dog constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import ElectricActuator, reflected_inertia, rpm_to_rad
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

WHEEL_DOG_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "wheel_dog" / "xmls" / "wheel_dog.xml"
)
assert WHEEL_DOG_XML.exists()


def get_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_file(str(WHEEL_DOG_XML))


##
# Actuator config.
##

# Rotor inertia.
# Ref: https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go1_description/urdf/go1.urdf#L515
# Extracted Ixx (rotation along x-axis).
# TODO(me): 虽然可能没用，需要修改
ROTOR_INERTIA = 0.000111842

# Gearbox.
# TODO(me): 采用输出轴参数，可能用不到,考虑设置为1
HIP_GEAR_RATIO = 9
# 没有齿轮，不加1.5
# CALF_GEAR_RATIO = HIP_GEAR_RATIO * 1.5
CALF_GEAR_RATIO = HIP_GEAR_RATIO

HIP_ACTUATOR = ElectricActuator(
  reflected_inertia=reflected_inertia(ROTOR_INERTIA, HIP_GEAR_RATIO),
  velocity_limit=rpm_to_rad(480),
  effort_limit=36.0,
)

THIGH_ACTUATOR = ElectricActuator(
  reflected_inertia=reflected_inertia(ROTOR_INERTIA, HIP_GEAR_RATIO),
  velocity_limit=rpm_to_rad(480),
  effort_limit=36.0,
)

CALF_ACTUATOR = ElectricActuator(
  reflected_inertia=reflected_inertia(ROTOR_INERTIA, CALF_GEAR_RATIO),
  velocity_limit=rpm_to_rad(195),
  effort_limit=60.0,
)

WHEEL_ACTUATOR = ElectricActuator(
  reflected_inertia=reflected_inertia(ROTOR_INERTIA, 1),
  velocity_limit=rpm_to_rad(410),
  effort_limit=17.0,
)

WHEEL_DOG_HIP_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_hip_joint",),
  stiffness=40.0,
  damping=1.0,
  effort_limit=HIP_ACTUATOR.effort_limit,
  armature=HIP_ACTUATOR.reflected_inertia,
)

WHEEL_DOG_THIGH_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_thigh_joint",),
  stiffness=40.0,
  damping=1.0,
  effort_limit=THIGH_ACTUATOR.effort_limit,
  armature=THIGH_ACTUATOR.reflected_inertia,
)

WHEEL_DOG_CALF_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_calf_joint",),
  stiffness=40.0,
  damping=1.0,
  effort_limit=CALF_ACTUATOR.effort_limit,
  armature=CALF_ACTUATOR.reflected_inertia,
)

WHEEL_DOG_WHEEL_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_wheel_joint",),
  stiffness=40.0,
  damping=1.0,
  effort_limit=WHEEL_ACTUATOR.effort_limit,
  armature=WHEEL_ACTUATOR.reflected_inertia,
)
##
# Keyframes.
##

# TODO(me): 确认下，这应该是站立的位置
INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.278),
  joint_pos={
    ".*thigh": 0.8,
    ".*calf": -1.5,
    ".*R_hip": 0.1,
    ".*L_hip": -0.1,
  },
  joint_vel={".*": 0.0},
)


##
# Collision config.
##
# TODO(me): 碰撞不知如何设置，先设置成和 GO1 一样的配置，后续再调整。
# This disables all collisions except the feet.
# Furthermore, feet self collisions are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(".*_wheel_collision",),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
  solimp=(0.9, 0.95, 0.023),
)

# # This enables all collisions.
# # Foot collisions are given custom condim, friction.
# FULL_COLLISION = CollisionCfg(
#   geom_names_expr=(".*_collision",),
#   # Harden all collision geoms.
#   solref=(0.01, 1),
#   # Configure feet colliders. Other colliders are frictionless (condim=1).
#   condim={_foot_regex: 6, ".*_collision": 1},
#   priority={_foot_regex: 1},
#   friction={_foot_regex: (1, 5e-3, 5e-4)},
# )

##
# Final config.
##

WHEEL_DOG_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    WHEEL_DOG_HIP_ACTUATOR_CFG,
    WHEEL_DOG_THIGH_ACTUATOR_CFG,
    WHEEL_DOG_CALF_ACTUATOR_CFG,
    WHEEL_DOG_WHEEL_ACTUATOR_CFG,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_wheel_dog_robot_cfg() -> EntityCfg:
  """Get a fresh WheelDog robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=INIT_STATE,
    collisions=(FEET_ONLY_COLLISION,),
    spec_fn=get_spec,
    articulation=WHEEL_DOG_ARTICULATION,
  )


WHEEL_DOG_ACTION_SCALE: dict[str, float] = {}
for a in WHEEL_DOG_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    WHEEL_DOG_ACTION_SCALE[n] = 0.25 * e / s
    # WHEEL_DOG_ACTION_SCALE[n] = 1


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_wheel_dog_robot_cfg())
    
  viewer.launch(robot.spec.compile())
