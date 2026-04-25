from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  wheel_dog_flat_env_cfg,
  wheel_dog_rough_env_cfg,
)
from .rl_cfg import wheel_dog_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-Wheel-Dog",
  env_cfg=wheel_dog_rough_env_cfg(),
  play_env_cfg=wheel_dog_rough_env_cfg(play=True),
  rl_cfg=wheel_dog_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Wheel-Dog",
  env_cfg=wheel_dog_flat_env_cfg(),
  play_env_cfg=wheel_dog_flat_env_cfg(play=True),
  rl_cfg=wheel_dog_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
