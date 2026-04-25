"""Velocity tracking environments for legged robots."""
from mjlab.tasks.registry import register_mjlab_task
from mjlab.rl.runner import MjlabOnPolicyRunner

from .config.wheel_dog.env_cfgs import wheel_dog_flat_env_cfg
from .config.wheel_dog.rl_cfg import wheel_dog_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-WheelDog",
  env_cfg=wheel_dog_flat_env_cfg(),
  play_env_cfg=wheel_dog_flat_env_cfg(play=True),
  rl_cfg=wheel_dog_ppo_runner_cfg(),
  runner_cls=MjlabOnPolicyRunner,
)