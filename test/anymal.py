"""ANYmal C in plain MuJoCo, simulated the way OnePolicy trained its policy.

assets/anymal_c/anymal_c.xml is the scene and robot.json the controller, both
taken from OnePolicy's play environment (tools/extract_anymal_c.py). This file
rebuilds the rest of that environment: the 51 actor observations and the
joint-position action driving DC motors.

Observation (OnePolicy's actor group, in order):
  base angular velocity (IMU gyro, base frame)       3
  projected gravity (base frame)                     3
  velocity command (forward, left, yaw; robot units) 3
  gait phase (sin, cos; zero when standing)          2
  joint positions - default pose                    12
  joint velocities                                  12
  knee direction (sign of the default knee angle)    4
  last action                                       12

Commands are given in Go2 units and scaled to ANYmal C by Froude similarity
(speed x sqrt(s_L), yaw rate / sqrt(s_L)), as OnePolicy does.
"""

import json
from pathlib import Path

import mujoco
import numpy as np

ASSETS = Path(__file__).resolve().parent / "assets" / "anymal_c"
N_OBS = 51
N_ACT = 12


class AnymalC:
  def __init__(self, asset_dir: Path = ASSETS):
    cfg = json.loads((asset_dir / "robot.json").read_text())
    self.model = mujoco.MjModel.from_xml_path(str(asset_dir / "anymal_c.xml"))
    self.data = mujoco.MjData(self.model)
    self.decimation = cfg["decimation"]
    self.step_dt = cfg["timestep"] * self.decimation
    assert abs(self.model.opt.timestep - cfg["timestep"]) < 1e-12

    names = [f"robot/{n}" for n in cfg["joint_names"]]
    self.qpos_adr = np.array([self.model.joint(n).qposadr[0] for n in names])
    self.qvel_adr = np.array([self.model.joint(n).dofadr[0] for n in names])
    self.default_pos = np.array(cfg["default_joint_pos"])
    self.action_scale = cfg["action_scale"]
    acts = cfg["actuators"]
    self.ctrl_ids = np.array([a["ctrl"] for a in acts])
    self.kp = np.array([a["kp"] for a in acts])
    self.kd = np.array([a["kd"] for a in acts])
    self.effort_limit = np.array([a["effort_limit"] for a in acts])
    self.saturation = np.array([a["saturation_effort"] for a in acts])
    self.velocity_limit = np.array([a["velocity_limit"] for a in acts])
    # Joint speed where the torque-speed line crosses the effort limit
    self.corner_velocity = self.velocity_limit * (1 + self.effort_limit / self.saturation)

    self.speed_scale = np.sqrt(cfg["length_scale"])
    self.time_scale = np.sqrt(cfg["length_scale"])
    self.phase_period = cfg["phase_period_go2"] * self.time_scale
    self.command_ranges = cfg["command_ranges_go2"]
    calves = [i for i, n in enumerate(cfg["joint_names"]) if n.endswith("_calf_joint")]
    self.knee_direction = np.sign(self.default_pos[calves])
    self.base = self.model.body(cfg["base_body"]).id
    gyro = self.model.sensor(cfg["gyro_sensor"])
    self.gyro_adr = slice(gyro.adr[0], gyro.adr[0] + 3)
    self.home = self.model.key("home").id
    self.reset()

  def reset(self) -> None:
    mujoco.mj_resetDataKeyframe(self.model, self.data, self.home)
    mujoco.mj_forward(self.model, self.data)
    self.steps = 0
    self.last_action = np.zeros(N_ACT)
    self.command_go2 = np.zeros(3)

  def set_command(self, forward: float, left: float, yaw: float) -> None:
    """Velocity command in Go2 units [m/s, m/s, rad/s], clipped to play's ranges."""
    r = self.command_ranges
    self.command_go2 = np.array([np.clip(forward, *r["forward"]), np.clip(left, *r["left"]),
                                 np.clip(yaw, *r["yaw"])])

  def observe(self) -> np.ndarray:
    d = self.data
    rot = d.xmat[self.base].reshape(3, 3)  # base -> world
    command = np.array([self.command_go2[0] * self.speed_scale,
                        self.command_go2[1] * self.speed_scale,
                        self.command_go2[2] / self.time_scale])
    phase = np.zeros(2)
    if np.linalg.norm(self.command_go2) >= 0.1:
      t = (self.steps * self.step_dt) % self.phase_period / self.phase_period
      phase[:] = np.sin(2 * np.pi * t), np.cos(2 * np.pi * t)
    obs = np.concatenate([
      d.sensordata[self.gyro_adr],
      rot.T @ np.array([0.0, 0.0, -1.0]),
      command,
      phase,
      d.qpos[self.qpos_adr] - self.default_pos,
      d.qvel[self.qvel_adr],
      self.knee_direction,
      self.last_action,
    ])
    assert obs.shape == (N_OBS,)
    return obs.astype(np.float32)

  def _motor_torque(self, target: np.ndarray) -> np.ndarray:
    """PD towards `target`, limited by the DC motor's torque-speed line."""
    q = self.data.qpos[self.qpos_adr]
    qd = self.data.qvel[self.qvel_adr]
    torque = self.kp * (target - q) - self.kd * qd
    v = np.clip(qd, -self.corner_velocity, self.corner_velocity)
    top = np.minimum(self.saturation * (1.0 - v / self.velocity_limit), self.effort_limit)
    bottom = np.maximum(self.saturation * (-1.0 - v / self.velocity_limit), -self.effort_limit)
    return np.clip(torque, bottom, top)

  def step(self, action: np.ndarray) -> None:
    """Apply one policy action (20 ms: `decimation` physics steps)."""
    action = np.asarray(action, dtype=float)
    target = self.default_pos + self.action_scale * action
    for _ in range(self.decimation):
      self.data.ctrl[self.ctrl_ids] = self._motor_torque(target)
      mujoco.mj_step(self.model, self.data)
    # mj_step leaves kinematics and sensors at the last substep's start; the
    # observation needs them at the current state, as mjlab senses after stepping
    mujoco.mj_forward(self.model, self.data)
    self.last_action = action
    self.steps += 1

  def fallen(self) -> bool:
    """Base tilted past ~70 deg (OnePolicy's bad_orientation) or below 15 cm."""
    up_z = self.data.xmat[self.base][8]
    return up_z < np.cos(np.radians(70)) or self.data.xpos[self.base][2] < 0.15

  def base_velocity(self) -> np.ndarray:
    """Base linear velocity in its own frame [forward, left, up] (m/s)."""
    rot = self.data.xmat[self.base].reshape(3, 3)
    return rot.T @ self.data.qvel[:3]
