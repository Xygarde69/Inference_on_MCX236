"""Regenerate test/assets/anymal_c from OnePolicy (provenance; not needed to run the test).

The test simulates ANYmal C exactly as OnePolicy's play.py does, but in plain
MuJoCo. This builds OnePolicy's play environment for anymal_c_xknee once and
writes what the test needs:

  anymal_c.xml   the scene MuJoCo compiles: the robot rewritten into OnePolicy's
                 convention (Go2 joint names and order, see OnePolicy's
                 generation/adapt.py), the ground plane, physics options and the
                 IMU, with the per-robot contact softness, armature and motor
                 limits that OnePolicy's morphology event writes at startup
  assets/        its meshes and textures
  robot.json     the controller: joint order, default pose, action scale, the DC
                 motor model's gains and limits, the robot's Froude scales

Startup randomisation (friction, COM offset, encoder bias) is left out, so this
is the nominal robot. Needs a OnePolicy checkout and its .venv:

  ~/Documents/Projects/OnePolicy/.venv/bin/python test/tools/extract_anymal_c.py \
      --onepolicy ~/Documents/Projects/OnePolicy
"""

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROBOT = "anymal_c_xknee"
OUT = Path(__file__).resolve().parents[1] / "assets" / "anymal_c"


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--onepolicy", type=Path, required=True, help="OnePolicy checkout.")
  parser.add_argument("--out", type=Path, default=OUT)
  args = parser.parse_args()
  sys.path.insert(0, str(args.onepolicy / "Training"))

  import mujoco
  import numpy as np
  from mjlab.envs import ManagerBasedRlEnv
  from play import PlayConfig, prepare_robot
  from tasks.velocity.config.morph_quadruped import (
    drop_body_contact_if_unmatched,
    morph_quadruped_flat_env_cfg,
    set_morphologies,
  )

  robot_dir = Path(tempfile.mkdtemp(prefix="extract_"))
  try:
    pose, _, _ = prepare_robot(PlayConfig(robot=ROBOT), robot_dir)
    cfg = morph_quadruped_flat_env_cfg(play=True)
    cfg.scene.num_envs = 1
    set_morphologies(cfg, robot_dir, pose)
    drop_body_contact_if_unmatched(cfg, robot_dir / "env_00000.xml")
    for name in ("foot_friction", "encoder_bias", "base_com"):
      cfg.events.pop(name, None)
    env = ManagerBasedRlEnv(cfg=cfg, device="cpu")
  finally:
    shutil.rmtree(robot_dir, ignore_errors=True)

  mj_model = env.sim.mj_model
  world0 = env.sim.model
  robot = env.scene["robot"]
  spec = env.scene.spec

  def value(field: str, index: int) -> np.ndarray:
    return np.asarray(getattr(world0, field)[0, index].cpu(), dtype=float)

  # The fields the morphology event changes per robot (geometry and inertia are
  # the adapted XML's own; MuJoCo derives invweight0 / subtreemass itself).
  for geom in spec.geoms:
    gid = mj_model.geom(geom.name).id
    geom.solref = value("geom_solref", gid)
    geom.solimp = value("geom_solimp", gid)
    geom.solmix = float(value("geom_solmix", gid))
  for joint in spec.joints:
    jid = mj_model.joint(joint.name).id
    if mj_model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
      continue
    joint.armature = float(value("dof_armature", mj_model.jnt_dofadr[jid]))
    joint.range = value("jnt_range", jid)
  for act in spec.actuators:
    aid = mj_model.actuator(act.name).id
    act.ctrlrange = value("actuator_ctrlrange", aid)
    act.forcerange = value("actuator_forcerange", aid)

  # mjlab sets the physics options on the compiled model (MujocoCfg), not in the spec
  for name in dir(spec.option):
    if not name.startswith("_") and hasattr(mj_model.opt, name):
      setattr(spec.option, name, getattr(mj_model.opt, name))

  # Training-only contact sensors; the policy reads the IMU gyro alone.
  for sensor in list(spec.sensors):
    if not sensor.name.startswith("robot/imu"):
      spec.delete(sensor)

  joint_names = list(robot.joint_names)
  default_pos = robot.data.default_joint_pos[0].cpu().numpy()
  root = robot.data.default_root_state[0].cpu().numpy()
  for key in list(spec.keys):  # mjlab's init_state, with a placeholder base height
    spec.delete(key)
  key = spec.add_key()
  key.name = "home"
  key.qpos = np.concatenate([root[:7], [default_pos[joint_names.index(j.name.split("/")[-1])]
                                        for j in spec.joints
                                        if j.type != mujoco.mjtJoint.mjJNT_FREE]])

  args.out.mkdir(parents=True, exist_ok=True)
  asset_dir = args.out / "assets"
  if asset_dir.exists():
    shutil.rmtree(asset_dir)
  for name, data in spec.assets.items():
    path = asset_dir / Path(name).name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
  spec.meshdir = "assets"
  spec.texturedir = "assets"
  for mesh in spec.meshes:
    mesh.file = Path(mesh.file).name
  for texture in spec.textures:
    if texture.file:
      texture.file = Path(texture.file).name
  spec.assets = {}
  xml_path = args.out / "anymal_c.xml"
  # to_xml writes the root default class as an empty <default/>, which the parser rejects
  xml = "\n".join(line for line in spec.to_xml().splitlines() if line.strip() != "<default/>")
  xml_path.write_text(
    "<!-- ANYmal C in OnePolicy's convention, generated by test/tools/extract_anymal_c.py. -->\n"
    + xml + "\n")

  # Check the written scene against the environment it came from.
  model = mujoco.MjModel.from_xml_path(str(xml_path))
  for field in ("geom_solref", "geom_solimp", "geom_solmix", "dof_armature", "jnt_range",
                "actuator_ctrlrange", "actuator_forcerange", "body_mass", "body_inertia",
                "geom_size", "body_pos"):
    ref = getattr(world0, field)[0].cpu().numpy() if field in env.sim.expanded_fields \
      else getattr(mj_model, field)
    # to_xml prints ~6 significant digits
    err = (np.abs(getattr(model, field) - ref) / np.maximum(np.abs(ref), 1.0)).max()
    if err > 1e-5:
      raise RuntimeError(f"{field} differs from OnePolicy's model (relative {err:.1e})")

  for name in ("timestep", "iterations", "ls_iterations", "integrator", "cone", "solver",
               "ccd_iterations", "tolerance", "ls_tolerance", "impratio", "disableflags", "enableflags"):
    if np.any(np.asarray(getattr(model.opt, name)) != np.asarray(getattr(mj_model.opt, name))):
      raise RuntimeError(f"option {name} differs from OnePolicy's model")

  actuators = []
  for act in robot.actuators:
    for k, (name, ctrl) in enumerate(zip(act._target_names, act.global_ctrl_ids.tolist())):
      actuators.append({
        "joint": name,
        "ctrl": ctrl,
        "kp": float(act.stiffness[0, k]),
        "kd": float(act.damping[0, k]),
        "effort_limit": float(act.force_limit[0, k]),
        "saturation_effort": float(act.saturation_effort[0, k]),
        "velocity_limit": float(act.velocity_limit_motor[0, k]),
      })
  actuators.sort(key=lambda a: joint_names.index(a["joint"]))
  term = env.action_manager._terms["joint_pos"]
  scales = env.morph_scales
  ranges = env_cfg_ranges(cfg)
  robot_json = {
    "source": f"OnePolicy play.py --robot {ROBOT} (flat task, play mode, nominal: no startup randomisation)",
    "joint_names": joint_names,
    "default_joint_pos": default_pos.tolist(),
    "action_scale": float(np.asarray(term._scale.cpu() if hasattr(term._scale, "cpu")
                                     else term._scale).ravel()[0]),
    "actuators": actuators,
    "length_scale": float(scales.length[0]),
    "mass_scale": float(scales.mass[0]),
    "phase_period_go2": 0.6,
    "command_ranges_go2": ranges,
    "timestep": float(mj_model.opt.timestep),
    "decimation": int(cfg.decimation),
    "base_body": "robot/base_link",
    "gyro_sensor": "robot/imu_ang_vel",
  }
  (args.out / "robot.json").write_text(json.dumps(robot_json, indent=2) + "\n")
  license_src = args.onepolicy / "quadrupeds" / "anybotics_anymal_c" / "LICENSE"
  shutil.copy(license_src, args.out / "LICENSE")
  print(f"Wrote {xml_path}, {len(spec.meshes)} meshes, robot.json")


def env_cfg_ranges(cfg) -> dict:
  r = cfg.commands["twist"].ranges
  return {"forward": list(r.lin_vel_x), "left": list(r.lin_vel_y), "yaw": list(r.ang_vel_z)}


if __name__ == "__main__":
  main()
