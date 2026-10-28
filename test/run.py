"""Watch ANYmal C walk in real time with OnePolicy running on the FRDM-MCXN236.

  uv run test/run.py                       # policy on the board, tilt the board to walk
  uv run test/run.py --policy pc           # no board: ONNX on the PC, arrow keys to walk
  uv run test/run.py --headless --duration 20 --command 0.5 0 0

Every 20 ms the PC sends ANYmal C's 51 observations to the board and applies the
12 joint actions it returns. The board also returns the velocity command it
reads from its tilt (forward / sideways); the pose at start is "stand still".

Keys in the viewer: Left / Right arrow turn (yaw command steps of 0.25 rad/s,
Go2 units); with --policy pc or --no-tilt, Up / Down arrow change the forward
command by 0.25 m/s. The robot is reset when it falls.
"""

import argparse
import time

import numpy as np

from anymal import AnymalC
from board import DEFAULT_BAUD, DEFAULT_PORT, Board
from policy import DEFAULT_ONNX, OnnxPolicy

KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
KEY_STEP = 0.25
LOG_EVERY_S = 5.0
# The board's actions must match the PC's ONNX: above this, it holds another policy
MAX_BOARD_DIFF = 1e-3


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--policy", choices=("board", "pc"), default="board",
                 help="Where the policy runs (default: the board).")
  p.add_argument("--port", default=DEFAULT_PORT)
  p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
  p.add_argument("--onnx", default=str(DEFAULT_ONNX), help="PC policy / reference for the board.")
  p.add_argument("--no-tilt", action="store_true", help="Board runs the policy only; keys steer.")
  p.add_argument("--command", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                 metavar=("FORWARD", "LEFT", "YAW"),
                 help="Starting command in Go2 units [m/s m/s rad/s] (the tilt overrides forward/left).")
  p.add_argument("--headless", action="store_true", help="No viewer; print the stats.")
  p.add_argument("--duration", type=float, default=None, help="Stop after this many seconds.")
  args = p.parse_args()

  sim = AnymalC()
  onnx = OnnxPolicy(args.onnx)
  board = None
  use_tilt = False
  if args.policy == "board":
    board = Board(args.port, args.baud)
    if (board.n_obs, board.n_act) != (51, 12):
      raise RuntimeError(f"Board runs a {board.n_obs} -> {board.n_act} policy, ANYmal C needs 51 -> 12.")
    use_tilt = not args.no_tilt
    if use_tilt and not board.accel_ok:
      raise RuntimeError("The board found no accelerometer; use --no-tilt.")
    if use_tilt:
      board.zero()
    print(f"Board on {args.port}: model 0x{board.crc:08X}, core {board.core_hz / 1e6:.0f} MHz, "
          f"accelerometer {'ok' if board.accel_ok else 'missing'}"
          + ("; zeroed, tilt it to walk" if use_tilt else ""))
  else:
    print(f"Policy on the PC: {args.onnx}")

  command = list(args.command)

  def on_key(key: int) -> None:
    if key in (KEY_LEFT, KEY_RIGHT):
      command[2] += KEY_STEP if key == KEY_LEFT else -KEY_STEP
    elif key in (KEY_UP, KEY_DOWN) and not use_tilt:
      command[0] += KEY_STEP if key == KEY_UP else -KEY_STEP
    else:
      return
    r = sim.command_ranges
    command[0] = float(np.clip(command[0], *r["forward"]))
    command[2] = float(np.clip(command[2], *r["yaw"]))
    print(f"command forward {command[0]:+.2f} m/s, yaw {command[2]:+.2f} rad/s (Go2 units)")

  stats = {"steps": 0, "board_s": 0.0, "trip_s": 0.0, "diff": 0.0, "vel": np.zeros(3), "late": 0}

  def control_step() -> None:
    sim.set_command(*command)
    obs = sim.observe()
    if board is not None:
      start = time.perf_counter()
      action, tilt, seconds = board.infer(obs)
      stats["trip_s"] += time.perf_counter() - start
      stats["board_s"] += seconds
      diff = float(np.abs(action - onnx(obs)).max())
      if diff > MAX_BOARD_DIFF:
        raise RuntimeError(f"Board actions differ from {args.onnx} by {diff:.1e}: flash the "
                           "firmware built from it (README: Build and flash).")
      stats["diff"] = max(stats["diff"], diff)
      if use_tilt:
        command[0], command[1] = tilt
    else:
      action = onnx(obs)
    sim.step(action)
    stats["steps"] += 1
    stats["vel"] += sim.base_velocity()
    if sim.fallen():
      print("Fell over: reset")
      sim.reset()

  def log() -> None:
    n = max(stats["steps"], 1)
    vel = stats["vel"] / n / sim.speed_scale  # Go2 units, like the command
    line = (f"[{sim.steps * sim.step_dt:6.1f} s] command forward {command[0]:+.2f} left {command[1]:+.2f} "
            f"yaw {command[2]:+.2f} | walked forward {vel[0]:+.2f} left {vel[1]:+.2f} (Go2 m/s)")
    if board is not None:
      line += (f" | board {1e3 * stats['board_s'] / n:.1f} ms, round trip {1e3 * stats['trip_s'] / n:.1f} ms, "
               f"max |board - ONNX| {stats['diff']:.0e}")
    if stats["late"]:
      line += f" | {stats['late']} steps late"
    print(line, flush=True)
    stats.update(steps=0, board_s=0.0, trip_s=0.0, diff=0.0, vel=np.zeros(3), late=0)

  def run(alive):
    """Real-time control loop; yields after every step (for the viewer)."""
    next_step = next_log = time.perf_counter()  # from here, so opening the viewer isn't counted
    end = None if args.duration is None else next_step + args.duration
    while alive() and (end is None or time.perf_counter() < end):
      control_step()
      yield
      next_step += sim.step_dt
      wait = next_step - time.perf_counter()
      if wait > 0:
        time.sleep(wait)
      else:
        stats["late"] += wait < -sim.step_dt  # more than a step behind real time
        next_step = time.perf_counter()
      if time.perf_counter() >= next_log:
        if stats["steps"]:
          log()
        next_log += LOG_EVERY_S

  if args.headless:
    for _ in run(lambda: True):
      pass
  else:
    import mujoco.viewer

    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as viewer:
      viewer.cam.trackbodyid = sim.base
      viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
      viewer.cam.distance = 3.0
      for _ in run(viewer.is_running):
        viewer.sync()
  if stats["steps"]:
    log()


if __name__ == "__main__":
  main()
