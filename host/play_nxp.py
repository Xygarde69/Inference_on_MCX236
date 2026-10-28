"""OnePolicy's play.py with the policy running on the FRDM-MCXN236.

The PC simulates the robot; the board runs the actor and steers it: tilting the
board sets the forward / sideways velocity command. OnePolicy is used as is
(its Training/play.py is imported, not copied); set ONEPOLICY_DIR if it isn't
at ../OnePolicy next to this project.

  PY=~/Documents/Projects/OnePolicy/.venv/bin/python
  $PY host/play_nxp.py --robot anymal_c_xknee

Every other argument goes to play.py (see `$PY host/play_nxp.py --help`). Turning
still comes from play.py's own command (--command-port, --joystick or
--command): its yaw is kept and only forward / sideways come from the tilt.
Hold the board in its neutral pose when this starts; that pose is "stand still".
--command-source pc ignores the tilt and runs only the policy on the board.
"""

import argparse
import sys

from nxp_policy import DEFAULT_BAUD, DEFAULT_CHECKPOINT, DEFAULT_PORT, ONEPOLICY_DIR, NxpPolicy

parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)  # play.py has --command
parser.add_argument("--port", default=DEFAULT_PORT, help="Board serial port.")
parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
parser.add_argument("--command-source", choices=("tilt", "pc"), default="tilt",
                    help="Forward / sideways command from the board's tilt, or from play.py.")
parser.add_argument("--no-zero", action="store_true",
                    help="Keep the board's neutral pose instead of zeroing at start.")
args, play_args = parser.parse_known_args()

sys.path.insert(0, str(ONEPOLICY_DIR / "Training"))
import play  # noqa: E402

Runner = play.VelocityOnPolicyRunner
_load = Runner.load
_get_inference_policy = Runner.get_inference_policy


def load(self, path, *a, **kw):
  self.nxp_checkpoint = path
  return _load(self, path, *a, **kw)


def get_inference_policy(self, device=None):
  nxp = NxpPolicy(args.port, args.baud, self.nxp_checkpoint,
                  reference=_get_inference_policy(self, device=device))
  if args.command_source == "tilt":
    if not nxp.board.accel_ok:
      raise RuntimeError("The board found no accelerometer; use --command-source pc.")
    if not args.no_zero:
      nxp.board.zero()
      print("[INFO]: Board zeroed: its current pose means stand still", flush=True)
    term = self.env.unwrapped.command_manager.get_term("twist")
    other = term.gamepad  # play.py's own command, if any: it keeps the yaw

    def command() -> tuple[float, float, float]:
      yaw = other()[2] if other is not None else 0.0
      return (*nxp.command, yaw)

    term.gamepad = command
    print("[INFO]: Forward / sideways command from the board's tilt", flush=True)
  return nxp


Runner.load = load
Runner.get_inference_policy = get_inference_policy

if "--help" in play_args or "-h" in play_args:
  parser.print_help()
  print("\nplay.py options:")
elif not any(a == "--policy" or a.startswith("--policy=") for a in play_args):
  play_args += ["--policy", str(DEFAULT_CHECKPOINT)]
sys.argv = [sys.argv[0]] + play_args
play.main()
