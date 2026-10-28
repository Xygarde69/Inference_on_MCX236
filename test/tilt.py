"""Print the board's tilt command live, to check its directions before walking.

  uv run test/tilt.py

Hold the board in its neutral pose when this starts (that pose is "stand still"),
then dip each edge. If forward / left come out reversed or swapped, change
TILT_FORWARD_SIGN / TILT_LEFT_SIGN / TILT_FORWARD_AXIS in firmware/app/Kconfig.
"""

import argparse
import time

from board import DEFAULT_BAUD, DEFAULT_PORT, Board


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--port", default=DEFAULT_PORT)
  p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
  args = p.parse_args()
  board = Board(args.port, args.baud)
  if not board.accel_ok:
    raise RuntimeError("The board found no accelerometer.")
  board.zero()
  print("Zeroed at the current pose. Tilt the board (Ctrl+C to stop):")
  try:
    while True:
      forward, left = board.command()
      print(f"\r  forward {forward:+.2f} m/s   left {left:+.2f} m/s   (Go2 units)", end="", flush=True)
      time.sleep(0.02)
  except KeyboardInterrupt:
    print()


if __name__ == "__main__":
  main()
