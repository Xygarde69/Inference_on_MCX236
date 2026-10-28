"""PC side of the policy firmware (../policy): model export and the serial link.

The board holds OnePolicy's actor (observation normalizer + MLP) in flash and
reads its accelerometer. Every control step the PC sends the raw actor
observations; the board runs the MLP and replies with the actions and the
velocity command it reads from its tilt. play_nxp.py feeds that command into
the next observation, so tilting the board steers the robot.

Run with OnePolicy's interpreter (it has torch for reading checkpoints):

  PY=~/Documents/Projects/OnePolicy/.venv/bin/python
  $PY host/nxp_policy.py export [checkpoint]   # -> policy/model/policy_model.[ch]
  $PY host/nxp_policy.py tilt                  # live tilt command, to check the axes
  $PY host/nxp_policy.py bench [checkpoint]    # board vs float64 NumPy, and timing

Serial protocol (little-endian, every frame ends with the XOR of its cmd and
payload bytes):

  PC -> board  0xA5 cmd payload xor     'I', 'C', 'Z': no payload
                                        'P': n_obs float32 observations
  board -> PC  0x5A cmd payload xor     'I': u32 model CRC, u16 n_obs, u16 n_act, u32 core Hz, u8 accel ok
                                        'P': n_act f32 actions, f32 forward, f32 left, u32 inference cycles
                                        'C': f32 forward, f32 left (Go2 m/s)
                                        'Z': u8 ok (current pose is now the no-command pose)
                                        'E': u8 error code (bad checksum / command / UART)
"""

import argparse
import os
import select
import struct
import termios
import time
import tty
import zlib
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MODEL_DIR = HERE.parent / "policy" / "model"
ONEPOLICY_DIR = Path(os.environ.get("ONEPOLICY_DIR", HERE.parents[1] / "OnePolicy"))
DEFAULT_CHECKPOINT = ONEPOLICY_DIR / "OnePolicy" / "OnePolicy.pt"
# The MCU-Link VCOM by its stable name; its /dev/ttyACM<n> can change when the board re-enumerates
DEFAULT_PORT = next((str(p) for p in Path("/dev/serial/by-id").glob("*MCU-LINK_FRDM-MCXN236*-if02")),
                    "/dev/ttyACM0")
DEFAULT_BAUD = 921600  # the firmware's BOARD_DEBUG_UART_BAUDRATE

SYNC_REQUEST = 0xA5
SYNC_REPLY = 0x5A
# Flash left for the weights: 1 MB minus the vector table and ~60 kB of code.
FLASH_BUDGET = 0xF0000


class ActorNet:
  """The actor of an rsl_rl checkpoint as plain float32 arrays."""

  def __init__(self, checkpoint: str | Path):
    import torch
    import yaml

    checkpoint = Path(checkpoint)
    state = torch.load(checkpoint, map_location="cpu", weights_only=False)["actor_state_dict"]
    agent_yaml = checkpoint.parent / "params" / "agent.yaml"
    if agent_yaml.is_file():
      actor = yaml.unsafe_load(agent_yaml.read_text())["actor"]
      if actor["activation"] != "elu":
        raise ValueError(f"The firmware implements ELU, the actor uses {actor['activation']}.")
    eps = 1e-2  # rsl_rl EmpiricalNormalization: (x - mean) / (std + eps)
    self.mean = state["obs_normalizer._mean"][0].numpy().astype(np.float32)
    self.denom = (state["obs_normalizer._std"][0] + eps).numpy().astype(np.float32)
    self.layers: list[tuple[np.ndarray, np.ndarray]] = []
    i = 0
    while f"mlp.{i}.weight" in state:
      self.layers.append((state[f"mlp.{i}.weight"].numpy().astype(np.float32),
                          state[f"mlp.{i}.bias"].numpy().astype(np.float32)))
      i += 2  # mlp.{odd} are the activations
    self.n_obs = self.layers[0][0].shape[1]
    self.n_act = self.layers[-1][0].shape[0]

  def arrays(self) -> list[np.ndarray]:
    return [self.mean, self.denom] + [a for layer in self.layers for a in layer]

  def crc(self) -> int:
    """CRC32 of every array in firmware order; the board reports it in 'I'."""
    crc = 0
    for a in self.arrays():
      crc = zlib.crc32(np.ascontiguousarray(a).tobytes(), crc)
    return crc

  def nbytes(self) -> int:
    return sum(a.nbytes for a in self.arrays())

  def forward64(self, obs: np.ndarray) -> np.ndarray:
    """Reference forward pass in float64."""
    h = (obs.astype(np.float64) - self.mean) / self.denom
    for i, (w, b) in enumerate(self.layers):
      h = w.astype(np.float64) @ h + b
      if i < len(self.layers) - 1:
        h = np.where(h > 0, h, np.expm1(h))
    return h


def _c_array(name: str, a: np.ndarray) -> str:
  values = [float(v).hex() + "f" for v in a.ravel()]  # hex floats are exact
  lines = [", ".join(values[i:i + 8]) for i in range(0, len(values), 8)]
  return f"const float {name}[{len(values)}] = {{\n    " + ",\n    ".join(lines) + "\n};\n"


def export(checkpoint: Path, out_dir: Path) -> None:
  net = ActorNet(checkpoint)
  if net.nbytes() > FLASH_BUDGET:
    raise ValueError(f"Actor is {net.nbytes()} bytes, the MCXN236 has room for "
                     f"{FLASH_BUDGET} in flash.")
  out_dir.mkdir(parents=True, exist_ok=True)
  dims = [net.n_obs] + [w.shape[0] for w, _ in net.layers]
  header = (
    "/* Generated by host/nxp_policy.py export; do not edit. */\n"
    f"/* Source: {checkpoint.resolve()} */\n"
    "#ifndef POLICY_MODEL_H_\n#define POLICY_MODEL_H_\n\n"
    '#include "policy_net.h"\n\n'
    f"#define POLICY_MODEL_CRC  0x{net.crc():08X}U\n"
    f"#define POLICY_N_OBS      {net.n_obs}U\n"
    f"#define POLICY_N_ACT      {net.n_act}U\n"
    f"#define POLICY_N_LAYERS   {len(net.layers)}U\n"
    f"#define POLICY_MAX_WIDTH  {max(dims)}U\n\n"
    "extern const float POLICY_OBS_MEAN[POLICY_N_OBS];\n"
    "extern const float POLICY_OBS_DENOM[POLICY_N_OBS];\n"
    "extern const policy_layer_t POLICY_LAYERS[POLICY_N_LAYERS];\n\n"
    "#endif /* POLICY_MODEL_H_ */\n"
  )
  body = [
    "/* Generated by host/nxp_policy.py export; do not edit. */\n",
    '#include "policy_model.h"\n\n',
    "/* Observation normalizer: x_norm = (x - mean) / denom, denom = std + 0.01 */\n",
    _c_array("POLICY_OBS_MEAN", net.mean),
    _c_array("POLICY_OBS_DENOM", net.denom),
  ]
  for i, (w, b) in enumerate(net.layers):
    body.append(f"\n/* Layer {i}: {w.shape[1]} -> {w.shape[0]}, weights [out][in] */\n")
    body.append(_c_array(f"L{i}_W", w))
    body.append(_c_array(f"L{i}_B", b))
  body.append("\nconst policy_layer_t POLICY_LAYERS[POLICY_N_LAYERS] = {\n")
  for i, (w, _) in enumerate(net.layers):
    body.append(f"    {{L{i}_W, L{i}_B, {w.shape[1]}U, {w.shape[0]}U}},\n")
  body.append("};\n")
  (out_dir / "policy_model.h").write_text(header)
  (out_dir / "policy_model.c").write_text("".join(body))
  print(f"Exported {checkpoint} -> {out_dir}")
  print(f"  {' -> '.join(map(str, dims))}, {net.nbytes() / 1024:.0f} kB, CRC 0x{net.crc():08X}")


class SerialPort:
  """Raw 8N1 tty (the MCU-Link VCOM), without pyserial."""

  def __init__(self, path: str, baud: int, timeout: float = 1.0):
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
      raise ValueError(f"Unsupported baud rate {baud}.")
    self.timeout = timeout
    self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(self.fd)
    attrs = termios.tcgetattr(self.fd)
    attrs[2] = (attrs[2] | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
    attrs[4] = attrs[5] = speed
    termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

  def write(self, data: bytes) -> None:
    view = memoryview(data)
    while view:
      view = view[os.write(self.fd, view):]

  def read(self, n: int) -> bytes:
    out = bytearray()
    deadline = time.monotonic() + self.timeout
    while len(out) < n:
      left = deadline - time.monotonic()
      if left <= 0 or not select.select([self.fd], [], [], left)[0]:
        break
      out += os.read(self.fd, n - len(out))
    return bytes(out)

  def flush_input(self) -> None:
    termios.tcflush(self.fd, termios.TCIFLUSH)


class Board:
  """The policy firmware on the other end of the serial port."""

  def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
    self.port = SerialPort(port, baud)
    time.sleep(0.1)
    self.port.flush_input()  # boot banner
    self.crc, self.n_obs, self.n_act, self.core_hz, accel = struct.unpack(
      "<IHHIB", self.request("I"))
    self.accel_ok = bool(accel)

  def request(self, cmd: str, payload: bytes = b"") -> bytes:
    code = ord(cmd)
    check = code
    for byte in payload:
      check ^= byte
    self.port.write(bytes([SYNC_REQUEST, code]) + payload + bytes([check]))
    head = self.port.read(2)
    if len(head) != 2 or head[0] != SYNC_REPLY:
      raise RuntimeError(f"No reply from the board (got {head!r}). Is the policy firmware "
                         "flashed, and no terminal open on the port?")
    sizes = {"I": 13, "C": 8, "Z": 1, "E": 1, "P": 4 * getattr(self, "n_act", 0) + 12}
    size = sizes.get(chr(head[1]))
    if size is None:
      raise RuntimeError(f"Unknown reply {head!r} from the board.")
    body = self.port.read(size + 1)
    check = head[1]
    for byte in body[:-1]:
      check ^= byte
    if len(body) != size + 1 or check != body[-1]:
      raise RuntimeError("Corrupt reply from the board.")
    if head[1] == ord("E"):
      raise RuntimeError(f"Board rejected the request (error {body[0]}).")
    return body[:-1]

  def infer(self, obs: np.ndarray) -> tuple[np.ndarray, tuple[float, float], float]:
    """(actions, tilt command (forward, left), inference seconds) for one observation."""
    reply = self.request("P", np.ascontiguousarray(obs, dtype="<f4").tobytes())
    actions = np.frombuffer(reply[:-12], dtype="<f4")
    forward, left, cycles = struct.unpack("<ffI", reply[-12:])
    return actions, (forward, left), cycles / self.core_hz

  def command(self) -> tuple[float, float]:
    return struct.unpack("<ff", self.request("C"))

  def zero(self) -> bool:
    return bool(self.request("Z")[0])


class NxpPolicy:
  """Policy callable for OnePolicy's play.py that runs the actor on the board.

  `command` is the board's latest tilt command (forward, left) in Go2 m/s.
  `reference` (the PyTorch policy) is evaluated on the same observations so the
  log can show how far the board's actions are from the PC's."""

  def __init__(self, port: str, baud: int, checkpoint: str | Path, reference=None,
               log_every: int = 250):
    self.net = ActorNet(checkpoint)
    self.board = Board(port, baud)
    b = self.board
    if (b.n_obs, b.n_act) != (self.net.n_obs, self.net.n_act):
      raise RuntimeError(f"Board runs a {b.n_obs} -> {b.n_act} policy, the checkpoint is "
                         f"{self.net.n_obs} -> {self.net.n_act}.")
    if b.crc != self.net.crc():
      raise RuntimeError(f"Board model CRC 0x{b.crc:08X} != checkpoint 0x{self.net.crc():08X}: "
                         "export it with nxp_policy.py export and reflash.")
    self.reference = reference
    self.log_every = log_every
    self.command = (0.0, 0.0)
    print(f"[INFO]: NXP policy on {port} @ {baud} baud: model 0x{b.crc:08X}, "
          f"{b.n_obs} -> {b.n_act}, core {b.core_hz / 1e6:.0f} MHz, "
          f"accelerometer {'ok' if b.accel_ok else 'NOT FOUND'}", flush=True)
    self._reset_stats()

  def _reset_stats(self):
    self.calls = 0
    self.inferences = 0
    self.board_s = 0.0
    self.round_trip_s = 0.0
    self.max_diff = 0.0

  def __call__(self, obs):
    import torch

    x = obs["actor"]
    rows = []
    start = time.perf_counter()
    for row in x.detach().cpu().numpy():
      actions, self.command, seconds = self.board.infer(row)
      rows.append(actions)
      self.board_s += seconds
      self.inferences += 1
    self.round_trip_s += time.perf_counter() - start
    actions = torch.from_numpy(np.stack(rows)).to(x.device)
    if self.reference is not None:
      with torch.no_grad():
        diff = (actions - self.reference(obs)).abs().max().item()
      self.max_diff = max(self.max_diff, diff)
    self.calls += 1
    if self.calls == self.log_every:
      ref = f", max |action - PyTorch| {self.max_diff:.1e}" if self.reference is not None else ""
      print(f"[NXP]: last {self.calls} steps: board {1e3 * self.board_s / self.inferences:.2f} ms per "
            f"inference, round trip {1e3 * self.round_trip_s / self.calls:.2f} ms per step{ref}, "
            f"tilt command forward {self.command[0]:+.2f} left {self.command[1]:+.2f} m/s",
            flush=True)
      self._reset_stats()
    return actions


def tilt(port: str, baud: int) -> None:
  board = Board(port, baud)
  if not board.accel_ok:
    raise RuntimeError("The board found no accelerometer.")
  board.zero()
  print("Zeroed at the current pose. Tilt the board (Ctrl+C to stop):")
  try:
    while True:
      forward, left = board.command()
      print(f"\r  forward {forward:+.2f} m/s   left {left:+.2f} m/s   ", end="", flush=True)
      time.sleep(0.02)
  except KeyboardInterrupt:
    print()


def bench(port: str, baud: int, checkpoint: Path, n: int = 200) -> None:
  net = ActorNet(checkpoint)
  board = Board(port, baud)
  if board.crc != net.crc():
    raise RuntimeError(f"Board model CRC 0x{board.crc:08X} != checkpoint 0x{net.crc():08X}.")
  rng = np.random.default_rng(0)
  diffs, infer_s, rtt = [], [], []
  for _ in range(n):
    x = (net.mean + (net.denom - 0.01) * rng.standard_normal(net.n_obs)).astype(np.float32)
    start = time.perf_counter()
    actions, _, seconds = board.infer(x)
    rtt.append(time.perf_counter() - start)
    infer_s.append(seconds)
    diffs.append(np.abs(actions - net.forward64(x)).max())
  print(f"{n} random observations: max |board - float64| {max(diffs):.1e}, "
        f"inference {1e3 * np.mean(infer_s):.2f} ms, round trip {1e3 * np.mean(rtt):.2f} ms "
        f"(max {1e3 * np.max(rtt):.2f})")


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = parser.add_subparsers(dest="action", required=True)
  p = sub.add_parser("export", help="Write a checkpoint's actor into the firmware.")
  p.add_argument("checkpoint", type=Path, nargs="?", default=DEFAULT_CHECKPOINT)
  p.add_argument("--out", type=Path, default=MODEL_DIR)
  for name, help_ in (("tilt", "Print the board's tilt command live."),
                      ("bench", "Check the board's actions and timing.")):
    p = sub.add_parser(name, help=help_)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    if name == "bench":
      p.add_argument("checkpoint", type=Path, nargs="?", default=DEFAULT_CHECKPOINT)
  args = parser.parse_args()
  if args.action == "export":
    export(args.checkpoint, args.out)
  elif args.action == "tilt":
    tilt(args.port, args.baud)
  else:
    bench(args.port, args.baud, args.checkpoint)


if __name__ == "__main__":
  main()
