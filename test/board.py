"""Serial link to the policy firmware (../firmware/app) over the MCU-Link VCOM.

Protocol (little-endian, every frame ends with the XOR of its cmd and payload bytes):

  PC -> board  0xA5 cmd payload xor     'I', 'C', 'Z': no payload
                                        'P': 51 float32 observations
  board -> PC  0x5A cmd payload xor     'I': u32 model CRC, u16 n_obs, u16 n_act, u32 core Hz, u8 accel ok
                                        'P': n_act f32 actions, f32 forward, f32 left, u32 inference cycles
                                        'C': f32 forward, f32 left (tilt command, Go2 m/s)
                                        'Z': u8 ok (current pose is now the no-command pose)
                                        'E': u8 error code (1 checksum, 2 command, 3 UART)
"""

import os
import select
import struct
import termios
import time
import tty
from pathlib import Path

import numpy as np

# The MCU-Link VCOM by its stable name; its /dev/ttyACM<n> can change when the board re-enumerates
DEFAULT_PORT = next((str(p) for p in Path("/dev/serial/by-id").glob("*MCU-LINK_FRDM-MCXN236*-if02")),
                    "/dev/ttyACM0")
DEFAULT_BAUD = 921600  # the firmware's BOARD_DEBUG_UART_BAUDRATE

SYNC_REQUEST = 0xA5
SYNC_REPLY = 0x5A


class SerialPort:
  """Raw 8N1 tty, without pyserial."""

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
    self.n_act = 0
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
    size = {"I": 13, "C": 8, "Z": 1, "E": 1, "P": 4 * self.n_act + 12}.get(chr(head[1]))
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
    """(actions, tilt command (forward, left) in Go2 m/s, inference seconds)."""
    reply = self.request("P", np.ascontiguousarray(obs, dtype="<f4").tobytes())
    actions = np.frombuffer(reply[:-12], dtype="<f4")
    forward, left, cycles = struct.unpack("<ffI", reply[-12:])
    return actions, (forward, left), cycles / self.core_hz

  def command(self) -> tuple[float, float]:
    return struct.unpack("<ff", self.request("C"))

  def zero(self) -> bool:
    return bool(self.request("Z")[0])
