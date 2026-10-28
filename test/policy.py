"""The policy on the PC (ONNX Runtime), for running without the board and for
checking the board's actions against."""

from pathlib import Path

import numpy as np
import onnxruntime

DEFAULT_ONNX = Path(__file__).resolve().parents[1] / "firmware" / "policy" / "OnePolicy.onnx"


class OnnxPolicy:
  def __init__(self, path: Path = DEFAULT_ONNX):
    self.session = onnxruntime.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    self.input = self.session.get_inputs()[0].name

  def __call__(self, obs: np.ndarray) -> np.ndarray:
    return self.session.run(None, {self.input: obs[None].astype(np.float32)})[0][0]
