"""
EMG MVC Normalizer Module

Wraps a filtered EMG source with per-channel MVC normalization to produce
muscle excitation values in [0, 1] — the input distribution expected by
RL policies trained against SCONE muscle_excitation.

Pipeline:
    BLEEMGModule (raw ADC) -> FilteredEMGModule (envelope) -> NormalizedEMGModule ([0,1])

MVC values are loaded from a JSON file produced by calibrate_mvc.py.
"""

import json
import os

import numpy as np


class NormalizedEMGModule:
    """Wraps a filtered EMG source with MVC normalization.

    For each channel, output = clip(envelope / mvc, 0, 1).

    Args:
        source:    Filtered EMG source (must expose init / read / close).
        mvc_path:  Path to JSON file containing per-channel MVC values.
                   Top-level keys must include ``CHANNEL_NAMES``.
        clip:      Clip output to [0, 1] (default True).
    """

    CHANNEL_NAMES = ("ABE_T2", "ABE_T4", "ABB_T2", "ABB_T4")

    def __init__(self, source, mvc_path, clip=True):
        self._source = source
        self._clip = clip
        self._mvc = self._load_mvc(mvc_path)

    @classmethod
    def _load_mvc(cls, path):
        path = os.path.expanduser(str(path))
        with open(path, "r") as f:
            data = json.load(f)
        try:
            mvc = np.array(
                [float(data[name]) for name in cls.CHANNEL_NAMES],
                dtype=np.float64,
            )
        except KeyError as e:
            raise ValueError(
                f"MVC file {path} missing channel {e}. "
                f"Required: {cls.CHANNEL_NAMES}"
            ) from e
        if np.any(mvc <= 0):
            raise ValueError(
                f"MVC values must be > 0, got "
                f"{dict(zip(cls.CHANNEL_NAMES, mvc.tolist()))}"
            )
        return mvc

    def init(self) -> bool:
        return self._source.init()

    def read(self) -> np.ndarray:
        envelope = self._source.read()
        excitation = envelope / self._mvc
        if self._clip:
            excitation = np.clip(excitation, 0.0, 1.0)
        return excitation

    def read_envelope(self) -> np.ndarray:
        """Pre-normalization envelope, for diagnostics / replotting."""
        return self._source.read()

    def reset_filter(self):
        if hasattr(self._source, "reset_filter"):
            self._source.reset_filter()

    def get_stats(self):
        if hasattr(self._source, "get_stats"):
            return self._source.get_stats()
        return {}

    def close(self):
        self._source.close()

    @property
    def mvc(self) -> np.ndarray:
        return self._mvc.copy()
