"""
EMG Module Abstract Interface

Unified init/read/close pattern for FPGA migration readiness.
All EMG implementations should inherit from this base class.
"""

from abc import ABC, abstractmethod
import numpy as np


class EMGInterface(ABC):
    """Abstract base class for EMG sensor modules."""

    @abstractmethod
    def init(self) -> bool:
        """Initialize the EMG sensor.
        Returns True on success, False on failure.
        """
        ...

    @abstractmethod
    def read(self) -> np.ndarray:
        """Read latest EMG data.
        Returns np.ndarray of shape (n_channels,) with raw ADC values.
        Returns zeros if no data available yet.
        """
        ...

    @abstractmethod
    def close(self):
        """Release resources and disconnect."""
        ...
