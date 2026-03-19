"""
EMG Signal Processing Filter Module

Real-time three-stage EMG processing pipeline:
    1. High-pass filter (30 Hz, 2nd-order Butterworth) — DC/motion artifact removal
    2. Full-wave rectification — abs()
    3. Low-pass filter (6 Hz, 4th-order Butterworth) — Envelope extraction

Uses SOS (second-order sections) form for numerical stability
and maintains per-channel filter state for streaming operation.
"""

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi


class EMGFilterPipeline:
    """Real-time EMG signal processing pipeline.

    Maintains internal filter state (zi) for each channel so that
    consecutive calls to process() / process_batch() produce a
    continuous filtered output stream.

    Args:
        n_channels: Number of EMG channels (default 4).
        fs:         Sampling frequency in Hz (default 500).
        hp_cutoff:  High-pass cutoff in Hz (default 30).
        hp_order:   High-pass filter order (default 2).
        lp_cutoff:  Low-pass cutoff in Hz (default 6).
        lp_order:   Low-pass filter order (default 4).
    """

    def __init__(self, n_channels: int = 4, fs: float = 500.0,
                 hp_cutoff: float = 30.0, hp_order: int = 2,
                 lp_cutoff: float = 6.0, lp_order: int = 4):
        self.n_channels = n_channels
        self.fs = fs

        # Stage 1: High-pass Butterworth (SOS)
        self._hp_sos = butter(hp_order, hp_cutoff, btype='highpass',
                              fs=fs, output='sos')
        # Stage 3: Low-pass Butterworth (SOS)
        self._lp_sos = butter(lp_order, lp_cutoff, btype='lowpass',
                              fs=fs, output='sos')

        # Per-channel filter state vectors
        self._hp_zi = self._make_zi(self._hp_sos)
        self._lp_zi = self._make_zi(self._lp_sos)

    def _make_zi(self, sos):
        """Create zero-initialized filter state for each channel."""
        zi_template = sosfilt_zi(sos)  # shape (n_sections, 2)
        return [zi_template * 0.0 for _ in range(self.n_channels)]

    def process(self, raw: np.ndarray) -> np.ndarray:
        """Process a single sample vector.

        Args:
            raw: shape (n_channels,) raw ADC values.
        Returns:
            shape (n_channels,) envelope values.
        """
        return self.process_batch(raw.reshape(-1, 1)).ravel()

    def process_batch(self, raw_batch: np.ndarray) -> np.ndarray:
        """Process a batch of samples.

        Args:
            raw_batch: shape (n_channels, n_samples) raw ADC values.
        Returns:
            shape (n_channels, n_samples) envelope values.
        """
        n_ch = raw_batch.shape[0]
        result = np.empty_like(raw_batch, dtype=np.float64)

        for ch in range(n_ch):
            x = raw_batch[ch].astype(np.float64)

            # Stage 1: High-pass filter
            hp_out, self._hp_zi[ch] = sosfilt(self._hp_sos, x,
                                              zi=self._hp_zi[ch])
            # Stage 2: Full-wave rectification
            rect_out = np.abs(hp_out)

            # Stage 3: Low-pass envelope filter
            env_out, self._lp_zi[ch] = sosfilt(self._lp_sos, rect_out,
                                               zi=self._lp_zi[ch])
            result[ch] = env_out

        return result

    def reset(self):
        """Reset all filter states to zero."""
        self._hp_zi = self._make_zi(self._hp_sos)
        self._lp_zi = self._make_zi(self._lp_sos)


class FilteredEMGModule:
    """Wraps a raw EMG module with real-time signal processing.

    Provides the same read()/read_batch() interface but returns
    filtered envelope data instead of raw ADC values.

    Usage:
        from ble_emg_module import BLEEMGModule

        raw_emg = BLEEMGModule()
        emg = FilteredEMGModule(raw_emg, fs=500.0)
        emg.init()
        envelope = emg.read()       # filtered (n_channels,)
        raw = emg.read_raw()        # bypass filter
        emg.close()
    """

    def __init__(self, source, fs: float = 500.0, **filter_kwargs):
        """
        Args:
            source:        An EMG module with init/read/read_batch/close.
            fs:            Sampling frequency in Hz.
            **filter_kwargs: Passed to EMGFilterPipeline (hp_cutoff, lp_cutoff, etc.)
        """
        self._source = source
        self._pipeline = EMGFilterPipeline(n_channels=4, fs=fs, **filter_kwargs)

    def init(self) -> bool:
        return self._source.init()

    def read(self) -> np.ndarray:
        """Read one sample, return filtered envelope."""
        raw = self._source.read()
        return self._pipeline.process(raw)

    def read_batch(self, n_samples: int = 7) -> np.ndarray:
        """Read N samples, return filtered envelope."""
        raw = self._source.read_batch(n_samples)
        return self._pipeline.process_batch(raw)

    def read_raw(self) -> np.ndarray:
        """Read one sample bypassing the filter pipeline."""
        return self._source.read()

    def get_stats(self) -> dict:
        return self._source.get_stats()

    def reset_filter(self):
        """Reset filter states (e.g. after reconnection)."""
        self._pipeline.reset()

    def close(self):
        self._source.close()
