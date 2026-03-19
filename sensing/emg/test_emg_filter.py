#!/usr/bin/env python3
"""
EMG Filter Pipeline Test — synthetic signals, no hardware needed.

Tests:
    1. High-pass rejects DC / low-frequency, passes EMG band
    2. Full-wave rectification + low-pass produces smooth envelope
    3. Streaming continuity — batch-by-batch matches one-shot
"""

import sys
import numpy as np

sys.path.insert(0, ".")
from emg_filter import EMGFilterPipeline


FS = 500.0        # Hz
DURATION = 2.0    # seconds
N = int(FS * DURATION)
t = np.arange(N) / FS


def test_highpass_rejects_dc():
    """A pure DC signal should be almost fully attenuated."""
    pipeline = EMGFilterPipeline(n_channels=1, fs=FS)

    dc = np.ones((1, N)) * 1000.0
    out = pipeline.process_batch(dc)

    # After settling, output should be near zero
    tail = out[0, N // 2:]
    rms = np.sqrt(np.mean(tail ** 2))
    print(f"[HP reject DC]  Input=1000 DC  →  Output RMS = {rms:.4f}")
    assert rms < 1.0, f"DC not rejected: RMS={rms:.4f}"
    print("  PASS\n")


def test_highpass_passes_emg_band():
    """A 100 Hz sine (within EMG band) should pass through high-pass."""
    pipeline = EMGFilterPipeline(n_channels=1, fs=FS)

    sig = np.sin(2 * np.pi * 100 * t).reshape(1, -1)
    out = pipeline.process_batch(sig)

    # After high-pass only (before rectification), check the high-pass output
    # We test the full pipeline, so the output is the envelope.
    # A 100 Hz sine rectified and low-passed → ~constant envelope.
    tail = out[0, N // 2:]
    mean_val = np.mean(tail)
    print(f"[HP pass 100Hz]  100Hz sine → envelope mean = {mean_val:.4f}")
    # Rectified sine mean = 2*A/pi ≈ 0.637 for A=1, then LP smooths it
    assert mean_val > 0.2, f"100Hz signal lost: mean={mean_val:.4f}"
    print("  PASS\n")


def test_lowfreq_rejected():
    """A 5 Hz sine (below HP cutoff) should be heavily attenuated."""
    pipeline = EMGFilterPipeline(n_channels=1, fs=FS)

    sig = np.sin(2 * np.pi * 5 * t).reshape(1, -1)
    out = pipeline.process_batch(sig)

    tail = out[0, N // 2:]
    rms = np.sqrt(np.mean(tail ** 2))
    print(f"[HP reject 5Hz]  5Hz sine → output RMS = {rms:.4f}")
    assert rms < 0.1, f"5Hz not rejected: RMS={rms:.4f}"
    print("  PASS\n")


def test_envelope_smoothness():
    """Rectified EMG-like signal should produce a smooth envelope."""
    pipeline = EMGFilterPipeline(n_channels=1, fs=FS)

    # Simulate EMG: 100 Hz carrier modulated by 2 Hz envelope
    envelope = 0.5 + 0.5 * np.sin(2 * np.pi * 2 * t)
    emg = envelope * np.sin(2 * np.pi * 100 * t)
    out = pipeline.process_batch(emg.reshape(1, -1))

    tail = out[0, N // 2:]
    # Smoothness: std of derivative should be small relative to signal
    deriv = np.diff(tail)
    smoothness = np.std(deriv) / (np.mean(np.abs(tail)) + 1e-9)
    print(f"[Envelope smooth]  Modulated EMG → smoothness = {smoothness:.4f}")
    assert smoothness < 0.5, f"Envelope not smooth: {smoothness:.4f}"
    print("  PASS\n")


def test_streaming_continuity():
    """Processing in small batches should match one-shot processing."""
    # One-shot
    pipeline_full = EMGFilterPipeline(n_channels=2, fs=FS)
    sig = np.random.randn(2, N) * 100
    out_full = pipeline_full.process_batch(sig)

    # Streaming in chunks of 7 (one BLE packet)
    pipeline_stream = EMGFilterPipeline(n_channels=2, fs=FS)
    chunks = []
    for i in range(0, N, 7):
        end = min(i + 7, N)
        chunk = pipeline_stream.process_batch(sig[:, i:end])
        chunks.append(chunk)
    out_stream = np.concatenate(chunks, axis=1)

    diff = np.max(np.abs(out_full - out_stream))
    print(f"[Streaming]  One-shot vs 7-sample chunks → max diff = {diff:.2e}")
    assert diff < 1e-10, f"Streaming mismatch: {diff:.2e}"
    print("  PASS\n")


def test_multichannel():
    """4-channel processing should work independently."""
    pipeline = EMGFilterPipeline(n_channels=4, fs=FS)

    sigs = np.zeros((4, N))
    sigs[0] = np.sin(2 * np.pi * 50 * t)   # 50 Hz — should pass
    sigs[1] = np.sin(2 * np.pi * 5 * t)    # 5 Hz — should be rejected
    sigs[2] = np.sin(2 * np.pi * 80 * t)   # 80 Hz — should pass
    sigs[3] = np.ones(N) * 500              # DC — should be rejected

    out = pipeline.process_batch(sigs)
    tail = out[:, N // 2:]
    rms = np.sqrt(np.mean(tail ** 2, axis=1))

    print(f"[Multichannel]  RMS: ch0(50Hz)={rms[0]:.4f}  ch1(5Hz)={rms[1]:.4f}  "
          f"ch2(80Hz)={rms[2]:.4f}  ch3(DC)={rms[3]:.4f}")
    assert rms[0] > 0.2, "50Hz lost"
    assert rms[1] < 0.1, "5Hz not rejected"
    assert rms[2] > 0.2, "80Hz lost"
    assert rms[3] < 1.0, "DC not rejected"
    print("  PASS\n")


if __name__ == "__main__":
    print("=" * 50)
    print("EMG Filter Pipeline — Synthetic Signal Tests")
    print(f"Sample rate: {FS} Hz, Duration: {DURATION}s, Samples: {N}")
    print("=" * 50 + "\n")

    tests = [
        test_highpass_rejects_dc,
        test_highpass_passes_emg_band,
        test_lowfreq_rejected,
        test_envelope_smoothness,
        test_streaming_continuity,
        test_multichannel,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"  FAIL: {e}\n")
            failed += 1
        except Exception as e:
            print(f"  ERROR: {e}\n")
            failed += 1

    print("=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 50)
    sys.exit(0 if failed == 0 else 1)
