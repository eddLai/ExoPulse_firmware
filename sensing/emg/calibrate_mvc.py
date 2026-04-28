"""
EMG MVC Calibration Tool

Records Maximum Voluntary Contraction (MVC) per channel, used by
NormalizedEMGModule to map filtered envelope -> muscle excitation [0, 1].

Channel layout (BLEEMGModule output order):
    [0] ABE_T2 - 右大腿後 (hamstrings_r)
    [1] ABE_T4 - 右大腿前 (rect_fem_r)
    [2] ABB_T2 - 左大腿後 (hamstrings_l)
    [3] ABB_T4 - 左大腿前 (rect_fem_l)

Usage:
    python calibrate_mvc.py -o mvc_calibration.json
    python calibrate_mvc.py -o mvc.json --duration 4 --rate 100
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ble_emg_module import BLEEMGModule
from emg_filter import FilteredEMGModule


CHANNEL_ORDER = ("ABE_T2", "ABE_T4", "ABB_T2", "ABB_T4")
CHANNEL_PROMPTS = {
    "ABE_T2": "右大腿後 (hamstrings_r) — 抗阻屈膝最大用力",
    "ABE_T4": "右大腿前 (rect_fem_r) — 抗阻伸膝最大用力",
    "ABB_T2": "左大腿後 (hamstrings_l) — 抗阻屈膝最大用力",
    "ABB_T4": "左大腿前 (rect_fem_l) — 抗阻伸膝最大用力",
}


def record_channel(emg, channel_idx, duration_sec, rate_hz):
    """Record envelope for one channel, return percentile statistics."""
    n = int(duration_sec * rate_hz)
    dt = 1.0 / rate_hz
    samples = np.zeros(n, dtype=np.float64)

    next_t = time.monotonic()
    for i in range(n):
        env = emg.read()
        samples[i] = env[channel_idx]
        next_t += dt
        sleep_left = next_t - time.monotonic()
        if sleep_left > 0:
            time.sleep(sleep_left)

    return {
        "p95": float(np.percentile(samples, 95)),
        "p99": float(np.percentile(samples, 99)),
        "max": float(np.max(samples)),
        "mean": float(np.mean(samples)),
        "n_samples": n,
    }


def warm_up_filter(emg, seconds=1.0, rate_hz=100.0):
    dt = 1.0 / rate_hz
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        emg.read()
        time.sleep(dt)


def main():
    parser = argparse.ArgumentParser(description="EMG MVC calibration")
    parser.add_argument("-o", "--output", required=True, help="Output JSON path")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Recording seconds per channel (default 3)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Read rate (Hz). Should match deployment step rate "
                             "(default 100, i.e. step_size=0.01)")
    parser.add_argument("--countdown", type=int, default=3,
                        help="Pre-record countdown seconds (default 3)")
    parser.add_argument("--rest", type=float, default=3.0,
                        help="Rest seconds between channels (default 3)")
    args = parser.parse_args()

    print(f"[MVC] Output: {args.output}")
    print(f"[MVC] Duration={args.duration}s @ {args.rate}Hz")
    print()

    print("Connecting BLE EMG ...")
    raw = BLEEMGModule()
    emg = FilteredEMGModule(raw, fs=args.rate)
    if not emg.init():
        print("[error] EMG init failed", file=sys.stderr)
        sys.exit(1)
    print("EMG ready.\n")

    print("Warming up filter (1 s)...")
    warm_up_filter(emg, seconds=1.0, rate_hz=args.rate)

    results = {}
    try:
        for idx, ch in enumerate(CHANNEL_ORDER):
            print(f"\n[{idx + 1}/4] {ch}: {CHANNEL_PROMPTS[ch]}")
            input("        Press ENTER when ready ...")
            for c in range(args.countdown, 0, -1):
                print(f"   {c}...", end="", flush=True)
                time.sleep(1.0)
            print(" GO!")

            emg.reset_filter()
            stats = record_channel(emg, idx, args.duration, args.rate)
            results[ch] = stats
            print(f"   p95={stats['p95']:.1f}  p99={stats['p99']:.1f}  "
                  f"max={stats['max']:.1f}  mean={stats['mean']:.1f}")

            print(f"   Rest {args.rest:.0f}s ...")
            time.sleep(args.rest)
    finally:
        emg.close()

    out = {ch: results[ch]["p95"] for ch in CHANNEL_ORDER}
    out["_stats"] = results
    out["_meta"] = {
        "duration_sec": args.duration,
        "rate_hz": args.rate,
        "channels": list(CHANNEL_ORDER),
        "method": "p95",
    }

    out_path = os.path.expanduser(args.output)
    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)

    print(f"\n[OK] Saved to {out_path}")
    for ch in CHANNEL_ORDER:
        print(f"  {ch}: p95 = {out[ch]:.1f}")


if __name__ == "__main__":
    main()
