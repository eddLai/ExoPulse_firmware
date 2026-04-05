#!/usr/bin/env python3
"""
將 .pt checkpoint 轉換為 .npz（純 numpy 格式）。

用法:
    conda activate forward_sim
    python convert_checkpoint.py <checkpoint.pt>

範例:
    python convert_checkpoint.py /path/to/scheme_b_imu/checkpoints/expert_3/step_2300000.pt

轉換後會在同目錄產生 .npz 檔，之後 realtime_controller.py 不再需要 torch。
"""

import sys
from pathlib import Path

ACTOR_KEYS = [
    "actor.encoder.observation_normalizer._mean",
    "actor.encoder.observation_normalizer._std",
    "actor.torso.model.0.weight", "actor.torso.model.0.bias",
    "actor.torso.model.2.weight", "actor.torso.model.2.bias",
    "actor.torso.model.4.weight", "actor.torso.model.4.bias",
    "actor.head.action_layer.0.weight", "actor.head.action_layer.0.bias",
]


def convert(pt_path: str):
    import torch
    import numpy as np

    pt_path = Path(pt_path)
    npz_path = pt_path.with_suffix(".npz")

    if npz_path.exists():
        print(f"已存在: {npz_path}")
        ans = input("要覆蓋嗎? [y/N] ").strip().lower()
        if ans != "y":
            print("取消。")
            return

    print(f"載入: {pt_path}")
    cp = torch.load(str(pt_path), map_location="cpu")

    weights = {}
    for key in ACTOR_KEYS:
        if key not in cp:
            print(f"  警告: 找不到 {key}，跳過")
            continue
        weights[key] = cp[key].float().numpy()
        print(f"  {key}: {weights[key].shape}")

    np.savez(str(npz_path), **weights)
    size_kb = npz_path.stat().st_size / 1024
    print(f"\n完成: {npz_path} ({size_kb:.1f} KB)")
    print(f"之後 realtime_controller.py 不需要 torch 即可運行。")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python convert_checkpoint.py <checkpoint.pt>")
        print("範例: python convert_checkpoint.py ../../../scheme_b_imu/checkpoints/expert_3/step_2300000.pt")
        sys.exit(1)
    convert(sys.argv[1])
