# Isaac Sim 2023.1.1 - guaranteed replicator output (no USD downloads)
# Writes RGB + 2D tight bboxes to /workspace/output/ambiguous_mini_YYYYMMDD_HHMMSS
# Fails loudly if no PNGs are written.

import os
import sys
import time
import math
import random
import glob
from datetime import datetime

from omni.isaac.kit import SimulationApp

# IMPORTANT (2023.1.1): enable replicator extension BEFORE importing rep
simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RayTracedLighting",
    "extra_args": ["--enable", "omni.replicator.core"],
})

import carb
import omni.replicator.core as rep

def log(msg: str):
    carb.log_info(f"[WORKS_2023_1_1] {msg}")
    print(f"[WORKS_2023_1_1] {msg}", flush=True)

def main():
    random.seed(7)

    NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "30"))
    W = int(os.environ.get("W", "960"))
    H = int(os.environ.get("H", "540"))
    RES = (W, H)

    out_dir = os.environ.get(
        "OUT_DIR",
        f"/workspace/output/ambiguous_mini_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    os.makedirs(out_dir, exist_ok=True)

    log(f"OUT_DIR={out_dir}")
    log(f"NUM_FRAMES={NUM_FRAMES} RES={RES}")
