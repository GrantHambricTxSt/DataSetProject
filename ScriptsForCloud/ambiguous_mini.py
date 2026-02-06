# ambiguous_mini.py
# Scaled-down "final style" Replicator script: fast test run, same structure as full dataset.
# Outputs: RGB + 2D tight bboxes (and basic metadata)
#
# Run (headless):
#   /isaac-sim/python.sh ambiguous_mini.py --/omni/replicator/rt_subframes=4
#
# Env overrides:
#   NUM_FRAMES=60 W=960 H=540 OUT_DIR=/workspace/output/somewhere /isaac-sim/python.sh ambiguous_mini.py

print("=== ambiguous_mini.py STARTED ===", flush=True)

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
import sys
import time
import random
import math
from datetime import datetime

import carb
import omni.replicator.core as rep


def cos_deg(deg): return math.cos(math.radians(deg))
def sin_deg(deg): return math.sin(math.radians(deg))


# -----------------------------
# CONFIG (scale these later)
# -----------------------------
SEED = 7

NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "60"))  # scale to 2k+ later
RESOLUTION = (int(os.environ.get("W", "960")), int(os.environ.get("H", "540")))  # scale to 1920x1080 later

# Default to unique folder so you never confuse new vs old output
DEFAULT_OUT_DIR = f"/workspace/output/ambiguous_mini_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
OUTPUT_DIR = os.environ.get("OUT_DIR", DEFAULT_OUT_DIR)

# Your URLs (as requested)
ASSET_URLS = [
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/Mugs/SM_Mug_A2.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
]

# For "ambiguity": allow partial occlusion and similar poses
MIN_OBJECTS = 2
MAX_OBJECTS = 3

# World/tabletop sizes (meters-ish; Isaac units are usually meters)
TABLE_SIZE_X = 0.9
TABLE_SIZE_Y = 0.9
TABLE_Z = 0.0

# Camera orbit ranges (simple + stable)
CAM_RADIUS_RANGE = (0.55, 0.85)
CAM_HEIGHT_RANGE = (0.25, 0.55)
CAM_LOOK_AT_Z_RANGE = (0.05, 0.20)

# Light randomization (currently fixed lights, you can re-enable jitter later)
KEY_LIGHT_INTENSITY = (250, 900)
FILL_LIGHT_INTENSITY = (50, 300)


# -----------------------------
# Helpers
# -----------------------------
def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def log(msg: str):
    carb.log_info(f"[ambiguous_mini] {msg}")


# -----------------------------
# Main
# -----------------------------
def main():
    random.seed(SEED)
    ensure_dir(OUTPUT_DIR)

    log(f"Output: {OUTPUT_DIR}")
    log(f"Frames: {NUM_FRAMES}, Resolution: {RESOLUTION}")
    print(f"[ambiguous_mini] OUT_DIR={OUTPUT_DIR}", flush=True)

    # Build everything inside a new layer
    with rep.new_layer():
        # -----------------------------
        # Scene: simple tabletop plane (fast)
        # -----------------------------
        table = rep.create.plane(
            position=(0, 0, TABLE_Z),
            rotation=(0, 0, 0),
            scale=(TABLE_SIZE_X, TABLE_SIZE_Y, 1.0),
            visible=True,
        )

        # -----------------------------
        # Occluders (created once)
        # -----------------------------
        occluders = []
        NUM_OCCLUDERS = 3
        for _ in range(NUM_OCCLUDERS):
            occ = rep.create.cube(
                position=(0.0, 0.0, 0.15),
                scale=(0.10, 0.45, 0.22),  # thin vertical bars
                visible=False,
            )
            occluders.append(occ.node)

        # A low wall occluder that creates ambiguity
        occluder = rep.create.cube(
            position=(0.0, 0.0, 0.10),
            scale=(0.12, 0.40, 0.20),
            visible=True,
        )

        # -----------------------------
        # Clutter pool (created once)
        # -----------------------------
        CLUTTER_POOL = 12
        clutter_prims = []
        for _ in range(CLUTTER_POOL):
            kind = random.choice(["sphere", "cube", "cylinder"])
            if kind == "sphere":
                c = rep.create.sphere(position=(0, 0, -10), scale=0.03, visible=False)
            elif kind == "cylinder":
                c = rep.create.cylinder(position=(0, 0, -10), scale=(0.03, 0.03, 0.08), visible=False)
            else:
                c = rep.create.cube(position=(0, 0, -10), scale=0.04, visible=False)
            clutter_prims.append(c.node)

        # -----------------------------
        # Camera + render product
        # -----------------------------
        camera = rep.create.camera()
        render_product = rep.create.render_product(camera, RESOLUTION)

        # -----------------------------
        # Lights (created once)
        # -----------------------------
        # Add a dome light for robustness (prevents flat/blank-gray “nothing lit” cases)
        rep.create.light(light_type="dome", intensity=8000)

        key = rep.create.light(light_type="distant", intensity=2500)
        fill = rep.create.light(light_type="distant", intensity=800)
        rim = rep.create.light(light_type="distant", intensity=600)

        # -----------------------------
        # Create objects (toggle visibility per frame)
        # -----------------------------
        objs = []
        for i, url in enumerate(ASSET_URLS):
            obj = rep.create.from_usd(url).node
            rep.modify.semantics(input_prims=[obj], semantics=[("class", f"obj_{i}")])
            objs.append(obj)

        # -----------------------------
        # Writer
        # -----------------------------
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir=str(OUTPUT_DIR),
            rgb=True,
            bounding_box_2d_tight=True,
        )
        writer.attach([render_product])

        # -----------------------------
        # Per-frame randomization (your ambiguity)
        # -----------------------------
        def randomize_frame():
            # Choose how many objects this frame
            k = random.randint(MIN_OBJECTS, MAX_OBJECTS)
            chosen = set(random.sample(range(len(objs)), k))

            # Visibility toggling
            for idx, prim in enumerate(objs):
                rep.modify.visibility(input_prims=[prim], value=(idx in chosen))

            # Randomize object placement/rotation on table
            for idx, prim in enumerate(objs):
                if idx not in chosen:
                    continue

                x = random.uniform(-TABLE_SIZE_X * 0.35, TABLE_SIZE_X * 0.35)
                y = random.uniform(-TABLE_SIZE_Y * 0.35, TABLE_SIZE_Y * 0.35)
                yaw = random.uniform(0, 360)
                pitch = random.uniform(-6, 6)
                roll = random.uniform(-6, 6)

                rep.modify.pose(
                    input_prims=[prim],
                    position=(x, y, TABLE_Z + 0.02),
                    rotation=(pitch, roll, yaw),
                )

            # Occluder behavior (ambiguity)
            # - sometimes make bars visible
            for occ_prim in occluders:
                rep.modify.visibility(input_prims=[occ_prim], value=(random.random() < 0.45))

            # animate the wall occluder so it sometimes blocks the objects
            ox = random.uniform(-0.15, 0.15)
            oy = random.uniform(-0.05, 0.25)
            rep.modify.pose(
                input_prims=[occluder.node],
                position=(ox, oy, 0.10),
                rotation=(0, 0, random.uniform(-20, 20)),
            )

            # Clutter: toggle a few random clutter prims and place them near edges
            active_clutter = random.randint(2, 6)
            selected = random.sample(clutter_prims, active_clutter)
            for prim in clutter_prims:
                rep.modify.visibility(input_prims=[prim], value=(prim in selected))

            for prim in selected:
                cx = random.uniform(-TABLE_SIZE_X * 0.45, TABLE_SIZE_X * 0.45)
                cy = random.uniform(-TABLE_SIZE_Y * 0.45, TABLE_SIZE_Y * 0.45)
                cz = TABLE_Z + random.uniform(0.01, 0.06)
                rep.modify.pose(
                    input_prims=[prim],
                    position=(cx, cy, cz),
                    rotation=(0, 0, random.uniform(0, 360)),
                )

            # Camera orbit jitter
            r = random.uniform(*CAM_RADIUS_RANGE)
            theta = random.uniform(0, 360)
            cam_x = r * cos_deg(theta)
            cam_y = r * sin_deg(theta)
            cam_z = random.uniform(*CAM_HEIGHT_RANGE)
            look_at_z = random.uniform(*CAM_LOOK_AT_Z_RANGE)
            rep.modify.pose(
                input_prims=[camera],
                position=(cam_x, cam_y, cam_z),
                look_at=(0.0, 0.0, look_at_z),
            )

        rep.randomizer.register(randomize_frame)

        # Trigger: run per frame (exactly NUM_FRAMES)
        with rep.trigger.on_frame(max_execs=NUM_FRAMES):
            rep.randomizer.randomize_frame()

    # Let Isaac settle before stepping replicator (helps headless)
    log("Starting orchestration…")
    for _ in range(30):
        simulation_app.update()

    print("STARTING REPLICATOR", flush=True)

    # ✅ Correct API for N frames:
    rep.orchestrator.step(NUM_FRAMES)

    # Allow frames to flush/render in headless mode
    for _ in range(NUM_FRAMES + 30):
        simulation_app.update()
        time.sleep(0.03)

    print("REPLICATOR FINISHED", flush=True)

    # Avoid simulation_app.close() on this build (you saw segfaults after success)
    os._exit(0)


if __name__ == "__main__":
    main()
