# ambiguous_mini.py (Isaac Sim 2023.1.1 safe)
print("=== ambiguous_mini.py STARTED ===", flush=True)

import os
import sys
import time
import random
import math
from datetime import datetime
import glob

import carb
import omni.replicator.core as rep
from omni.isaac.kit import SimulationApp

# Force headless RTX path as much as 2023.1.1 allows
simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RayTracedLighting",
})

def cos_deg(deg): return math.cos(math.radians(deg))
def sin_deg(deg): return math.sin(math.radians(deg))

# -----------------------------
# CONFIG
# -----------------------------
SEED = 7
NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "60"))
RESOLUTION = (int(os.environ.get("W", "960")), int(os.environ.get("H", "540")))

DEFAULT_OUT_DIR = f"/workspace/output/ambiguous_mini_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
OUTPUT_DIR = os.environ.get("OUT_DIR", DEFAULT_OUT_DIR)

ASSET_URLS = [
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/Mugs/SM_Mug_A2.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
]

MIN_OBJECTS = 2
MAX_OBJECTS = 3

TABLE_SIZE_X = 0.9
TABLE_SIZE_Y = 0.9
TABLE_Z = 0.0

CAM_RADIUS_RANGE = (0.55, 0.85)
CAM_HEIGHT_RANGE = (0.25, 0.55)
CAM_LOOK_AT_Z_RANGE = (0.05, 0.20)

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def log(msg: str):
    carb.log_info(f"[ambiguous_mini] {msg}")

def main():
    random.seed(SEED)
    ensure_dir(OUTPUT_DIR)

    log(f"Output: {OUTPUT_DIR}")
    log(f"Frames: {NUM_FRAMES}, Resolution: {RESOLUTION}")
    print(f"[ambiguous_mini] OUT_DIR={OUTPUT_DIR}", flush=True)

    with rep.new_layer():
        # Table
        rep.create.plane(
            position=(0, 0, TABLE_Z),
            rotation=(0, 0, 0),
            scale=(TABLE_SIZE_X, TABLE_SIZE_Y, 1.0),
            visible=True,
        )

        # Occluders
        occluders = []
        for _ in range(3):
            occ = rep.create.cube(
                position=(0.0, 0.0, 0.15),
                scale=(0.10, 0.45, 0.22),
                visible=False,
            )
            occluders.append(occ.node)

        occluder_wall = rep.create.cube(
            position=(0.0, 0.0, 0.10),
            scale=(0.12, 0.40, 0.20),
            visible=True,
        )

        # Clutter pool
        clutter_prims = []
        for _ in range(12):
            kind = random.choice(["sphere", "cube", "cylinder"])
            if kind == "sphere":
                c = rep.create.sphere(position=(0, 0, -10), scale=0.03, visible=False)
            elif kind == "cylinder":
                c = rep.create.cylinder(position=(0, 0, -10), scale=(0.03, 0.03, 0.08), visible=False)
            else:
                c = rep.create.cube(position=(0, 0, -10), scale=0.04, visible=False)
            clutter_prims.append(c.node)

        # Camera + render product
        camera = rep.create.camera()
        render_product = rep.create.render_product(camera, RESOLUTION)

        # Lights
        rep.create.light(light_type="dome", intensity=8000)
        rep.create.light(light_type="distant", intensity=2500)
        rep.create.light(light_type="distant", intensity=800)
        rep.create.light(light_type="distant", intensity=600)

        # Objects
        objs = []
        for i, url in enumerate(ASSET_URLS):
            obj = rep.create.from_usd(url).node
            rep.modify.semantics(input_prims=[obj], semantics=[("class", f"obj_{i}")])
            objs.append(obj)

        # Writer
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir=str(OUTPUT_DIR),
            rgb=True,
            bounding_box_2d_tight=True,
        )
        writer.attach([render_product])

        # Per-frame randomization
        def randomize_frame():
            k = random.randint(MIN_OBJECTS, MAX_OBJECTS)
            chosen = set(random.sample(range(len(objs)), k))

            for idx, prim in enumerate(objs):
                rep.modify.visibility(input_prims=[prim], value=(idx in chosen))

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

            for occ_prim in occluders:
                rep.modify.visibility(input_prims=[occ_prim], value=(random.random() < 0.45))

            ox = random.uniform(-0.15, 0.15)
            oy = random.uniform(-0.05, 0.25)
            rep.modify.pose(
                input_prims=[occluder_wall.node],
                position=(ox, oy, 0.10),
                rotation=(0, 0, random.uniform(-20, 20)),
            )

            active = random.randint(2, 6)
            selected = random.sample(clutter_prims, active)
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

        with rep.trigger.on_frame(max_execs=NUM_FRAMES):
            rep.randomizer.randomize_frame()

    # Warmup
    log("Warmup updates…")
    for _ in range(120):
        simulation_app.update()

    # Reset if available
    try:
        rep.orchestrator.reset()
    except Exception:
        pass

    print("STARTING REPLICATOR (run + update loop)", flush=True)

    # 2023.1.1: run() + update loop is more reliable than step()
    rep.orchestrator.run()

    # Drive Kit long enough for frames + flush
    ticks = NUM_FRAMES + 240
    for i in range(ticks):
        simulation_app.update()
        if i % 60 == 0:
            print(f"[ambiguous_mini] update tick {i}/{ticks}", flush=True)

    # Stop (safe)
    try:
        rep.orchestrator.stop()
    except Exception:
        pass

    # Extra flush
    for _ in range(120):
        simulation_app.update()

    print("REPLICATOR FINISHED", flush=True)
    print(f"[ambiguous_mini] Wrote to: {OUTPUT_DIR}", flush=True)

    # Assert output exists (INSIDE main)
    rgb_dir = os.path.join(OUTPUT_DIR, "rgb")
    rgb_files = glob.glob(os.path.join(rgb_dir, "*.png"))
    print(f"[ambiguous_mini] rgb_dir={rgb_dir}", flush=True)
    print(f"[ambiguous_mini] rgb_files_found={len(rgb_files)}", flush=True)

    if len(rgb_files) == 0:
        raise RuntimeError(
            "No RGB images were written. Replicator ran, but render/writer did not produce frames. "
            "Next checks: asset downloads reachable, renderer init, permissions, or graphics context."
        )

    simulation_app.close()
    sys.exit(0)

if __name__ == "__main__":
    main()
