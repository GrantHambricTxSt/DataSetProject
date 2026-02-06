# Isaac Sim 2023.1.1 - Ambiguous tabletop dataset (PRIMITIVES ONLY, no USD downloads)
# Outputs RGB + 2D tight bboxes to /workspace/output/ambig_tabletop_YYYYMMDD_HHMMSS
#
# Run (headless):
#   NUM_FRAMES=60 W=960 H=540 /isaac-sim/python.sh ambiguous_tabletop_prims_2023_1_1.py \
#       --no-window --/app/window/enabled=false
#
# Optional ambiguity knobs (env):
#   OCCLUSION_P=0.65          # chance each bar occluder shows
#   WALL_OCCLUSION_P=0.85     # chance the moving wall is active
#   LOW_CONTRAST_P=0.60       # chance objects get near-table colors
#   CLUTTER_MIN=6 CLUTTER_MAX=16
#   CAM_LOWANGLE_P=0.55       # chance camera gets low and flat (harder)
#   LIGHT_JITTER=1            # 1 enables per-frame light jitter
#   SIMILAR_SHAPES_P=0.65     # push mug/bottle/can toward similar geometry

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
    carb.log_info(f"[AMBIG_TABLETOP_PRIMS] {msg}")
    print(f"[AMBIG_TABLETOP_PRIMS] {msg}", flush=True)


def cos_deg(deg): return math.cos(math.radians(deg))
def sin_deg(deg): return math.sin(math.radians(deg))


def clamp(x, a, b):
    return max(a, min(b, x))


def main():
    # -----------------------
    # Core config
    # -----------------------
    SEED = int(os.environ.get("SEED", "7"))
    random.seed(SEED)

    NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "60"))
    W = int(os.environ.get("W", "960"))
    H = int(os.environ.get("H", "540"))
    RES = (W, H)

    out_dir = os.environ.get(
        "OUT_DIR",
        f"/workspace/output/ambig_tabletop_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    os.makedirs(out_dir, exist_ok=True)

    # Table + camera ranges (meters-ish)
    TABLE_Z = 0.0
    TABLE_SIZE_X = float(os.environ.get("TABLE_X", "0.95"))
    TABLE_SIZE_Y = float(os.environ.get("TABLE_Y", "0.95"))

    CAM_RADIUS_RANGE = (0.55, 0.95)
    CAM_HEIGHT_RANGE = (0.18, 0.65)
    CAM_LOOK_Z_RANGE = (0.03, 0.22)

    # -----------------------
    # Ambiguity knobs (env)
    # -----------------------
    OCCLUSION_P = float(os.environ.get("OCCLUSION_P", "0.65"))
    WALL_OCCLUSION_P = float(os.environ.get("WALL_OCCLUSION_P", "0.85"))
    LOW_CONTRAST_P = float(os.environ.get("LOW_CONTRAST_P", "0.60"))
    CLUTTER_MIN = int(os.environ.get("CLUTTER_MIN", "6"))
    CLUTTER_MAX = int(os.environ.get("CLUTTER_MAX", "16"))
    CAM_LOWANGLE_P = float(os.environ.get("CAM_LOWANGLE_P", "0.55"))
    LIGHT_JITTER = int(os.environ.get("LIGHT_JITTER", "1"))
    SIMILAR_SHAPES_P = float(os.environ.get("SIMILAR_SHAPES_P", "0.65"))

    log(f"OUT_DIR={out_dir}")
    log(f"NUM_FRAMES={NUM_FRAMES} RES={RES} SEED={SEED}")
    log(f"Ambiguity: OCCLUSION_P={OCCLUSION_P} WALL_OCCLUSION_P={WALL_OCCLUSION_P} "
        f"LOW_CONTRAST_P={LOW_CONTRAST_P} CLUTTER=[{CLUTTER_MIN},{CLUTTER_MAX}] "
        f"CAM_LOWANGLE_P={CAM_LOWANGLE_P} LIGHT_JITTER={LIGHT_JITTER} SIMILAR_SHAPES_P={SIMILAR_SHAPES_P}")

    # -----------------------
    # Build graph (once)
    # -----------------------
    with rep.new_layer():
        # Table (plane) + backboard (gives horizon/edges)
        table = rep.create.plane(
            position=(0, 0, TABLE_Z),
            rotation=(0, 0, 0),
            scale=(TABLE_SIZE_X, TABLE_SIZE_Y, 1.0),
            visible=True,
        ).node

        backboard = rep.create.cube(
            position=(0.0, -0.55, 0.35),
            rotation=(80, 0, 0),
            scale=(1.6, 1.6, 0.03),
            visible=True,
        ).node

        # Base “mostly-white” scene colors (then we push low-contrast sometimes)
        def apply_scene_materials(low_contrast: bool):
            if low_contrast:
                # table and board close to each other: hard edges
                base = random.uniform(0.80, 0.93)
                t = (base, base, base)
                b = (clamp(base + random.uniform(-0.04, 0.04), 0.75, 0.97),) * 3
            else:
                # more separation
                t = (random.uniform(0.65, 0.90),) * 3
                b = (random.uniform(0.50, 0.80),) * 3

            rep.modify.material(
                input_prims=[table],
                diffuse=t,
                roughness=random.uniform(0.35, 0.95),
                metallic=0.0,
            )
            rep.modify.material(
                input_prims=[backboard],
                diffuse=b,
                roughness=random.uniform(0.4, 0.95),
                metallic=0.0,
            )

        # -----------------------
        # Occluders (bars + moving wall)
        # -----------------------
        bar_occluders = []
        for _ in range(5):
            bar = rep.create.cube(
                position=(0.0, 0.0, 0.18),
                scale=(0.05, 0.65, 0.35),
                visible=False,
            ).node
            bar_occluders.append(bar)

        wall = rep.create.cube(
            position=(0.0, 0.08, 0.12),
            rotation=(0, 0, 0),
            scale=(0.18, 0.85, 0.26),
            visible=True,
        ).node

        # -----------------------
        # Target objects (primitives)
        # mug-ish, bottle-ish, can-ish
        # -----------------------
        mug = rep.create.cylinder(position=(0, 0, -10), scale=(0.07, 0.07, 0.10), visible=True).node
        bottle = rep.create.cylinder(position=(0, 0, -10), scale=(0.055, 0.055, 0.18), visible=True).node
        can = rep.create.cylinder(position=(0, 0, -10), scale=(0.065, 0.065, 0.12), visible=True).node

        rep.modify.semantics(input_prims=[mug], semantics=[("class", "mug")])
        rep.modify.semantics(input_prims=[bottle], semantics=[("class", "bottle")])
        rep.modify.semantics(input_prims=[can], semantics=[("class", "can")])

        targets = [mug, bottle, can]

        # -----------------------
        # Clutter pool (lots of distractors)
        # -----------------------
        clutter_pool = []
        for _ in range(40):
            kind = random.choice(["cube", "sphere", "cylinder"])
            if kind == "sphere":
                c = rep.create.sphere(position=(0, 0, -10), scale=random.uniform(0.015, 0.05), visible=False).node
            elif kind == "cylinder":
                c = rep.create.cylinder(position=(0, 0, -10),
                                        scale=(random.uniform(0.015, 0.045),
                                               random.uniform(0.015, 0.045),
                                               random.uniform(0.03, 0.16)),
                                        visible=False).node
            else:
                s = random.uniform(0.02, 0.06)
                c = rep.create.cube(position=(0, 0, -10), scale=(s, s, s), visible=False).node
            rep.modify.semantics(input_prims=[c], semantics=[("class", "clutter")])
            clutter_pool.append(c)

        # -----------------------
        # Camera + render product
        # -----------------------
        camera = rep.create.camera().node
        render_product = rep.create.render_product(camera, RES)

        # -----------------------
        # Lighting (robust base + jitter option)
        # -----------------------
        dome = rep.create.light(light_type="dome", intensity=9000).node
        key = rep.create.light(light_type="distant", intensity=2600).node
        fill = rep.create.light(light_type="distant", intensity=900).node
        rim = rep.create.light(light_type="distant", intensity=700).node

        # -----------------------
        # Writer
        # -----------------------
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir=str(out_dir),
            rgb=True,
            bounding_box_2d_tight=True,
        )
        writer.attach([render_product])

        # -----------------------
        # Per-frame randomization
        # -----------------------
        def randomize_frame():
            # Decide global ambiguity mode
            low_contrast = (random.random() < LOW_CONTRAST_P)
            apply_scene_materials(low_contrast=low_contrast)

            # Materials for targets: in low-contrast, push colors near table
            def rand_color_near_gray():
                g = random.uniform(0.78, 0.95)
                return (clamp(g + random.uniform(-0.03, 0.03), 0.0, 1.0),
                        clamp(g + random.uniform(-0.03, 0.03), 0.0, 1.0),
                        clamp(g + random.uniform(-0.03, 0.03), 0.0, 1.0))

            def rand_color_normal():
                # muted colors that can still be confusing
                base = random.uniform(0.15, 0.85)
                return (clamp(base + random.uniform(-0.18, 0.18), 0.0, 1.0),
                        clamp(base + random.uniform(-0.18, 0.18), 0.0, 1.0),
                        clamp(base + random.uniform(-0.18, 0.18), 0.0, 1.0))

            for prim in targets:
                col = rand_color_near_gray() if low_contrast else rand_color_normal()
                rep.modify.material(
                    input_prims=[prim],
                    diffuse=col,
                    roughness=random.uniform(0.15, 0.95),
                    metallic=random.uniform(0.0, 0.35),
                )

            # Make shapes more similar sometimes (harder classification)
            if random.random() < SIMILAR_SHAPES_P:
                # converge all heights/radii toward a shared range
                r = random.uniform(0.055, 0.070)
                h = random.uniform(0.10, 0.16)
                rep.modify.pose(input_prims=[mug], scale=(r, r, h))
                rep.modify.pose(input_prims=[bottle], scale=(r * random.uniform(0.95, 1.05), r * random.uniform(0.95, 1.05), h * random.uniform(0.95, 1.10)))
                rep.modify.pose(input_prims=[can], scale=(r * random.uniform(0.95, 1.05), r * random.uniform(0.95, 1.05), h * random.uniform(0.85, 1.05)))
            else:
                # keep them somewhat distinct
                rep.modify.pose(input_prims=[mug], scale=(random.uniform(0.06, 0.08), random.uniform(0.06, 0.08), random.uniform(0.08, 0.13)))
                rep.modify.pose(input_prims=[bottle], scale=(random.uniform(0.045, 0.07), random.uniform(0.045, 0.07), random.uniform(0.14, 0.22)))
                rep.modify.pose(input_prims=[can], scale=(random.uniform(0.055, 0.08), random.uniform(0.055, 0.08), random.uniform(0.09, 0.15)))

            # Place targets close-ish to each other to induce overlaps/bbox ambiguity
            for prim in targets:
                x = random.uniform(-TABLE_SIZE_X * 0.20, TABLE_SIZE_X * 0.20)
                y = random.uniform(-TABLE_SIZE_Y * 0.18, TABLE_SIZE_Y * 0.22)
                yaw = random.uniform(0, 360)
                # small tilt makes silhouettes confusing
                pitch = random.uniform(-12, 12)
                roll = random.uniform(-12, 12)
                rep.modify.pose(
                    input_prims=[prim],
                    position=(x, y, TABLE_Z + random.uniform(0.03, 0.06)),
                    rotation=(pitch, roll, yaw),
                )

            # Occlusion bars
            for bar in bar_occluders:
                rep.modify.visibility(input_prims=[bar], value=(random.random() < OCCLUSION_P))
                rep.modify.pose(
                    input_prims=[bar],
                    position=(random.uniform(-0.25, 0.25), random.uniform(-0.05, 0.25), random.uniform(0.12, 0.26)),
                    rotation=(0, 0, random.uniform(-25, 25)),
                )
                rep.modify.material(
                    input_prims=[bar],
                    diffuse=(random.uniform(0.0, 0.12),) * 3,
                    roughness=random.uniform(0.3, 0.95),
                    metallic=0.0,
                )

            # Moving wall occluder
            wall_active = (random.random() < WALL_OCCLUSION_P)
            rep.modify.visibility(input_prims=[wall], value=wall_active)
            if wall_active:
                rep.modify.pose(
                    input_prims=[wall],
                    position=(random.uniform(-0.18, 0.18), random.uniform(-0.05, 0.30), random.uniform(0.10, 0.16)),
                    rotation=(0, 0, random.uniform(-28, 28)),
                )
                rep.modify.material(
                    input_prims=[wall],
                    diffuse=(random.uniform(0.0, 0.10),) * 3,
                    roughness=random.uniform(0.25, 0.9),
                    metallic=0.0,
                )

            # Clutter: lots of small distractors around/near targets
            active_clutter = random.randint(CLUTTER_MIN, CLUTTER_MAX)
            chosen = random.sample(clutter_pool, active_clutter)
            for prim in clutter_pool:
                rep.modify.visibility(input_prims=[prim], value=(prim in chosen))

            for prim in chosen:
                cx = random.uniform(-TABLE_SIZE_X * 0.45, TABLE_SIZE_X * 0.45)
                cy = random.uniform(-TABLE_SIZE_Y * 0.40, TABLE_SIZE_Y * 0.48)
                cz = TABLE_Z + random.uniform(0.01, 0.08)
                rep.modify.pose(
                    input_prims=[prim],
                    position=(cx, cy, cz),
                    rotation=(random.uniform(-25, 25), random.uniform(-25, 25), random.uniform(0, 360)),
                )
                # clutter often same-ish tones to reduce separability
                if low_contrast:
                    col = (random.uniform(0.72, 0.92),) * 3
                else:
                    col = (random.uniform(0.10, 0.90),
                           random.uniform(0.10, 0.90),
                           random.uniform(0.10, 0.90))
                rep.modify.material(
                    input_prims=[prim],
                    diffuse=col,
                    roughness=random.uniform(0.25, 0.98),
                    metallic=random.uniform(0.0, 0.20),
                )

            # Camera: sometimes low and flat (hard), sometimes normal orbit
            r = random.uniform(*CAM_RADIUS_RANGE)
            theta = random.uniform(0, 360)
            cam_x = r * cos_deg(theta)
            cam_y = r * sin_deg(theta)

            if random.random() < CAM_LOWANGLE_P:
                cam_z = random.uniform(0.14, 0.28)  # low
                look_z = random.uniform(0.03, 0.10) # low look-at
            else:
                cam_z = random.uniform(*CAM_HEIGHT_RANGE)
                look_z = random.uniform(*CAM_LOOK_Z_RANGE)

            rep.modify.pose(
                input_prims=[camera],
                position=(cam_x, cam_y, cam_z),
                look_at=(random.uniform(-0.03, 0.03), random.uniform(0.05, 0.14), look_z),
            )

            # Light jitter: makes shadows/specular confusing
            if LIGHT_JITTER:
                rep.modify.pose(input_prims=[key], rotation=(random.uniform(-15, 15), random.uniform(-15, 15), random.uniform(0, 360)))
                rep.modify.pose(input_prims=[fill], rotation=(random.uniform(-25, 25), random.uniform(-25, 25), random.uniform(0, 360)))
                rep.modify.pose(input_prims=[rim], rotation=(random.uniform(-25, 25), random.uniform(-25, 25), random.uniform(0, 360)))

        rep.randomizer.register(randomize_frame)

        with rep.trigger.on_frame(max_execs=NUM_FRAMES):
            rep.randomizer.randomize_frame()

    # -----------------------
    # IMPORTANT: headless flush strategy (the part that actually makes files)
    # This is the same “warmup + step per frame” pattern you proved works. :contentReference[oaicite:1]{index=1}
    # -----------------------
    log("Warmup updates…")
    for _ in range(60):
        simulation_app.update()

    try:
        rep.orchestrator.reset()
    except Exception:
        pass

    print("STARTING REPLICATOR (stepping 1 frame at a time)", flush=True)
    for i in range(NUM_FRAMES):
        rep.orchestrator.step(1)
        simulation_app.update()
        if i % 10 == 0:
            print(f"[AMBIG_TABLETOP_PRIMS] stepped {i}/{NUM_FRAMES}", flush=True)

    # extra flush frames
    for _ in range(40):
        simulation_app.update()

    rgb_dir = os.path.join(out_dir, "rgb")
    rgb_files = glob.glob(os.path.join(rgb_dir, "*.png"))
    log(f"rgb_dir={rgb_dir} rgb_files_found={len(rgb_files)}")

    if len(rgb_files) == 0:
        raise RuntimeError("No RGB images were written (render product did not render).")

    print("REPLICATOR FINISHED", flush=True)
    print(f"[AMBIG_TABLETOP_PRIMS] Wrote to: {out_dir}", flush=True)

    # Avoid close() issues in some cloud builds after success
    os._exit(0)


if __name__ == "__main__":
    main()
