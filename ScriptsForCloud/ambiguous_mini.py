# ambiguous_mini.py
# Scaled-down "final style" Replicator script: fast test run, same structure as full dataset.
# Outputs: RGB + 2D tight bboxes (and basic metadata)
#
# Run (headless):
#   ./python.sh ambiguous_mini.py --/omni/replicator/rt_subframes=4
#
# If your environment doesn't have python.sh, use the Isaac Sim provided launcher for your version.
print("=== ambiguous_mini.py STARTED ===", flush=True)

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
import random
import math
import carb
import omni.replicator.core as rep

def cos_deg(deg): return math.cos(math.radians(deg))
def sin_deg(deg): return math.sin(math.radians(deg))

# -----------------------------
# CONFIG (scale these later)
# -----------------------------
SEED = 7

NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "60"))     # scale to 2k+ later
RESOLUTION = (int(os.environ.get("W", "960")), int(os.environ.get("H", "540")))  # scale to 1920x1080 later

OUTPUT_DIR = os.environ.get("OUT_DIR", "/workspace/output/ambiguous_mini")

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

# Light randomization
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

    # Reset Replicator state (safe when re-running)
    #rep.reset()

    # -----------------------------
    # Scene: simple tabletop plane (fast)
    # -----------------------------
    with rep.new_layer():
        # Table plane
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


        # Optional: a low wall/occluder (creates ambiguity without heavy assets)
        # We’ll animate its position so sometimes it blocks part of objects.
        occluder = rep.create.cube(
            position=(0.0, 0.0, 0.10),
            scale=(0.12, 0.40, 0.20),
            visible=True,
        )

        # -----------------------------
        # Clutter pool (created once)
        # -----------------------------
        CLUTTER_POOL = 12  # total clutter prims available
        clutter_prims = []

        for _ in range(CLUTTER_POOL):
            # mix primitive types for variety
            kind = random.choice(["sphere", "cube", "cylinder"])
            if kind == "sphere":
                c = rep.create.sphere(position=(0,0,-10), scale=0.03, visible=False)
            elif kind == "cylinder":
                c = rep.create.cylinder(position=(0,0,-10), scale=(0.03, 0.03, 0.08), visible=False)
            else:
                c = rep.create.cube(position=(0,0,-10), scale=0.04, visible=False)

            clutter_prims.append(c.node)


        # Camera
        camera = rep.create.camera()
        render_product = rep.create.render_product(camera, RESOLUTION)

        # Lights: one key + one fill
        # -----------------------------


        # Lights (created once)
        # -----------------------------
        key = rep.create.light(light_type="distant", intensity=2500)
        fill = rep.create.light(light_type="distant", intensity=800)
        rim  = rep.create.light(light_type="distant", intensity=600)



        # -----------------------------
        # Create objects (we’ll toggle visibility per frame)
        # -----------------------------
        objs = []
        for i, url in enumerate(ASSET_URLS):
            obj = rep.create.from_usd(url)
            # Give them stable names for annotations
            obj = obj.node
            rep.modify.semantics(
                input_prims=[obj],
                semantics=[("class", f"obj_{i}")]
            )

            objs.append(obj)

        # Writer: RGB + bbox2d tight + basic metadata
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir=OUTPUT_DIR,
            rgb=True,
            bounding_box_2d_tight=True,
            # You can add these later when scaling:
            # bounding_box_2d_loose=True,
            # segmentation=True,
            # depth=True,
            # normals=True,
        )
        writer.attach([render_product])

        # -----------------------------
        # Per-frame randomization
        # -----------------------------


        def randomize_frame():
            # Choose how many objects this frame
            k = random.randint(MIN_OBJECTS, MAX_OBJECTS)
            chosen = set(random.sample(range(len(objs)), k))

            # Visibility toggling: chosen visible, others hidden
            for idx, prim in enumerate(objs):
                rep.modify.visibility(input_prims=[prim], value=(idx in chosen))

            # Randomize object placement/rotation on table
            for idx, prim in enumerate(objs):
                if idx not in chosen:
                    continue

                # Sample x,y in table bounds
                x = random.uniform(-TABLE_SIZE_X * 0.35, TABLE_SIZE_X * 0.35)
                y = random.uniform(-TABLE_SIZE_Y * 0.35, TABLE_SIZE_Y * 0.35)
                yaw = random.uniform(0, 360)

                # Small tilt sometimes to create “ambiguous” viewpoints
                pitch = random.uniform(-6, 6)
                roll = random.uniform(-6, 6)

                rep.modify.pose(
                    input_prims=[prim],
                    position=(x, y, TABLE_Z + 0.02),
                    rotation=(pitch, roll, yaw),
                )

                # Mild scale jitter (subtle ambiguity)
                s = random.uniform(0.92, 1.06)
                rep.modify.scale(input_prims=[prim], scale=(s, s, s))


            # Occluder moves: sometimes blocks part of the scene
            if random.random() < 0.65:
                ox = random.uniform(-0.10, 0.10)
                oy = random.uniform(-0.25, 0.25)
                rep.modify.pose(input_prims=[occluder], position=(ox, oy, 0.10))
                rep.modify.visibility(input_prims=[occluder], value=True)
            else:
                rep.modify.visibility(input_prims=[occluder], value=False)


            # -----------------------------
            # Photometric randomization
            # -----------------------------
            # Intensities
            rep.modify.attribute(input_prims=[key], attributes={"intensity": random.uniform(1200, 7000)})
            rep.modify.attribute(input_prims=[fill], attributes={"intensity": random.uniform(200, 2500)})
            rep.modify.attribute(input_prims=[rim],  attributes={"intensity": random.uniform(0, 1500)})

            # Directions (rotate distant lights)
            rep.modify.pose(input_prims=[key],  rotation=(random.uniform(-70,-20), random.uniform(-40,40), 0))
            rep.modify.pose(input_prims=[fill], rotation=(random.uniform(-70,-20), random.uniform(120,220), 0))
            rep.modify.pose(input_prims=[rim],  rotation=(random.uniform(-80,-10), random.uniform(-180,180), 0))
            rep.modify.attribute(input_prims=[key], attributes={"color": (random.uniform(0.9,1.0), random.uniform(0.85,1.0), random.uniform(0.8,1.0))})




            # -----------------------------
            # Clutter randomization
            # -----------------------------
            num_clutter = random.randint(0, 5)  # 0-5 clutter items this frame
            active = set(random.sample(range(len(clutter_prims)), num_clutter))

            for i, prim in enumerate(clutter_prims):
                if i in active:
                    # scatter around tabletop center
                    x = random.uniform(-0.30, 0.30)
                    y = random.uniform(-0.30, 0.30)
                    z = random.uniform(0.03, 0.10)

                    rep.modify.pose(
                        input_prims=[prim],
                        position=(x, y, z),
                        rotation=(random.uniform(0,180), random.uniform(0,180), random.uniform(0,180)),
                    )
                    rep.modify.scale(input_prims=[prim], scale=random.uniform(0.6, 1.4))
                    rep.modify.visibility(input_prims=[prim], value=True)
                else:
                    rep.modify.visibility(input_prims=[prim], value=False)



            # -----------------------------
            # Occluder randomization
            # -----------------------------
            num_active = random.randint(0, len(occluders))

            active_ids = set(random.sample(range(len(occluders)), num_active))

            for i, occ in enumerate(occluders):
                if i in active_ids:
                    ox = random.uniform(-0.15, 0.15)
                    oy = random.uniform(-0.30, 0.30)
                    oz = random.uniform(0.10, 0.20)

                    rep.modify.pose(
                        input_prims=[occ],
                        position=(ox, oy, oz),
                        rotation=(0, random.uniform(0, 180), 0),
                    )
                    rep.modify.visibility(input_prims=[occ], value=True)
                else:
                    rep.modify.visibility(input_prims=[occ], value=False)



            # Camera random orbit
            radius = random.uniform(*CAM_RADIUS_RANGE)
            theta = random.uniform(0, 360)
            cam_x = radius * cos_deg(theta)
            cam_y = radius * sin_deg(theta)
            cam_z = random.uniform(*CAM_HEIGHT_RANGE)

            look_at_z = random.uniform(*CAM_LOOK_AT_Z_RANGE)
            rep.modify.pose(
                input_prims=[camera],
                position=(cam_x, cam_y, cam_z),
                look_at=(0.0, 0.0, look_at_z),
            )

            # Light jitter
            # 
            # rep.modify.attribute(
                # input_prims=[key],
                # attributes={"intensity": random.uniform(*KEY_LIGHT_INTENSITY)}
            # )

            # rep.modify.attribute(
                # input_prims=[fill],
                # attributes={"intensity": random.uniform(*FILL_LIGHT_INTENSITY)}
            # )

            # Light directions a bit
            # rep.modify.pose(input_prims=[key],
                            # rotation=(random.uniform(25, 70), random.uniform(-60, 10), 0))
            # rep.modify.pose(input_prims=[fill],
                            # rotation=(random.uniform(40, 85), random.uniform(10, 80), 0))

        

        # Register randomizer
        rep.randomizer.register(randomize_frame)

        # Trigger: run per frame
        with rep.trigger.on_frame(max_execs=NUM_FRAMES):
            rep.randomizer.randomize_frame()

    log("Starting orchestration…")
    for _ in range(30):
        simulation_app.update()

    rep.orchestrator.run()

    log("Done. Flushing writer...")
    for _ in range(240):
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()


