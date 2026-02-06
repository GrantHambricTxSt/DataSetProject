# ambiguous_tabletop_2023_1_1_WORKING.py
# Isaac Sim 2023.1.1 - Headless Replicator dataset with strong ambiguity
# Outputs: rgb + bounding_box_2d_tight
#
# Run:
#   cd /workspace/DataSetProject/ScriptsForCloud
#   NUM_FRAMES=80 W=960 H=540 \
#   /isaac-sim/python.sh ambiguous_tabletop_2023_1_1_WORKING.py --no-window --/app/window/enabled=false
#
# Ambiguity knobs (env):
#   OCCLUSION_P=0.70
#   WALL_OCCLUSION_P=0.90
#   LOW_CONTRAST_P=0.60
#   CLUTTER_MIN=8 CLUTTER_MAX=18
#   CAM_LOWANGLE_P=0.55
#   LIGHT_JITTER=1
#   SIMILAR_SHAPES_P=0.65
#   OUT_DIR=/workspace/output/my_run_name

import os, sys, time, math, random, glob
from datetime import datetime

# --------------------------
# READ ENV FIRST
# --------------------------
SEED = int(os.environ.get("SEED", "7"))
random.seed(SEED)

NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "80"))
W = int(os.environ.get("W", "960"))
H = int(os.environ.get("H", "540"))
RES = (W, H)

OUT_DIR = os.environ.get(
    "OUT_DIR",
    f"/workspace/output/ambig_tabletop_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
)
os.makedirs(OUT_DIR, exist_ok=True)

OCCLUSION_P = float(os.environ.get("OCCLUSION_P", "0.70"))
WALL_OCCLUSION_P = float(os.environ.get("WALL_OCCLUSION_P", "0.90"))
LOW_CONTRAST_P = float(os.environ.get("LOW_CONTRAST_P", "0.60"))
CLUTTER_MIN = int(os.environ.get("CLUTTER_MIN", "8"))
CLUTTER_MAX = int(os.environ.get("CLUTTER_MAX", "18"))
CAM_LOWANGLE_P = float(os.environ.get("CAM_LOWANGLE_P", "0.55"))
LIGHT_JITTER = int(os.environ.get("LIGHT_JITTER", "1"))
SIMILAR_SHAPES_P = float(os.environ.get("SIMILAR_SHAPES_P", "0.65"))

TABLE_X = float(os.environ.get("TABLE_X", "0.95"))
TABLE_Y = float(os.environ.get("TABLE_Y", "0.95"))
TABLE_Z = float(os.environ.get("TABLE_Z", "0.0"))

def clamp(x, a, b):
    return max(a, min(b, x))

def cosd(d): return math.cos(math.radians(d))
def sind(d): return math.sin(math.radians(d))

def log(msg):
    print(f"[AMBIG] {msg}", flush=True)

# --------------------------
# SIM APP (2023.1.1 SAFE)
# --------------------------
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RayTracedLighting",
    "width": W,
    "height": H,
    "anti_aliasing": 4,
    "extra_args": [
        "--enable", "omni.replicator.core",
        "--/app/window/enabled=false",
        "--/renderer/multiGpu/enabled=false",
        "--/rtx/raytracing/enabled=true",
    ],
})

import carb
import omni.replicator.core as rep

# --------------------------
# BUILD GRAPH
# --------------------------
with rep.new_layer():
    # --- Environment: table + backboard
    table = rep.create.plane(
        position=(0, 0, TABLE_Z),
        rotation=(0, 0, 0),
        scale=(TABLE_X, TABLE_Y, 1.0),
        visible=True
    )

    backboard = rep.create.cube(
        position=(0.0, -0.60, 0.40),
        rotation=(80, 0, 0),
        scale=(1.8, 1.8, 0.03),
        visible=True
    )

    # --- Targets (primitive cylinders; semantics = class labels)
    mug = rep.create.cylinder(position=(0, 0, -10), scale=(0.07, 0.07, 0.11), visible=True)
    bottle = rep.create.cylinder(position=(0, 0, -10), scale=(0.055, 0.055, 0.18), visible=True)
    can = rep.create.cylinder(position=(0, 0, -10), scale=(0.065, 0.065, 0.12), visible=True)

    rep.modify.semantics([mug], semantics=[("class", "mug")])
    rep.modify.semantics([bottle], semantics=[("class", "bottle")])
    rep.modify.semantics([can], semantics=[("class", "can")])

    targets = [mug, bottle, can]

    # --- Clutter pool
    clutter_pool = []
    for _ in range(50):
        kind = random.choice(["cube", "sphere", "cylinder"])
        if kind == "sphere":
            prim = rep.create.sphere(position=(0, 0, -10), scale=random.uniform(0.015, 0.05), visible=False)
        elif kind == "cylinder":
            prim = rep.create.cylinder(
                position=(0, 0, -10),
                scale=(random.uniform(0.015, 0.05), random.uniform(0.015, 0.05), random.uniform(0.03, 0.16)),
                visible=False
            )
        else:
            s = random.uniform(0.02, 0.07)
            prim = rep.create.cube(position=(0, 0, -10), scale=(s, s, s), visible=False)
        rep.modify.semantics([prim], semantics=[("class", "clutter")])
        clutter_pool.append(prim)

    # --- Occluders: bars + moving wall
    bars = []
    for _ in range(6):
        b = rep.create.cube(position=(0, 0, 0.2), scale=(0.05, 0.75, 0.35), visible=False)
        bars.append(b)

    wall = rep.create.cube(position=(0.0, 0.10, 0.13), scale=(0.20, 0.95, 0.28), visible=True)

    # --- Camera + render product (EXPLICIT)
    camera = rep.create.camera(position=(0.6, 0.6, 0.5), look_at=(0, 0, 0.08))
    render_product = rep.create.render_product(camera, RES)

    # --- Lighting
    dome = rep.create.light(light_type="dome", intensity=9000)
    key = rep.create.light(light_type="distant", intensity=2400)
    fill = rep.create.light(light_type="distant", intensity=900)
    rim = rep.create.light(light_type="distant", intensity=650)

    # --- Writer
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=OUT_DIR,
        rgb=True,
        bounding_box_2d_tight=True,
    )
    writer.attach([render_product])

    # --------------------------
    # RANDOMIZER (per-frame)
    # --------------------------
    def apply_material(prim, color, rough=None, metal=None):
        rep.modify.material(
            prim,
            diffuse=color,
            roughness=rough if rough is not None else random.uniform(0.2, 0.95),
            metallic=metal if metal is not None else random.uniform(0.0, 0.25),
        )

    def randomize_frame():
        # global ambiguity mode
        low_contrast = (random.random() < LOW_CONTRAST_P)

        # table/backboard near-white, but sometimes slightly separated
        if low_contrast:
            base = random.uniform(0.82, 0.94)
            t = (base, base, base)
            b = (clamp(base + random.uniform(-0.03, 0.03), 0.78, 0.97),) * 3
        else:
            t = (random.uniform(0.65, 0.90),) * 3
            b = (random.uniform(0.50, 0.82),) * 3

        apply_material(table, t, rough=random.uniform(0.4, 0.98), metal=0.0)
        apply_material(backboard, b, rough=random.uniform(0.4, 0.98), metal=0.0)

        # shapes: sometimes make mug/bottle/can similar
        if random.random() < SIMILAR_SHAPES_P:
            r = random.uniform(0.055, 0.072)
            h = random.uniform(0.10, 0.17)
            rep.modify.pose(mug, scale=(r, r, h))
            rep.modify.pose(bottle, scale=(r * random.uniform(0.95, 1.05), r * random.uniform(0.95, 1.05), h * random.uniform(0.95, 1.10)))
            rep.modify.pose(can, scale=(r * random.uniform(0.95, 1.05), r * random.uniform(0.95, 1.05), h * random.uniform(0.85, 1.05)))
        else:
            rep.modify.pose(mug, scale=(random.uniform(0.06, 0.08), random.uniform(0.06, 0.08), random.uniform(0.08, 0.13)))
            rep.modify.pose(bottle, scale=(random.uniform(0.045, 0.07), random.uniform(0.045, 0.07), random.uniform(0.14, 0.22)))
            rep.modify.pose(can, scale=(random.uniform(0.055, 0.08), random.uniform(0.055, 0.08), random.uniform(0.09, 0.15)))

        # target colors: near-table in low contrast
        def target_color():
            if low_contrast:
                g = random.uniform(0.78, 0.95)
                return (clamp(g + random.uniform(-0.03, 0.03), 0, 1),
                        clamp(g + random.uniform(-0.03, 0.03), 0, 1),
                        clamp(g + random.uniform(-0.03, 0.03), 0, 1))
            base = random.uniform(0.15, 0.85)
            return (clamp(base + random.uniform(-0.18, 0.18), 0, 1),
                    clamp(base + random.uniform(-0.18, 0.18), 0, 1),
                    clamp(base + random.uniform(-0.18, 0.18), 0, 1))

        for prim in targets:
            apply_material(prim, target_color(), rough=random.uniform(0.12, 0.95), metal=random.uniform(0.0, 0.35))

        # place targets close to induce overlap
        for prim in targets:
            x = random.uniform(-TABLE_X * 0.20, TABLE_X * 0.20)
            y = random.uniform(-TABLE_Y * 0.18, TABLE_Y * 0.22)
            z = TABLE_Z + random.uniform(0.03, 0.06)
            yaw = random.uniform(0, 360)
            pitch = random.uniform(-12, 12)
            roll = random.uniform(-12, 12)
            rep.modify.pose(prim, position=(x, y, z), rotation=(pitch, roll, yaw))

        # clutter
        k = random.randint(CLUTTER_MIN, CLUTTER_MAX)
        chosen = random.sample(clutter_pool, k)
        for prim in clutter_pool:
            rep.modify.visibility(prim, prim in chosen)

        for prim in chosen:
            cx = random.uniform(-TABLE_X * 0.45, TABLE_X * 0.45)
            cy = random.uniform(-TABLE_Y * 0.40, TABLE_Y * 0.48)
            cz = TABLE_Z + random.uniform(0.01, 0.08)
            rep.modify.pose(
                prim,
                position=(cx, cy, cz),
                rotation=(random.uniform(-25, 25), random.uniform(-25, 25), random.uniform(0, 360)),
            )
            if low_contrast:
                col = (random.uniform(0.72, 0.92),) * 3
            else:
                col = (random.uniform(0.10, 0.90), random.uniform(0.10, 0.90), random.uniform(0.10, 0.90))
            apply_material(prim, col, rough=random.uniform(0.25, 0.98), metal=random.uniform(0.0, 0.20))

        # bars
        for b in bars:
            rep.modify.visibility(b, random.random() < OCCLUSION_P)
            rep.modify.pose(
                b,
                position=(random.uniform(-0.25, 0.25), random.uniform(-0.05, 0.30), random.uniform(0.12, 0.28)),
                rotation=(0, 0, random.uniform(-28, 28)),
            )
            apply_material(b, (random.uniform(0.0, 0.12),) * 3, rough=random.uniform(0.25, 0.95), metal=0.0)

        # wall
        active = (random.random() < WALL_OCCLUSION_P)
        rep.modify.visibility(wall, active)
        if active:
            rep.modify.pose(
                wall,
                position=(random.uniform(-0.20, 0.20), random.uniform(-0.05, 0.32), random.uniform(0.10, 0.17)),
                rotation=(0, 0, random.uniform(-30, 30)),
            )
            apply_material(wall, (random.uniform(0.0, 0.10),) * 3, rough=random.uniform(0.25, 0.9), metal=0.0)

        # camera orbit + low angle chance
        r = random.uniform(0.55, 0.95)
        th = random.uniform(0, 360)
        cam_x = r * cosd(th)
        cam_y = r * sind(th)

        if random.random() < CAM_LOWANGLE_P:
            cam_z = random.uniform(0.14, 0.28)
            look_z = random.uniform(0.03, 0.10)
        else:
            cam_z = random.uniform(0.22, 0.70)
            look_z = random.uniform(0.05, 0.22)

        rep.modify.pose(camera, position=(cam_x, cam_y, cam_z),
                        look_at=(random.uniform(-0.03, 0.03), random.uniform(0.05, 0.14), look_z))

        # lights
        if LIGHT_JITTER:
            rep.modify.pose(key, rotation=(random.uniform(-15, 15), random.uniform(-15, 15), random.uniform(0, 360)))
            rep.modify.pose(fill, rotation=(random.uniform(-25, 25), random.uniform(-25, 25), random.uniform(0, 360)))
            rep.modify.pose(rim, rotation=(random.uniform(-25, 25), random.uniform(-25, 25), random.uniform(0, 360)))

    rep.randomizer.register(randomize_frame)

    with rep.trigger.on_frame(max_execs=NUM_FRAMES):
        rep.randomizer.randomize_frame()

# --------------------------
# RUN / FLUSH (KNOWN GOOD PATTERN)
# --------------------------
log(f"OUT_DIR={OUT_DIR}")
log(f"NUM_FRAMES={NUM_FRAMES} RES={RES} SEED={SEED}")

# warmup
for _ in range(80):
    simulation_app.update()

try:
    rep.orchestrator.reset()
except Exception:
    pass

print("STARTING REPLICATOR (step=1)", flush=True)
for i in range(NUM_FRAMES):
    rep.orchestrator.step(1)
    simulation_app.update()
    if i % 10 == 0:
        print(f"[AMBIG] stepped {i}/{NUM_FRAMES}", flush=True)

# flush frames so writer finishes
for _ in range(80):
    simulation_app.update()

rgb_dir = os.path.join(OUT_DIR, "rgb")
rgb_files = glob.glob(os.path.join(rgb_dir, "*.png"))
log(f"rgb_dir={rgb_dir} rgb_files_found={len(rgb_files)}")

if len(rgb_files) == 0:
    raise RuntimeError("No RGB images written. Render product did not render in headless mode.")

print("REPLICATOR FINISHED", flush=True)
print(f"[AMBIG] Wrote to: {OUT_DIR}", flush=True)

# avoid shutdown edge cases
os._exit(0)
