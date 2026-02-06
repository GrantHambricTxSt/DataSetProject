import os
import time
import random
import omni.replicator.core as rep
from omni.isaac.kit import SimulationApp

# ----------------------------
# CONFIG
# ----------------------------
NUM_FRAMES = 25
RESOLUTION = (1280, 720)

OUT_DIR = os.environ.get(
    "OUT_DIR",
    f"/workspace/output/ambiguous_mini_{time.strftime('%Y%m%d_%H%M%S')}"
)
os.makedirs(OUT_DIR, exist_ok=True)

print("[ambiguous_mini] OUT_DIR =", OUT_DIR, flush=True)

# ----------------------------
# START SIM
# ----------------------------
simulation_app = SimulationApp(
    {
        "headless": True,
        "renderer": "RayTracedLighting",
        "width": RESOLUTION[0],
        "height": RESOLUTION[1],
    }
)

# ----------------------------
# WRITER
# ----------------------------
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir=OUT_DIR,
    rgb=True,
)
writer.attach([])

# ----------------------------
# CAMERA
# ----------------------------
camera = rep.create.camera(
    position=(0, 1.5, 2.5),
    look_at=(0, 0.75, 0),
)

writer.attach([camera])

# ----------------------------
# LIGHTS (AMBIGUOUS)
# ----------------------------
def randomize_lighting():
    rep.create.light(
        light_type=random.choice(["sphere", "disk", "rect"]),
        intensity=random.uniform(200, 1500),
        position=(
            random.uniform(-2, 2),
            random.uniform(1.0, 3.0),
            random.uniform(-2, 2),
        ),
    )

# ----------------------------
# PROPS (AMBIGUOUS)
# ----------------------------
def randomize_scene():
    for _ in range(random.randint(2, 5)):
        rep.create.cube(
            position=(
                random.uniform(-0.5, 0.5),
                random.uniform(0.0, 0.8),
                random.uniform(-0.5, 0.5),
            ),
            scale=random.uniform(0.05, 0.25),
        )

# ----------------------------
# BUILD GRAPH (ONCE)
# ----------------------------
with rep.new_layer():
    randomize_scene()
    randomize_lighting()

rep.orchestrator.run()   # IMPORTANT: no frame count here

print("STARTING FRAME LOOP", flush=True)

# ----------------------------
# FRAME LOOP (THIS IS THE KEY)
# ----------------------------
for frame in range(NUM_FRAMES):
    print(f"Frame {frame}", flush=True)

    with rep.new_layer():
        randomize_scene()
        randomize_lighting()

    simulation_app.update()
    time.sleep(0.05)

print("REPLICATOR FINISHED", flush=True)

simulation_app.close()
