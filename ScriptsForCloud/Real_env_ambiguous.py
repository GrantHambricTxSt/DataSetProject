from omni.isaac.kit import SimulationApp
import carb
import os
import omni.usd
import omni.replicator.core as rep

simulation_app = SimulationApp({"headless": True})

SEED = 7

NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "60"))     # scale to 2k+ later
RESOLUTION = (int(os.environ.get("W", "960")), int(os.environ.get("H", "540")))  # scale to 1920x1080 later

OUTPUT_DIR = os.environ.get("OUT_DIR", "/workspace/output/real_evn_ambiguous")

MIN_OBJECTS = 2
MAX_OBJECTS = 3

ASSET_URLS = [
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/Mugs/SM_Mug_A2.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
]

WAREHOUSE_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"

try:
    omni.usd.get_context().open_stage(WAREHOUSE_PATH)
except Exception as e:
    print(f"[ERROR] Could not load warehouse: {e}")
    rep.create.plane(scale=(10,10,1), visible=True) ## Fallback to a plane ( floor )

rep.set_global_seed(SEED)

def main():
    with rep.new_layer():
        
        TABLE_HEIGHT = 0.75
        TABLE_SURFACE = TABLE_HEIGHT
        
        table = rep.create.cube(
            position=(0, 0, TABLE_HEIGHT / 2),
            scale=(0.5, 0.5, TABLE_HEIGHT),
            visible=True,
            ## semantics=[("class", "table")] If table needs to be detected as well, remove the comment
        )

        occluder = rep.create.cube(
            position=(0.0, 0.0, TABLE_SURFACE + 0.05),
            scale=(0.12, 0.40, 0.20),
            visible=True,
        )
        
        ## This is just a center pivot for the camera
        camera_rig = rep.create.xform(position=(0, 0, 0), name="CamRig")
        
        camera = rep.create.camera( parent = camera_rig, name = "Camera")
        
        render_product = rep.create.render_product(camera, resolution = RESOLUTION)
        
        key_light = rep.create.light(light_type="distant", intensity=600, rotation=(45, -30, 0))
        fill_light = rep.create.light(light_type="distant", intensity=150, rotation=(65, 40, 0))
        
        with table:
            rep.physics.collider()
    
        assets = []
        
        for url in ASSET_URLS:
            obj = rep.create.from_usd(url)
            class_label = url.split("/")[-1].replace(".usd", "")
            
            with obj:
                rep.modify.semantics([("class", class_label)])
                
                ## Move the obj under the cam so that it is not seen by the camera
                rep.modify.pose(position=(0, 0, -500))
            
            assets.append(obj)
    
    def randomize_scene():
        
        instances = rep.randomizer.instantiate(
            paths=assets,
            size=rep.distribution.choice(list(range(MIN_OBJECTS, MAX_OBJECTS + 1))),
            mode='scene_instance' 
        )
        
        with instances:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.3, -0.3, TABLE_SURFACE + 0.005), (0.3, 0.3, TABLE_SURFACE + 0.01)),
                rotation=rep.distribution.uniform((-6, -6, 0), (6, 6, 360)),
                scale=rep.distribution.uniform(0.92, 1.06)
            )
        
        with occluder:
            rep.modify.pose(position=rep.distribution.uniform((-0.1, -0.25, TABLE_SURFACE + 0.005), (0.1, 0.25, TABLE_SURFACE + 0.01)))
            rep.modify.visibility(rep.distribution.choice([True, False], weights=[0.65, 0.35]))
            
        with camera_rig:
            rep.modify.pose(rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)))
        
        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform((0.55, 0, 0.25), (0.85, 0, 0.55)),
                look_at=(0,0,TABLE_SURFACE)
            )
        
        with key_light:
            rep.modify.attribute("intensity", rep.distribution.uniform(250, 900))
            rep.modify.pose(rotation=rep.distribution.uniform((25, -60, 0), (70, 10, 0)))
        
        with fill_light:
            rep.modify.attribute("intensity", rep.distribution.uniform(50, 300))
        
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=OUTPUT_DIR, rgb=True, bounding_box_2d_tight=True)
    writer.attach([render_product])

    rep.randomizer.register(randomize_scene)
    
    with rep.trigger.on_frame(num_frames=NUM_FRAMES):
        rep.randomizer.randomize_scene()

    rep.orchestrator.run()
    simulation_app.close()

if __name__ == "__main__":
    main()