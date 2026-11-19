"""Construct tiny mesh fixtures for the real Blender converter smoke.

blender -b --factory-startup --python-exit-code 2 --python tests/blender_fixture.py -- /tmp/sigma-blender-fixture
The fixture contains constructed ground and vehicle meshes with one frozen state.
"""
from pathlib import Path
import sys
import bpy
import yaml

output = Path(sys.argv[sys.argv.index("--") + 1]).resolve()
output.mkdir(parents=True, exist_ok=True)
assets = output / "assets/vehicle"
assets.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.mesh.primitive_cube_add(size=1)
car = bpy.context.object
car.name = "vehicle_tesla_fixture"
car.dimensions = [4.5, 1.8, 1.5]
car.location.z = .75
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
bpy.ops.export_scene.gltf(filepath=str(assets / "tesla.glb"), export_format="GLB", use_selection=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.mesh.primitive_cube_add(size=1)
ground = bpy.context.object
ground.name = "Ground_itu_concrete"
ground.dimensions = [40, 40, .2]
ground.location.z = -.1
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
material = bpy.data.materials.new("itu_concrete")
ground.data.materials.append(material)
bpy.ops.wm.save_as_mainfile(filepath=str(output / "site.blend"))
world = {"site_id": "blender_fixture", "trajectory_id": 1, "frame_idx": 0,
         "world_frame": 25, "time_s": 1.25,
         "nodes": {
             "rsu": {"rsu_001": {"node_id": "rsu_001", "index": 0, "location": [0, 0, 4], "rotation": {}}},
             "cav": {"cav_001": {"node_id": "cav_001", "index": 0, "actor_id": 101,
                       "bp_id": "vehicle.tesla.model3", "location": [10, 0, .2], "rotation": {}}}},
         "scene": {"vehicles": {101: {"id": 101, "bp_id": "vehicle.tesla.model3", "location": [10, 0, .2],
                     "angle": [0, 0, 0], "center": [0, 0, .75], "extent": [2.25, .9, .75],
                     "velocity": [0, 0, 0], "speed": 0}}}}
state = output / "dataset/stage1_multimodal/000001/world_state"
state.mkdir(parents=True, exist_ok=True)
(state / "world_state_000000.yaml").write_text(yaml.safe_dump(world))
config = {"schema_version": 1, "site_id": "blender_fixture", "stage3": {
    "blender": {"static_scene": "site.blend", "actor_assets": "assets", "crop": False},
    "radio": {"enabled": False}, "radar": {"enabled": False}}}
(output / "config.yaml").write_text(yaml.safe_dump(config))
print(output / "config.yaml")
