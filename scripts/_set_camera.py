"""Switch the active scene camera, for render_logical.py's
--overview/--chase/--combined camera selection.

Reads the target camera's object name from the RENDER_LOGICAL_CAMERA env
var; a no-op if that's unset (the normal case, when no camera override was
requested), so this is safe to always include in the render command.

Usage (loads the .blend as the background scene, then runs this against it):
    RENDER_LOGICAL_CAMERA=cam_overview blender --background <blend_file> \
        --python scripts/_set_camera.py
"""

import os

import bpy

# Optional: enable GPU rendering for Cycles under --factory-startup (which
# resets preferences, so no compute device is enabled and Cycles silently
# falls back to CPU). Set RENDER_LOGICAL_CYCLES_DEVICE=OPTIX (or CUDA) to
# enable those devices and switch the scene to GPU compute.
dev = os.environ.get("RENDER_LOGICAL_CYCLES_DEVICE")
if dev:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = dev
    prefs.get_devices()
    n = 0
    for d in prefs.devices:
        d.use = d.type != "CPU"
        n += d.use
    bpy.context.scene.cycles.device = "GPU"
    print(f"_set_camera.py: Cycles -> GPU ({dev}), {n} device(s) enabled")

name = os.environ.get("RENDER_LOGICAL_CAMERA")
if name:
    cam = bpy.data.objects.get(name)
    if cam is None or cam.type != "CAMERA":
        raise SystemExit(
            f"camera '{name}' not found in {bpy.data.filepath} -- "
            "this .blend may not support --overview/--chase/--combined")
    bpy.context.scene.camera = cam
