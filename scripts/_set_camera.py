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

name = os.environ.get("RENDER_LOGICAL_CAMERA")
if name:
    cam = bpy.data.objects.get(name)
    if cam is None or cam.type != "CAMERA":
        raise SystemExit(
            f"camera '{name}' not found in {bpy.data.filepath} -- "
            "this .blend may not support --overview/--chase/--combined")
    bpy.context.scene.camera = cam
