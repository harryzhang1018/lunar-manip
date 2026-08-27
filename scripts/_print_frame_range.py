"""Print a .blend file's baked frame range, for render_logical.py to probe.

Usage (loads the .blend as the background scene, then runs this against it):
    blender --background <blend_file> --python scripts/_print_frame_range.py
"""

import bpy

print(f"FRAME_RANGE {bpy.context.scene.frame_start} {bpy.context.scene.frame_end}")
