"""Build the whole 15-rank AMD-UW site into ONE .blend, using the dataset's
own tools/blender_import.py (not our converter) -- its CLI only takes one
--rank at a time, but its build() function is directly importable, and it
never clears the scene between calls, so calling it once per rank in the
same empty scene merges them all naturally.

    blender --background --python build_all_ranks_reference.py -- <out.blend> [--end SECONDS]

Edit DATASET/MESHES below to point at wherever amd-uw-run16 is unzipped.
"""
import glob
import os
import re
import sys

DATASET = os.path.expanduser("~/Downloads/amdZip/amd-uw-run16/data")
MESHES = os.path.expanduser("~/Downloads/amdZip/amd-uw-run16/meshes")
TOOLS_DIR = os.path.expanduser("~/Downloads/amdZip/amd-uw-run16/tools")

sys.path.insert(0, TOOLS_DIR)
import blender_import as ref  # the dataset's own tools/blender_import.py

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
out_path = argv[0]
end_time = None
if "--end" in argv:
    end_time = float(argv[argv.index("--end") + 1])

bpy.ops.wm.read_factory_settings(use_empty=True)

# blender_import.py has no rank-discovery helper of its own (that's in
# read_trajectory.py, which this script doesn't need) -- same glob pattern.
ranks = sorted(
    int(re.search(r"rank_(\d+)_frames\.bin$", p).group(1))
    for p in glob.glob(os.path.join(DATASET, "rank_*_frames.bin")))
print(f"found ranks: {ranks}")

scene = bpy.context.scene
max_frame_end = 1
for rank in ranks:
    print(f"building rank {rank} ...")
    ref.build(DATASET, rank, out_path=None, mesh_dir=MESHES, end=end_time)
    # build() sets scene.frame_end for THIS rank alone each time it's
    # called, so the last call would otherwise leave the scene's playback
    # range reflecting only the last (typically shortest, since ranks vary
    # in how many frames they recorded) rank instead of the true max.
    max_frame_end = max(max_frame_end, scene.frame_end)

scene.frame_start = 1
scene.frame_end = max_frame_end

bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out_path))
print(f"saved {out_path} ({len(ranks)} ranks, frames 1..{max_frame_end})")
