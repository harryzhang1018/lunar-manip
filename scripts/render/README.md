# Rendering the demos with Blender

The Irrlicht window is fine for watching a run, but its frames (`--save-frames`)
are not presentable. This pipeline renders the same run with Blender/Cycles
instead, in two halves that are independent of each other:

1. **Export (Chrono side).** The scenario writes a Chrono::Postprocess Blender
   scene while it simulates: `exported.assets.py` (every mesh and material, once)
   plus `output/stateNNNNN.py/.dat` (every body's pose) per exported frame.
   `scenarios/blender_export.py` wraps `pychrono.postprocess.ChBlender` with the
   frame cadence and the mid-run body pickup; the scenario just calls it once per
   step. No render window is involved, so it runs `--headless`.
2. **Render (Blender side).** `render_orbitbuilder.py` runs *inside* Blender: it
   imports the export with Chrono's `chrono_import` add-on, restyles every body
   by name (hull, tracks, arm, rocks, terrain, orbit markers), places a camera and
   sun, and renders with Cycles on the GPU -- one frame, or the whole sequence.

## Export

```bash
conda run --no-capture-output -n chrono python scenarios/TrackedVeh_OrbitBuilder.py \
    --headless --export-blender artifacts/blender/orbitbuilder_s1 \
    --state-dir artifacts/states/orbitbuilder_s1
```

`--export-blender [DIR]` (default `artifacts/blender/trackedveh_orbitbuilder`)
writes `BLENDER_FPS` = 7.5 frames per simulated second (`--blender-fps N` to
change), matching the Irrlicht movie recipe (30/s at a stride of 4), so the
default 135 s run is ~1000 frames. It composes with everything else --
`--continue`, `--run-time`, `--no-wall`, a render window -- so a quick look is

```bash
# 3 s of the two-section site (mode 2), 23 frames, ~20 s wall time
conda run --no-capture-output -n chrono python scenarios/TrackedVeh_OrbitBuilder.py \
    --headless --run-time 3 --no-state \
    --continue artifacts/states/orbitbuilder_newrock_s2/rock_state.csv \
    --export-blender artifacts/blender/orbitbuilder_s3_preview
```

The export directory ends up with:

| file | what |
|---|---|
| `exported.assets.py` | meshes + materials, shared by every frame (~28 MB: the M113 has a mesh per track shoe) |
| `output/stateNNNNN.py` | one per frame: every body's pose, referencing the assets by id |
| `blender_export_summary.json` | frame count, fps and `frame_times` (frame index -> sim time) |

Bodies that appear mid-run (the wall dumps, mode 2's rock load) are picked up
automatically: the exporter re-registers every body before each frame and appends
any new shape to the assets file, which Blender loads in full at import.

## Render

```bash
scripts/render/render.sh --export-dir artifacts/blender/orbitbuilder_s3_preview --frame 15
scripts/render/render.sh --export-dir artifacts/blender/orbitbuilder_s1 --time 60 \
    --output artifacts/renders/s1_t60.png --samples 256
scripts/render/render.sh --export-dir artifacts/blender/orbitbuilder_s1 --animate \
    --anim-dir artifacts/renders/orbitbuilder_s1/anim
ffmpeg -framerate 30 -i artifacts/renders/orbitbuilder_s1/anim/frame%04d.png -pix_fmt yuv420p s1.mp4
```

`render.sh` is `blender --background --factory-startup --python
render_orbitbuilder.py -- ARGS`; every argument goes to the script (`--help` lists
them). Defaults: 1920x1080, 128 samples, Cycles on OptiX (the RTX 4090; ~2-3 s a
frame for the 4.7k-rock site, plus ~6 s to import), output
`<export-dir>/renders/frameNNNN.png`. `--time T` picks the frame nearest a sim
time. `--animate` renders `[--anim-start, --anim-end]` in one Blender session
(`--skip-existing` resumes).

The camera is anchored on the M113 hull and offset in the hull's frame
(`--cam-back/--cam-side/--cam-height`, aim `--look-ahead/--look-side/--look-height`;
negative side = the builder's right, the rock-pile side), so the default -- a
raised view from the pile side across the hull onto the wall -- frames the
builder at any station of the orbit. `--cam-pos X Y Z --look-at X Y Z` sets it in
world coordinates instead. Look controls: `--sun-elevation/--sun-azimuth/
--sun-strength`, `--sky-strength`, `--bg-color`, `--exposure`, `--view-transform`,
`--hide-markers`, `--hide-terrain`.

### Requirements

- **Blender >= 4.1** with Chrono's `chrono_import.py` add-on installed under
  `~/.config/blender/<ver>/scripts/addons/` and enabled once in the UI. On this
  machine that is `~/blender-5.1.2-linux-x64/blender` (the default in
  `render.sh`; `BLENDER=... render.sh` overrides). Blender 4.0 fails at import:
  the pychrono 10 exporter emits `bpy.ops.object.shade_auto_smooth` for its
  cylinder primitives, an operator added in 4.1. The add-on source lives in the
  Chrono tree at `src/importer_blender/for_blender_X/chrono_import.py`.
- The `chrono` conda env for the export (see the repo README).

### How the render script works, and why it is shaped like this

- The add-on builds the scene one frame at a time: `scene.frame_set(f)` runs its
  handler, which deletes and re-creates every object in `chrono_frame_objects`
  from `stateNNNNN.py`. So styling has to be re-applied after every frame change,
  and the handler is removed around each `render()` call (rendering fires it too)
  and put back afterwards.
- Materials are assigned on the *mesh data*, which the add-on shares between all
  bodies drawing the same Chrono visual shape (all rocks of one of the 12
  variants, say). Thousands of rocks therefore cost a dozen assignments, and the
  per-object material link the add-on sets is switched back to the mesh's.
- Bodies are classified by their Chrono name (the add-on parents each shape to an
  empty named after the body): `Chassis body` -> hull; `*_shoe` -> track;
  wheels/idlers/sprockets/suspension -> dark metal; `base-1 .. wrist-1` -> arm;
  `endeffector-1`, `finger-*` -> gripper; `rock_*`, `wall_*`, `s<N>_*` -> rock;
  `patch_*` -> ground; `orbit_markers`, `place_markers*` -> the plan markers
  (their ring is told apart by the imported colour). `--debug` prints the groups.
- The rock material tints each object from `Object Info > Random`, so a wall of a
  dozen shared meshes does not read as tiling; the ground is a procedural
  regolith (large-scale mottling + fine bump), replacing the export's stretched
  dirt tile.

Adapted from the NeDM project's `blender-render/render_tracked_arm_rollout.py`
and `src/nedm/blender_export.py`.
