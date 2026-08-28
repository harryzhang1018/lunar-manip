

## Rendering (recommended): the combined logical timeline

Use [`scripts/render_logical.py`](../scripts/render_logical.py). 

```
scripts/render_logical.py [--section1|--section2] [--overview|--chaseCollector|--chaseBuilder|--combined] <start> <end> <output.mp4> [engine [samples]]
```

`[samples]` overrides the render sample count (only valid together with
`[engine]`); leave it off to use whatever's already baked into the .blend
file.

`--section2` (the default, kept for backward compatibility) is the combined
`section2_mode1.blend` -> `section2_mode2.blend` timeline described below.
`--section1` renders `data/section1.blend` alone, with just its first 20
frames trimmed off (it's a single file, so there's no mode1/mode2 cut or
tail trim to worry about).

`data/section1.blend` has three cameras baked in (see
`tools/render_site_anim.py` in the AMD-UW pipeline that produced it): the
overhead `cam_overview`, and two vehicle-following chase cams,
`cam_chase_a_r1_collector` and `cam_chase_b_r8_builder`. Pick which one
renders with `--overview`, `--chaseCollector`, or `--chaseBuilder`, or use
`--combined` (the default whenever `--section1` is used and none of these
is given explicitly) for overview and chaseCollector side by side, overview
on the left. `--chaseBuilder` is standalone-only -- it never appears in the
split screen. These flags only apply to `--section1`; `--section2`'s files
each have one fixed camera already, so passing a camera flag there fails
loudly instead of silently doing nothing.

Examples:

```
# A clip that crosses the mode1 -> mode2 cut -- the script figures out
# which frames come from which file and joins them for you.
scripts/render_logical.py --section2 3600 4200 data/builder/clip.mp4

# A clip from section1, overview | chaseCollector side by side (the default or can use --combined)
scripts/render_logical.py --section1 100 105 data/builder/s1_clip.mp4

# Just the overhead camera
scripts/render_logical.py --section1 --overview 100 105 data/builder/s1_overview.mp4

# Just the collector chase camera
scripts/render_logical.py --section1 --chaseCollector 100 105 data/builder/s1_chase.mp4

# Just the builder chase camera (never part of --combined)
scripts/render_logical.py --section1 --chaseBuilder 100 105 data/builder/s1_builder.mp4

# A single frame, as a one-frame video
scripts/render_logical.py 100 100 data/builder/frame100.mp4

# A single frame, as a still PNG instead -- give it a .png output and the
# same start/end frame
scripts/render_logical.py 4200 4200 data/builder/frame100.png

# The whole trimmed, combined video (an end past the real last logical
# frame is clamped automatically)
scripts/render_logical.py 1 999999 data/builder/full.mp4

# Force Cycles instead of the default EEVEE
scripts/render_logical.py 1 999999 data/builder/full.mp4 CYCLES

# Force Cycles with 64 samples instead of whatever's baked into the file
scripts/render_logical.py 1 999999 data/builder/full.mp4 CYCLES 64

# Keep EEVEE (the default engine) but override its samples to 32 -- engine
# still has to be named explicitly since [samples] only applies alongside
# an explicit [engine]
scripts/render_logical.py 1 999999 data/builder/full.mp4 BLENDER_EEVEE 32
```

The script prints the combined timeline's real total length (from each
file's own baked frame range, probed and cached in
`data/.section_frame_range_cache.json`) on every run, so you know the valid
`<end>` if you don't already.


`<output_prefix>` is passed to Blender as `//<output_prefix>` -- Blender's
`//` means "relative to the .blend file's own location", not your current
shell directory -- so the first example above writes
`data/builder/clip.mp4


everything below this is unnecessary for use but here for documentation

the above code hides both the two-file split and each file's unusable start/end frames --
`section2_mode1.blend`'s first 12 frames and last 200 frames, and
`section2_mode2.blend`'s first 15 frames and last 50 frames, are trimmed
off automatically -- so you just give it a frame range on one continuous
"logical" timeline (1-based; logical frame 1 is mode1's first usable
frame) and get back one joined video. It never leaves the untrimmed or
per-section clips behind -- only the final requested output file.

Every Blender invocation in this pipeline (render_logical.py, render_section.sh)
passes `--factory-startup`, which skips the user's installed add-ons --
a Poliigon add-on background thread was observed segfaulting Blender during
a `--background` render, and nothing rendered here depends on any add-on
being active.

# Rendered section .blend files

`section2_mode1.blend` and `section2_mode2.blend` are baked-animation
Blender files produced by
[`scripts/blend_from_chrono_export.py`](../scripts/blend_from_chrono_export.py)
from a two-section `TrackedVeh_OrbitBuilder.py --two-sections`-style Chrono
run (see that script's module docstring for the full export -> convert
workflow). They are two separate files for technical reasons only -- each
section is its own Chrono system / export, so each gets its own .blend --
but they represent **one continuous video**: mode2 continues right where
mode1 leaves off. Treat them as one logical unit; render with
[`scripts/render_logical.py`](../scripts/render_logical.py) below rather
than driving each file by hand.

Tracked via Git LFS (`data/*.blend`, see [`.gitattributes`](../.gitattributes))
since each file is 90-130MB. Run `git lfs install` once per machine before
cloning/pulling this repo, or you'll only get small LFS pointer files
instead of the real `.blend` content.

Both files' textures live in `data/textures/` (also Git LFS-tracked) and
are referenced by relative path (`//textures/<file>`), not packed into the
`.blend`s or dependent on any external asset folder on your machine (e.g.
Poliigon, a Downloads folder) -- so as long as `data/textures/` comes along
with the repo, they render correctly anywhere. If you ever regenerate these
files from a fresh Chrono export and the textures come back broken/external
again, see [`scripts/localize_blend_textures.py`](../scripts/localize_blend_textures.py).
## Rendering a single file's raw frame range (lower-level)

If you need to bypass the logical-timeline trimming/joining -- e.g. to
inspect a trimmed-off frame, or render straight from one file -- use
[`scripts/render_section.sh`](../scripts/render_section.sh) directly with
that file's own real frame numbers:

```
scripts/render_section.sh <blend_file> <start_frame> <end_frame> <output_prefix> [engine]
```

```
scripts/render_section.sh data/section2_mode1.blend 12 3810 builder/builder1
scripts/render_section.sh data/section2_mode2.blend 15 5000 builder/builder2

# A single frame -- start and end the same
scripts/render_section.sh data/section2_mode1.blend 500 500 builder/frame500
```


To join clips rendered this way yourself, see
[`scripts/concat_videos.sh`](../scripts/concat_videos.sh) (ffmpeg concat
demuxer, stream copy -- all input clips must share codec/resolution/
framerate, which they will if rendered with the same `[engine]`).
