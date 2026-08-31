#!/usr/bin/env python3
"""render_logical.py -- render a frame range from the *logical* combined
video for one section, as if its underlying .blend file(s) were one
continuous timeline, even though --section2 is backed by two separate
.blend files for technical reasons (each is its own Chrono system / export,
see scripts/blend_from_chrono_export.py) rather than one merged file.

Each file also has some unusable animation at its very start (and, for
--section2, its end too -- motor spin-up, settling, etc.) that gets trimmed
off automatically -- see the SECTIONS table below. Logical frame 1 is the
first usable frame of the section's first file, and logical frame numbers
keep counting up with no gap across a mode1 -> mode2 cut. Callers never see
raw per-file frame numbers, trimmed-off frames, or the intermediate
per-file clips -- only the final joined output.

Usage:
    scripts/render_logical.py [--section1|--section2] \
        [--overview|--chaseCollector|--chaseBuilder|--combined] \
        <start> <end> <output.mp4> [engine [samples]]

--section1 renders data/section1.blend alone (its first 20 frames trimmed,
no tail trim). --section2 (the default, for backward compatibility) renders
the combined data/section2_mode1.blend -> data/section2_mode2.blend
timeline. <start>/<end> are logical frame numbers (1-based, inclusive). A
single still-frame render is just <start> == <end> -- give it a .png output
path to get a still image instead of a one-frame video container.

data/section1.blend has three cameras baked in (see
tools/render_site_anim.py in the AMD-UW render pipeline that produced it):
an overhead 'cam_overview' and two vehicle-following chase cams,
'cam_chase_a_r1_collector' (--chaseCollector) and 'cam_chase_b_r8_builder'
(--chaseBuilder). --combined (the default whenever --section1 is used and
none of these flags is given explicitly) renders overview and
chaseCollector and places them side by side, overview on the left,
chaseCollector on the right -- chaseBuilder is standalone-only
(--chaseBuilder) and never appears in the split screen. These flags only
apply to --section1 -- --section2's files each have a single fixed camera,
so passing one there fails loudly instead of silently doing nothing.

Examples:
    # A clip crossing the section2 mode1 -> mode2 cut
    scripts/render_logical.py --section2 3500 3700 data/builder/clip.mp4

    # A clip from section1, overview | chaseCollector side by side (the default)
    scripts/render_logical.py --section1 100 300 data/builder/s1_clip.mp4

    # Just the overhead camera
    scripts/render_logical.py --section1 --overview 100 300 data/builder/s1_overview.mp4

    # Just the collector chase camera
    scripts/render_logical.py --section1 --chaseCollector 100 300 data/builder/s1_chase.mp4

    # Just the builder chase camera (never part of --combined)
    scripts/render_logical.py --section1 --chaseBuilder 100 300 data/builder/s1_builder.mp4

    # One single frame, as a still image
    scripts/render_logical.py 100 100 data/builder/frame100.png

    # One single frame, as a one-frame video instead
    scripts/render_logical.py 100 100 data/builder/frame100.mp4

    # The whole trimmed, combined video (an end past the real last logical
    # frame is clamped, so a large round number like this works fine)
    scripts/render_logical.py 1 999999 data/builder/full.mp4

    # Force Cycles instead of the default EEVEE
    scripts/render_logical.py 1 999999 data/builder/full.mp4 CYCLES

    # Force Cycles with 64 samples instead of whatever's baked into the file
    scripts/render_logical.py 1 999999 data/builder/full.mp4 CYCLES 64
"""

import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(REPO_ROOT, "data")
PROBE_SCRIPT = os.path.join(SCRIPT_DIR, "_print_frame_range.py")
SET_CAMERA_SCRIPT = os.path.join(SCRIPT_DIR, "_set_camera.py")
CACHE_PATH = os.path.join(DATA_DIR, ".section_frame_range_cache.json")

# --overview/--chaseCollector/--chaseBuilder/--combined -> the object name of
# that camera in data/section1.blend (see tools/render_site_anim.py's
# `cameras` dict in the AMD-UW render pipeline this file came from). Only
# section1 has multiple cameras baked in -- section2's files each have one
# fixed camera already. --combined (in main()) only ever pairs overview
# with chaseCollector -- chaseBuilder is standalone-only.
CAMERA_NAMES = {
    "overview": "cam_overview",
    "chaseCollector": "cam_chase_a_r1_collector",
    "chaseBuilder": "cam_chase_b_r8_builder",
}
CAMERA_MODES = (*CAMERA_NAMES, "combined")

# `blender` in a non-interactive shell resolves to the old system 3.0.1, not
# the aliased 5.1.2 (the alias only lives in ~/.bashrc, loaded by
# interactive shells) -- so this defaults to the real 5.1.2 binary directly.
BLENDER = os.environ.get(
    "BLENDER",
    "/home/zacharyrichmond/Downloads/blender-5.1.2-linux-x64/blender-5.1.2-linux-x64/blender",
)

# section number -> [(filename in data/, frames to drop off the start,
# frames to drop off the end), ...], in logical playback order. See
# data/README_renders.md for what these trims are covering.
SECTIONS = {
    "1": [("section1.blend", 20, 0)],
    "2": [
        ("section2_mode1.blend", 12, 200),
        ("section2_mode2.blend", 193, 50),
    ],
    # section2 + background vehicles (scripts/add_section1_background.py);
    # identical timeline/trims, just different source files
    "2bg": [
        ("section2_mode1_bg.blend", 12, 200),
        ("section2_mode2_bg.blend", 193, 50),
    ],
}


def _load_cache():
    if os.path.exists(CACHE_PATH):
        try:
            with open(CACHE_PATH) as f:
                return json.load(f)
        except (OSError, ValueError):
            # e.g. a concurrent render job is mid-write -- just re-probe
            return {}
    return {}


def _save_cache(cache):
    with open(CACHE_PATH, "w") as f:
        json.dump(cache, f, indent=2)


def probe_frame_range(blend_path):
    """(frame_start, frame_end) baked into `blend_path`'s scene, cached by
    the file's mtime -- opening a 100MB+ .blend just to read two ints is
    slow enough to not want to repeat it on every render."""
    mtime = os.path.getmtime(blend_path)
    cache = _load_cache()
    entry = cache.get(blend_path)
    if entry and entry["mtime"] == mtime:
        return entry["frame_start"], entry["frame_end"]

    result = subprocess.run(
        [BLENDER, "--factory-startup", "--background", blend_path, "--python", PROBE_SCRIPT],
        capture_output=True, text=True, check=True,
    )
    line = next(l for l in result.stdout.splitlines() if l.startswith("FRAME_RANGE"))
    _, start, end = line.split()
    frame_start, frame_end = int(start), int(end)

    cache[blend_path] = {"mtime": mtime, "frame_start": frame_start, "frame_end": frame_end}
    _save_cache(cache)
    return frame_start, frame_end


def usable_ranges(section):
    """[(blend_path, usable_start, usable_end), ...] for every file in
    `section`, in logical playback order, after applying each file's
    head/tail trim."""
    ranges = []
    for filename, head_trim, tail_trim in SECTIONS[section]:
        blend_path = os.path.join(DATA_DIR, filename)
        frame_start, frame_end = probe_frame_range(blend_path)
        usable_start = frame_start + head_trim
        usable_end = frame_end - tail_trim
        if usable_start > usable_end:
            raise SystemExit(
                f"{filename}: trim removes its entire usable range "
                f"(baked {frame_start}-{frame_end}, trimmed by {head_trim}/{tail_trim})")
        ranges.append((blend_path, usable_start, usable_end))
    return ranges


def logical_to_segments(section, logical_start, logical_end):
    """Split a [logical_start, logical_end] request into per-file (blend_path,
    file_start, file_end) segments, clamped to what's actually available."""
    segments = []
    logical_cursor = 1
    for blend_path, usable_start, usable_end in usable_ranges(section):
        length = usable_end - usable_start + 1
        section_logical_start = logical_cursor
        section_logical_end = logical_cursor + length - 1

        overlap_start = max(logical_start, section_logical_start)
        overlap_end = min(logical_end, section_logical_end)
        if overlap_start <= overlap_end:
            file_start = usable_start + (overlap_start - section_logical_start)
            file_end = usable_start + (overlap_end - section_logical_start)
            segments.append((blend_path, file_start, file_end))

        logical_cursor = section_logical_end + 1

    total = logical_cursor - 1
    if not segments:
        raise SystemExit(
            f"requested logical range {logical_start}-{logical_end} is out of bounds "
            f"(combined timeline is 1-{total})")
    return segments, total


def render_segment(blend_path, file_start, file_end, engine, samples=None, as_png=False,
                   camera_name=None):
    """Render one file's frame range into its own temp dir (so Blender's
    output filename, which we don't control, can't collide across segments
    or across concurrent runs) and return the resulting clip/image path.

    `as_png` renders `file_start` as a single still image (`-F PNG -f`)
    instead of an `-s`/`-e`/`-a` video range -- only valid when
    file_start == file_end, which main() enforces before calling this.

    `samples` overrides the render sample count on whichever engine ends up
    active (the one just set by `-E`, or the file's own default if `engine`
    is None) -- left alone (None) means "whatever's baked into the file".

    `camera_name` switches the active scene camera before rendering (see
    scripts/_set_camera.py) -- left alone (None) means "whatever camera is
    already set in the file"."""
    tmp_dir = tempfile.mkdtemp(dir=DATA_DIR, prefix=".render_logical_")
    try:
        engine_args = ["-E", engine] if engine else []
        samples_args = []
        if samples is not None:
            # runs after -E above (Blender applies args in order), so this
            # sees whichever engine is actually active for this render
            samples_args = ["--python-expr", (
                "import bpy; scn = bpy.context.scene; eng = scn.render.engine; "
                f"scn.cycles.samples = {samples} if eng == 'CYCLES' else scn.cycles.samples; "
                f"scn.eevee.taa_render_samples = {samples} if eng == "
                "'BLENDER_EEVEE' else scn.eevee.taa_render_samples"
            )]
        out_prefix = os.path.relpath(tmp_dir, DATA_DIR) + "/clip"
        # --factory-startup skips the user's installed add-ons (e.g. a
        # flaky Poliigon add-on background thread has been observed
        # segfaulting Blender during a --background run) -- nothing this
        # pipeline renders depends on any add-on being active.
        base_cmd = [BLENDER, "--factory-startup", "-b", blend_path,
                    "--python", SET_CAMERA_SCRIPT, *engine_args, *samples_args]
        if as_png and file_start == file_end:
            cmd = base_cmd + ["-o", f"//{out_prefix}", "-F", "PNG", "-f", str(file_start)]
        elif as_png:
            cmd = base_cmd + ["-s", str(file_start), "-e", str(file_end),
                              "-o", f"//{out_prefix}", "-F", "PNG", "-a"]
        else:
            cmd = base_cmd + ["-s", str(file_start), "-e", str(file_end),
                              "-o", f"//{out_prefix}", "-F", "FFMPEG", "-a"]
        env = dict(os.environ)
        if camera_name:
            env["RENDER_LOGICAL_CAMERA"] = camera_name
        else:
            env.pop("RENDER_LOGICAL_CAMERA", None)
        subprocess.run(cmd, check=True, env=env)
        clips = sorted(glob.glob(os.path.join(tmp_dir, "clip*")))
        if as_png and file_start != file_end:
            expected = file_end - file_start + 1
            if len(clips) != expected:
                raise SystemExit(
                    f"expected {expected} rendered frames in {tmp_dir}, found {len(clips)}")
            return clips, tmp_dir
        if len(clips) != 1:
            raise SystemExit(f"expected exactly one rendered output in {tmp_dir}, found {clips}")
        return clips[0], tmp_dir
    except BaseException:
        shutil.rmtree(tmp_dir, ignore_errors=True)
        raise


def concat_clips(clip_paths, output_path):
    list_file = tempfile.NamedTemporaryFile(
        mode="w", dir=DATA_DIR, suffix=".txt", delete=False)
    try:
        for clip in clip_paths:
            list_file.write(f"file '{os.path.abspath(clip)}'\n")
        list_file.close()
        subprocess.run(
            ["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", list_file.name,
             "-c", "copy", output_path],
            check=True,
        )
    finally:
        os.unlink(list_file.name)


def hstack_clips(left_path, right_path, output_path, as_png):
    """Composite two already-rendered clips/images side by side (left |
    right) into one output -- used for --combined (overview | chase).
    Unlike concat_clips this re-encodes (hstack draws a new frame out of
    two), so it can't be a plain stream copy."""
    if as_png:
        cmd = ["ffmpeg", "-y", "-i", left_path, "-i", right_path,
              "-filter_complex", "hstack=inputs=2", output_path]
    else:
        cmd = ["ffmpeg", "-y", "-i", left_path, "-i", right_path,
              "-filter_complex", "[0:v][1:v]hstack=inputs=2[o]", "-map", "[o]",
              "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", output_path]
    subprocess.run(cmd, check=True)


def render_camera(section, logical_start, logical_end, output_path, engine, samples, as_png,
                  camera_name):
    """Render the full requested logical range (joining across a
    section2-style multi-file cut if needed) from one camera into
    `output_path`. This is the whole single-camera pipeline; --combined
    calls it twice (once per camera) and composites the results."""
    segments, total = logical_to_segments(section, logical_start, logical_end)
    print(f"combined logical timeline: 1-{total}; rendering "
         f"{len(segments)} segment(s) for requested {logical_start}-{logical_end}"
         f"{f' (camera: {camera_name})' if camera_name else ''}")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    as_png_seq = as_png and logical_start != logical_end
    seq_stem = os.path.splitext(output_path)[0]
    logical_cursor = logical_start
    n_frames = 0
    tmp_dirs = []
    clip_paths = []
    try:
        for blend_path, file_start, file_end in segments:
            print(f"  {os.path.basename(blend_path)}: frames {file_start}-{file_end}")
            clip_path, tmp_dir = render_segment(
                blend_path, file_start, file_end, engine, samples, as_png, camera_name)
            tmp_dirs.append(tmp_dir)
            if as_png_seq:
                # renumber this segment's frames onto the logical timeline
                # (a one-frame segment comes back as a single path)
                if isinstance(clip_path, str):
                    clip_path = [clip_path]
                for i, frame_path in enumerate(clip_path):
                    shutil.move(frame_path, f"{seq_stem}_{logical_cursor + i:05d}.png")
                logical_cursor += len(clip_path)
                n_frames += len(clip_path)
            else:
                clip_paths.append(clip_path)

        if as_png_seq:
            print(f"wrote {n_frames} frames {seq_stem}_{logical_start:05d}.png .. "
                  f"{seq_stem}_{logical_end:05d}.png")
        elif len(clip_paths) == 1:
            shutil.move(clip_paths[0], output_path)
        else:
            concat_clips(clip_paths, output_path)
    finally:
        for tmp_dir in tmp_dirs:
            shutil.rmtree(tmp_dir, ignore_errors=True)


def main():
    args = sys.argv[1:]
    section = "2"
    if args and args[0] in ("--section1", "--section2", "--section2bg"):
        section = args[0][len("--section"):]
        args = args[1:]

    camera_mode = None
    if args and args[0].startswith("--") and args[0][2:] in CAMERA_MODES:
        camera_mode = args[0][2:]
        args = args[1:]
    elif section == "1":
        camera_mode = "combined"  # data/section1.blend's default view

    if len(args) not in (3, 4, 5):
        raise SystemExit(
            "usage: render_logical.py [--section1|--section2] "
            "[--overview|--chaseCollector|--chaseBuilder|--combined] "
            "<start_frame> <end_frame> <output.mp4> [engine [samples]]")
    logical_start = int(args[0])
    logical_end = int(args[1])
    output_path = os.path.abspath(args[2])
    engine = args[3] if len(args) >= 4 else None
    samples = int(args[4]) if len(args) == 5 else None

    as_png = output_path.lower().endswith(".png")
    if as_png and logical_start != logical_end and camera_mode == "combined":
        raise SystemExit(
            "--combined only supports a single-frame .png (or any .mp4 range) -- "
            "for a PNG *sequence*, render each camera separately "
            "(--overview / --chaseCollector) and hstack the frames afterwards, "
            "which is what scripts/cluster/render_section2.sh does")

    if camera_mode != "combined":
        camera_name = CAMERA_NAMES.get(camera_mode)
        render_camera(section, logical_start, logical_end, output_path, engine, samples,
                     as_png, camera_name)
    else:
        # Always overview | chaseCollector -- chaseBuilder is standalone-only
        # (--chaseBuilder), never part of the split screen.
        tmp_dir = tempfile.mkdtemp(dir=DATA_DIR, prefix=".render_logical_combined_")
        try:
            ext = ".png" if as_png else ".mp4"
            overview_path = os.path.join(tmp_dir, f"overview{ext}")
            chase_path = os.path.join(tmp_dir, f"chaseCollector{ext}")
            render_camera(section, logical_start, logical_end, overview_path, engine, samples,
                         as_png, CAMERA_NAMES["overview"])
            render_camera(section, logical_start, logical_end, chase_path, engine, samples,
                         as_png, CAMERA_NAMES["chaseCollector"])
            os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
            hstack_clips(overview_path, chase_path, output_path, as_png)
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)

    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()
