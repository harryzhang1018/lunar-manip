#!/usr/bin/env bash
# render_section.sh -- render a frame range of a .blend file to a video clip.
#
# Wraps the blender CLI invocation used to render section .blend files (see
# data/README_renders.md), so rendering a clip is one command instead of
# remembering blender's -b/-E/-s/-e/-o/-F/-a flags each time.
#
# Usage:
#   scripts/render_section.sh <blend_file> <start_frame> <end_frame> <output_prefix> [engine]
#
# <output_prefix> is a path *relative to the .blend file's own location*
# (passed to Blender as //<output_prefix>) -- e.g. "builder/builder1" writes
# builder/builder1<frame>.mkv next to <blend_file>, not in your current
# shell directory.
# [engine] is EEVEE (default -- matches how these .blend files were saved,
# see blend_from_chrono_export.py's main()) or CYCLES.
#
# Examples:
#   scripts/render_section.sh data/section2_mode1.blend 12 3810 builder/builder1
#   scripts/render_section.sh data/section2_mode2.blend 15 5000 builder/builder2
#   scripts/render_section.sh data/section2_mode1.blend 12 4048 builder/builder1 CYCLES
#   scripts/render_section.sh data/section2_mode1.blend 500 500 builder/frame500  # single frame (video)
#
# For a single frame as a still PNG instead of a one-frame video, call
# Blender directly with -F PNG -f <frame> (or use scripts/render_logical.py
# with a .png output, which does this for you on the logical timeline):
#   blender --factory-startup -b data/section2_mode1.blend -o //builder/frame500 -F PNG -f 500
#
# See data/README_renders.md for how to stitch multiple rendered clips into
# one final video afterward (scripts/concat_videos.sh).

set -euo pipefail

# `blender` in a non-interactive shell resolves to the old system 3.0.1, not
# the user's aliased 5.1.2 (the alias only lives in ~/.bashrc, loaded by
# interactive shells) -- so this defaults to the real 5.1.2 binary directly.
# Override with BLENDER=... if that install ever moves.
BLENDER="${BLENDER:-/home/zacharyrichmond/Downloads/blender-5.1.2-linux-x64/blender-5.1.2-linux-x64/blender}"

if [ "$#" -lt 4 ]; then
  echo "usage: $0 <blend_file> <start_frame> <end_frame> <output_prefix> [engine]" >&2
  exit 1
fi

blend_file="$1"
start_frame="$2"
end_frame="$3"
output_prefix="$4"
engine="${5:-}"

if [ ! -f "$blend_file" ]; then
  echo "error: blend file not found: $blend_file" >&2
  exit 1
fi

engine_args=()
if [ -n "$engine" ]; then
  engine_args=(-E "$engine")
fi

# --factory-startup skips the user's installed add-ons (a flaky Poliigon
# add-on background thread has been observed segfaulting Blender during a
# --background run) -- nothing rendered here depends on any add-on.
"$BLENDER" --factory-startup -b "$blend_file" "${engine_args[@]}" -s "$start_frame" -e "$end_frame" \
  -o "//${output_prefix}" -F FFMPEG -a
