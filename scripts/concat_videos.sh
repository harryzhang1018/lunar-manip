#!/usr/bin/env bash
# concat_videos.sh -- stitch multiple already-rendered video clips into one
# continuous output file, e.g. joining a section1 clip and a section2 clip
# (each produced by scripts/render_section.sh) into one final video.
#
# Usage:
#   scripts/concat_videos.sh <output.mp4> <clip1> [clip2] [clip3] ...
#
# Example:
#   scripts/concat_videos.sh data/builder/combined.mp4 \
#     data/builder/builder10012-3810.mkv \
#     data/builder/builder20015-5000.mkv
#
# Uses ffmpeg's concat demuxer (stream copy, no re-encode) -- fast, but
# every input clip must share the same codec/resolution/framerate, since
# nothing is transcoded to match. Clips rendered by render_section.sh with
# the same [engine] and the section .blend files' shared render settings
# (see main() in scripts/blend_from_chrono_export.py) satisfy this.

set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: $0 <output.mp4> <clip1> <clip2> [clip3] ..." >&2
  exit 1
fi

output="$1"
shift

list_file="$(mktemp)"
trap 'rm -f "$list_file"' EXIT

for clip in "$@"; do
  if [ ! -f "$clip" ]; then
    echo "error: clip not found: $clip" >&2
    exit 1
  fi
  printf "file '%s'\n" "$(realpath "$clip")" >> "$list_file"
done

ffmpeg -f concat -safe 0 -i "$list_file" -c copy "$output"
