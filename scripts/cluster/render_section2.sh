#!/bin/bash
#SBATCH --job-name=s2render
#SBATCH --output=logs/s2render_%A_%a.out
#SBATCH --error=logs/s2render_%A_%a.err
#SBATCH --partition=research
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=08:00:00
#SBATCH --array=0-14

# Renders a window of the section2 demo on Euler as a Slurm array: every
# array task ("rank") renders an equal chunk of the logical frame range to
# PNG frames with scripts/render_logical.py (EEVEE on the task's GPU), and
# whichever task finishes last encodes the frames into one H.264 mp4.
#
# Runs out of a self-contained workspace, NOT the git checkout -- Euler has
# no git-lfs, so the LFS .blend files and textures are rsync'ed there by
# hand. Expected layout (see "Before first submit" below):
#
#   $WORKSPACE/
#     scripts/render_logical.py, scripts/_print_frame_range.py
#     data/section2_mode{1,2}.blend, data/section2_mode{1,2}_bg.blend
#     data/textures/
#     logs/  renders/
#
# Before first submit (from the local machine):
#   ssh euler 'mkdir -p ~/lunar-manip-render/{scripts,data,logs,renders}'
#   rsync -a scripts/render_logical.py scripts/_print_frame_range.py scripts/cluster/render_section2.sh euler:lunar-manip-render/scripts/
#   rsync -a data/textures data/section2_mode*.blend euler:lunar-manip-render/data/
#   # Blender 5.1.2 (the module blender/4.0.2 can't open these 5.1 files):
#   ssh euler 'cd ~/Packages && wget -q https://download.blender.org/release/Blender5.1/blender-5.1.2-linux-x64.tar.xz && tar xJf blender-5.1.2-linux-x64.tar.xz'
#   # ffmpeg with libx264 for the final encode (the system one has none):
#   ssh euler 'module load conda/miniforge; bootstrap-conda; conda create -y -n s2render -c conda-forge ffmpeg'
#   # Pre-probe the frame-range cache once so 15 tasks don't all do it at t=0:
#   ssh euler 'cd ~/lunar-manip-render && BLENDER=~/Packages/blender-5.1.2-linux-x64/blender python3 -c "import sys; sys.path.insert(0,\"scripts\"); import render_logical as r; print(r.logical_to_segments(\"2bg\",1,999999)[1])"'
#
# Typical usage (0-60 s of the section2 demo with background vehicles):
#   ssh euler 'cd ~/lunar-manip-render && sbatch scripts/render_section2.sh'
#
# Overrides at submit time (all optional):
#   sbatch --export=ALL,START_SEC=60,END_SEC=120 scripts/render_section2.sh
#   sbatch --export=ALL,SECTION=2 scripts/render_section2.sh              # plain section2, no background vehicles
#   sbatch --export=ALL,ENGINE=CYCLES,SAMPLES=64 scripts/render_section2.sh
#   sbatch --export=ALL,CONDA_ENV= scripts/render_section2.sh               # no conda env (system ffmpeg, openh264)
#   sbatch --array=0-29 scripts/render_section2.sh                        # more ranks
#   sbatch --array=0 --export=ALL,START_FRAME=800,END_FRAME=802,OUT_DIR=renders/test scripts/render_section2.sh   # smoke test
#   sbatch -p sbel --gres=gpu:a100:1 scripts/render_section2.sh
#
# Resuming: re-submitting with the same OUT_DIR and array size skips frames
# already on disk. For a fresh run with a different frame range or array
# size, use a new OUT_DIR (or rm -rf the old one) -- the per-task .done
# markers and chunking are tied to both.
#
# Output: $OUT_DIR/<name>.mp4. The PNG frames it is encoded from
# ($OUT_DIR/frames/, ~7 MB each at 2560x1920, i.e. ~12 GB for 60 s) are
# deleted after a successful encode unless KEEP_FRAMES=1.

set -euo pipefail

# Euler's conda. The `s2render` env only has to provide ffmpeg with libx264
# (create once: conda create -n s2render -c conda-forge ffmpeg); rendering
# itself is just Blender + python3. CONDA_ENV= (empty) skips activation and
# falls back to /usr/bin/ffmpeg (openh264 only).
module load conda/miniforge
bootstrap-conda
CONDA_ENV="${CONDA_ENV-s2render}"
if [ -n "$CONDA_ENV" ]; then
  conda activate "$CONDA_ENV"
fi
# The conda module prepends its lib/ to LD_LIBRARY_PATH, which makes
# /usr/bin/ffmpeg (and potentially Blender) load a mismatched libstdc++
# ("CXXABI_1.3.15 not found"). Nothing here needs it -- conda binaries use
# RPATH and Blender ships its own libs -- so drop it.
unset LD_LIBRARY_PATH

WORKSPACE="${WORKSPACE:-$HOME/lunar-manip-render}"
export BLENDER="${BLENDER:-$HOME/Packages/blender-5.1.2-linux-x64/blender}"
SECTION="${SECTION:-2bg}"          # 2bg = with section1 background vehicles, 2 = plain
FPS="${FPS:-30}"                   # the .blend files are baked at 30 fps
START_SEC="${START_SEC:-0}"
END_SEC="${END_SEC:-60}"
START_FRAME="${START_FRAME:-$((START_SEC * FPS + 1))}"   # logical frames, 1-based inclusive
END_FRAME="${END_FRAME:-$((END_SEC * FPS))}"
ENGINE="${ENGINE:-}"               # empty = whatever is baked in the file (EEVEE, 64 samples)
SAMPLES="${SAMPLES:-}"             # only honoured together with ENGINE
CRF="${CRF:-18}"                   # x264 quality for the final mp4
KEEP_FRAMES="${KEEP_FRAMES:-0}"    # 1 = keep the PNG frames after a successful encode (~12 GB / 60 s)
OUT_DIR="${OUT_DIR:-renders/section${SECTION}_${START_SEC}-${END_SEC}s}"
NAME="${NAME:-section${SECTION}_${START_SEC}-${END_SEC}s}"

cd "$WORKSPACE"
mkdir -p "$OUT_DIR/frames" logs

ntasks="${SLURM_ARRAY_TASK_COUNT:-1}"
task="${SLURM_ARRAY_TASK_ID:-0}"
total=$((END_FRAME - START_FRAME + 1))
chunk=$(( (total + ntasks - 1) / ntasks ))
s=$((START_FRAME + task * chunk))
e=$((s + chunk - 1))
[ "$e" -gt "$END_FRAME" ] && e="$END_FRAME"

echo "job=${SLURM_ARRAY_JOB_ID:-local} task=$task/$ntasks node=${SLURMD_NODENAME:-?}"
echo "section=$SECTION frames=$START_FRAME-$END_FRAME ($total frames, $((total / FPS)) s @ $FPS fps), this task: $s-$e"
echo "engine=${ENGINE:-file default} samples=${SAMPLES:-file default} out=$OUT_DIR blender=$BLENDER"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader || true
"$BLENDER" --version | head -1

frame_png() { printf '%s/frames/frame_%05d.png' "$OUT_DIR" "$1"; }

if [ "$s" -gt "$END_FRAME" ]; then
  echo "task $task: no frames left for this rank"
else
  # resume: frames render in order, so skip the already-finished prefix
  first_missing=""
  for ((f = s; f <= e; f++)); do
    if [ ! -s "$(frame_png "$f")" ]; then first_missing=$f; break; fi
  done
  if [ -z "$first_missing" ]; then
    echo "task $task: frames $s-$e already rendered, skipping"
  else
    echo "task $task: rendering $first_missing-$e ($(date))"
    t0=$(date +%s)
    python3 scripts/render_logical.py "--section${SECTION}" "$first_missing" "$e" \
      "$OUT_DIR/frames/frame.png" $ENGINE $SAMPLES
    t1=$(date +%s)
    n=$((e - first_missing + 1))
    echo "task $task: done, $n frames in $((t1 - t0)) s ($(( (t1 - t0) / (n > 0 ? n : 1) )) s/frame)"
  fi
fi
echo "$s $e" > "$OUT_DIR/frames/.task_${task}.done"

# The last task to finish encodes. flock keeps two simultaneous finishers
# from both encoding; the .done count keeps an early finisher from encoding.
(
  flock 9
  done_count=$(ls "$OUT_DIR"/frames/.task_*.done 2>/dev/null | wc -l)
  if [ "$done_count" -lt "$ntasks" ]; then
    echo "task $task: $done_count/$ntasks ranks done, not encoding yet"
    exit 0
  fi
  if [ -s "$OUT_DIR/$NAME.mp4" ]; then
    echo "task $task: $OUT_DIR/$NAME.mp4 already exists"
    exit 0
  fi
  missing=0
  for ((f = START_FRAME; f <= END_FRAME; f++)); do
    [ -s "$(frame_png "$f")" ] || missing=$((missing + 1))
  done
  if [ "$missing" -gt 0 ]; then
    echo "task $task: all ranks done but $missing frames missing -- not encoding (re-submit to fill gaps)"
    exit 1
  fi
  # (no `... | grep -q` here: under pipefail, grep -q quitting early makes
  # the pipeline "fail" and this would silently pick the fallback)
  encoders=$(ffmpeg -hide_banner -encoders 2>/dev/null || true)
  if grep -qE '^ V[^ ]* +libx264 ' <<<"$encoders"; then
    codec=(-c:v libx264 -preset slow -crf "$CRF")
  else
    codec=(-c:v libopenh264 -b:v 20M)      # /usr/bin/ffmpeg has no libx264
  fi
  echo "task $task: all $ntasks ranks done, encoding $total frames -> $OUT_DIR/$NAME.mp4 with $(which ffmpeg) ${codec[1]} ($(date))"
  ffmpeg -hide_banner -loglevel warning -y \
    -framerate "$FPS" -start_number "$START_FRAME" -i "$OUT_DIR/frames/frame_%05d.png" \
    -frames:v "$total" "${codec[@]}" -pix_fmt yuv420p -movflags +faststart \
    "$OUT_DIR/$NAME.mp4"
  ls -la "$OUT_DIR/$NAME.mp4"
  echo "task $task: encode done ($(date))"
  if [ "$KEEP_FRAMES" != "1" ]; then
    rm -rf "$OUT_DIR/frames"
    echo "task $task: deleted $OUT_DIR/frames (KEEP_FRAMES=1 to keep them)"
  fi
) 9>"$OUT_DIR/.encode.lock"
