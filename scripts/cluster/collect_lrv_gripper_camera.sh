#!/bin/bash
#SBATCH --job-name=lrv-camera
#SBATCH --output=logs/lrv_camera_%j.out
#SBATCH --error=logs/lrv_camera_%j.err
#SBATCH --partition=sbel
#SBATCH --gres=gpu:a100:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=04:00:00

# Collects the LRV gripper-camera dataset on one GPU. This is intentionally a
# single Slurm job, not a large array: Chrono Sensor/OptiX needs a GPU, while
# each episode is small enough to run sequentially for an initial dataset pass.
#
# Before first submit:
#   mkdir -p logs
#
# Typical usage:
#   sbatch scripts/cluster/collect_lrv_gripper_camera.sh
#
# Override collection settings at submit time:
#   sbatch --export=ALL,EPISODES=1000,SEED=100,OUT_DIR=artifacts/datasets/lrv_camera_1k scripts/cluster/collect_lrv_gripper_camera.sh
#
# If the sbel A100 node is busy or OptiX has trouble with that GPU type, override
# the partition/GPU request from the sbatch command line:
#   sbatch -p research --gres=gpu:rtx4000ada:1 scripts/cluster/collect_lrv_gripper_camera.sh
#   sbatch --gres=gpu:rtxa4500:1 scripts/cluster/collect_lrv_gripper_camera.sh
#   sbatch -p research --gres=gpu:h100:1 scripts/cluster/collect_lrv_gripper_camera.sh

set -euo pipefail

module load conda/miniforge
bootstrap-conda
conda activate pychrono

REPO_DIR="${REPO_DIR:-$HOME/lunar-manip}"
EPISODES="${EPISODES:-100}"
SEED="${SEED:-7}"
OUT_DIR="${OUT_DIR:-artifacts/datasets/lrv_gripper_camera_${SLURM_JOB_ID:-local}}"
SIM2_DURATION="${SIM2_DURATION:-10.0}"
EXTRA_ARGS="${EXTRA_ARGS:-}"

cd "$REPO_DIR"

echo "job_id=${SLURM_JOB_ID:-local}"
echo "node=${SLURMD_NODENAME:-unknown}"
echo "repo=$PWD"
echo "episodes=$EPISODES"
echo "seed=$SEED"
echo "output=$OUT_DIR"
echo "sim2_duration=$SIM2_DURATION"
echo "extra_args=$EXTRA_ARGS"
echo "conda_prefix=$CONDA_PREFIX"
nvidia-smi

python -m py_compile scenarios/LRV_Arm.py scenarios/collect_data.py

python scenarios/collect_data.py \
  --episodes "$EPISODES" \
  --seed "$SEED" \
  --sim2-duration "$SIM2_DURATION" \
  --require-sensors \
  --output-dir "$OUT_DIR" \
  $EXTRA_ARGS

python - "$OUT_DIR" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
index_path = root / "index.jsonl"
rows = [json.loads(line) for line in index_path.read_text().splitlines() if line.strip()]
picked = sum(1 for row in rows if row["success"]["picked_up"])

print(f"dataset={root}")
print(f"samples={len(rows)}")
print(f"picked_up={picked}/{len(rows)}")
if rows:
    print(
        "lift_delta_m="
        + ", ".join(f"{row['rock']['lift_delta_m']:.3f}" for row in rows[:10])
        + (" ..." if len(rows) > 10 else "")
    )
PY
