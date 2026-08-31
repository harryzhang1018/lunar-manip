#!/usr/bin/env bash
# Render frames of a Chrono Blender export with scripts/render/render_orbitbuilder.py.
#
#   scripts/render/render.sh --export-dir artifacts/blender/trackedveh_orbitbuilder --frame 0
#
# Every argument is passed through to the render script (see its --help / docstring).
# BLENDER overrides the Blender binary; the default is the 5.1.2 install that has the
# chrono_import add-on set up (~/.config/blender/5.1/scripts/addons/chrono_import.py).
# Blender 4.0 does not work: the export uses an operator added in 4.1.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BLENDER="${BLENDER:-$HOME/blender-5.1.2-linux-x64/blender}"
exec "$BLENDER" --background --factory-startup \
    --python "$here/render_orbitbuilder.py" -- "$@"
