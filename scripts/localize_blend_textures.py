"""Repoint every image in the currently-open .blend at a real file under
data/textures/, so the .blend has no dependency on external asset folders
that aren't tracked in git (e.g. ~/Poliigon/Library, ~/Downloads/rock).

Handles both cases a freshly re-exported .blend can be in:
  - a broken external link (relative path computed against wherever the
    file used to live, e.g. ~/Documents/.../blendFiles/ -- doesn't resolve
    once the file is copied into data/) -- resolved by searching
    SEARCH_ROOTS for a same-named file.
  - an already-packed image (embedded pixel data, no usable external file)
    -- resolved by writing packed_file.data straight out to data/textures/.

Either way the image ends up pointing at a relative `//textures/<basename>`
path (relative to the .blend's own location -- both section .blend files
live directly in data/, so this always resolves to data/textures/).
Multiple image datablocks sharing a basename (Blender's own '.001'/'.002'
per-append duplicates, or the same texture reused across section .blend
files) collapse onto one shared file instead of one copy each.

Usage:
    blender --background <blend_file> --factory-startup --python \
        scripts/localize_blend_textures.py
"""

import os

import bpy

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEXTURES_DIR = os.path.join(REPO_ROOT, "data", "textures")

# External asset libraries on this machine that these scenarios' --texture /
# --robot-texture flags have pulled from (see blend_from_chrono_export.py) --
# not tracked in git, searched here only to find a same-named source file
# for an image whose link is currently broken.
SEARCH_ROOTS = [
    os.path.expanduser("~/Poliigon/Library"),
    os.path.expanduser("~/Downloads/rock"),
    os.path.expanduser("~/Downloads/moon2"),
    os.path.expanduser("~/Downloads/lunar-rock-bl-1"),
    os.path.expanduser("~/Downloads"),
]


def build_basename_index():
    index = {}
    for root in SEARCH_ROOTS:
        for dirpath, _dirnames, filenames in os.walk(root):
            for filename in filenames:
                index.setdefault(filename, os.path.join(dirpath, filename))
    return index


def main():
    os.makedirs(TEXTURES_DIR, exist_ok=True)
    index = build_basename_index()
    written = set()
    from_pack, from_search, already_local, missing = [], [], [], []

    for img in bpy.data.images:
        if not img.filepath:
            continue
        basename = os.path.basename(img.filepath)
        dest = os.path.join(TEXTURES_DIR, basename)

        already_ok = bpy.path.abspath(img.filepath) == dest and os.path.exists(dest)

        if already_ok:
            already_local.append(img.name)
            continue

        if basename not in written:
            if img.packed_file:
                with open(dest, "wb") as f:
                    f.write(img.packed_file.data)
                from_pack.append(img.name)
            else:
                source = index.get(basename)
                current_abspath = bpy.path.abspath(img.filepath)
                if source is None and os.path.exists(current_abspath):
                    source = current_abspath
                if source is None:
                    missing.append(img.name)
                    continue
                with open(source, "rb") as f_in, open(dest, "wb") as f_out:
                    f_out.write(f_in.read())
                from_search.append(img.name)
            written.add(basename)

        img.filepath = f"//textures/{basename}"
        if img.packed_file:
            img.unpack(method='REMOVE')
        img.reload()

    print(f"  already local: {len(already_local)}")
    print(f"  extracted from packed data: {len(from_pack)}")
    print(f"  copied in from an external asset folder: {len(from_search)}")
    if missing:
        print(f"  WARNING: {len(missing)} image(s) not found anywhere: {missing}")

    bpy.ops.wm.save_mainfile()
    print(f"  saved {bpy.data.filepath}")


if __name__ == "__main__":
    main()
