"""Build a .blend file from an AMD-UW pose recording (see Downloads/amdZip's
README.md for the format this reads).

This is the same idea as blend_from_chrono_export.py -- bake everything into
ordinary Blender keyframes once, up front, so the result opens and scrubs
anywhere with no importer/add-on alive -- applied to a different source
format. The dataset ships its own `tools/blender_import.py`, which already
gets the geometry right (aabb-fit mesh replacement, correct shape-frame
parenting, OBJ axis handling); what it does NOT do is scale: it calls
`obj.keyframe_insert()` once per body per frame, which is the exact
per-call-insert-and-sort cost blend_from_chrono_export.py had to eliminate
to survive a few thousand frames, and this format runs to ~86,000 frames per
rank. So the geometry/mesh-resolution logic below is ported close to
verbatim from their tools/blender_import.py and tools/read_trajectory.py
(binary reader, aabb-fit, OBJ axis convention), and the keyframing instead
uses this repo's bulk foreach_set bake (bootstrap each F-curve once, fill
every point in one call -- ~300x faster, see bake_keyframes below) and the
same unchanged-sample dedup that keeps a parked body's F-curve from growing
by one point per frame for the rest of the run.

Usage (via the real Blender executable's bundled Python):

    blender --background --python blend_from_amd_uw_export.py -- \\
        <dataset_dir> <output.blend> [options]

    --ranks 1,2,3 | all       which ranks to load (default: all present)
    --groups g1,g2            restrict to these groups (default: all)
    --meshes DIR               replacement meshes, matched by basename
    --mesh-map FILE.json       replacement meshes, matched by regex (see below)
    --no-fit                   place replacement meshes verbatim, no aabb fit
    --start T / --end T        restrict to this time range (seconds)
    --stride N                 keyframe every Nth sample
    --no-static                skip static_props.jsonl (terrain/rings/pad)
    --heightmap PATH            build the real terrain mesh from this BMP
                                 heightmap instead of a flat placeholder --
                                 see "About the terrain" below
    --terrain-texture PATH      color/albedo photo applied to the real
                                 heightfield mesh built by --heightmap,
                                 UV-matched to the same grid -- see "About
                                 the terrain" below. No effect without
                                 --heightmap (nothing to UV-match against)
    --terrain-detail-texture DIR  a PBR texture set (color/normal/roughness,
                                 auto-detected like --texture) tiled many
                                 times smaller and overlay-blended on top of
                                 --terrain-texture for close-up surface
                                 detail the base photo is too low-res for.
                                 No effect without --terrain-texture.
    --terrain-detail-scale N     how many times the detail texture repeats
                                 across the terrain (default 40) -- higher
                                 is finer/smaller detail
    --scm                       build real regolith deformation geometry
                                 from every loaded rank's rank_N_scm.bin, if
                                 present -- see "About SCM terrain" below
    --scm-texture DIR            PBR texture set (auto-detected like
                                 --texture) applied to the SCM rut mesh,
                                 tiled by real-world distance and mixed
                                 toward grey by --scm-grey-shift. Falls back
                                 to a flat gray without this.
    --scm-grey-shift F           0..1, how much to mix --scm-texture's color
                                 toward neutral grey (default 0.3)
    --texture DIR family[,family...]     PBR-texture objects in these groups
    --part-texture DIR substr[,substr...]  PBR-texture objects by NAME
                                 substring rather than group (e.g. track
                                 shoes, which have no group of their own) --
                                 excluded from --robot-texture below
    --robot-texture DIR exclude_families  same, for everything else, EXCEPT
                                 any part whose manifest color is genuinely
                                 non-gray (a real color-coded part, e.g. a
                                 reddish chassis, is left alone) (see
                                           blend_from_chrono_export.py's
                                           --robot-texture for the exact rule)
    --world BLEND_FILE COLLECTION_OR_WORLD_NAME   append a world/environment
    --append BLEND_FILE COLLECTION_NAME   append an arbitrary collection (scene
                                           dressing carried over from a
                                           hand-assembled file, e.g. "Collection 6"
                                           -- same mechanism as
                                           blend_from_chrono_export.py's
                                           positional append args; repeatable)

--mesh-map is JSON: {"regex pattern": "/path/to/replacement.obj"}, matched
against the manifest mesh's basename OR the object's "group/part" string.
Same as the dataset's own blender_import.py.

Example -- one rank, quick look, first two minutes, every other sample:

    blender --background --python scripts/blend_from_amd_uw_export.py -- \\
      ~/Downloads/amdZip/amd-uw-run16/data run16_rank1.blend \\
      --ranks 1 --end 120 --stride 2

Example -- the whole site (all 15 ranks), with rock/robot PBR texturing:

    blender --background --python scripts/blend_from_amd_uw_export.py -- \\
      ~/Downloads/amdZip/amd-uw-run16/data run16_full.blend \\
      --meshes ~/Downloads/amdZip/amd-uw-run16/meshes \\
      --texture ~/Downloads/rock rock \\
      --robot-texture ~/Poliigon/Library/MetalGalvanizedSteelWorn001 ''

About the terrain: each rank_N_meta.json names the real ground as a
heightmap image (e.g. "terrain":{"heightmap":"terrain/terrain2_graded.png",
"length":1024,"width":1024,"min_height":-25,"max_height":25}) -- but that
image is a reference to wherever the sim actually ran, and is not itself
included in most archives of this data (unlike the OBJ meshes, which get
mirrored into meshes/). It does exist in the AMD-UW_SBEL repo itself, at
data/terrain/<name> on the scm branch. Without it, `--heightmap` has
nothing to load and terrain falls back to a flat placeholder plane
sized/positioned from static_props' own recorded aabb -- flat, and not
where any given vehicle's wheels actually contact the ground, since that
was decided by the real (missing) relief. `--heightmap` reads both .bmp and
.png (8 or 16-bit grayscale); point it at the real file and this builds an
actual displaced grid mesh, fit the same way the reference tool
(tools/replay_run.py's terrain_mesh()) does: the image's own observed
grey range mapped onto the terrain shape's recorded aabb, NOT meta.json's
declared min_height/max_height -- that declared range does not match what
the sim actually built (confirmed against a real run: declared [-25, 25],
actual aabb z=-13.82..12.65), so using it would silently place every
vehicle floating above or sunk into the ground.

The heightmap is grayscale -- elevation only, no color. The AMD-UW_SBEL
repo separately has data/terrain/terrain2.png, a much higher-resolution
(1254x1254 vs. the heightmap's 256x256) photographic lunar-surface color
image covering the SAME physical area. `--terrain-texture PATH` applies it
(or any other color photo) as the heightfield mesh's base color, UV-mapped
1:1 to the same row/column grid the heights come from -- see
build_heightfield_mesh's UV assignment. Requires --heightmap (there is no
matching UV grid to apply it to otherwise); has no effect on the flat
placeholder or a --mesh-map replacement mesh (the latter brings its own
material already).

A third option that needs neither: `--mesh-map` (see above) is tried for the
terrain shape too, matched against its manifest `shape_name` ("terrain2")
the same way any other shape matches by basename -- so a real terrain OBJ
you already have (with its own regolith texture/material) can be substituted
in directly, fitted to the recorded aabb like any other replacement mesh:

    echo '{"terrain2": "/path/to/your/lunar_terrain.obj"}' > terrain_map.json
    blender --background --python scripts/blend_from_amd_uw_export.py -- \\
      ~/Downloads/amdZip/amd-uw-run16/data run16.blend \\
      --mesh-map terrain_map.json

`--heightmap` and `--mesh-map` can both be given; a mesh-map match for the
terrain shape wins (it brings its own real material, which a generated
heightfield mesh does not), falling through to `--heightmap` and then the
flat placeholder in that order.

About SCM terrain: a run recorded with Chrono's deformable Soil Contact
Model additionally writes rank_N_scm.bin -- real, dynamic regolith height
data (not a placeholder) at every grid node any vehicle actually disturbed,
sampled at deformation_rate_hz. `--scm` reads every LOADED rank's file
(so pass --ranks to control which), keeps each node's final height (later
overwrites earlier), and builds one quad per 2x2 block of nodes that are
ALL present -- so a rover's rut lane comes out as a real disconnected
ribbon of geometry, not a filled-in area bridging real gaps. This is
separate from (and additional to) the flat/heightmap/mesh-map terrain
options above: it only covers what a vehicle actually touched, not the
whole site, so it's meant to sit alongside one of those, not replace it.
"""

import array
import glob
import json
import math
import os
import re
import struct
import sys

import bmesh
import bpy
import mathutils


# --------------------------------------------------------------------- reading

HEADER = struct.Struct("<8sIIdd")
FRAME_HEADER = struct.Struct("<IdI")
RECORD = struct.Struct("<I7f")
FILE_MAGIC = b"AMDUWTRJ"
FRAME_MAGIC = 0x544A5246


def discover_ranks(dataset):
    ranks = []
    for path in glob.glob(os.path.join(dataset, "rank_*_frames.bin")):
        m = re.search(r"rank_(\d+)_frames\.bin$", path)
        if m:
            ranks.append(int(m.group(1)))
    return sorted(ranks)


def load_jsonl(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def load_manifest(dataset, rank):
    return load_jsonl(os.path.join(dataset, f"rank_{rank}_objects.jsonl"))


def read_frames(dataset, rank, start=None, end=None, stride=1):
    """Yields (time, {index: (pos_xyz, quat_wxyz)}). Tolerates a truncated
    final frame (a run stopped by hand leaves a partial tail, per the
    format's own README)."""
    path = os.path.join(dataset, f"rank_{rank}_frames.bin")
    with open(path, "rb") as f:
        head = f.read(HEADER.size)
        magic, version, file_rank, rate, step = HEADER.unpack(head)
        if magic != FILE_MAGIC:
            raise ValueError(f"{path}: not an AMD-UW recording")
        n = 0
        while True:
            fh = f.read(FRAME_HEADER.size)
            if len(fh) < FRAME_HEADER.size:
                return
            fmagic, time, count = FRAME_HEADER.unpack(fh)
            if fmagic != FRAME_MAGIC:
                raise ValueError(f"{path}: lost frame sync at t={time}")
            payload = f.read(RECORD.size * count)
            if len(payload) < RECORD.size * count:
                return
            if end is not None and time > end:
                return
            take = (start is None or time >= start) and (n % stride == 0)
            n += 1
            if not take:
                continue
            poses = {}
            for i in range(count):
                idx, px, py, pz, qw, qx, qy, qz = RECORD.unpack_from(payload, i * RECORD.size)
                poses[idx] = ((px, py, pz), (qw, qx, qy, qz))
            yield time, poses


SCM_HEADER = struct.Struct("<8sIIdd7dii")
SCM_FRAME = struct.Struct("<IdI")
SCM_NODE = struct.Struct("<iif")
SCM_FILE_MAGIC = b"AMDUWSCM"
SCM_FRAME_MAGIC = 0x4D435353


def read_scm(path):
    """A rank's SCM (Soil Contact Model) deformation recording: real,
    dynamic regolith height changes as vehicles disturb the ground, at
    `deformation_rate_hz` (10 in this dataset). Returns (delta, plane, nodes)
    where `nodes` is {(i, j): z}, the FINAL absolute height of every grid
    node this rank ever touched (later frames overwrite earlier ones for the
    same node -- the recorder periodically re-emits every touched node as a
    keyframe so a consumer joining mid-file is only stale, never wrong, per
    AMD-UW_SBEL's tools/replay_run.py, which this format/logic is ported
    from). Node (i, j) sits at local point (i*delta, j*delta, z); `plane`
    (a (pos, quat) pair) transforms that into world space -- identity in
    every file seen so far, but applied generally rather than assumed."""
    with open(path, "rb") as f:
        buf = f.read()
    if len(buf) < SCM_HEADER.size:
        raise ValueError(f"{path}: shorter than its header")
    fields = SCM_HEADER.unpack_from(buf, 0)
    magic = fields[0]
    if magic != SCM_FILE_MAGIC:
        raise ValueError(f"{path}: bad magic {magic!r}")
    delta = fields[4]
    plane = fields[5:12]  # (px, py, pz, qw, qx, qy, qz)
    off = SCM_HEADER.size
    nodes = {}
    while off + SCM_FRAME.size <= len(buf):
        fmagic, time, count = SCM_FRAME.unpack_from(buf, off)
        if fmagic != SCM_FRAME_MAGIC:
            raise ValueError(f"{path}: lost frame sync at offset {off}")
        off += SCM_FRAME.size
        need = SCM_NODE.size * count
        if off + need > len(buf):
            break  # truncated final sample
        for k in range(count):
            i, j, z = SCM_NODE.unpack_from(buf, off + k * SCM_NODE.size)
            nodes[(i, j)] = z
        off += need
    return delta, plane, nodes


def scm_node_world_pos(i, j, z, delta, plane):
    pos = mathutils.Vector(plane[0:3])
    quat = mathutils.Quaternion(plane[3:7])
    local = mathutils.Vector((i * delta, j * delta, z))
    return pos + quat @ local


def read_bmp_heightmap(path):
    """A grayscale BMP as a row-major list of lists of 0..1 floats -- enough
    to read the terrain heightmap this dataset's meta.json references (see
    the module docstring's "About the terrain") if it's ever tracked down.
    Handles the common uncompressed (BI_RGB) 8-bit palette and 24/32-bit
    cases; anything else (RLE compression, 16-bit) raises rather than
    silently misreading pixels as terrain height."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:2] != b"BM":
        raise ValueError(f"{path}: not a BMP (bad magic)")
    pixel_offset = struct.unpack_from("<I", data, 10)[0]
    dib_size = struct.unpack_from("<I", data, 14)[0]
    width, height = struct.unpack_from("<ii", data, 18)
    bpp = struct.unpack_from("<H", data, 28)[0]
    compression = struct.unpack_from("<I", data, 30)[0] if dib_size >= 40 else 0
    if compression != 0:
        raise ValueError(f"{path}: compressed BMP (method {compression}) not supported")
    if bpp not in (8, 24, 32):
        raise ValueError(f"{path}: {bpp}-bit BMP not supported (need 8/24/32)")
    top_down = height < 0
    height = abs(height)
    bytes_per_pixel = bpp // 8
    row_size = ((bpp * width + 31) // 32) * 4  # rows padded to a 4-byte boundary
    rows = []
    for r in range(height):
        row_off = pixel_offset + r * row_size
        row = []
        for c in range(width):
            px_off = row_off + c * bytes_per_pixel
            if bpp == 8:
                v = data[px_off] / 255.0
            else:  # 24/32-bit BGR(A) -- luminance from the color channels
                b, g, rr = data[px_off], data[px_off + 1], data[px_off + 2]
                v = (0.114 * b + 0.587 * g + 0.299 * rr) / 255.0
            row.append(v)
        rows.append(row)
    if not top_down:
        rows.reverse()  # BMP rows are bottom-up by default
    return rows


def _png_unfilter(raw, width, height, bytes_per_pixel):
    """Reverse PNG's per-scanline filtering (each row prefixed with a filter
    type byte: None/Sub/Up/Average/Paeth), leaving raw pixel bytes."""
    stride = width * bytes_per_pixel
    out = bytearray(stride * height)
    src_pos = 0
    prev_row = bytearray(stride)
    for r in range(height):
        ftype = raw[src_pos]
        src_pos += 1
        row = bytearray(raw[src_pos:src_pos + stride])
        src_pos += stride
        for i in range(stride):
            a = row[i - bytes_per_pixel] if i >= bytes_per_pixel else 0
            b = prev_row[i]
            c = prev_row[i - bytes_per_pixel] if i >= bytes_per_pixel else 0
            if ftype == 0:
                pass
            elif ftype == 1:
                row[i] = (row[i] + a) & 0xFF
            elif ftype == 2:
                row[i] = (row[i] + b) & 0xFF
            elif ftype == 3:
                row[i] = (row[i] + (a + b) // 2) & 0xFF
            elif ftype == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pred = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                row[i] = (row[i] + pred) & 0xFF
            else:
                raise ValueError(f"unknown PNG filter type {ftype}")
        out[r * stride:(r + 1) * stride] = row
        prev_row = row
    return bytes(out)


def read_png_heightmap(path):
    """A grayscale PNG as a row-major list of lists of 0..1 floats, same
    contract as read_bmp_heightmap. Handles color type 0 (grayscale) at bit
    depth 8 or 16, uncompressed-filter-wise via zlib (the only compression
    method PNG defines) -- which covers this dataset's actual heightmap
    (terrain2_graded.png: 256x256, 16-bit grayscale) without pulling in an
    image library dependency Blender's bundled Python doesn't ship."""
    import zlib

    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path}: not a PNG (bad signature)")
    pos = 8
    width = height = bit_depth = color_type = None
    idat = bytearray()
    while pos < len(data):
        length, = struct.unpack_from(">I", data, pos)
        ctype = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        if ctype == b"IHDR":
            width, height, bit_depth, color_type, comp, filt, interlace = \
                struct.unpack(">IIBBBBB", chunk)
            if comp != 0 or filt != 0 or interlace != 0:
                raise ValueError(f"{path}: unsupported IHDR flags (interlaced PNG?)")
        elif ctype == b"IDAT":
            idat += chunk
        elif ctype == b"IEND":
            break
        pos += 8 + length + 4
    if color_type != 0 or bit_depth not in (8, 16):
        raise ValueError(f"{path}: only grayscale 8/16-bit PNG supported "
                         f"(color_type={color_type}, bit_depth={bit_depth})")
    bytes_per_pixel = bit_depth // 8
    raw = zlib.decompress(bytes(idat))
    pixels = _png_unfilter(raw, width, height, bytes_per_pixel)
    maxval = (1 << bit_depth) - 1
    rows = []
    for r in range(height):
        row = []
        row_off = r * width * bytes_per_pixel
        for c in range(width):
            off = row_off + c * bytes_per_pixel
            v = pixels[off] if bit_depth == 8 else (pixels[off] << 8 | pixels[off + 1])
            row.append(v / maxval)
        rows.append(row)
    return rows


def read_heightmap(path):
    """Dispatch to the right reader by extension -- read_bmp_heightmap or
    read_png_heightmap, same {rows} contract either way."""
    ext = os.path.splitext(path)[1].lower()
    if ext == ".bmp":
        return read_bmp_heightmap(path)
    if ext == ".png":
        return read_png_heightmap(path)
    raise ValueError(f"{path}: unsupported heightmap format {ext!r} (need .bmp or .png)")


# -------------------------------------------------------------- mesh resolution

def resolve_mesh(src_path, group_part, mesh_dir, mesh_map):
    """Pick the file to actually load for a manifest mesh reference -- ported
    from the dataset's own blender_import.py (mesh_map by regex first, then
    a basename/stem search under mesh_dir, then the recorded absolute path
    if it happens to still exist on this machine)."""
    base = os.path.basename(src_path)
    for pattern, replacement in (mesh_map or {}).items():
        if re.search(pattern, base) or re.search(pattern, group_part):
            return replacement
    if mesh_dir:
        for root, _dirs, files in os.walk(mesh_dir):
            if base in files:
                return os.path.join(root, base)
            stem = os.path.splitext(base)[0]
            for candidate in files:
                if os.path.splitext(candidate)[0] == stem:
                    return os.path.join(root, candidate)
    return src_path if os.path.exists(src_path) else None


class MeshCache:
    """Loads each mesh file once and hands out cheap `.copy()`s that share
    mesh data -- the dataset's meshes/ folder has 28 files total, but they're
    referenced by hundreds of bodies (126 track shoes alone)."""

    def __init__(self):
        self._templates = {}

    def get(self, path):
        if path in self._templates:
            return self._templates[path]
        before = set(bpy.data.objects)
        ext = os.path.splitext(path)[1].lower()
        try:
            if ext == ".obj":
                # forward='Y', up='Z' -- NO axis conversion. Both Chrono and
                # Blender are Z-up right-handed and Chrono read these OBJs
                # verbatim, so the importer's default -Z/Y remap would
                # rotate every part 90 degrees (see the dataset README's
                # "Coordinate system" section).
                if hasattr(bpy.ops.wm, "obj_import"):
                    bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")
                else:
                    bpy.ops.import_scene.obj(filepath=path, axis_forward="Y", axis_up="Z")
            elif ext in (".glb", ".gltf"):
                bpy.ops.import_scene.gltf(filepath=path)
            elif ext == ".fbx":
                bpy.ops.import_scene.fbx(filepath=path)
            elif ext == ".stl":
                if hasattr(bpy.ops.wm, "stl_import"):
                    bpy.ops.wm.stl_import(filepath=path)
                else:
                    bpy.ops.import_mesh.stl(filepath=path)
            else:
                self._templates[path] = None
                return None
        except Exception as exc:                                  # noqa: BLE001
            print(f"  warning: failed to import {path}: {exc}")
            self._templates[path] = None
            return None
        new = [o for o in bpy.data.objects if o not in before]
        if not new:
            self._templates[path] = None
            return None
        if len(new) > 1:                     # multi-object file -> one template
            for o in bpy.data.objects:
                o.select_set(False)
            for o in new:
                o.select_set(True)
            bpy.context.view_layer.objects.active = new[0]
            bpy.ops.object.join()
            new = [bpy.context.view_layer.objects.active]
        template = new[0]
        for coll in list(template.users_collection):
            coll.objects.unlink(template)    # keep it out of the scene itself
        self._templates[path] = template
        return template


def _local_bbox(obj):
    cs = [mathutils.Vector(c) for c in obj.bound_box]
    lo = mathutils.Vector((min(c.x for c in cs), min(c.y for c in cs), min(c.z for c in cs)))
    hi = mathutils.Vector((max(c.x for c in cs), max(c.y for c in cs), max(c.z for c in cs)))
    return lo, hi


def fit_to_aabb(child, aabb_min, aabb_max):
    """Scale+offset `child` (already at its native size) so its own bounding
    box matches the recorded aabb_min/aabb_max. Not a nicety: Chrono bakes
    transforms into mesh vertices at load time, so a rock reporting
    scale=[1,1,1] can still be drawn at 0.2x and re-based to sit on the
    ground -- the aabb is the only reliable record of where the geometry
    actually was (see the dataset README's "Three conventions" section)."""
    lo, hi = _local_bbox(child)
    tgt_lo, tgt_hi = mathutils.Vector(aabb_min), mathutils.Vector(aabb_max)
    cur = hi - lo
    want = tgt_hi - tgt_lo
    s = [(want[i] / cur[i]) if abs(cur[i]) > 1e-9 else 1.0 for i in range(3)]
    child.scale = s
    centre_now = mathutils.Vector([(lo[i] + hi[i]) * 0.5 * s[i] for i in range(3)])
    centre_want = (tgt_lo + tgt_hi) * 0.5
    child.location = centre_want - centre_now


# ----------------------------------------------------------------- PBR texturing
# Copied from blend_from_chrono_export.py so both converters build materials
# the same way -- see that file for the fuller rationale in comments.

_TEXTURE_MAP_KEYWORDS = {
    # Covers both Poliigon-style full words (diffuse, normal) and Poly
    # Haven-style abbreviations (diff, nor/nor_gl -- "gl" alone is too
    # generic to list, but "nor" alone is specific enough and catches
    # "nor_gl"/"nor_dx" once split into individual '_'-delimited tokens).
    "color": ("color", "albedo", "basecolor", "diffuse", "diff", "col"),
    "roughness": ("roughness", "rough"),
    "metalness": ("metalness", "metallic", "metal"),
    "normal": ("normalgl", "normal_gl", "normal", "nor", "nrm"),
    "ao": ("ambientocclusion", "ao"),
    "displacement": ("displacement", "height", "disp"),
}
_TEXTURE_NON_MAP_SUFFIXES = {"metalness", "specular", "metallic", "gloss"}


def _core_tokens(filename):
    stem = os.path.splitext(filename)[0]
    tokens = [t.lower() for t in re.split(r"[_\-]+", stem) if t]
    if tokens and tokens[-1] in _TEXTURE_NON_MAP_SUFFIXES:
        tokens = tokens[:-1]
    if tokens and re.fullmatch(r"\d+k", tokens[-1]):
        tokens = tokens[:-1]
    return tokens


def find_pbr_maps(folder):
    """{map_key: path or None} for every _TEXTURE_MAP_KEYWORDS entry found
    in `folder`, matched by filename keyword (see _core_tokens) -- shared by
    build_pbr_material_from_folder and build_terrain_detail_material so
    there's one place that knows how to auto-detect a PBR map set by name."""
    folder = os.path.abspath(os.path.expanduser(folder))
    files = os.listdir(folder)
    file_tokens = {fname: _core_tokens(fname) for fname in files}

    def find(map_key):
        for keyword in _TEXTURE_MAP_KEYWORDS[map_key]:
            for fname, tokens in file_tokens.items():
                if keyword in tokens:
                    return os.path.join(folder, fname)
        return None

    return {key: find(key) for key in _TEXTURE_MAP_KEYWORDS}


def build_pbr_material_from_folder(name, folder):
    folder = os.path.abspath(os.path.expanduser(folder))
    paths = find_pbr_maps(folder)
    found = {k: v for k, v in paths.items() if v}
    print(f"  texture '{name}' from {folder}: {list(found)}")

    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (400, 0)
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (0, 0)
    links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])

    def add_image_node(path, non_color, y):
        img = bpy.data.images.load(path)
        img.colorspace_settings.name = 'Non-Color' if non_color else 'sRGB'
        node = nodes.new("ShaderNodeTexImage")
        node.image = img
        node.location = (-600, y)
        return node

    y = 400
    if paths["color"]:
        color_node = add_image_node(paths["color"], non_color=False, y=y)
        if paths["ao"]:
            ao_node = add_image_node(paths["ao"], non_color=True, y=y - 300)
            mix = nodes.new("ShaderNodeMixRGB")
            mix.blend_type = 'MULTIPLY'
            mix.inputs["Fac"].default_value = 1.0
            mix.location = (-300, y)
            links.new(color_node.outputs["Color"], mix.inputs["Color1"])
            links.new(ao_node.outputs["Color"], mix.inputs["Color2"])
            links.new(mix.outputs["Color"], bsdf.inputs["Base Color"])
        else:
            links.new(color_node.outputs["Color"], bsdf.inputs["Base Color"])
        y -= 300
    if paths["roughness"]:
        rough_node = add_image_node(paths["roughness"], non_color=True, y=y)
        links.new(rough_node.outputs["Color"], bsdf.inputs["Roughness"])
        y -= 300
    if paths["metalness"]:
        metal_node = add_image_node(paths["metalness"], non_color=True, y=y)
        links.new(metal_node.outputs["Color"], bsdf.inputs["Metallic"])
        y -= 300
    if paths["normal"]:
        normal_img_node = add_image_node(paths["normal"], non_color=True, y=y)
        normal_map_node = nodes.new("ShaderNodeNormalMap")
        normal_map_node.location = (-300, y)
        links.new(normal_img_node.outputs["Color"], normal_map_node.inputs["Color"])
        links.new(normal_map_node.outputs["Normal"], bsdf.inputs["Normal"])
        y -= 300
    if paths["displacement"]:
        disp_img_node = add_image_node(paths["displacement"], non_color=True, y=y)
        disp_node = nodes.new("ShaderNodeDisplacement")
        disp_node.location = (100, -400)
        links.new(disp_img_node.outputs["Color"], disp_node.inputs["Height"])
        links.new(disp_node.outputs["Displacement"], output.inputs["Displacement"])

    return mat


def smart_uv_unwrap(objects, angle_limit_degrees=66.0):
    view_layer = bpy.context.view_layer
    seen = set()
    for obj in objects:
        if obj.type != 'MESH' or obj.data.name in seen:
            continue
        seen.add(obj.data.name)
        for o in view_layer.objects:
            o.select_set(False)
        obj.select_set(True)
        view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.uv.smart_project(angle_limit=math.radians(angle_limit_degrees))
        bpy.ops.object.mode_set(mode='OBJECT')
    if seen:
        print(f"  smart-UV-unwrapped {len(seen)} unique mesh(es) across {len(objects)} object(s)")


def append_collection(filepath, collection_name):
    """Copy a named collection (and everything it references -- objects,
    meshes, materials) from another .blend file into the current one, and
    link it under the scene. Same as blend_from_chrono_export.py's
    append_collection -- used to carry over manually-built scene dressing
    (camera, lighting, terrain) that isn't part of the AMD-UW recording
    itself."""
    filepath = os.path.abspath(os.path.expanduser(filepath))
    with bpy.data.libraries.load(filepath, link=False) as (data_from, data_to):
        if collection_name not in data_from.collections:
            print(f"  warning: collection '{collection_name}' not found in {filepath} "
                 f"(has: {list(data_from.collections)})")
            return
        data_to.collections = [collection_name]
    for coll in data_to.collections:
        if coll is not None:
            bpy.context.scene.collection.children.link(coll)
            print(f"  appended collection '{coll.name}' from {filepath}")


def append_world(filepath, world_name):
    filepath = os.path.abspath(os.path.expanduser(filepath))
    before = set(bpy.data.worlds)
    with bpy.data.libraries.load(filepath, link=False) as (data_from, data_to):
        if world_name in data_from.worlds:
            data_to.worlds = [world_name]
    new = [w for w in bpy.data.worlds if w not in before]
    if new:
        bpy.context.scene.world = new[0]
        print(f"  appended world '{new[0].name}' from {filepath} and set as scene world")
    else:
        print(f"  warning: world '{world_name}' not found in {filepath}")


# ------------------------------------------------------------------- body build

_color_material_cache = {}


def get_color_material(color):
    """One shared flat-color material per distinct manifest color, reused
    across every shape that reports it -- instead of one new material per
    shape instance (thousands of near-identical materials for, say, 126
    track shoes that are all the same color)."""
    key = tuple(round(c, 4) for c in color[:3])
    mat = _color_material_cache.get(key)
    if mat is None:
        mat = bpy.data.materials.new(f"amduw_color_{key}")
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf is not None:
            bsdf.inputs["Base Color"].default_value = (*key, 1.0)
        _color_material_cache[key] = mat
    return mat


def build_terrain_color_material(image):
    """A minimal Principled BSDF material for a single color/albedo photo
    (image texture -> Base Color, nothing else) -- unlike
    build_pbr_material_from_folder, which auto-detects a whole PBR map set
    by filename keyword, terrain has just one real image available
    (terrain2.png, a photographic lunar-surface texture; no matching
    roughness/normal/etc. maps exist for it), so there's nothing to wire up
    beyond color."""
    mat = bpy.data.materials.new(f"{image.name}_color")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    tex_node = nodes.new("ShaderNodeTexImage")
    tex_node.image = image
    tex_node.location = (-300, 0)
    links.new(tex_node.outputs["Color"], bsdf.inputs["Base Color"])
    return mat


def build_terrain_detail_material(base_image, detail_folder, detail_scale=40.0):
    """Two-layer terrain material: `base_image` (terrain2.png, a real
    photographic lunar-surface color image, UV-matched 1:1 to the
    heightfield mesh's own grid -- see build_heightfield_mesh) provides the
    large-scale color and crater placement, UNCHANGED. `detail_folder` (a
    PBR texture set, matched by filename keyword via find_pbr_maps -- e.g.
    color/normal/roughness) is tiled many times smaller via a Mapping node
    scaling the SAME UVs by `detail_scale`, so up close it adds fine surface
    grain the low-res base image doesn't have. This is the standard
    macro-plus-detail-texture technique real terrain shaders use for
    exactly this problem (one photo can't be both planet-scale accurate AND
    close-up sharp) -- not a replacement for the base texture, a multiply
    blend on top of it, plus the detail set's own normal map for actual
    surface relief the base image's color alone can't provide."""
    maps = find_pbr_maps(detail_folder)
    print(f"  terrain detail texture from {detail_folder}: {[k for k, v in maps.items() if v]}")

    mat = bpy.data.materials.new(f"{base_image.name}_detail")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    bsdf = nodes.get("Principled BSDF")

    base_node = nodes.new("ShaderNodeTexImage")
    base_node.image = base_image
    base_node.location = (-700, 300)

    uv_node = nodes.new("ShaderNodeUVMap")
    uv_node.location = (-1100, -200)
    mapping = nodes.new("ShaderNodeMapping")
    mapping.location = (-900, -200)
    mapping.inputs["Scale"].default_value = (detail_scale, detail_scale, detail_scale)
    links.new(uv_node.outputs["UV"], mapping.inputs["Vector"])

    color_out = base_node.outputs["Color"]
    if maps.get("color"):
        detail_color_img = bpy.data.images.load(maps["color"])
        detail_color_img.colorspace_settings.name = 'sRGB'
        detail_node = nodes.new("ShaderNodeTexImage")
        detail_node.image = detail_color_img
        detail_node.location = (-700, -50)
        links.new(mapping.outputs["Vector"], detail_node.inputs["Vector"])
        # OVERLAY, not MULTIPLY -- MULTIPLY darkens by the detail texture's
        # own value at every pixel, so tiling it smaller (more repeats, the
        # whole point of a detail texture) stamps more high-frequency noise
        # multiplicatively over the base image and visibly muddies its
        # large-scale crater contrast. OVERLAY blends around each pixel's
        # own midtone instead, which is why it's the standard choice for
        # combining a macro base with a tiled detail layer: contrast from
        # the base survives regardless of how finely the detail repeats.
        # Fac is also much lower (a detail layer should be a subtle
        # influence, not compete with the base for visual weight).
        mix = nodes.new("ShaderNodeMixRGB")
        mix.blend_type = 'OVERLAY'
        mix.inputs["Fac"].default_value = 0.25
        mix.location = (-350, 150)
        links.new(base_node.outputs["Color"], mix.inputs["Color1"])
        links.new(detail_node.outputs["Color"], mix.inputs["Color2"])
        color_out = mix.outputs["Color"]
    links.new(color_out, bsdf.inputs["Base Color"])

    if maps.get("normal"):
        normal_img = bpy.data.images.load(maps["normal"])
        normal_img.colorspace_settings.name = 'Non-Color'
        normal_tex = nodes.new("ShaderNodeTexImage")
        normal_tex.image = normal_img
        normal_tex.location = (-700, -350)
        links.new(mapping.outputs["Vector"], normal_tex.inputs["Vector"])
        normal_map = nodes.new("ShaderNodeNormalMap")
        normal_map.location = (-350, -350)
        links.new(normal_tex.outputs["Color"], normal_map.inputs["Color"])
        links.new(normal_map.outputs["Normal"], bsdf.inputs["Normal"])

    if maps.get("roughness"):
        rough_img = bpy.data.images.load(maps["roughness"])
        rough_img.colorspace_settings.name = 'Non-Color'
        rough_tex = nodes.new("ShaderNodeTexImage")
        rough_tex.image = rough_img
        rough_tex.location = (-700, -600)
        links.new(mapping.outputs["Vector"], rough_tex.inputs["Vector"])
        links.new(rough_tex.outputs["Color"], bsdf.inputs["Roughness"])

    return mat


def build_shape(shape, body_empty, name, mesh_cache, mesh_dir, mesh_map, fit_aabb, si):
    """One visual shape of a body: a mesh (or wire-cube placeholder) parented
    through a "shape frame" holder Empty, matching `world = body_pose *
    shape_frame` (see the dataset README's "A body is not one mesh at its
    origin" convention)."""
    child = None
    if shape.get("type") == "box":
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        child = bpy.context.active_object
        size = shape.get("size") or [0.1, 0.1, 0.1]
        child.scale = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
    elif shape.get("type") == "cylinder":
        # Blender's cylinder primitive is centered at its own origin and
        # runs along local Z, `depth` long -- exactly the frame the shape's
        # own pos/rot (applied via the holder Empty below) already expects,
        # same as every other shape type here.
        radius = shape.get("radius", 0.05)
        height = shape.get("height", 0.1)
        bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=height)
        child = bpy.context.active_object
    else:
        src = shape.get("file")
        if src:
            target = resolve_mesh(src, name, mesh_dir, mesh_map)
            template = mesh_cache.get(target) if target else None
            if template is not None:
                child = template.copy()
                child.data = template.data
                bpy.context.scene.collection.objects.link(child)
                if fit_aabb and shape.get("aabb_min"):
                    fit_to_aabb(child, shape["aabb_min"], shape["aabb_max"])
    if child is None:
        # No mesh (a primitive we don't special-case, or a file we couldn't
        # load): a wire box of the recorded size beats leaving nothing.
        size = None
        if shape.get("aabb_min") and shape.get("aabb_max"):
            size = [shape["aabb_max"][i] - shape["aabb_min"][i] for i in range(3)]
        size = size or shape.get("size") or [0.1, 0.1, 0.1]
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        child = bpy.context.active_object
        child.scale = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
        child.display_type = 'WIRE'
    child.name = f"{name}.shape{si}"

    color = shape.get("color")
    if color and child.type == 'MESH':
        # Always override, not just when the mesh has no material yet: an
        # OBJ importer still creates a placeholder material for every
        # `usemtl` name it finds even when the actual .mtl file fails to
        # load (this archive ships OBJs with no .mtl sidecars at all) --
        # that placeholder defaults to flat gray, which silently looked like
        # "no texture" while actually blocking the real manifest color from
        # ever being applied.
        child.data.materials.clear()
        child.data.materials.append(get_color_material(color))

    holder = bpy.data.objects.new(f"{name}.frame{si}", None)
    holder.empty_display_size = 0.05
    holder.rotation_mode = 'QUATERNION'
    holder.location = shape.get("pos", [0, 0, 0])
    holder.rotation_quaternion = mathutils.Quaternion(shape.get("rot", [1, 0, 0, 0]))
    bpy.context.scene.collection.objects.link(holder)
    holder.parent = body_empty
    child.parent = holder
    return child


def build_bodies(objects, rank_label, group, mesh_cache, mesh_dir, mesh_map, fit_aabb):
    """One Empty per recorded body, its shapes parented through per-shape
    frame holders (see build_shape). Returns {index: empty}. No keyframes
    here -- see BodyBaker below."""
    root = bpy.data.objects.new(rank_label, None)
    root.empty_display_size = 1.0
    bpy.context.scene.collection.objects.link(root)

    wanted = set(group) if group else None
    empties = {}
    for obj in sorted(objects, key=lambda o: o["index"]):
        if wanted is not None and obj["group"] not in wanted:
            continue
        name = f"{rank_label}.{obj['group']}.{obj['part']}"
        empty = bpy.data.objects.new(name, None)
        empty.empty_display_type = 'ARROWS'
        empty.empty_display_size = 0.25
        empty.rotation_mode = 'QUATERNION'
        empty.parent = root
        bpy.context.scene.collection.objects.link(empty)
        empties[obj["index"]] = empty
        for si, shape in enumerate(obj.get("shapes", [])):
            build_shape(shape, empty, name, mesh_cache, mesh_dir, mesh_map, fit_aabb, si)
    return empties


class BodyBaker:
    """Buffers every body's pose samples as flat `array('d')`s while frames
    stream in, then writes them to real F-Curves in one bulk pass -- the
    same approach blend_from_chrono_export.py's SceneBuilder uses, and for
    the same reason: `keyframe_insert()` once per body per frame does not
    scale to tens of thousands of frames (the dataset's own blender_import.py
    does exactly that, per-frame, and is the thing this class replaces).
    """

    def __init__(self, empties, fps):
        self.empties = empties
        self.fps = fps
        self.frames = {idx: array.array('d') for idx in empties}
        self.locs = {idx: array.array('d') for idx in empties}
        self.rots = {idx: array.array('d') for idx in empties}

    def add_frame(self, time, poses):
        frame_no = int(round(time * self.fps)) + 1
        for idx, (pos, quat) in poses.items():
            if idx not in self.empties:
                continue
            frames, locs, rots = self.frames[idx], self.locs[idx], self.rots[idx]
            # Same dedup as blend_from_chrono_export.py: this format re-lists
            # every currently-existing body every frame even when it hasn't
            # moved (a stalled/retired builder can sit still for most of a
            # 1441s run -- see the dataset README's "Builders retire" note),
            # so skipping an unchanged sample keeps those F-curves from
            # growing by one point per frame for no reason.
            if locs and tuple(locs[-3:]) == pos and tuple(rots[-4:]) == quat:
                continue
            if frames and frames[-1] == frame_no:
                # self.fps can be lower than the recording's own rate_hz (60
                # native, 30 requested is the common case), so two raw
                # samples can round to the same target frame number. Keep the
                # later one rather than appending both -- two keyframe points
                # at the same F-curve x position is an invalid/ambiguous
                # curve, not just a wasted point.
                locs[-3:] = array.array('d', pos)
                rots[-4:] = array.array('d', quat)
                continue
            frames.append(float(frame_no))
            locs.extend(pos)
            rots.extend(quat)

    def bake(self):
        total = len(self.empties)
        n = 0
        max_frame = 1
        for idx, empty in self.empties.items():
            frames = self.frames.pop(idx)
            locs = self.locs.pop(idx)
            rots = self.rots.pop(idx)
            n += 1
            if not frames:
                continue  # body never appeared in the requested frame range
            max_frame = max(max_frame, int(frames[-1]))
            num_frames = len(frames)

            empty.location = locs[0:3]
            empty.rotation_quaternion = rots[0:4]
            empty.keyframe_insert(data_path="location", frame=frames[0])
            empty.keyframe_insert(data_path="rotation_quaternion", frame=frames[0])

            channelbag = empty.animation_data.action.layers[0].strips[0].channelbags[0]

            def bulk_fill(data_path, num_channels, flat_values):
                for ci in range(num_channels):
                    fc = channelbag.fcurves.find(data_path, index=ci)
                    fc.keyframe_points.add(num_frames - 1)
                    co = array.array('d', (0.0,)) * (2 * num_frames)
                    co[0::2] = frames
                    co[1::2] = flat_values[ci::num_channels]
                    fc.keyframe_points.foreach_set('co', co)
                    fc.update()

            bulk_fill("location", 3, locs)
            bulk_fill("rotation_quaternion", 4, rots)

            first_frame = int(frames[0])
            if first_frame > 1:
                # Hide bodies that spawn mid-run (harvested rocks) until
                # their first recorded frame, same convention as
                # blend_from_chrono_export.py's add_popin_visibility.
                empty.hide_viewport = True
                empty.hide_render = True
                empty.keyframe_insert(data_path="hide_viewport", frame=first_frame - 1)
                empty.keyframe_insert(data_path="hide_render", frame=first_frame - 1)
                empty.hide_viewport = False
                empty.hide_render = False
                empty.keyframe_insert(data_path="hide_viewport", frame=first_frame)
                empty.keyframe_insert(data_path="hide_render", frame=first_frame)
                for data_path in ("hide_viewport", "hide_render"):
                    fc = channelbag.fcurves.find(data_path)
                    for kp in fc.keyframe_points:
                        kp.interpolation = 'CONSTANT'

            if n % 200 == 0:
                print(f"  baked keyframes for {n}/{total} bodies")
        return max_frame


# ------------------------------------------------------------------- static props

# Fallback flat-terrain height when no real heightmap is available. Not the
# aabb's own midpoint (that spans the WHOLE 1024x1024m terrain including
# crater walls/hills far outside the work site) -- the dataset README states
# the actual site surface (where every vehicle in this recording drives)
# runs roughly z=2..6m, and this is the middle of that band.
SITE_GROUND_Z_FALLBACK = 4.0


def build_heightfield_mesh(rows, lo, hi, normalize_observed):
    """A real terrain mesh from heightmap `rows` (read_heightmap's output,
    already 0..1 by pixel bit depth): one vertex per pixel, gridded across
    `lo`..`hi`'s X/Y extent, height mapped onto `lo`..`hi`'s own Z range.

    Two mapping modes, matching the reference tool's own two cases
    (AMD-UW_SBEL's tools/replay_run.py, terrain_mesh()):

    `normalize_observed=True` -- when `lo`/`hi` come from a real recorded
    aabb (a terrain shape found in static_props.jsonl): the pixel's OWN
    observed min..max grey value maps onto the aabb's Z range. NOT
    meta.json's declared min_height/max_height -- confirmed wrong on a real
    run (declared [-25, 25], actual aabb z=-13.82..12.65), because the sim's
    mapping runs over the image's own grey range onto whatever patch it
    actually built, not the declared range.

    `normalize_observed=False` -- when no aabb exists at all (this dataset's
    static_props.jsonl doesn't always carry a terrain shape) and `lo`/`hi`
    were synthesized from meta.json's declared numbers instead: raw pixel
    value maps directly onto that declared range, same as the reference
    tool's own no-aabb fallback -- there's no better information available
    in that case, declared numbers are all there is."""
    h = len(rows)
    w = len(rows[0]) if h else 0
    if normalize_observed:
        grey_min = min(v for row in rows for v in row)
        grey_span = max(v for row in rows for v in row) - grey_min
    else:
        grey_min, grey_span = 0.0, 1.0
    mesh = bpy.data.meshes.new("terrain_heightfield")
    bm = bmesh.new()
    uv_layer = bm.loops.layers.uv.new()
    grid = [[None] * w for _ in range(h)]
    x0, y0 = lo[0], lo[1]
    length, width = hi[0] - lo[0], hi[1] - lo[1]
    for r in range(h):
        for c in range(w):
            x = x0 + (c / (w - 1)) * length if w > 1 else (lo[0] + hi[0]) / 2
            # `r` counts down from the image's own top row (rows[0], per
            # read_heightmap's plain top-down scanline order) -- but the
            # reference tool (replay_run.py's terrain_mesh()) loads the same
            # heightmap through pyvista/VTK, which flips PNGs vertically on
            # load (row 0 becomes the file's BOTTOM row; confirmed
            # empirically against a real VTK read, not assumed), and maps
            # ITS row 0 to the lowest Y. So the file's bottom row is what
            # belongs at y0, not the top row -- inverting this put the
            # terrain's real features (the graded pad, the crater layout)
            # upside-down relative to the vehicles' real (unaffected, from
            # trajectory data) positions.
            y = y0 + ((h - 1 - r) / (h - 1)) * width if h > 1 else (lo[1] + hi[1]) / 2
            z = (lo[2] + (hi[2] - lo[2]) * (rows[r][c] - grey_min) / grey_span
                if grey_span > 0 else lo[2])
            grid[r][c] = bm.verts.new((x, y, z))
    bm.verts.ensure_lookup_table()
    for r in range(h - 1):
        for c in range(w - 1):
            face = bm.faces.new((grid[r][c], grid[r][c + 1], grid[r + 1][c + 1], grid[r + 1][c]))
            # UV = grid position 0..1; v flipped (row 0 is the image's top
            # row, and image row-major top-down is v=1 in Blender's UV
            # convention, which puts v=0 at the bottom) so a texture applied
            # here reads right-side-up rather than mirrored top-to-bottom.
            corners = ((r, c), (r, c + 1), (r + 1, c + 1), (r + 1, c))
            for loop, (rr, cc) in zip(face.loops, corners):
                loop[uv_layer].uv = (cc / (w - 1) if w > 1 else 0.0,
                                     1.0 - (rr / (h - 1) if h > 1 else 0.0))
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new("terrain_heightfield", mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def load_scm_terrain(dataset, ranks):
    """Every disturbed SCM node across `ranks`' scm.bin files, merged into
    one {(i, j): z} dict (see read_scm) -- later-touched wins where two
    ranks' patches happen to share a node (rare: ~2% of cells in a full
    15-rank run per replay_run.py's own measurement), tracked by keeping
    whichever rank recorded a HIGHER final z last, since node-write order
    across separate per-rank files isn't otherwise comparable. Returns
    (delta, plane, nodes), using the first file's delta/plane -- every file
    seen shares one delta and an identity plane, so this doesn't attempt to
    reconcile a mismatch, just uses whichever came first."""
    delta = plane = None
    nodes = {}
    for rank in ranks:
        path = os.path.join(dataset, f"rank_{rank}_scm.bin")
        if not os.path.exists(path):
            continue
        try:
            d, p, rank_nodes = read_scm(path)
        except (OSError, ValueError) as exc:
            print(f"  warning: {path}: {exc}")
            continue
        if delta is None:
            delta, plane = d, p
        nodes.update(rank_nodes)
    return delta, plane, nodes


def build_scm_terrain_mesh(nodes, delta, plane, uv_tile_meters=2.0):
    """A mesh from real SCM deformation nodes (load_scm_terrain's output):
    one quad per 2x2 block of nodes that are ALL present, so genuinely
    disconnected patches (a rover's ruts are a lane, not a filled area --
    see replay_run.py's deformed_cells docstring) stay disconnected instead
    of being bridged across real gaps with fabricated geometry.

    UVs come straight from each node's own (i, j) grid index * delta (its
    real world-space spacing), divided by `uv_tile_meters` -- unlike the
    heightfield mesh (one clean rectangular grid, 0..1 across the whole
    patch), this mesh is a sparse, irregularly-shaped set of disconnected
    quads, so there's no single patch extent to normalize against; tiling
    by a fixed real-world distance instead keeps texture density consistent
    regardless of how big or small a given rank's disturbed area is."""
    mesh = bpy.data.meshes.new("scm_terrain")
    bm = bmesh.new()
    uv_layer = bm.loops.layers.uv.new()
    verts = {}
    for (i, j), z in nodes.items():
        verts[(i, j)] = bm.verts.new(scm_node_world_pos(i, j, z, delta, plane))
    bm.verts.ensure_lookup_table()
    n_faces = 0
    for (i, j) in nodes:
        v00 = verts.get((i, j))
        v10 = verts.get((i + 1, j))
        v01 = verts.get((i, j + 1))
        v11 = verts.get((i + 1, j + 1))
        if v10 is not None and v01 is not None and v11 is not None:
            face = bm.faces.new((v00, v10, v11, v01))
            for loop, (ii, jj) in zip(face.loops, ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1))):
                loop[uv_layer].uv = (ii * delta / uv_tile_meters, jj * delta / uv_tile_meters)
            n_faces += 1
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new("scm_terrain", mesh)
    bpy.context.scene.collection.objects.link(obj)
    print(f"  built SCM terrain: {len(nodes)} nodes, {n_faces} quads")
    return obj


def build_scm_color_material(detail_folder, grey_shift=0.3):
    """A regolith material for the SCM rut mesh: `detail_folder`'s color and
    normal maps (auto-detected via find_pbr_maps, tiled through the mesh's
    own UVs -- see build_scm_terrain_mesh), mixed toward neutral grey by
    `grey_shift` (0=texture's own color unchanged, 1=flat grey) -- unlike
    the terrain's macro+detail blend, there's no separate base photo here to
    preserve contrast against, so this is a plain color mix rather than an
    OVERLAY (see build_terrain_detail_material's docstring for why OVERLAY
    matters there and not here)."""
    maps = find_pbr_maps(detail_folder)
    print(f"  scm terrain texture from {detail_folder}: {[k for k, v in maps.items() if v]}")

    mat = bpy.data.materials.new("scm_terrain_color")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    bsdf = nodes.get("Principled BSDF")

    color_out = None
    if maps.get("color"):
        color_img = bpy.data.images.load(maps["color"])
        color_img.colorspace_settings.name = 'sRGB'
        color_node = nodes.new("ShaderNodeTexImage")
        color_node.image = color_img
        color_node.location = (-500, 200)
        grey = nodes.new("ShaderNodeMixRGB")
        grey.blend_type = 'MIX'
        grey.inputs["Fac"].default_value = grey_shift
        grey.inputs["Color2"].default_value = (0.5, 0.5, 0.5, 1.0)
        grey.location = (-200, 200)
        links.new(color_node.outputs["Color"], grey.inputs["Color1"])
        color_out = grey.outputs["Color"]
    if color_out is not None:
        links.new(color_out, bsdf.inputs["Base Color"])

    if maps.get("normal"):
        normal_img = bpy.data.images.load(maps["normal"])
        normal_img.colorspace_settings.name = 'Non-Color'
        normal_tex = nodes.new("ShaderNodeTexImage")
        normal_tex.image = normal_img
        normal_tex.location = (-500, -100)
        normal_map = nodes.new("ShaderNodeNormalMap")
        normal_map.location = (-200, -100)
        links.new(normal_tex.outputs["Color"], normal_map.inputs["Color"])
        links.new(normal_map.outputs["Normal"], bsdf.inputs["Normal"])

    if maps.get("roughness"):
        rough_img = bpy.data.images.load(maps["roughness"])
        rough_img.colorspace_settings.name = 'Non-Color'
        rough_tex = nodes.new("ShaderNodeTexImage")
        rough_tex.image = rough_img
        rough_tex.location = (-500, -350)
        links.new(rough_tex.outputs["Color"], bsdf.inputs["Roughness"])

    return mat


def build_static_props(dataset, mesh_cache, mesh_dir, mesh_map, fit_aabb):
    """`static_props.jsonl` bodies (orbit rings, centre pad, decorative
    already-laid rocks, and -- on some archives, not all -- a terrain
    shape) placed once at their t=0 pose -- they never move, so no
    keyframes at all. A terrain shape with no mesh file (this dataset's
    ground is a heightmap image, not an OBJ) is skipped here and returned
    separately: see build_terrain, which handles it whether or not this
    file happens to carry one (this new run's static_props.jsonl has no
    terrain entry at all -- only rings/pad/rocks -- unlike an earlier
    archive of the same project, so this can't assume it's always there)."""
    props = load_jsonl(os.path.join(dataset, "static_props.jsonl"))
    if not props:
        return [], None
    root = bpy.data.objects.new("static_props", None)
    root.empty_display_size = 1.0
    bpy.context.scene.collection.objects.link(root)

    built = []
    terrain_info = None
    for obj in props:
        name = f"static.{obj['group']}.{obj['part']}"
        body_empty = bpy.data.objects.new(name, None)
        body_empty.empty_display_type = 'PLAIN_AXES'
        body_empty.rotation_mode = 'QUATERNION'
        body_empty.location = obj.get("first_pos", [0, 0, 0])
        body_empty.rotation_quaternion = mathutils.Quaternion(obj.get("first_rot", [1, 0, 0, 0]))
        body_empty.parent = root
        bpy.context.scene.collection.objects.link(body_empty)

        for si, shape in enumerate(obj.get("shapes", [])):
            if shape.get("type") == "trimesh" and not shape.get("file"):
                if terrain_info is None:
                    terrain_info = (shape, name, si, body_empty)
                continue
            child = build_shape(shape, body_empty, name, mesh_cache, mesh_dir, mesh_map,
                                fit_aabb, si)
            built.append(child)
        built.append(body_empty)
    print(f"  built {len(props)} static prop(s)")
    return built, terrain_info


def build_terrain(terrain_info, dataset, ranks, mesh_cache, mesh_dir, mesh_map, fit_aabb,
                  heightmap, terrain_texture_image=None, terrain_detail_dir=None,
                  terrain_detail_scale=40.0):
    """The terrain, however it's available -- see the module docstring's
    "About the terrain". `terrain_info` is build_static_props' find (a real
    recorded aabb to fit against) or None (no terrain shape in this file's
    static_props.jsonl at all, e.g. an SCM-only run): falls back to
    meta.json's declared terrain block, centered at the site origin,
    matching the reference tool's own no-aabb fallback (see
    build_heightfield_mesh's docstring for why the height MAPPING also
    differs between these two cases, not just the source of lo/hi)."""
    if terrain_info is not None:
        shape, owner_name, si, body_empty = terrain_info
        lo, hi = shape["aabb_min"], shape["aabb_max"]
        shape_name = shape.get("shape_name", "terrain")
        color = shape.get("color")
        normalize_observed = True
        parent = body_empty
    else:
        meta_path = os.path.join(dataset, f"rank_{ranks[0]}_meta.json")
        with open(meta_path) as f:
            terr = json.load(f).get("terrain", {})
        if not terr:
            return None
        length = float(terr.get("length", 1024.0))
        width = float(terr.get("width", 1024.0))
        min_h = float(terr.get("min_height", 0.0))
        max_h = float(terr.get("max_height", 1.0))
        lo = (-length / 2, -width / 2, min_h)
        hi = (length / 2, width / 2, max_h)
        shape_name = os.path.splitext(os.path.basename(terr.get("heightmap", "terrain")))[0]
        color = None
        normalize_observed = False
        parent = None
        owner_name = "static.world.terrain"
        si = 0

    center = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)
    # `file` is empty (no archive seen has ever had a real terrain OBJ to
    # record a path for), but --mesh-map/--meshes can still supply a real
    # replacement -- e.g. a proper lunar terrain mesh with its own regolith
    # texture -- the same way it would for any other shape. Match against
    # the manifest's `shape_name` since there's no real filename to go by.
    target = resolve_mesh(shape_name + ".obj", owner_name, mesh_dir, mesh_map)
    template = mesh_cache.get(target) if target else None
    if template is not None:
        terrain_obj = template.copy()
        terrain_obj.data = template.data
        bpy.context.scene.collection.objects.link(terrain_obj)
        if fit_aabb:
            fit_to_aabb(terrain_obj, lo, hi)
        print(f"  terrain: using replacement mesh {target}")
    elif heightmap is not None:
        terrain_obj = build_heightfield_mesh(heightmap, lo, hi, normalize_observed)
        print(f"  built real terrain heightfield: {len(heightmap[0])}x{len(heightmap)} verts")
        if terrain_texture_image is not None:
            if terrain_detail_dir is not None:
                mat = build_terrain_detail_material(terrain_texture_image, terrain_detail_dir,
                                                    terrain_detail_scale)
            else:
                mat = build_terrain_color_material(terrain_texture_image)
            terrain_obj.data.materials.append(mat)
            color = None  # the texture wins -- don't also paint the flat manifest color over it
    else:
        # No real heightmap available -- see the module docstring's "About
        # the terrain". Flat placeholder at SITE_GROUND_Z_FALLBACK (the
        # aabb/declared Z range spans the WHOLE terrain including crater
        # walls/hills far outside the work site; the dataset README states
        # the actual site surface runs roughly z=2..6m, and this is the
        # middle of that band), covering the aabb's X/Y extent.
        bpy.ops.mesh.primitive_plane_add(size=1.0, location=(center[0], center[1],
                                                              SITE_GROUND_Z_FALLBACK))
        terrain_obj = bpy.context.active_object
        # primitive_plane_add(size=1.0) has HALF-extent 0.5, so the scale
        # that lands its full extent on (hi - lo) is (hi - lo) itself, not
        # half of it.
        terrain_obj.scale = (hi[0] - lo[0], hi[1] - lo[1], 1.0)
    terrain_obj.name = f"{owner_name}.terrain{si}"
    if parent is not None:
        terrain_obj.parent = parent
    # Only paint the flat manifest color onto our own placeholder geometry
    # -- a real replacement mesh brings its own material (that's the whole
    # point of supplying one), and overwriting it here would throw that away.
    if template is None and color:
        terrain_obj.data.materials.append(get_color_material(color))
    return terrain_obj


# ---------------------------------------------------------------------------- CLI

def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    if len(argv) < 2:
        print(__doc__)
        sys.exit(1)
    dataset, out_path = argv[0], argv[1]
    if os.path.dirname(out_path) == "":
        # A bare filename (no directory component) lands in this repo's own
        # working directory otherwise, since Blender resolves it relative to
        # whatever the shell's cwd was -- not obviously wrong, but not where
        # a .blend belongs (it isn't source, and sits there as an untracked
        # file). Route it to the same folder blend_from_chrono_export.py's
        # outputs live in instead.
        out_path = os.path.join(
            os.path.expanduser("~/Documents/BlenderDocuments/blenderfiles/blendFiles"),
            out_path)
    rest = argv[2:]

    opts = {
        "ranks": None, "groups": None, "mesh_dir": None, "mesh_map": None,
        "fit_aabb": True, "start": None, "end": None, "stride": 1,
        "no_static": False, "textures": [], "part_textures": [], "robot_texture": None,
        "world_append": None, "fps": 30, "heightmap": None, "scm": False,
        "terrain_texture": None, "terrain_detail_texture": None,
        "terrain_detail_scale": 40.0, "scm_texture": None, "scm_grey_shift": 0.3,
        "appends": [],
    }
    i = 0
    while i < len(rest):
        arg = rest[i]
        if arg == "--fps":
            opts["fps"] = int(rest[i + 1])
            i += 2
        elif arg == "--ranks":
            opts["ranks"] = None if rest[i + 1] == "all" else [int(r) for r in rest[i + 1].split(",")]
            i += 2
        elif arg == "--groups":
            opts["groups"] = rest[i + 1].split(",")
            i += 2
        elif arg == "--meshes":
            opts["mesh_dir"] = rest[i + 1]
            i += 2
        elif arg == "--mesh-map":
            with open(rest[i + 1]) as f:
                opts["mesh_map"] = json.load(f)
            i += 2
        elif arg == "--no-fit":
            opts["fit_aabb"] = False
            i += 1
        elif arg == "--start":
            opts["start"] = float(rest[i + 1])
            i += 2
        elif arg == "--end":
            opts["end"] = float(rest[i + 1])
            i += 2
        elif arg == "--stride":
            opts["stride"] = int(rest[i + 1])
            i += 2
        elif arg == "--no-static":
            opts["no_static"] = True
            i += 1
        elif arg == "--heightmap":
            opts["heightmap"] = rest[i + 1]
            i += 2
        elif arg == "--terrain-texture":
            opts["terrain_texture"] = rest[i + 1]
            i += 2
        elif arg == "--terrain-detail-texture":
            opts["terrain_detail_texture"] = rest[i + 1]
            i += 2
        elif arg == "--terrain-detail-scale":
            opts["terrain_detail_scale"] = float(rest[i + 1])
            i += 2
        elif arg == "--scm":
            opts["scm"] = True
            i += 1
        elif arg == "--scm-texture":
            opts["scm_texture"] = rest[i + 1]
            i += 2
        elif arg == "--scm-grey-shift":
            opts["scm_grey_shift"] = float(rest[i + 1])
            i += 2
        elif arg == "--texture":
            opts["textures"].append((rest[i + 1], rest[i + 2].split(",")))
            i += 3
        elif arg == "--part-texture":
            opts["part_textures"].append((rest[i + 1], rest[i + 2].split(",")))
            i += 3
        elif arg == "--robot-texture":
            excludes = [f for f in rest[i + 2].split(",") if f]
            opts["robot_texture"] = (rest[i + 1], excludes)
            i += 3
        elif arg == "--world":
            opts["world_append"] = (rest[i + 1], rest[i + 2])
            i += 3
        elif arg == "--append":
            opts["appends"].append((rest[i + 1], rest[i + 2]))
            i += 3
        else:
            print(f"unrecognized argument: {arg}")
            sys.exit(1)
    return dataset, out_path, opts


def main():
    dataset, out_path, opts = parse_args()
    ranks = opts["ranks"] or discover_ranks(dataset)
    print(f"loading ranks {ranks} from {dataset} ...")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'

    mesh_cache = MeshCache()
    all_texturable = []      # (group, object) pairs for --texture matching
    max_frame_overall = 1
    fps = opts["fps"]

    for rank in ranks:
        objects = load_manifest(dataset, rank)
        rank_label = f"rank_{rank}"
        empties = build_bodies(objects, rank_label, opts["groups"], mesh_cache,
                               opts["mesh_dir"], opts["mesh_map"], opts["fit_aabb"])
        by_index = {o["index"]: o for o in objects}
        for idx, empty in empties.items():
            all_texturable.append((by_index[idx]["group"], empty))

        baker = BodyBaker(empties, fps)
        n_frames = 0
        for time, poses in read_frames(dataset, rank, opts["start"], opts["end"], opts["stride"]):
            baker.add_frame(time, poses)
            n_frames += 1
            if n_frames % 5000 == 0:
                print(f"  rank {rank}: read {n_frames} frames, t={time:.1f}")
        print(f"  rank {rank}: {n_frames} frames read, {len(empties)} bodies -> baking")
        max_frame_overall = max(max_frame_overall, baker.bake())

    scene.render.fps = fps or 60
    scene.frame_start = 1
    scene.frame_end = max_frame_overall
    print(f"timeline: frames 1..{max_frame_overall} at {scene.render.fps} fps")

    heightmap = None
    if opts["heightmap"]:
        heightmap = read_heightmap(opts["heightmap"])
        print(f"  loaded heightmap {opts['heightmap']}: {len(heightmap[0])}x{len(heightmap)} px")

    terrain_texture_image = None
    if opts["terrain_texture"]:
        terrain_texture_image = bpy.data.images.load(os.path.abspath(opts["terrain_texture"]))
        print(f"  loaded terrain texture {opts['terrain_texture']}: "
             f"{terrain_texture_image.size[0]}x{terrain_texture_image.size[1]} px")

    if not opts["no_static"]:
        static_objs, terrain_info = build_static_props(dataset, mesh_cache, opts["mesh_dir"],
                                                        opts["mesh_map"], opts["fit_aabb"])
        terrain_obj = build_terrain(terrain_info, dataset, ranks, mesh_cache, opts["mesh_dir"],
                                    opts["mesh_map"], opts["fit_aabb"], heightmap,
                                    terrain_texture_image, opts["terrain_detail_texture"],
                                    opts["terrain_detail_scale"])
        if terrain_obj is not None:
            static_objs.append(terrain_obj)
        for obj in static_objs:
            name = obj.name.split(".")[1] if obj.name.startswith("static.") else ""
            all_texturable.append((name, obj))

    if opts["scm"]:
        delta, plane, nodes = load_scm_terrain(dataset, ranks)
        print(f"  loaded SCM deformation: {len(nodes)} node(s) across rank(s) {ranks}")
        if nodes:
            scm_obj = build_scm_terrain_mesh(nodes, delta, plane)
            if opts["scm_texture"]:
                mat = build_scm_color_material(opts["scm_texture"], opts["scm_grey_shift"])
                scm_obj.data.materials.append(mat)
            else:
                # No texture supplied -- same flat gray the undisturbed
                # terrain patch reports in static_props.jsonl (this data
                # carries no color of its own).
                scm_obj.data.materials.append(get_color_material((0.55, 0.5, 0.45)))
            all_texturable.append(("scm_terrain", scm_obj))

    for append_path, collection_name in opts["appends"]:
        append_collection(append_path, collection_name)

    if opts["world_append"]:
        append_world(*opts["world_append"])

    # ---- PBR texturing ----
    mesh_objs_by_group = {}

    def mesh_descendants(obj):
        """Every MESH object under `obj` -- shapes hang two levels below a
        body Empty (body -> shape-frame holder -> mesh), so this can't just
        look at direct children."""
        out = []
        if obj.type == 'MESH':
            out.append(obj)
        for child in obj.children:
            out.extend(mesh_descendants(child))
        return out

    for group, obj in all_texturable:
        mesh_objs_by_group.setdefault(group, []).extend(mesh_descendants(obj))

    for folder, families in opts["textures"]:
        mat = build_pbr_material_from_folder("_".join(families) + "_pbr", folder)
        targets = []
        for fam in families:
            targets.extend(mesh_objs_by_group.get(fam, []))
        smart_uv_unwrap(targets)
        for obj in targets:
            obj.data.materials.clear()
            obj.data.materials.append(mat)
        print(f"  applied material '{mat.name}' to {len(targets)} object(s) across {families}")

    # Objects matched by --part-texture (name substring, not group -- e.g.
    # track shoes, which live inside the "builder"/"collector" groups
    # rather than having a group of their own) are painted here and then
    # excluded from --robot-texture below, same split
    # blend_from_chrono_export.py uses for track shoes vs. everything else
    # (plastic tread, metal body).
    part_textured = set()
    for folder, substrings in opts["part_textures"]:
        mat = build_pbr_material_from_folder("_".join(substrings) + "_pbr", folder)
        targets = [o for objs in mesh_objs_by_group.values() for o in objs
                  if any(s in o.name for s in substrings)]
        smart_uv_unwrap(targets)
        for obj in targets:
            obj.data.materials.clear()
            obj.data.materials.append(mat)
            part_textured.add(obj)
        print(f"  applied material '{mat.name}' to {len(targets)} object(s) matching {substrings}")

    def has_real_color(obj):
        """True if `obj`'s current material is a genuinely distinguishing
        (non-gray) manifest color -- e.g. the chassis's reddish paint --
        rather than an achromatic near-white/gray/black placeholder (like
        the many parts whose manifest color happened to be literal white).
        Robot-texturing skips these so real color-coded parts aren't
        overwritten with a generic metal material."""
        if not obj.data.materials or obj.data.materials[0] is None:
            return False
        mat = obj.data.materials[0]
        if not mat.use_nodes or not mat.node_tree:
            return False
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf is None or bsdf.inputs["Base Color"].is_linked:
            return False
        r, g, b = bsdf.inputs["Base Color"].default_value[:3]
        return (max(r, g, b) - min(r, g, b)) > 0.05

    if opts["robot_texture"]:
        folder, excludes = opts["robot_texture"]
        mat = build_pbr_material_from_folder("robot_pbr", folder)
        excluded_groups = set(excludes)
        targets = [o for g, objs in mesh_objs_by_group.items() if g not in excluded_groups
                  for o in objs if g != "rock" and not g.startswith("static")
                  and not has_real_color(o) and o not in part_textured]
        smart_uv_unwrap(targets)
        for obj in targets:
            obj.data.materials.clear()
            obj.data.materials.append(mat)
        print(f"  applied material 'robot_pbr' to {len(targets)} object(s), excluding {excluded_groups}"
             " (and any part with its own distinguishing color)")

    # ---- render settings, matching blend_from_chrono_export.py's conventions ----
    scene.render.engine = 'BLENDER_EEVEE'
    if hasattr(scene, "eevee"):
        if hasattr(scene.eevee, "use_raytracing"):
            scene.eevee.use_raytracing = True
        if hasattr(scene.eevee, "shadow_step_count"):
            scene.eevee.shadow_step_count = 10
    scene.cycles.device = 'GPU'
    scene.render.resolution_percentage = 250

    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out_path))
    print(f"saved {out_path} ({max_frame_overall} frames, {len(ranks)} rank(s))")


if __name__ == "__main__":
    main()
