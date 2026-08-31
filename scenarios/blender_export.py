"""Time-throttled Chrono::Postprocess Blender export for the scenarios.

`pychrono.postprocess.ChBlender` writes one `<name>.assets.py` (the shared
meshes and materials) plus `output/stateNNNNN.py/.dat` per exported frame (every
body's pose), which Blender's `chrono_import.py` add-on turns back into a scene
that can be scrubbed and rendered -- see `scripts/render/README.md` for that half.

This module keeps the exporter setup and the frame cadence in one place so a
scenario only has to create a `BlenderFrameExporter` and call it once per
simulation step; it writes at most one frame per `1 / fps` seconds of sim time.

Two things the raw exporter does not do on its own, both handled here:

  * Bodies spawned mid-run are picked up. The scenarios add thousands of bodies
    after the scene is built (the wall dumps, mode 2's rock load), so every
    body in the system is (re-)registered with the exporter before each frame.
    `Add` is a set insert, so re-adding is free, and `ExportData` appends any
    shape it has not seen to the assets file, so a shape that first appears at
    frame 500 still ends up in the (single) assets file the add-on loads.
  * Only bodies are exported -- not links, meshes or other physics items. The
    scenarios' links (motors, the gripper lock, the hull pin) have nothing to
    draw, and exporting them writes `chrono_view_link_csys` globals the add-on
    does not define, which trips its import.

Adapted from NeDM's `src/nedm/blender_export.py`.
"""

import json
import shutil
from pathlib import Path

import pychrono as chrono
import pychrono.postprocess as postprocess


def _vec(values):
    return chrono.ChVector3d(float(values[0]), float(values[1]), float(values[2]))


class BlenderFrameExporter:
    """Throttled wrapper around `postprocess.ChBlender`; call it every step."""

    def __init__(self, system, output_dir, fps=7.5, max_frames=None,
                 script_name="exported", picture_size=(1280, 720),
                 camera_location=(-10.0, -12.0, 5.0), camera_aim=(0.0, 0.0, 1.0),
                 camera_angle_deg=45.0, light_location=(-10.0, -8.0, 12.0),
                 clean=True):
        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")
        self.system = system
        self.output_dir = Path(output_dir).expanduser().resolve()
        if clean:
            self._clean_known_outputs(self.output_dir, script_name)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.fps = float(fps)
        self.period_s = 1.0 / self.fps
        self.max_frames = int(max_frames) if max_frames else None
        self.script_name = script_name
        self.frame_count = 0
        self.frame_times = []
        self.next_export_time_s = float(system.GetChTime())

        self.exporter = postprocess.ChBlender(system)
        self.exporter.SetBasePath(str(self.output_dir))
        self.exporter.SetOutputScriptFile(script_name)
        self.exporter.SetPictureFilebase("picture")
        self.exporter.SetPictureSize(int(picture_size[0]), int(picture_size[1]))
        self.exporter.SetBlenderUp_is_ChronoZ()
        # The camera/light/background here are only the add-on's defaults; the
        # render script replaces all of them.
        self.exporter.SetCamera(_vec(camera_location), _vec(camera_aim),
                                float(camera_angle_deg), False)
        self.exporter.SetLight(_vec(light_location), chrono.ChColor(1.0, 1.0, 1.0), True)
        self.exporter.SetBackground(chrono.ChColor(0.78, 0.84, 0.90))
        self.exporter.SetShowItemsFrames(False, 0.25)
        self.exporter.SetShowLinksFrames(False, 0.0)
        self.exporter.SetShowContactsOff()
        self.exporter.SetUseSingleAssetFile(True)
        self._add_render_items()
        self.exporter.ExportScript()

    # ---- per-step entry points ----
    def __call__(self):
        return self.maybe_export()

    def maybe_export(self, force=False):
        """Export a frame if `1 / fps` of sim time has passed since the last one."""
        if self.max_frames is not None and self.frame_count >= self.max_frames:
            return False
        time_s = float(self.system.GetChTime())
        if not force and time_s + 1.0e-9 < self.next_export_time_s:
            return False
        self._add_render_items()
        self.exporter.ExportData()
        self.frame_count += 1
        self.frame_times.append(time_s)
        if force:
            self.next_export_time_s = time_s + self.period_s
        else:
            while self.next_export_time_s <= time_s + 1.0e-9:
                self.next_export_time_s += self.period_s
        return True

    # ---- bookkeeping ----
    def summary(self):
        output_path = self.output_dir / "output"
        state_files = sorted(output_path.glob("state*.py")) if output_path.is_dir() else []
        return {
            "output_dir": str(self.output_dir),
            "assets_script": str(self.output_dir / f"{self.script_name}.assets.py"),
            "state_file_count": len(state_files),
            "captured_frames": int(self.frame_count),
            "fps": float(self.fps),
            "max_frames": self.max_frames,
            # Blender frame index -> Chrono sim time, so a render can be aimed
            # at a moment of the demo rather than a frame number.
            "frame_times": [round(t, 4) for t in self.frame_times],
        }

    def write_summary(self, path=None):
        summary_path = (Path(path) if path is not None
                        else self.output_dir / "blender_export_summary.json")
        summary_path.write_text(json.dumps(self.summary(), indent=2), encoding="utf-8")
        return summary_path

    def _add_render_items(self):
        for body in self.system.GetBodies():
            self.exporter.Add(body)

    @staticmethod
    def _clean_known_outputs(output_dir, script_name):
        if not output_dir.exists():
            return
        for child in (output_dir / "output", output_dir / "anim"):
            if child.exists():
                shutil.rmtree(child)
        for file_path in output_dir.glob("*.assets.py"):
            file_path.unlink()
        summary = output_dir / "blender_export_summary.json"
        if summary.exists():
            summary.unlink()
