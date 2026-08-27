"""Attach a Chrono::Postprocess Blender export to any Chrono scenario, unmodified.

to run from my machine:
conda run -n chrono python /home/zacharyrichmond/lunar-manip/scripts/export_any_chrono_run.py \
    /home/zacharyrichmond/lunar-manip/scenarios/TrackedVeh_OrbitBuilder.py --fps 30 \
    -- --headless --run-time 20
    
`TrackedVeh_OrbitBuilder.py` (see its `--export-blender` flag) wires up a
`postprocess.ChBlender` by hand: construct it against the scenario's
`ChSystem`, `AddAll()` the bodies that exist at that point, `ExportScript()`
once, then call `exporter.ExportData()` every ~1/30 s of sim time from inside
its own step loop -- and separately call `exporter.Add(body)` for anything
spawned mid-run (the wall dumps, the crest rocks), because `AddAll()` only
ever sees what existed when it ran.

This script gets the same result for a scenario that has none of that code,
by hooking the mechanism instead of the script. `pychrono.ChSystem` is the
one object practically every Chrono scenario funnels through: bodies reach it
via `Add`/`AddBody` (directly or via a vehicle model's internals), and the
sim loop advances it via `DoStepDynamics` (or, for vehicle scenarios, via a
`.Advance()` call -- see below). Monkey-patching those before the target
script even imports `pychrono` gives the hooks needed:


  * every `Add`/`AddBody` call registers that item with a `ChBlender`
    exporter -- created on the very first call, so this also replaces
    `AddAll()` and the scenario's own mid-run `exporter.Add()` calls in one
    mechanism, without needing to know when "setup" ends.
  * the first step writes the export script (by which point scenario setup
    has run; anything added after this still registers via the `Add` hook
    above).
  * every step exports a frame once sim time has advanced `1/fps` past the
    last export, mirroring the cadence `TrackedVeh_OrbitBuilder.py` uses.

Two things make "every step" harder than one patch on `ChSystem`:

  1. A plain Chrono script calls `system.DoStepDynamics(dt)` straight from
     Python, which that patch catches fine. But a vehicle scenario like
     `TrackedVeh_OrbitBuilder.py` never calls that itself -- it calls
     `m113.Advance(dt)`, which steps the system from *inside compiled C++*.
     A Python-level monkey-patch on `ChSystem` cannot see a call the C++
     side makes to itself without ever going back through Python. And
     `pychrono.vehicle` has no one base class to patch instead:
     `ChTrackedVehicle`/`ChWheeledVehicle` each override `Advance` rather
     than inheriting `ChVehicle`'s, and the top-level convenience wrappers
     scenarios actually instantiate (`veh.M113()`, `veh.HMMWV()`, ...)
     override it again on top of that. So every class in `pychrono.vehicle`
     that both defines its own `Advance` and exposes `GetSystem` gets
     patched -- that combination is exactly "this `Advance` steps some
     system", wherever in the hierarchy a scenario happens to call it.

  2. Whatever fires the step hook, it cannot be matched back to the exact
     `ChSystem` object bodies were added to by comparing identity or even
     `id()`/pointer value: a getter like `.GetSystem()` returns a *new*
     Python proxy every call, and under C++'s multiple inheritance that
     proxy's own pointer can differ numerically from the concrete object's
     (e.g. `ChSystemNSC`'s) -- so `self.GetSystem()` inside a step hook is
     simply not comparable to the `self` a `ChSystem.Add` hook saw. Rather
     than fight that, this script tracks exactly one export session for the
     whole run: the first `Add`/`AddBody` call creates it (and is the system
     every export reads `GetChTime()` from), and every step hook just drives
     that same session forward. This is correct for every scenario in this
     repo (one `ChSystem` for the run) and for the overwhelming majority of
     Chrono scripts in general; a script that truly runs multiple
     independent systems at once would need per-system tracking this script
     does not attempt.

The target script then runs via `runpy`, exactly as if invoked directly
(same argv, same `__main__` guard) -- nothing about it needs to change, and
it never has to import or know about this module.

Usage:

    conda run -n chrono python scripts/export_any_chrono_run.py \\
        scenarios/TrackedVeh_OrbitBuilder.py \\
        [--export-dir DIR] [--fps N] [--no-clean] \\
        [-- <args passed through to the scenario's own argv>]

Everything before `--` belongs to this wrapper; everything after belongs to
the scenario. Example:

    conda run -n chrono python scripts/export_any_chrono_run.py \\
        scenarios/TrackedVeh_OrbitBuilder.py --fps 30 \\
        -- --headless --run-time 20

Without `--export-dir`, the export lands in `<ChronoOutputPath>/BLENDER`,
the same place `--export-blender` writes to -- open it with
`scripts/blend_from_chrono_export.py` as usual.
"""

import argparse
import os
import runpy
import shutil
import sys

import pychrono as chrono
import pychrono.postprocess as postprocess

try:
    import pychrono.vehicle as veh
except ImportError:
    veh = None

DEFAULT_FPS = 30.0

# The one export session tracked for this run -- see the module docstring for
# why this deliberately doesn't try to key state per-ChSystem.
_STATE = None


def _ensure_state(system, base_path, fps):
    """Create the export session on the first `Add`/`AddBody` call, if needed."""
    global _STATE
    if _STATE is not None:
        return _STATE
    os.makedirs(base_path, exist_ok=True)
    exporter = postprocess.ChBlender(system)
    exporter.SetBasePath(base_path)
    exporter.SetBlenderUp_is_ChronoZ()
    _STATE = {
        "exporter": exporter,
        "system": system,
        "base_path": base_path,
        "period": 1.0 / fps,
        "next_export_time": None,
        "script_written": False,
    }
    print(f"[export-any-chrono-run] tracking system -> exporting to {base_path}")
    return _STATE


def _register(system, item, base_path, fps):
    state = _ensure_state(system, base_path, fps)
    try:
        state["exporter"].Add(item)
    except Exception as exc:
        # Not every ChSystem item is something ChBlender knows how to export
        # (e.g. some link/constraint types) -- skip it rather than aborting
        # the whole run over one unexportable item.
        print(f"[export-any-chrono-run] skipped {item!r}: {exc}")


def _on_step():
    """Run once per Python-visible simulation step.

    Writes the export script on the first step seen, then exports a frame
    once `1/fps` of sim time has passed since the last one. Reads time off
    the system captured when the session was created, not off whatever
    object triggered this call -- see the module docstring, point 2.
    """
    if _STATE is None:
        return
    system = _STATE["system"]
    if not _STATE["script_written"]:
        _STATE["exporter"].ExportScript()
        _STATE["script_written"] = True
        _STATE["next_export_time"] = system.GetChTime()
    t = system.GetChTime()
    if t >= _STATE["next_export_time"]:
        _STATE["exporter"].ExportData()
        _STATE["next_export_time"] = t + _STATE["period"]


def install_hooks(base_path, fps):
    """Monkey-patch pychrono so the target scenario's export happens on its own.

    Must run before the target scenario imports `pychrono` and builds its
    system, so `runpy.run_path` in `main()` calls this first.
    """
    original_add = chrono.ChSystem.Add
    original_add_body = chrono.ChSystem.AddBody
    original_step = chrono.ChSystem.DoStepDynamics

    def patched_add(self, item):
        result = original_add(self, item)
        _register(self, item, base_path, fps)
        return result

    def patched_add_body(self, body):
        result = original_add_body(self, body)
        _register(self, body, base_path, fps)
        return result

    def patched_step(self, dt):
        result = original_step(self, dt)
        _on_step()
        return result

    chrono.ChSystem.Add = patched_add
    chrono.ChSystem.AddBody = patched_add_body
    chrono.ChSystem.DoStepDynamics = patched_step

    # Vehicle scenarios step the system from inside a `.Advance()` call
    # instead -- see the module docstring, point 1.
    if veh is not None:
        def make_patched_advance(original):
            def patched_advance(self, dt):
                result = original(self, dt)
                _on_step()
                return result
            return patched_advance

        for name in dir(veh):
            cls = getattr(veh, name)
            if (isinstance(cls, type) and "Advance" in cls.__dict__
                    and hasattr(cls, "GetSystem")):
                cls.Advance = make_patched_advance(cls.__dict__["Advance"])


def main():
    argv = sys.argv[1:]
    if "--" in argv:
        split = argv.index("--")
        own_argv, scenario_argv = argv[:split], argv[split + 1:]
    else:
        own_argv, scenario_argv = argv, []

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", help="path to the Chrono scenario script")
    parser.add_argument("--export-dir", default=None,
                        help="Blender export folder (default: <ChronoOutputPath>/BLENDER)")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS,
                        help=f"export cadence, frames per sim second (default: {DEFAULT_FPS:.0f})")
    parser.add_argument("--no-clean", action="store_true",
                        help="don't delete an existing export dir before writing")
    args = parser.parse_args(own_argv)

    base_path = args.export_dir or (chrono.GetChronoOutputPath() + "BLENDER")
    if not args.no_clean and os.path.exists(base_path):
        shutil.rmtree(base_path)

    install_hooks(base_path, args.fps)

    scenario_path = os.path.abspath(args.scenario)
    # Make the scenario see the same argv and __main__ guard it would get run
    # directly, and let it import sibling modules from its own directory.
    sys.argv = [scenario_path] + scenario_argv
    sys.path.insert(0, os.path.dirname(scenario_path))

    print(f"[export-any-chrono-run] running {scenario_path} {scenario_argv}")
    runpy.run_path(scenario_path, run_name="__main__")

    if _STATE is None:
        print("[export-any-chrono-run] warning: no ChSystem ever added anything -- "
              "nothing was exported")
    else:
        print(f"[export-any-chrono-run] wrote export to {_STATE['base_path']}")


if __name__ == "__main__":
    main()
