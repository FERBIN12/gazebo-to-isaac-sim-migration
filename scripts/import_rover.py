#!/usr/bin/env python3
"""a later check: import the rover URDF into Isaac Sim, from a SCRIPT.

WHY THIS IS A SCRIPT AND NOT THE GUI IMPORTER DIALOG (notes 3.1):
Isaac Sim offers three paths from a URDF -- a GUI extension, this Python API,
and command line tooling -- and they do not produce identical results, because
they expose different defaults. The GUI dialog also REMEMBERS your last
choices, so your third import is not your first and nothing tells you. A
script has no memory and no hidden state, so when the result changes the diff
says why.

That is only true if the script PRINTS THE PARAMETERS IT USED, so it does,
every run, before importing anything. Those printed values are the "five
guesses" of notes 3.1: they are settings the URDF never stated and the
importer had to decide.

Run:
    cd ~/gazebo-to-isaac-migration
    ~/.venv-isaacsim/bin/python scripts/import_rover.py

Writes:
    sim-workspace/isaac/rover.usd      the imported stage
    data/import_params.json            exactly what was used, for the readback

MEASURED against Isaac Sim 6.0.1, isaacsim.asset.importer.urdf. The importer
config surface was read off the running extension rather than assumed; every
field set below exists on URDFImporterConfig in that version.
"""
import json
import subprocess
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
URDF_XACRO = BASE / "sim-workspace/src/rover_description/urdf/rover.urdf.xacro"
# The converter treats usd_path as an output DIRECTORY and writes a layered
# stage inside it (rover/rover.usda plus payloads/ for geometry, materials
# and physics). It is not a single .usd file, so nothing here may assume one.
OUT_DIR = BASE / "sim-workspace/isaac/rover.usd"
OUT_USD = OUT_DIR / "rover/rover.usda"
OUT_URDF = BASE / "data/rover.urdf"
PARAMS = BASE / "data/import_params.json"

# THE IMPORT SETTINGS, DECLARED HERE RATHER THAN LEFT DEFAULT.
#
# Every one of these is a decision the URDF does not contain. They are written
# down so the run is reproducible and so notes 3.1 can show them on screen as
# what they are: guesses, made by something other than the robot's author.
#
#   collision_type     'Convex Hull' is the importer default. It is also the
#                      reason 3.2's caster stops being a sphere: a hull is a
#                      polyhedron, and a polyhedron rolling on a plane has a
#                      contact that jumps between faces.
#   fix_base           False: this robot drives. A URDF cannot say whether its
#                      root is bolted down, so the importer has to be told.
#   merge_fixed_joints False: KEEP the fixed-joint structure. Merging is often
#                      faster in PhysX but it destroys the link-by-link
#                      correspondence every measurement in this section needs.
#   allow_self_collision False: enabling it broadly makes the robot touch its
#                      own wheels at rest (3.1 s11).
#   joint_drive_type / stiffness / damping are left None DELIBERATELY, so that
#                      section 4 can show that the drives are not in the file.
CONFIG = {
    "collision_type": "Convex Hull",
    "fix_base": False,
    "merge_fixed_joints": False,
    "allow_self_collision": False,
    "collision_from_visuals": False,
    "link_density": 0.0,          # 0 => trust the URDF's own inertials
    "robot_type": "Default",
}


def expand_xacro() -> str:
    """xacro -> plain URDF, with the workspace overlay sourced.

    The overlay is REQUIRED, not optional: the description uses package://
    paths (that is FLAW-3), so a bare `source /opt/ros/jazzy/setup.bash` fails
    with PackageNotFoundError and no URDF is produced at all.
    """
    ws = BASE / "sim-workspace"
    cmd = ("source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
           f"xacro {URDF_XACRO}")
    r = subprocess.run(["bash", "-c", cmd], cwd=ws, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"xacro failed:\n{r.stderr}")
    return r.stdout


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    OUT_URDF.parent.mkdir(parents=True, exist_ok=True)

    urdf_text = expand_xacro()
    OUT_URDF.write_text(urdf_text)

    print("=" * 72)
    print("URDF IMPORT -- parameters used for this run")
    print("=" * 72)
    print(f"  source xacro : {URDF_XACRO.relative_to(BASE)}")
    print(f"  expanded urdf: {OUT_URDF.relative_to(BASE)}  ({len(urdf_text)} bytes)")
    print(f"  output stage : {OUT_USD.relative_to(BASE)}")
    print("  importer     : isaacsim.asset.importer.urdf (Python API)")
    for k in sorted(CONFIG):
        print(f"    {k:22s} = {CONFIG[k]!r}")
    print("=" * 72)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.asset.importer.urdf")
    import isaacsim.asset.importer.urdf as U

    cfg = U.URDFImporterConfig()
    for k, v in CONFIG.items():
        setattr(cfg, k, v)
    cfg.urdf_path = str(OUT_URDF)
    cfg.usd_path = str(OUT_DIR)
    # THE package:// PATHS, AND THE SHAPE THIS FIELD ACTUALLY WANTS.
    #
    # ros_package_paths is NOT a list of directory strings. The converter does
    # `package.get("name")` on every entry, so a list of strings dies with
    # `AttributeError: 'str' object has no attribute 'get'` from deep inside
    # urdf_usd_converter -- a traceback that says nothing about what you passed.
    # Each entry is a {"name", "path"} pair. Read off convert.py in 6.0.1.
    #
    # This is FLAW-3 (package:// mesh paths) meeting the importer: Gazebo
    # resolved those paths from the sourced workspace, and the importer has no
    # workspace and no ament index, so it has to be told where the package is
    # or the reference silently resolves to nothing.
    share = BASE / "sim-workspace/install/rover_description/share"
    cfg.ros_package_paths = [{"name": "rover_description",
                              "path": str(share / "rover_description")}]

    result = U.URDFImporter(cfg).import_urdf()

    # EVERYTHING THAT MATTERS HAPPENS BEFORE app.close().
    #
    # The app is started with /app/fastShutdown=True, and that shutdown does not
    # return: the first version of this script wrote import_params.json and
    # printed its IMPORTED line AFTER app.close(), and the process exited 0 with
    # the stage on disk, no params file, and none of those prints. An exit code
    # of 0 and a written stage looked like a complete run. Verify the artefact,
    # not the exit status -- and do the work while the interpreter is still alive.
    if not OUT_USD.exists():
        sys.exit(f"importer returned {result!r} but wrote no stage at {OUT_USD}")

    PARAMS.write_text(json.dumps(
        {"config": CONFIG, "urdf": str(OUT_URDF), "usd": str(OUT_USD),
         "importer": "isaacsim.asset.importer.urdf",
         "result": str(result)}, indent=2))
    print(f"\nIMPORTED -> {OUT_USD.relative_to(BASE)} "
          f"({OUT_USD.stat().st_size} bytes)")
    print(f"parameters -> {PARAMS.relative_to(BASE)}")
    sys.stdout.flush()
    app.close()


if __name__ == "__main__":
    main()
