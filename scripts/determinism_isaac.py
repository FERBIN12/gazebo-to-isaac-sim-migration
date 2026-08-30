"""Measure whether Isaac reproduces the same run twice — the counterpart probe on the other engine.

`determinism_gz.py` measured Gazebo: three identical runs spread by 7.51 mm over
1.61 m, so Gazebo is NOT bit-deterministic. This is the matching Isaac
measurement, deliberately built to be the SAME experiment rather than a similar
one, because this check exists precisely because I once compared a
measured side against a quoted side.

WHAT IS HELD THE SAME AS THE GAZEBO PROBE:
  * the same wheel command profile, converted through the same wheel radius
  * the same duration in SIMULATED seconds
  * the same statistic: max - min across runs, and an "identical" flag at 1e-9
  * the same refusal: if the robot did not move, there is no experiment to call
    deterministic, so raise rather than report three matching zeroes

WHAT NECESSARILY DIFFERS, and is recorded rather than hidden:
  * Gazebo is driven over ROS through diff_drive_controller; Isaac is driven
    in-process through the articulation. That is the instrumentation asymmetry
    RUN.md already records as the 4th rig asymmetry, and it is unavoidable here
    because there is no Isaac ros2_control stack in this rig.
  * Gazebo's pose comes from wheel odometry, Isaac's from get_world_poses().

Usage:
    scripts/determinism_isaac.py [--runs 3] [--secs 4]
    python3 scripts/determinism_isaac.py --selftest
"""
import json
import math
import statistics
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/determinism_isaac.json"

DT = 0.005
WHEEL_RADIUS = 0.10       # matches controllers.yaml and the xacro
TARGET_MS = 0.4           # the same 0.4 m/s the Gazebo probe commands


def spread(vals):
    """max - min across runs, and whether every run agreed to 1e-9.

    Identical to determinism_gz.spread, on purpose: the two halves of a
    comparison must be reduced by the same statistic.
    """
    if not vals:
        return None
    lo, hi = min(vals), max(vals)
    return {"n": len(vals), "min": lo, "max": hi, "spread": hi - lo,
            "identical": (hi - lo) < 1e-9,
            "median": statistics.median(vals)}


def selftest():
    ok = True
    s = spread([1.5, 1.5, 1.5])
    if not (s["identical"] and s["spread"] == 0.0):
        print(f"  FAIL identical case {s}"); ok = False
    s = spread([1.5, 1.5001, 1.5])
    if s["identical"]:
        print(f"  FAIL 1e-4 difference reported identical"); ok = False
    if not spread([1.5, 1.5 + 1e-12, 1.5])["identical"]:
        print("  FAIL 1e-12 reported non-identical"); ok = False
    if spread([]) is not None:
        print("  FAIL empty input"); ok = False
    # THE DEAD-PATH GUARD, same as the Gazebo probe: all-zero motion must be
    # refused, a real run must not be.
    secs = 4.0
    expected_min = 0.25 * TARGET_MS * secs
    if not (max([0.0, 0.0, 0.0]) < expected_min):
        print("  FAIL dead-path guard would not fire"); ok = False
    if not (max([1.03, 1.03, 1.03]) >= expected_min):
        print("  FAIL guard would reject a real run"); ok = False
    # the command conversion must round-trip: 0.4 m/s at r=0.10 is 4 rad/s
    if abs(TARGET_MS / WHEEL_RADIUS - 4.0) > 1e-9:
        print("  FAIL wheel conversion"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()
    runs, secs = 3, 4.0
    if "--runs" in sys.argv:
        runs = int(sys.argv[sys.argv.index("--runs") + 1])
    if "--secs" in sys.argv:
        secs = float(sys.argv[sys.argv.index("--secs") + 1])
    if not STAGE.exists():
        sys.exit(f"no imported stage at {STAGE}")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import PhysicsMaterial

    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    gm = PhysicsMaterial(prim_path="/World/gm", static_friction=1.0,
                         dynamic_friction=1.0, restitution=0.0)
    GroundPlane(prim_path="/World/ground", size=50.0, physics_material=gm)
    add_reference_to_stage(usd_path=str(STAGE), prim_path="/World/rover")
    rover = Articulation(prim_paths_expr="/World/rover", name="rover")
    world.scene.add(rover)
    world.reset()
    rover.initialize()

    names = rover.dof_names
    li, ri = names.index("left_wheel_joint"), names.index("right_wheel_joint")
    tgt_rad = TARGET_MS / WHEEL_RADIUS          # 4.0 rad/s

    def pose():
        p, q = rover.get_world_poses()
        p = np.array(p)[0]
        q = np.array(q)[0]
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                         1 - 2 * (q[2] ** 2 + q[3] ** 2))
        return float(p[0]), float(p[1]), float(yaw)

    results = []
    steps = int(secs / DT)
    for i in range(runs):
        world.reset()
        rover.initialize()
        for _ in range(int(0.5 / DT)):          # settle
            world.step(render=False)
        start = pose()
        cmd = np.zeros((1, len(names)))
        cmd[0, li] = tgt_rad
        cmd[0, ri] = tgt_rad
        for _ in range(steps):
            rover.set_joint_velocity_targets(cmd)
            world.step(render=False)
        cmd[:] = 0.0
        for _ in range(int(0.5 / DT)):
            rover.set_joint_velocity_targets(cmd)
            world.step(render=False)
        end = pose()
        results.append({"run": i, "start": start, "end": end,
                        "dx": end[0] - start[0], "dy": end[1] - start[1],
                        "dyaw": end[2] - start[2]})
        print(f"  run {i}: dx={results[-1]['dx']:+.6f} "
              f"dy={results[-1]['dy']:+.6f} dyaw={results[-1]['dyaw']:+.6f}")

    # DO NOT close the app before writing the result. `app.close()` tears the
    # process down hard enough that code after it does not reliably run: the
    # first version printed all three runs, called close(), and never wrote the
    # JSON or the summary. Everything that must survive is computed and written
    # BEFORE the close.
    expected_min = 0.25 * TARGET_MS * secs
    moved = [abs(r["dx"]) for r in results]
    if max(moved) < expected_min:
        raise SystemExit(
            f"REFUSING: largest dx was {max(moved):.6f} m, expected at least "
            f"{expected_min:.3f} m. The robot did not move.")

    res = {"runs": runs, "secs_sim": secs, "dt": DT,
           "target_ms": TARGET_MS, "target_rad_s": tgt_rad,
           "results": results,
           "moved_min_m": round(min(moved), 6), "moved_max_m": round(max(moved), 6),
           "dx": spread([r["dx"] for r in results]),
           "dy": spread([r["dy"] for r in results]),
           "dyaw": spread([r["dyaw"] for r in results])}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))
    for k in ("dx", "dy", "dyaw"):
        s = res[k]
        print(f"{k:5s} min={s['min']:+.6f} max={s['max']:+.6f} "
              f"spread={s['spread']:.2e} identical={s['identical']}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
