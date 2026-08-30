"""Measure Isaac's real-time factor under stated load — the counterpart probe on the other engine.

`rtf_probe.py` measured Gazebo: overall RTF 0.9946, but per-second samples from
0.5766 to 1.6388 with 7.7 % below 0.9. The overall number was a lie by omission;
the min and the stall fraction told the real story.

This is the Isaac half, reduced by the SAME statistics so the two are
comparable. Isaac has no /clock to subscribe to here, so sim time is counted as
steps * physics_dt and wall time from time.monotonic() -- which is exactly the
5.3 trap, stated openly: sim seconds on one axis, wall seconds on the other, and
the ratio is the RTF.

Reported with the conditions, because an RTF without a machine and a load is not
a measurement.

Usage:
    scripts/rtf_isaac.py [--secs 20]
    python3 scripts/rtf_isaac.py --selftest
"""
import json
import os
import statistics
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/rtf_isaac.json"
DT = 0.005


def summarise(samples):
    """Identical reduction to rtf_probe.summarise, on purpose."""
    if not samples:
        return None
    return {"n": len(samples),
            "min": round(min(samples), 4),
            "median": round(statistics.median(samples), 4),
            "max": round(max(samples), 4),
            "mean": round(statistics.fmean(samples), 4),
            "frac_below_0.9": round(sum(1 for s in samples if s < 0.9) / len(samples), 4)}


def selftest():
    ok = True
    s = summarise([1.0] * 10)
    if not (s["min"] == s["median"] == s["max"] == 1.0 and s["frac_below_0.9"] == 0.0):
        print(f"  FAIL steady {s}"); ok = False
    steady = summarise([0.9] * 10)
    stall = summarise([1.0] * 9 + [0.0])
    if abs(steady["mean"] - stall["mean"]) > 0.001:
        print("  FAIL fixture means differ"); ok = False
    if steady["min"] == stall["min"]:
        print("  FAIL min cannot separate steady-slow from stalled"); ok = False
    if steady["frac_below_0.9"] == stall["frac_below_0.9"]:
        print("  FAIL stall fraction cannot separate them"); ok = False
    if summarise([]) is not None:
        print("  FAIL empty"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()
    secs = 20.0
    if "--secs" in sys.argv:
        secs = float(sys.argv[sys.argv.index("--secs") + 1])
    if not STAGE.exists():
        sys.exit(f"no imported stage at {STAGE}")

    load_start = os.getloadavg()
    ncpu = os.cpu_count()

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
    cmd = np.zeros((1, len(names)))
    cmd[0, li] = 4.0
    cmd[0, ri] = 4.0

    # warm up: the first steps include lazy init and are not representative
    for _ in range(200):
        rover.set_joint_velocity_targets(cmd)
        world.step(render=False)

    samples = []
    t_bucket = time.monotonic()
    sim_bucket = 0.0
    sim_total = 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        rover.set_joint_velocity_targets(cmd)
        world.step(render=False)
        sim_total += DT
        sim_bucket += DT
        now = time.monotonic()
        if now - t_bucket >= 1.0:
            samples.append(sim_bucket / (now - t_bucket))
            t_bucket, sim_bucket = now, 0.0
    wall_total = time.monotonic() - t0
    load_end = os.getloadavg()

    overall = sim_total / wall_total
    res = {"window_s": secs, "ncpu": ncpu, "dt": DT,
           "load_start": load_start, "load_end": load_end,
           "steps": int(sim_total / DT),
           "overall_rtf": round(overall, 4),
           "per_second": summarise(samples)}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    print(f"window          {secs:.0f} s wall")
    print(f"cpus            {ncpu}")
    print(f"load  start     {load_start[0]:.2f}   end {load_end[0]:.2f}")
    print(f"physics steps   {res['steps']}")
    print(f"overall RTF     {overall:.4f}")
    s = res["per_second"]
    print(f"per-second RTF  n={s['n']} min={s['min']} median={s['median']} "
          f"max={s['max']} mean={s['mean']}")
    print(f"  samples below 0.9: {s['frac_below_0.9']:.1%}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
