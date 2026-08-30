"""Measure how Isaac's step cost scales with robot count — 9.3's "measured scaling".

The claim this repo is supposed to support is that GPU physics changes what
"many robots at once" costs. That is a claim about a CURVE, not a number, so a
single count proves nothing: 1 robot at 1.3x real time tells you nothing about
16.

WHAT IT MEASURES
  For each N in a sweep, N copies of the rover in one scene, stepped for a fixed
  number of physics steps, timing only the stepping (not the loading).

WHAT MAKES A SCALING CLAIM HONEST
  * report step cost PER ROBOT, because total cost obviously rises
  * compare against LINEAR, which is the null hypothesis: if 16 robots cost 16x
    one robot, the GPU bought you nothing
  * state the load and the GPU, because 8.5 established an RTF without its
    conditions is a boast
  * warm up first: the first steps include lazy allocation and are not
    representative, which 8.5 also established

WHAT IT DOES NOT MEASURE
  Gazebo's scaling. There is no equivalent multi-robot Gazebo scene in this rig,
  so this is an Isaac-side curve, not a comparison. this repo made that mistake
  visible enough times that I am stating it before running.

Usage:
    scripts/scaling_isaac.py [--counts 1,2,4,8,16] [--steps 400]
    python3 scripts/scaling_isaac.py --selftest
"""
import json
import os
import statistics
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/scaling_isaac.json"
DT = 0.005


def per_robot_cost(total_ms, n):
    """Milliseconds of step time per robot. The statistic that matters."""
    return total_ms / n


def linearity(rows):
    """How far from linear scaling, as a ratio at the largest count.

    Returns (observed_ratio, linear_ratio, efficiency). Efficiency of 1.0 means
    perfectly linear (the GPU bought nothing); above 1.0 means sublinear (each
    additional robot costs less than the first).
    """
    if len(rows) < 2:
        return None
    first, last = rows[0], rows[-1]
    n_ratio = last["n"] / first["n"]
    cost_ratio = last["total_ms_per_step"] / first["total_ms_per_step"]
    return {"n_ratio": n_ratio, "cost_ratio": round(cost_ratio, 4),
            "efficiency": round(n_ratio / cost_ratio, 4)}


def selftest():
    ok = True
    # per-robot cost must divide, not subtract
    if abs(per_robot_cost(16.0, 4) - 4.0) > 1e-9:
        print("  FAIL per_robot_cost"); ok = False
    # PERFECTLY LINEAR: 16x the robots for 16x the cost -> efficiency 1.0
    lin = linearity([{"n": 1, "total_ms_per_step": 1.0},
                     {"n": 16, "total_ms_per_step": 16.0}])
    if abs(lin["efficiency"] - 1.0) > 1e-9:
        print(f"  FAIL linear case gave efficiency {lin['efficiency']}"); ok = False
    # SUBLINEAR: 16x robots for 4x cost -> efficiency 4.0, the GPU helped
    sub = linearity([{"n": 1, "total_ms_per_step": 1.0},
                     {"n": 16, "total_ms_per_step": 4.0}])
    if abs(sub["efficiency"] - 4.0) > 1e-9:
        print(f"  FAIL sublinear case gave {sub['efficiency']}"); ok = False
    # SUPERLINEAR: 16x robots for 32x cost -> efficiency 0.5, it got WORSE
    sup = linearity([{"n": 1, "total_ms_per_step": 1.0},
                     {"n": 16, "total_ms_per_step": 32.0}])
    if abs(sup["efficiency"] - 0.5) > 1e-9:
        print(f"  FAIL superlinear case gave {sup['efficiency']}"); ok = False
    # the three cases must be DISTINGUISHABLE, which is the whole point
    if not (sup["efficiency"] < lin["efficiency"] < sub["efficiency"]):
        print("  FAIL the three regimes are not ordered"); ok = False
    # one row cannot support a scaling claim
    if linearity([{"n": 1, "total_ms_per_step": 1.0}]) is not None:
        print("  FAIL a single row produced a scaling claim"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()
    counts = [1, 2, 4, 8, 16]
    steps = 400
    if "--counts" in sys.argv:
        counts = [int(x) for x in sys.argv[sys.argv.index("--counts") + 1].split(",")]
    if "--steps" in sys.argv:
        steps = int(sys.argv[sys.argv.index("--steps") + 1])
    if not STAGE.exists():
        sys.exit(f"no imported stage at {STAGE}")

    load_start = os.getloadavg()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import PhysicsMaterial

    rows = []
    for n in counts:
        world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
        gm = PhysicsMaterial(prim_path="/World/gm", static_friction=1.0,
                             dynamic_friction=1.0, restitution=0.0)
        GroundPlane(prim_path="/World/ground", size=200.0, physics_material=gm)
        for i in range(n):
            add_reference_to_stage(usd_path=str(STAGE),
                                   prim_path=f"/World/rover_{i}")
        # RESET BEFORE CONSTRUCTING THE VIEW. Articulation reads physics
        # metadata at construction time, and before the first world.reset()
        # that metadata is None -- the constructor fails with
        # "'NoneType' object has no attribute 'link_names'" on a perfectly
        # valid stage. The reset has to come first.
        world.reset()
        rovers = Articulation(prim_paths_expr="/World/rover_.*", name="rovers")
        world.scene.add(rovers)
        rovers.initialize()
        # spread them out so they are not interpenetrating
        pos = np.array([[(i % 8) * 3.0, (i // 8) * 3.0, 0.05] for i in range(n)])
        rovers.set_world_poses(positions=pos)

        # WARM UP. 8.5 established the first steps include lazy work.
        for _ in range(100):
            world.step(render=False)

        t0 = time.perf_counter()
        for _ in range(steps):
            world.step(render=False)
        elapsed = time.perf_counter() - t0

        ms_per_step = elapsed * 1000.0 / steps
        rows.append({"n": n, "steps": steps,
                     "total_ms_per_step": round(ms_per_step, 4),
                     "ms_per_step_per_robot": round(per_robot_cost(ms_per_step, n), 4),
                     "rtf": round((steps * DT) / elapsed, 4)})
        print(f"  n={n:3d}  {ms_per_step:8.4f} ms/step  "
              f"{per_robot_cost(ms_per_step, n):7.4f} ms/step/robot  "
              f"rtf {(steps*DT)/elapsed:.3f}")
        world.clear()

    res = {"counts": counts, "steps": steps, "dt": DT,
           "ncpu": os.cpu_count(),
           "load_start": load_start, "load_end": os.getloadavg(),
           "rows": rows, "scaling": linearity(rows)}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))
    s = res["scaling"]
    if s:
        print(f"\n{s['n_ratio']:.0f}x the robots cost {s['cost_ratio']:.2f}x the time")
        print(f"efficiency {s['efficiency']:.2f}  "
              f"(1.0 = linear, >1 = sublinear, <1 = worse than linear)")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
