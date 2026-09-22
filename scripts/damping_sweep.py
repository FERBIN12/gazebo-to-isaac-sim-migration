"""Sweep the PhysX wheel-drive gain and measure what the joint actually does.

Until this script existed, the sweep table in RUN.md was produced by hand.
A claim that says "here is the measurement" must be runnable by anyone, so
this reproduces the table from scratch and writes it as JSON.

WHAT IT MEASURES. Command a constant wheel velocity and record two things per
gain:

  * `reached`  -- the joint velocity SAMPLED at the end of the run
  * `dyaw`     -- the yaw INTEGRATED over the whole run

Both come from the same run deliberately. The sampled speed is non-monotone in
the gain (see the 0.3 row) while the integrated yaw is monotone, and the point
of the notes is that when two statistics from one run disagree like that, the
statistic is the suspect, not the system.

TWO TRAPS, both measured rather than guessed, both from RUN.md:

  1. **reset THEN edit.** `set_gains` reaches the articulation immediately, but
     a USD attribute edit only reaches PhysX at the next `initialize()`. A
     `set_damping(); world.reset()` sequence therefore applies the PREVIOUS
     run's gain -- a one-run lag that once made a whole divergence shape come
     out as flat zero.

  2. **The PhysX joint API is in DEGREE units.** `get_gains` reports 0.5729578
     for a URDF damping of 0.01, which is 0.01 * 180/pi. Passing a
     radian-intended number straight in is a silent 57x error.

Usage:
    ~/.venv-isaacsim/bin/python scripts/damping_sweep.py
    ~/.venv-isaacsim/bin/python scripts/damping_sweep.py --selftest
"""
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/damping_sweep.json"

WHEEL_R = 0.075
TRACK = 0.30
DT = 1.0 / 200.0
SECS = 2.0
CMD_W = 1.0              # rad/s commanded body spin
MAX_ANG_ACCEL = 2.0
DEG_PER_RAD = 180.0 / math.pi

# The imported value first, then the sweep. 0.01 is what the URDF actually
# carries, so it is the row that describes the robot as migrated.
GAINS = [0.01, 0.03, 0.1, 0.3, 1.0, 100.0]

# Isaac has been running at 1/200 s while the Gazebo world declares
# max_step_size 0.001 -- a 5x longer step on the Isaac side, present under
# EVERY comparison in this project. --dt sweeps the timestep at the gain 4.4
# chose, to find out whether that asymmetry is part of the 16.4 % residual.
DTS = [1.0 / 1000.0, 1.0 / 500.0, 1.0 / 200.0, 1.0 / 100.0]


def selftest():
    """Assert the unit conversion and the monotonicity claim, without Isaac.

    Built synthetically rather than pinned to a recorded run: a fixture that
    points at a real measurement goes stale the moment that measurement is
    re-taken, and then prints BROKEN forever while the code is fine.
    """
    ok = True

    # 1. The degree conversion is the whole 57x trap. Assert both directions.
    if abs(0.01 * DEG_PER_RAD - 0.5729578) > 1e-6:
        print("SELFTEST FAIL: 0.01 rad should report as 0.5729578 in degrees")
        ok = False
    if abs(1.0 * DEG_PER_RAD - 57.29578) > 1e-4:
        print("SELFTEST FAIL: 1.0 rad should report as 57.29578 in degrees")
        ok = False

    # 2. The monotonicity check must PASS a monotone series and FAIL a
    #    non-monotone one. A checker that only ever passes is not a checker.
    if not _monotone([1.015, 1.520, 1.700, 1.752, 1.771, 1.779]):
        print("SELFTEST FAIL: a rising series should read as monotone")
        ok = False
    if _monotone([1.207, 1.789, 1.943, 1.377, 1.991, 2.006]):
        print("SELFTEST FAIL: the 0.3 dip should NOT read as monotone")
        ok = False

    print("selftest: OK" if ok else "selftest: BROKEN")
    return 0 if ok else 1


def _monotone(vals, tol=1e-9):
    return all(b >= a - tol for a, b in zip(vals, vals[1:]))


def main():
    # physics_dt is fixed when the World is constructed, so a timestep sweep
    # cannot run in one process: each value needs its own invocation. Exposed
    # as a flag rather than hard-coded so the sweep is a shell loop.
    global DT
    global SOLVER
    SOLVER = None
    for a in sys.argv[1:]:
        if a.startswith("--dt="):
            DT = float(a.split("=", 1)[1])
        elif a.startswith("--solver="):
            # TGS vs PGS is a different contact solver, not a different number
            # of iterations -- a far stronger lever than the articulation
            # iteration counts that 4.8/4.9 found did nothing.
            SOLVER = a.split("=", 1)[1].upper()
    if not STAGE.exists():
        sys.exit(f"no imported stage at {STAGE}; run scripts/import_rover.py")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import PhysicsMaterial

    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    if SOLVER:
        world.get_physics_context().set_solver_type(SOLVER)
        print(f"  solver type: {SOLVER}")
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

    def set_damping(v):
        """Set the wheel velocity gain, converting rad -> deg for the API."""
        _, kd = rover.get_gains()
        kd = np.array(kd)
        kd[0][li] = v * DEG_PER_RAD
        kd[0][ri] = v * DEG_PER_RAD
        rover.set_gains(kds=kd)

    def yaw():
        _, q = rover.get_world_poses()
        q = [float(x) for x in q[0]]
        return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                          1 - 2 * (q[2] ** 2 + q[3] ** 2))

    def run(gain):
        # RESET FIRST, THEN SET THE GAIN. The other order measures the previous
        # run's value; see the module docstring.
        world.reset()
        rover.initialize()
        set_damping(gain)

        # Confirm the gain actually landed, in the API's own units. Without
        # this the one-run lag is invisible and every row looks plausible.
        _, kd_now = rover.get_gains()
        applied = float(np.array(kd_now)[0][li]) / DEG_PER_RAD

        y0 = yaw()
        w_cur = 0.0
        for _ in range(int(SECS / DT)):
            w_cur = min(CMD_W, w_cur + MAX_ANG_ACCEL * DT)
            v_l = (-w_cur * TRACK / 2.0) / WHEEL_R
            v_r = (+w_cur * TRACK / 2.0) / WHEEL_R
            tgt = np.zeros(len(names), dtype=np.float32)
            tgt[li], tgt[ri] = v_l, v_r
            rover.set_joint_velocity_targets(np.array([tgt]))
            world.step(render=False)

        jv = rover.get_joint_velocities()[0]
        reached = abs(float(jv[ri]))
        y1 = yaw()
        dyaw = abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))
        return applied, reached, dyaw

    rows = []
    for g in GAINS:
        applied, reached, dyaw = run(g)
        # A gain that did not land invalidates its own row, so say so loudly
        # rather than recording a number produced by a different gain.
        lag = abs(applied - g) > max(1e-6, 0.01 * g)
        rows.append({"damping": g, "applied": round(applied, 6),
                     "reached_rad_s": round(reached, 3),
                     "dyaw_rad": round(dyaw, 3),
                     "gain_landed": not lag})
        flag = "" if not lag else f"   ** GAIN DID NOT LAND (applied {applied:.4f})"
        print(f"  damping {g:>7}   reached {reached:6.3f} rad/s   "
              f"dyaw {dyaw:6.3f} rad{flag}")

    reached = [r["reached_rad_s"] for r in rows]
    dyaws = [r["dyaw_rad"] for r in rows]
    result = {
        "engine": "isaac-physx",
        "physics_dt": DT,
        "solver_type": SOLVER or "default(TGS)",
        "gazebo_max_step_size": 0.001,
        "dt_ratio_vs_gazebo": round(DT / 0.001, 3),
        "commanded_body_spin_rad_s": CMD_W,
        "secs": SECS,
        "rows": rows,
        # The notes's actual claim, computed rather than asserted.
        "reached_monotone": _monotone(reached),
        "dyaw_monotone": _monotone(dyaws),
        "all_gains_landed": all(r["gain_landed"] for r in rows),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    if SOLVER:
        out = OUT.with_name(f"damping_sweep_{SOLVER.lower()}.json")
    elif DT != 1.0 / 200.0:
        out = OUT.with_name(f"damping_sweep_dt{int(round(1.0 / DT))}.json")
    else:
        out = OUT
    out.write_text(json.dumps(result, indent=2))
    print(f"\n  sampled speed monotone in the gain : {result['reached_monotone']}")
    print(f"  integrated yaw monotone in the gain: {result['dyaw_monotone']}")
    print(f"  -> {out.relative_to(BASE)}")
    app.close()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
