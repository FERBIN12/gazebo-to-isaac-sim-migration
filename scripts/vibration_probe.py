"""Look for an Isaac-only vibration, and be willing to find nothing.

WHY THIS EXISTS. The curriculum promises write-up 4.7, "the vibration that
appears only in Isaac", as a FAILURE notes. Nothing in this repository has
ever measured such a vibration. a later check is the notes about asserting one
side of a comparison without running it, and 4.3 is the notes about
explaining a measurement that turned out not to exist. Writing 4.7 from the
curriculum row would be both mistakes at once.

So this measures first. It holds the robot at a constant command and records
the joint velocity every physics step, then reports whether the signal
oscillates and at what amplitude.

WHAT COUNTS AS A VIBRATION. Not "the number moves" -- a settling transient
moves too. The test is sign changes in the derivative during the STEADY window,
after the ramp is over: a signal that repeatedly reverses direction while the
command is constant is oscillating, and one that approaches and stays is not.

A NEGATIVE RESULT IS A RESULT. If Isaac does not vibrate here, the notes
becomes "the failure the curriculum predicted, that the measurement refused to
confirm", which is a more honest notes than the planned one.

Usage:
    ~/.venv-isaacsim/bin/python scripts/vibration_probe.py
    ~/.venv-isaacsim/bin/python scripts/vibration_probe.py --selftest
"""
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/vibration_probe.json"

WHEEL_R = 0.075
TRACK = 0.30
DT = 1.0 / 200.0
SECS = 4.0
CMD_W = 1.0
MAX_ANG_ACCEL = 2.0
DEG_PER_RAD = 180.0 / math.pi

# Ignore the first second: that is the commanded ramp, and a ramp is not a
# vibration. Everything reported comes from the steady window.
SETTLE_S = 1.5

# Gains worth probing. The imported value, the one 4.4 chose, and a high one:
# if a gain-driven oscillation exists anywhere, a stiff drive is where it lives.
GAINS = [0.01, 1.0, 100.0]

# Write-up 4.8 left an obvious untested hypothesis: that the ripple is a solver
# artefact, and that raising the ARTICULATION's iteration counts (not the
# scene's) would reduce it. --solver runs the same measurement at gain 1.0
# across a sweep of position-iteration counts so 4.9 can answer it instead of
# carrying it forward.
SOLVER_ITERS = [4, 16, 64, 255]


def analyse(samples, dt):
    """Return oscillation statistics for one steady-window signal."""
    if len(samples) < 8:
        return {"n": len(samples), "reversals": 0, "reversal_rate_hz": 0.0,
                "peak_to_peak": 0.0, "mean": 0.0, "oscillates": False}
    d = [b - a for a, b in zip(samples, samples[1:])]
    # Count sign changes in the derivative, ignoring exact zeros (a perfectly
    # flat signal has no direction to reverse).
    reversals = 0
    last = 0
    for v in d:
        s = (v > 0) - (v < 0)
        if s == 0:
            continue
        if last != 0 and s != last:
            reversals += 1
        last = s
    span = len(samples) * dt
    p2p = max(samples) - min(samples)
    mean = sum(samples) / len(samples)
    # Two thresholds, both required. A slow drift can reverse a few times from
    # solver noise without being a vibration, and a tiny ripple on a large
    # signal is not one either.
    rate = reversals / span if span else 0.0
    rel = p2p / abs(mean) if abs(mean) > 1e-9 else 0.0
    return {"n": len(samples), "reversals": reversals,
            "reversal_rate_hz": round(rate, 2),
            "peak_to_peak": round(p2p, 5), "mean": round(mean, 5),
            "relative_p2p": round(rel, 5),
            "oscillates": bool(rate > 5.0 and rel > 0.02)}


def selftest():
    """Assert the detector fires on a vibration and stays quiet on a ramp."""
    ok = True
    n = 500
    # A clean settling ramp: monotone, no reversals. Must NOT read as vibration.
    ramp = [2.0 * (1 - math.exp(-i / 60.0)) for i in range(n)]
    if analyse(ramp, DT)["oscillates"]:
        ok = False
        print("SELFTEST FAIL: a settling ramp read as a vibration")
    # A real oscillation: 20 Hz, 10 % amplitude on a mean of 2.0.
    osc = [2.0 + 0.2 * math.sin(2 * math.pi * 20.0 * i * DT) for i in range(n)]
    if not analyse(osc, DT)["oscillates"]:
        ok = False
        print("SELFTEST FAIL: a 20 Hz 10 % oscillation was not detected")
    # A tiny ripple on a big signal is NOT a vibration worth reporting.
    tiny = [2.0 + 0.0005 * math.sin(2 * math.pi * 20.0 * i * DT) for i in range(n)]
    if analyse(tiny, DT)["oscillates"]:
        ok = False
        print("SELFTEST FAIL: a 0.025 % ripple read as a vibration")
    print("selftest: OK" if ok else "selftest: BROKEN")
    return 0 if ok else 1


def main():
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
        _, kd = rover.get_gains()
        kd = np.array(kd)
        kd[0][li] = v * DEG_PER_RAD
        kd[0][ri] = v * DEG_PER_RAD
        rover.set_gains(kds=kd)

    def run(gain):
        # Reset THEN set the gain: the other order applies the previous run's
        # value (see scripts/damping_sweep.py).
        world.reset()
        rover.initialize()
        set_damping(gain)
        return run_after_setup()

    def run_after_setup():
        """The measuring half, with the world already reset and configured.

        Split out so --solver can set solver iteration counts AFTER the reset
        without run() overwriting them.
        """
        wheel, base_z = [], []
        w_cur = 0.0
        for step in range(int(SECS / DT)):
            w_cur = min(CMD_W, w_cur + MAX_ANG_ACCEL * DT)
            v_l = (-w_cur * TRACK / 2.0) / WHEEL_R
            v_r = (+w_cur * TRACK / 2.0) / WHEEL_R
            tgt = np.zeros(len(names), dtype=np.float32)
            tgt[li], tgt[ri] = v_l, v_r
            rover.set_joint_velocity_targets(np.array([tgt]))
            world.step(render=False)
            if step * DT >= SETTLE_S:
                wheel.append(abs(float(rover.get_joint_velocities()[0][ri])))
                p, _ = rover.get_world_poses()
                base_z.append(float(p[0][2]))
        return analyse(wheel, DT), analyse(base_z, DT)

    if "--solver" in sys.argv:
        # Hold the gain at the value 4.4 chose and vary only the solver.
        rows = []
        for it in SOLVER_ITERS:
            world.reset()
            rover.initialize()
            set_damping(1.0)
            # CREATE THE ATTRIBUTE, do not just set it. The importer writes
            # NO physxArticulation:* attributes at all, so the property does
            # not exist on the prim and set_prim_property raises
            # "Empty typeName". Applying PhysxSchema and authoring the
            # attribute explicitly is what actually configures the solver.
            from pxr import Sdf
            import omni.usd as _ou
            _stage = _ou.get_context().get_stage()
            _prim = _stage.GetPrimAtPath(
                "/World/rover/Geometry/base_footprint/base_link")
            for _name in ("physxArticulation:solverPositionIterationCount",
                          "physxArticulation:solverVelocityIterationCount"):
                _a = _prim.GetAttribute(_name)
                if not _a:
                    _a = _prim.CreateAttribute(_name, Sdf.ValueTypeNames.Int)
                _a.Set(int(it))
            w, _z = run_after_setup()
            rows.append({"position_iterations": it, "wheel_velocity": w})
            print(f"  position iters {it:>4}   mean {w['mean']:.4f}  "
                  f"p2p {w['peak_to_peak']:.5f}  relative {w['relative_p2p']:.3f}  "
                  f"-> {'OSCILLATES' if w['oscillates'] else 'steady'}")
        rels = [r["wheel_velocity"]["relative_p2p"] for r in rows]
        # A SPREAD IS NOT A TREND. The first version reported
        # "solver_changes_ripple: True" purely because max-min exceeded a
        # threshold, which is true of any noisy series. If more solver work
        # helped, the ripple would fall monotonically with iteration count;
        # measured it goes 1.391, 1.301, 1.220, 1.398, which is not a trend and
        # ends higher than it started. Report both, and let the monotone test
        # carry the claim.
        monotone_down = all(b <= a + 1e-9 for a, b in zip(rels, rels[1:]))
        still_oscillates = all(r["wheel_velocity"]["oscillates"] for r in rows)
        out = {"engine": "isaac-physx", "damping": 1.0,
               "solver_position_iterations": SOLVER_ITERS, "rows": rows,
               "relative_p2p_range": [min(rels), max(rels)],
               "monotone_decreasing": monotone_down,
               "oscillates_at_every_setting": still_oscillates,
               "solver_fixes_the_ripple": bool(monotone_down and not still_oscillates)}
        o2 = OUT.with_name("vibration_solver_sweep.json")
        o2.write_text(json.dumps(out, indent=2))
        print(f"\n  relative ripple across the sweep: {min(rels):.3f} - {max(rels):.3f}")
        print(f"  monotone decreasing:       {out['monotone_decreasing']}")
        print(f"  oscillates at every setting: {out['oscillates_at_every_setting']}")
        print(f"  solver FIXES the ripple:   {out['solver_fixes_the_ripple']}")
        print(f"  -> {o2.relative_to(BASE)}")
        app.close()
        return

    rows = []
    for g in GAINS:
        w, z = run(g)
        rows.append({"damping": g, "wheel_velocity": w, "base_height": z})
        print(f"  damping {g:>7}")
        print(f"    wheel vel : mean {w['mean']:.4f}  p2p {w['peak_to_peak']:.5f}"
              f"  reversals {w['reversals']:>4} ({w['reversal_rate_hz']} Hz)"
              f"  -> {'OSCILLATES' if w['oscillates'] else 'steady'}")
        print(f"    base z    : mean {z['mean']:.4f}  p2p {z['peak_to_peak']:.5f}"
              f"  reversals {z['reversals']:>4} ({z['reversal_rate_hz']} Hz)"
              f"  -> {'OSCILLATES' if z['oscillates'] else 'steady'}")

    any_osc = any(r["wheel_velocity"]["oscillates"] or r["base_height"]["oscillates"]
                  for r in rows)
    result = {"engine": "isaac-physx", "physics_dt": DT, "secs": SECS,
              "settle_s": SETTLE_S, "commanded_body_spin_rad_s": CMD_W,
              "rows": rows, "any_oscillation_found": any_osc}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"\n  any oscillation found: {any_osc}")
    print(f"  -> {OUT.relative_to(BASE)}")
    app.close()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
