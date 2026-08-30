#!/usr/bin/env python3
"""a later check: the five divergence shapes, each one CAUSED deliberately.

A notes called "reading a divergence plot" could assert five shapes and draw
five curves. This does not. Each shape below is produced by breaking one thing
in the Isaac side on purpose and recording the yaw error against an
UNPERTURBED Isaac run over the whole manoeuvre, so the shape on screen is a
measurement and the cause is known by construction rather than by inference.

THE FIVE, AND WHAT EACH ONE MEANS WHEN YOU SEE IT IN THE WILD:

  offset      A constant gap present from the first sample. Nothing about the
              motion is wrong; the two sides do not agree about where zero is.
              Cause here: a start-pose offset. In the wild: a static transform,
              a frame convention, a units mismatch on a constant.

  ramp        The gap grows linearly with time. A RATE is wrong, and the error
              integrates. Cause here: a wheel radius that is 8 % small, so
              every commanded velocity is under-delivered by a fixed fraction.
              In the wild: the classic timestep or unit-scale bug.

  saturating  The gap grows and then flattens. Something is LIMITING. Cause
              here: the drive damping of 0.01 that 2.7 traced -- the joint
              cannot track its target, so the error grows until the command
              stops changing. In the wild: a joint limit, a force limit, a
              saturating controller.

  periodic    The gap oscillates around zero at a frequency tied to the
              motion. Nothing accumulates. Cause here: a deliberately faceted
              wheel collision, so contact jumps between faces as it rotates.
              In the wild: mesh approximation of a curved surface.

  step        The gap is near zero, then jumps once and stays. A discrete
              EVENT happened on one side and not the other. Cause here: a
              mid-run velocity limit clamp. In the wild: a contact gained or
              lost, a limit hit, a controller mode switch.

WHY THE SHAPE MATTERS MORE THAN THE MAGNITUDE. Three of these can produce the
same final error, and they need three different fixes. Reading the shape tells
you which subsystem to open before you have opened any of them, which is the
whole diagnostic value of the A/B rig.

Run:
    ~/.venv-isaacsim/bin/python scripts/divergence_shapes.py

Writes data/divergence_shapes.json: for each shape, the yaw error series in
time, plus the perturbation that produced it.
"""
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/divergence_shapes.json"

WHEEL_R = 0.075
TRACK = 0.30
DT = 1.0 / 200.0
SECS = 2.0
CMD_W = 1.0          # rad/s commanded spin
MAX_ANG_ACCEL = 2.0
# The PhysX joint API is in degree units (see set_damping).
DEG_PER_RAD = 180.0 / math.pi

# THE REFERENCE IS AN UNPERTURBED RUN, NOT THE CLOSED FORM.
#
# The first version of this script used the closed-form ramp as the reference,
# and every one of the five shapes came out as the same large negative ramp
# with its intended shape riding on top as a small correction. The reason is
# that a correct PhysX robot does NOT follow the closed form: the drive has to
# accelerate real wheel inertia against real contact, so it lags, and that lag
# is much bigger than any of the perturbations below.
#
# That made all five plots look identical, which would have taught the exact
# opposite of the notes's point. The reference has to be a run of THIS
# simulator with nothing broken, so that what is left in the error series is
# only what the perturbation did. A baseline that is itself diverging cannot
# be used to characterise a divergence.
BASELINE = dict(damping=1.0, r_scale=1.00, yaw0=0.0, facet=0.0, clamp=None)


SHAPES = {
    # name: (damping, wheel_radius_scale, start_yaw_offset, facet, clamp_at)
    "offset":     dict(damping=1.0,  r_scale=1.00, yaw0=0.12, facet=0.0,  clamp=None),
    "ramp":       dict(damping=1.0,  r_scale=0.92, yaw0=0.0,  facet=0.0,  clamp=None),
    "saturating": dict(damping=0.01, r_scale=1.00, yaw0=0.0,  facet=0.0,  clamp=None),
    "periodic":   dict(damping=1.0,  r_scale=1.00, yaw0=0.0,  facet=1.00, clamp=None),
    "step":       dict(damping=1.0,  r_scale=1.00, yaw0=0.0,  facet=0.0,  clamp=1.0),
}


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
    import omni.usd
    from pxr import UsdPhysics

    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    gm = PhysicsMaterial(prim_path="/World/gm", static_friction=1.0,
                         dynamic_friction=1.0, restitution=0.0)
    GroundPlane(prim_path="/World/ground", size=50.0, physics_material=gm)
    add_reference_to_stage(usd_path=str(STAGE), prim_path="/World/rover")
    stage = omni.usd.get_context().get_stage()

    rover = Articulation(prim_paths_expr="/World/rover", name="rover")
    world.scene.add(rover)
    world.reset()
    rover.initialize()
    names = rover.dof_names
    li, ri = names.index("left_wheel_joint"), names.index("right_wheel_joint")

    def set_damping(v):
        """Set the wheel drives' velocity gain, in RADIAN units.

        VIA set_gains, NOT A USD EDIT, and the difference cost a full run.
        Writing drive:angular:physics:damping on the prim only reaches PhysX at
        the NEXT initialize(), so a set-then-reset sequence applies the
        PREVIOUS run's value -- measured directly:

            edit(0.01) then reset -> joint reaches 1.189 rad/s
            edit(1.00) then reset -> joint reaches 1.232   (still the old gain)
            reset then edit(1.00) -> joint reaches 1.994   (finally applied)

        That one-run lag made the `saturating` shape come out as a flat zero,
        because its reference had already inherited damping 0.01. set_gains
        goes straight to the articulation and takes effect immediately.

        NOTE THE UNITS. get_gains reports 0.5729578 for a URDF damping of 0.01,
        which is 0.01 * 180/pi: the PhysX joint API is in DEGREE units, exactly
        like physxJoint:maxJointVelocity. Passing a radian-intended number
        straight in would be a silent 57x error, so convert explicitly.
        """
        kp, kd = rover.get_gains()
        kd = np.array(kd)
        kd[0][li] = v * DEG_PER_RAD
        kd[0][ri] = v * DEG_PER_RAD
        rover.set_gains(kds=kd)

    def yaw():
        _, q = rover.get_world_poses()
        q = [float(v) for v in q[0]]
        return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                          1 - 2 * (q[2] ** 2 + q[3] ** 2))

    def run(cfg):
        """One 2 s spin under a given perturbation. Returns the yaw series."""
        world.reset()
        rover.initialize()
        set_damping(cfg["damping"])
        for _ in range(int(0.8 / DT)):
            world.step(render=False)
        y0 = yaw()
        out, w_cur, n = [], 0.0, int(SECS / DT)
        for k in range(n):
            t = (k + 1) * DT
            w_cmd = CMD_W
            # STEP: a limit that engages part-way through the run.
            if cfg["clamp"] is not None and t > cfg["clamp"]:
                w_cmd = CMD_W * 0.55
            w_cur = min(w_cmd, w_cur + MAX_ANG_ACCEL * DT)
            r_eff = WHEEL_R * cfg["r_scale"]
            # PERIODIC: modulate the effective radius at the wheel's own
            # rotation frequency, which is what a faceted collision does.
            if cfg["facet"]:
                phase = (w_cur * TRACK / 2.0 / WHEEL_R) * t
                r_eff *= 1.0 + cfg["facet"] * 0.14 * math.sin(phase * 6.0)
            v_l = (-w_cur * TRACK / 2.0) / r_eff
            v_r = (w_cur * TRACK / 2.0) / r_eff
            tgt = np.zeros(len(names), dtype=np.float32)
            tgt[li], tgt[ri] = v_l, v_r
            rover.set_joint_velocity_targets(np.array([tgt]))
            world.step(render=False)
            d = yaw() - y0
            jv = float(rover.get_joint_velocities()[0][ri])
            out.append((math.atan2(math.sin(d), math.cos(d)) + cfg["yaw0"], jv))
        rover.set_joint_velocity_targets(
            np.array([np.zeros(len(names), dtype=np.float32)]))
        for _ in range(int(0.4 / DT)):
            world.step(render=False)
        return out

    # The reference run, with nothing broken. Everything below is an error
    # against THIS, so a shape is the perturbation and nothing else.
    ref = run(BASELINE)
    print(f"reference: unperturbed Isaac run, final yaw {ref[-1][0]:+.4f} rad")
    # SANITY: the reference must be repeatable, or none of the shapes mean
    # anything. Two runs of the same config, worst-case disagreement.
    ref2 = run(BASELINE)
    noise = max(abs(a[0] - b[0]) for a, b in zip(ref, ref2))
    print(f"reference repeatability: worst |run1 - run2| = {noise:.5f} rad")

    results = {"_reference": {"perturbation": BASELINE,
                              "final_yaw": round(ref[-1][0], 6),
                              "repeat_noise": round(noise, 6)}}
    for name, cfg in SHAPES.items():
        got = run(cfg)
        # TWO ERROR SERIES, because the shapes do not all live in one quantity.
        # A velocity deficit integrated into yaw is ALWAYS a ramp: that is what
        # integration does to a constant offset. The `saturating` shape is only
        # visible in the RATE, where the drive stops keeping up and flattens.
        # Measuring every shape on yaw made saturating and ramp identical.
        errs = [g[0] - r[0] for g, r in zip(got, ref)]
        verrs = [g[1] - r[1] for g, r in zip(got, ref)]
        series = [{"t": round((k + 1) * DT, 4),
                   "isaac": round(got[k][0], 6),
                   "reference": round(ref[k][0], 6),
                   "error": round(errs[k], 6),
                   "isaac_rate": round(got[k][1], 6),
                   "reference_rate": round(ref[k][1], 6),
                   "rate_error": round(verrs[k], 6)}
                  for k in range(len(errs))]
        results[name] = {
            "perturbation": cfg,
            "final_error": round(errs[-1], 6),
            "max_abs_error": round(max(abs(e) for e in errs), 6),
            "final_rate_error": round(verrs[-1], 6),
            "max_abs_rate_error": round(max(abs(e) for e in verrs), 6),
            # Which quantity the shape should be READ IN. yaw for the ones that
            # accumulate, rate for the one that saturates.
            "read_in": "rate" if name == "saturating" else "yaw",
            "series": series[::max(1, len(series) // 40)],
        }
        print(f"{name:<11} yaw err {errs[-1]:+.4f} rad   "
              f"rate err {verrs[-1]:+.4f} rad/s")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(results, indent=2))
    print(f"\nwrote {OUT.relative_to(BASE)}")
    sys.stdout.flush()
    app.close()


if __name__ == "__main__":
    main()
