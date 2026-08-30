#!/usr/bin/env python3
"""a later check: the diff script. What "the same" means, numerically.

Reads `data/baseline_gz.json` and `data/baseline_isaac.json` -- both written by
the two baseline scripts, never by a live simulator -- and reports the
difference field by field against tolerances declared in this file.

WHY TOLERANCES LIVE HERE, AT THE TOP, IN CODE (G24). The acceptable difference
is stated BEFORE the measurement is looked at. Deciding afterwards what counts
as close enough is how a broken migration gets signed off. If a number below
has to change, that is a decision with a reason, and the reason belongs in the
commit message.

WHAT A TOLERANCE MEANS HERE. These are not float comparisons: two different
physics engines integrating a wheeled robot with contact will never agree to
machine precision, and demanding that would be theatre. They are engineering
bounds on quantities a migrated robot has to preserve to be the same robot.

Run:
    python3 scripts/diff_ab.py
    python3 scripts/diff_ab.py --selftest

Exit 1 if any field is outside tolerance, so it can gate.
"""
import json
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
GZ = BASE / "data/baseline_gz.json"
ISO = BASE / "data/baseline_isaac.json"

# THE TOLERANCES, DECLARED BEFORE THE MEASUREMENT.
#
#   dx    forward travel over a 2 s ramped move. 10 % of the ~0.72 m predicted
#         is 72 mm. Two contact models will not agree closer than that on a
#         wheeled robot, and a difference LARGER than that changes where the
#         robot ends up in a room.
#   dy    lateral drift. A diff-drive robot cannot translate sideways, so this
#         is not a tolerance on physics but on symmetry: either engine showing
#         more than 20 mm means its two wheels are not behaving equally.
#   dyaw  heading. 0.05 rad is 2.9 deg, roughly the point at which a Nav2 goal
#         tolerance starts caring.
#   peak  steady-state wheel speed. This one is TIGHT (5 %) on purpose: it is
#         kinematics, not contact. The command asks for 0.400 m/s at the rim
#         and both engines should deliver it almost exactly. A difference here
#         means a joint limit, a drive mode or a unit is wrong, which is a much
#         more serious finding than a contact disagreement.
TOL = {
    "dx":              {"abs": 0.072, "unit": "m",     "why": "10% of predicted travel"},
    "dy":              {"abs": 0.020, "unit": "m",     "why": "wheel symmetry, not physics"},
    "dyaw":            {"abs": 0.050, "unit": "rad",   "why": "2.9 deg, Nav2-relevant"},
    "peak_wheel_m_s":  {"abs": 0.020, "unit": "m/s",   "why": "5% of 0.400, kinematic"},
}
FIELDS = ["dx", "dy", "dyaw", "peak_wheel_m_s"]
RUNS = ["straight", "spin"]


def compare(gz, iso):
    """Return (rows, fails). A row per run per field."""
    rows, fails = [], []
    for run in RUNS:
        a, b = gz.get(run), iso.get(run)
        if not a or not b:
            fails.append(f"{run}: missing on "
                         f"{'gazebo' if not a else 'isaac'} side")
            continue
        for f in FIELDS:
            av, bv = a.get(f), b.get(f)
            if av is None or bv is None:
                fails.append(f"{run}.{f}: not recorded on both sides")
                continue
            d = bv - av
            lim = TOL[f]["abs"]
            ok = abs(d) <= lim
            rows.append({"run": run, "field": f, "gazebo": av, "isaac": bv,
                         "diff": d, "tol": lim, "within": ok})
            if not ok:
                fails.append(f"{run}.{f}: {av:+.3f} vs {bv:+.3f} "
                             f"= {d:+.3f} {TOL[f]['unit']}, "
                             f"outside +/-{lim} ({TOL[f]['why']})")
    return rows, fails


def main():
    if not GZ.exists():
        sys.exit(f"no {GZ.name}; run scripts/baseline_gz.py against a live sim")
    if not ISO.exists():
        sys.exit(f"no {ISO.name}; run scripts/baseline_isaac.py")
    gz, iso = json.loads(GZ.read_text()), json.loads(ISO.read_text())

    print("A/B DIFF   gazebo-harmonic  vs  isaac-physx")
    print("tolerances declared in diff_ab.py, before the measurement:")
    for f in FIELDS:
        print(f"    {f:<16} +/- {TOL[f]['abs']:<6} {TOL[f]['unit']:<5} "
              f"{TOL[f]['why']}")
    print()

    rows, fails = compare(gz, iso)
    hdr = f"  {'run':<9} {'field':<16} {'gazebo':>9} {'isaac':>9} {'diff':>9}   verdict"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for r in rows:
        print(f"  {r['run']:<9} {r['field']:<16} {r['gazebo']:>+9.3f} "
              f"{r['isaac']:>+9.3f} {r['diff']:>+9.3f}   "
              f"{'ok' if r['within'] else 'OUTSIDE'}")
    print()

    out = BASE / "data/diff_ab.json"
    out.write_text(json.dumps(
        {"tolerances": TOL, "rows": rows, "fails": fails}, indent=2))

    n_out = sum(1 for r in rows if not r["within"])
    if fails:
        print(f"DIFF FAILS: {n_out} of {len(rows)} fields outside tolerance")
        for f in fails:
            print("  -", f)
        print(f"\nwrote {out.relative_to(BASE)}")
        return 1
    print(f"DIFF PASSES: all {len(rows)} fields within tolerance")
    print(f"wrote {out.relative_to(BASE)}")
    return 0


def selftest():
    """Both directions, from synthetic literals.

    A diff script that cannot fail is worse than no diff script: it produces a
    signed-off migration and a false sense of rigour. So this asserts that a
    difference just inside tolerance passes and one just outside fails, per
    field, rather than trusting the arithmetic by inspection.
    """
    bad = []
    for f in FIELDS:
        lim = TOL[f]["abs"]
        base = {r: {k: 0.0 for k in FIELDS} for r in RUNS}
        # just inside
        inside = json.loads(json.dumps(base))
        inside["straight"][f] = lim * 0.99
        _, fails = compare(base, inside)
        if fails:
            bad.append(f"  {f}: a diff of {lim * 0.99} (inside +/-{lim}) FAILED")
        # just outside
        outside = json.loads(json.dumps(base))
        outside["straight"][f] = lim * 1.01
        _, fails = compare(base, outside)
        if not fails:
            bad.append(f"  {f}: a diff of {lim * 1.01} (outside +/-{lim}) PASSED")
    # a missing run must fail rather than be skipped silently
    _, fails = compare({"straight": {k: 0.0 for k in FIELDS}},
                       {"straight": {k: 0.0 for k in FIELDS}})
    if not fails:
        bad.append("  a missing 'spin' run passed silently")
    if bad:
        print("SELFTEST FAILED:")
        print("\n".join(bad))
        return 1
    print(f"selftest ok: {len(FIELDS)} fields checked both sides of tolerance, "
          f"plus a missing run")
    return 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    sys.exit(main())
