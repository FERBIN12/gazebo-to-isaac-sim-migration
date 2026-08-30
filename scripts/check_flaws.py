#!/usr/bin/env python3
"""Prove the FOUR deliberate flaws are still in the source robot.

Sections 3, 4, 6 and 7 each teach a migration failure that only exists because
the source xacro is imperfect in a specific, typical way. A well-meaning cleanup
that "fixes" the robot would silently destroy those notes, and nothing would
notice until a take recorded a failure that no longer happens.

Exit 0 = all four flaws present. Exit 1 = a notes just lost its subject.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
XACRO = ROOT / "sim-workspace/src/rover_description/urdf/rover.urdf.xacro"

CHECKS = [
    ("FLAW-1 inertia", r'ixx="0\.0446"',
     "base_link's uniform-solid inertia (a later check derives the real tensor)"),
    ("FLAW-2 joint limit", r'<limit effort="12\.0" velocity="8\.0"/>',
     "the 8 rad/s wheel limit below Nav2's 0.8 m/s request (a later check)"),
    ("FLAW-3 gz-only plugin", r"LinearBatteryPlugin",
     "the battery plugin with no Isaac equivalent (a later check)"),
    ("FLAW-4 rate mismatch", r"<update_rate>30</update_rate>",
     "the 30 Hz depth camera against a 1 ms physics step (a later check)"),
]


def main() -> int:
    if not XACRO.exists():
        print(f"check_flaws: {XACRO} not found", file=sys.stderr)
        return 2
    text = XACRO.read_text()
    bad = []
    for name, pattern, why in CHECKS:
        if re.search(pattern, text):
            print(f"  ok   {name}")
        else:
            print(f"  GONE {name}: {why}")
            bad.append(name)
    if bad:
        print(f"\ncheck_flaws: {len(bad)} deliberate flaw(s) missing. "
              f"A notes just lost its subject.", file=sys.stderr)
        return 1
    print("\ncheck_flaws: OK, all four flaws present.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
