"""Measure what the Gazebo sensors ACTUALLY publish, against what they declare.

The URDF declares a lidar at 10 Hz with 640 samples over +/-2.356 rad and a
depth camera at 30 Hz at 640x480. A comment in the same file calls the rate
mismatch "FLAW-4" and asserts it reproduces a 0.33x bug seen elsewhere.

That comment is a PREDICTION. a later check of this project exists because I once
took a comment in this very file as a finding, and a later check and 5.5 exist
because curriculum rows turned out to be predictions too. So this measures.

WHAT IT CHECKS, per sensor:
  * declared rate vs achieved rate, over a fixed sim-time window
  * declared sample count vs the array that actually arrives
  * declared range vs the min/max finite return observed
  * how many returns are inf/nan, because a lidar pointed at open sky
    legitimately returns inf and that is not a fault

RATES ARE MEASURED ON THE SIM CLOCK. a later check was a day lost to integrating
against the wall clock while the simulator ran on its own; a rate in particular
is meaningless without saying which clock it is per second of.

Usage:
    scripts/sensor_probe_gz.py            # needs a running gz sim
    python3 scripts/sensor_probe_gz.py --selftest
"""
import json
import math
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/sensor_probe_gz.json"

WINDOW_S = 4.0          # sim seconds to observe
DECLARED = {
    "scan":  {"rate_hz": 10.0, "samples": 640,
              "range_min": 0.12, "range_max": 12.0},
    "depth": {"rate_hz": 30.0, "width": 640, "height": 480,
              "clip_near": 0.10, "clip_far": 10.0},
}


def rate_verdict(declared, achieved, tol=0.15):
    """Fraction of the declared rate achieved, and whether it is acceptable."""
    if declared <= 0:
        return None, False
    frac = achieved / declared
    return round(frac, 3), abs(frac - 1.0) <= tol


def selftest():
    ok = True
    f, good = rate_verdict(10.0, 10.0)
    if not (f == 1.0 and good):
        print("SELFTEST FAIL: an exact rate should read 1.0 and pass"); ok = False
    f, good = rate_verdict(30.0, 10.0)
    if not (abs(f - 0.333) < 0.001 and not good):
        print("SELFTEST FAIL: a 0.33x rate should read 0.333 and FAIL"); ok = False
    f, good = rate_verdict(10.0, 10.8)
    if not good:
        print("SELFTEST FAIL: 8 % over should still pass a 15 % tolerance"); ok = False
    f, good = rate_verdict(10.0, 8.0)
    if good:
        print("SELFTEST FAIL: 20 % under should not pass a 15 % tolerance"); ok = False
    print("selftest: OK" if ok else "selftest: BROKEN")
    return 0 if ok else 1


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Image

    rclpy.init()
    n = Node("sensor_probe_gz")
    obs = {"scan": [], "depth": []}
    scan_meta = {}
    depth_meta = {}

    def stamp_of(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_scan(m):
        obs["scan"].append(stamp_of(m))
        if not scan_meta:
            finite = [r for r in m.ranges if math.isfinite(r)]
            scan_meta.update({
                "samples": len(m.ranges),
                "angle_min": round(m.angle_min, 4),
                "angle_max": round(m.angle_max, 4),
                "range_min_declared_in_msg": round(m.range_min, 4),
                "range_max_declared_in_msg": round(m.range_max, 4),
                "finite_returns": len(finite),
                "infinite_returns": len(m.ranges) - len(finite),
                "observed_min": round(min(finite), 4) if finite else None,
                "observed_max": round(max(finite), 4) if finite else None,
            })

    def on_depth(m):
        obs["depth"].append(stamp_of(m))
        if not depth_meta:
            depth_meta.update({"width": m.width, "height": m.height,
                               "encoding": m.encoding})

    n.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    n.create_subscription(Image, "/depth", on_depth, qos_profile_sensor_data)

    # Spin until WINDOW_S of SIM time has passed, judged by the stamps we
    # receive. Fall back to a generous wall-clock guard so a stalled sim cannot
    # hang the probe forever.
    guard = time.time()
    first = None
    while True:
        rclpy.spin_once(n, timeout_sec=0.01)
        stamps = obs["scan"] + obs["depth"]
        if stamps:
            first = first if first is not None else min(stamps)
            if max(stamps) - first >= WINDOW_S:
                break
        if time.time() - guard > WINDOW_S * 20 + 30:
            print("  WARNING: wall-clock guard fired", file=sys.stderr)
            break

    result = {"engine": "gazebo-dart", "window_s": WINDOW_S, "sensors": {}}
    for name in ("scan", "depth"):
        st = sorted(obs[name])
        if len(st) < 2:
            result["sensors"][name] = {"messages": len(st),
                                       "verdict": "NO DATA"}
            print(f"  {name:>6}: NO DATA ({len(st)} messages)")
            continue
        span = st[-1] - st[0]
        achieved = (len(st) - 1) / span if span > 0 else 0.0
        decl = DECLARED[name]["rate_hz"]
        frac, good = rate_verdict(decl, achieved)
        entry = {"messages": len(st), "span_sim_s": round(span, 3),
                 "declared_rate_hz": decl,
                 "achieved_rate_hz": round(achieved, 2),
                 "fraction_of_declared": frac,
                 "rate_ok": good,
                 "declared": DECLARED[name]}
        entry.update(scan_meta if name == "scan" else depth_meta)
        result["sensors"][name] = entry
        print(f"  {name:>6}: {len(st)} msgs over {span:.2f} s sim  "
              f"-> {achieved:.2f} Hz of {decl:.0f} declared "
              f"({frac:.3f}x) {'ok' if good else 'OFF'}")

    if scan_meta:
        print(f"          scan samples {scan_meta['samples']} "
              f"(declared {DECLARED['scan']['samples']}), "
              f"{scan_meta['infinite_returns']} infinite returns")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"  -> {OUT.relative_to(BASE)}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
