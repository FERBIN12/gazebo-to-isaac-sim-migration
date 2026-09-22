"""Measure the real-time factor honestly, under stated load, on this machine.

Write-up 8.5 is "Performance: real-time factor, honestly", and the honesty is the
hard part: RTF is not a property of a simulator. It is a property of a
simulator, a scene, a machine, and whatever else that machine is doing. A single
number with none of that stated is not a measurement.

So this records the conditions with the result, and takes several samples rather
than one, because 5.3 and 7.1 both went wrong on a single reading.

WHAT IT RECORDS ALONGSIDE THE RTF:
  * load average at the start and end of the window
  * how many CPUs the box has, so the load is interpretable
  * the sampling window and the number of samples
  * min/median/max, not a mean, because a mean hides a stall

WHY NOT A MEAN. A simulator that runs at 1.0 for nine seconds and stalls for one
averages 0.9, which reads as "slightly slow" rather than "stalled". The median
and the min say different things and both matter.

Usage:
    scripts/rtf_probe.py [--window 20]     # needs a running stack
    python3 scripts/rtf_probe.py --selftest
"""
import json
import os
import statistics
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/rtf_probe.json"


def summarise(samples):
    """min / median / max, and the fraction of samples below 0.9.

    The stall fraction is the number that a mean would hide.
    """
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
    # a steady simulator: every statistic agrees
    s = summarise([1.0] * 10)
    if not (s["min"] == s["median"] == s["max"] == 1.0 and s["frac_below_0.9"] == 0.0):
        print(f"  FAIL steady case {s}"); ok = False
    # THE CASE A MEAN HIDES: nine good samples and one stall
    s = summarise([1.0] * 9 + [0.0])
    if s["mean"] != 0.9:
        print(f"  FAIL mean of the stall case is {s['mean']}, expected 0.9"); ok = False
    if s["min"] != 0.0:
        print(f"  FAIL min did not see the stall: {s['min']}"); ok = False
    if s["median"] != 1.0:
        print(f"  FAIL median moved on one stall: {s['median']}"); ok = False
    if s["frac_below_0.9"] != 0.1:
        print(f"  FAIL stall fraction {s['frac_below_0.9']}, expected 0.1"); ok = False
    # THE POINT OF THE SUMMARY: a steadily-slow sim and a stalling sim have
    # nearly the same MEAN and must be distinguishable by min and stall
    # fraction. Assert both halves.
    steady = summarise([0.9] * 10)          # mean 0.90, never stalls
    stall = summarise([1.0] * 9 + [0.0])    # mean 0.90, stalls once
    if abs(steady["mean"] - stall["mean"]) > 0.001:
        print(f"  FAIL fixture means differ: {steady['mean']} vs {stall['mean']}")
        ok = False
    if steady["min"] == stall["min"]:
        print("  FAIL min cannot distinguish steady-slow from stalled"); ok = False
    if steady["frac_below_0.9"] == stall["frac_below_0.9"]:
        print("  FAIL stall fraction cannot distinguish them either"); ok = False
    # empty input must be None, not a crash or a zero
    if summarise([]) is not None:
        print("  FAIL empty input did not return None"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()
    window = 20.0
    if "--window" in sys.argv:
        window = float(sys.argv[sys.argv.index("--window") + 1])

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock

    load_start = os.getloadavg()
    ncpu = os.cpu_count()

    rclpy.init()
    n = Node("rtf_probe")
    marks = []          # (wall, sim) pairs

    def on_clock(m):
        marks.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9))

    n.create_subscription(Clock, "/clock", on_clock, 10)
    t0 = time.time()
    while time.time() - t0 < window:
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.0005)
    n.destroy_node(); rclpy.shutdown()
    load_end = os.getloadavg()

    if len(marks) < 3:
        print(f"only {len(marks)} clock messages in {window}s; is the sim running?")
        return 1

    # per-second RTF samples, so a stall shows up as a low sample rather than
    # being averaged away across the whole window
    samples = []
    bucket_start = marks[0]
    for wall, sim in marks[1:]:
        if wall - bucket_start[0] >= 1.0:
            dw = wall - bucket_start[0]
            ds = sim - bucket_start[1]
            samples.append(ds / dw)
            bucket_start = (wall, sim)

    overall = (marks[-1][1] - marks[0][1]) / (marks[-1][0] - marks[0][0])
    res = {"window_s": window, "ncpu": ncpu,
           "load_start": load_start, "load_end": load_end,
           "clock_msgs": len(marks),
           "overall_rtf": round(overall, 4),
           "per_second": summarise(samples)}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    print(f"window          {window:.0f} s")
    print(f"cpus            {ncpu}")
    print(f"load  start     {load_start[0]:.2f}   end {load_end[0]:.2f}")
    print(f"clock messages  {len(marks)}")
    print(f"overall RTF     {overall:.4f}")
    s = res["per_second"]
    print(f"per-second RTF  n={s['n']} min={s['min']} median={s['median']} "
          f"max={s['max']} mean={s['mean']}")
    print(f"  samples below 0.9: {s['frac_below_0.9']:.1%}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
