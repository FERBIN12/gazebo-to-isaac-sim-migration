"""Measure what a Gazebo depth image ACTUALLY contains, value by value.

Notes 6.4 claims both simulators produce a depth image and both are "wrong in
different ways". That is a claim about VALUES, not about rates, so measuring the
rate (6.7) does not support it. This measures the values.

The specific questions, and why each one is a question:

  * ENCODING. 32FC1 means metres as float. 16UC1 means millimetres as uint16,
    which silently quantises and clips at 65.535 m. Reading a 16UC1 buffer as
    float gives garbage that looks like noise, so the encoding is load-bearing.

  * WHAT NO-RETURN LOOKS LIKE. Gazebo can emit inf, nan, or 0 for a pixel with
    nothing in range, and each one breaks a different downstream consumer. A
    consumer that filters nan but not inf produces a point cloud with points at
    infinity; one that treats 0 as valid puts obstacles at the camera.

  * RANGE vs THE DECLARED CLIP. The URDF declares near 0.10 and far 10.0. If
    finite values appear outside that, the clip is not doing what it says.

  * PLANAR vs RADIAL. This is the one that matters and the one nobody checks.
    A "depth" image can hold Z (perpendicular distance to the image plane) or
    the euclidean range along the ray. They agree ONLY at the principal point
    and diverge as 1/cos(angle) toward the edges: at our 60 deg horizontal FOV
    a corner pixel differs by ~15%. Both conventions are called "depth".

    Measured by pointing the camera at a known flat wall: if the image holds Z,
    every pixel on the wall reads the same value. If it holds range, the edges
    read further than the centre by exactly 1/cos.

Usage:
    scripts/depth_probe_gz.py            # needs a running gz sim
    python3 scripts/depth_probe_gz.py --selftest
"""
import json
import math
import struct
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/depth_probe_gz.json"

WINDOW_S = 3.0
DECLARED = {"width": 640, "height": 480, "clip_near": 0.10, "clip_far": 10.0,
            "hfov_rad": 1.0472}


def planar_vs_radial(hfov_rad, width, height):
    """How much a corner pixel differs between the two depth conventions.

    Z and euclidean range are related by range = Z / cos(theta), where theta is
    the angle of the ray off the optical axis. Returns the corner ratio, which
    is the largest divergence in the image.
    """
    fx = width / (2 * math.tan(hfov_rad / 2))
    # corner offset in pixels from the principal point
    dx, dy = width / 2, height / 2
    theta = math.atan(math.hypot(dx, dy) / fx)
    return 1.0 / math.cos(theta), math.degrees(theta)


def classify_invalid(vals):
    """Count how no-return is spelled: inf, nan or zero."""
    n_inf = sum(1 for v in vals if math.isinf(v))
    n_nan = sum(1 for v in vals if math.isnan(v))
    n_zero = sum(1 for v in vals if v == 0.0)
    return {"inf": n_inf, "nan": n_nan, "zero": n_zero}


def selftest():
    """Both directions, on synthetic data. Never pinned to a real notes."""
    ok = True

    # planar/radial divergence must be a real, known number
    ratio, theta = planar_vs_radial(1.0472, 640, 480)
    if not (1.23 < ratio < 1.24):
        print(f"  FAIL corner ratio {ratio:.4f}, expected ~1.2332"); ok = False
    # the horizontal edge must come out at exactly hfov/2 off-axis; if the
    # geometry is wrong anywhere, these two disagree
    fx = 640 / (2 * math.tan(1.0472 / 2))
    edge = 1.0 / math.cos(math.atan((640 / 2) / fx))
    if abs(edge - 1.1547) > 1e-3:
        print(f"  FAIL horizontal edge ratio {edge:.4f}, expected 1.1547"); ok = False
    # and it must be 1.0 at the centre of a zero-FOV camera (degenerate check)
    r0, _ = planar_vs_radial(1e-9, 640, 480)
    if abs(r0 - 1.0) > 1e-6:
        print(f"  FAIL degenerate ratio {r0}, expected 1.0"); ok = False

    # invalid classifier must SEE each spelling, and not confuse them
    c = classify_invalid([float("inf"), float("nan"), 0.0, 1.5, 2.0])
    if c != {"inf": 1, "nan": 1, "zero": 1}:
        print(f"  FAIL classify_invalid {c}"); ok = False
    # a clean image must report zero of everything
    c2 = classify_invalid([1.0, 2.0, 3.0])
    if c2 != {"inf": 0, "nan": 0, "zero": 0}:
        print(f"  FAIL clean image {c2}"); ok = False

    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    n = Node("depth_probe")
    got = []

    def on_depth(m):
        if len(got) >= 3:
            return
        # decode ONE row through the principal point plus the full frame stats
        if m.encoding not in ("32FC1", "16UC1"):
            got.append({"encoding": m.encoding, "unsupported": True})
            return
        w, h = m.width, m.height
        if m.encoding == "32FC1":
            vals = list(struct.unpack(f"<{w*h}f", bytes(m.data[:w*h*4])))
            scale = 1.0
        else:
            raw = list(struct.unpack(f"<{w*h}H", bytes(m.data[:w*h*2])))
            vals = [v / 1000.0 for v in raw]   # mm -> m
            scale = 0.001
        finite = [v for v in vals if math.isfinite(v) and v != 0.0]
        mid_row = vals[(h // 2) * w:(h // 2 + 1) * w]
        mid_finite = [v for v in mid_row if math.isfinite(v) and v != 0.0]
        got.append({
            "encoding": m.encoding, "width": w, "height": h,
            "step": m.step, "is_bigendian": m.is_bigendian, "scale": scale,
            "invalid": classify_invalid(vals),
            "n_total": len(vals), "n_finite": len(finite),
            "min_finite": min(finite) if finite else None,
            "max_finite": max(finite) if finite else None,
            # centre vs edge on the middle row: the planar/radial tell
            "centre": mid_row[w // 2] if math.isfinite(mid_row[w // 2]) else None,
            "left_edge": next((v for v in mid_finite), None),
            "right_edge": next((v for v in reversed(mid_finite)), None),
        })

    n.create_subscription(Image, "/depth", on_depth, qos_profile_sensor_data)
    import time
    t0 = time.time()
    while len(got) < 3 and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node(); rclpy.shutdown()

    if not got:
        print("no /depth messages in 20 s; is gz sim running with the bridge?")
        return 1

    ratio, theta = planar_vs_radial(DECLARED["hfov_rad"],
                                    DECLARED["width"], DECLARED["height"])
    res = {"declared": DECLARED, "frames": got,
           "corner_ratio_if_radial": round(ratio, 4),
           "corner_angle_deg": round(theta, 3)}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    f = got[0]
    print(f"encoding        {f['encoding']}  ({f['width']}x{f['height']}, step {f.get('step')})")
    if f.get("unsupported"):
        print("  UNSUPPORTED encoding; not decoding further")
        return 0
    print(f"invalid pixels  {f['invalid']}")
    print(f"finite          {f['n_finite']} of {f['n_total']}")
    print(f"finite range    {f['min_finite']} .. {f['max_finite']} m")
    print(f"declared clip   {DECLARED['clip_near']} .. {DECLARED['clip_far']} m")
    print(f"centre pixel    {f['centre']}")
    print(f"row edges       {f['left_edge']} .. {f['right_edge']}")
    print(f"corner ratio if the image were RADIAL: {ratio:.4f} "
          f"(off-axis {theta:.2f} deg)")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
