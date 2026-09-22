"""Measure the QoS every publisher on this stack OFFERS, and which subscriber
profiles would silently get nothing from it.

Write-up 7.3 is about "the subscriber that silently got nothing", and I hit that
failure myself in 7.1: I subscribed to /tf_static with default QoS and received
zero messages, which made every fixed joint in the robot read as absent.

WHY THIS IS SILENT. DDS matches a publisher and a subscriber only if the
subscriber's requested QoS is COMPATIBLE with what the publisher offers. When it
is not, there is no error, no warning and no log line -- the two simply never
connect. `ros2 topic list` shows the topic, `ros2 topic info` shows one publisher
and one subscriber, and no message ever arrives.

THE TWO RULES THAT ACTUALLY BITE (requested vs offered):
  * RELIABILITY: a RELIABLE subscriber cannot read a BEST_EFFORT publisher.
    The reverse is fine. Sensor data is conventionally BEST_EFFORT, so a node
    written with default (RELIABLE) QoS gets nothing from a sensor topic.
  * DURABILITY: a TRANSIENT_LOCAL subscriber cannot read a VOLATILE publisher.
    The reverse is fine, BUT a VOLATILE subscriber to a TRANSIENT_LOCAL topic
    connects and then misses the latched history -- which for /tf_static is the
    entire content, because it is published once at startup.

That second case is the one that got me, and it is worse than incompatibility:
the connection SUCCEEDS and you still receive nothing, because everything was
sent before you arrived.

Usage:
    scripts/qos_probe.py                  # needs a running stack
    python3 scripts/qos_probe.py --selftest
"""
import json
import re
import subprocess
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/qos_probe.json"


def compatible(offered_rel, offered_dur, want_rel, want_dur):
    """Is a subscriber requesting (want_*) able to match (offered_*)?

    Returns (matches, receives_history, reason). `matches` is DDS
    compatibility; `receives_history` is whether a late joiner actually gets
    the already-published data, which is a separate question and the one that
    bit me.
    """
    if want_rel == "RELIABLE" and offered_rel == "BEST_EFFORT":
        return False, False, "RELIABLE subscriber vs BEST_EFFORT publisher"
    if want_dur == "TRANSIENT_LOCAL" and offered_dur == "VOLATILE":
        return False, False, "TRANSIENT_LOCAL subscriber vs VOLATILE publisher"
    # matches. but does a late joiner see what was already sent?
    history = not (offered_dur == "TRANSIENT_LOCAL" and want_dur == "VOLATILE")
    reason = ("matches, but a late joiner misses the latched history"
              if not history else "matches")
    return True, history, reason


def topic_qos(topic):
    """The QoS each publisher on `topic` offers."""
    p = subprocess.run(["ros2", "topic", "info", topic, "-v"],
                       capture_output=True, text=True, timeout=30)
    pubs = []
    cur = None
    in_pub = False
    for line in p.stdout.splitlines():
        if "Endpoint type: PUBLISHER" in line:
            in_pub = True
            cur = {"reliability": None, "durability": None}
        elif "Endpoint type: SUBSCRIPTION" in line:
            if in_pub and cur:
                pubs.append(cur)
            in_pub = False
            cur = None
        elif in_pub and cur is not None:
            m = re.search(r"Reliability:\s*(\S+)", line)
            if m:
                cur["reliability"] = m.group(1)
            m = re.search(r"Durability:\s*(\S+)", line)
            if m:
                cur["durability"] = m.group(1)
    if in_pub and cur:
        pubs.append(cur)
    return pubs


def selftest():
    ok = True
    # THE TWO INCOMPATIBLE CASES must be rejected
    m, _, _ = compatible("BEST_EFFORT", "VOLATILE", "RELIABLE", "VOLATILE")
    if m:
        print("  FAIL RELIABLE sub vs BEST_EFFORT pub reported compatible"); ok = False
    m, _, _ = compatible("RELIABLE", "VOLATILE", "RELIABLE", "TRANSIENT_LOCAL")
    if m:
        print("  FAIL TRANSIENT_LOCAL sub vs VOLATILE pub reported compatible"); ok = False
    # THE REVERSES must be accepted
    m, _, _ = compatible("RELIABLE", "VOLATILE", "BEST_EFFORT", "VOLATILE")
    if not m:
        print("  FAIL BEST_EFFORT sub vs RELIABLE pub reported incompatible"); ok = False
    m, h, _ = compatible("RELIABLE", "TRANSIENT_LOCAL", "RELIABLE", "TRANSIENT_LOCAL")
    if not (m and h):
        print("  FAIL matching TRANSIENT_LOCAL pair rejected"); ok = False
    # THE CASE THAT BIT ME: matches, but no history. Must be flagged separately.
    m, h, why = compatible("RELIABLE", "TRANSIENT_LOCAL", "RELIABLE", "VOLATILE")
    if not m:
        print("  FAIL VOLATILE sub vs TRANSIENT_LOCAL pub should MATCH"); ok = False
    if h:
        print("  FAIL VOLATILE sub should NOT receive latched history"); ok = False
    if "latched" not in why:
        print(f"  FAIL reason did not mention the history: {why}"); ok = False
    # and a plain volatile pair does get its (empty) history
    m, h, _ = compatible("RELIABLE", "VOLATILE", "RELIABLE", "VOLATILE")
    if not (m and h):
        print("  FAIL plain volatile pair flagged"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()

    aud = BASE / "data/interface_audit.json"
    topics = sorted(json.loads(aud.read_text())["expected"]) if aud.exists() else []
    topics += ["/tf_static"]

    # the profiles a node might plausibly be written with
    PROFILES = {
        "default (RELIABLE/VOLATILE)": ("RELIABLE", "VOLATILE"),
        "sensor_data (BEST_EFFORT/VOLATILE)": ("BEST_EFFORT", "VOLATILE"),
        "latched (RELIABLE/TRANSIENT_LOCAL)": ("RELIABLE", "TRANSIENT_LOCAL"),
    }

    res = {}
    for t in topics:
        pubs = topic_qos(t)
        if not pubs:
            res[t] = {"publishers": 0}
            continue
        o = pubs[0]
        entry = {"publishers": len(pubs), "offered": o, "profiles": {}}
        for name, (wr, wd) in PROFILES.items():
            m, h, why = compatible(o["reliability"], o["durability"], wr, wd)
            entry["profiles"][name] = {"matches": m, "gets_history": h,
                                       "reason": why}
        res[t] = entry

    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    for t, e in res.items():
        if not e.get("publishers"):
            print(f"{t:44s} NO PUBLISHER")
            continue
        o = e["offered"]
        print(f"{t:44s} offers {o['reliability']}/{o['durability']}")
        for name, v in e["profiles"].items():
            flag = "OK  " if v["matches"] and v["gets_history"] else (
                   "HIST" if v["matches"] else "FAIL")
            print(f"    [{flag}] {name:38s} {v['reason']}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
