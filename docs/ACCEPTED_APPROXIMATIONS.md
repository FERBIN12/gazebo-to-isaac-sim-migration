# Accepted approximations

Defects that were found, bounded, costed, and deliberately NOT fixed. Each one
has a measured bound and the regime the measurement was taken in, because a
number of this kind is not reproducible without both.

**An unfixed defect nobody wrote down is technical debt. An unfixed defect with
a measured bound beside it is a decision.** This file is what makes the second
one true.

---

## 1. The chassis inertia is a solid-box approximation

**What is written:** `base_link` inertia `ixx 0.0446, iyy 0.0850, izz 0.1192`,
which is the uniform-solid value for a 0.4 x 0.28 x 0.10 m box of 6 kg.

**What the part is:** a shell with a battery and boards inside it. The solid
value is therefore a LOWER bound on the true inertia.

**The bound:** a thin-walled shell of the same outside dimensions and mass has
**1.69x** the solid value on every axis (numerically integrated over the six
faces). So the true `izz` lies in **[0.1192, 0.2013]**.

**What it costs, measured:**

| chassis inertia | 2 s spin at 1 rad/s, drive gain 1.0 |
|---|---|
| as written | 1.551370 rad |
| as a thin shell | 1.543421 rad |
| **difference** | **7.9 mrad = 0.51 %** |

**And under a weaker controller:**

| drive gain | difference |
|---|---|
| 1.0 | 0.51 % |
| 0.02 | **4.22 %** |

**Decision: not fixed.** Under the control regime this robot uses, the entire
uncertainty is 0.51 % of a spin, inside the declared A/B tolerance.

**Reread this before tuning drive gains.** The 8x amplification means the same
approximation becomes a 4 % effect in a soft-drive regime.

---

## 2. The mast's inertia axes are permuted by the importer

**What happens:** the mast's axial moment (6.0e-5) comes back on Y instead of
Z, via `physics:principalAxes = (0.5, 0.5, 0.5, 0.5)` — a 120 deg rotation
about (1,1,1) that cyclically permutes the axes. Same three values, wrong axes,
so any sorted-magnitude or trace comparison passes it.

**What it costs, measured:** 5.1 mrad over a 1.551 rad spin = **0.33 %**,
against a run-to-run noise floor of 0.00000 rad. Real, and small.

**Decision: not fixed.** It could be fixed by writing the tensor in the
principal frame the importer expects. On a 300 g link at ~4 % of the robot's
mass, 0.33 % does not justify diverging the description from what a reader
would expect to see.

**Caveat, unmeasured:** the mast carries the lidar. 0.33 % of yaw is nothing
for navigation and may not be nothing for a scan assembled while the robot
rotates, where the error accumulates over the scan rather than the manoeuvre.

---

## 3. The Nav2 velocity exceeds the joint limit (FLAW-2)

**Not an approximation — a real configuration defect**, recorded here because
the decision is deliberate and deferred rather than applied now.

`controllers.yaml` asks for `linear.x.max_velocity: 0.8`. The wheel joints
declare `<limit velocity="8.0"/>`, which at r=0.075 is **0.600 m/s**.

**Measured:** PhysX enforces the limit exactly; Gazebo ignores it entirely.

| commanded | steady reached |
|---|---|
| 5.33 rad/s (0.400 m/s) | 5.321 (0.399) — tracks |
| 8.00 rad/s (0.600 m/s) | 7.993 (0.599) — at the limit |
| **10.67 rad/s (0.800 m/s)** | **7.981 (0.599) — CLAMPED** |
| 16.00 rad/s (1.200 m/s) | 7.923 (0.594) — clamped |

So Nav2 asks for 0.8 m/s and gets 0.599: a **25.1 % shortfall**, with no error
raised anywhere. Every Nav2 duration estimate is a quarter optimistic, and the
symptom is a robot that is consistently late in a way that looks like tuning.

**Decision: the limit stays at 8.0 rad/s.** It is a fact about the motors and
gearbox, not a negotiation between simulators. Nav2 asking for more was always
a bug; Gazebo was hiding it.

**Do not "fix" this by raising the effort limit.** The clamp is on joint
velocity, not torque — the 16 rad/s command settles slightly LOWER (7.923) than
the 8 rad/s one, which is the signature of a velocity clamp rather than a
saturating force.

---

## The regime, stated once

Every number above was measured with:

- the 2 s spin at 1.0 rad/s, angular accel 2.0 rad/s^2
- PhysX at 200 Hz, drive damping 1.0 unless stated
- Isaac against Isaac, one attribute changed, never across engines

Change any of those and the numbers change. That is not a weakness of the
measurements; it is what "how much does this matter" actually means.
