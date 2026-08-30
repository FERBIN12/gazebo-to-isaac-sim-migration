# Gazebo → Isaac Sim: migrating a working ROS 2 robot without breaking it

A diff-drive rover — LiDAR, depth camera, `ros2_control` diff-drive stack — built once in Gazebo Harmonic under ROS 2 Jazzy, then migrated into NVIDIA Isaac Sim through the URDF importer. The question this repo answers isn't "does it run in both simulators" — it's **where the two engines quietly disagree, by how much, and why**.

The source robot is deliberately imperfect in four tagged ways (`FLAW-1`..`FLAW-4` in the URDF comments, checked by `scripts/check_flaws.py`), each chosen because it produces a divergence between Gazebo and Isaac that a naive migration would miss entirely.

## What's actually in here

```
sim-workspace/
  src/rover_description/   # the ROS 2 package: URDF/xacro, ros2_control config, Gazebo world, launch file
  isaac/rover.usd/          # the migrated robot as a USD stage (physics, materials, geometry payloads)
  RUN.md                    # a 800+ line running log of every measurement, in the order it happened
scripts/                    # standalone probes and diff harnesses (see below)
data/                       # the JSON output of every probe — the actual measured numbers
docs/ACCEPTED_APPROXIMATIONS.md   # defects found, bounded, costed, and deliberately not fixed
```

There is no video, audio, or slide content here — this is the simulation/robotics engineering only.

## The measurement harness

Rather than eyeballing "it looks the same in both," every claim is backed by a script that produces a number:

- **`baseline_gz.py` / `baseline_isaac.py` / `diff_ab.py`** — drive the same commanded trajectory in both engines, then diff position, orientation, and velocity fields with an explicit tolerance. First honest A/B: 6 of 8 fields pass, 2 fail — traced to PhysX drive damping acting as an implicit velocity-tracking gain that Gazebo's ODE backend doesn't have.
- **`determinism_gz.py` / `determinism_isaac.py`** — is either engine bit-reproducible across identical runs? (Answer: Gazebo isn't — 7.5 mm spread over three runs; Isaac is, to 1e-9.)
- **`rtf_probe.py` / `rtf_isaac.py`** — real-time factor under load, measured with rendering on and off, because a number quoted from the wrong regime is worse than no number.
- **`readback_isaac.py`** — imports the URDF, reads the resulting USD stage back out, and diffs it field-by-field against the source. 44 of 45 values matched exactly; the one that didn't turned out to be a reader bug, not an importer bug — four separate reader bugs, in fact, each of which looked like a real divergence until isolated.
- **`friction_probe.py` / `slip_probe.py` / `slip_probe_gz.py`** — does the URDF's per-link friction actually bind to anything at the physics level, in either engine?
- **`caster_probe.py` / `ground_probe.py` / `vibration_probe.py` / `vibration_probe_gz.py`** — contact and suspension behavior under the caster wheel, checked for a vibration mode hypothesized but never actually observed (a negative result is reported as a negative result, not omitted).
- **`check_com.py` / `check_inertia_plausible.py` / `check_meshes.py` / `check_units.py`** — static consistency checks: does the imported stage's center of mass, inertia, and mesh set match what the URDF declares?
- **`damping_sweep.py` / `divergence_shapes.py` / `interface_audit.py`** — parameter sweeps and structural audits used to separate "this diverges because of a real physics difference" from "this diverges because the two harnesses aren't measuring the same thing."

Every probe's actual output is checked into `data/*.json` — the numbers in `docs/ACCEPTED_APPROXIMATIONS.md` are read from these files, not asserted.

## What the approximations file is for

`docs/ACCEPTED_APPROXIMATIONS.md` documents defects that were found and *deliberately not fixed*, each with a measured cost and the operating regime the measurement holds under. Example: the chassis inertia is modeled as a solid box when the real part is a shell, which under-states `izz` by up to 1.69x — but under the drive gain this robot actually uses, that shows up as a 0.51% difference in a 2-second spin, rising to 4.2% only if the controller is weakened by 50x. The decision to leave it as a solid box is recorded next to the number that justifies it, not as a TODO with no bound attached.

## Running it

```bash
cd sim-workspace
colcon build --packages-select rover_description
export ROS_DOMAIN_ID=77 GZ_PARTITION=c5rover   # isolates this stack's ROS graph + Gazebo transport bus
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rover_description gz.launch.py             # with GUI
ros2 launch rover_description gz.launch.py gui:="-s -v4"  # headless
```

`RUN.md` has the full diagnostic trail — including a session-costing bug where two unrelated ROS 2 / Gazebo stacks on the same machine shared a DDS graph and Gazebo transport bus, and the fix (domain ID + partition isolation, not `ROS_LOCALHOST_ONLY`, which is deprecated in Jazzy and makes discovery worse).

For the Isaac side, `scripts/import_rover.py` re-imports the URDF from a script (not the GUI dialog) so the import config is versioned and reproducible; `scripts/readback_isaac.py` verifies the result.

## Stack

ROS 2 Jazzy · Gazebo Harmonic (`ros_gz`, `gz_ros2_control`) · NVIDIA Isaac Sim 6.0.1 · Python
