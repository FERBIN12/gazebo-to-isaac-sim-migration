# Running the Gazebo baseline

## Isolation is mandatory, not optional

This machine runs other ROS 2 / Gazebo projects (an AMR workspace under
`~/amr_ws` with a full Nav2 stack and its own `gz sim` warehouse world). Without
isolation those stacks and this one **share both the ROS graph and the Gazebo
transport bus**, and the symptom is not a clean error — it is the rover driving
itself across the world with nothing commanding it, wheels pinned at the joint
velocity limit, ending up against a wall with the wheels still spinning.

That was diagnosed on 2026-08-20 and cost most of a session. Two variables fix it,
and **both** are required:

| Variable | Isolates | Without it |
|---|---|---|
| `ROS_DOMAIN_ID=77` | the ROS 2 DDS graph | another Nav2 stack's `cmd_vel` reaches our controller |
| `GZ_PARTITION=c5rover` | the Gazebo transport bus | `/clock` and `/stats` collide; `gz_ros2_control` fails to load and `controller_manager` never appears |

`ROS_LOCALHOST_ONLY=1` is **deprecated in Jazzy and makes it worse** — it broke
`gz_ros2_control` discovery entirely. Do not add it back.

## Launch

```bash
cd ~/gazebo-to-isaac-migration/sim-workspace
colcon build --packages-select rover_description
export ROS_DOMAIN_ID=77 GZ_PARTITION=c5rover
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rover_description gz.launch.py gui:="-s -v4"   # headless
ros2 launch rover_description gz.launch.py                 # with the GUI
```

Wait for **two** "Configured and activated" lines. Verify:

```bash
ros2 control list_controllers    # both active
ros2 control list_hardware_interfaces
```

`left_wheel_joint/velocity` and `right_wheel_joint/velocity` must each be
`[claimed]` exactly once. The `diff_drive_base_controller/{linear,angular}/velocity`
entries are the controller's own reference interfaces and are expected — they are
not duplicate hardware.

## The baseline measurement (a later check)

```bash
cd ~/gazebo-to-isaac-migration
ROS_DOMAIN_ID=77 python3 scripts/baseline_gz.py
```

Measured 2026-08-20, three consecutive runs, Gazebo Harmonic 9.5.0 / ROS 2 Jazzy:

| Quantity | Predicted | Run 1 | Run 2 | Run 3 |
|---|---|---|---|---|
| straight, 2.0 s @ 0.4 m/s | 0.720 m | 0.771 | 0.775 | 0.770 |
| lateral drift | 0 | -0.000 | +0.000 | +0.000 |
| yaw while straight | 0 | -0.000 | +0.000 | +0.000 |
| spin, 2.0 s @ 1.0 rad/s | 1.875 rad | 1.891 | 1.871 | 1.887 |
| peak wheel speed | 0.400 m/s | 0.400 | 0.400 | 0.400 |

**Why the prediction is 0.720 m and not 0.800 m:** the controller's
`max_acceleration` of 1.0 m/s^2 means the robot ramps. 0.40 s of ramp covering
0.080 m, then 1.60 s of cruise covering 0.640 m. The rectangle v*t is the wrong
model and would have made a correct robot look broken. This is a a later check
teaching point, not a footnote.

**Why the measurement projects into the body frame:** a diff-drive robot cannot
translate sideways. When an early version of this script diffed world-frame x/y
without rotating by the starting heading, a robot that began at a non-zero yaw
(left over from the previous test's spin) read as sliding 0.705 m sideways with
zero yaw change — kinematically impossible, and therefore a bug in the
measurement, not the robot. `drive()` now rotates the displacement into the body
frame at the start of each move. **If a measurement reports something the
kinematics forbid, suspect the measurement first.**

---

## Scene authoring notes (learned building 1.1)

**The rover's ink reaches 250 px below its baseline at scale 2.05**, measured by
rendering a probe rather than derived from `FLOOR` (the wheels and the ground
shadow both sit below it, and the constant does not predict them). With the stage
band ending at y=915 the maximum baseline is 665. Scale it linearly for other
sizes. `studioSweep({cy:770})` is the BACKDROP centre and is not the rover
baseline; conflating the two put every early scene through the caption band.

**Two rovers do not stack.** Two lanes of rover plus shadow do not fit inside
430..915. Place a comparison side by side in the single lane instead, which also
fills a frame that the stacked version left two-thirds empty.

**Titles must be <= 26 characters** to hold one line at 74 px in the 1180 px
block. A two-line title pushes the subtitle down onto the data table.

**Data keys must be <= 9 characters.** `.st-data .k` gives them a 118 px column,
which is about 9 monospace glyphs at 22 px; longer keys collide with their own
values (`publishers0`). Shorten the key, do not edit the locked theme.

**The readout column is x < 620. No robot may enter it.** LAYOUT.md rule 1. The
first cut of `s7` drove the rover's lidar mast straight through the words
"jammed, wheels turning".

**Headless `--screenshot` does not capture the chrome.** `studioChrome` builds
HTML divs with CSS entry animations, and a headless screenshot catches them at
zero opacity, so the frame looks like the title and data never rendered. C3's
own shipped scenes read the same way. `produce.py` records a real kiosk Chrome
with `ffmpeg x11grab`, where they render normally. **Verify with a kiosk render,
not a headless screenshot**, or you will chase a defect that is not there.

---

## Real GUI capture (NOT headless) — the working recipe

Verified 2026-08-21. Both simulators can be captured as real windowed GUIs on a
virtual display, so captured footage is genuine simulator video rather than an
animation of one.

### The blocker, and the fix

Gazebo's `gz-rendering-ogre2` needs a GPU-backed GL context. On a plain Xvfb it
fails with:

```
libEGL warning: pci id for fd 111: 10de:28a1, driver (null)
libEGL warning: egl: failed to create dri2 screen
```

and the window maps but paints **nothing** — captured frames are luminance 0.0
in every quadrant, which is the same signature as a clip captured before Chrome
painted, so it is easy to misread as a capture-timing problem. It is not.

This is a hybrid-graphics laptop: X runs on the Intel iGPU (`glxinfo` on the
real display reports `Mesa Intel(R) Graphics (ADL GT2)`) and the RTX 4050 is
reachable only via PRIME offload. There is no `prime-run` wrapper installed, so
set the variables directly:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

With those, `glxinfo` reports `NVIDIA GeForce RTX 4050 Laptop GPU` **on the Xvfb
display as well as the real one**, and Gazebo renders. Some libEGL warnings
still print; they are not fatal, so do not gate on them — gate on the captured
frame's luminance.

### Displays

| display | purpose |
|---|---|
| `:1587` | scene rendering (Chrome, no GPU needed) |
| `:1601` | **real simulator capture** — Gazebo and Isaac GUIs |
| `:1591` | do NOT use: another local project has Chrome + its own gz sim here |
| `:1`     | the real desktop. never. |

`:1591` was already occupied when I first tried it, and the capture returned a
an unrelated slide window instead of Gazebo. Always
check `xdotool search --name .` and list window names before capturing.

### Launch

```bash
setsid nohup env DISPLAY=:1601 \
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ROS_DOMAIN_ID=77 GZ_PARTITION=c5real \
  bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash &&
           exec ros2 launch rover_description gz.launch.py gui:="-v4"' &
```

Then size and place the window deterministically before recording, per 2.6's
scripted-layout rule:

```bash
W=$(DISPLAY=:1601 xdotool search --name "Gazebo Sim" | head -1)
DISPLAY=:1601 xdotool windowsize "$W" 1700 950
DISPLAY=:1601 xdotool windowmove "$W" 60 40
DISPLAY=:1601 xdotool windowraise "$W"
```

### Verified working state

The captured frame shows the real GUI: entity tree with `ground_plane`, `sun`,
`rover` and `rover_battery` (the FLAW-3 battery plugin), physics solver
`DantzigBoxedLcpSolver`, world `flat`, real-time factor 94.68%, and the rover on
the grid. Quadrant luminance 108-163, not 0.

### Recording a take

`produce.py` already supports `("real", "<file>.mp4")` and
`("realov", ("<clip>.mp4", "<overlay>.html"))` segments, so real footage drops
into a notes plan beside scene clips with no pipeline changes. Record with the
same x11grab call `render_one.sh` uses, pointed at `:1601`.

**Check the frame, not the log.** A black capture is the failure mode, and it
looks identical to success in every log line.

### Isaac Sim GUI, and both at once — VERIFIED 2026-08-21

Isaac Sim runs windowed on the same display with the same offload env:

```bash
setsid nohup env DISPLAY=:1601 \
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ~/.venv-isaacsim/bin/python your_script.py &
```

with `SimulationApp({"headless": False, "width": 1600, "height": 900})`. The
window is named `Isaac Sim Python 6.0.1`. Captured frame shows the real
application: RTX - Real-Time 2.0 renderer, Perspective viewport with a
ray-traced ground plane, Stage panel with physicsScene and World, the Content
browser with Robots / IsaacLab / Environments, and the Z-up axis gizmo.

**MEASURED VRAM, and it corrects an assumption this project has been repeating.**

| state | VRAM used of 6141 MiB |
|---|---|
| idle | 17 MiB |
| Gazebo GUI alone | ~520 MiB |
| Isaac Sim GUI alone | **1755 MiB** |
| **both, simultaneously** | **2357 MiB (38%)** |

RE-MEASURED with both VRAM and RSS, which reconciles the two claims:

| state | VRAM of 6141 MiB | RSS |
|---|---|---|
| idle | 17 MiB | - |
| Gazebo GUI alone | 592 MiB | 1452 MiB |
| Isaac + Gazebo together | **2325 MiB (38%)** | isaac 5535 + gz 901 |
| system RAM, both up | - | **11721 / 15676 MiB (75%)** |

An earlier measurement's "Kit alone is 6.2-6.4 GB" was correct about RSS -- Isaac's resident
set really is ~5.5 GB -- but that is SYSTEM memory, not VRAM. The distinction
matters for this project because it changes which resource is scarce:

  * VRAM is NOT the constraint. Both simulators use 38% of the card.
  * SYSTEM RAM is the constraint. Both up puts the machine at 75%, and the
    remaining ~4 GB is what a browser, a recorder and ffmpeg have to share.

So the honest rule for a side-by-side take is: both simulators, plus x11grab,
and nothing else. Not because the GPU is full, but because system RAM is. And
the "run them in sequence" fallback in 2.6 is about RAM headroom on smaller
machines, not about the GPU.

Verified side by side in one capture at 1920x1080: Gazebo left with the rover,
ground_plane, sun and rover_battery, physics engine dartsim, collision detector
ode, solver DantzigBoxedLcpSolver, RTF 85.87%; Isaac right with RTX Real-Time
2.0 and its own physicsScene. One screen, one grab, no compositing.

---

## The URDF import, and the readback diff (a later check / 3.2)

Verified 2026-08-21, Isaac Sim 6.0.1, `isaacsim.asset.importer.urdf`.

```bash
~/.venv-isaacsim/bin/python scripts/import_rover.py   # writes the stage
python3 scripts/readback_isaac.py                     # diffs it, exit 1 on any diff
```

`import_rover.py` prints every parameter it used before importing, so the run
is reproducible and 3.1's "five guesses" are on screen as measured values.

### Three API facts, each of which cost a run

| | |
|---|---|
| `ros_package_paths` | a list of `{"name","path"}` DICTS, not path strings. A list of strings dies as `AttributeError: 'str' object has no attribute 'get'` inside `urdf_usd_converter/_impl/convert.py` |
| `usd_path` | an output **directory**. The converter writes `rover/rover.usda` plus `payloads/` inside it. It is not a single file |
| `app.close()` | does not return under `/app/fastShutdown=True`. Anything after it never runs: the first version wrote its params JSON and printed its result AFTER the close and exited 0 having done neither |

### THE MEASURED RESULT: 44 of 45 values identical, 1 differs

9 links x 5 fields (mass, inertia, origin, collision type, collision dims).

**The one real discrepancy is `mast.inertia`.** The mast is a Z-axis cylinder:
`ixx=iyy=8.5e-4` transverse, `izz=6.0e-5` axial, stated in the link frame with
no `<inertial><origin rpy>`. The imported stage holds

    physics:diagonalInertia = (6e-05, 0.00085, 0.00085)
    physics:principalAxes   = (0.5, 0.5, 0.5, 0.5)

That quaternion is a **120 deg rotation about (1,1,1)**, which cyclically
permutes x->y->z->x. Rotating the stored diagonal back into the link frame
gives `[8.5e-4, 6.0e-5, 8.5e-4]`: the axial value lands on **Y, not Z**. Same
three numbers, wrong axes. `lidar_link`, an identical Z-cylinder whose
principalAxes came back as identity, transfers correctly.

**Why no naive check catches it:** as a multiset the values are identical, so
comparing sorted magnitudes passes, and comparing raw diagonals across the two
frames flags four links of which three are false. It is a genuinely
axis-shaped defect and it needs a frame-aware comparison to see.

### Four reader bugs that each looked exactly like an importer bug

Worth keeping because every one of them is the same lesson: when the
measurement says the tool is catastrophically broken, suspect the measurement.

1. **apiSchemas live in the `( ... )` metadata block before the `{ }` body.**
   Reading only the body made every collision prim type `None` -- all nine
   links "differing in collision_type".
2. **USD nests links inside their parents**, so a whole-subtree attribute
   search returns a descendant's value. `base_footprint`, a massless frame,
   came back as 6 kg -- base_link's mass. Direct children only.
3. **A USD `Cube` has one `size`;** a non-cubic box is a unit cube with a
   non-uniform `xformOp:scale`. Reading `extent` alone reported the chassis as
   1.0 x 1.0 x 1.0 instead of 0.4 x 0.28 x 0.1.
4. **`<inertial>` carries its own `<origin rpy>`.** The wheels state their
   tensor under `rpy="1.5707963 0 0"`, so the raw numbers are NOT link-frame.
   Ignoring that reported both wheels as axis-swapped -- the same signature as
   the real mast defect, which is exactly why it had to be ruled out rather
   than assumed.

### The joint pass, and the number that looks 57x wrong and is not

`physxJoint:maxJointVelocity` on a revolute joint is **degrees** per second.
The URDF's `velocity="8.0"` (rad/s) is stored as **458.36624**, because
`urdf_to_mjc_physx_conversion_utils.py:98` does

    joint_max_velocity_deg = joint_max_velocity * 180 / 3.1415926

Read raw, that is a limit 57.29578x too permissive -- and 57.29578 is exactly
180/pi, which is the tell. **The limit is preserved correctly.** FLAW-2's
prediction (PhysX enforces the 8 rad/s that Gazebo ignores) therefore stands.
This is a G25 case: trace the number to a mechanism before calling it a defect.

Verified: 8 joints x 4 fields, all 32 values agree once units are matched.

### Two joints have no joint prim, and that is correct

Only 6 of the 8 URDF joints exist as prims. `base_joint` and
`camera_optical_joint` do not, because **one end of each is a massless frame**
(`base_footprint`, `camera_optical_frame`). PhysX constrains rigid bodies; a
pure frame is not one, so the importer bakes the transform into the Xform
hierarchy instead -- `camera_optical_frame` carries the rpy as
`xformOp:orient = (-0.5, 0.5, -0.5, 0.5)`.

The rule is "either END is massless", not "the child is": base_joint runs
base_footprint -> base_link, so its massless link is the PARENT, and a
child-only rule reported it as a missing joint. The readback asserts the
child Xform is actually present, so a frame the importer really did drop
still fails.

**Total: 77 values compared, 1 differs.**

### Selftest

`python3 scripts/readback_isaac.py --selftest` -- 11 synthetic comparison cases
plus the frame maths, built from literals rather than pinned to a real
artefact, and asserting BOTH directions. Verified to fail when `close()` is
stubbed to always return True, including the specific case that a
sorted-magnitude comparison would let a permutation through.

### The caster is NOT a mesh in 6.0.1

An earlier draft of 3.2 was written around the caster's collision sphere being
converted to a mesh. **Measured, it is not:** with `collision_type='Convex
Hull'` the importer emits `def Sphere "sphere_1"` with `PhysicsCollisionAPI`
and `radius = 0.03`, an analytic sphere. That notes's premise has to change
to the mast inertia, which is a real defect in this version.

---

## The first honest A/B diff (a later check) — it fails, and the mechanism is the drive gain

Measured 2026-08-21. Both sides from scripts, both written to JSON:

```bash
# Gazebo (needs the sim up, see Launch above)
ROS_DOMAIN_ID=77 GZ_PARTITION=c5rover python3 scripts/baseline_gz.py
# Isaac
~/.venv-isaacsim/bin/python scripts/baseline_isaac.py
# the diff, tolerances declared in the file BEFORE the measurement
python3 scripts/diff_ab.py            # exit 1 if any field is outside
python3 scripts/diff_ab.py --selftest
```

### The result: 6 of 8 fields pass, 2 fail

| run | field | gazebo | isaac | diff | verdict |
|---|---|---|---|---|---|
| straight | dx | +0.725 | +0.682 | -0.043 | ok (tol 0.072) |
| straight | dy | -0.000 | -0.000 | -0.000 | ok |
| straight | dyaw | -0.000 | -0.002 | -0.002 | ok |
| straight | peak wheel | +0.400 | +0.404 | +0.004 | ok |
| spin | dx | +0.023 | -0.002 | -0.025 | ok |
| spin | dy | +0.000 | -0.001 | -0.002 | ok |
| spin | **dyaw** | **+1.857** | **+0.926** | **-0.932** | **OUTSIDE** |
| spin | **peak wheel** | +0.209 | +0.182 | -0.027 | **OUTSIDE** |

**The robot drives straight almost identically and rotates half as far.** That
is the honest first diff, and it is what this repo onward exists to explain.

### THE MECHANISM: PhysX drive damping IS the velocity-tracking gain

The URDF says `<dynamics damping="0.01"/>`, and the importer puts that straight
into `drive:angular:physics:damping = 0.01` with `stiffness = 0`. For a PhysX
**velocity** drive, damping is the gain that pulls the joint toward its target,
and 0.01 is far too weak to hold it. Commanding 2.0 rad/s, the joint only ever
reaches 1.207 rad/s. Measured sweep, constant 2.0 rad/s target:

| drive damping | joint reached (rad/s) | dyaw (rad) |
|---|---|---|
| **0.01 (imported)** | **1.207** | **1.015** |
| 0.03 | 1.789 | 1.520 |
| 0.1 | 1.943 | 1.700 |
| 0.3 | 1.377 | 1.752 |
| 1.0 | 1.991 | **1.771** |
| 100.0 | 2.006 | 1.779 |

### Reproducing the sweep

```bash
~/.venv-isaacsim/bin/python scripts/damping_sweep.py            # -> data/damping_sweep.json
~/.venv-isaacsim/bin/python scripts/damping_sweep.py --selftest # unit + monotonicity checks
```

Two runs of the full sweep agree to 0.000 on every row. The script sets the
gain AFTER `world.reset()`, converts rad -> deg for the PhysX joint API, and
asserts the gain actually landed before recording the row.

**RESOLVED 2026-08-22: the damping 0.3 dip WAS NOT REAL.**

The hand-made table above recorded 1.377 rad/s at damping 0.3, lower than the
1.943 at 0.1, and I had written a hypothesis about drive oscillation to explain
it. `scripts/damping_sweep.py` now reproduces the sweep from scratch and the
dip does not exist:

| damping | reached (rad/s) | dyaw (rad) |
|---|---|---|
| 0.01 | 1.158 | 0.863 |
| 0.03 | 1.832 | 1.312 |
| 0.1 | 1.975 | 1.477 |
| **0.3** | **2.008** | 1.527 |
| 1.0 | 2.017 | 1.551 |
| 100.0 | 2.016 | 1.553 |

Reached speed rises monotonically to saturation; the only inversion is
2.017 -> 2.016 at the top, which is rounding at the third decimal. **Two
consecutive runs agree to 0.000 on every row**, so this is not noise now, and
the old row was an artefact of the one-off hand run (most likely the
reset-then-edit ordering lag documented below, which was found and fixed after
that table was written).

**The lesson stands even though the row did not.** A sampled statistic and an
integrated one from the same run disagreeing IS a reason to suspect the
statistic -- it is just that here the right move was to re-run the measurement
with a script, not to reason about what might explain it. The hypothesis was
never promoted to a finding, which is the only reason this cost nothing.

Yaw values are also systematically lower than the old table (1.553 vs 1.779 at
saturation). The script is the reference from here; the hand table is not.

dyaw is **monotone** in damping and saturates at 1.77-1.78, against the
closed-form prediction of 1.750 and Gazebo's measured 1.857. So a damping of
about 1.0 reproduces Gazebo. The same number means two different things in the
two engines: in Gazebo, `<dynamics damping>` is joint friction and the
controller's PID does the tracking; in Isaac there is no controller and that
number IS the tracking gain. This is a later check, "translating a tuned
controller", and it is the single hardest step in the migration.

### Two hypotheses that were WRONG, and had to be measured to be killed

Both looked obviously right and cost a run each. Worth keeping, because the
notes is more honest for showing them.

1. **"The friction annotations are inert, so the wheels slip."** True that they
   are inert — the URDF `<surface><friction><ode><mu>` is imported as a
   `custom string urdf:text` inside an empty `Scope`, and the stage binds **no
   physics material at all**. But binding the real frictions (wheels 1.0,
   caster 0.05) made the spin **worse**, 0.926 -> 0.813 rad. Not the mechanism.
2. **"The caster cannot swivel, so it drags the turn."** It is a rigid body on a
   fixed joint, so it must skid. Measured with its collision disabled:
   **0.843 rad vs 0.845 with it on.** The caster is irrelevant to the yaw.

`FRICTION=urdf ~/.venv-isaacsim/bin/python scripts/baseline_isaac.py` still
applies the URDF frictions, for the notes that needs to show hypothesis 1.

### baseline_gz.py had a wrong prediction constant

Its spin prediction read **1.875** with a comment claiming "trapezoid at
2.0 rad/s^2". It is not — 1.0 rad/s ramped at 2.0 rad/s^2 is 0.25 rad of ramp
plus 1.5 rad of cruise = **1.750 rad**; 1.875 is the trapezoid for accel 4.0,
and `controllers.yaml` says 2.0. It hid because the measured Gazebo runs
(1.857-1.891) sit close to the wrong number and so always "passed". Against the
correct 1.750 that is a real **+6.1 % overshoot**: Gazebo under-applies its own
angular acceleration limit. Fixed, and it now writes `data/baseline_gz.json`
rather than only printing, which is what let a diff script exist at all.

---

## The five divergence shapes (a later check), each one caused deliberately

```bash
~/.venv-isaacsim/bin/python scripts/divergence_shapes.py   # -> data/divergence_shapes.json
```

Every shape is produced by breaking ONE thing in the Isaac side and recording
the error against an unperturbed Isaac run. Measured 2026-08-21:

| shape | read in | first | mid | last | perturbation |
|---|---|---|---|---|---|
| offset | yaw | +0.120 | +0.120 | +0.120 | start-pose offset 0.12 rad |
| ramp | yaw | +0.000 | +0.059 | +0.142 | wheel radius 8 % small |
| saturating | **rate** | -0.010 | -0.783 | -0.831 | drive damping 0.01 |
| periodic | yaw | -0.000 | +0.005 | +0.002 | faceted wheel collision |
| step | yaw | +0.000 | -0.001 | -0.399 | velocity clamp at t = 1.0 s |

Reference repeatability: **worst |run1 - run2| = 0.00000 rad**, so nothing in
the table above is run-to-run noise.

### Three design errors, each of which would have taught the wrong lesson

1. **The closed form is not a valid reference.** Using the analytic ramp made
   all five shapes come out as the same large negative ramp with the intended
   shape riding on top as a small correction. A correct PhysX robot does not
   follow the closed form -- the drive accelerates real wheel inertia against
   real contact and lags, by much more than any perturbation here. The
   reference has to be an unperturbed run of the same simulator.

2. **A USD drive edit only reaches PhysX at the next `initialize()`.** So
   `set_damping(); world.reset()` applies the PREVIOUS run's gain, a one-run
   lag that made `saturating` come out as a flat zero. Measured:

   | sequence | joint reaches |
   |---|---|
   | edit(0.01) then reset | 1.189 rad/s |
   | edit(1.00) then reset | 1.232 rad/s (still the old gain) |
   | reset then edit(1.00) | 1.994 rad/s |

   Use `Articulation.set_gains()` after `initialize()` instead. **And note the
   units: `get_gains` reports 0.5729578 for a URDF damping of 0.01, which is
   0.01 x 180/pi.** The PhysX joint API is in degree units throughout, exactly
   like `physxJoint:maxJointVelocity`.

3. **A velocity deficit integrated into position is ALWAYS a ramp.** That is
   what integration does to a constant offset, so measuring every shape on yaw
   made `saturating` and `ramp` identical curves. The saturation lives in the
   RATE. Each shape now records which quantity it must be read in, and the
   JSON carries both series.

---

## A perfectly horizontal filtered path renders to NOTHING under swiftshader

Found 2026-08-21 building 2.8's `s6_shape_offset`, and it cost several passes
because every gate said the scene was fine.

`rigTrace` strokes its series through `filter="url(#gRef)"`, an feGaussianBlur.
The capture runs Chrome with `--use-gl=swiftshader`, and that software
rasteriser **drops a 3px stroke with zero vertical extent**. Measured inside
the plot box, same notes, same render:

| scene | series shape | coloured px in plot |
|---|---|---|
| s8_shape_ramp | sloped | 521 |
| s15_shape_step | sloped + one break | 651 |
| **s6_shape_offset** | **exactly flat** | **23** (the end dot alone) |

Headless Chrome drew all 33 points correctly, and an instrumented copy of the
same scene reported `pts=33` with a full-width `d` attribute. So the geometry,
the push loop and the kit were all correct the whole time — only the rasteriser
disagreed, and only for the flat case.

**Nothing catches this.** `check_motion` passes, because the playhead and the
dot still move. `audit_layout` passes, because the text is fine. `node --check`
and `check_globals` pass. The only way it surfaced was sampling raw pixels
along the row the line should occupy and finding background values.

**The fix:** for a series that is exactly flat, draw an unfiltered `<line>` on
top. Do not remove the filter from the kit — 20+ approved scenes rely on the
glow, and the failure only affects zero-slope segments.

**The general lesson for this project:** when a frame looks wrong but every gate
is green, sample the pixels where the missing thing should be. "The gate passed"
and "the frame is right" are different claims.

---

## Units and frames (a later check): the readback is blind to both

```bash
python3 scripts/check_units.py            # 7 layers x 3 fields
python3 scripts/check_units.py --selftest  # 5 single-layer + 1 mixed-layer case
```

The real import is clean: all 7 layers declare `metersPerUnit = 1`,
`kilogramsPerUnit = 1`, `upAxis = "Z"`.

**MEASURED, 2026-08-21 — and this is why the check is separate.** Copy the
stage, rewrite one metadata field in all 7 layers, and run both checks:

| broken stage | `readback_isaac.py` | `check_units.py` |
|---|---|---|
| `metersPerUnit = 0.01` (centimetres) | **1 of 77 differ** — identical to the correct stage | 7 failures |
| `upAxis = "Y"` | **1 of 77 differ** — identical to the correct stage | 7 failures |

The readback cannot see either, and that is not a bug in it: every length it
compares is still the same NUMBER. A 0.075 m wheel radius reads 0.075 in a
centimetre stage too. The value is preserved and the meaning is not, so a
value-by-value diff passes a robot that is 100x the wrong size or lying on its
side.

**Check EVERY layer, not the root.** The importer writes a layered stage and
each layer carries its own metadata block; USD composes a root that says metres
with a payload that says centimetres without complaint. `check_units.py`'s
selftest includes exactly that mixed case, because a root-only check passes it.

**Missing is not wrong.** A layer that declares nothing inherits, so the check
reports `inherits` and does not fail it. Only a stated disagreement fails.

---

## Meshes and `package://` (a later check): the robot now HAS a mesh

The robot was entirely primitives until 2026-08-21, even though an earlier
write-up claimed "the mesh paths in this description use the package
protocol". It now has one, so that claim is true:

`sim-workspace/src/rover_description/meshes/lidar_housing.stl` — a generated
16-sided prism (64 triangles, r=0.035, h=0.040), referenced from `lidar_link`'s
visual as `package://rover_description/meshes/lidar_housing.stl`. The link keeps
its analytic **cylinder** collision, so visual and collision now disagree by
construction, and the 16 facets set up the hull discussion in 3.7.

`meshes` was added to the `install(DIRECTORY ...)` line in CMakeLists.txt.

**Watch the XML comments.** A `--` inside an XML comment is illegal, and the
first version of the FLAW-5 comment contained two. xacro fails with
`not well-formed (invalid token)` pointing at the comment, not at the mesh.

### MEASURED: what an unresolved `package://` does

| | with `ros_package_paths` | with it EMPTY |
|---|---|---|
| layers written | **9** | 7 |
| `payloads/geometries.usd` | present, 2422 bytes, holds `lidar_housing` | **absent** |
| `payloads/instances.usda` | present | **absent** |
| importer result | success, exit 0 | **success, exit 0** |
| `readback_isaac.py` | 1 of 77 differ | **1 of 77 differ** — identical |

The import does not fail, does not warn, and does not mention the mesh in its
log at all. Two whole layers simply are not written. And the readback cannot
see it, because it compares COLLISION geometry and the lidar's collision is the
cylinder either way — the missing *visual* is outside what it looks at.

`scripts/check_meshes.py` closes that gap. Selftest: 5 stage cases plus a
no-mesh case, both directions.

### Two traps found writing this

1. **The importer silently reuses an existing output directory.** Re-running
   `import_rover.py` over an existing `rover.usd` printed `IMPORTED` and left a
   six-hour-old stage in place. `rm -rf` the output first, or you will measure
   the previous run. This is why the mesh appeared not to import at all.
2. **Meshes are NOT in `base.usda`.** The converter writes a separate binary
   geometry library (`payloads/geometries.usd`) referenced through
   `payloads/instances.usda`. Grepping `base.usda` for `def Mesh` returns
   nothing on a perfectly correct import, which is how the first pass at this
   concluded the mesh had failed to resolve when it had not.

---

## A WARM Gazebo sim reads ~7% further than a fresh one

Measured 2026-08-21 while adding the mesh, and it briefly looked like the mesh
had changed the physics. It had not.

| sim state | straight dx, runs | mean |
|---|---|---|
| **fresh launch, 5 runs** | 0.715 0.731 0.723 0.731 0.723 | **0.725** (sd 0.007) |
| left running a long time, 3 runs | 0.781 0.773 0.771 | 0.775 |

That is a **50 mm difference from nothing but sim uptime**, which is 69% of the
72 mm tolerance the A/B diff declares. Write-up 2.7 shipped quoting 0.725, and
the fresh-sim mean is exactly 0.725, so it is right -- but only because that
run happened to be on a freshly started sim.

**So: restart the sim before a baseline, and never diff a warm run against a
cold one.** The A/B rig's whole premise is that the only difference between the
two sides is the engine, and sim uptime is a difference the rig cannot see.
a later check (determinism and reproducibility) is where this belongs as a
teaching point; it is recorded here so it does not get rediscovered.

Also fixed while adding the mesh: `gz.launch.py` now sets
`GZ_SIM_RESOURCE_PATH` to the directory ABOVE the package share dir. Without
it, gz-sim rewrites `package://` to `model://`, fails to resolve it, prints
three red errors and puts the robot on the ground anyway with the lidar housing
missing. Note the contrast with Isaac, which drops the mesh **silently**.

---

## What the mast's permuted inertia actually costs (a later check)

Measured 2026-08-21 (`data/mast_inertia_cost.json`). Same 2 s spin at 1 rad/s,
drive damping 1.0, the ONLY difference being the mast's inertia:

| mast inertia | dyaw |
|---|---|
| as imported: diag (6e-5, 8.5e-4, 8.5e-4), principalAxes (0.5,0.5,0.5,0.5) | +1.551370 rad |
| as the URDF states it: diag (8.5e-4, 8.5e-4, 6e-5), identity axes | +1.556509 rad |
| **difference** | **0.005139 rad = 5.1 mrad, 0.33 %** |

**Small, and real.** `divergence_shapes.py` measured reference repeatability at
0.00000 rad, so 5.1 mrad is roughly 5000x the noise floor -- this is not run
variance. But it is 0.33 % of the manoeuvre, on a 300 g link, and the honest
notes says so rather than inflating it.

That is the right shape for the teaching point: the defect is **real,
findable, and currently harmless on this robot**. It matters because the same
importer behaviour on a heavier or more eccentric link is not harmless, and
because you cannot know which case you are in without measuring. A defect you
have quantified is a defect you can decide about; an unquantified one is just
anxiety.

**API note:** `UsdPhysics.MassAPI.Get(prim)` raises a Boost.Python
ArgumentError -- it wants `(stage, path)`. Use
`UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI) else
UsdPhysics.MassAPI.Apply(prim)`.

---

## `collision_type` does nothing unless `collision_from_visuals` is True (3.8)

Our import sets `collision_type = "Convex Hull"`, and **not one collision on
the robot is a convex hull.** Measured two ways.

**At runtime**, every collision prim is an exact analytic primitive with no
approximation attribute at all:

```
  Cylinder   (none set)   cylinder_1      <- both wheels
  Sphere     (none set)   sphere_1        <- the caster
  Cylinder   (none set)   cylinder_1      <- mast, lidar
  Cube       (none set)   box_1           <- the chassis
```

**In the importer's source**, `converter.py:210` gates it:

```python
if self.config.collision_from_visuals:
    importer_utils.collision_from_visuals(self.stage, self.config.collision_type)
```

That is the ONLY place `collision_type` is read. With
`collision_from_visuals=False` the collisions come from the URDF's own
`<collision>` elements and stay exactly what they were written as.

**Re-imported with `collision_from_visuals=True`** to confirm, and the picture
inverts: the VISUALS become the collisions, so the lidar's mesh gains

```
over "lidar_housing" ( apiSchemas = [..., "PhysicsMeshCollisionAPI"] )
{ token physics:approximation = "convexHull" }
```

and the analytic primitives are replaced by whatever the visual was.

**So the practical rule:** a setting named for a thing you want does not mean
you got it. `collision_type` is a *mesh* approximation policy, and it is inert
on a robot whose collisions are primitives. Ours is the good case -- analytic
shapes are exact, cheaper and better behaved than a hull of the same shape --
but it is good by accident of `collision_from_visuals` being off, not because
the hull setting was chosen well.

---

## The chassis inertia bound, and why a stiff drive HIDES it (3.9)

Measured 2026-08-21. The chassis is 0.4 x 0.28 x 0.10 m, 6 kg, and its written
inertia is the solid-box value. A thin-walled shell of the same outside
dimensions has **1.69x** that (numerically integrated over the six faces), so
the true value lies somewhere in that range:

| | Ixx | Iyy | Izz |
|---|---|---|---|
| solid box | 0.0442 | 0.0850 | 0.1192 |
| **written** | **0.0446** | **0.0850** | **0.1192** |
| thin shell | 0.0746 | 0.1436 | 0.2013 |

The chassis is **82 %** of the robot's rotational inertia about the yaw axis, so
that 69 % uncertainty is a **57 % error in the total**.

### And it moves the answer by 0.51 %

| chassis inertia | dyaw, 2 s spin |
|---|---|
| as written | +1.551370 rad |
| as a thin shell | +1.543421 rad |
| **difference** | **7.9 mrad = 0.51 %** |

A 57 % error in total inertia producing a 0.51 % error in yaw is not intuitive,
and it is the most useful thing in this section. **The manoeuvre is velocity
controlled.** The drive holds the commanded wheel speed regardless of what it is
pushing, so the inertia only matters during the brief ramp, and the steady
cruise erases it.

### Confirmed by weakening the drive

Same experiment, drive gain 1.0 -> 0.02:

| drive gain | dyaw as written | dyaw as shell | difference |
|---|---|---|---|
| 1.0 | 1.551370 | 1.543421 | 0.51 % |
| **0.02** | **1.191215** | **1.140972** | **4.22 %** |

**An 8x amplification from the gain alone.** So "how much does this parameter
matter" has no answer independent of the control regime you measure it in. A
stiff drive masks mass-property errors; a weak one exposes them. this repo's
tuning notes are where this stops being a curiosity: the same robot,
correctly modelled, will hide or reveal the same defect depending only on how
hard the controller is pushing.

This also qualifies 3.7's triage rule. Sorting links by m·r² tells you which
link CAN matter. Whether it DOES matter depends on the manoeuvre and the gains,
and both have to be stated with any number of this kind.

---

## FLAW-2 confirmed: PhysX enforces the joint limit Gazebo ignores (4.2)

Measured 2026-08-21. The URDF says `<limit velocity="8.0"/>` on the wheel
joints. 8 rad/s at r=0.075 is **0.600 m/s**, and `controllers.yaml` asks Nav2
for **0.8 m/s** (10.667 rad/s). Commanding past the limit:

| commanded | steady reached | verdict |
|---|---|---|
| 5.33 rad/s (0.400 m/s) | 5.321 (0.399 m/s) | tracks |
| 8.00 rad/s (0.600 m/s) | 7.993 (0.599 m/s) | tracks, at the limit |
| **10.67 rad/s (0.800 m/s)** | **7.981 (0.599 m/s)** | **CLAMPED** |
| 16.00 rad/s (1.200 m/s) | 7.923 (0.594 m/s) | clamped |

So the Nav2 configuration asks for 0.8 m/s and the robot delivers 0.6, silently,
with no error anywhere. Every Nav2 timing assumption is then 33 % optimistic.

### Measure the STEADY value, not the peak

The first run recorded the peak over the whole command and produced a
non-monotone table: 8.38 at the 8.0 command, **9.53** at the 10.67 command, and
8.01 at 16.0. That looked like the limit being exceeded and then re-imposed.

It was the ramp transient. Averaging the last 0.5 s instead gives 7.99 / 7.98 /
7.92 — flat, monotone, and obviously a clamp. **A peak includes the overshoot
on the way in; a ceiling is a steady-state property.** Reporting the wrong one
turns a clean result into a puzzle.

---

## CORRECTION: FLAW-2 is not a migration defect. Both engines clamp.

Measured 2026-08-21, and it overturns what the xacro comment has claimed since
this repo. The comment says "a velocity limit Gazebo silently ignores... but
which PhysX enforces". **Gazebo does not ignore it.**

| commanded | Gazebo wheel | Isaac wheel |
|---|---|---|
| 0.40 m/s | 5.333 rad/s (0.400) | 5.321 (0.399) |
| 0.60 m/s | 8.000 rad/s (0.600) | 7.993 (0.599) |
| **0.80 m/s** | **8.000 rad/s (0.600)** | **7.981 (0.599)** |
| 1.20 m/s | 8.000 rad/s (0.600) | 7.923 (0.594) |

**The two engines agree to within 1 %.** `gz_ros2_control` enforces the joint
`<limit velocity>` just as PhysX does.

**Where the clamp comes from:** not the `ros2_control` `command_interface`
min/max. Removing those bounds and rebuilding changes nothing — still 8.000. It
is the joint `<limit>` itself.

### What this means for this project

- **The Nav2 shortfall is REAL** and worth a notes: the config asks for
  0.8 m/s and the robot does 0.6, a 25 % gap with no error raised. But it is a
  configuration bug that was always present, in BOTH simulators, and migration
  is not what reveals it.
- **FLAW-2 as planted does not exist.** 4.2 must be rewritten around what is
  actually true, which is arguably a better notes: the same limit, enforced
  identically by two engines, and a Nav2 config that has been wrong since
  notes one without either simulator complaining.
- **I asserted the Gazebo half from a comment.** The Isaac side was measured;
  the Gazebo side was read off the xacro and repeated. Measuring both sides of
  a claimed divergence is the entire point of an A/B rig, and I skipped it on
  the side I "already knew".


---

## The wheel-velocity ripple (a later check), measured on BOTH engines

```bash
~/.venv-isaacsim/bin/python scripts/vibration_probe.py      # -> data/vibration_probe.json
~/.venv-isaacsim/bin/python scripts/vibration_probe.py --selftest
# and, against a running headless gz sim:
scripts/vibration_probe_gz.py                               # -> data/vibration_probe_gz.json
```

The curriculum row for 4.7 predicted "the vibration that appears only in
Isaac". **It is not Isaac-only.** Measured 2026-08-22, constant 1.0 rad/s body
spin, statistics taken from the steady window only (first 1.5 s discarded, so
the commanded ramp is excluded), both sides through the SAME `analyse()`:

| engine | gain | mean (rad/s) | peak-to-peak | relative ripple |
|---|---|---|---|---|
| Gazebo | n/a | 2.033 | 0.233 | **0.115** |
| Isaac | 0.01 | 1.272 | 1.456 | 1.145 |
| Isaac | 1.0 | 1.990 | 2.179 | 1.095 |
| Isaac | 100.0 | 1.993 | 1.601 | **0.804** |

**Both engines ripple. Isaac is at least 7x worse**, taking its best gain
against Gazebo. On Isaac the peak-to-peak exceeds the mean at every gain, so
the wheel speed is momentarily reaching zero or reversing. The chassis height
does NOT oscillate (p2p 0.0003 m), so this is a joint-level effect, not the
robot bouncing.

### The instrument limit, stated because it changes the claim

The Isaac probe samples every physics step (200 Hz, Nyquist 100 Hz) and finds
reversal rates of 85-130 Hz. The Gazebo probe reads `/joint_states`, published
at the controller_manager's `update_rate` of **100 Hz, so Nyquist 50 Hz**.

**A 130 Hz oscillation is invisible to a 100 Hz sampler.** The Gazebo number
above is therefore a lower bound on Gazebo's ripple, not a measurement of it,
and `vibration_probe_gz.py` reports `INCONCLUSIVE` for that band rather than
"steady". Closing this properly needs Gazebo sampled at the physics rate, not
the publish rate.

**What is safe to say:** in the band both instruments CAN see, Isaac's ripple
is far larger. What is NOT safe to say: that Gazebo is smooth.


---

## Does the solver explain the ripple? NO (a later check)

```bash
~/.venv-isaacsim/bin/python scripts/vibration_probe.py --solver
# -> data/vibration_solver_sweep.json
```

Write-up 4.8 raised the obvious hypothesis: the 4.7 wheel ripple is a solver
artefact, and raising the **articulation's** iteration counts (not the scene's)
would reduce it. Tested at damping 1.0:

| position iterations | mean (rad/s) | peak-to-peak | relative ripple |
|---|---|---|---|
| 4 | 1.833 | 2.550 | 1.391 |
| 16 | 1.925 | 2.504 | 1.301 |
| 64 | 2.004 | 2.445 | **1.220** |
| 255 | 2.015 | 2.816 | 1.398 |

**A 64x increase in solver work does not remove the oscillation.** The signal
oscillates at every setting, the ripple is not monotone in iteration count, and
it ends HIGHER at 255 than it started at 4. Hypothesis refuted.

### Two traps found writing this

1. **The importer writes NO `physxArticulation:*` attributes at all** (grep
   returns 0). `set_solver_position_iteration_counts` therefore fails with
   `Empty typeName` — the property does not exist on the prim. You have to
   `CreateAttribute` it before you can set it. Worth knowing generally: solver
   tuning on an imported robot starts by creating the knob, not turning it.

2. **A spread is not a trend.** The first version of this script reported
   `solver_changes_ripple: True` because max-min exceeded a threshold, which is
   true of any noisy series. The verdict now requires the ripple to fall
   *monotonically* AND to stop oscillating. Both are false here.


---

## Is the residual a TIMESTEP artefact? NO (a later check)

Gazebo's world declares `max_step_size 0.001`; every Isaac measurement here
ran at `1/200 s`. That is a **5x longer step on the Isaac side**, present
underneath every comparison, and an obvious candidate for the 16.4 % residual.

`physics_dt` is fixed when the `World` is constructed, so a sweep needs one
process per value:

```bash
for dt in 0.001 0.002 0.005; do
  ~/.venv-isaacsim/bin/python scripts/damping_sweep.py --dt=$dt
done
# -> data/damping_sweep_dt1000.json, _dt500.json, damping_sweep.json
```

At drive damping 1.0:

| Isaac dt | reached (rad/s) | dyaw (rad) | vs Gazebo 1.857 |
|---|---|---|---|
| **1 ms (Gazebo's own)** | 1.991 | **1.529** | **17.7 %** |
| 2 ms | 2.032 | 1.534 | 17.4 % |
| 5 ms (what we used) | 2.017 | 1.551 | 16.5 % |

**Running Isaac at Gazebo's exact timestep does not close the gap — it widens
it slightly.** The timestep is not the residual. That is a fifth elimination,
and it also means the 5x asymmetry we shipped in every earlier comparison cost
about 1 % of yaw, not 16 %.

Eliminated so far for the residual and/or the ripple: the drive gain, the
chassis, solver iterations, Isaac-exclusivity, and now the timestep. All five
measured.


## The friction annotations, proved inert (a later check)

```bash
python3 scripts/friction_probe.py --static     # no Isaac needed
python3 scripts/friction_probe.py --selftest
~/.venv-isaacsim/bin/python scripts/friction_probe.py
```

Measured 2026-08-22:

| what | count |
|---|---|
| `<mu>` values the URDF asks for | 2 (1.0 wheels, 0.05 caster) |
| `urdf:text` scopes in the imported stage | **6** |
| bound physics materials in the stage | **0** |

**Values survived: True. Meaning survived: False.** `<mu>1.0</mu>` arrives as
`custom string urdf:text = "1.0"` inside an empty `Scope` named `mu`. Nothing
reads it, no `PhysicsMaterial` is created, and every contact on this robot uses
PhysX defaults.

Behaviour with PhysX defaults: **dyaw 1.551 rad**, which matches
`damping_sweep.json` at the same gain to three decimals — two independent
scripts agreeing is worth more than either alone.

This is the cleanest example in this project of a value transferring perfectly
while its meaning does not, and it is NOT the residual: binding the real
frictions makes the spin *worse* (0.926 → 0.813 rad, recorded above).


---

## Slip, measured in Isaac and NOT measurable over ROS (a later check)

```bash
python3 scripts/slip_probe.py --selftest
~/.venv-isaacsim/bin/python scripts/slip_probe.py     # -> data/slip_probe.json
scripts/slip_probe_gz.py                              # -> REJECTS its own result
```

Slip is defined as `1 - ground_travel / rim_travel`, where for a pure spin
`rim_travel = swept_wheel_angle * R` and `ground_travel = dyaw * TRACK/2`.

**Isaac: slip = 0.1132.** Wheel swept 3.4981 rad against a theoretical 3.501 —
agreement to three decimals, so the instrument is sound.

**Gazebo: NOT MEASURED.** The ROS-topic instrument is not good enough. Five runs
of the identical command:

| run | swept (rad) | slip |
|---|---|---|
| 1 (wall clock) | 9.237 | 0.937 |
| 2 | 5.384 | 0.381 |
| 3 | 5.072 | 0.372 |
| 4 | 6.941 | 0.733 |
| 5 | 8.146 | rejected |

Theory says 3.501. **A 0.57 spread in slip cannot resolve an 0.11 measurement**,
so `slip_probe_gz.py` now checks its swept angle against theory and returns
`slip_fraction: null` with `verdict: REJECTED` rather than a number.

### Two real bugs found on the way, and one that remains

1. **Wall-clock integration.** Integrating `/joint_states` velocity against
   `time.time()` counts real seconds while the sim counts its own; at this
   machine's RTF that over-counted by 2.6x. Fixed: integrate on the message
   stamp.
2. **The command window did not close with the command.** Accumulation
   continued while the stop propagated and the robot coasted, stretching 2.00 s
   into 2.90 s. Fixed: freeze the accumulator before publishing the stop.
3. **Still unresolved:** even after both fixes the sweep varies run to run.
   `/joint_states` sampling, sim-time stamps and window edges interact, and
   none is under the script's control the way PhysX is on the Isaac side.
   Closing this needs the Gazebo-side value read from the engine rather than
   from a topic.

**So the 5.1 prediction ("the residual is slip") is HALF-TESTED.** Isaac slips
11.3 %. Whether Gazebo slips more, less or the same is unknown, and 11.3 % on
its own does not establish anything about the 16.4 % residual.


## Is it the CONTACT SOLVER? NO (a later check)

```bash
~/.venv-isaacsim/bin/python scripts/damping_sweep.py --solver=PGS
# -> data/damping_sweep_pgs.json
```

a later check tested articulation *iteration counts* and found nothing.
Swapping the contact solver outright is a much stronger lever: TGS and PGS are
different algorithms, not different budgets.

| solver | dyaw at gain 1.0 | vs Gazebo 1.857 |
|---|---|---|
| TGS (default) | 1.551 | 16.5 % |
| PGS | 1.545 | 16.8 % |

**The solver choice moves the yaw by 0.006 rad — 0.3 % of the target, and 2 %
of the residual.** PGS is very slightly *worse*. Sixth elimination.

### Running tally of eliminated hypotheses (all measured)

1. the drive gain — survives 0.01 → 100
2. the chassis — base height p2p 0.0003 m
3. articulation solver iterations — 4 → 255 changes nothing, 255 is worse
4. Isaac-exclusivity — Gazebo ripples too, ~7x less
5. the timestep — Isaac at Gazebo's own 1 ms is *worse* (17.7 % vs 16.5 %)
6. **the contact solver type — TGS vs PGS moves 0.3 %**

Still open: whether Gazebo's *slip* differs from Isaac's measured 11.3 %, which
needs an in-process Gazebo probe (see a later check).


## Does the caster chatter? NO (a later check)

```bash
python3 scripts/caster_probe.py --selftest
~/.venv-isaacsim/bin/python scripts/caster_probe.py   # -> data/caster_probe.json
```

The curriculum row for 5.5 promised "the caster that chattered". Measured while
driving straight at 0.4 m/s, steady window only, same `analyse()` the vibration
probe uses:

| body | mean height | peak-to-peak | relative | verdict |
|---|---|---|---|---|
| base_link | 75.01 mm | 0.280 mm | 0.37 % | steady |
| caster | 49.43 mm | **0.320 mm** | **0.66 %** | **steady** |

**The caster does not chatter.** 0.32 mm of vertical movement, which is
0.66 % of its ride height and far below the 2 % amplitude the detector requires.
For contrast, the 4.7 wheel-velocity ripple is 0.804-1.145 relative — the
caster is roughly **122x quieter**.

Note the reversal rate is high (122 Hz) but the amplitude is negligible; that
combination is numerical noise on a resting contact, not chatter, and is exactly
why the detector requires BOTH a rate and an amplitude.

**Restitution: neither side declares any.** `grep -c restitution` returns 0 in
both `rover.urdf.xacro` and `flat.sdf`. Whatever each engine defaults to is what
the robot has, and nobody chose it. The Isaac ground plane in our scripts sets
`restitution=0.0` explicitly; the Gazebo side does not, so this is one more
place the two rigs are not symmetric.

**Second curriculum row corrected by measurement** (the first was 4.7's
"vibration only in Isaac"). Rows are predictions.


---

## The ground sweep, and the DEATH of the slip hypothesis (a later check)

```bash
python3 scripts/ground_probe.py --selftest
~/.venv-isaacsim/bin/python scripts/ground_probe.py   # -> data/ground_probe.json
```

a later check established that the URDF's per-link frictions are inert, so the
ground material is the only place friction is actually decided. Swept it, at
drive damping 1.0:

| ground mu | dyaw (rad) | vs Gazebo 1.857 |
|---|---|---|
| 0.05 (near ice) | **1.5545** | 16.3 % |
| 0.2 | 1.5553 | 16.2 % |
| 0.5 | 1.5504 | 16.5 % |
| 1.0 | 1.5510 | 16.5 % |
| 2.0 | 1.5228 | 18.0 % |

**A 40x change in friction moves the yaw by 0.0325 rad — 10.6 % of the
residual — and not monotonically.** At mu = 0.05, which is close to ice, the
robot still turns 1.5545 rad.

### This kills the a later check prediction

5.1 predicted the residual was **slip**: the wheels turn the right amount, the
robot turns less, and the difference slides. If that were true, the turn would
be friction-limited, and dropping mu to 0.05 would wreck it. It does not. The
turn is **not friction-limited at all**, so slip is not the mechanism.

That also explains 5.3's 11.3 % measured slip: it is real, it just is not
what is costing the yaw.

### And it kills 5.2's ratio explanation

5.2 argued that binding the URDF frictions made the spin worse because it
changed the wheel-to-caster *ratio*. Tested directly with per-shape overrides:

| wheel mu | caster mu | ratio | dyaw |
|---|---|---|---|
| 1.0 | 1.0 | 1 | 1.5424 |
| 1.0 | 0.05 | 20 | 1.5431 |
| 0.5 | 0.05 | 10 | 1.5468 |
| 1.0 | 0.5 | 2 | 1.5418 |

**A 20x change in ratio moves the yaw by 0.005 rad — 1.6 % of the residual.**
The ratio explanation was a plausible story for someone else's measurement and
it does not survive its own test.

**Seventh and eighth eliminations.** Remaining candidates for the 16.4 %: none
that I have named. The next honest step is to stop guessing mechanisms and
measure what the two engines actually differ on during the turn.


---

## FLAW-4 does not reproduce: the sensors hit their declared rates (a later check)

```bash
python3 scripts/sensor_probe_gz.py --selftest
scripts/sensor_probe_gz.py            # needs a running gz sim
# -> data/sensor_probe_gz.json
```

The xacro carries a planted comment calling the sensor/physics rate mismatch
"FLAW-4" and asserting it reproduces a measured 0.33x bug from an earlier measurement.
**Measured over a 4 s sim-time window:**

| sensor | declared | achieved | fraction | verdict |
|---|---|---|---|---|
| /scan (gpu_lidar) | 10 Hz | **10.00 Hz** | 1.000x | ok |
| /depth (depth_camera) | 30 Hz | **29.79 Hz** | 0.993x | ok |

**Both sensors publish at their declared rate.** There is no 0.33x bug on this
robot in Gazebo Harmonic 9.5.0. The rate/timestep mismatch is real — the physics
runs at 1000 Hz, the lidar at 10 and the camera at 30 — but Gazebo schedules
sensors independently of the physics step and hits both targets.

Other declared-vs-actual checks, all matching:

| field | declared | measured |
|---|---|---|
| lidar samples | 640 | **640** |
| angle range | ±2.356 rad | **-2.356 to 2.356** |
| observed returns | 0.12-12.0 m | 2.346-11.609 m (in-range) |

**262 of 640 returns are `inf`.** That is the lidar's 135-degree fan seeing past
the edge of nothing on an empty ground plane, not a fault — worth stating because
a naive "count the NaNs" check would flag a healthy sensor.

**Third planted/predicted flaw contradicted by measurement**, after 4.7's
"vibration only in Isaac" and 5.5's "caster that chattered". The pattern is now
strong enough to state as a rule: **anything in this repo that was written before
it was measured is a prediction**, including my own comments and curriculum rows.


## The sensors do not survive the import AT ALL (a later check)

Measured by scanning every `.usda` in `sim-workspace/isaac/rover.usd`:

| what | count |
|---|---|
| `<sensor>` elements in the URDF | 2 (gpu_lidar, depth_camera) |
| mounting links in the stage (`lidar_link`, `camera_link`) | **2, present** |
| fixed joints holding them (`lidar_joint`, `camera_joint`) | **2, present** |
| sensor prims of ANY type in the stage | **0** |
| prim types present in total | 10, none sensor-like |

**The geometry and the mounting transfer perfectly. The sensors do not exist.**
Not "arrive misconfigured" and not "arrive as inert annotations" like the
friction in 5.2 — there is no sensor prim of any kind.

That is structurally different from every other gap in this project so far, and
it is the reason this repo is not an A/B comparison notes: you cannot diff a
lidar against a lidar that was never created. Isaac sensors are RTX sensors
created through its own extensions (`isaacsim.sensors.*`), against a URDF
`<sensor>` element the importer has no mapping for.

**Practical consequence for a migration:** the sensor stack is not ported, it is
rebuilt, and the mount frames the importer DID bring across are the useful part
— they are exactly where the new sensors go.


## The mount frames DO transfer, exactly (a later check)

The sensors do not arrive (above), but the frames they hang from do. Verified by
comparing the URDF joint origins against `payloads/base.usda`:

| link | URDF origin | USD `xformOp:translate` | |
|---|---|---|---|
| lidar_link | `0 0 0.11` | `(0, 0, 0.11)` | **exact** |
| camera_link | `0.03 0 0.04` | `(0.03, 0, 0.04)` | **exact** |

Both parented to `mast`, both with identity rotation, both as fixed joints in
`physics.usda`.

**So the extrinsics survive and the sensor configuration does not.** That split
is the useful shape of the sensor migration: the fiddly part (where exactly is
the sensor, at what angle) transfers for free, and the part you were going to
retune anyway (sample count, rate, noise model) has to be rebuilt.

Practically: create the RTX sensor, parent it to the imported `lidar_link` or
`camera_link`, and the mounting is already correct.

## 6.3 — camera intrinsics and distortion (measured 2026-08-22)

**What Gazebo declares** (`rover.urdf.xacro`, `depth_camera` sensor):

    horizontal_fov  1.0472 rad   (60.0000 deg)
    image           640 x 480, R_FLOAT32
    clip            near 0.10, far 10.0
    distortion      NOT DECLARED

**What the USD stage contains.** Zero camera prims. `grep -E "def Camera|
UsdGeomCamera"` over the whole stage returns nothing; the prim-type census is
22 Scope, 13 Xform, 7 Cylinder, 6 Material, 4 PhysicsFixedJoint, 3 Shader,
3 Cube, 2 Sphere. `camera_link` is a plain `Xform`:

    def Xform "camera_link"
        quatf  xformOp:orient    = (1, 0, 0, 0)
        double3 xformOp:translate = (0.03, 0, 0.04)

So there is no `focalLength`, no `horizontalAperture`, no `clippingRange`, and
no distortion coefficients anywhere in the stage. Same finding as 6.1: not
misconfigured, absent.

**The conversion you must do by hand.** Gazebo states the lens as a field of
view. USD states it as a focal length against an aperture, both in millimetres,
because `UsdGeomCamera` models a physical camera. Same lens, different
parameterisation:

    fx = fy = W / (2 tan(hfov/2)) = 640 / (2 tan(0.5236)) = 554.26 px
    cx, cy = 320.0, 240.0

    horizontalAperture = 20.955 mm  (the USD default, keep it)
    focalLength        = 20.955 / (2 tan(hfov/2)) = 18.1475 mm
    verticalAperture   = 20.955 * 480/640 = 15.7162 mm   (square pixels)

    round trip: 2 atan(20.955 / (2 * 18.1475)) = 60.0001 deg   OK

The vertical aperture is the trap. USD does not derive it from the image
aspect: set only `horizontalAperture` and the vertical aperture keeps its
default 15.2908 mm, which is not 480/640 of 20.955 (that is 15.7162). Measured
consequence, at focalLength 18.1475 mm:

    vertical FOV, default aperture   45.6906 deg
    vertical FOV, correct aperture   46.8266 deg
    error                            +1.1360 deg   (+2.49%)

The horizontal FOV is still exactly 60 deg, so a horizontal check passes. The
pixels are non-square by 2.78% and every vertical measurement is short: at 10 m
the camera sees 4.2129 m of vertical extent where it should see 4.3301 m, an
11.7 cm error at the top and bottom of the frame.

**Distortion.** Neither side has any. Gazebo's `<distortion>` block is absent,
so it renders a pinhole; USD has no distortion at all and Isaac expresses it,
when you want it, as a separate F-theta or polynomial lens on the render
product. So this migration carries no distortion to lose — but a robot whose
Gazebo camera DID declare k1..k3 has nowhere for them to land, and that is a
real, silent loss of fidelity rather than a conversion.

## 6.4 — planar vs radial depth (computed 2026-08-22, `scripts/depth_probe_gz.py`)

A "depth" image can hold **Z**, the perpendicular distance to the image plane,
or the **euclidean range** along the ray. Both are called depth. They agree only
at the principal point and diverge as `1/cos(theta)` off-axis. On our camera
(60 deg hfov, 640x480, fx = 554.2547 px):

| pixel | off-axis | range/Z |
|---|---|---|
| principal point | 0.000 deg | 1.0000 |
| vertical edge | 23.413 deg | 1.0897 |
| horizontal edge | 30.000 deg | 1.1547 |
| **corner** | **35.818 deg** | **1.2332** |

The horizontal-edge angle is exactly `hfov/2`, which is the check that the
geometry is right.

**I got this wrong first and the selftest caught it.** I wrote "a corner pixel
differs by ~15%" — that is the *horizontal edge* figure. The corner is the
diagonal, so its off-axis angle is `atan(hypot(w/2,h/2)/fx) = 35.818 deg`, not
`hfov/2`, and the divergence is **23.32%**, not 15%. The selftest asserted a
band around the value I had computed rather than around the value I expected, so
it failed immediately instead of enshrining the guess.

That is the fourth prediction of mine this project has contradicted, after 4.7's
Isaac-only vibration, 5.5's chattering caster and 6.7's rate bug — and the first
one caught by a gate rather than by a simulator.

**Not yet measured:** which convention each engine actually uses. That needs the
camera pointed at a known flat wall, because the two are distinguishable only by
whether wall pixels read equal (Z) or grow toward the edges by exactly the ratios
above (range). 6.4 must not claim a divergence until both sides are measured —
that is the 4.2 mistake.

## 6.5 — noise models (measured 2026-08-22, parsed from the xacro)

**Per-sensor noise declarations in our URDF:**

| sensor | type | `<noise>` blocks | `<resolution>` |
|---|---|---|---|
| `lidar` | gpu_lidar | **1** — gaussian, mean 0.0, stddev 0.01 | 0.01 |
| `depth_camera` | depth_camera | **0** | none |

Two things worth saying out loud.

**The asymmetry is in our own robot.** The lidar gets noise; the depth camera
gets a perfectly clean measurement. That is not a Gazebo limitation — Gazebo's
camera sensor supports a `<noise>` block — it is a modelling choice that was
made (by me, in this file) and never revisited. Any comparison of "how noisy is
Gazebo" that averages over our two sensors is averaging over one sensor with
noise and one with none.

**The stddev exactly equals the range resolution.** Both are 0.01 m, so 1 sigma
of Gaussian noise is precisely one quantisation step, and 95% of returns land
within ±1.96 cm, i.e. ±1.96 steps. Those two mechanisms are therefore
indistinguishable in a single measurement: you cannot tell a quantised clean
return from a noisy unquantised one when the step and the sigma are the same
size. Separating them needs many samples of a *static* scene — quantisation is
deterministic per geometry, noise is not.

**What Isaac does instead, stated as structure not as a measurement.** There is
no `<noise>` equivalent to import, and 6.1 already established there are no
sensor prims at all, so nothing about noise transferred. An RTX lidar's
imperfection comes from the render and the material model rather than from an
additive term you configure. That is a different *shape* of error: additive
Gaussian is independent per beam by construction, whereas a render-derived
error is correlated across neighbouring beams that hit the same surface at the
same angle.

**Not measured:** the actual per-beam distribution in either engine. Doing it
properly needs a static scene and many frames, and 6.5 must state the structural
difference without claiming a measured magnitude — the 4.2 rule again.

## 6.6 — there is NO IMU on this robot (measured 2026-08-22)

The curriculum row for 6.6 reads "IMU, odometry and drift compared". **This
robot has no IMU.** Measured, not assumed:

    sensors declared in rover.urdf.xacro:
      <sensor name="lidar"        type="gpu_lidar">
      <sensor name="depth_camera" type="depth_camera">

    grep -E '\bimu\b|type="imu"|<imu>'  ->  no matches anywhere

    ros_gz_bridge topics:  /clock, /depth, /scan   (no /imu)

My first grep for "imu" returned 1 hit, which was the substring inside
**"simulators"** in a comment. A whole-word search returns nothing. That is the
second time this session a substring match nearly became a finding.

**So where does the rotation actually come from?** `diff_drive_controller`, from
wheel joint positions, with `enable_odom_tf: true` and `odom_frame_id: odom` in
`controllers.yaml`. It is wheel odometry — a kinematic integration of two joint
encoders — and it has no inertial input at all.

**And a RIG ASYMMETRY I had not noticed, which is worse than the missing IMU.**
I went to write "the residual and the odometry are the same number reported
twice" and checked the probes first. They do not measure the same thing:

| side | script | signal |
|---|---|---|
| Gazebo | `slip_probe_gz.py` | `/diff_drive_base_controller/odom` — **wheel-derived** |
| Isaac | `damping_sweep.py` | `rover.get_world_poses()` — **ground truth** |

**Then I checked what the residual is actually made of, and it is fine.** The
16.4% comes from `damping_sweep.py`, which reads BOTH numbers from Isaac:
reached wheel speed 2.017 rad/s and achieved yaw 1.551 rad, the latter from
`get_world_poses()`. Same engine, same run, both ground truth. So the residual
is an internally consistent Isaac measurement and the missing IMU does not touch
it. I nearly wrote that it did.

What IS true is narrower: **no Gazebo probe in this repo reads ground-truth
pose.** `baseline_gz.py` and `slip_probe_gz.py` both subscribe to
`/diff_drive_base_controller/odom`, which is wheel-derived and therefore blind
to slip by construction. So every Gazebo number in this project is odometry, and
every Isaac number is truth. That is the fourth rig asymmetry, after the
timestep, the instrumentation depth and the restitution — and unlike those, this
one has never mattered yet, because the residual never crossed engines.

**Two wrong sentences caught by checking, in one paragraph.** First "the
residual and the odometry are the same number reported twice" (they are
different signals); then "the residual compares odom against truth" (it does
not — both halves are Isaac). Neither survived reading the two scripts. The
pattern: I reasoned about instrumentation from what the rig *ought* to look
like instead of opening the probes.

So the honest question for 6.6 is: what would an independent witness cost, and
what would it settle? There is no inertial signal in this rig at all, so the
rotation has exactly one ground-truth witness (Isaac's pose) and one
wheel-derived witness (Gazebo's odom), and no engine has both.

Recorded rather than hidden: the curriculum row was written before the robot
was, and it is a **prediction**, like FLAW-2, FLAW-4, the 4.7 vibration and the
5.5 caster. Fifth one.

## 6.8 — the correlated-error arithmetic (computed 2026-08-22)

Our Gazebo lidar: **640 beams, gaussian stddev 0.01 m, drawn independently per
beam** (measured, 6.5). A scan matcher averaging independent per-beam errors
suppresses them by `sqrt(N)`:

    sqrt(640)                = 25.30
    independent pose error   = 0.01 / 25.30 = 0.395 mm

Against an illustrative sensor with **half** the per-beam error (0.005 m) but
**correlated** across a surface, averaging buys nothing:

    correlated pose error    = 5.0 mm
    ratio                    = 12.6x WORSE

So a sensor twice as accurate per beam can be **12.6x worse** in pose. I first
wrote "more than ten times worse", which is true but vague; the exact figure is
12.6x and the write-up now says that.

**Status of each half.** The 0.01 m stddev, the independent draw and the 640
beams are measured facts about our Gazebo lidar. The 0.005 m correlated
alternative is **illustrative** — there is no RTX lidar in our stage to measure,
so this is a mechanism demonstration, not a measurement of Isaac. 6.8 says so
explicitly.

## 7.1 — the interface, DERIVED not typed (2026-08-22, `scripts/interface_audit.py`)

Write-up 6.9 concluded that a tolerance cannot catch an absence and an existence
check has to come first. this repo's condition is already binary, so this is
that check written as code.

**The expected set is derived from the launch file and controller config**, not
hand-typed — a hand-typed roster here would be a prediction, and this project has
contradicted five of those. It parses the `ros_gz_bridge` entries out of
`gz.launch.py` and the controller names out of `controllers.yaml`:

| topic | type | source |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | bridge entry |
| `/scan` | `sensor_msgs/msg/LaserScan` | bridge entry |
| `/depth` | `sensor_msgs/msg/Image` | bridge entry |
| `/diff_drive_base_controller/odom` | `nav_msgs/msg/Odometry` | controller |
| `/diff_drive_base_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | controller |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster |
| `/tf` | `tf2_msgs/msg/TFMessage` | robot_state_publisher |

Seven topics. **Three bridged, four from ros2_control.** The selftest asserts it
finds all of those AND that it does **not** invent an `/imu` — which it would
have, if I had typed the list from the curriculum instead of parsing the robot.

**Four checks, deliberately separate**, because each fails differently:
1. **exists** — in the graph at all
2. **right type** — a topic with the wrong type is worse than a missing one: the
   subscriber never matches and nothing logs an error
3. **actually publishing** — existing is not publishing; a dead topic stays in
   `ros2 topic list` forever. Counted on the sim clock.
4. **the TF tree, edge by edge** — a missing edge is the failure that shows up
   three layers away as a perception bug

Selftest passes both directions: it distinguishes a MISSING topic from a
WRONG-TYPE topic as separate failures, and reports neither on a clean graph.

### 7.1 — the audit RUN, and three bugs in my own meter (2026-08-22)

Live against a headless Gazebo stack, `ROS_DOMAIN_ID=77 GZ_PARTITION=c5iface`:

```
expected      7          live in graph 22
present, right type   7
MISSING       none       WRONG TYPE    none
SILENT        /diff_drive_base_controller/cmd_vel

  /clock                                 706.89 Hz
  /depth                                  25.97 Hz
  /diff_drive_base_controller/odom        48.17 Hz
  /joint_states                           85.11 Hz
  /scan                                    9.74 Hz
  /diff_drive_base_controller/cmd_vel     SILENT

tf edges  9
  odom -> base_footprint          base_footprint -> base_link
  base_link -> left_wheel         base_link -> right_wheel
  base_link -> caster             base_link -> mast
  mast -> lidar_link              mast -> camera_link
  camera_link -> camera_optical_frame
```

**All 7 expected topics exist with the right type. The TF tree is complete: 9
edges, every URDF link.** `/cmd_vel` being silent is correct — it is an *input*
with nothing commanding the robot.

**Three bugs, all in my measurement, none in the robot.** Every one of them
would have shipped as a finding:

1. **`ros2 topic hz` never exits.** Calling it under a subprocess timeout raised
   `TimeoutExpired` on a perfectly healthy `/clock`, which reads as a dead
   topic. Replaced with a direct subscription that counts.
2. **One `/tf` message is not the tree.** `echo --once` returned a single edge
   (`odom -> base_footprint`) and the wheels looked **missing** — while
   `tf2_ros tf2_monitor` listed `left_wheel` and `right_wheel` among 9 frames.
   `/tf` is dynamic: each message carries only what changed that tick. And
   `/tf_static` is **TRANSIENT_LOCAL**, so a default subscription gets nothing
   from it and every fixed joint reads as absent. Fixed by accumulating over a
   window with the right QoS on each.
3. **`spin_once(timeout_sec=0.1)` starved the slow topics.** `/scan` measured
   **2.96 Hz** while a dedicated probe read **10.03 Hz per sim second**. One
   spin serviced one callback and then slept, so the topic sharing a graph with
   ~1000 Hz `/clock` traffic lost. The rate was the meter, not the sensor.

**And a 12-hour orphan, found by the audit.** `/cmd_vel` first measured
19.56 Hz with the stack idle. `ros2 node list` showed a `/capture_drive` node;
`pgrep -f capture_drive` found nothing (it matched only its own shell, exit 144
— that trap again). Scanning `/proc` for python3 cmdlines found
**`/tmp/g2i_drive_test.py`, running for 12 h 14 m**, declaring
`Node("capture_drive")` and publishing `linear.x = 0.4`. It had been driving the
robot since the previous session. Terminated; `/cmd_vel` then correctly read
SILENT.

That is worth keeping: **an interface audit found a stale publisher that no
process search did**, because it asked what the graph contains rather than what
the process table contains.

**Rates here are per WALL second.** The declared rates are per *simulated*
second, so a comparison needs the RTF. Measured RTF 0.977: 98 scans in 10.00 s
wall = 9.80 Hz wall = **10.03 Hz per sim second** against 10 declared. The
docstring now says so, because 5.3 already cost a day to this exact confusion.

## 7.2 — the TF tree, both sides (measured 2026-08-22, `scripts/tf_from_usd.py`)

**Gazebo publishes 9 `/tf` edges**, 8 robot-internal plus `odom -> base_footprint`
which is produced by the controller at runtime, not by the description.

**The USD stage contains all 8 robot edges**, as Xform nesting:

| edge | in Gazebo /tf | in USD hierarchy | in USD joint prims |
|---|---|---|---|
| `base_footprint -> base_link` | yes | yes | **no** |
| `base_link -> left_wheel` | yes | yes | yes |
| `base_link -> right_wheel` | yes | yes | yes |
| `base_link -> caster` | yes | yes | yes |
| `base_link -> mast` | yes | yes | yes |
| `mast -> lidar_link` | yes | yes | yes |
| `mast -> camera_link` | yes | yes | yes |
| `camera_link -> camera_optical_frame` | yes | yes | **no** |
| `lidar_link -> lidar_housing` | **no** | yes | no |
| `odom -> base_footprint` | yes | **no** | no |

**Nothing is missing from the stage.** `only_in_gazebo` is empty.

Three asymmetries, each with a reason:

- **6 joint prims, not 8.** Exactly the two joints write-up 4.1 measured as
  having no prim — `base_joint` and `camera_optical_joint`, both fixed joints to
  a massless frame, baked into the hierarchy instead. So the joint-prim count is
  expected to be short and the **hierarchy** is authoritative.
- **`lidar_link -> lidar_housing` is in the stage and not in /tf.** It is a
  visual mesh with its own transform. `robot_state_publisher` publishes one
  frame per *link*, and a visual is not a link. Correct on both sides.
- **`odom -> base_footprint` is in /tf and not in the stage.** Odometry is
  produced by the controller at runtime. A description cannot contain it.

**A bug in my own parser, caught by grepping the `def` type.** Counting `Scope`
prims as frames gave **17** edges against Gazebo's 8. The 9 extras were not
frames: 5 were the imported SDF friction tags (`surface -> friction -> ode ->
mu/mu2`, all `Scope`), `Geometry` is a Scope container, and the rest visual
meshes. **A Scope carries no transform, so it can never be a TF frame.** Fixed
to Xform-only; the selftest now asserts Scope exclusion in **both** directions,
because the original fixtures used only Xforms and so could not have caught it.

**What this does NOT establish.** Isaac has no `robot_state_publisher`. The
stage *containing* the tree is not the same as Isaac *publishing* it: that needs
an ActionGraph you build. So 7.2 signs "the tree survives the import" and does
not sign "the tree appears on /tf in Isaac".

## 7.3 — QoS, measured on a live stack (2026-08-22, `scripts/qos_probe.py`)

**Every publisher's offered QoS, measured, not assumed:**

| topic | reliability | durability | publisher |
|---|---|---|---|
| `/clock` | RELIABLE | VOLATILE | ros_gz_bridge |
| `/scan` | **RELIABLE** | VOLATILE | ros_gz_bridge |
| `/depth` | **RELIABLE** | VOLATILE | ros_gz_bridge |
| `/tf` | RELIABLE | VOLATILE | robot_state_publisher |
| `/tf_static` | RELIABLE | **TRANSIENT_LOCAL** | robot_state_publisher |
| `/joint_states` | RELIABLE | **TRANSIENT_LOCAL** | joint_state_broadcaster |
| `.../odom` | RELIABLE | **TRANSIENT_LOCAL** | diff_drive_controller |
| `.../cmd_vel` | — | — | **no publisher** (an input) |

### The surprise: the sensor topics are RELIABLE, not BEST_EFFORT

The conventional advice is that sensor topics use `sensor_data` QoS
(BEST_EFFORT), and that a node written with default (RELIABLE) QoS therefore
gets nothing from them. **On this stack that is backwards.** `ros_gz_bridge`
publishes `/scan` and `/depth` as **RELIABLE/VOLATILE**, so:

- a **default** subscriber works on every topic here
- a **`sensor_data`** subscriber also works (BEST_EFFORT requesting from a
  RELIABLE publisher is compatible — the rule is one-directional)
- a **latched** (TRANSIENT_LOCAL) subscriber **FAILS** on `/scan`, `/depth`,
  `/clock` and `/tf`, because TRANSIENT_LOCAL requesting from VOLATILE is
  incompatible

So the failure direction on this stack is the opposite of the one people warn
about. **Measure the offered QoS; do not apply the convention.**

### The failure that actually bit me, and it is not incompatibility

In 7.1 I subscribed to `/tf_static` with default QoS and received **nothing**,
which made every fixed joint in the robot read as absent. But that pairing is
*compatible*: RELIABLE/VOLATILE requesting from RELIABLE/TRANSIENT_LOCAL matches
fine.

The problem is the **latched history**. `/tf_static` is published once at
startup. A VOLATILE subscriber connects successfully and then waits forever,
because everything it wanted was sent before it arrived. That is worse than an
incompatibility: the connection succeeds, `ros2 topic info` shows a matched
publisher and subscriber, and no message ever comes.

The probe reports those as two distinct states — `FAIL` (never matches) and
`HIST` (matches, misses the latched history) — because they need different
fixes. Three topics here are `HIST` traps for a default subscriber:
`/tf_static`, `/joint_states` and `.../odom`.

The selftest asserts both incompatible directions, both compatible reverses, and
the matches-but-no-history case specifically.

### An unrelated failure worth recording: exit code -15

The first run of this probe brought the stack up, activated **both** controllers
successfully, and then `gz sim` died with **exit code -15** — SIGTERM. Nothing
sends itself SIGTERM, so something external did. `produce.py`'s inter-scene
cleanup is `pkill -f "/tmp/chrome_prod_..."`, which cannot match `gz sim`, and
no teardown loop of mine was running. Re-running with a 7.1 render in flight
worked fine, so it is not render contention either. Cause unidentified;
recorded rather than guessed at. **Same lesson as before: a negative exit code
means somebody killed it, and that somebody is never the process itself.**

## 7.4 — sim time vs wall time (measured 2026-08-22, `scripts/time_probe.py`)

**`use_sim_time`, queried from each running node** (not read from the launch
file, because a param file can set it too):

| node | use_sim_time |
|---|---|
| `/controller_manager` | True |
| `/diff_drive_base_controller` | True |
| `/joint_state_broadcaster` | True |
| `/robot_state_publisher` | True |
| `/ros_gz_bridge` | True |
| **`/gz_ros_control`** | **False** |

**Measured RTF 0.9952**, sim clock at 63.074 s against a wall clock of
1.787e9 — the two are never confusable at those magnitudes, which is why
classifying a header stamp by which clock it is near works at all.

**Every published stamp is on the sim clock:**

| topic | clock | last stamp | msgs in 5 s |
|---|---|---|---|
| `/joint_states` | **sim** | 63.079 | 405 |
| `/scan` | **sim** | 63.000 | 50 |
| `.../odom` | **sim** | 63.070 | 203 |

### The one node with use_sim_time False, and why it is NOT a defect

Five of six nodes read `/clock`; `/gz_ros_control` reads the system clock. That
looks exactly like the bug this notes is about — one node in a graph on a
different clock, producing timestamps meaningless to its neighbours.

**I checked before writing it up, and it is harmless here.** `ros2 node info
/gz_ros_control` shows it publishes **no data topics at all** — only
`/rosout`, `/parameter_events`, and its own parameter services. It is the
`gz_ros2_control` plugin's node handle, existing to host parameters and
services; the actual `ros2_control` update loop runs inside
`/controller_manager`, which does have `use_sim_time: True`. A node that never
stamps a message cannot stamp one on the wrong clock.

So: **a mixed-clock graph is a real failure mode, and this instance of it is
not one.** The check that separated them was asking what the node publishes,
not what its parameter says. Had I stopped at the parameter table I would have
reported a sixth planted-style defect that does not exist.

**What the failure would look like if it were real.** Two nodes disagreeing on
the clock do not produce a clock error. Node A stamps at 63.07, node B stamps at
1.787e9, and the consumer computes a transform age of ~56 years — which
surfaces as a **TF extrapolation error**, three layers from the cause. Same
shape as 7.3's QoS trap and 6.3's vertical aperture: the symptom lands in a
subsystem with its own plausible knobs.

## 7.5 — the plugin inventory, and FLAW-3 is a PREDICTION (measured 2026-08-22)

**13 `<plugin>` elements across the robot and the world.** Enumerated, not
recalled:

| where | count | what |
|---|---|---|
| `rover.urdf.xacro` | 3 | `gz_ros2_control/GazeboSimSystem` (hardware), `gz_ros2_control-system`, `gz-sim-linearbatteryplugin-system` |
| `flat.sdf` | 10 | physics, sensors, scene-broadcaster, user-commands + 6 GUI plugins (MinimalScene, GzSceneManager, WorldControl, WorldStats, InteractiveViewControl) |

Only a minority need an Isaac equivalent: the engine-internal systems
(`gz-sim-physics-system`, `gz-sim-sensors-system`, `gz-sim-scene-broadcaster`)
are what Isaac *is*, and the 6 GUI plugins are the Isaac viewport. The real
migration work is `gz_ros2_control` and whatever the battery plugin was for.

### FLAW-3 does not do what its comment says

The xacro comment reads:

> "FLAW-3: a Gazebo-only plugin with no Isaac equivalent. The stack subscribes
> to `/rover/battery` and a node downstream gates motion on it. Nothing in Isaac
> publishes this, so after migration the robot appears to work and then stops."

**Measured, four ways, all negative:**

| claim | check | result |
|---|---|---|
| `/rover/battery` is bridged | `grep battery` in `gz.launch.py` | **0 matches** |
| a node subscribes to it | `grep -rl battery` across `sim-workspace/src` | **no files** |
| a node gates motion on it | same | **no such node exists** |
| it is in the expected interface | the 7.1 derived topic set | **absent** (7 topics, no battery) |

So the *plugin* is real and is in the expanded URDF (`grep -c linearbattery` = 1
in both the installed xacro and its xacro-expanded output), but **the ROS-side
consumer the comment describes does not exist.** There is no subscriber, no
gating node, and no bridge entry. Migrating away from it breaks nothing, because
nothing depends on it.

**Sixth contradicted prediction**, after FLAW-2 (the velocity limit), FLAW-4
(the rate bug), 4.7's Isaac-only vibration, 5.5's chattering caster and 6.6's
IMU. Every one of them was written in this repo, by me, before it was measured.

**What I could NOT determine, and am not claiming.** Whether the battery plugin
publishes a Gazebo-transport topic at all. `gz topic -l` on the sim's partition
returned only `/clock`, `/depth`, `/scan` — and those are the *bridged* three,
with no `/model/rover/...` entries of any kind, so my `gz topic` query was very
likely not seeing the simulator's own transport namespace rather than the
simulator having no internal topics. The `-v1` log names no plugin loads at all,
including `gz_ros2_control`, whose controllers demonstrably work — so the log
cannot be used as evidence of loading either. Recorded as unresolved rather than
guessed: **the ROS-side claim is refuted, the transport-side question is open.**

## 7.6 — one stack, two backends (measured 2026-08-22)

Classified every launch action in `gz.launch.py` (95 lines) by whether it names
a Gazebo-specific package:

| action | package | executable | backend |
|---|---|---|---|
| `rsp` | `robot_state_publisher` | robot_state_publisher | **SHARED** |
| `jsb` | `controller_manager` | spawner | **SHARED** |
| `ddc` | `controller_manager` | spawner | **SHARED** |
| `spawn` | `ros_gz_sim` | create | **GAZEBO-ONLY** |
| `bridge` | `ros_gz_bridge` | parameter_bridge | **GAZEBO-ONLY** |
| the sim itself | `ros_gz_sim` (included) | gz_sim.launch.py | **GAZEBO-ONLY** |

**Four of six actions are backend-agnostic**, and only 9 of 95 lines name
`gz`/`ros_gz` at all. So the split is not 50/50 — the majority of the launch
file is ROS-side plumbing that does not care which simulator is underneath.

**What that implies for structure.** The natural refactor is three files, not
two:

- `robot.launch.py` — RSP + both controller spawners. Backend-agnostic, and it
  is the one that carries `use_sim_time` (7.4) and the controller config.
- `gazebo.launch.py` — the sim include, `ros_gz_sim create`, and the
  `parameter_bridge` topic list.
- `isaac.launch.py` — whatever starts Isaac and its ActionGraph, publishing
  `/clock` (7.4) and `/tf` (7.2).

The interesting consequence: the **bridge topic list is the interface
contract**. It is the one place in the Gazebo launch file that enumerates
exactly what crosses from simulator to ROS — the three topics 7.1 measured. In
an Isaac launch file that same list becomes the set of ActionGraph publishers.
So the two backend files are not arbitrary alternatives; they are two
implementations of one enumerated contract, and `scripts/interface_audit.py`
already checks that contract against either.

**Not measured:** I have not written or run `isaac.launch.py`, so the
three-file structure is a recommendation derived from the classification, not a
tested configuration. Stated as such in 7.6.

## 8.5 — real-time factor, honestly (measured 2026-08-22, `scripts/rtf_probe.py`)

**Conditions, stated because RTF is meaningless without them:** 16 CPUs, load
average 5.56 at the start and 4.91 at the end (a slide render was in flight on
the same box), headless Gazebo, `flat.sdf`, 30 s window, 9805 `/clock` messages.

| statistic | value |
|---|---|
| overall RTF over the window | **0.9946** |
| per-second samples | 13 |
| **min** | **0.5766** |
| median | 0.9977 |
| **max** | **1.6388** |
| mean | 1.0119 |
| samples below 0.9 | **7.7 %** |

### The overall number is a lie by omission

**0.9946 reads as "essentially real time".** The per-second samples say something
different: the simulator drops to **0.58x**, then runs at **1.64x** to catch up.
It is not steady-slow, it is *stalling and recovering*, and 1 sample in 13 was
below 0.9.

That distinction is exactly what a mean destroys, and the probe's selftest
asserts it on synthetic fixtures rather than trusting me to remember:

    steady-slow  [0.9]*10        -> mean 0.90, min 0.90, frac<0.9 = 0.0
    stalling     [1.0]*9 + [0.0] -> mean 0.90, min 0.00, frac<0.9 = 0.1

**Identical means. The `min` and the stall fraction are what tell them apart.**
A simulator that runs at 1.0 for nine seconds and stalls for one averages 0.9,
which reads as "slightly slow" rather than "stalled".

### Why this matters beyond a vanity number

A stall-and-catch-up profile is worse than a uniform slowdown for anything with
a control loop in it. A controller tuned at 1.0x sees its effective sample
interval nearly double during the 0.58x window, and a planner that budgets wall
time per iteration gets less compute than it asked for, intermittently. A
uniform 0.99x would be harmless; this is not the same thing.

**Do not quote a single RTF.** Quote min, median, and the fraction below your
tolerance, with the load and the CPU count. A single number cannot distinguish
the two cases above, and they have different consequences.

**Not measured:** Isaac's RTF. There is no Isaac stack in this rig, so this is
the Gazebo baseline under stated load, not a comparison. Same limitation as
this repo's sign-off, and stated the same way.

## 8.6 — determinism: BLOCKED by memory pressure, not measured (2026-08-22)

`scripts/determinism_gz.py` is written and selftested (it distinguishes
identical runs from a 1e-4 difference, and treats 1e-12 as float noise). It has
**not produced a result**, and I am recording why rather than reporting a number.

**The run died with exit code -9 — SIGKILL.** Nothing sends itself SIGKILL. The
cause was memory: 15 GB total, **6 GB available**, with another workload on the
same box running a Gazebo GUI (1.24 GB RSS), a headless Gazebo (0.64 GB), an
`x11grab` capture at 115 % CPU, and an Edge browser. My headless sim was the
thing the kernel chose.

**Retried once with 8 GB free and it was SIGKILLed again** — the other workload
had a headless Gazebo at 40 % CPU and 0.63 GB plus its own capture running
throughout, so the pressure is continuous rather than a transient spike. Three
`-9`/`-15` deaths this session now (7.3's QoS run was the first). **A negative exit code means somebody killed
it, and on a shared box that somebody is usually the kernel or another
that workload's teardown.** It is never the process itself.

**Also fixed a real bug in the probe while getting here.** `pump()` waited only
for `/clock` and then read `state["odom"]`, which was still `None` — the odom
subscription had not delivered yet, and the run crashed with
`TypeError: 'NoneType' object is not subscriptable` on a perfectly healthy
simulator. Now it waits for **both** subscriptions with a 20 s deadline and
raises a legible error instead. *A probe must not read a topic it has not
confirmed arriving* — the same class of mistake as 7.1's three meter bugs.

**Isaac's half of 8.6 is already measured** (`damping_sweep.py`): two
consecutive runs agree to **0.000 on every row** of the damping sweep, and the
16.4 % residual reproduces exactly between runs. So 8.6 can state the Isaac side
as measured and the Gazebo side as **not yet measured**, which is the honest
position and the same one Sections 6 and 7 took.

## 8.1 — there is NO Nav2 in this repo, and 4.2 misnamed the source (2026-08-22)

The curriculum row for 8.1 is "Nav2 against both simulators | the tuned config,
unchanged". **There is no Nav2 configuration in this repository.** Measured:

    config/ contains exactly one file:  controllers.yaml
    grep -rl "controller_server|bt_navigator|FollowPath" src/  ->  no matches
    find for a nav2 package or param file                      ->  no matches

So there is no tuned Nav2 config to run unchanged, and 8.1 cannot be the notes
the row describes.

### And write-up 4.2 named the wrong file — a shipped error

4.2 says, three times, "the **Nav2 configuration** asks for nought point eight
metres per second". The 0.8 is real and the finding built on it is intact, but it
lives in **`controllers.yaml`**, as a `diff_drive_controller` limit:

    linear.x.max_velocity: 0.8
    linear.x.min_velocity: -0.8

That is the ros2_control velocity limit on the base controller, not a Nav2
planner parameter. **The measurement stands** — 0.8 m/s is above what the 8 rad/s
joint limit allows (0.6 m/s), both engines clamp at 8.000 rad/s, and the value
has never been achievable. Only the *attribution* is wrong.

**Why it matters enough to record.** 7.1 measured that 4 of 7 interface topics
come from `ros2_control` rather than the bridge, and this is the same confusion
in the opposite direction: I attributed a `ros2_control` limit to a navigation
stack that is not installed. A learner following 4.2 would go looking in a Nav2
params file and find nothing.

**How 8.1 handles it.** Not by pretending Nav2 is there. 8.1 becomes "the
velocity limit nobody owns": the same measured facts, correctly attributed, plus
the observation that a limit set in a controller config and a limit set by a
joint are enforced by different code at different layers and neither one knows
about the other. That is a better notes than a Nav2 walkthrough I cannot run,
and it is honest about the row being wrong.

## 8.6 — determinism MEASURED, and Gazebo is not bit-reproducible (2026-08-22)

Three runs, identical command profile (0.4 m/s for 4 sim seconds), robot
returned to the origin with `/world/flat/set_pose` between runs, all timing on
the sim clock:

| run | dx (m) | dy (m) | dyaw (rad) |
|---|---|---|---|
| 0 | +1.615074 | -0.051284 | -0.000003 |
| 1 | +1.610828 | -0.050996 | +0.000099 |
| 2 | +1.607562 | -0.050891 | -0.000003 |

| quantity | spread | as % of travel |
|---|---|---|
| dx | **7.51 mm** | 0.466 % |
| dy | 0.393 mm | — |
| dyaw | 0.102 mrad | — |

**Gazebo is NOT bit-deterministic.** Identical commands from an identical start
pose give trajectories that differ by ~7.5 mm over 1.61 m.

**Isaac IS**, on the equivalent test: `damping_sweep.py` gives two consecutive
runs agreeing to **0.000 on every row**, and the 16.4 % residual reproduces
exactly. So this is a genuine, measured divergence between the engines — the
first one in this project where Isaac is the *more* reproducible side.

### Four bugs in my own probe before it produced a number

Every one of them would have shipped a false finding:

1. **`pump()` read `state["odom"]` before confirming it arrived** — crashed with
   `TypeError: 'NoneType' object is not subscriptable` on a healthy simulator.
2. **`gz service` did not inherit `GZ_PARTITION`**, so the world reset reached a
   different transport namespace and reset nothing. The call "succeeded".
3. **Nothing verified the reset.** With bug 2 in place, three runs each started
   where the last ended, every `d*` came out `0.000000`, and the probe reported
   **`identical=True`**. *Three identical zeroes is not determinism, it is a dead
   path.* The probe now refuses to report determinism unless the robot travelled
   at least 25 % of its expected distance, and the selftest asserts that guard
   fires on all-zero motion and does **not** fire on a real 1.03 m run.
4. **`/world/flat/control` with `reset: {all: true}` does not work** — the call
   returns fine and the robot stays put (odom read 60.06 m after a "reset").
   `/world/flat/set_pose` does. Found only because bug 3's guard was in place.

### And a deadlock that is worth its own note

**`diff_drive_controller` publishes `/odom` only after it receives a command —
even a zero one.** Measured: 0 odom messages in 15 s while idle, 1 within 0.1 s
of a zero `cmd_vel`. My probe waited for odom before publishing anything, so it
waited for a topic that would not exist until it spoke. Neither QoS profile
helped, because the topic genuinely was not publishing.

That is a **readiness check weaker than the thing it gates**: "the controller is
active" was true, `ros2 topic list` showed `/odom`, and both facts were
compatible with zero messages forever. `pump()` now sends a zero command while
waiting.

### 8.6 — the Isaac half, and the comparison is now complete (2026-08-22)

`scripts/determinism_isaac.py`, deliberately the **same experiment**: same
0.4 m/s target (converted through the same 0.10 m wheel radius to 4.0 rad/s),
same 4 sim seconds, same statistic, same dead-path refusal.

| engine | dx spread | identical | travel |
|---|---|---|---|
| Gazebo | **7.5130 mm** | **False** | 1.6108 m |
| Isaac | **0.0000 mm** | **True** | 1.1103 m |

**Isaac is bit-deterministic on this test; Gazebo is not.** All three Isaac runs
returned `dx=+1.110322, dy=-0.002110, dyaw=-0.001950` — identical to every digit,
on all three axes. Gazebo's three runs differ by 7.5 mm in dx, 0.39 mm in dy and
0.10 mrad in yaw.

**This is the first measured divergence in this project where Isaac is the better
side.** Every earlier finding was either "they agree" (the velocity limit, the
sensor rates, the mount frames) or "Isaac is missing something" (every sensor).

**Why the travel distances differ, and why that is NOT part of the finding.**
1.61 m vs 1.11 m over the same commanded 4 s is a large gap, and it is
explained by the instrumentation asymmetry already recorded as the 4th rig
asymmetry: Gazebo's number is **wheel odometry**, which cannot see slip, while
Isaac's is **ground truth** from `get_world_poses()`. Gazebo's figure is what
the wheels *claim*; Isaac's is where the body *went*. **The determinism claim
does not depend on that**, because each engine is compared against *itself*
across runs — the spread is within-engine, so the asymmetry cancels.

I am stating that explicitly rather than quietly comparing 1.61 against 1.11,
because this check exists precisely because I once compared a measured
side against a quoted one.

**And a fifth probe bug, caught by the missing output.** `app.close()` tears the
Isaac process down hard enough that code after it does not reliably run: the
first version printed all three runs, called `close()`, and then never wrote the
JSON or the summary. The runs were correct and the result was lost. Everything
that must survive is now computed and written **before** the close.

### 8.5 — the Isaac half of the RTF comparison (2026-08-22, `scripts/rtf_isaac.py`)

Same reduction function as the Gazebo probe (min / median / max / mean /
fraction below 0.9), on purpose.

| engine | overall | min | median | max | <0.9 | load at start |
|---|---|---|---|---|---|---|
| Gazebo | 0.9946 | **0.5766** | 0.9977 | 1.6388 | **7.7 %** | 5.56 |
| Isaac | **1.3321** | **1.2144** | 1.3241 | 1.6040 | **0.0 %** | **16.23** |

**Isaac ran faster than real time and never stalled once.** 29 per-second
samples, minimum 1.2144, zero below 0.9. Gazebo's overall 0.9946 hid a minimum
of 0.5766 and 7.7 % of samples stalling.

**And Isaac did it under three times the load.** Load average was 16.23 rising
to 21.11 during the Isaac run (a slide render plus the Isaac process itself on
16 CPUs) against 5.56 for Gazebo. So the comparison is *unfair to Isaac* and
Isaac still won on every statistic — which is the direction that makes a
conclusion safe rather than suspect.

**Two things this does NOT establish.** The scenes are not identical: Gazebo ran
`flat.sdf` with the bridge, ros2_control and a lidar and depth camera publishing
over ROS; Isaac ran a ground plane, the rover articulation, and no sensors and no
ROS. **Isaac's scene is doing less work**, and part of the 1.33 is that. This is
a measurement of *these two configurations*, not of the engines in the abstract,
and stating the configuration is the whole point of calling the notes
"honestly".

Second, Isaac ran headless with `render=False` on every step. An RTX sensor would
put rendering back in the loop, which 6.7 established is where Isaac's sensor
timing lives. **A 1.33 with no sensors does not predict a 1.33 with sensors.**

**What it does establish, and it is the useful half:** Gazebo's stall-and-catch-up
profile is real and Isaac's absence of stalls is real, measured with the same
statistic. For anything with a control loop, a steady 1.21-and-up is a different
animal from a 0.58-to-1.64 sawtooth, even when the *means* are similar.

## 8.4 — SLAM, and there is no SLAM stack here either (2026-08-22)

Same finding as 8.1: `grep -rln "slam_toolbox|amcl|map_server"` across
`sim-workspace/src` returns **nothing**. There is no SLAM or localisation stack
in this repository, so 8.4 cannot be a walkthrough of one.

**What it can be, from numbers already measured**, is the question a SLAM stack
would actually ask of each engine.

### What scan matching gets from our lidar

| quantity | value | source |
|---|---|---|
| samples | 640 | 6.7, measured |
| rate | 10.00 Hz of 10 declared | 6.7, measured |
| `inf` returns (open sky) | 262 of 640 | 6.7, measured |
| **finite, information-carrying returns** | **378** | derived |
| per-beam noise sigma | 0.01 m, gaussian, independent | 6.5, from the xacro |
| suppression from averaging | sqrt(378) = **19.44x** | derived |
| **implied pose error** | **0.514 mm** | derived |

### And the number that makes this a notes

8.2 measured Gazebo's **reproducibility floor at 7.51 mm** across identical
runs. So:

    lidar-implied pose error   0.514 mm
    odometry reproducibility   7.51   mm
    ratio                      ~15x

**The odometry floor is about fifteen times larger than the lidar's own pose
error.** On this robot, in this simulator, the sensor is not the limiting factor
for localisation — the platform's own reproducibility is. Adding a better lidar
would not move the number; the 7.51 mm sits underneath it.

That inverts the usual instinct, which is to reach for a better sensor.

### Both halves stated honestly

- **Isaac side: unmeasurable here.** 6.1 measured **0 sensor prims** — there is
  no lidar in the stage to scan-match with. So this is a Gazebo-side analysis of
  what a SLAM stack would see, not a comparison.
- **The 0.514 mm is derived, not measured.** It follows from the measured 378
  finite returns and the declared 0.01 m sigma under the independence assumption
  6.5 established Gazebo's noise model actually satisfies. It is arithmetic on
  measurements, and I am labelling it as such rather than as an observed pose
  error.
- **The 262 `inf` returns are correct behaviour**, not a fault (6.7) — but they
  do mean 41 % of the beams carry no information for scan matching, which is
  why the suppression is sqrt(378) and not sqrt(640).

## 8.7 — the regression suite, and a check that passed on nothing (2026-08-22)

`scripts/regression_suite.py`. Write-up 1.7 promised the conditions would live
"in one file in the repository, with the conditions as executable checks rather
than prose". This is that file.

```
[PASS] interface: topics exist and typed           7/7 present with the right type   (7.1)
[PASS] tf tree: stage contains every robot edge    8/8 robot edges present           (7.2)
[PASS] sensor rates within 5 % of declared         2 sensor(s) within tolerance      (6.7)
[PASS] determinism: gazebo within its floor        dx spread 7.5125 mm (tol 15.0250) (8.6)
[PASS] determinism: isaac within its floor         dx spread 0.0000 mm (tol 15.0250) (8.6)
[PASS] rtf: gazebo stall fraction                  7.7% below 0.9, min 0.5766        (8.5)
[PASS] rtf: isaac stall fraction                   0.0% below 0.9, min 1.2144        (8.5)
[SKIP] sensor equivalence: ray within 2 cm at 3 m  0 sensor prims; untestable        (6.1/6.9)

7 pass, 0 fail, 1 skip
```

### Three design decisions, each from a failure earlier in this project

**1. Every threshold is DERIVED, not chosen.** The trajectory tolerance is
`2 x 7.5125 mm`, twice the reproducibility floor 8.6 *measured* on Gazebo. A
suite asserting bit-identical trajectories fails forever on a healthy Gazebo —
8.6 measured exactly why. The selftest asserts the tolerance is above the
measured floor **and** below a 10x regression, so it cannot be widened into
uselessness without failing its own test.

**2. SKIP is a distinct outcome from PASS.** this repo established that a
tolerance cannot catch an absence. A suite that can only pass or fail will
report green on a rig that cannot test half its claims. The sensor-equivalence
condition from 1.7 is **untestable** — 0 sensor prims, no ray to compare — so it
reports SKIP with that reason, and the summary says *"SKIPs are not passes"*.

**3. Every row names the notes that measured it**, so a failure is traceable
to the run that set the expectation rather than to a magic number.

### And the suite's own first bug: a check that passed on zero sensors

The rates check read `v.get("fraction")`. **That field does not exist** — the
real key in `sensor_probe_gz.json` is `fraction_of_declared`. Every lookup
returned `None`, every sensor was skipped by the `is not None` guard, and the
check reported **PASS on zero sensors examined**.

It passed for the same reason a gate passes a missing file: *it examined
nothing, and nothing was out of tolerance.*

Found by feeding it a **known-bad** case — a fabricated `0.33x` rate, the bug
6.7 refuted — and confirming it reported FAIL. It did, which proved the
comparison worked and sent me to look at why the real data never reached it.

Fixed two ways: read the key that exists, **and count what was actually
examined**, reporting SKIP when that count is zero. The suite now says
"2 sensor(s) within tolerance" rather than an unqualified PASS.

**The rule: a check must report how many things it examined.** "PASS" and
"PASS, 0 items" are indistinguishable in a summary line and mean opposite things.

## 8.8 — the overall condition, 20 runs per engine (measured 2026-08-22)

The condition from 1.7: *"Run the same mission on both simulators, twenty times
each, and the completion rate must agree within ten percent, with no failure
mode present in Isaac that is absent in Gazebo."*

`scripts/mission_runs.py`, 20 runs each, same command (0.4 m/s for 4 sim s):

| engine | completed | rate | median dx | failure modes |
|---|---|---|---|---|
| Gazebo | 20/20 | **100 %** | 1.6159 m | none |
| Isaac | 0/20 | **0 %** | 1.1103 m | `short_travel` |

**A 100-point gap against a 10 % tolerance.** And a failure mode present in
Isaac and absent in Gazebo, which 1.7 said "matters more than the first".

### But my threshold was reasoned from an irrelevant constraint

I set completion at **75 % of commanded distance**, justified in the docstring
by 8.1's finding that the 8 rad/s joint limit caps 0.8 m/s at 0.6 m/s — i.e.
75 %. **That justification does not apply here.** This mission commands
0.4 m/s, which needs 4.0 rad/s: *half* the limit. Nothing is clamped. I reasoned
the threshold from a constraint that does not bind at this speed.

**So I tested whether the verdict survives the threshold:**

| threshold | Gazebo | Isaac |
|---|---|---|
| 60 % | PASS | PASS |
| 65 % | PASS | PASS |
| **69 %** | PASS | **PASS** |
| **70 %** | PASS | **FAIL** |
| 75 % | PASS | FAIL |
| 90 % | PASS | FAIL |

**The verdict flips between 69 % and 70 %.** Isaac sits at 69.4 % of commanded,
right against the line. So "0/20" is an artefact of where I drew it, and quoting
100 % vs 0 % as *the* result would be quoting my own arbitrary choice.

### The finding that IS threshold-independent

    Isaac travel / Gazebo travel = 1.1103 / 1.6159 = 0.6871

**Isaac travels 31.3 % less than Gazebo reports, on the same command**, and that
ratio does not depend on any threshold. It is also consistent across runs: Isaac
is bit-identical (8.6) and Gazebo's spread is 7.5 mm.

**And it is not a clean engine comparison**, for the reason recorded four times
already: Gazebo's 1.6159 m is **wheel odometry**, Isaac's 1.1103 m is **ground
truth**. Odometry cannot see slip. So the 31.3 % is an upper bound on a real
divergence, inflated by an instrumentation asymmetry I have not been able to
remove in this rig.

**What I sign:** both engines run the mission repeatably, with no crash and no
instability, 20 runs each. **What I do not sign:** the completion-rate clause,
because the rates are 100 % and 0 % under a threshold whose justification I got
wrong, and 69.4 % vs 100.7 % under any threshold — a real gap, measured with
mismatched instruments.

## 9.3 — multi-robot scaling in Isaac (measured 2026-08-22, `scripts/scaling_isaac.py`)

**Conditions:** 16 CPUs, RTX 4050 Laptop (6 GiB), headless, `render=False`,
400 physics steps per count after a 100-step warm-up, load 3.66 → 7.07.

| robots | ms/step | ms/step/robot | RTF |
|---|---|---|---|
| 1 | 1.5425 | **1.5425** | 3.242 |
| 2 | 2.7440 | 1.3720 | 1.822 |
| 4 | 3.7094 | 0.9273 | 1.348 |
| 8 | 6.1327 | 0.7666 | 0.815 |
| 16 | 11.0925 | **0.6933** | 0.451 |

**16x the robots cost 7.19x the time. Efficiency 2.22 — sublinear.**

Per-robot step cost **falls by 55 %** from 1 to 16 robots (1.5425 → 0.6933 ms).
That is the claim this repo needed: adding robots is genuinely cheaper per robot,
not merely possible.

**The null hypothesis this rules out.** If 16 robots cost 16x one robot,
efficiency would be 1.00 and the GPU would have bought nothing. The selftest
asserts all three regimes are distinguishable — linear (1.0), sublinear (>1) and
**superlinear** (<1, i.e. it got *worse*) — because a scaling probe that cannot
report "worse than linear" cannot be trusted when it reports "better".

**Where it crosses real time.** RTF falls below 1.0 between **n=4 (1.348)** and
**n=8 (0.815)**. So on this machine, 4 rovers still run faster than real time and
8 do not. That is the number to quote, not the efficiency, if you are deciding
how many robots to put in one scene.

**What this does NOT measure.** Gazebo's scaling. There is no equivalent
multi-robot Gazebo scene in this rig, so this is an **Isaac-side curve, not a
comparison** — stated before running, because this repo made that failure mode
visible enough times.

Also: no sensors, no ROS, `render=False`. 8.5 established that RTX sensor
timing is coupled to rendering, so **this curve does not predict the
sensor-equipped case**.

### A probe bug worth recording

`Articulation(prim_paths_expr=...)` reads physics metadata **at construction
time**, and before the first `world.reset()` that metadata is `None`. The
constructor died with `AttributeError: 'NoneType' object has no attribute
'link_names'` on a perfectly valid stage. The reset has to come **before** the
view is constructed, not after.

## 9.1 — what the RTX render path costs (measured 2026-08-22)

`scripts/render_probe_isaac.py`. Every performance number in this project so far
was measured with `render=False` — 8.5's RTF of 1.3321 and 9.3's whole scaling
curve. **This measures the thing I disabled.**

Same scene, same robot, same step count, each mode warmed up separately (the
first render pass allocates):

| mode | ms/step | RTF |
|---|---|---|
| `render=False` | 2.1163 | **2.363** |
| `render=True` | 10.2363 | **0.488** |
| **cost ratio** | | **4.8370x** |

**Rendering costs 4.84x per step**, and it takes a single rover from **2.36x
real time to 0.49x** — i.e. *below* real time with one robot on a ground plane.

### Why this is not optional overhead

6.7 established that **an RTX sensor IS a render operation**. So this is not
cosmetic cost you can decline: it is the price of having a camera at all. A writeup that quotes 1.33x with rendering off and then recommends RTX sensors has
quoted a number from a configuration it is not recommending — which is exactly
what I would have done had I not measured this.

### It re-frames 9.3's scaling curve

9.3 measured real time crossing between **4 and 8 rovers** with rendering off.
At 4.84x, a single rover with rendering on is already at 0.488. **The
sensor-equipped interactive ceiling is therefore below one robot on this
hardware**, not between four and eight.

I am not extrapolating the scaling curve into the rendered case — 9.3 explicitly
declined to, and the two measurements were taken separately. What I can say is
that the *rendering-off* ceiling does not describe a scene with cameras in it.

### The selftest asserts the embarrassing direction

`cost_ratio` must be able to report **below 1.0** — rendering somehow cheaper.
A probe that cannot report the result that would undermine its own headline
cannot be trusted reporting the headline. Same rule as 9.3's superlinear case.

**Conditions:** 16 CPUs, RTX 4050 Laptop 6 GiB, headless, 200 timed steps after
60 warm-up steps per mode, one rover, no sensors, no ROS.

## 8.6 re-measured, 2026-08-23 — and why the number moved

`data/determinism_gz.json` was **overwritten at 01:51 on 2026-08-23** by the
promo spin captures: they drove the same robot on the same domain, so the file
ended up holding a spinning run's x-displacement
(`dx = 1.601, 0.071, -1.526`, "spread" 3126.99 mm). That is not a determinism
result at all, and `scripts/regression_suite.py` caught it as
`[FAIL] determinism: gazebo within its floor  dx spread 3126.9925 mm > 15.0250 mm`.

**Re-ran the probe on a quiet box:**

```
run 0: dx=+1.607216   run 1: dx=+1.615964   run 2: dx=+1.603645
spread 12.3189 mm     identical=False
```

**The finding is unchanged and the number is not.** 8.6 shipped 7.5125 mm; this
re-run gives 12.3189 mm. Both are far above Isaac's 0.0000 mm and both refuse
`identical`, so the notes's claim — *Gazebo is not bit-reproducible, Isaac is*
— stands on either run. What moved is the magnitude of Gazebo's spread, which is
itself load-dependent: this box was at load 4-8 with another workload's Gazebo
alive, where the original was measured quieter.

So the honest statement of 8.6's result is **"Gazebo's spread is of order
10 mm and varies with machine load; Isaac's is exactly zero"**, not
"Gazebo's spread is 7.5125 mm". A single-run figure for a quantity that depends
on scheduling was over-precise, and the re-run is what exposed it.

`GAZEBO_TRAJ_FLOOR_M` stays at the 8.6 value (0.0075125) because the tolerance
is 2x the floor and 12.3189 mm passes it; raising the constant to match the
noisier run would loosen a gate to fit a worse measurement.

**Process lesson, recorded because it nearly shipped:** a capture script and a
measurement script sharing `ROS_DOMAIN_ID` and a data directory can silently
overwrite each other's results. The regression suite is the only reason this was
caught before this project was called finished.
