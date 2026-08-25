# robotsim — a Blender-based robot simulation platform

<img src="/examples/3_arm_demo/Animation/gifmaker_me.gif"/>

## What this is

A simulation platform for **mobile robots with manipulators**, built on Blender.

You describe a robot in Python — a wheeled base, one or more arms loaded from
`.blend` files, a mast of cameras — and the platform gives you a fixed-timestep
loop in which you can drive the base, command the joints, render what each camera
sees, and bake the whole run onto the Blender timeline as an ordinary animation.

The target application is **vision-driven control**. The camera renders from each
robot are intended as the input tensor for a neural network trained in PyTorch;
the trained network is then deployed to an onboard microcontroller (a Jetson Nano
class device) where it drives the wheel motors directly. Blender is the world
model, the renderer, and the ground-truth source for that training loop.

## Why Blender

Blender already ships the hard parts: an IK solver, an armature system with joint
limits, a renderer, a timeline with interpolation, and a Python API over all of
it. Rigging a new arm is a modelling task rather than a URDF-authoring task, and
the result is immediately visualisable. The trade is that Blender is an animation
tool, not a physics engine — so this platform is deliberately *kinematic* today,
with the seams left in the right places to bolt on an external solver later.

## Quick start

```bash
git clone https://github.com/crustos/blender_manipulator_motion_demo
cd blender_manipulator_motion_demo
make install        # chmod the entry points, apt-get install blender
make test_all       # run the full test suite headless
```

Scripts are sh/Python polyglots: they exec Blender on themselves, so they run
directly.

```bash
./robotsim.py                 # interactive, with a UI
./headless.py my_scene.py     # background, runs your script inside Blender
```

Any `.py` passed after `--` is exec'd with the platform's globals available, so a
script can use `Robot`, `RobotSim`, `Arm` and friends without importing anything.

## A minimal simulation

```python
#!../headless.py

bot = Robot(size=(1, 1, 0.1), wheels=4, drive='differential')
rec = RobotSim.record()          # bake this run onto the timeline

bot.drive.drive(1.0, 0.4)        # 1 m/s forward, 0.4 rad/s yaw
arm = bot.arms[0]

@RobotSim
def tick(dt):
    arm.set_tip(location=(0.3, 0.4, 0.5))     # IK: move the tool tip
    views = bot.render_cameras(frame=RobotSim.frame)   # 4 PNGs per robot
    if RobotSim.ticks >= 100:
        RobotSim.stop()

while RobotSim.callbacks:
    RobotSim.update()
```

## Architecture

| Module | Depends on bpy | Purpose |
| --- | --- | --- |
| `robotsim.py` | yes | Entry point and assembly layer. Scene primitives, `.blend` loading, the `Robot` model, `RobotSimpleSim` fixed-timestep loop, camera rendering. |
| `kinematics.py` | yes | `Arm` and `Joint`. Joint-space read/write over an armature, IK muting, joint limits, tool-tip control. |
| `drive.py` | **no** | Base motion. `DifferentialDrive` and `AckermannDrive`, integrated with a real timestep. Pure maths, unit-testable outside Blender. |
| `recorder.py` | yes | Binds sim ticks to scene frames and bakes state into keyframes. |
| `ros2/joint_export.py` | yes | Exports a recorded arm motion as a ROS2-style joint trajectory (JSON or text). |
| `ros2/blender_to_text.py` | yes | The original upstream exporter, kept for the hand-animated workflow. |

## Core concepts

### Frame convention

Derived from how `Robot` is built — the `front` camera and `FRONT.HUB` both face
`+Y`, so:

| Axis | Direction |
| --- | --- |
| `+Y` | forward |
| `+X` | right |
| `+Z` | up |
| `rotation_euler.z` | yaw, measured from `+Y` |

### Reading joints vs writing joints

This asymmetry causes most of the confusion when scripting Blender armatures, so
it is worth stating plainly:

- **Reading works in any mode.** `arm.angles` recovers each joint angle from the
  evaluated bone matrices relative to the rest pose. Reading
  `pose_bone.rotation_euler` while IK drives the chain returns the *authored*
  value, not the solved one.
- **Writing requires IK off.** The solver overwrites `rotation_euler` on the next
  depsgraph evaluation. `arm.set_angles()` mutes the IK constraints for you;
  `arm.set_tip()` turns them back on.

The same rule governs recording: with IK live the recorder keyframes the tool-tip
empty and lets the solver re-derive the chain on playback; with IK off it bakes
the joint rotations directly.

### Timestep and the timeline

`RobotSim.dt` is *simulated* time per tick, not wall-clock — the loop runs as
fast as it can and motion advances by exactly `dt`, so a run is reproducible no
matter how slow rendering is. Recording maps one tick to one scene frame and sets
the scene frame rate from `dt` (stored as Blender's rational `fps/fps_base`, so
arbitrary `dt` values encode exactly). Keyframes are forced to LINEAR
interpolation: Blender's default Bezier easing would invent motion between
samples that the integrator never produced.

## Base motion

Kinematic models — they integrate a velocity command into a pose and spin the
wheel meshes to match. No mass, no traction, no slip, no contact.

```python
bot.drive.drive(v, omega)              # differential: body-frame command
bot.drive.set_wheel_speeds(l, r)       # differential: wheel-space command
car.drive.drive(speed, steer)          # ackermann: rear-drive, front-steer
```

`AckermannDrive` gives the front wheels true Ackermann angles — the inner wheel
turns sharper than the outer — so the rendered geometry is correct.

## Testing

```bash
make test              # camera render smoke test
make test_anim         # multi-robot animated render, writes a GIF
make test_joints       # joint read/write, limits, IK round-trip
make test_drive        # drive models, wheel roll, sim clock
make test_record       # timeline binding, scrub reproduces sim state
make test_arm_record   # arm recording in both IK and FK modes, plus export
make test_all          # everything
```

Tests assert against physical invariants rather than golden values — a closed
circle returns to its start, scrubbing the timeline reproduces the pose the sim
had, a commanded joint reaches the angle it was given.

## Status

Working today:

- Procedural robot construction: base, wheels, hubs, camera mast, multiple arms
- Joint-space and task-space arm control with limits and IK management
- Differential / skid-steer and Ackermann base motion on a real timestep
- Multi-camera rendering per robot, per frame
- Timeline recording, scrubbing, and standard Blender animation rendering
- ROS2-style joint trajectory export from any recording

Not yet built:

- Physics: no mass, contact, or collision. Wheels do not slip.
- External physics engine hookup (the drive interface is shaped for it)
- Sensors beyond RGB cameras — no depth, segmentation, lidar or proximity
- The PyTorch training loop and Jetson deployment path
- Custom arm mount placement (all arms currently land at the same spot)
- Wheel counts other than 4

---

