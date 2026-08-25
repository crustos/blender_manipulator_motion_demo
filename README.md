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

This began as a fork of
[TanJunKiat/blender_manipulator_motion_demo](https://github.com/TanJunKiat/blender_manipulator_motion_demo),
an excellent set of rigged industrial arms and a hand-driven workflow for offline
manipulator motion planning. That workflow still works and is documented below
in full. What this fork adds is everything needed to drive those arms *from code*
rather than from the viewport, and to put a robot underneath them.

> **On the name:** `robotsim` is the working name, taken from the entry-point
> module. Renaming the project and the repository is still an open decision.

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

# Upstream: manipulator motion planning in Blender

Everything below is the original documentation from
[TanJunKiat/blender_manipulator_motion_demo](https://github.com/TanJunKiat/blender_manipulator_motion_demo)
by Tan Jun Kiat, covering the hand-driven workflow: rigging an arm, setting up
IK, inserting keyframes, and the catalogue of included robots. It remains
accurate — the scripting layer above sits on top of these same rigs, and if you
are adding a new arm, this is the section you want.

## Getting started
1. Install [Blender](https://docs.blender.org/manual/en/latest/getting_started/installing/index.html)
2. Download/clone this repository
```
git clone https://github.com/TanJunKiat/blender_manipulator_motion_demo
```
3. Open the desired robotic arm blender file (e.g. UR10.blend)

## Basic Usage / Features

### Moving via tool tip reference
1. Creating tool tip reference
<img src="/resources/getting_started/1_moving_via_tool_tip/1_insert_reference.png"  width="250"/>
For the tool tip, we will be using either of these 3, a plain axes, an arrow axes, or a single arrow. 

> [!NOTE]
> While the tool tip is not limited to these (you can freely use any object as a tool tip reference), it is important to note that the tool tip reference is a reference object and the actual tool tip will not follow the reference closely, especially in scenarios where there is no solution to the reference.

> [!TIP]
> The initial location of the tool tip reference should be conicide with the initial position of the physical tool tip, while the robotic arm is in its neutral state.

2. Linking tool tip reference to armature
<img src="/resources/getting_started/1_moving_via_tool_tip/2_linking_tool_tip_reference.png" width="250"/>

- Under "Bone constraint properties" of the last bone, there should be a Inverse Kinematics (IK) element. Change the "Target" to the tool tip reference object that you just created

3. Moving tool tip reference
<img src="/resources/getting_started/1_moving_via_tool_tip/3_moving_tool_tip.png" height="250"/>
The tool tip reference position can be changed under the "Object properties". The location and rotation is at the global frame. Another way is to change the "Delta Transformation" instead to do it in the intial position of the tool tip reference.

> [!TIP]
> To add an actual tool onto the robotic arm, you can parent the tool on the last link of the robotic arm. This will ensure that the motion of the robotic arm will move the physical tool. 

### Inserting keyframes

1. Selecting time frame in timeline
<img src="/resources/getting_started/2_inserting_keyframe/1_time_line.png" height="150"/>

Under the "Timeline" panel, drag the time frame pointer to the desired time frame

> [!TIP]
> Remember to change the frame rate under the "Output Properties" to get the desired translation from timeframe to time.

2. Enabling properties to animate
Next to each property, there is a diamond shaped button. Clicking it adds a keyframe of that property to the timeline.

> [!NOTE]
> The way Blender animation works is that any non-animated property that is inbetween two animated of the same property will interpolated. So make sure to animate properties that you want to keep constant

### Changing Inverse Kinematics settings

Under your armature and the "Object Data Properties" tab, the inverse kinematics property can be found. Users can change the following parameters:

| Property | Description | Remark |
| ------------- | ------------- |------------- |
| IK Solver  | What kind of solver to use | So far, the best performance is the [iTaSC solver](https://docs.blender.org/manual/en/latest/animation/armatures/posing/bone_constraints/inverse_kinematics/introduction.html) |
| Mode  | Which mode to run the iTaSC | There is animation and simulation. Simulation is preferred as it estimates the velocity of the motion. |
| Reiteration  | When to trigger a re-iteration / re-planning | "Always" is preferred as it will perform a re-plan after every frame. |
| Precision  | The maximum variation of the end effector between two successive iterations at which a pose is obtained that is stable enough and the solver should stop the iterations.  | Default: 0.005 |
| Iterations  | The upper bound for the number of iterations. | Default: 1000 |
| Auto Step  | A substep is a subdivision on the time between two frames for which the solver evaluates the IK equation and updates the joint position. | Use this option if you want to let the solver set how many substeps should be executed for each frame. Default is True |
| Solver  | Which inverse Jacobian solver that iTaSC will use. | Default: SDLS |
| Feedback  | Coefficient on end effector position error to set corrective joint velocity.  | Default: 1.0 |
| Max Velocity  | ndicative maximum joint velocity in radian per second.  | Default: 100.0 |
| Steps Min  | Proposed minimum substep duration (in second). The auto step algorithm may reduce the substep further based on joint velocity. | Default: 0.01 |
| Max | Maximum substep duration (in second). The auto step algorithm will not allow substep longer than this value. | Default: 0.06 |


## Intermediate Features
### Toggling tool tip behaviour

<img src="/resources/intermediate_features/toggling_tool_tip_behaviour/1_changing_joint_limits.png" width="250"/>

- Under "Bone constraint properties" of the last bone and the Inverse Kinematics (IK) tab, you can animate the following parameters:

| Property | Description | Remark |
| ------------- | ------------- |------------- |
| Position | To enable position tracking | Default: True |
| Weight | Weight of position control | Default: 1.0 |
| Lock | Constrain position of axis tool tip to target | Default: True for all |
| Rotation | To enable rotation tracking | Default: True |
| Weight | Weight of rotation control | Default: 1.0 |
| Lock | Constrain rotation of axis tool tip to target | Default: True for all |

> [!TIP]
> The way to toggle these properties in animation time is similar to the above mentioned method.

### Changing joint limits

<img src="/resources/intermediate_features/changing_joint_limits/1_changing_joint_limits.png" width="250"/>

Under "Bone constraint properties" of the bones, there should be two properties, "Limit Rotation" and "Limit Location". 

The rotation of a joint is by default about the Y-axis (direction from head to tail of a bone). So the rotation should be removed for the X and Z axes (limit to 0 degree for max and min). The joint angle of the joint can be saturated by setting the limit of the Y-axis.

Since all the joints of a manipulator are rotaries, all the axes in the limit rotation constraint should be set to true and zeros.

### Setting up multiple robotic arms in one environment
- Import robotic arms using the "Link" or "Append" feature in Blender

> [!NOTE]
> "Link" is to import items from a blender file without breaking the connection; meaning any changes in the referred blender file will update the main file. "Append" is to create a copy of the items from the referred file to the main file. This will allow users to manipulate the items without affecting the original referred file, which is preferred.

## Useful Tips
### Clearing animations
<img src="/resources/useful_tips/clearing_animations/1_clear_animation.png" width="250"/>

1. Select the tool tip reference
2. Select Object > Animation > Clear Keyframes

## Robotic arm catalogue

| Robotic arm  | Brand | Status |
| ------------- | ------------- |------------- |
| UR3  | Universal Robots  |  Available :green_circle: |
| UR3e  | Universal Robots  |  In progress  :yellow_circle: |
| UR5  | Universal Robots  |  Available :green_circle: |
| UR5e  | Universal Robots  |  In pipeline  :red_circle: |
| UR10  | Universal Robots  |  Available :green_circle: |
| UR10e  | Universal Robots  |  In pipeline  :red_circle: |
| UR16e  | Universal Robots  |  In pipeline  :red_circle: |
| UR20  | Universal Robots  |  In pipeline  :red_circle: |
| UR30  | Universal Robots  |  Available :green_circle: |
| ------------- | ------------- |------------- |
| IRB120  | ABB  |  Available  :green_circle: |
| IRB1010  | ABB  |  In pipeline  :red_circle: |
| IRB1090  | ABB  |  In pipeline  :red_circle: |
| IRB1100  | ABB  |  In pipeline  :red_circle: |
| IRB1200  | ABB  |  In pipeline  :red_circle: |
| IRB1300  | ABB  |  In pipeline  :red_circle: |
| IRB1500ID  | ABB  |  In pipeline  :red_circle: |
| IRB1600  | ABB  |  In pipeline  :red_circle: |
| ------------- | ------------- |------------- |
| xArm6  | UFactory  |  Available  :green_circle: |
| ------------- | ------------- |------------- |
| VS-68  | DENSO  |  Available :green_circle: |
| ------------- | ------------- |------------- |

## Examples

Example scenes live in `examples/`. Open any of them in Blender and scrub the
timeline to see the motion.

| Scene | File |
| --- | --- |
| Three arms working in one cell | `examples/3_arm_demo/3_arm_demo.blend` |
| Pick and place | `examples/pick_and_place_demo/pick_and_place_demo.blend` |
| Rotating about a point | `examples/rotate_about_a_point_demo/rotate_about_a_point_demo.blend` |
| Boundary scanning | `examples/boundary_scanning_demo/` (IRB120, UR10, UR30) |
| Path scanning | `examples/path_scanning_demo/xarm6_path_scanning.blend` |
| Following a curve | `examples/curve_following_demo/ur10_following_curve.blend` |
| Multiple UR arms | `examples/ur_arms_demo/ur_arms_demo.blend.zip` |

## Add your own custom robotic arm

1. Import mesh under "File > Import"

<img src="/resources/custom_robotic_arm/1_import_mesh.png" width="250"/>


> [!IMPORTANT]
> The mesh needs to be arranged in the neutral state / starting state so the behaviour will be visually accurate after parenting the bones to the mesh.

2. Create and align bones

<img src="/resources/custom_robotic_arm/2_insert_bone.png" width="250"/>

> [!NOTE]
> The bones do not need to be connected.

> [!NOTE]
> For ease of visualisation, the Y-axis of the bone (which is the direction pointing from the head to tail) represents the direction of rotation of the joint.

3. Parenting bones to mesh

<img src="/resources/custom_robotic_arm/3_bone_parenting.png" width="250"/>

- Switch to Pose mode
- Select the mesh from the "Outliner" panel
- Select the bone that you want to parent
- Parent the bone to the mesh using the "Bone" parenting option

> [!TIP]
> Ctrl+P is the shortcut to bring out the parenting dialog

4. Setting up inverse kinematics
- Select the last bone / joint
- Under "Bone Constraint Properties", add a "Inverse Kinematics" property
- Follow the steps in [here](#moving_via_tool_tip_reference) to set up the tool tip reference
- Follow the steps in [here](#changing-inverse-kinematics-settings) to set up the inverse kinematics of the armature


## Useful information
- [Documentation on Blender animations](https://www.blender.org/features/animation/)
- [Documentation on Blender armatures](https://docs.blender.org/manual/en/latest/animation/armatures/index.html)
- [Documentation on Blender Inverse Kinematics](https://docs.blender.org/manual/en/latest/animation/armatures/posing/bone_constraints/inverse_kinematics/introduction.html)

## Contributors
Tan Jun Kiat - Robotics researcher, currently working in Changi General Hospital as a Simulation Robotics Engineer

Email: tanjunkiat@outlook.sg

LinkedIn: https://www.linkedin.com/in/tan-jun-kiat/

