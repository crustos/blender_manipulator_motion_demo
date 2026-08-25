"""
Bind simulated time to the Blender timeline and bake sim state into keyframes.

Without this, the sim and the scene are two unrelated clocks: `RobotSimpleSim`
counts ticks, the scene sits on frame 1 forever, and nothing that consumes the
timeline (the video sequencer, `ros2/blender_to_text.py`, the render animation
operator, or a human scrubbing the timeline) can see what the sim did.

Recording turns a run into an ordinary Blender action, which means the result is
scrubbable, renderable with the normal animation pipeline, exportable with the
existing ROS2 script, and saveable in a .blend.

THE IK CAVEAT, again:
    Keyframing `pose_bone.rotation_euler` while an IK constraint is driving the
    chain records the *authored* value, not the solved one -- you get a file
    full of zeros. So arms are recorded one of two ways:

      ik_enabled  -> keyframe the tool-tip empty. The IK constraint re-solves
                     the same chain on playback, so the arm reproduces exactly
                     and the recording stays small.
      not enabled -> keyframe the joint rotations directly (true FK bake).

    `bake_joints=True` forces the second form even under IK by reading the
    solved angles out of the bone matrices first.
"""

import math

import bpy


def _linearise(obj_or_bone_owner):
    """
    Force LINEAR interpolation on every keyframe of an object's action.

    Blender defaults to BEZIER, which eases in and out between samples. For a
    fixed-timestep recording that is wrong: it invents motion between samples
    that the integrator never produced, so scrubbing to a recorded frame gives
    a slightly different pose than the sim had. LINEAR reproduces the sim.
    """
    anim = getattr(obj_or_bone_owner, 'animation_data', None)
    if not anim or not anim.action:
        return
    for fcurve in anim.action.fcurves:
        for kp in fcurve.keyframe_points:
            kp.interpolation = 'LINEAR'


class Recorder:
    """
    Samples a RobotSimpleSim once per tick and writes keyframes.

    One sim tick maps to one scene frame. The scene's frame rate is set from
    the sim's `dt` so that timeline seconds and simulated seconds agree.
    """

    def __init__(self, sim, start_frame=1, wheels=True, arms=True,
                 bake_joints=False, scene=None):
        self.sim = sim
        self.start_frame = start_frame
        self.wheels = wheels
        self.arms = arms
        self.bake_joints = bake_joints
        self.scene = scene or bpy.context.scene
        self.frames_written = 0
        self.last_frame = start_frame - 1
        self.bind_scene()

    # -- clock --------------------------------------------------------------

    def bind_scene(self):
        """
        Make one scene frame equal one sim tick.

        Blender stores frame rate as the rational fps/fps_base, so an arbitrary
        dt is expressed exactly rather than rounded to an integer fps.
        """
        dt = self.sim.dt
        if dt <= 0:
            return
        fps = max(1, int(round(1.0 / dt)))
        self.scene.render.fps = fps
        self.scene.render.fps_base = fps * dt
        self.scene.frame_start = self.start_frame

    @property
    def frame(self):
        return self.start_frame + self.sim.ticks

    # -- capture ------------------------------------------------------------

    def capture(self, frame=None):
        """Write one keyframe for every tracked object at the current state."""
        if frame is None:
            frame = self.frame
        for bot in self.sim.bots:
            self.capture_robot(bot, frame)
        self.frames_written += 1
        self.last_frame = frame
        if frame > self.scene.frame_end:
            self.scene.frame_end = frame

    def capture_robot(self, bot, frame):
        root = bot.root
        root.keyframe_insert(data_path='location', frame=frame)
        root.keyframe_insert(data_path='rotation_euler', frame=frame)
        _linearise(root)

        if self.wheels:
            for wheel in getattr(bot, 'wheel_map', {}).values():
                wheel.keyframe_insert(data_path='rotation_euler', frame=frame)
                _linearise(wheel)

        if self.arms:
            for arm in getattr(bot, 'arms', []):
                self.capture_arm(arm, frame)

    def capture_arm(self, arm, frame):
        if arm.ik_enabled and not self.bake_joints:
            # Record the target, let IK re-solve on playback.
            if arm.tip is not None:
                arm.tip.keyframe_insert(data_path='location', frame=frame)
                arm.tip.keyframe_insert(data_path='rotation_euler', frame=frame)
                _linearise(arm.tip)
            return

        # FK bake. Read the solved angles out of the matrices *before* touching
        # rotation_euler, because reading after muting IK would give the pose
        # from whatever was last authored rather than what the sim produced.
        solved = arm.angles
        if arm.ik_enabled:
            arm.ik_enabled = False
        for joint, angle in zip(arm.joints, solved):
            pb = joint.pose_bone
            if pb.rotation_mode == 'QUATERNION':
                pb.rotation_mode = 'XYZ'
            setattr(pb.rotation_euler, joint.axis or 'y', angle)
            pb.keyframe_insert(data_path='rotation_euler', frame=frame)
        _linearise(arm.armature)

    # -- lifecycle ----------------------------------------------------------

    def finish(self, rewind=True):
        """
        Close out the recording and trim the frame range to what was actually
        recorded. This matters: a fresh Blender scene has frame_end=250, so
        without trimming, `render animation` and ros2/blender_to_text.py would
        both grind through hundreds of empty frames past the end of the run.
        """
        self.scene.frame_start = self.start_frame
        if self.frames_written:
            self.scene.frame_end = self.last_frame
        if rewind:
            self.scene.frame_set(self.start_frame)
        return self

    @property
    def duration(self):
        """Recorded length in seconds."""
        return self.frames_written * self.sim.dt

    def clear(self, bots=None):
        """Drop all recorded animation, so a run can be repeated cleanly."""
        for bot in (bots if bots is not None else self.sim.bots):
            targets = [bot.root] + list(getattr(bot, 'wheel_map', {}).values())
            for arm in getattr(bot, 'arms', []):
                targets.append(arm.armature)
                if arm.tip is not None:
                    targets.append(arm.tip)
            for ob in targets:
                if ob and ob.animation_data:
                    ob.animation_data_clear()
        self.frames_written = 0
        self.last_frame = self.start_frame - 1
