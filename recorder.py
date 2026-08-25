"""
Bind simulated time to the Blender timeline and bake sim state into keyframes.

Without this, the sim and the scene are two unrelated clocks: `RobotSimpleSim`
counts ticks, the scene sits on frame 1 forever, and nothing that consumes the
timeline (the video sequencer, `ros2/joint_export.py`, the render animation
operator, or a human scrubbing the timeline) can see what the sim did.

Recording turns a run into an ordinary Blender action, which means the result is
scrubbable, renderable with the normal animation pipeline, exportable, and
saveable in a .blend.

WHY SAMPLES ARE BUFFERED RATHER THAN KEYFRAMED AS THEY ARRIVE:
    The moment an object owns an action, *every* depsgraph evaluation overwrites
    that object's transform with the action evaluated at the current scene frame.
    During a run the playhead does not move, so writing keyframes tick by tick
    means that from the second tick onward, anything calling
    `view_layer.update()` -- which `Arm.set_angles()` and `Arm.set_tip()` both do
    -- snaps the robot back to its frame-1 pose. The sim then records that frozen
    pose forever, and a scrub test passes trivially because every frame holds the
    same values.

    So `capture()` only reads state into memory. All keyframes are written in
    `finish()`, once the run is over and the feedback loop cannot form.

THE IK CAVEAT:
    Keyframing `pose_bone.rotation_euler` while an IK constraint is driving the
    chain records the *authored* value, not the solved one -- you get a file full
    of zeros. So arms are recorded one of two ways:

      ik_enabled  -> sample the tool-tip empty. The IK constraint re-solves the
                     same chain on playback, so the arm reproduces exactly and
                     the recording stays small.
      not enabled -> sample the joint angles directly (a true FK bake).

    `bake_joints=True` forces the second form even under IK, reading the solved
    angles out of the bone matrices.
"""

import math

import bpy


def _linearise(owner):
    """
    Force LINEAR interpolation on every keyframe of an object's action.

    Blender defaults to BEZIER, which eases in and out between samples. For a
    fixed-timestep recording that is wrong: it invents motion between samples
    that the integrator never produced, so scrubbing to a recorded frame gives a
    slightly different pose than the sim had. LINEAR reproduces the sim.
    """
    anim = getattr(owner, 'animation_data', None)
    if not anim or not anim.action:
        return
    for fcurve in anim.action.fcurves:
        for kp in fcurve.keyframe_points:
            kp.interpolation = 'LINEAR'


class Recorder:
    """
    Samples a RobotSimpleSim once per tick and writes keyframes when finished.

    One sim tick maps to one scene frame. The scene's frame rate is set from the
    sim's `dt` so that timeline seconds and simulated seconds agree.
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
        self._samples = []      ## [(frame, [action, ...])] pending keyframes
        self._touched = []      ## objects that received animation data
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

    @property
    def duration(self):
        """Recorded length in seconds."""
        return self.frames_written * self.sim.dt

    @property
    def pending(self):
        """Frames sampled but not yet written to the timeline."""
        return len(self._samples)

    # -- sampling -----------------------------------------------------------

    def capture(self, frame=None):
        """
        Read the current state of every tracked object into memory.

        Deliberately does not touch the timeline; see the module docstring.
        """
        if frame is None:
            frame = self.frame
        actions = []
        for bot in self.sim.bots:
            self._sample_robot(bot, actions)
        self._samples.append((frame, actions))
        self.frames_written += 1
        self.last_frame = frame

    def _sample_robot(self, bot, actions):
        root = bot.root
        actions.append((root, 'location', tuple(root.location)))
        actions.append((root, 'rotation_euler', tuple(root.rotation_euler)))

        if self.wheels:
            for wheel in getattr(bot, 'wheel_map', {}).values():
                actions.append((wheel, 'rotation_euler', tuple(wheel.rotation_euler)))

        if self.arms:
            for arm in getattr(bot, 'arms', []):
                self._sample_arm(arm, actions)

    def _sample_arm(self, arm, actions):
        if arm.ik_enabled and not self.bake_joints:
            ## Record the target and let IK re-solve on playback.
            if arm.tip is not None:
                actions.append((arm.tip, 'location', tuple(arm.tip.location)))
                actions.append((arm.tip, 'rotation_euler', tuple(arm.tip.rotation_euler)))
            return
        ## FK bake: read the solved angles out of the evaluated matrices.
        for joint, angle in zip(arm.joints, arm.angles):
            actions.append((arm, joint.bone_name, joint.axis or 'y', angle))

    # -- writing ------------------------------------------------------------

    def flush(self):
        """
        Write every buffered sample to the timeline as keyframes.

        Safe to call more than once; the buffer is emptied as it is written.
        """
        if not self._samples:
            return 0
        written = 0
        for frame, actions in self._samples:
            for action in actions:
                if len(action) == 3:
                    obj, data_path, values = action
                    setattr(obj, data_path, values)
                    obj.keyframe_insert(data_path=data_path, frame=frame)
                    if obj not in self._touched:
                        self._touched.append(obj)
                else:
                    arm, bone_name, axis, angle = action
                    ## Muting IK is required before authored rotations mean
                    ## anything; the solver would otherwise overwrite them.
                    if arm.ik_enabled:
                        arm.ik_enabled = False
                    pb = arm.armature.pose.bones[bone_name]
                    if pb.rotation_mode == 'QUATERNION':
                        pb.rotation_mode = 'XYZ'
                    setattr(pb.rotation_euler, axis, angle)
                    pb.keyframe_insert(data_path='rotation_euler', frame=frame)
                    if arm.armature not in self._touched:
                        self._touched.append(arm.armature)
            written += 1
        self._samples = []
        for obj in self._touched:
            _linearise(obj)
        return written

    def finish(self, rewind=True):
        """
        Write the recording and trim the frame range to what was captured.

        Trimming matters: a fresh Blender scene has frame_end=250, so without it
        `render animation` and the trajectory exporters would grind through
        hundreds of empty frames past the end of the run.
        """
        self.flush()
        self.scene.frame_start = self.start_frame
        if self.frames_written:
            self.scene.frame_end = self.last_frame
        if rewind:
            self.scene.frame_set(self.start_frame)
        return self

    # -- lifecycle ----------------------------------------------------------

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
        self._samples = []
        self._touched = []
        self.frames_written = 0
        self.last_frame = self.start_frame - 1
