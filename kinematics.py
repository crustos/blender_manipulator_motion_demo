"""
Joint-space access for Blender armatures used as robot manipulators.

The upstream workflow drives an arm by dragging an IK tool-tip empty and keyframing
it. That is fine for authoring, but a simulation platform needs to *read* and *write*
joint angles directly. This module provides that, with one important asymmetry:

  READING  works in both IK and FK mode. The solved pose only ever exists in the
           evaluated bone matrices, so angles are recovered from
           `pose_bone.matrix` relative to the rest pose. Reading
           `pose_bone.rotation_euler` while IK is driving the chain returns the
           stale authored value, not the solved one -- a classic footgun.

  WRITING  requires IK to be off for the affected chain, otherwise the IK solver
           overwrites whatever is written on the next depsgraph evaluation.
           `Arm.ik_enabled = False` mutes the constraints for you.

Angles are radians throughout, matching bpy.
"""

import math

import bpy
from mathutils import Matrix

AXES = ('x', 'y', 'z')


# ---------------------------------------------------------------------------
# rest / pose maths
# ---------------------------------------------------------------------------

def _rest_local(bone):
    """Rest transform of `bone` expressed in its parent's bone space."""
    if bone.parent:
        return bone.parent.matrix_local.inverted_safe() @ bone.matrix_local
    return bone.matrix_local.copy()


def _pose_local(pose_bone):
    """Evaluated transform of `pose_bone` expressed in its parent's bone space."""
    if pose_bone.parent:
        return pose_bone.parent.matrix.inverted_safe() @ pose_bone.matrix
    return pose_bone.matrix.copy()


def joint_delta(pose_bone):
    """
    Rotation of this bone away from its rest pose, in bone-local space.

    This is the actual joint transform: rest offset divided out, parent motion
    divided out. Works whether the pose came from IK, FK, a constraint, or an
    action, because it reads the evaluated matrices rather than the authored
    rotation properties.
    """
    rest = _rest_local(pose_bone.bone)
    pose = _pose_local(pose_bone)
    return rest.inverted_safe() @ pose


# ---------------------------------------------------------------------------
# Joint
# ---------------------------------------------------------------------------

class Joint:
    """
    A single revolute DOF, backed by one pose bone.

    Bones are referenced by *name*, not by object reference, because pose-bone
    references go stale across file loads and undo steps.
    """

    def __init__(self, arm, bone_name, axis=None):
        self.arm = arm
        self.bone_name = bone_name
        self.axis = axis if axis in AXES else self.infer_axis()

    def __repr__(self):
        return '<Joint %s axis=%s angle=%.3f>' % (self.bone_name, self.axis, self.angle)

    @property
    def pose_bone(self):
        return self.arm.armature.pose.bones[self.bone_name]

    # -- axis ---------------------------------------------------------------

    def infer_axis(self):
        """
        Work out which axis this joint actually rotates about.

        Armatures authored for IK usually lock the two axes they do not use
        (`lock_ik_x` etc). If exactly one axis is free, that is the joint axis.
        If the rig does not lock anything, return None and fall back to the
        largest-magnitude Euler component at read time -- the same heuristic
        ros2/blender_to_text.py uses, kept only as a last resort.
        """
        pb = self.pose_bone
        free = [ax for ax in AXES if not getattr(pb, 'lock_ik_' + ax, False)]
        if len(free) == 1:
            return free[0]
        return None

    # -- state --------------------------------------------------------------

    @property
    def angle(self):
        """Current joint angle in radians. Valid under both IK and FK."""
        euler = joint_delta(self.pose_bone).to_euler('XYZ')
        if self.axis:
            return getattr(euler, self.axis)
        # Unknown axis: assume one dominant DOF and take the largest component.
        vals = [euler.x, euler.y, euler.z]
        return max(vals, key=abs)

    @property
    def limits(self):
        """(min, max) in radians from the bone's IK limits, or (None, None)."""
        pb = self.pose_bone
        ax = self.axis
        if not ax or not getattr(pb, 'use_ik_limit_' + ax, False):
            return (None, None)
        return (getattr(pb, 'ik_min_' + ax), getattr(pb, 'ik_max_' + ax))

    def clamp(self, value):
        lo, hi = self.limits
        if lo is not None and value < lo:
            return lo
        if hi is not None and value > hi:
            return hi
        return value

    @angle.setter
    def angle(self, value):
        """
        Command this joint (FK). Requires `arm.ik_enabled = False` first --
        otherwise the solver silently overwrites this on the next evaluation.
        """
        pb = self.pose_bone
        if pb.rotation_mode == 'QUATERNION':
            # Pose bones default to quaternion; Euler is far easier to reason
            # about for a single-DOF revolute joint.
            pb.rotation_mode = 'XYZ'
        ax = self.axis or 'y'
        setattr(pb.rotation_euler, ax, self.clamp(value))


# ---------------------------------------------------------------------------
# Arm
# ---------------------------------------------------------------------------

class Arm:
    """
    A manipulator: one armature, its joints, its IK constraints, and the
    tool-tip empty that the IK chain targets.
    """

    def __init__(self, armature, tip=None, root=None, skip_base_bone=True):
        assert armature.type == 'ARMATURE', armature
        self.armature = armature
        self.tip = tip            # the IK target empty ("Tool tip")
        self.root = root          # ARM.ROOT empty this arm is parented under
        self._ik_enabled = True

        bones = list(armature.pose.bones)
        # The first bone is conventionally the fixed base/pedestal in these rigs
        # and is not a controllable DOF.
        if skip_base_bone and bones:
            bones = bones[1:]
        self.joints = [Joint(self, pb.name) for pb in bones]

    def __repr__(self):
        return '<Arm %s joints=%d ik=%s>' % (
            self.armature.name, len(self.joints), self._ik_enabled)

    @classmethod
    def from_objects(cls, objects, root=None):
        """
        Build Arms from the list returned by robotsim.load_blend_objects().
        Returns a list, since one .blend may contain more than one armature.
        """
        tip = None
        for ob in objects:
            if ob.name.startswith('Tool tip'):
                tip = ob
        return [cls(ob, tip=tip, root=root)
                for ob in objects if ob.type == 'ARMATURE']

    # -- IK -----------------------------------------------------------------

    @property
    def ik_constraints(self):
        out = []
        for pb in self.armature.pose.bones:
            for con in pb.constraints:
                if con.type == 'IK':
                    out.append(con)
        return out

    @property
    def ik_enabled(self):
        return self._ik_enabled

    @ik_enabled.setter
    def ik_enabled(self, value):
        """Mute/unmute IK so FK joint commands are not overwritten by the solver."""
        value = bool(value)
        for con in self.ik_constraints:
            con.mute = not value
        self._ik_enabled = value

    # -- joint space --------------------------------------------------------

    @property
    def names(self):
        return [j.bone_name for j in self.joints]

    @property
    def angles(self):
        """Current joint vector, radians. Valid under IK or FK."""
        return [j.angle for j in self.joints]

    @property
    def degrees(self):
        return [math.degrees(a) for a in self.angles]

    def set_angles(self, values, clamp=True, update=True):
        """
        Command the joint vector (FK). Silently switches IK off, because
        leaving it on makes the write a no-op and that failure is invisible.
        """
        assert len(values) == len(self.joints), (
            'expected %d angles, got %d' % (len(self.joints), len(values)))
        if self._ik_enabled and self.ik_constraints:
            self.ik_enabled = False
        for joint, value in zip(self.joints, values):
            joint.angle = value if clamp else value
        if update:
            self.update()

    def home(self, update=True):
        """Return every joint to its rest pose."""
        self.set_angles([0.0] * len(self.joints), update=update)

    # -- task space ---------------------------------------------------------

    def set_tip(self, location=None, rotation=None, update=True):
        """
        Move the IK target empty. This is the upstream authoring workflow,
        exposed as an API call rather than a viewport drag.
        """
        assert self.tip is not None, 'this arm has no tool tip reference'
        if not self._ik_enabled:
            self.ik_enabled = True
        if location is not None:
            self.tip.location = location
        if rotation is not None:
            self.tip.rotation_euler = rotation
        if update:
            self.update()

    @property
    def tip_world(self):
        """World-space matrix of the last bone's tail -- the physical tool tip."""
        pb = self.armature.pose.bones[-1]
        return self.armature.matrix_world @ Matrix.Translation(pb.tail)

    # -- evaluation ---------------------------------------------------------

    def update(self):
        """
        Force a depsgraph evaluation so poses, IK solutions and world matrices
        reflect the writes above. Without this, a read or a render immediately
        after a write sees the previous frame's state.
        """
        bpy.context.view_layer.update()

    def state(self):
        """Serialisable snapshot, shaped for a ROS2 JointState-style consumer."""
        return {
            'name': self.armature.name,
            'joint_names': self.names,
            'position': self.angles,
        }
