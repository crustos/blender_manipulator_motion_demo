"""
Export a recorded arm motion as a ROS2-style joint trajectory.

`blender_to_text.py` is kept as-is for the upstream hand-animated workflow. This
module is the sim-facing replacement, and differs in three ways that matter:

  1. It uses `kinematics.Arm`, so joint angles come from the rest-relative bone
     matrices rather than the largest-magnitude Euler component. The old
     heuristic assumes one DOF per bone on an unknown axis and quietly reports a
     child's *world* rotation when the parent has moved.
  2. Nothing is hardcoded. The old script assumes a scene called "Scene", an
     object called "Armature", and writes to os.getcwd(), which under
     `blender --background` is wherever the shell happened to be.
  3. It restores the scene's original frame when it is done, so exporting is not
     a destructive act in the middle of a session.

Velocities are finite differences of position over the frame interval, which is
what a JointTrajectoryPoint wants and what a downstream controller will use.
"""

import json
import math
import os
import sys

import bpy

# Allow `import joint_export` to work whether or not robotsim has already put the
# repository root on the path.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.append(_ROOT)

from kinematics import Arm  # noqa: E402

TWO_PI = math.pi * 2


def shortest_delta(a, b):
    """b - a, wrapped to (-pi, pi]. Stops a joint crossing +/-pi from reading
    as a ~2pi/dt velocity spike."""
    return (b - a + math.pi) % TWO_PI - math.pi


def sample_trajectory(arm, start=None, end=None, scene=None):
    """
    Walk the timeline and read the arm's joint state at every frame.

    Returns a dict shaped like a ROS2 JointTrajectory. Works regardless of how
    the motion was recorded -- IK targets, baked joint keys, or hand animation --
    because the angles are read from the evaluated pose, not from the fcurves.
    """
    scene = scene or bpy.context.scene
    start = scene.frame_start if start is None else start
    end = scene.frame_end if end is None else end

    fps = scene.render.fps / scene.render.fps_base
    dt = 1.0 / fps if fps else 0.0

    restore = scene.frame_current
    points = []
    try:
        for frame in range(start, end + 1):
            scene.frame_set(frame)
            positions = arm.angles
            point = {
                'frame': frame,
                'time_from_start': (frame - start) * dt,
                'positions': positions,
                'velocities': [0.0] * len(positions),
            }
            if points and dt:
                prev = points[-1]['positions']
                point['velocities'] = [shortest_delta(p, c) / dt
                                       for p, c in zip(prev, positions)]
            points.append(point)
    finally:
        # Leave the scene where we found it.
        scene.frame_set(restore)

    return {
        'joint_names': arm.names,
        'fps': fps,
        'frame_start': start,
        'frame_end': end,
        'points': points,
    }


def write_json(trajectory, path):
    """Machine-readable form, for feeding a controller or a training pipeline."""
    with open(path, 'w') as fh:
        json.dump(trajectory, fh, indent=1)
    return path


def write_text(trajectory, path, precision=4):
    """
    Human-readable form, close in spirit to blender_to_text.py's output but
    with the frame rate and joint names stated once at the top.
    """
    lines = [
        'ROS2 JointTrajectory',
        'Frame rate: {0}'.format(trajectory['fps']),
        'Joint names: {0}'.format(trajectory['joint_names']),
    ]
    for point in trajectory['points']:
        lines.append('{0}'.format([round(v, precision) for v in point['positions']]))
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines))
    return path


def export(arm, path, fmt=None, start=None, end=None, scene=None):
    """
    Sample and write in one call. Format is taken from the file extension
    unless given explicitly.
    """
    trajectory = sample_trajectory(arm, start=start, end=end, scene=scene)
    if fmt is None:
        fmt = 'json' if path.lower().endswith('.json') else 'txt'
    writer = write_json if fmt == 'json' else write_text
    writer(trajectory, path)
    return trajectory


def export_all(bots, directory, fmt='json', scene=None):
    """Export every arm on every robot. Returns the paths written."""
    paths = []
    for bot in bots:
        for index, arm in enumerate(getattr(bot, 'arms', [])):
            name = '%s.arm%d.%s' % (bot.root.name, index, fmt)
            path = os.path.join(directory, name)
            export(arm, path, fmt=fmt, scene=scene)
            paths.append(path)
    return paths
