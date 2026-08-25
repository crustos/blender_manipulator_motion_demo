#!../headless.py
print('hello arm record test...')
import math
import json
import joint_export

FRAMES = 12
TOL = 1e-3

def close(a, b, tol=TOL):
    return abs(a - b) < tol

scene = bpy.context.scene
bot = Robot()                      ## default arm (abb/irb120)
assert bot.arms, 'no arms were built'
arm = bot.arms[0]
print('arm:', arm, arm.names)

rec = Recorder(RobotSim, start_frame=1, bake_joints=True)
print('scene fps:', scene.render.fps / scene.render.fps_base)


def sweep(i):
    """A per-frame joint target that stays inside limits where they exist."""
    out = []
    for k, joint in enumerate(arm.joints):
        lo, hi = joint.limits
        amp = 0.25
        if lo is not None and hi is not None:
            amp = min(0.25, (hi - lo) * 0.25)
        out.append(math.sin((i + k) * 0.4) * amp)
    return out


## ---------------------------------------------------------------- FK bake
## Commands joints directly, then bakes the solved angles onto pose bones.
## This is the code path record_test.py never reached (it used arms=[]).
truth_fk = {}
for i in range(FRAMES):
    frame = 1 + i
    arm.set_angles(sweep(i))
    truth_fk[frame] = arm.angles
    rec.capture(frame=frame)
rec.finish()

print('fk frames written:', rec.frames_written, 'range', scene.frame_start, '..', scene.frame_end)
assert rec.frames_written == FRAMES
assert scene.frame_end == FRAMES

## the armature must now own an action with one key per frame per joint channel
anim = arm.armature.animation_data
assert anim and anim.action, 'bake_joints wrote no action onto the armature'
curves = anim.action.fcurves
print('armature fcurves:', len(curves), 'keys on first:', len(curves[0].keyframe_points))
assert len(curves) >= len(arm.joints), 'expected at least one channel per joint'
assert len(curves[0].keyframe_points) == FRAMES
for fc in curves:
    for kp in fc.keyframe_points:
        assert kp.interpolation == 'LINEAR', kp.interpolation

## scrubbing must reproduce the commanded joint angles
worst = 0.0
for frame, angles in sorted(truth_fk.items()):
    scene.frame_set(frame)
    got = arm.angles
    for want, have in zip(angles, got):
        worst = max(worst, abs(want - have))
    assert all(close(w, h) for w, h in zip(angles, got)), \
        'frame %d fk mismatch\n  want %s\n  got  %s' % (frame, angles, got)
print('fk scrub reproduces commanded angles, worst error:', worst)

## ---------------------------------------------------------------- export
traj = joint_export.export(arm, '/tmp/arm_fk.json')
assert os.path.isfile('/tmp/arm_fk.json')
print('exported points:', len(traj['points']), 'joints:', len(traj['joint_names']))
assert len(traj['points']) == FRAMES
assert traj['joint_names'] == arm.names
assert close(traj['fps'], 1.0 / RobotSim.dt, 1e-6)

## the export must agree with what the timeline actually holds
for point in traj['points']:
    want = truth_fk[point['frame']]
    for w, h in zip(want, point['positions']):
        assert close(w, h), 'export mismatch at frame %d' % point['frame']

## time_from_start must advance by dt, and velocities be finite differences
p0, p1 = traj['points'][0], traj['points'][1]
assert close(p0['time_from_start'], 0.0)
assert close(p1['time_from_start'], RobotSim.dt, 1e-6)
for k in range(len(arm.joints)):
    expect = (p1['positions'][k] - p0['positions'][k]) / RobotSim.dt
    assert close(p1['velocities'][k], expect, 1e-3), (p1['velocities'][k], expect)
print('velocities are finite differences, dt =', RobotSim.dt)

## exporting must not leave the scene scrubbed somewhere unexpected
before = scene.frame_current
joint_export.sample_trajectory(arm)
assert scene.frame_current == before, 'sample_trajectory moved the playhead'

## the text writer should also work
joint_export.write_text(traj, '/tmp/arm_fk.txt')
assert os.path.isfile('/tmp/arm_fk.txt')

## ---------------------------------------------------------------- IK record
## With IK live, the recorder keyframes the tool tip and lets the solver
## re-derive the chain on playback.
rec.clear()
assert not arm.armature.animation_data, 'clear() left the armature animated'

arm.ik_enabled = True
rec.bake_joints = False
tip = arm.tip
assert tip is not None, 'arm has no tool tip reference'
home = tuple(tip.location)

truth_ik = {}
for i in range(FRAMES):
    frame = 1 + i
    arm.set_tip(location=(home[0],
                          home[1] + 0.02 * math.sin(i * 0.5),
                          home[2] + 0.02 * i / FRAMES))
    truth_ik[frame] = (tuple(tip.location), arm.angles)
    rec.capture(frame=frame)
rec.finish()

assert tip.animation_data and tip.animation_data.action, 'tool tip was not keyframed'
assert not arm.armature.animation_data, \
    'IK mode should keyframe the tip, not bake the bones'
print('ik frames written:', rec.frames_written)

moved = max(abs(truth_ik[FRAMES][1][k] - truth_ik[1][1][k])
            for k in range(len(arm.joints)))
print('largest joint excursion during ik run:', moved)
assert moved > 1e-4, 'tool tip moved but the chain never responded'

worst = 0.0
for frame, (loc, angles) in sorted(truth_ik.items()):
    scene.frame_set(frame)
    for want, have in zip(loc, tuple(tip.location)):
        assert close(want, have), 'frame %d tip mismatch' % frame
    for want, have in zip(angles, arm.angles):
        worst = max(worst, abs(want - have))
    assert all(close(w, h) for w, h in zip(angles, arm.angles)), \
        'frame %d ik pose did not re-solve to the recorded angles' % frame
print('ik scrub re-solves to the recorded pose, worst error:', worst)

## the exporter reads an IK recording just as well, since it samples the
## evaluated pose rather than the fcurves
traj_ik = joint_export.export(arm, '/tmp/arm_ik.json')
assert len(traj_ik['points']) == FRAMES
for point in traj_ik['points']:
    for w, h in zip(truth_ik[point['frame']][1], point['positions']):
        assert close(w, h), 'ik export mismatch at frame %d' % point['frame']

with open('/tmp/arm_ik.json') as fh:
    reread = json.load(fh)
assert reread['joint_names'] == arm.names
print('exported and re-read', len(reread['points']), 'ik points')

print('arm record test OK')
