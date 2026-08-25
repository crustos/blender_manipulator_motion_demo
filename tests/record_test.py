#!../headless.py
print('hello record test...')
import math

DT = 1.0 / 30.0
TICKS = 20

def close(a, b, tol=1e-4):
    return abs(a - b) < tol

RobotSim.dt = DT
bot = Robot(arms=[])
rec = RobotSim.record(start_frame=1)

## the scene clock should now agree with the sim clock
scene = bpy.context.scene
fps = scene.render.fps / scene.render.fps_base
print('scene fps:', fps, 'sim dt:', DT, '-> 1/dt =', 1.0 / DT)
assert close(fps, 1.0 / DT, 1e-6), fps

## drive a curve so both position and yaw change every tick
bot.drive.drive(1.0, 0.6)

truth = {}   ## frame -> (x, y, yaw, left wheel roll)

@RobotSim
def callback(dt):
    f = RobotSim.frame
    truth[f] = (bot.root.location.x, bot.root.location.y,
                bot.root.rotation_euler.z,
                bot.wheel_map['W.L.REAR'].rotation_euler.x)
    if RobotSim.ticks >= TICKS - 1:
        RobotSim.stop()

while RobotSim.callbacks:
    RobotSim.update()

print('ticks:', RobotSim.ticks, 'frames written:', rec.frames_written)
print('recorded duration:', rec.duration, 's   sim time:', RobotSim.time, 's')
assert rec.frames_written == RobotSim.ticks, (rec.frames_written, RobotSim.ticks)
assert close(rec.duration, RobotSim.time), (rec.duration, RobotSim.time)

## the frame range should cover the run, so render animation / the ROS2 exporter
## both see the whole thing
print('frame range:', scene.frame_start, '..', scene.frame_end)
assert scene.frame_start == 1
assert scene.frame_end == max(truth), (scene.frame_end, max(truth))
assert scene.frame_end == RobotSim.ticks, 'frame_end not trimmed to the run'

## an action must actually exist
assert bot.root.animation_data and bot.root.animation_data.action, 'root has no action'
curves = bot.root.animation_data.action.fcurves
print('root fcurves:', len(curves), 'keys on first:', len(curves[0].keyframe_points))
assert len(curves[0].keyframe_points) == RobotSim.ticks

## interpolation must be LINEAR, or scrubbing will not reproduce the sim
for fc in curves:
    for kp in fc.keyframe_points:
        assert kp.interpolation == 'LINEAR', kp.interpolation

## THE REAL TEST: scrub the timeline and check the pose matches what the sim had.
## This only passes if the keyframes, the frame mapping and the interpolation are
## all correct together.
worst = 0.0
for frame, (x, y, yaw, roll) in sorted(truth.items()):
    scene.frame_set(frame)
    dx = abs(bot.root.location.x - x)
    dy = abs(bot.root.location.y - y)
    dyaw = abs(bot.root.rotation_euler.z - yaw)
    droll = abs(bot.wheel_map['W.L.REAR'].rotation_euler.x - roll)
    worst = max(worst, dx, dy, dyaw, droll)
    assert dx < 1e-4 and dy < 1e-4, 'frame %d position mismatch %s %s' % (frame, dx, dy)
    assert dyaw < 1e-4, 'frame %d yaw mismatch %s' % (frame, dyaw)
    assert droll < 1e-3, 'frame %d wheel roll mismatch %s' % (frame, droll)
print('scrub reproduces sim state, worst error:', worst)

## between two samples, linear interpolation should land between them, not
## overshoot the way bezier easing would
f0, f1 = sorted(truth)[2], sorted(truth)[3]
scene.frame_set(f0)
y0 = bot.root.location.y
scene.frame_set(f1)
y1 = bot.root.location.y
scene.frame_set(f0)  ## integer frames only; check monotonicity instead
assert (y1 - y0) > 0, 'expected forward motion between consecutive frames'

## clear() should make a rerun possible
rec.clear()
assert not bot.root.animation_data, 'clear() left animation data behind'
print('record test OK')
