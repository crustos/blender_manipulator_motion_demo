#!../headless.py
print('hello rig test...')
import math

DT = 1.0 / 60.0
SIZE = (1.0, 1.0, 0.1)


def run(bot, secs, dt=DT):
    for _ in range(int(round(secs / dt))):
        bot.step(dt)


def close(a, b, tol=1e-3):
    return abs(a - b) < tol


def local(name, bot):
    """Body-frame location of a wheel, by unsuffixed name."""
    for w in bot.wheel_list:
        if w.name.split('.')[:3] == name.split('.')[:3]:
            return w
    raise AssertionError('no wheel %s in %s' % (name, [w.name for w in bot.wheel_list]))


## -------------------------------------------------------------- wheel counts
## The four-wheel layout is the one every existing test and .blend depends on,
## so it must come out byte-for-byte where it always was.
bot = Robot(arms=[], wheels=4)
expect = {
    'W.L.FRONT': (-0.5,  0.5, -0.05),
    'W.R.FRONT': ( 0.5,  0.5, -0.05),
    'W.L.REAR':  (-0.5, -0.5, -0.05),
    'W.R.REAR':  ( 0.5, -0.5, -0.05),
}
assert len(bot.wheel_map) == 4, list(bot.wheel_map)
for name, want in expect.items():
    w = local(name, bot)
    got = tuple(w.obj.location)
    print('   %-11s %s' % (name, tuple(round(v, 3) for v in got)))
    for a, b in zip(got, want):
        assert close(a, b), '%s moved: %s vs %s' % (name, got, want)
assert close(bot.drive.track, 1.0), bot.drive.track

## every supported count builds, and builds the shape we claimed
for count, names in [
    (1, ['W.C.MID']),
    (2, ['W.L.MID', 'W.R.MID']),
    (3, ['W.C.FRONT', 'W.L.REAR', 'W.R.REAR']),
    (4, ['W.L.FRONT', 'W.R.FRONT', 'W.L.REAR', 'W.R.REAR']),
    (6, ['W.L.FRONT', 'W.R.FRONT', 'W.L.MID', 'W.R.MID', 'W.L.REAR', 'W.R.REAR']),
]:
    b = Robot(arms=[], wheels=count)
    got = ['.'.join(w.name.split('.')[:3]) for w in b.wheel_list]
    print('wheels=%d -> %s' % (count, got))
    assert len(got) == count, (count, got)
    assert sorted(got) == sorted(names), (got, names)

## a robot with no wheels at all is legal -- a fixed base, or a tracked one
b = Robot(arms=[], wheels=0)
assert b.wheel_list == [], b.wheel_list
b.drive.drive(1.0)
run(b, 1.0)
assert close(b.root.location.y, 1.0), 'wheel-less base should still move'

## ------------------------------------------------------- odd counts / casters
## The odd wheel goes on the centre line at the front and is not powered.
tri = Robot(arms=[], wheels=3)
caster = local('W.C.FRONT', tri)
assert caster.side == 'C', caster
assert caster.driven is False, 'the odd wheel out should be passive'
assert close(caster.x, 0.0), 'caster is not on the centre line'
assert local('W.L.REAR', tri).driven is True

## it still rolls, though: dragging a caster forward spins it
tri.drive.drive(1.0)
run(tri, 2.0)
expect_roll = -2.0 / tri.wheel_radius
print('caster roll after 2m:', round(caster.obj.rotation_euler.x, 3),
      'expect', round(expect_roll, 3))
assert close(caster.obj.rotation_euler.x, expect_roll, 1e-2)

## --------------------------------------------------- six wheels actually work
## The real risk with extra axles is that the drive model silently rolls them at
## the wrong speed. In a turn, every wheel on a side shares one surface speed.
six = Robot(arms=[], wheels=6)
six.drive.drive(1.0, 0.8)
run(six, 1.0)
lefts = [w for w in six.wheel_list if w.side == 'L']
rights = [w for w in six.wheel_list if w.side == 'R']
assert len(lefts) == 3 and len(rights) == 3, (lefts, rights)
lroll = [w.obj.rotation_euler.x for w in lefts]
rroll = [w.obj.rotation_euler.x for w in rights]
print('6-wheel roll  L:', [round(v, 3) for v in lroll])
print('              R:', [round(v, 3) for v in rroll])
for v in lroll[1:]:
    assert close(v, lroll[0], 1e-6), 'left wheels disagree: %s' % lroll
for v in rroll[1:]:
    assert close(v, rroll[0], 1e-6), 'right wheels disagree: %s' % rroll
## outer (right) side of a left turn travels further
assert abs(rroll[0]) > abs(lroll[0]), 'outer wheels should roll further in a turn'
## and by exactly the commanded amount
assert close(lroll[0], -six.drive.left / six.wheel_radius, 1e-2), lroll[0]
assert close(rroll[0], -six.drive.right / six.wheel_radius, 1e-2), rroll[0]

## a six-wheeler must still close a circle
six2 = Robot(arms=[], wheels=6)
six2.drive.drive(1.0, 1.0)
run(six2, 2 * math.pi, 1.0 / 240.0)
print('6-wheel closed circle:', tuple(round(v, 4) for v in six2.root.location))
assert close(six2.root.location.x, 0.0, 0.02), six2.root.location.x
assert close(six2.root.location.y, 0.0, 0.02), six2.root.location.y

## ------------------------------------------------------- tricycle steering
## Three wheels under ackermann is a tricycle: the single front wheel steers,
## and takes the centre-line angle rather than an Ackermann-corrected one.
trike = Robot(arms=[], wheels=3, drive='ackermann')
trike.drive.drive(1.0, math.radians(20))
run(trike, 0.5)
front = local('W.C.FRONT', trike)
assert front.steerable, 'front centre wheel should steer'
print('trike steer:', round(front.obj.rotation_euler.z, 4),
      'commanded', round(trike.drive.steer, 4))
assert close(front.obj.rotation_euler.z, trike.drive.steer), front.obj.rotation_euler.z
for w in trike.wheel_list:
    if not w.steerable:
        assert close(w.obj.rotation_euler.z, 0.0), '%s should not steer' % w.name

## ----------------------------------------------------- custom wheel placement
## Wheels given by hand: a narrow rear pair and one long-reach front wheel.
custom = Robot(arms=[], wheels=[
    {'name': 'W.L.REAR',  'location': (-0.2, -0.4, -0.05)},
    {'name': 'W.R.REAR',  'location': ( 0.2, -0.4, -0.05)},
    {'name': 'W.C.FRONT', 'location': ( 0.0,  0.9, -0.05), 'driven': False},
])
assert len(custom.wheel_list) == 3, custom.wheel_list
assert close(custom.drive.track, 0.4), custom.drive.track   ## measured, not the body width
print('custom track:', custom.drive.track)
## side is inferred from the sign of the offset when not stated
assert local('W.L.REAR', custom).side == 'L'
assert local('W.R.REAR', custom).side == 'R'
assert local('W.C.FRONT', custom).side == 'C'
## a narrower track means a given yaw rate needs a smaller wheel-speed split
custom.drive.drive(0.0, 1.0)
assert close(custom.drive.right - custom.drive.left, 0.4), (custom.drive.left, custom.drive.right)
custom.drive.drive(0.0, math.pi / 2)
run(custom, 1.0)
print('custom spin yaw:', round(custom.root.rotation_euler.z, 4))
assert close(custom.root.rotation_euler.z, math.pi / 2), custom.root.rotation_euler.z

## -------------------------------------------------------------- arm mounting
## One arm still lands exactly where it always did: centre of the front edge.
one = Robot(arms=[DEFAULT_ARM])
assert len(one.arm_roots) == 1
loc = tuple(one.arm_roots[0].location)
print('single arm mount:', tuple(round(v, 3) for v in loc))
for got, want in zip(loc, (0.0, 0.5, 0.1)):
    assert close(got, want), (loc, 'legacy placement changed')
assert close(one.arm_roots[0].rotation_euler.z, math.pi / 2)

## Several arms must land in *different* places -- this is the actual bug.
three = Robot(arms=[DEFAULT_ARM] * 3)
xs = [round(r.location.x, 4) for r in three.arm_roots]
print('three arm mount x:', xs)
assert len(three.arms) == 3, three.arms
assert len(set(xs)) == 3, 'arms are still stacked at one point: %s' % xs
assert xs == sorted(xs), 'arms should be laid out left to right: %s' % xs
for r in three.arm_roots:
    assert abs(r.location.x) <= SIZE[0] * 0.5, 'arm hangs off the side of the body'
## each arm is independently controllable, i.e. they are really separate rigs
a, b_arm = three.arms[0], three.arms[1]
a.set_angles([0.2] * len(a.joints))
b_arm.set_angles([-0.2] * len(b_arm.joints))
assert all(abs(v - 0.2) < 1e-3 for v in a.angles), a.angles
assert all(abs(v + 0.2) < 1e-3 for v in b_arm.angles), b_arm.angles

## Explicit placement wins over the default layout.
placed = Robot(arms=[
    (DEFAULT_ARM, (0.3, -0.2, 0.4)),
    {'path': DEFAULT_ARM, 'location': (-0.3, 0.2, 0.4), 'rotation': (0, 0, math.pi)},
])
got = tuple(round(v, 3) for v in placed.arm_roots[0].location)
print('placed arm 0:', got, ' arm 1:', tuple(round(v, 3) for v in placed.arm_roots[1].location))
assert got == (0.3, -0.2, 0.4), got
assert close(placed.arm_roots[1].location.x, -0.3)
assert close(placed.arm_roots[1].rotation_euler.z, math.pi), 'explicit rotation ignored'

## An arm can be mounted onto a named part rather than the root, and added after
## the robot has already been built.
late = Robot(arms=[])
assert late.arms == []
late.add_arm(DEFAULT_ARM, location=(0, 0, 0.3), parent='front_hub')
assert len(late.arms) == 1, late.arms
mounted = late.arm_roots[0]
assert mounted.parent is late.front_hub, mounted.parent
## parented to the hub means it inherits the hub's offset in world space
world_y = (mounted.matrix_world.translation).y
print('hub-mounted arm world y:', round(world_y, 3))
assert world_y > 0.4, 'arm did not inherit the front hub transform: %s' % world_y
assert late.mounts[0]['parent'] == late.front_hub.name, late.mounts

## the arm mounted on the hub rides along when the robot drives
before = mounted.matrix_world.translation.copy()
late.drive.drive(1.0)
run(late, 1.0)
bpy.context.view_layer.update()
after = mounted.matrix_world.translation
print('arm moved with base:', round(after.y - before.y, 3))
assert close(after.y - before.y, 1.0, 1e-2), (before.y, after.y)

print('rig test OK')
