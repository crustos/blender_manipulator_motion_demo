#!../headless.py
print('hello contact test...')
import bpy, math, time

## Lanes again: robots left stacked at the origin collide with each other, which
## is correct behaviour and useless as a test.
LANE = [0]
def next_lane(step=120.0):
    LANE[0] += step
    return LANE[0]

## Big enough for every lane: a robot that runs off the edge of the ground has
## nothing to settle onto and silently keeps its spawn height.
GROUND = create_cube('GROUND', size=(8000, 8000, 0.2), location=(0, 0, -0.1))
bpy.context.view_layer.update()

DT = 1.0 / 60.0


def close(a, b, tol=1e-2):
    return abs(a - b) < tol


def run(bot, ticks):
    for _ in range(ticks):
        bot.step(DT)


def robot_at(x, y=0.0, z=0.4, **kw):
    bot = Robot(arms=[], cameras='none', **kw)
    bot.root.location = (x, y, z)
    bpy.context.view_layer.update()
    return bot


def block(name, x, y, size=(20, 0.4, 2), z=1.0, rot_z=0.0):
    b = create_cube(name, size=size, location=(x, y, z))
    b.rotation_euler.z = rot_z
    bpy.context.view_layer.update()
    return b


def test_inertia():
    """Acceleration limits give the base momentum, without modelling a force."""
    x = next_lane()
    ## unlimited by default: the base is its own command within one tick
    quick = robot_at(x)
    quick.drive.drive(2.0)
    quick.step(DT)
    assert close(quick.drive.v, 2.0), quick.drive.v

    heavy = robot_at(x + 20, max_accel=1.0)
    heavy.drive.drive(2.0)
    for _ in range(30):
        heavy.step(DT)
    print('after 0.5s at 1.0 m/s^2: v=%.3f' % heavy.drive.v)
    assert close(heavy.drive.v, 0.5, 0.02), heavy.drive.v
    ## and it saturates at the command, not beyond it
    for _ in range(300):
        heavy.step(DT)
    assert close(heavy.drive.v, 2.0, 0.01), heavy.drive.v

    ## commanding a stop makes it coast: v^2/2a metres, not an instant halt
    y0 = heavy.root.location.y
    heavy.drive.drive(0.0)
    for _ in range(300):
        heavy.step(DT)
    coasted = heavy.root.location.y - y0
    print('coasted %.3f m stopping from 2.0 (v^2/2a = %.3f)' % (coasted, 4.0 / 2.0))
    assert close(heavy.drive.v, 0.0, 1e-6), heavy.drive.v
    assert close(coasted, 2.0, 0.05), coasted
    print('inertia OK')


def test_slip_reports_lockup():
    """Slip means wheels disagreeing with the base, in either direction."""
    x = next_lane()
    bot = robot_at(x, max_accel=1.0)
    bot.drive.drive(2.0)
    bot.step(DT)
    ## spinning up: wheels already at speed, base barely moving
    print('accelerating slip %.2f' % bot.drive.slip)
    assert bot.drive.slip > 0.9, bot.drive.slip

    for _ in range(300):
        bot.step(DT)
    assert close(bot.drive.slip, 0.0, 1e-6), 'no slip once up to speed'

    ## brakes locked, base still sliding: total slip, not undefined
    bot.drive.drive(0.0)
    bot.step(DT)
    print('lockup slip %.2f' % bot.drive.slip)
    assert close(bot.drive.slip, 1.0, 1e-6), bot.drive.slip
    print('slip reporting OK')


def test_impact_takes_the_speed():
    """A collision consumes velocity, so recovery is not instantaneous."""
    x = next_lane()
    bot = robot_at(x, max_accel=2.0)
    bot.enable_contact()
    wall = block('WALL.I', x, 8)
    bot.drive.drive(2.0)
    for _ in range(400):
        bot.step(DT)
        if bot.drive.last_contact and bot.drive.last_contact.blocked:
            break
    assert bot.drive.last_contact.blocked, 'never reached the wall'
    assert close(bot.drive.v, 0.0, 1e-6), 'impact should take the speed: %s' % bot.drive.v

    ## pinned: it settles at exactly body-radius from the face and stays there
    run(bot, 400)
    face = 8 - 0.2
    assert close(bot.root.location.y, face - bot.contact.radius, 1e-3), bot.root.location.y
    assert close(bot.drive.v, 0.0, 1e-6), bot.drive.v

    ## take the wall away: it must accelerate again from rest rather than
    ## resuming full speed, which is what a pose-only seam could not express
    bpy.data.objects.remove(wall)
    bpy.context.view_layer.update()
    bot.step(DT)
    print('one tick after the wall is gone: v=%.3f' % bot.drive.v)
    assert bot.drive.v < 0.1, 'leapt back to speed: %s' % bot.drive.v
    run(bot, 120)
    assert close(bot.drive.v, 2.0, 0.01), bot.drive.v
    print('impact velocity OK')


def test_no_contact_is_unchanged():
    """Without a contact model the commanded pose is still the pose."""
    x = next_lane()
    bot = robot_at(x, z=0.15)
    block('GHOST.WALL', x, 5)
    assert bot.drive.contact is None
    start_z = bot.root.location.z
    bot.drive.drive(2.0)
    run(bot, 300)
    print('no contact: y=%.2f z=%.3f' % (bot.root.location.y, bot.root.location.z))
    assert bot.root.location.y > 8, 'should have driven straight through the wall'
    assert close(bot.root.location.z, start_z), 'z should not move without contact'
    ## float round-trip through the pose, so not bit-exact zero
    assert close(bot.drive.slip, 0.0, 1e-6), bot.drive.slip
    print('no-contact behaviour preserved OK')


def test_settles_onto_ground():
    """The base rides at ride_height above whatever is under it."""
    x = next_lane()
    high = robot_at(x, z=5.0)
    high.enable_contact()
    high.step(DT)
    expect = high.wheel_radius * 1.5
    print('dropped from 5.0 -> %.3f (ride height %.3f)' % (high.root.location.z, expect))
    assert close(high.root.location.z, expect), high.root.location.z

    ## and lifted from below, not only dropped from above
    low = robot_at(x + 20, z=-0.4)
    low.enable_contact()
    low.step(DT)
    assert close(low.root.location.z, expect), low.root.location.z
    assert low.drive.last_contact.grounded
    assert close(low.drive.last_contact.ground_z, 0.0), low.drive.last_contact.ground_z
    print('ground settling OK')


def test_wall_blocks_and_skids():
    """A wall stops the base, and the wheels keep turning against it."""
    x = next_lane()
    bot = robot_at(x)
    bot.enable_contact()
    wall = block('WALL.B', x, 6)
    bot.drive.drive(2.0)
    run(bot, 60)
    assert close(bot.drive.slip, 0.0, 1e-6), 'should be running free before the wall'
    assert bot.root.location.y > 1.0

    run(bot, 400)
    ## the body stops with its leading edge at the wall face, not its centre
    face = 6 - 0.2
    expect = face - bot.contact.radius
    print('stopped at y=%.3f (wall face %.2f, body radius %.2f)'
          % (bot.root.location.y, face, bot.contact.radius))
    assert close(bot.root.location.y, expect, 0.05), (bot.root.location.y, expect)

    info = bot.drive.last_contact
    assert info.blocked, 'contact should report the block'
    assert info.object is wall, info.object
    assert close(bot.drive.slip, 1.0), bot.drive.slip

    ## wheels keep turning while the base is held: that is what skid is
    wheel = bot.wheel_list[0].obj
    before = wheel.rotation_euler.x
    held = bot.root.location.y
    run(bot, 60)
    assert abs(wheel.rotation_euler.x - before) > 0.5, 'wheels should still spin'
    assert close(bot.root.location.y, held), 'base should not creep into the wall'
    print('blocking and skid OK')


def test_slide_along_wall():
    """A shallow approach skates along the surface instead of stopping dead."""
    x = next_lane()
    bot = robot_at(x)
    bot.enable_contact()
    block('WALL.S', x, 6, size=(30, 0.4, 2), rot_z=math.radians(35))
    bot.drive.drive(2.0)
    run(bot, 600)
    drift = bot.root.location.x - x
    print('after sliding: x drift %.2f, y %.2f' % (drift, bot.root.location.y))
    assert abs(drift) > 1.0, 'expected to slide along the wall, drifted %.2f' % drift
    assert 0.0 < bot.drive.slip < 1.0, 'sliding is partial progress: %s' % bot.drive.slip
    print('sliding OK')


def test_climbs_ramp():
    """A drivable slope is climbed, not treated as a wall."""
    x = next_lane()
    slope = math.radians(12)
    ramp = create_cube('RAMP', size=(12, 12, 0.06), location=(x, 10, 0))
    ramp.rotation_euler.x = slope
    ramp.location.z = 6 * math.sin(slope)
    bpy.context.view_layer.update()

    bot = robot_at(x)
    bot.enable_contact(level=True)
    bot.drive.drive(1.5)
    run(bot, 180)
    y0, z0 = bot.root.location.y, bot.root.location.z
    run(bot, 300)
    y1, z1 = bot.root.location.y, bot.root.location.z

    rise = (z1 - z0) / (y1 - y0)
    print('on ramp: rise/run %.4f (tan %.0f deg = %.4f), pitch %.2f deg'
          % (rise, math.degrees(slope), math.tan(slope),
             math.degrees(bot.root.rotation_euler.x)))
    assert not bot.drive.last_contact.blocked, 'a ramp is not a wall'
    ## the base follows the surface: rise over run is the slope itself
    assert close(rise, math.tan(slope), 0.02), (rise, math.tan(slope))
    ## and levelling pitches the body onto the slope
    assert close(bot.root.rotation_euler.x, slope, 0.02), bot.root.rotation_euler.x
    print('ramp climbing and levelling OK')


def test_overhang_is_not_ground():
    """Driving under something must not snap the robot up onto it."""
    x = next_lane()
    bot = robot_at(x)
    bot.enable_contact()
    ## a slab low enough to be within probe range, high enough to drive under
    create_cube('ROOF', size=(20, 8, 0.2), location=(x, 8, 1.0))
    bpy.context.view_layer.update()
    bot.drive.drive(2.0)
    run(bot, 400)
    print('under the overhang: y=%.2f z=%.3f' % (bot.root.location.y, bot.root.location.z))
    ## A downward probe cannot tell a floor from a ceiling; only the climb limit
    ## stops the robot teleporting onto the roof.
    assert close(bot.root.location.z, bot.wheel_radius * 1.5), bot.root.location.z
    assert bot.root.location.y > 8, 'should have driven under it'
    print('overhang rejected OK')


def test_tilt_is_clamped():
    """A wheel over a discontinuity must not flip the robot onto its back."""
    x = next_lane()
    bot = robot_at(x)
    limit = math.radians(30)
    contact = bot.enable_contact(level=True, max_tilt=limit)

    ## the clamp itself
    assert close(contact.clamp_tilt(math.radians(80)), limit), contact.clamp_tilt(math.radians(80))
    assert close(contact.clamp_tilt(math.radians(-80)), -limit)
    assert close(contact.clamp_tilt(math.radians(10)), math.radians(10))

    ## and in the situation that motivated it: the lip where a ramp meets the
    ## platform it feeds, where the contact points straddle a step rather than a
    ## slope and the raw angle across the robot approaches vertical
    slope = math.radians(10)
    ramp = create_cube('RAMP.J', size=(14, 14, 0.06), location=(x, 12, 0))
    ramp.rotation_euler.x = slope
    ramp.location.z = 7 * math.sin(slope)
    create_cube('PLAT.J', size=(14, 14, 0.06),
                location=(x, 26.5, 14 * math.sin(slope)))
    bpy.context.view_layer.update()

    bot.drive.drive(1.6)
    worst = 0.0
    for _ in range(900):
        bot.step(DT)
        worst = max(worst, abs(bot.root.rotation_euler.x), abs(bot.root.rotation_euler.y))
    print('worst tilt crossing the ramp/platform lip: %.1f deg (limit 30)'
          % math.degrees(worst))
    assert worst <= limit + 1e-6, math.degrees(worst)
    print('tilt clamp OK')


def test_contact_points_are_the_wheels():
    """Ground probes go where the robot actually touches."""
    x = next_lane()
    bot = robot_at(x)
    contact = bot.enable_contact()
    assert len(contact.contact_points) == len(bot.wheel_list) == 4
    xs = sorted(p[0] for p in contact.contact_points)
    assert close(xs[0], -bot.size[0] / 2), xs
    assert close(xs[-1], bot.size[0] / 2), xs
    print('contact points follow the wheels OK')


def test_cost():
    """Contact is a handful of ray casts, not a simulation step."""
    x = next_lane()
    bot = robot_at(x)
    bot.enable_contact()
    bot.drive.drive(1.0)
    run(bot, 5)
    t = time.time()
    run(bot, 500)
    each = (time.time() - t) / 500.0
    print('contact step cost: %.1f us' % (each * 1e6))
    assert each < 0.002, 'contact step took %.4fs' % each
    print('cost OK')


test_inertia()
test_slip_reports_lockup()
test_impact_takes_the_speed()
test_no_contact_is_unchanged()
test_settles_onto_ground()
test_wall_blocks_and_skids()
test_slide_along_wall()
test_climbs_ramp()
test_overhang_is_not_ground()
test_tilt_is_clamped()
test_contact_points_are_the_wheels()
test_cost()
print('contact test OK')
