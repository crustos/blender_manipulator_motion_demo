#!../headless.py
print('hello lidar test...')
import bpy, math

## Each test gets its own patch of ground. Robots left stacked at the origin see
## *each other* -- correctly, since self-filtering only masks a robot's own
## parts -- which quietly invalidates every range in the scan.
LANE = [0]
def next_lane(step=200.0):
    LANE[0] += step
    return LANE[0]

GROUND = create_cube('GROUND', size=(4000, 4000, 0.2), location=(0, 0, -0.1))
## Labelled at creation: the ray caster reads the *evaluated* depsgraph, so a
## pass_index assigned later is not visible to a scan until the view layer has
## been updated.
GROUND.pass_index = SEGMENT_CLASSES['ground']
bpy.context.view_layer.update()


def close(a, b, tol=1e-2):
    return abs(a - b) < tol


def wrapped(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def beam_index(scan, azimuth):
    return min(range(scan.beams), key=lambda i: abs(wrapped(scan.azimuths[i] - azimuth)))


def sensor_pos(lidar):
    bpy.context.view_layer.update()
    return lidar.mount.matrix_world.translation.copy()


def wall(name, x, y, size=(40, 0.2, 4), z=2, index=0):
    w = create_cube(name, size=size, location=(x, y, z))
    w.pass_index = index
    bpy.context.view_layer.update()
    return w


def robot_in_lane(x, **kw):
    bot = Robot(cameras='none', **kw)
    bot.root.location = (x, 0, 0.15)
    bpy.context.view_layer.update()
    return bot


def test_known_geometry():
    """Ranges are metres to the actual surface."""
    x = next_lane()
    bot = robot_in_lane(x, arms=[])
    lidar = bot.add_lidar()
    print(lidar)
    wall('WALL.N', x, 12)
    p = sensor_pos(lidar)
    scan = lidar.scan()

    expect = (12 - 0.1) - p.y          ## wall face, measured from the sensor
    got = scan.sector(-0.02, 0.02)
    print('forward: got %.3f expect %.3f' % (got, expect))
    assert close(got, expect), (got, expect)

    ## nothing behind us, and a miss is inf rather than 0 -- a 0 would read as
    ## an obstacle touching the sensor, the most dangerous way to be wrong
    assert scan.sector(math.pi - 0.02, -math.pi + 0.02) == NO_RETURN
    assert NO_RETURN == float('inf')
    assert all(r > 0 for r in scan.ranges), 'a miss must not read as zero range'
    return lidar, expect


def test_range_is_radial(lidar, d):
    """
    Range to a flat wall grows as d/cos(theta) across the sweep.

    This is exactly what a depth-render lidar gets wrong: Blender's Z pass is
    planar depth, so it reports the same value at every angle. Ray casting
    reports true slant range.
    """
    scan = lidar.scan()
    for degrees in (0, 15, 30, 45):
        theta = math.radians(degrees)
        expect = d / math.cos(theta)
        got = scan.range_at(beam_index(scan, theta))
        print('  %2d deg -> %7.3f   (expect %7.3f, planar depth would say %.3f)'
              % (degrees, got, expect, d))
        assert close(got, expect, 0.05), (degrees, got, expect)
    print('radial range OK')


def test_self_filter():
    """Beams pass through the robot's own parts, but not through the world."""
    x = next_lane()
    bot = robot_in_lane(x)               ## default arm: tall and close to the mast
    blind = bot.add_lidar(name='LIDAR.MASKED', self_filter=True)
    seeing = bot.add_lidar(name='LIDAR.RAW', self_filter=False)

    own = {SEGMENT_CLASSES['body'], SEGMENT_CLASSES['wheel'],
           SEGMENT_CLASSES['hub'], SEGMENT_CLASSES['arm']}
    masked = set(blind.scan().labels)
    raw = set(seeing.scan().labels)
    print('masked sees', sorted(masked), ' unmasked sees', sorted(raw))
    assert not (masked & own), 'self filter let the robot see itself: %s' % masked
    assert raw & own, 'unfiltered lidar should see the robot it is mounted on'

    ## Masking must not blind the sector: beams continue past the robot to
    ## whatever is behind it, rather than being discarded.
    down = bot.add_lidar(name='LIDAR.DOWN', channels=3, v_fov=math.radians(60))
    labels = set(down.scan().labels)
    print('downward lidar sees', sorted(labels))
    assert SEGMENT_CLASSES['ground'] in labels, 'beams did not reach the ground'
    assert not (labels & own), 'downward beams saw the robot they are mounted on'
    print('self filter OK')


def test_labels_and_channels():
    """Beams carry the class of what they hit, on every channel."""
    x = next_lane()
    bot = robot_in_lane(x, arms=[])
    lidar = bot.add_lidar(channels=5, v_fov=math.radians(40))
    assert lidar.rays == len(lidar.azimuths) * 5, lidar.rays
    assert len(lidar.elevations) == 5
    assert close(lidar.elevations[0], -math.radians(20)), lidar.elevations
    assert close(lidar.elevations[-1], math.radians(20)), lidar.elevations
    assert close(lidar.elevations[2], 0.0), lidar.elevations

    wall('TARGET.L', x, 8, size=(4, 0.2, 4), index=77)
    scan = lidar.scan()
    assert 77 in scan.labels, 'target not seen'
    i = scan.labels.index(77)
    assert scan.ranges[i] != NO_RETURN, 'labelled beam has no range'
    print('labels and channels OK: %d beams x %d channels' % (scan.beams, scan.channels))


def test_elevation_geometry():
    """Channels really do point at different heights."""
    x = next_lane()
    bot = robot_in_lane(x, arms=[])
    ## A wall that straddles the sensor height (mast top is ~0.70) but is low
    ## enough for a steeply raised beam to clear: level hits, angled up misses.
    wall('WALL.LOW', x, 5, size=(40, 0.2, 1.4), z=0.5, index=55)
    lidar = bot.add_lidar(channels=3, v_fov=math.radians(80))
    scan = lidar.scan()
    forward = beam_index(scan, 0.0)
    level = scan.range_at(forward, 1)
    up = scan.range_at(forward, 2)
    print('level beam %.3f, raised beam %s' % (level, up))
    assert level != NO_RETURN, 'level beam should hit the low wall'
    assert up == NO_RETURN or up > level, 'raised beam should clear the wall'
    print('elevation geometry OK')


def test_moves_with_robot():
    """The scan is taken from wherever the robot currently is."""
    x = next_lane()
    bot = robot_in_lane(x, arms=[])
    lidar = bot.add_lidar()
    wall('WALL.M', x, 20)

    before = lidar.scan().sector(-0.02, 0.02)
    bot.drive.drive(1.0)
    for _ in range(60):
        bot.step(1.0 / 60.0)
    bpy.context.view_layer.update()
    after = lidar.scan().sector(-0.02, 0.02)
    print('range before %.3f, after driving 1m %.3f' % (before, after))
    assert close(before - after, 1.0, 0.02), (before, after)
    print('scan follows the robot OK')


def test_rate():
    """Lidar is a fast sense: it defaults to every tick, unlike the cameras."""
    bot = robot_in_lane(next_lane(), arms=[])
    fast = bot.add_lidar()
    assert fast.interval == 1, fast.interval
    assert [t for t in range(6) if bot.scan_lidars(tick=t)] == [0, 1, 2, 3, 4, 5]

    slow = robot_in_lane(next_lane(), arms=[])
    slow.add_lidar(interval=3)
    fired = [t for t in range(10) if slow.scan_lidars(tick=t)]
    print('interval 3 fired on', fired)
    assert fired == [0, 3, 6, 9], fired
    ## the camera on the same robot stays on its own, much slower clock
    assert slow.camera_interval == CAMERA_INTERVAL
    assert slow.camera_interval > fast.interval
    print('rate OK')


def test_points():
    """Hit points are recoverable in sensor and world frames."""
    x = next_lane()
    bot = robot_in_lane(x, arms=[])
    lidar = bot.add_lidar()
    wall('WALL.P', x, 9)
    scan = lidar.scan()
    local = scan.points()
    world = scan.points(world=True)
    assert len(local) == len(world) == scan.hits, (len(local), scan.hits)

    ## world points land on the wall face; the sensor-frame copies are offset by
    ## the robot's position, which is the whole difference between the frames
    front = [w for w in world if abs(w.y - (9 - 0.1)) < 0.05]
    assert front, 'no world points landed on the wall face'
    assert all(abs(w.x - x) < 40 for w in front), 'world points in the wrong lane'
    assert all(abs(p.x) < 40 for p in local)
    print('points OK: %d returns, %d on the wall face' % (len(world), len(front)))


lidar, distance = test_known_geometry()
test_range_is_radial(lidar, distance)
test_self_filter()
test_labels_and_channels()
test_elevation_geometry()
test_moves_with_robot()
test_rate()
test_points()
print('lidar test OK')
