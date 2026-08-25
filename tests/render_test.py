#!../headless.py
print('hello render test...')
from PIL import Image


def test1():
    """The original smoke test: every camera on every robot writes a PNG."""
    from random import uniform
    robots = [Robot(), Robot()]
    for r in robots:
        r.root.location.x = uniform(-5,5)
        r.root.location.y = uniform(-5,5)
        r.root.rotation_euler.z = uniform(-3,3)

    for r in robots:
        pngs = r.render_cameras()
        print(pngs)
        for png in pngs:
            assert os.path.isfile(png)
    return robots


def test_resolution(bot):
    """Renders default to a tensor-sized image, not something to look at."""
    png = bot.render_cameras()[0]
    img = Image.open(png)
    print('default render size:', img.size)
    assert img.size == (64, 32), img.size
    ## an explicit size still wins, for anything meant to be viewed
    png = bot.render_cameras(resolution=(96, 48))[0]
    assert Image.open(png).size == (96, 48)


def test_camera_sets():
    """A robot can carry one forward camera instead of the full mast."""
    simple = Robot(arms=[], cameras='front')
    assert list(simple.cameras) == ['front'], simple.cameras
    ## 'front' must actually face forward (+Y): no yaw applied
    assert abs(simple.cameras['front'].rotation_euler.z) < 1e-9

    full = Robot(arms=[], cameras='all')
    assert len(full.cameras) == 4, full.cameras

    picked = Robot(arms=[], cameras=['front', 'back'])
    assert sorted(picked.cameras) == ['back', 'front'], picked.cameras

    blind = Robot(arms=[], cameras='none')
    assert blind.cameras == {}, blind.cameras
    assert blind.render_cameras() == []
    ## a blind robot is never due, so a control loop can call sample_cameras()
    ## unconditionally without special-casing it
    assert blind.sample_cameras() == []

    ## one camera is a quarter of the render work of four
    assert len(simple.render_cameras()) == 1
    assert len(full.render_cameras()) == 4
    print('camera sets OK')


def test_capture_rate():
    """
    Vision runs on a slower clock than the control loop.

    This is the behaviour the real device needs -- the base loop reacts at full
    rate on cheap sensors and only peeks at the cameras periodically -- and it is
    also what keeps the test suite fast.
    """
    bot = Robot(arms=[], cameras='front', camera_interval=5)
    got = [t for t in range(12) if bot.sample_cameras(tick=t)]
    print('captured on ticks:', got, 'of 12')
    ## the first tick always captures: a controller needs an initial view
    assert got == [0, 5, 10], got
    assert bot.captures == 3, bot.captures

    ## interval 0 disables capture entirely
    off = Robot(arms=[], cameras='front', camera_interval=0)
    assert [t for t in range(6) if off.sample_cameras(tick=t)] == []
    ## but force still works, for an explicit one-off grab
    assert off.sample_cameras(tick=3, force=True), 'force should override'

    ## interval 1 captures every tick
    every = Robot(arms=[], cameras='front', camera_interval=1)
    assert len([t for t in range(4) if every.sample_cameras(tick=t)]) == 4

    ## the default is slow enough that a short run barely renders at all
    lazy = Robot(arms=[], cameras='front')
    hits = len([t for t in range(12) if lazy.sample_cameras(tick=t)])
    print('default interval %d -> %d captures in 12 ticks' % (lazy.camera_interval, hits))
    assert hits == 1, hits

    ## render_cameras() stays unconditional -- an explicit call always renders
    assert lazy.render_cameras(), 'render_cameras should ignore the interval'
    print('capture rate OK')


def test_decimation():
    """Heavy CAD meshes are cut down on load."""
    def verts(objs):
        return [len(o.data.vertices) for o in objs if o.type == 'MESH']

    raw = load_blend_objects(DEFAULT_ARM, decimate=False)
    lean = load_blend_objects(DEFAULT_ARM)
    before, after = sum(verts(raw)), sum(verts(lean))
    print('irb120 vertices: %d -> %d (%.1fx lighter)' % (before, after, before / float(after)))
    assert before > 40000, 'expected the raw CAD mesh to be heavy: %d' % before
    assert after < before / 4, 'decimation barely helped: %d -> %d' % (before, after)

    ## nothing is left above the threshold we promised to bring down
    worst = max(verts(lean))
    print('worst remaining mesh:', worst)
    assert worst <= DECIMATE_THRESHOLD, worst

    ## meshes already under the threshold are left exactly alone
    untouched = sorted(n for n in verts(raw) if n <= DECIMATE_THRESHOLD)
    assert untouched, 'expected at least one already-small mesh'
    kept = sorted(verts(lean))
    for n in untouched:
        assert n in kept, 'a small mesh was decimated anyway: %d' % n

    ## the arm still works afterwards: decimation must touch the meshes hanging
    ## off the armature, never the armature itself
    bot = Robot(cameras='none')
    arm = bot.arms[0]
    arm.set_angles([0.2] * len(arm.joints))
    assert all(abs(a - 0.2) < 1e-3 for a in arm.angles), arm.angles
    print('decimation OK')


def test_engine():
    """Engine is switchable by short name."""
    assert set_render_engine('workbench') == 'BLENDER_WORKBENCH'
    got = set_render_engine('eevee')
    assert got.startswith('BLENDER_EEVEE'), got
    print('engine switching OK')


## test1 and the resolution check run on the scene's real engine (EEVEE), so the
## production render path stays covered. Everything after them is about counts,
## scheduling and geometry rather than image quality, and EEVEE costs a fixed
## ~2.2s per render here regardless of resolution -- so they run on Workbench,
## which is ~29x faster and just as good at proving a camera produced pixels.
robots = test1()
test_resolution(robots[0])
set_render_engine('workbench')
test_camera_sets()
test_capture_rate()
test_decimation()
test_engine()
print('render test OK')
