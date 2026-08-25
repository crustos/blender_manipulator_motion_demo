#!../headless.py
print('hello telemetry test...')
import bpy, math, os
import telemetry
from telemetry import Telemetry, span_limits

## span_limits and the readers are plain arithmetic and are checked even when
## matplotlib is missing; only drawing needs the library.
assert span_limits([0.15] * 10) == (0.149, 0.151), span_limits([0.15] * 10)
assert span_limits([0.0, 5.0]) is None
assert span_limits([]) is None
assert telemetry._num(float('inf')) == 0.0
assert telemetry._num(None) == 0.0
assert telemetry._num('nope', 7.0) == 7.0
print('span limits and coercion OK')

OUT = '/tmp/robotsim-telemetry'
os.system('rm -rf %s; mkdir -p %s' % (OUT, OUT))
DT = 1.0 / 60.0
create_cube('GROUND', size=(4000, 4000, 0.2), location=(0, 0, -0.1))
bpy.context.view_layer.update()

LANE = [0]
def next_lane(step=150.0):
    LANE[0] += step
    return LANE[0]


def close(a, b, tol=1e-6):
    return abs(a - b) < tol


def moving_robot():
    x = next_lane()
    bot = Robot(arms=[], cameras='none', max_accel=2.0)
    bot.root.location = (x, 0, 0.4)
    bot.enable_contact()
    lidar = bot.add_lidar(h_resolution=math.radians(6.0), range_max=25)
    bpy.context.view_layer.update()
    return bot, lidar, x


def test_records_what_happened():
    """Sampling is pull-based: one call per channel, per tick."""
    bot, lidar, x = moving_robot()
    tel = Telemetry(name='basic')
    tel.watch_robot(bot, prefix='base')
    tel.watch('custom', lambda: 42.0, group='custom')

    bot.drive.drive(1.0)
    for i in range(120):
        bot.step(DT)
        tel.sample(i * DT)

    assert len(tel.times) == 120, len(tel.times)
    for channel in tel.channels:
        assert len(channel.values) == 120, (channel.name, len(channel.values))
    assert all(v == 42.0 for v in tel.series('custom'))

    ## the recorded values are the robot's, not a copy that drifted
    assert close(tel.series('base y')[-1], bot.root.location.y, 1e-4)
    assert close(tel.series('base speed')[-1], bot.drive.v, 1e-4)
    ## and the robot actually moved, so the series is not trivially flat
    assert tel.series('base y')[-1] > tel.series('base y')[0] + 0.5
    print('recording OK: %d samples x %d channels' % (len(tel.times), len(tel.channels)))


def test_groups_and_order():
    """Channels sharing a group share a subplot, in declaration order."""
    bot, lidar, _x = moving_robot()
    tel = Telemetry()
    tel.watch_robot(bot, prefix='base')
    groups = tel.groups
    print('groups:', groups)
    assert groups[0] == 'position', groups
    assert 'slip' in groups and 'contact' in groups
    ## x and y share one plot; z is its own
    assert len([c for c in tel.channels if c.group == 'position']) == 2
    assert len([c for c in tel.channels if c.group == 'height']) == 1
    try:
        tel.series('nope')
        raise AssertionError('unknown channel should raise')
    except KeyError:
        pass
    print('grouping OK')


def test_infinities_are_plottable():
    """
    A lidar that saw nothing reports inf, and inf cannot be drawn.

    Reported as the sensor's own maximum instead, because rescaling the whole
    axis to infinity would hide every real reading on it.
    """
    bot, lidar, x = moving_robot()
    tel = Telemetry()
    tel.watch_lidar(lidar, prefix='lidar', sectors={'ahead': (-0.3, 0.3)})

    ## nothing scanned yet, and nothing in range once it has
    tel.sample(0.0)
    assert tel.series('lidar nearest')[0] == lidar.range_max, tel.series('lidar nearest')
    bot.scan_lidars(tick=0)
    tel.sample(1.0)
    for value in tel.series('lidar nearest') + tel.series('lidar ahead'):
        assert value == value and abs(value) != float('inf'), value

    ## and a real return is recorded as itself
    create_cube('TGT', size=(20, 0.4, 3), location=(x, 8.0, 1.5))
    bpy.context.view_layer.update()
    bot.scan_lidars(tick=1, force=True)
    tel.sample(2.0)
    nearest = tel.series('lidar nearest')[-1]
    print('lidar nearest with a wall at 8m: %.2f' % nearest)
    assert 0 < nearest < lidar.range_max, nearest
    print('infinity handling OK')


def test_csv_needs_no_matplotlib():
    """Getting the data out must not depend on being able to draw it."""
    bot, lidar, _x = moving_robot()
    tel = Telemetry()
    tel.watch('a', lambda: 1.0)
    tel.watch('b', lambda: 2.0)
    for i in range(5):
        tel.sample(i * 0.5)
    path = tel.to_csv(os.path.join(OUT, 'run.csv'))
    rows = open(path).read().strip().split('\n')
    print('csv:', rows[0], '|', rows[1])
    assert rows[0] == 't,a,b', rows[0]
    assert len(rows) == 6, rows
    assert float(rows[-1].split(',')[0]) == 2.0
    print('csv OK')


def test_plot():
    """The panel draws, with every group and the firmware console."""
    if not telemetry.available():
        print('SKIP plotting: %s' % telemetry.why_unavailable())
        return
    from PIL import Image

    bot, lidar, _x = moving_robot()
    tel = Telemetry(name='panel test')
    tel.watch_robot(bot, prefix='base')
    tel.watch_lidar(lidar, prefix='lidar')

    bot.drive.drive(1.0, 0.3)
    for i in range(90):
        bot.step(DT)
        bot.scan_lidars(tick=i)
        tel.sample(i * DT)
    tel.mark('halfway', 45 * DT)
    assert tel.events == [('halfway', 45 * DT)] or tel.events[0][1] == 'halfway'

    path = tel.plot(os.path.join(OUT, 'panel.png'), title='test panel')
    assert os.path.isfile(path)
    img = Image.open(path)
    print('panel: %dx%d, %d bytes' % (img.width, img.height, os.path.getsize(path)))
    assert img.width > 400 and img.height > 300, img.size
    ## one row per group, so more groups must make a taller image
    lean = Telemetry()
    lean.watch('only', lambda: 1.0)
    for i in range(10):
        lean.sample(i)
    small = Image.open(lean.plot(os.path.join(OUT, 'small.png')))
    assert small.height < img.height, (small.height, img.height)
    print('plot OK')


def test_plot_refuses_an_empty_run():
    """Nothing recorded is a mistake worth naming, not a blank image."""
    if not telemetry.available():
        return
    tel = Telemetry()
    tel.watch('a', lambda: 1.0)
    try:
        tel.plot(os.path.join(OUT, 'empty.png'))
        raise AssertionError('should have refused to plot nothing')
    except RuntimeError as e:
        print('refused as expected: %s' % e)
    print('empty run OK')


def test_compose():
    """A render and its panel stack into one image."""
    if not telemetry.available():
        return
    from PIL import Image
    set_render_engine('workbench')
    bot, lidar, _x = moving_robot()
    tel = Telemetry()
    tel.watch_robot(bot, prefix='base')
    for i in range(30):
        bot.step(DT)
        tel.sample(i * DT)
    panel = tel.plot(os.path.join(OUT, 'c-panel.png'))

    cam = create_camera('telcam')
    cam.location = (_x + 6, -6, 5)
    cam.rotation_euler = (math.radians(62), 0, math.radians(45))
    render = quick_render(cam, resolution_x=320, resolution_y=120,
                          output_path=os.path.join(OUT, 'c-render.png'))
    out = telemetry.compose(render, panel, os.path.join(OUT, 'combined.png'))
    a, b, c = (Image.open(p) for p in (render, panel, out))
    print('composed %s + %s -> %s' % (a.size, b.size, c.size))
    assert c.width == max(a.width, b.width)
    ## the render is scaled to the panel's width, so height is the sum plus gap
    assert c.height > b.height, c.size
    print('compose OK')


def test_module_is_bpy_free():
    source = open(os.path.join(HERE, 'telemetry.py')).read()
    assert 'import bpy' not in source, 'telemetry.py should not import bpy'
    print('bpy-free OK')


test_records_what_happened()
test_groups_and_order()
test_infinities_are_plottable()
test_csv_needs_no_matplotlib()
test_plot()
test_plot_refuses_an_empty_run()
test_compose()
test_module_is_bpy_free()
print('telemetry test OK')
