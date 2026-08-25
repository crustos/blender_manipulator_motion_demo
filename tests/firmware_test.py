#!../headless.py
print('hello firmware test...')
import bpy, math, os, time
import firmware

## Optional dependency: skip rather than fail when crust is not cloned beside
## robotsim. Reported, not raised -- a machine without the checkout should get a
## clear message, not a traceback.
if not firmware.available():
    print('SKIP firmware test: %s' % firmware.why_unavailable())
    print('firmware test OK (skipped)')
    raise SystemExit(0)

print('crust:     %s' % firmware.crust_root())
print('armulator: %s' % (firmware.armulator_root() or 'not cloned (offline path unavailable)'))

## HERE is robotsim's own module directory, provided by robotsim.py -- tests
## are exec'd into its globals, so __file__ here would be robotsim.py rather
## than this file.
ROOT = HERE
DRIVE_C = os.path.join(ROOT, 'boards', 'drive_node.c')
PID_CPP = os.path.join(ROOT, 'boards', 'pid_node.cpp')

DT = 1.0 / 60.0
GROUND = create_cube('GROUND', size=(8000, 8000, 0.2), location=(0, 0, -0.1))
bpy.context.view_layer.update()

LANE = [0]
def next_lane(step=200.0):
    LANE[0] += step
    return LANE[0]


def close(a, b, tol=1e-2):
    return abs(a - b) < tol


def robot_with(source=DRIVE_C, target=4000, mode='wheel', accel=2.0, contact=True, **kw):
    x = next_lane()
    bot = Robot(arms=[], cameras='none', max_accel=accel)
    bot.root.location = (x, 0, 0.4)
    if contact:
        bot.enable_contact()
    bpy.context.view_layer.update()
    fw = bot.attach_firmware(source, target=target, source_mode=mode, **kw)
    return bot, fw, x


def run(bot, ticks):
    for _ in range(ticks):
        bot.step(DT)


def test_build():
    """C builds; C++ is lowered through the subset front end first."""
    so = firmware.build(DRIVE_C)
    assert os.path.isfile(so), so

    ## cached: a second build of an unchanged source must not rerun gcc
    t = time.time()
    again = firmware.build(DRIVE_C)
    assert again == so
    assert time.time() - t < 0.25, 'cache miss: rebuilt an unchanged source'

    cpp = firmware.build(PID_CPP)
    assert os.path.isfile(cpp), cpp
    ## the lowered C is plain C with the class flattened to Type_method
    lowered = '/tmp/robotsim-fw-pid_node.c'
    assert os.path.isfile(lowered), lowered
    text = open(lowered).read()
    ## Positive evidence of the lowering, rather than grepping for the absence
    ## of "class" -- the source comments say the word, so a naive negative test
    ## fails on the documentation instead of the code.
    assert 'struct Pid' in text, 'the class did not become a struct'
    assert 'Pid_step(Pid *this' in text, 'the method did not become Type_method'
    print('build and C++ lowering OK')


def test_clock_does_not_drift():
    """
    The board and the plant share one clock.

    dt is 1/60 s against a 19.2 MHz counter, so a step is a whole number of
    ticks here but will not be for every dt. Truncating each step would drift
    the board's clock away from the plant's silently and forever, so the
    fraction is carried.
    """
    bot, fw, _x = robot_with()
    period = 1.0 / fw.board.loop_hz

    ## Lag must stay bounded rather than accumulating: time is granted in whole
    ## loop periods, so the board can trail the plant by up to one period, but
    ## the remainder is carried and the gap must not grow with the run.
    run(bot, 600)
    early = abs(fw.board.elapsed - 600 * DT)
    run(bot, 600)
    late = abs(fw.board.elapsed - 1200 * DT)
    print('board lag after 10s: %.6fs, after 20s: %.6fs (one period = %.6fs)'
          % (early, late, period))
    assert early <= period, early
    assert late <= period, late
    assert late <= early + 1e-9, 'lag grew: %.6f -> %.6f -- the clock is drifting' % (early, late)

    ## and with a dt that is not a whole number of counter ticks
    bot2, fw2, _x = robot_with()
    odd = 1.0 / 7.0
    for _ in range(70):
        bot2.step(odd)
    print('odd dt: board %.4fs vs plant %.4fs' % (fw2.board.elapsed, 70 * odd))
    assert abs(fw2.board.elapsed - 70 * odd) <= period, fw2.board.elapsed
    print('clock OK')


def test_closed_loop_converges():
    """Real C, compiled and run, drives the simulated robot to a setpoint."""
    bot, fw, x = robot_with(mode='body', target=4000)
    run(bot, 900)
    print('body odometry: y=%.3f encoder=%d' % (bot.root.location.y, fw.board.encoder))
    assert close(bot.root.location.y, 4.0, 0.05), bot.root.location.y
    ## the firmware actually ran: it printed, and it drove
    assert fw.board.lines, 'no console output'
    assert '[drive]' in fw.board.console
    assert fw.board.steps > 800
    print('closed loop OK')


def test_cpp_controls_the_same_plant():
    """The C++ subset build controls the same robot the same way."""
    bot, fw, _x = robot_with(source=PID_CPP, mode='body', target=4000)
    run(bot, 900)
    print('c++ loop: y=%.3f encoder=%d' % (bot.root.location.y, fw.board.encoder))
    assert '[pid]' in fw.board.console, fw.board.console[:200]
    assert close(bot.root.location.y, 4.0, 0.05), bot.root.location.y
    print('C++ firmware OK')


def test_wheel_encoder_accumulates_slip():
    """
    A shaft encoder measures the wheels, not the ground.

    This is the default because it is what hardware does. The wheels reach the
    commanded speed before the body does, so the count runs ahead of the truth
    and the firmware's dead reckoning is wrong by the accumulated slip -- which
    is a real error a Python control loop never has to face.
    """
    bot, fw, _x = robot_with(mode='wheel', target=4000, accel=2.0)
    run(bot, 900)
    believed = fw.board.encoder / 1000.0
    actual = bot.root.location.y
    print('wheel encoder: firmware believes %.3f m, actually %.3f m' % (believed, actual))
    assert close(believed, 4.0, 0.05), believed
    assert actual < believed - 0.5, 'expected dead-reckoning error, got %.3f vs %.3f' % (actual, believed)
    print('slip accumulation OK')


def test_blocked_robot_deceives_its_firmware():
    """Held against a wall, the wheels keep turning and the count keeps rising."""
    x = next_lane()
    bot = Robot(arms=[], cameras='none', max_accel=2.0)
    bot.root.location = (x, 0, 0.4)
    bot.enable_contact()
    create_cube('WALL.F', size=(20, 0.4, 2), location=(x, 2.0, 1.0))
    bpy.context.view_layer.update()
    fw = bot.attach_firmware(DRIVE_C, target=4000, source_mode='wheel')
    run(bot, 900)
    believed = fw.board.encoder / 1000.0
    actual = bot.root.location.y
    print('walled: firmware believes %.3f m, actually %.3f m' % (believed, actual))
    assert actual < 2.0, 'should have been stopped by the wall: %.3f' % actual
    assert believed > 3.0, 'wheels should have kept counting: %.3f' % believed
    print('blocked-robot deception OK')


def test_stuck_encoder_runs_away():
    """
    A sensor fault the firmware cannot see, and its consequence.

    fault_encoder_stuck is the nasty one: the shaft turns and the sensor keeps
    reporting a plausible value that never changes, so the controller sees a
    constant error and drives at full duty forever.
    """
    bot, fw, _x = robot_with(mode='wheel', target=4000)
    fw.board.fault_encoder_stuck(True)
    run(bot, 900)
    print('stuck encoder: encoder=%d duty=%d actual y=%.2f'
          % (fw.board.encoder, fw.board.motor_duty, bot.root.location.y))
    assert fw.board.encoder == 0, fw.board.encoder
    assert fw.board.motor_duty >= 900, 'should still be commanding full duty'
    assert bot.root.location.y > 10.0, 'should have run away past the setpoint'
    print('stuck encoder OK')


def test_module_is_bpy_free():
    """firmware.py must import without Blender, like drive.py."""
    import sys
    source = open(os.path.join(ROOT, 'firmware.py')).read()
    assert 'import bpy' not in source, 'firmware.py should not import bpy'
    print('bpy-free OK')


test_build()
test_clock_does_not_drift()
test_closed_loop_converges()
test_cpp_controls_the_same_plant()
test_wheel_encoder_accumulates_slip()
test_blocked_robot_deceives_its_firmware()
test_stuck_encoder_runs_away()
test_module_is_bpy_free()
print('firmware test OK')
