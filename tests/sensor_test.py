#!../headless.py
print('hello sensor test...')
import bpy

OUT = '/tmp/sensors'
os.system('rm -rf %s; mkdir -p %s' % (OUT, OUT))

## Count actual renders, so "one render per camera" is asserted rather than
## assumed. This is the whole premise of the multi-pass design.
RENDERS = [0]
def _count(scene, depsgraph=None):
    RENDERS[0] += 1
bpy.app.handlers.render_post.append(_count)


def close(a, b, tol=1e-2):
    return abs(a - b) < tol


def test_one_render_per_camera():
    """N passes from one render, not one render per pass."""
    bot = Robot(arms=[], cameras='all', out_dir=OUT,
                passes=('rgb', 'depth', 'segmentation'))
    RENDERS[0] = 0
    got = bot.capture(frame=1, resolution=(64, 32))
    print('cameras:', len(bot.cameras), 'passes: 3, renders:', RENDERS[0])
    assert len(got) == 4, got
    assert RENDERS[0] == 4, 'expected 1 render per camera, got %d' % RENDERS[0]
    for cam, passes in got.items():
        assert sorted(passes) == ['depth', 'rgb', 'segmentation'], passes
        for name, path in passes.items():
            assert os.path.isfile(path), '%s %s missing: %s' % (cam, name, path)

    ## asking for one pass costs the same number of renders as asking for three
    one = Robot(arms=[], cameras='all', out_dir=OUT, passes=('rgb',))
    RENDERS[0] = 0
    one.capture(frame=1, resolution=(64, 32))
    assert RENDERS[0] == 4, RENDERS[0]
    print('one render per camera OK')


def test_segmentation_labels():
    """The segmentation pass carries the semantic class of each part."""
    bot = Robot(cameras='front', out_dir=OUT, passes=('segmentation',))
    bot.cameras['front'].location.z = 0.3
    got = bot.capture(frame=2, resolution=(64, 32))
    w, h, vals = read_pass(got['front']['segmentation'])
    labels = sorted(set(int(round(v)) for v in vals))
    print('segmentation labels present:', labels)
    ## values must be the class indices exactly -- not colours, not normalised
    for v in labels:
        assert v in SEGMENT_CLASSES.values(), 'unexpected label %s' % v
    assert SEGMENT_CLASSES['background'] in labels, 'nothing read as background'
    assert SEGMENT_CLASSES['body'] in labels, 'the robot body should be visible'
    ## the robot has an arm, and it should be distinguishable from the body
    assert SEGMENT_CLASSES['arm'] in labels, 'arm not labelled'
    print('segmentation labels OK')


def test_instance_offsets():
    """Two robots can be told apart by offsetting their labels."""
    a = Robot(arms=[], cameras='none', out_dir=OUT)
    b = Robot(arms=[], cameras='none', out_dir=OUT)
    a.label_parts(offset=0)
    b.label_parts(offset=10)
    assert a.body.pass_index == SEGMENT_CLASSES['body']
    assert b.body.pass_index == SEGMENT_CLASSES['body'] + 10
    ## class structure survives the offset: a wheel is still a wheel
    assert b.wheels[0].pass_index - b.body.pass_index == \
        SEGMENT_CLASSES['wheel'] - SEGMENT_CLASSES['body']
    print('instance offsets OK')


def test_depth_is_metric():
    """
    Depth is in metres, and agrees with segmentation about where things are.

    Using segmentation to pick out the target's pixels tests both passes at once
    and makes the check independent of exactly what else is in frame.
    """
    bot = Robot(arms=[], cameras='front', out_dir=OUT,
                passes=('depth', 'segmentation'))
    cam = bot.cameras['front']
    cam.location.z = 0.3
    bpy.context.view_layer.update()
    cam_y = cam.matrix_world.translation.y

    target = create_cube('TARGET', size=(1.0, 0.2, 1.0))
    MARK = 42
    target.pass_index = MARK

    def depth_of_target(at_y, frame):
        target.location = (0.0, at_y, 0.3)
        bpy.context.view_layer.update()
        got = bot.capture(frame=frame, resolution=(64, 32))
        _w, _h, seg = read_pass(got['front']['segmentation'])
        _w, _h, dep = read_pass(got['front']['depth'])
        hits = [d for s, d in zip(seg, dep) if int(round(s)) == MARK]
        assert hits, 'target not visible in the segmentation pass'
        return sum(hits) / len(hits), len(hits)

    near, near_px = depth_of_target(cam_y + 2.0, 3)
    far, far_px = depth_of_target(cam_y + 4.0, 4)
    print('target at 2m -> depth %.3f (%d px);  at 4m -> depth %.3f (%d px)'
          % (near, near_px, far, far_px))
    ## metric: the reported depth is the actual distance, not a normalised value
    assert close(near, 2.0, 0.15), near
    assert close(far, 4.0, 0.15), far
    ## and moving it 2m further away moves the depth by 2m
    assert close(far - near, 2.0, 0.15), (near, far)
    ## a more distant object of the same size covers fewer pixels
    assert far_px < near_px, (near_px, far_px)
    print('metric depth OK')


def test_engine_and_isolation():
    """Segmentation forces Cycles, and the rig leaves the scene alone after."""
    set_render_engine('workbench')
    bot = Robot(arms=[], cameras='front', out_dir=OUT, passes=('rgb', 'segmentation'))
    bot.capture(frame=5, resolution=(64, 32))
    assert bpy.context.scene.render.engine == 'CYCLES', bpy.context.scene.render.engine
    ## cycles must be configured for speed, not left at Blender's 4096 default
    assert bpy.context.scene.cycles.samples <= 64, bpy.context.scene.cycles.samples

    ## the compositor is dormant between captures, so an ordinary render does
    ## not quietly spray pass files into the output directory
    assert bpy.context.scene.use_nodes is False, 'compositor left enabled'
    before = set(os.listdir(OUT))
    quick_render(bot.cameras['front'], output_path='/tmp/plain_render.png')
    after = set(os.listdir(OUT))
    assert before == after, 'a plain render wrote pass files: %s' % (after - before)
    print('engine switching and isolation OK')


def test_rate_limited_capture():
    """sample() is the rate-limited form of capture()."""
    bot = Robot(arms=[], cameras='front', out_dir=OUT,
                passes=('rgb', 'depth'), camera_interval=4)
    hits = [t for t in range(10) if bot.sample(tick=t, resolution=(64, 32))]
    print('multi-pass captures on ticks:', hits)
    assert hits == [0, 4, 8], hits
    assert bot.captures == 3, bot.captures
    ## and it returns the same shape capture() does
    got = bot.sample(tick=99, resolution=(64, 32))
    assert sorted(got['front']) == ['depth', 'rgb'], got
    print('rate limited capture OK')


test_one_render_per_camera()
test_segmentation_labels()
test_instance_offsets()
test_depth_is_metric()
test_engine_and_isolation()
test_rate_limited_capture()
print('sensor test OK')
