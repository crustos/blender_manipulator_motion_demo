#!../headless.py
print('hello joint control test...')
from random import uniform

bot = Robot()
assert bot.arms, 'no arms were built'

for arm in bot.arms:
    print(arm, arm.names)
    assert arm.joints, 'arm has no joints'

    ## 1. reading works while IK is still driving the chain
    start = arm.angles
    print('start angles:', [round(a, 3) for a in start])
    assert len(start) == len(arm.joints)

    ## 2. axis inference and limits
    for j in arm.joints:
        print('   ', j.bone_name, 'axis:', j.axis, 'limits:', j.limits)

    ## 3. FK: commanding joints must actually move them.
    ##    set_angles() switches IK off, otherwise the solver would win.
    target = []
    for j in arm.joints:
        lo, hi = j.limits
        if lo is not None and hi is not None:
            target.append(uniform(lo, hi))
        else:
            target.append(uniform(-0.3, 0.3))
    arm.set_angles(target)
    assert arm.ik_enabled is False, 'set_angles should have muted IK'

    reached = arm.angles
    print('target :', [round(a, 3) for a in target])
    print('reached:', [round(a, 3) for a in reached])
    for want, got in zip(target, reached):
        assert abs(want - got) < 1e-3, 'joint did not reach commanded angle: %s vs %s' % (want, got)

    ## 4. home() returns to rest
    arm.home()
    for a in arm.angles:
        assert abs(a) < 1e-3, 'home() left a joint at %s' % a

    ## 5. IK: moving the tool tip re-enables the solver and changes the pose
    before = arm.angles
    tip = arm.tip
    arm.set_tip(location=(tip.location.x, tip.location.y + 0.1, tip.location.z + 0.1))
    assert arm.ik_enabled is True, 'set_tip should have re-enabled IK'
    after = arm.angles
    print('after ik:', [round(a, 3) for a in after])
    assert any(abs(b - a) > 1e-4 for b, a in zip(before, after)), \
        'tool tip moved but no joint responded -- is the IK constraint targeting it?'

    ## 6. serialisable state
    st = arm.state()
    print(st)
    assert len(st['joint_names']) == len(st['position'])

print('joint control test OK')
