#!../headless.py
print('hello drive test...')
import math

DT = 1.0 / 60.0

def run(bot, secs, dt=DT):
    for _ in range(int(round(secs / dt))):
        bot.step(dt)

def close(a, b, tol=1e-3):
    return abs(a - b) < tol

## ---------------------------------------------------------------- diff drive
bot = Robot(arms=[])
assert bot.drive, 'robot has no drive model'
assert len(bot.wheel_map) == 4, 'expected 4 named wheels, got %s' % list(bot.wheel_map)
print('wheels:', list(bot.wheel_map))

start_z = bot.root.location.z

## 1. straight ahead is +Y (matches the 'front' camera and FRONT.HUB)
bot.drive.drive(1.0)
run(bot, 2.0)
print('after 2s at 1m/s:', tuple(bot.root.location))
assert close(bot.root.location.y, 2.0), bot.root.location.y
assert close(bot.root.location.x, 0.0), bot.root.location.x
assert close(bot.root.location.z, start_z), 'base drifted vertically'

## 2. wheels actually rolled, and by the right amount
for name, wheel in bot.wheel_map.items():
    turns = wheel.rotation_euler.x
    expect = -2.0 / bot.wheel_radius
    print('   %-14s roll %+.3f rad (expect %+.3f)' % (name, turns, expect))
    assert close(turns, expect, 1e-2), name

## 3. spin on the spot: no translation, exact yaw
bot.stop()
here = tuple(bot.root.location)
bot.drive.drive(0.0, math.pi / 2)
run(bot, 1.0)
print('yaw after 1s at pi/2 rad/s:', bot.root.rotation_euler.z)
assert close(bot.root.rotation_euler.z, math.pi / 2), bot.root.rotation_euler.z
assert close(bot.root.location.x, here[0]) and close(bot.root.location.y, here[1]), \
    'spin in place translated the base'

## 4. after a 90 degree left turn, forward is -X
bot.drive.drive(1.0, 0.0)
run(bot, 1.0)
print('after turning left then driving 1m:', tuple(bot.root.location))
assert close(bot.root.location.x, here[0] - 1.0, 1e-2), bot.root.location.x

## 5. a closed circle returns to where it started
bot2 = Robot(arms=[])
bot2.drive.drive(1.0, 1.0)          # radius 1m
run(bot2, 2 * math.pi, 1.0 / 240.0)
print('after one full circle:', tuple(bot2.root.location))
assert close(bot2.root.location.x, 0.0, 0.02), bot2.root.location.x
assert close(bot2.root.location.y, 0.0, 0.02), bot2.root.location.y

## ---------------------------------------------------------------- ackermann
car = Robot(arms=[], drive='ackermann')
car.drive.drive(1.0, math.radians(20))
v, w = car.drive.twist()
print('ackermann twist:', v, w, 'radius', car.drive.turn_radius)
assert close(w, math.tan(math.radians(20)) / car.drive.wheelbase, 1e-9)

run(car, 1.0)
left, right = car.drive.steer_angles()
print('steer angles L/R:', left, right)
assert left > right > 0, 'left turn: left wheel should be the inner, sharper one'
for name, wheel in car.wheel_map.items():
    if '.FRONT' in name:
        assert abs(wheel.rotation_euler.z) > 0, '%s did not steer' % name
    else:
        assert close(wheel.rotation_euler.z, 0.0), '%s should not steer' % name

## a car cannot turn on the spot
car.drive.drive(0.0, math.radians(30))
v, w = car.drive.twist()
assert close(w, 0.0), 'ackermann yaw rate must be zero at zero speed'

## ---------------------------------------------------------------- sim clock
RobotSim.dt = DT
t0 = RobotSim.time

@RobotSim
def callback(dt):
    ## callbacks may take dt or take nothing; both work
    assert close(dt, DT), dt
    if RobotSim.ticks >= 5:
        RobotSim.stop()

for _ in range(6):
    RobotSim.update()
print('sim time advanced:', RobotSim.time - t0, 'over', RobotSim.ticks, 'ticks')
assert close(RobotSim.time - t0, RobotSim.ticks * DT)

print('drive test OK')
