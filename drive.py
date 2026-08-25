"""
Kinematic base motion for wheeled robots.

These are *kinematic* models: they integrate a velocity command into a pose and
spin the wheel meshes to match. There is no mass, no traction, no slip and no
contact. That is deliberate -- it is the right level of fidelity for camera-in-
the-loop work and it keeps the door open for an external physics engine later,
which would replace `step()` and leave the command interface unchanged.

Deliberately free of `bpy`: these classes only read and write `.location` and
`.rotation_euler` on whatever objects they are handed, so the maths is testable
outside Blender.

FRAME CONVENTION (matches how robotsim.Robot is built):
    +Y  forward   (the 'front' camera and FRONT.HUB both face +Y)
    -X  left, +X right
    +Z  up
    yaw = root.rotation_euler.z, measured from +Y

Pose is *not* cached. Every step re-reads the root's current location and yaw,
integrates, and writes back. That means external code (or an external physics
engine, or a keyframed animation) can move the robot without fighting the drive
model -- only the velocity command lives in this object.
"""

import math

TWO_PI = math.pi * 2


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return (a + math.pi) % TWO_PI - math.pi


def body_to_world(yaw, forward, strafe=0.0):
    """
    Convert body-frame displacement to world XY, given yaw about +Z measured
    from +Y. Forward is +Y in the body frame, so at yaw=0 forward maps to +Y.
    """
    sin_y, cos_y = math.sin(yaw), math.cos(yaw)
    x = -forward * sin_y - strafe * cos_y
    y = forward * cos_y - strafe * sin_y
    return x, y


class DriveBase:
    """
    Common plumbing: pose read/write on the root object, wheel spin, limits.

    Subclasses implement `twist()` -> (linear_velocity, yaw_rate) in the body
    frame and are responsible for their own wheel bookkeeping.
    """

    def __init__(self, root, wheels=None, wheel_radius=0.1,
                 max_speed=2.0, max_yaw_rate=math.pi):
        self.root = root
        self.wheels = wheels or {}
        self.wheel_radius = wheel_radius
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.spin_wheels = True

    # -- pose ---------------------------------------------------------------

    @property
    def yaw(self):
        return self.root.rotation_euler.z

    @yaw.setter
    def yaw(self, value):
        self.root.rotation_euler.z = wrap_angle(value)

    @property
    def position(self):
        loc = self.root.location
        return (loc.x, loc.y)

    # -- interface ----------------------------------------------------------

    def twist(self):
        """Return (v, omega): forward m/s and yaw rad/s in the body frame."""
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def step(self, dt):
        """Integrate one timestep and update the wheel meshes."""
        if dt <= 0:
            return
        v, omega = self.twist()
        v = max(-self.max_speed, min(self.max_speed, v))
        omega = max(-self.max_yaw_rate, min(self.max_yaw_rate, omega))

        yaw = self.yaw
        # Midpoint yaw: integrating position with the *average* heading over the
        # step rather than the starting heading. Straight Euler makes a turning
        # robot spiral outward at large dt; this keeps a constant-radius command
        # on a circle.
        mid_yaw = yaw + omega * dt * 0.5
        dx, dy = body_to_world(mid_yaw, v * dt)

        self.root.location.x += dx
        self.root.location.y += dy
        self.yaw = yaw + omega * dt

        if self.spin_wheels:
            self._spin(dt)

    def _spin(self, dt):
        pass

    def _roll_wheel(self, wheel, surface_speed, dt):
        """
        Advance a wheel's rolling angle.

        Wheel cylinders are built with their spin axis along local +X. With +Y
        forward and +Z up, rolling forward is a *negative* rotation about +X
        (v = omega x r, and X_hat x Z_hat = -Y_hat), hence the sign.
        """
        if wheel is None or not self.wheel_radius:
            return
        wheel.rotation_euler.x -= (surface_speed / self.wheel_radius) * dt


class DifferentialDrive(DriveBase):
    """
    Two independently driven sides. Covers both a true two-wheel differential
    base and the four-wheel skid-steer arrangement robotsim.Robot builds, where
    each side's wheels are commanded together.

    Command either in wheel space (`left`, `right`) or in body space
    (`drive(v, omega)`), which solves back to wheel speeds.
    """

    def __init__(self, root, wheels=None, wheel_radius=0.1, track=0.5, **kw):
        DriveBase.__init__(self, root, wheels, wheel_radius, **kw)
        self.track = track      # lateral distance between left and right wheels
        self.left = 0.0         # left-side surface speed, m/s
        self.right = 0.0

    def drive(self, v, omega=0.0):
        """Command body-frame forward speed and yaw rate."""
        half = 0.5 * omega * self.track
        self.left = v - half
        self.right = v + half
        return self

    def set_wheel_speeds(self, left, right):
        self.left, self.right = left, right
        return self

    def stop(self):
        self.left = self.right = 0.0
        return self

    def twist(self):
        v = 0.5 * (self.left + self.right)
        omega = (self.right - self.left) / self.track if self.track else 0.0
        return v, omega

    def _spin(self, dt):
        for name, wheel in self.wheels.items():
            speed = self.left if '.L.' in name else self.right
            self._roll_wheel(wheel, speed, dt)


class AckermannDrive(DriveBase):
    """
    Car-like steering: rear wheels drive, front wheels steer, and the vehicle
    cannot turn on the spot.

    `steer` is the virtual centre-line steering angle (positive = left). The
    front wheels are given true Ackermann angles -- the inner wheel turns more
    sharply than the outer -- so the rendered geometry is correct rather than
    both front wheels sharing one angle.
    """

    def __init__(self, root, wheels=None, wheel_radius=0.1,
                 wheelbase=0.6, track=0.5, max_steer=math.radians(35), **kw):
        DriveBase.__init__(self, root, wheels, wheel_radius, **kw)
        self.wheelbase = wheelbase   # front axle to rear axle
        self.track = track
        self.max_steer = max_steer
        self.speed = 0.0
        self._steer = 0.0

    @property
    def steer(self):
        return self._steer

    @steer.setter
    def steer(self, value):
        self._steer = max(-self.max_steer, min(self.max_steer, value))

    def drive(self, speed, steer=None):
        self.speed = speed
        if steer is not None:
            self.steer = steer
        return self

    def stop(self):
        self.speed = 0.0
        return self

    def twist(self):
        if not self.wheelbase:
            return self.speed, 0.0
        return self.speed, self.speed * math.tan(self._steer) / self.wheelbase

    @property
    def turn_radius(self):
        """Signed turn radius of the centre line; inf when going straight."""
        if abs(self._steer) < 1e-9:
            return float('inf')
        return self.wheelbase / math.tan(self._steer)

    def steer_angles(self):
        """(left, right) front wheel angles, true Ackermann geometry."""
        if abs(self._steer) < 1e-9:
            return (0.0, 0.0)
        # Work with the unsigned radius and re-apply the sign at the end.
        # Using the signed radius flips the inner/outer roles on right turns.
        radius = abs(self.wheelbase / math.tan(self._steer))
        half = self.track * 0.5
        # A radius tighter than half the track puts the inner wheel's centre
        # inside the turn centre; clamp so the geometry stays sane.
        inner = math.atan(self.wheelbase / max(radius - half, 1e-6))
        outer = math.atan(self.wheelbase / (radius + half))
        if self._steer > 0:
            return (inner, outer)      # left turn: left wheel is inner
        return (-outer, -inner)        # right turn: right wheel is inner

    def _spin(self, dt):
        left_angle, right_angle = self.steer_angles()
        for name, wheel in self.wheels.items():
            if wheel is None:
                continue
            self._roll_wheel(wheel, self.speed, dt)
            if '.FRONT' in name:
                # Blender's XYZ euler order applies X (roll) before Z (steer),
                # which is exactly the right nesting for a steered road wheel.
                wheel.rotation_euler.z = left_angle if '.L.' in name else right_angle


DRIVE_MODELS = {
    'differential': DifferentialDrive,
    'skid': DifferentialDrive,
    'ackermann': AckermannDrive,
}
