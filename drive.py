"""
Kinematic base motion for wheeled robots.

These are *kinematic* models: they integrate a velocity command into a pose and
spin the wheel meshes to match. There is no mass and no traction. That is
deliberate -- it is the right level of fidelity for camera-in-the-loop work.

Contact with the world is not modelled here either, but there is a seam for it:
`step()` builds the pose it wants and passes it to an optional `ContactModel`,
which returns the pose it actually gets. contact.py implements that by ray
casting, and an external physics engine would slot into the same place, leaving
the command interface unchanged.

Deliberately free of `bpy`: these classes only read and write `.location` and
`.rotation_euler` on whatever objects they are handed, so the maths is testable
outside Blender.

Wheel layout lives here too, for the same reason: where the wheels sit is pure
geometry, and the drive models derive track, wheelbase and per-wheel speed from
those positions rather than from how the wheels happen to be named.

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

SIDES = ('L', 'R', 'C')


def approach(current, target, max_delta):
    """
    Move `current` toward `target` by at most `max_delta`.

    The whole of inertia, in three lines: a base cannot change speed instantly,
    so it closes the gap to the commanded speed at a bounded rate. Braking uses
    the same limit as accelerating, which is a simplification -- real brakes beat
    real motors -- but it keeps the model honest about there being a limit at
    all.
    """
    if max_delta is None or max_delta <= 0:
        return target
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


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


# ---------------------------------------------------------------------------
# wheels
# ---------------------------------------------------------------------------

def parse_wheel_name(name):
    """
    Recover (side, axle) from the 'W.<SIDE>.<AXLE>' naming convention.

    Kept because a drive model may be handed a bare {name: object} dict -- the
    original interface -- rather than Wheel records. Anything unparseable is
    treated as a centre wheel on the middle axle, which is the harmless case.
    """
    parts = str(name).split('.')
    side = parts[1] if len(parts) >= 2 and parts[1] in SIDES else 'C'
    axle = parts[2] if len(parts) >= 3 and parts[2] else 'MID'
    return side, axle


class Wheel:
    """
    One wheel: the object to spin, plus the geometry the drive models need.

    `x` is the signed lateral offset from the centre line (+X right) and is what
    makes arbitrary wheel counts work -- a wheel's surface speed is determined by
    where it sits, not by which side its name says it is on. `y` is the
    longitudinal offset, used to measure the wheelbase.

    `steerable` defaults to "is this wheel on the front axle", which reproduces
    the original '.FRONT' string test for legacy callers while letting generated
    layouts state it explicitly.

    `driven` records whether the wheel is powered. Passive wheels (casters) still
    roll -- being dragged over the ground spins them -- so this is metadata for
    callers rather than something `_spin` acts on.
    """

    def __init__(self, obj, name=None, x=None, y=None, side=None, axle=None,
                 driven=True, steerable=None, radius=None):
        self.obj = obj
        self.name = name or getattr(obj, 'name', 'W.C.MID')
        parsed_side, parsed_axle = parse_wheel_name(self.name)
        self.side = side or parsed_side
        self.axle = axle or parsed_axle
        loc = getattr(obj, 'location', None)
        self.x = float(x) if x is not None else (float(loc.x) if loc is not None else 0.0)
        self.y = float(y) if y is not None else (float(loc.y) if loc is not None else 0.0)
        self.driven = driven
        self.steerable = (self.axle == 'FRONT') if steerable is None else bool(steerable)
        self.radius = radius

    def __repr__(self):
        return '<Wheel %s side=%s axle=%s x=%+.3f y=%+.3f%s%s>' % (
            self.name, self.side, self.axle, self.x, self.y,
            '' if self.driven else ' passive',
            ' steerable' if self.steerable else '')


def axle_labels(n_axles):
    """
    Front-to-rear axle names. FRONT and REAR are meaningful to the drive models
    (steering, wheelbase), so they are always the outermost two.
    """
    if n_axles <= 1:
        return ['MID']
    if n_axles == 2:
        return ['FRONT', 'REAR']
    if n_axles == 3:
        return ['FRONT', 'MID', 'REAR']
    return ['FRONT'] + ['MID%d' % i for i in range(n_axles - 2)] + ['REAR']


def axle_offsets(n_axles, length):
    """Front-to-rear y offsets, spread evenly over the body length."""
    if n_axles <= 1:
        return [0.0]
    half = length * 0.5
    step = length / (n_axles - 1)
    return [half - step * i for i in range(n_axles)]


def wheel_layout(count, size=(1.0, 1.0, 0.1)):
    """
    Generate wheel specs for `count` wheels on a body of `size`.

    Pure geometry -- returns dicts, creates nothing. robotsim.Robot turns each
    spec into a cylinder and a Wheel.

    Even counts are laid out as left/right pairs on count/2 axles. Odd counts get
    the same pairs plus one passive centre wheel on the front axle, which is the
    usual tricycle / caster arrangement rather than an unbalanced side. The
    four-wheel case reproduces the original hard-coded layout exactly.
    """
    x, y, z = size
    zc = -z * 0.5
    if count <= 0:
        return []
    if count == 1:
        ## A single wheel has no axle to be front or rear of.
        return [dict(name='W.C.MID', location=(0.0, 0.0, zc),
                     x=0.0, y=0.0, side='C', axle='MID')]

    has_caster = count % 2
    pairs = (count - has_caster) // 2
    n_axles = pairs + has_caster
    labels = axle_labels(n_axles)
    offsets = axle_offsets(n_axles, y)

    specs = []
    for i, (label, oy) in enumerate(zip(labels, offsets)):
        if has_caster and i == 0:
            ## The odd wheel out: unpowered, on the centre line, up front.
            specs.append(dict(name='W.C.%s' % label, location=(0.0, oy, zc),
                              x=0.0, y=oy, side='C', axle=label, driven=False))
            continue
        for side, ox in (('L', -x * 0.5), ('R', x * 0.5)):
            specs.append(dict(name='W.%s.%s' % (side, label), location=(ox, oy, zc),
                              x=ox, y=oy, side=side, axle=label))
    return specs


def layout_track(specs, fallback=0.5):
    """Lateral span of a wheel layout, i.e. the effective track width."""
    xs = [s['x'] for s in specs]
    span = (max(xs) - min(xs)) if xs else 0.0
    return span if span > 1e-9 else fallback


def layout_wheelbase(specs, fallback=0.6):
    """Longitudinal span of a wheel layout, i.e. front axle to rear axle."""
    ys = [s['y'] for s in specs]
    span = (max(ys) - min(ys)) if ys else 0.0
    return span if span > 1e-9 else fallback


class ContactModel:
    """
    The seam between a drive model and the world it is moving through.

    `step()` builds the pose it *wants*, and the contact model returns the pose
    it actually gets -- plus the velocity it comes away with. Velocity is the
    half that makes the seam able to host a real solver: a pose-only interface
    can stop a robot at a wall, but the robot resumes full speed the instant the
    wall is gone, because nothing carried the impact forward. Returning velocity
    lets contact express that hitting something *took your speed away*.

    Deliberately an interface rather than an implementation, so this module stays
    free of `bpy`. contact.RayContact is the ray-cast implementation, and an
    external physics engine slots in at exactly this point.

    `start` and `target` are (x, y, z, yaw); `velocity` is (v, omega) in the
    body frame. Returns (pose, velocity, info), where info is an arbitrary record
    of what happened or None.
    """

    def resolve(self, drive, start, target, velocity, dt):
        return target, velocity, None


class DriveBase:
    """
    Common plumbing: pose read/write on the root object, wheel spin, limits.

    Subclasses implement `twist()` -> (linear_velocity, yaw_rate) in the body
    frame and are responsible for their own wheel bookkeeping.
    """

    def __init__(self, root, wheels=None, wheel_radius=0.1,
                 max_speed=2.0, max_yaw_rate=math.pi, contact=None,
                 max_accel=None, max_yaw_accel=None):
        self.root = root
        self.wheel_list = self.normalise_wheels(wheels)
        ## Kept as {name: object} for callers and tests that predate Wheel.
        self.wheels = {w.name: w.obj for w in self.wheel_list}
        self.wheel_radius = wheel_radius
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.spin_wheels = True
        ## None means the old behaviour exactly: the commanded pose is the pose.
        self.contact = contact
        self.last_contact = None
        self.slip = 0.0
        ## Actual base velocity, as opposed to what the motors were told. These
        ## are the state that carries momentum from one tick to the next.
        self.v = 0.0
        self.omega = 0.0
        ## None means no limit, i.e. the base reaches its commanded speed within
        ## one tick -- which is what every drive did before inertia existed, and
        ## stays the default so nothing changes unless it is asked for.
        self.max_accel = max_accel
        self.max_yaw_accel = max_yaw_accel

    @staticmethod
    def normalise_wheels(wheels):
        """
        Accept any of: a {name: object} dict (the original interface), a list of
        objects, a list of Wheel, or a list of spec dicts as produced by
        wheel_layout(). Everything becomes a list of Wheel.
        """
        if not wheels:
            return []
        items = list(wheels.values()) if isinstance(wheels, dict) else list(wheels)
        out = []
        for item in items:
            if isinstance(item, Wheel):
                out.append(item)
            elif isinstance(item, dict):
                spec = dict(item)
                spec.pop('location', None)   ## placement is the caller's business
                out.append(Wheel(**spec))
            else:
                out.append(Wheel(item))
        return out

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
        """
        Integrate one timestep and update the wheel meshes.

        The commanded twist is what the motors are being asked for; `self.v` and
        `self.omega` are what the base is actually doing. With acceleration
        limits set the second lags the first, which is momentum without ever
        modelling a force -- and it is also where contact writes back the speed
        that an impact took away.
        """
        if dt <= 0:
            return
        v_cmd, omega_cmd = self.twist()
        v_cmd = max(-self.max_speed, min(self.max_speed, v_cmd))
        omega_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, omega_cmd))

        ## No limit means the base *is* its own command, exactly as before.
        v = approach(self.v, v_cmd, self.max_accel * dt) if self.max_accel else v_cmd
        omega = (approach(self.omega, omega_cmd, self.max_yaw_accel * dt)
                 if self.max_yaw_accel else omega_cmd)

        yaw = self.yaw
        # Midpoint yaw: integrating position with the *average* heading over the
        # step rather than the starting heading. Straight Euler makes a turning
        # robot spiral outward at large dt; this keeps a constant-radius command
        # on a circle.
        mid_yaw = yaw + omega * dt * 0.5
        dx, dy = body_to_world(mid_yaw, v * dt)

        loc = self.root.location
        start = (loc.x, loc.y, loc.z, yaw)
        target = (loc.x + dx, loc.y + dy, loc.z, yaw + omega * dt)

        ## The world gets a say before the pose is committed.
        info = None
        if self.contact is not None:
            target, (v, omega), info = self.contact.resolve(
                self, start, target, (v, omega), dt)
        self.last_contact = info
        self.v, self.omega = v, omega

        x1, y1, z1, yaw1 = target
        self.root.location.x = x1
        self.root.location.y = y1
        self.root.location.z = z1
        self.yaw = yaw1

        ## Slip ratio: the disagreement between the wheels and the base, scaled
        ## by whichever is moving faster. Written this way rather than as
        ## 1 - actual/commanded so that it still means something when the
        ## command is zero -- brakes locked with the base still sliding is total
        ## slip, not undefined, and the old form silently reported it as none.
        reference = max(abs(v_cmd), abs(self.v))
        self.slip = 0.0 if reference < 1e-9 else abs(v_cmd - self.v) / reference

        if self.spin_wheels:
            ## Spun from the clamped *command*, not from what the base managed:
            ## the wheels are driven by the motors, not dragged by the ground.
            self._spin(dt, v_cmd, omega_cmd)

    def surface_speed(self, wheel, v, omega):
        """
        Ground speed of one wheel's contact patch, from the body twist.

        A rigid body turning at `omega` moves a point offset `x` to the right of
        the centre line faster by `omega * x`. For a two-sided layout this
        recovers the commanded left/right speeds exactly, and unlike matching on
        '.L.' it also gives the right answer for centre wheels, extra axles and
        any custom placement.
        """
        return v + omega * wheel.x

    def _spin(self, dt, v, omega):
        for wheel in self.wheel_list:
            self._roll_wheel(wheel, self.surface_speed(wheel, v, omega), dt)

    def _roll_wheel(self, wheel, surface_speed, dt):
        """
        Advance a wheel's rolling angle.

        Wheel cylinders are built with their spin axis along local +X. With +Y
        forward and +Z up, rolling forward is a *negative* rotation about +X
        (v = omega x r, and X_hat x Z_hat = -Y_hat), hence the sign.
        """
        if wheel is None or wheel.obj is None:
            return
        radius = wheel.radius or self.wheel_radius
        if not radius:
            return
        wheel.obj.rotation_euler.x -= (surface_speed / radius) * dt


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

    ## No _spin override: DriveBase.surface_speed() derives each wheel's speed
    ## from its lateral offset, which reproduces `left` and `right` exactly for
    ## wheels at -track/2 and +track/2 and also handles extra axles and casters.


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

    def _spin(self, dt, v, omega):
        left_angle, right_angle = self.steer_angles()
        for wheel in self.wheel_list:
            if wheel.obj is None:
                continue
            self._roll_wheel(wheel, self.surface_speed(wheel, v, omega), dt)
            if not wheel.steerable:
                continue
            # Blender's XYZ euler order applies X (roll) before Z (steer),
            # which is exactly the right nesting for a steered road wheel.
            if wheel.side == 'L':
                angle = left_angle
            elif wheel.side == 'R':
                angle = right_angle
            else:
                ## A lone centre wheel on the front axle is a tricycle: it sits
                ## on the centre line, so it takes the centre-line angle rather
                ## than either Ackermann-corrected one.
                angle = self._steer
            wheel.obj.rotation_euler.z = angle


DRIVE_MODELS = {
    'differential': DifferentialDrive,
    'skid': DifferentialDrive,
    'ackermann': AckermannDrive,
}
