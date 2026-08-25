"""
Ranging sensors.

Lidar is built on Blender's ray caster rather than on a depth render, which is
worth justifying because the depth pass is right there and looks like it should
work:

  * Blender's Z pass is *planar depth* -- the distance to the camera plane, not
    along the ray. A flat wall perpendicular to a 16mm lens reads the same value
    at the centre of the image and at the edge, where the true range is 1.5x
    longer. Turning that into ranges means a per-pixel 1/cos correction.
  * A camera is a pinhole projection, so a 360 degree scan needs several renders
    stitched together, and pixel columns are tan-spaced rather than evenly spaced
    in angle -- so every beam angle needs resampling.
  * Ray casting has neither problem. Rays go exactly where the beams go, the
    range is the true radial distance, and the cast returns the object that was
    hit, which gives per-beam semantic labels for free.

It is also fast: ~3.7us per ray, so a 360-beam single-channel scan costs about
1.6ms and a 16-channel 11520-ray scan about 43ms. That is cheap enough to run on
the control loop every tick, which is the point -- lidar is one of the fast
senses the base is supposed to react on while vision is sampled rarely.

FRAME CONVENTION (matches drive.py and robotsim.Robot):
    +Y forward, +X right, +Z up
    azimuth   measured from +Y, positive turning left (same sense as yaw)
    elevation positive up
"""

import math

import bpy
from mathutils import Vector

TAU = math.pi * 2

## No return. Matches the ROS LaserScan convention, where a range outside
## [range_min, range_max] means the beam hit nothing rather than hit something
## at zero distance -- a 0.0 default would read as an obstacle touching the
## sensor, which is the most dangerous possible way to be wrong.
NO_RETURN = float('inf')


def beam_angles(fov, resolution, centre=0.0):
    """
    Evenly spaced beam angles covering `fov`, `resolution` radians apart.

    A full circle drops the duplicate endpoint, since -pi and +pi are the same
    beam; a partial sweep keeps both ends inclusive.
    """
    if resolution <= 0:
        return [centre]
    count = int(round(abs(fov) / resolution))
    if count <= 0:
        return [centre]
    start = centre - fov * 0.5
    if abs(abs(fov) - TAU) < 1e-9:
        return [start + i * resolution for i in range(count)]
    return [start + i * resolution for i in range(count + 1)]


def ray_direction(azimuth, elevation=0.0):
    """
    Unit direction for a beam, in the sensor's own frame.

    At azimuth 0 and elevation 0 this is +Y, i.e. straight ahead, and positive
    azimuth swings left -- the same sense as yaw in drive.py.
    """
    ce = math.cos(elevation)
    return Vector((-math.sin(azimuth) * ce,
                   math.cos(azimuth) * ce,
                   math.sin(elevation)))


def cast_ignoring(scene, depsgraph, origin, direction, distance, ignore=(),
                  max_steps=24, nudge=1e-4):
    """
    Cast a ray, stepping over anything in `ignore`.

    Shared by the lidar and by the contact model, because they need exactly the
    same query: "what is the first *real* thing along this ray, ignoring the
    robot doing the asking". Returns (object, location, distance, normal), with
    object None and distance NO_RETURN when nothing was hit.

    Restarting just past an ignored surface rather than discarding the ray
    matters: a mast in front of a lidar would otherwise punch a permanent blind
    sector into every scan instead of merely occluding what is behind it. Each
    solid crossed costs *two* hits, its near face and its far face, so the step
    budget needs headroom.
    """
    start = Vector(origin)
    direction = Vector(direction).normalized()
    remaining = distance
    travelled = 0.0
    for _ in range(max_steps + 1):
        hit, location, normal, _index, obj, _matrix = scene.ray_cast(
            depsgraph, start, direction, distance=remaining)
        if not hit:
            return None, None, NO_RETURN, None
        step = (location - start).length
        travelled += step
        original = getattr(obj, 'original', obj)
        if original not in ignore and obj not in ignore:
            if normal is not None and normal.dot(direction) > 0.0:
                ## A back-face means the ray is *inside* this object. That
                ## happens whenever an ignored surface is exactly coincident
                ## with a real one -- a wheel resting precisely on the ground,
                ## which is the normal state of a robot that is not falling.
                ## Stepping past the wheel lands the ray a hair inside the
                ## ground, and the next hit is the ground's underside: without
                ## this check a ground probe reports the floor one slab-
                ## thickness too low, and the robot sinks a little every step.
                ## The real surface is where this segment started.
                return obj, Vector(start), max(travelled - step, 0.0), normal
            return obj, location, travelled, normal
        remaining -= step
        if remaining <= 1e-6:
            break
        start = location + direction * nudge
        travelled += nudge
    return None, None, NO_RETURN, None


class LidarScan:
    """
    One full sweep: a range per beam, plus what each beam hit.

    Ranges are metres from the sensor origin, in beam order (all azimuths of
    channel 0, then channel 1, ...). Misses are NO_RETURN rather than 0.
    """

    def __init__(self, lidar, ranges, labels, azimuths, elevations, origin):
        self.lidar = lidar
        self.ranges = ranges
        self.labels = labels          ## pass_index of the object each beam hit
        self.azimuths = azimuths
        self.elevations = elevations
        self.origin = origin          ## world-space sensor position at scan time

    def __len__(self):
        return len(self.ranges)

    def __repr__(self):
        return '<LidarScan beams=%d hits=%d nearest=%.3f>' % (
            len(self), self.hits, self.min_range)

    # -- indexing -----------------------------------------------------------

    @property
    def beams(self):
        return len(self.azimuths)

    @property
    def channels(self):
        return len(self.elevations)

    def index(self, beam, channel=0):
        return channel * self.beams + beam

    def range_at(self, beam, channel=0):
        return self.ranges[self.index(beam, channel)]

    # -- summary ------------------------------------------------------------

    @property
    def hits(self):
        return sum(1 for r in self.ranges if r != NO_RETURN)

    @property
    def min_range(self):
        returns = [r for r in self.ranges if r != NO_RETURN]
        return min(returns) if returns else NO_RETURN

    def nearest(self):
        """(range, azimuth, channel) of the closest return, for obstacle work."""
        best = (NO_RETURN, 0.0, 0)
        for channel in range(self.channels):
            for beam in range(self.beams):
                r = self.range_at(beam, channel)
                if r < best[0]:
                    best = (r, self.azimuths[beam], channel)
        return best

    def sector(self, start, end):
        """
        Closest return with an azimuth in [start, end].

        Handles a sector that wraps through +/-pi, so a forward-looking check can
        be written as sector(-0.3, 0.3) without special casing.
        """
        start, end = wrap(start), wrap(end)
        closest = NO_RETURN
        for channel in range(self.channels):
            for beam, az in enumerate(self.azimuths):
                a = wrap(az)
                inside = (start <= a <= end) if start <= end else (a >= start or a <= end)
                if inside:
                    closest = min(closest, self.range_at(beam, channel))
        return closest

    # -- geometry -----------------------------------------------------------

    def points(self, world=False):
        """
        Hit positions as a list of Vectors, misses dropped.

        Sensor frame by default -- which is what a network wants, since it is
        the frame the robot acts in. `world=True` gives scene coordinates, which
        is what a debug visualisation wants.
        """
        out = []
        matrix = self.lidar.mount.matrix_world if world else None
        for channel, elevation in enumerate(self.elevations):
            for beam, azimuth in enumerate(self.azimuths):
                r = self.range_at(beam, channel)
                if r == NO_RETURN:
                    continue
                local = ray_direction(azimuth, elevation) * r
                out.append(matrix @ local if matrix else local)
        return out

    def to_mesh(self, name='LIDAR.CLOUD'):
        """Build a point-cloud mesh of this scan, for looking at it in Blender."""
        mesh = bpy.data.meshes.new(name)
        mesh.from_pydata([tuple(p) for p in self.points(world=True)], [], [])
        mesh.update()
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.scene.collection.objects.link(obj)
        return obj


def wrap(a):
    """Wrap to (-pi, pi]."""
    return (a + math.pi) % TAU - math.pi


class Lidar:
    """
    A ray-cast ranging sensor mounted on an object.

    `mount` is any object whose world matrix gives the sensor pose -- normally an
    empty parented to the robot, so the sensor rides along and can be keyframed
    like anything else.

    Defaults describe a cheap single-plane scanner: a full 360 degrees at one
    degree resolution, one channel, 50m range. `channels` > 1 spreads beams
    vertically over `v_fov`, which is the multi-plane arrangement of a spinning
    unit like a VLP-16.
    """

    def __init__(self, mount, channels=1, h_fov=TAU, h_resolution=math.radians(1.0),
                 v_fov=math.radians(30.0), v_offset=0.0,
                 range_min=0.05, range_max=50.0, ignore=(), interval=1,
                 max_self_hits=24):
        self.mount = mount
        self.range_min = range_min
        self.range_max = range_max
        ## Objects the beams should pass through -- normally the robot's own
        ## parts. A real scanner sees its own mast too and masks it out.
        self.ignore = set(ignore)
        ## Lidar is a fast sense: unlike the cameras it defaults to every tick.
        self.interval = interval
        ## Budget for stepping over ignored geometry. Each solid crossed costs
        ## *two* hits -- the near face and the far face -- so a beam leaving a
        ## robot through its body and a wheel has already spent four. Too small
        ## a budget silently returns no-hit for exactly the beams that pass
        ## closest to the robot, which are the ones that matter most.
        self.max_self_hits = max_self_hits
        self.last_scan_tick = None
        self.scans = 0
        ## The most recent sweep, kept so a telemetry recorder or a controller
        ## can read it without taking another one -- a scan is cheap but not
        ## free, and two callers wanting the same tick's data should share it.
        self.last_scan = None

        self.azimuths = beam_angles(h_fov, h_resolution)
        if channels <= 1:
            self.elevations = [v_offset]
        else:
            step = v_fov / float(channels - 1)
            self.elevations = [v_offset - v_fov * 0.5 + i * step for i in range(channels)]

    def __repr__(self):
        return '<Lidar %s beams=%d channels=%d range=%.1fm>' % (
            self.mount.name, len(self.azimuths), len(self.elevations), self.range_max)

    @property
    def rays(self):
        return len(self.azimuths) * len(self.elevations)

    # -- scanning -----------------------------------------------------------

    def cast(self, origin, direction, depsgraph, scene):
        """One beam, stepping over anything in `ignore`. See cast_ignoring()."""
        obj, location, distance, _normal = cast_ignoring(
            scene, depsgraph, origin, direction, self.range_max,
            ignore=self.ignore, max_steps=self.max_self_hits)
        return obj, location, distance

    def scan(self, depsgraph=None, scene=None):
        """Take one full sweep and return a LidarScan."""
        scene = scene or bpy.context.scene
        ## The evaluated depsgraph is what actually holds current world
        ## transforms; re-fetched each scan so the sweep reflects this tick's
        ## motion rather than the previous one's.
        depsgraph = depsgraph or bpy.context.evaluated_depsgraph_get()

        matrix = self.mount.matrix_world
        origin = matrix.translation.copy()
        rotation = matrix.to_3x3().normalized()

        ranges, labels = [], []
        for elevation in self.elevations:
            for azimuth in self.azimuths:
                direction = (rotation @ ray_direction(azimuth, elevation)).normalized()
                obj, _location, distance = self.cast(origin, direction, depsgraph, scene)
                if obj is None or distance < self.range_min or distance > self.range_max:
                    ranges.append(NO_RETURN)
                    labels.append(0)
                else:
                    ranges.append(distance)
                    labels.append(getattr(obj, 'pass_index', 0))

        self.scans += 1
        self.last_scan = LidarScan(self, ranges, labels, list(self.azimuths),
                                   list(self.elevations), origin)
        return self.last_scan

    # -- rate ---------------------------------------------------------------

    def due(self, tick):
        """Is a scan due this tick? Same scheduling shape as the cameras."""
        if not self.interval:
            return False
        if self.last_scan_tick is None:
            return True
        return (tick - self.last_scan_tick) >= self.interval

    def sample(self, tick=None, force=False, depsgraph=None, scene=None):
        """Scan if due, else return None."""
        if tick is None:
            tick = 0
        if not force and not self.due(tick):
            return None
        self.last_scan_tick = tick
        return self.scan(depsgraph=depsgraph, scene=scene)
