"""
Kinematic contact, built on the same ray caster the lidar uses.

This is not a physics engine and does not pretend to be one. There is still no
mass, no momentum and no force. What it adds is the part of "physics" that
matters most for driving a camera around a world: the robot sits on the ground
instead of hovering at a fixed height, it stops when it drives into something
instead of passing through, and the gap between what the wheels did and what the
base did is reported as slip.

It plugs into drive.ContactModel, which is the seam an external solver would use
too -- the drive models command a velocity and ask the contact model what pose
they actually get, and neither has to know which kind of answer is on the other
side.

Cost is a handful of ray casts per tick at ~3.7us each: a four-wheel robot with
ground following and collision is about six casts, so roughly 20us per step.
"""

import math

import bpy
from mathutils import Vector

from drive import ContactModel, body_to_world
from sensors import cast_ignoring, NO_RETURN

UP = Vector((0.0, 0.0, 1.0))


class Contact:
    """What happened during one resolve(), for callers that want to know."""

    def __init__(self, blocked=False, obj=None, normal=None, distance=None,
                 grounded=False, ground_z=None, slid=False):
        self.blocked = blocked
        self.object = obj
        self.normal = normal
        self.distance = distance
        self.grounded = grounded
        self.ground_z = ground_z
        self.slid = slid

    def __repr__(self):
        return '<Contact%s%s%s%s>' % (
            ' blocked' if self.blocked else '',
            ' slid' if self.slid else '',
            ' grounded' if self.grounded else '',
            (' on=%s' % self.object.name) if self.object else '')


class RayContact(ContactModel):
    """
    Ground following and collision blocking by ray casting.

    `ignore` is the robot's own geometry -- without it every downward probe hits
    the robot's own body and reports the ground as being inside itself.

    Ground following casts one ray down per contact point and lifts the base so
    it rides `ride_height` above the surface. With `level` on, several probes are
    used to also pitch and roll the base onto slopes.

    Collision casts along the direction of travel from the body centre. `radius`
    is how far the body extends that way, so the base stops with its edge at the
    obstacle rather than its centre.
    """

    def __init__(self, ignore=(), radius=0.5, ride_height=0.15,
                 ground=True, collide=True, level=False, slide=True,
                 probe_up=1.0, max_drop=10.0, contact_points=None,
                 max_climb=0.15, max_ground_rise=None,
                 climb_normal_z=0.5, climb_probe_up=6.0,
                 max_tilt=math.radians(45.0), stiction=1e-3, scene=None):
        self.ignore = set(ignore)
        self.radius = radius
        self.ride_height = ride_height
        self.ground = ground
        self.collide = collide
        self.level = level
        self.slide = slide
        self.probe_up = probe_up
        self.max_drop = max_drop
        ## Body-frame (x, y) offsets to probe the ground at -- normally the
        ## wheels. Levelling needs at least three; a single centre probe is
        ## enough to just sit on the surface.
        self.contact_points = list(contact_points or [(0.0, 0.0)])
        ## A step this tall or shorter is ridden over rather than treated as a
        ## wall, so kerbs and ramp lips do not stop the robot dead.
        self.max_climb = max_climb
        ## How far the ground may rise in a single step before it is treated as
        ## a ceiling rather than a floor. Deliberately separate from max_climb:
        ## they answer different questions, and a robot built to climb tall
        ## steps would otherwise also start snapping onto overhangs.
        self.max_ground_rise = (max_ground_rise if max_ground_rise is not None
                                else max_climb)
        ## A surface tilted less than this off horizontal counts as drivable.
        self.climb_normal_z = climb_normal_z
        ## How far above the base to start the step-height probe. Must clear any
        ## obstacle meant to block, or the probe starts inside it.
        self.climb_probe_up = climb_probe_up
        ## Levelling clamp. Where contact points straddle a discontinuity -- the
        ## lip between a ramp and the platform it meets, a wheel hanging over an
        ## edge -- the height difference across the robot is a cliff rather than
        ## a slope, and the raw angle can approach vertical. Clamping keeps a
        ## single bad probe from flipping the robot on its back.
        self.max_tilt = max_tilt
        ## Below this the residual velocity after a collision is called zero,
        ## so a base pinned against a wall settles instead of creeping on an
        ## ever-halving remainder.
        self.stiction = stiction
        self.scene = scene

    # -- plumbing -----------------------------------------------------------

    def context(self):
        scene = self.scene or bpy.context.scene
        ## Re-fetched every step: the evaluated depsgraph is what holds current
        ## world transforms, and a stale one resolves against last tick's world.
        return scene, bpy.context.evaluated_depsgraph_get()

    def probe_down(self, x, y, from_z, scene, depsgraph):
        """
        Ground height under a point, or (None, None) if there is nothing there.

        `from_z` is where the probe starts, in world space. It must be above
        whatever is being measured: a probe that starts *inside* geometry hits
        that geometry's underside on the way out and reports a surface far below
        the real one, which is how a wall gets mistaken for flat ground.
        """
        origin = Vector((x, y, from_z))
        obj, location, distance, normal = cast_ignoring(
            scene, depsgraph, origin, -UP, self.probe_up + self.max_drop,
            ignore=self.ignore)
        if obj is None or distance == NO_RETURN:
            return None, None
        return location.z, normal

    # -- the interface ------------------------------------------------------

    def resolve(self, drive, start, target, velocity, dt):
        scene, depsgraph = self.context()
        x0, y0, z0, _yaw0 = start
        x1, y1, z1, yaw1 = target
        v, omega = velocity
        info = Contact()

        if self.collide:
            wanted = math.hypot(x1 - x0, y1 - y0)
            (x1, y1), info = self.check_collision(
                (x0, y0), (x1, y1), z0, scene, depsgraph, info)
            if info.blocked:
                ## An impact takes the speed with it. Without this the base is
                ## merely held in place while its velocity stays at full value,
                ## so it leaps forward the instant the obstacle is cleared --
                ## the pose was corrected but nothing remembered the collision.
                ## Sliding keeps the fraction of the move that survived, so a
                ## glancing contact bleeds off speed rather than killing it.
                got = math.hypot(x1 - x0, y1 - y0)
                v *= (got / wanted) if wanted > 1e-12 else 0.0
                if v < self.stiction:
                    v = 0.0

        if self.ground:
            z1, info = self.settle(x1, y1, z1, yaw1, scene, depsgraph, info)

        if self.level:
            self.apply_level(drive, x1, y1, yaw1, scene, depsgraph)

        return (x1, y1, z1, yaw1), (v, omega), info

    # -- collision ----------------------------------------------------------

    def check_collision(self, origin_xy, target_xy, z, scene, depsgraph, info):
        """
        Stop the base at the first obstacle along its path, or slide along it.

        The probe is cast at `ride_height` above the ground rather than at the
        base origin, so it tests the body's own height band and is not fooled by
        the floor it is standing on.
        """
        move = Vector((target_xy[0] - origin_xy[0], target_xy[1] - origin_xy[1], 0.0))
        travel = move.length
        if travel < 1e-9:
            return target_xy, info

        allowed = self.free_travel(origin_xy, move, z, scene, depsgraph)
        if allowed is None:
            return target_xy, info      ## nothing in the way

        obj, location, normal, distance = allowed
        ## A ramp is not a wall. The forward probe cannot tell them apart on its
        ## own -- both are just geometry in the way -- so anything drivable is
        ## let through here and handled by ground following instead.
        ## Measured from the ground, not from the base origin: a step's height
        ## is how far it rises above what we are standing on.
        if self.ground and self.climbable(location, normal, z - self.ride_height,
                                          scene, depsgraph):
            return target_xy, info

        info.blocked = True
        info.object = obj
        info.normal = normal
        info.distance = distance

        ## Slide: keep the component of the move along the surface and drop the
        ## component into it. Without this a robot meeting a wall at a shallow
        ## angle stops dead, which is both unrealistic and a poor training
        ## signal -- real wheels skate along the wall.
        if self.slide and normal is not None:
            flat = Vector((normal.x, normal.y, 0.0))
            if flat.length > 1e-9:
                flat.normalize()
                tangent = move - flat * move.dot(flat)
                if tangent.length > 1e-9:
                    slid_to = (origin_xy[0] + tangent.x, origin_xy[1] + tangent.y)
                    if self.free_travel(origin_xy, tangent, z, scene, depsgraph) is None:
                        info.slid = True
                        return slid_to, info

        ## Blocked outright: hold position. Advancing to just short of the
        ## obstacle instead would creep into it over many ticks, since each
        ## step re-measures from the new position.
        return origin_xy, info

    def free_travel(self, origin_xy, move, z, scene, depsgraph):
        """
        None if `move` is clear, else (object, normal, distance) of what blocks.

        The ray runs the length of the move plus the body radius, so contact is
        detected when the body's leading edge reaches the obstacle rather than
        when its centre does.
        """
        travel = move.length
        if travel < 1e-9:
            return None
        direction = move.normalized()
        origin = Vector((origin_xy[0], origin_xy[1], z + self.ride_height))
        obj, location, distance, normal = cast_ignoring(
            scene, depsgraph, origin, direction, travel + self.radius,
            ignore=self.ignore)
        if obj is None or distance == NO_RETURN:
            return None
        return obj, location, normal, distance

    def climbable(self, point, normal, ground_z, scene, depsgraph):
        """
        Is the thing we just hit something to drive onto rather than stop at?

        Two ways to qualify. A surface whose normal points mostly up is a ramp or
        a floor, so it is drivable however far it rises. Otherwise it is a step,
        and it is drivable only if its top is within `max_climb` -- which is what
        lets a robot ride over a kerb but not through a wall.
        """
        if normal is not None and normal.z > self.climb_normal_z:
            return True
        if point is None:
            return False
        ## Probe from well above so the ray does not start inside the obstacle.
        top, _n = self.probe_down(point.x, point.y, ground_z + self.climb_probe_up,
                                  scene, depsgraph)
        if top is None:
            return False
        return (top - ground_z) <= self.max_climb

    # -- ground -------------------------------------------------------------

    def snap(self, root):
        """
        Place the base on the ground right now, ignoring the climb limit.

        Initial placement is not a step: a robot dropped in at an arbitrary
        height -- or spawned below the terrain -- has to reach the surface in one
        go, and the per-step rise limit that keeps it off ceilings would
        otherwise strand it there.
        """
        scene, depsgraph = self.context()
        loc = root.location
        z, info = self.settle(loc.x, loc.y, loc.z, root.rotation_euler.z,
                              scene, depsgraph, Contact(), clamp=False)
        loc.z = z
        return info

    def settle(self, x, y, z, yaw, scene, depsgraph, info, clamp=True):
        """Drop or lift the base so it rides above the surface beneath it."""
        current = z - self.ride_height       ## ground we are standing on now
        heights = []
        for ox, oy in self.contact_points:
            wx, wy = self.to_world(x, y, yaw, ox, oy)
            ground, _normal = self.probe_down(wx, wy, z + self.probe_up,
                                              scene, depsgraph)
            if ground is None:
                continue
            ## A downward probe cannot tell a floor from a ceiling: driving under
            ## an overhanging ramp, the moment its underside comes within
            ## probe_up the probe reports it as ground and the robot teleports up
            ## onto it. Anything rising faster than the robot could climb in one
            ## step is therefore not ground it is standing on.
            if clamp and ground - current > self.max_ground_rise:
                continue
            heights.append(ground)
        if not heights:
            return z, info          ## nothing to stand on: leave it where it is
        ## The highest qualifying contact wins: resting on the tallest thing
        ## under the robot is what stops a wheel sinking into a ramp it climbs.
        ground = max(heights)
        info.grounded = True
        info.ground_z = ground
        return ground + self.ride_height, info

    @staticmethod
    def to_world(x, y, yaw, ox, oy):
        """Body-frame offset to world XY, using the shared yaw convention."""
        dx, dy = body_to_world(yaw, oy, -ox)
        return x + dx, y + dy

    # -- levelling ----------------------------------------------------------

    def apply_level(self, drive, x, y, yaw, scene, depsgraph):
        """
        Pitch and roll the base onto the slope under it.

        Blender's XYZ euler builds its matrix as Rz @ Ry @ Rx, so the X and Y
        rotations act in the body frame *before* yaw is applied -- which is
        exactly what is wanted: X tilts nose up/down, Y rolls left/right, and
        both stay meaningful whatever direction the robot is facing.
        """
        root = drive.root
        front, rear, left, right = [], [], [], []
        for ox, oy in self.contact_points:
            wx, wy = self.to_world(x, y, yaw, ox, oy)
            ground, _normal = self.probe_down(wx, wy, root.location.z + self.probe_up,
                                              scene, depsgraph)
            if ground is None:
                continue
            (front if oy > 0 else rear).append(ground)
            (right if ox > 0 else left).append(ground)

        def mean(values):
            return sum(values) / len(values) if values else None

        f, r, l, rt = mean(front), mean(rear), mean(left), mean(right)
        span_y = self.span([oy for _ox, oy in self.contact_points])
        span_x = self.span([ox for ox, _oy in self.contact_points])
        if f is not None and r is not None and span_y > 1e-6:
            ## Nose higher than tail is a positive rotation about body +X.
            root.rotation_euler.x = self.clamp_tilt(math.atan2(f - r, span_y))
        if l is not None and rt is not None and span_x > 1e-6:
            ## Right higher than left rolls about body +Y (forward).
            root.rotation_euler.y = self.clamp_tilt(math.atan2(rt - l, span_x))

    def clamp_tilt(self, angle):
        return max(-self.max_tilt, min(self.max_tilt, angle))

    @staticmethod
    def span(values):
        return (max(values) - min(values)) if values else 0.0
