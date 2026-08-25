"""
Running real firmware against the simulated robot.

robotsim's control loops are Python calling `drive.drive(v, omega)`. That is the
right level for working on the robot, and the wrong level for answering whether
the C that will actually ship does the same thing. This module closes that gap:
the application is compiled by [crust](https://github.com/brentharts/crust)'s
`hostsim`, runs as real machine code on its own thread, and reads and writes a
plant that is this simulator.

    bot = Robot()
    board = bot.attach_firmware('drive.c')
    board.target = 4000              # firmware's setpoint, in encoder counts
    bot.step(dt)                     # firmware and plant advance together

Both crust and armulator are **optional**. Nothing here is imported unless they
are cloned beside robotsim, and `available()` reports what is present so a test
can skip cleanly rather than fail.

TWO FIDELITIES, AND WHY ONLY ONE IS IN THE LOOP
-----------------------------------------------
crust ships two ways to run an image, and is emphatic that neither replaces the
other:

  * `hostsim` compiles the application for the host and replaces the hardware
    under it. It executes no ARM, so it proves nothing about code generation --
    but it runs about 4000x faster than instruction emulation.
  * `armulator` executes AArch64 one instruction at a time, with the MMU, the
    exception levels and register-level peripherals. It proves an image boots,
    at roughly 17,000 instructions a second.

Only hostsim can be in the tick loop. armulator is ~80,000x slower than real
time, so a fifteen-second robotsim run -- 900 ticks -- would take on the order
of a fortnight. armulator belongs offline, gating an image before it is trusted;
see `armulator_available()`.

THE SEAM IS VALUES, NOT REGISTERS
---------------------------------
hostsim's seam is `sim_motor_write(duty)` and `sim_encoder_read()`, not a PWM
duty register and a quadrature counter. That is the right shape for asking
whether the *system* behaves and the wrong shape for asking whether a *driver*
is correct: firmware that programs a PCA9685 incorrectly will work perfectly
here. That question belongs to armulator, which models the registers.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIDE_BY_SIDE = os.path.dirname(HERE)

CRUST = os.path.join(SIDE_BY_SIDE, 'crust')
ARMULATOR = os.path.join(SIDE_BY_SIDE, 'armulator')

#: Architected counter frequency. Matches hostsim's default, which matches the
#: Jetson and the Pi 3 -- so simulated time lines up with the boards this is
#: standing in for.
DEFAULT_FREQ = 19200000


# ---------------------------------------------------------------------------
# discovery
# ---------------------------------------------------------------------------

def crust_root():
    """Path to a crust checkout beside robotsim, or None."""
    if os.path.isfile(os.path.join(CRUST, 'tools', 'hostsim.py')):
        return CRUST
    return None


def armulator_root():
    """Path to an armulator checkout beside robotsim, or None."""
    if os.path.isdir(os.path.join(ARMULATOR, 'armulator')):
        return ARMULATOR
    return None


def have_compiler(cc='gcc'):
    """hostsim compiles the application with a host compiler; is one here?"""
    try:
        subprocess.run([cc, '--version'], capture_output=True, check=True)
        return True
    except (OSError, subprocess.CalledProcessError):
        return False


def available():
    """
    True when firmware simulation can actually run.

    Checked rather than assumed, and reported rather than raised, so a test can
    skip on a machine without the optional checkouts instead of failing on one.
    """
    return bool(crust_root()) and have_compiler()


def armulator_available():
    return bool(armulator_root())


def why_unavailable():
    """A sentence explaining what is missing, for a skip message."""
    if not crust_root():
        return ('crust is not cloned beside robotsim (expected %s) -- '
                'git clone https://github.com/brentharts/crust.git' % CRUST)
    if not have_compiler():
        return 'no host compiler on PATH; hostsim needs gcc'
    return ''


def _import_hostsim():
    root = crust_root()
    if not root:
        raise RuntimeError(why_unavailable())
    tools = os.path.join(root, 'tools')
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import hostsim
    return hostsim


# ---------------------------------------------------------------------------
# building
# ---------------------------------------------------------------------------

def build(source, out=None, cc='gcc', defines=(), libs=(), rebuild=False,
          verbose=False):
    """
    Compile firmware to a shared object hostsim can load.

    C goes straight to `hostsim_build.py`. C++ is lowered to plain C first by
    crust's `tools/cpprust.py` -- the C++ front end accepts a subset and
    *refuses* what it cannot lower rather than guessing, so a rejection here is
    a real answer about the source, not a gap in the toolchain.

    The result is cached next to the source's basename in /tmp and only rebuilt
    when the source is newer, since a robot rebuilt every test would spend more
    time in gcc than in the simulator.
    """
    root = crust_root()
    if not root:
        raise RuntimeError(why_unavailable())
    source = os.path.abspath(source)
    if not os.path.isfile(source):
        raise FileNotFoundError(source)

    stem = os.path.splitext(os.path.basename(source))[0]
    out = out or os.path.join('/tmp', 'robotsim-fw-%s.so' % stem)

    if (not rebuild and os.path.isfile(out)
            and os.path.getmtime(out) >= os.path.getmtime(source)):
        return out

    csource = source
    if source.endswith(('.cpp', '.cc', '.cxx', '.C')):
        csource = os.path.join('/tmp', 'robotsim-fw-%s.c' % stem)
        cmd = [sys.executable, os.path.join(root, 'tools', 'cpprust.py'),
               source, '-o', csource]
        for d in defines:
            cmd += ['-D', d]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0 or not os.path.isfile(csource):
            raise RuntimeError('cpprust could not lower %s:\n%s%s'
                               % (source, proc.stdout, proc.stderr))

    cmd = [sys.executable, os.path.join(root, 'tools', 'hostsim_build.py'),
           csource, '-o', out, '--cc', cc]
    for d in defines:
        cmd += ['-D', d]
    for l in libs:
        cmd += ['-l', l]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not os.path.isfile(out):
        raise RuntimeError('firmware build failed for %s:\n%s%s'
                           % (source, proc.stdout, proc.stderr))
    if verbose:
        print(proc.stdout.rstrip())
    return out


# ---------------------------------------------------------------------------
# a board
# ---------------------------------------------------------------------------

class Board:
    """
    One simulated MCU, stepped in lockstep with the simulator.

    Wraps hostsim's `Sim` and adds the two things robotsim needs from it: time
    in seconds rather than counter ticks, and a console that accumulates instead
    of being lost if nobody reads it that tick.
    """

    def __init__(self, so, name=None, freq=DEFAULT_FREQ, echo=False,
                 loop_hz=1000):
        hostsim = _import_hostsim()
        self.sim = hostsim.Sim(so, freq=freq, name=name)
        self.name = self.sim.name
        self.freq = freq
        self.echo = echo
        ## The firmware's control-loop rate, and the size of a grant.
        ##
        ## This matters more than it looks. hostsim's timer_count() consumes
        ## the *whole* grant in one call -- it sets `now` to `deadline` -- so a
        ## firmware delay loop exits immediately however much time was granted,
        ## and the board executes exactly one loop iteration per grant. Handing
        ## it a whole robotsim tick at once would run a 1 kHz control loop at
        ## the tick rate (60 Hz) while the firmware's own arithmetic still
        ## believed it was 1 kHz: a control loop silently running 16x slow.
        ##
        ## So time is granted in whole loop periods instead, and the board gets
        ## the number of iterations its rate calls for.
        self.loop_hz = loop_hz
        self.grant = max(1, int(freq // loop_hz)) if loop_hz else None
        self.grants = 0
        self.lines = []          ## everything the firmware has printed
        self._partial = ''
        ## Fractional counter ticks carried between steps. dt is 1/60 s and the
        ## counter runs at 19.2 MHz, so a step is 320000.0 ticks here but will
        ## not be for every dt -- truncating each step would drift the board's
        ## clock away from the plant's silently and forever.
        self._debt = 0.0
        self.started = False
        self.steps = 0

    def __repr__(self):
        return '<Board %s t=%.3fs>' % (self.name, self.elapsed)

    # -- time ---------------------------------------------------------------

    def start(self):
        if not self.started:
            self.sim.start()
            self.started = True
            self.drain()
        return self

    def step(self, dt):
        """
        Advance the board by `dt` seconds of virtual time.

        hostsim advances only when told to, which is what makes a run
        repeatable regardless of host load -- and what lets the board and the
        plant share one clock instead of racing.
        """
        if not self.started:
            self.start()
        self._debt += self.freq * dt
        if self.grant:
            ## Whole loop periods only; the remainder is carried, so the board's
            ## clock lags the plant by less than one period and never drifts.
            while self._debt >= self.grant:
                self.sim.step(self.grant)
                self._debt -= self.grant
                self.grants += 1
                if self.sim.finished:
                    break
        else:
            whole = int(self._debt)
            self._debt -= whole
            if whole > 0:
                self.sim.step(whole)
                self.grants += 1
        self.steps += 1
        self.drain()
        return self

    @property
    def elapsed(self):
        """Virtual seconds the board has run."""
        return self.sim.now / float(self.freq)

    @property
    def finished(self):
        return self.sim.finished

    # -- console ------------------------------------------------------------

    def drain(self):
        """
        Collect console output into `lines`.

        Read every step whether or not anyone wants it: hostsim's console is a
        buffer, and firmware that prints steadily would otherwise overrun it and
        lose exactly the output that explains what went wrong.
        """
        text = self.sim.read()
        if not text:
            return []
        if self.echo:
            print(text, end='')
        self._partial += text
        fresh = self._partial.split('\n')
        self._partial = fresh.pop()
        self.lines.extend(fresh)
        return fresh

    @property
    def console(self):
        return '\n'.join(self.lines)

    def feed(self, text):
        self.sim.feed(text)
        return self

    # -- actuators and sensors ---------------------------------------------

    @property
    def motor_duty(self):
        return self.sim.motor_duty

    @property
    def encoder(self):
        return self.sim.encoder

    @encoder.setter
    def encoder(self, counts):
        self.sim.encoder = counts

    @property
    def target(self):
        return self.sim.target

    @target.setter
    def target(self, counts):
        self.sim.target = counts

    # -- link, frames, faults ----------------------------------------------

    def link_pop_all(self):
        return self.sim.link_pop_all()

    def link_push(self, data):
        return self.sim.link_push(data)

    @property
    def link_stats(self):
        return self.sim.link_stats

    def push_frame(self, data):
        """
        Hand the board one camera frame.

        One frame of slack, not a queue: an uncollected frame is overwritten as
        a camera DMAing into a double buffer would overwrite it. That is the
        same duty-cycle the cameras already model with `camera_interval`, and
        `frames_overwritten` counts what the firmware missed.
        """
        self.sim.push_frame(data)
        return self

    @property
    def frames(self):
        return {'pushed': self.sim.frames_pushed,
                'overwritten': self.sim.frames_overwritten,
                'size': self.sim.frame_size}

    def fault_encoder_stuck(self, on=True):
        """Shaft turns, sensor stops reporting -- the hard one to diagnose."""
        self.sim.fault_encoder_stuck(on)
        return self

    def fault_encoder_bias(self, counts):
        self.sim.fault_encoder_bias(counts)
        return self

    def fault_link_down(self, on=True):
        self.sim.fault_link_down(on)
        return self

    def fault_link_drop_every(self, n):
        self.sim.fault_link_drop_every(n)
        return self

    def close(self):
        self.sim.close()


# ---------------------------------------------------------------------------
# a bus between boards
# ---------------------------------------------------------------------------

class Network:
    """
    Boards that can talk to each other.

    Routing is crust's `Fleet`, used as it was designed to be: `deliver()` is
    standalone and needs only participants with a `name`, `link_pop_all()` and
    `link_push()`. What is *not* used is `Fleet.step()` -- robotsim owns the
    clock, because the boards have to advance in step with the plant, not on
    their own schedule.

    Delivery happens once every board has reached the same virtual time, so a
    message sent during a tick arrives at the start of the next one. That
    one-step latency is deliberate: it is roughly what a real link costs, and it
    stops results depending on the order boards happen to be listed in.

    `owner` is the robot whose `step()` should deliver this network. A network
    spanning several robots has no owner, because delivering it from inside one
    robot's step would route messages before the other robots had caught up --
    exactly the same-virtual-time invariant the one-step latency exists to
    preserve. Those are delivered by RobotSim.update(), or by hand.
    """

    def __init__(self, name='bus', owner=None, router=None, endpoints=()):
        hostsim = _import_hostsim()
        self.name = name
        self.owner = owner
        self.boards = []
        self._fleet = hostsim.Fleet(sims=[], router=router,
                                    endpoints=list(endpoints))
        self.delivered = 0

    def __repr__(self):
        return '<Network %s boards=%d delivered=%d>' % (
            self.name, len(self.boards), self.delivered)

    def add(self, board):
        """Put a board on the bus. Routing sees the underlying hostsim Sim."""
        if board in self.boards:
            return board
        self.boards.append(board)
        self._fleet.add(board.sim)
        return board

    def remove(self, board):
        if board in self.boards:
            self.boards.remove(board)
            self._fleet.sims.remove(board.sim)
        return board

    def add_endpoint(self, endpoint):
        """
        Add something that routes like a board but is not one.

        Anything with a `name`, `link_push(message)` and `link_pop_all()` will
        do -- a socket bridge, a recorder, a fake peer. crust's
        `hostsim_net.SocketBridge` is one, which is how a simulated fleet joins
        a real network.
        """
        return self._fleet.add_endpoint(endpoint)

    def by_name(self, name):
        """The Board with this name, or the raw participant if it is not one."""
        for board in self.boards:
            if board.name == name:
                return board
        return self._fleet.by_name(name)

    def deliver(self):
        """
        Route everything sent during the tick just finished.

        Call once per tick, after every board on the network has stepped.
        """
        moved = self._fleet.deliver()
        self.delivered += moved
        return moved

    @property
    def undelivered(self):
        """Messages that reached no recipient, rather than vanishing."""
        return self._fleet.undelivered

    @property
    def router(self):
        return self._fleet.router

    @router.setter
    def router(self, fn):
        """
        Set the routing rule. Called as router(fleet, sender, message) and
        returning (recipient, message) pairs, or None to drop.

        Recipients are hostsim Sims, not Boards -- `sim_of()` converts.
        """
        self._fleet.router = fn

    def sim_of(self, board):
        return board.sim

    def board_of(self, sim):
        for board in self.boards:
            if board.sim is sim:
                return board
        return None


def connect(*robots, **kw):
    """
    Put several robots' boards on one bus, for robot-to-robot messaging.

    The returned network has no owner, so no single robot's step() delivers it.
    Register it with RobotSim (`RobotSim.networks.append(net)`) or call
    `deliver()` by hand after stepping every robot -- delivering earlier would
    route messages before the other robots had reached the same virtual time.
    """
    net = Network(name=kw.pop('name', 'fleet'), owner=None, **kw)
    for robot in robots:
        for board in robot.boards:
            net.add(board)
        ## Point the robot at the shared bus so boards attached later join it,
        ## and so its own step() stops delivering (it is not the owner).
        robot._network = net
    return net


def point_to_point(target_name):
    """
    A router that sends everything to one named participant.

    The default is broadcast, which is right for a small bus and wrong as soon
    as a board should not overhear its neighbours -- so this is the common case
    worth having ready.
    """
    def route(fleet, sender, message):
        try:
            recipient = fleet.by_name(target_name)
        except KeyError:
            return None
        if recipient is sender:
            return None
        return [(recipient, message)]
    return route


# ---------------------------------------------------------------------------
# binding a board to a robot
# ---------------------------------------------------------------------------

class FirmwareDrive:
    """
    Wires a board's motor output and encoder input to a robot's base.

    Duck-typed on Robot so this module needs no `bpy` and can be imported and
    reasoned about outside Blender.

    Each tick runs in this order, and the order is the point:

        apply()   firmware's last duty becomes this tick's drive command
        <plant>   the simulator moves the robot, or fails to
        sense()   what the wheels did becomes the encoder reading

    Sensing after the plant rather than before is what lets the firmware see the
    consequence of its own command on the tick it lands, instead of a tick late.

    `source` decides what the encoder measures, and the default is the honest
    one:

      'wheel'  the wheels' own rotation, which is what a shaft encoder reads.
               A robot held against a wall still turns its wheels, so the
               encoder keeps counting and the firmware believes it is moving.
               That is a real failure mode and firmware should have to face it.
      'body'   ground-truth distance travelled -- perfect odometry no hardware
               has. Useful to isolate a control bug from a sensing one.
    """

    def __init__(self, board, robot, counts_per_metre=1000.0,
                 top_speed=None, duty_full=1000.0, source='wheel'):
        assert source in ('wheel', 'body'), source
        self.board = board
        self.robot = robot
        self.counts_per_metre = float(counts_per_metre)
        self.duty_full = float(duty_full)
        ## Duty is a fraction of full scale, so full duty must mean some speed.
        self.top_speed = float(top_speed if top_speed is not None
                               else getattr(robot.drive, 'max_speed', 2.0))
        self.source = source
        self.origin = self._raw()
        self.enabled = True

    def __repr__(self):
        return '<FirmwareDrive %s %s %.0f counts/m>' % (
            self.board.name, self.source, self.counts_per_metre)

    # -- measurement --------------------------------------------------------

    def _raw(self):
        if self.source == 'body':
            loc = self.robot.root.location
            return ('body', float(loc.x), float(loc.y))
        ## Wheel roll angle, averaged over the driven wheels. _roll_wheel
        ## accumulates on rotation_euler.x without wrapping, so this is a true
        ## odometer rather than an angle modulo a turn.
        driven = [w for w in self.robot.wheel_list if w.driven] or self.robot.wheel_list
        if not driven:
            return ('wheel', 0.0)
        total = sum(w.obj.rotation_euler.x for w in driven)
        return ('wheel', total / len(driven))

    def distance(self):
        """Metres travelled since the binding was made, as the encoder sees it."""
        now = self._raw()
        if now[0] == 'body':
            _tag, x0, y0 = self.origin
            _tag, x1, y1 = now
            ## Signed along the robot's heading, so reversing unwinds the count
            ## instead of adding to it.
            return _signed_distance(self.robot, x1 - x0, y1 - y0)
        radius = self.robot.wheel_radius
        ## Rolling forward is a negative rotation about +X; see _roll_wheel.
        return -(now[1] - self.origin[1]) * radius

    # -- the two halves of a tick -------------------------------------------

    def apply(self):
        """Firmware -> plant."""
        if not self.enabled:
            return
        speed = self.board.motor_duty / self.duty_full * self.top_speed
        self.robot.drive.drive(speed)

    def sense(self):
        """Plant -> firmware."""
        if not self.enabled:
            return
        self.board.encoder = int(round(self.distance() * self.counts_per_metre))


def _signed_distance(robot, dx, dy):
    """Displacement projected onto the robot's heading."""
    import math
    yaw = float(robot.root.rotation_euler.z)
    ## +Y forward, so the heading is (-sin yaw, cos yaw); see drive.body_to_world.
    return dx * -math.sin(yaw) + dy * math.cos(yaw)
