"""
Telemetry: recording what happened, and drawing it.

A Blender render shows where the robot ended up. It does not show why -- the
duty the firmware was commanding, the slip between wheels and ground, the moment
contact took the velocity away, the range that triggered the turn. Those are
numbers over time, and numbers over time want a plot.

    tel = Telemetry()
    tel.watch_robot(bot)
    tel.watch_board(fw.board)

    @RobotSim
    def tick(dt):
        tel.sample(RobotSim.time)

    tel.plot('/tmp/run.png')

Deliberately free of `bpy`, so it can be imported, unit-tested and used from an
ordinary Python process. It reaches into robots by duck typing, exactly as
firmware.py does.

matplotlib is **optional**, like crust. `available()` reports whether plotting
can happen, so a test skips with a message rather than failing on a Blender that
bundles its own Python without it. Recording works regardless -- only drawing
needs the library -- and `to_csv()` gets the data out either way.
"""

import os

#: Matplotlib is imported lazily and only when drawing. Importing it at module
#: scope would make a missing library break *recording* too, which does not need
#: it, and would pay the import cost for runs that never plot.
_pyplot = None


def available():
    """True when a plot can actually be drawn."""
    try:
        _import_pyplot()
        return True
    except Exception:
        return False


def why_unavailable():
    try:
        _import_pyplot()
        return ''
    except Exception as e:
        return 'matplotlib is not usable here (%s: %s)' % (type(e).__name__, e)


def _import_pyplot():
    global _pyplot
    if _pyplot is None:
        import matplotlib
        ## Agg before pyplot: there is no display in a headless Blender, and
        ## the default backend would try to find one and fail at import.
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        _pyplot = plt
    return _pyplot


def _num(value, default=0.0):
    """Coerce to a float, mapping None and inf to something plottable."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if value != value or value in (float('inf'), float('-inf')):
        return default
    return value


class Channel:
    """One recorded signal: a name, a way to read it, and where it is drawn."""

    def __init__(self, name, read, group=None, unit='', scale=1.0):
        self.name = name
        self.read = read
        ## Channels sharing a group share a subplot, which is what makes a
        ## panel readable: duty against encoder is noise, duty against duty
        ## limits is a story.
        self.group = group or name
        self.unit = unit
        self.scale = scale
        self.values = []

    def __repr__(self):
        return '<Channel %s group=%s n=%d>' % (self.name, self.group, len(self.values))

    def sample(self):
        value = _num(self.read()) * self.scale
        self.values.append(value)
        return value


class Telemetry:
    """
    Records channels once per tick and draws them as a stacked panel.

    Sampling is pull-based: the recorder holds a callable per channel and reads
    them all when told. That keeps the simulator free of any knowledge that
    telemetry exists, and means a custom signal is one lambda rather than a
    plumbing change.
    """

    def __init__(self, name='telemetry', console_of=None):
        self.name = name
        self.channels = []
        self.times = []
        ## Boards whose console output is shown under the plots. Firmware print
        ## statements are the other half of debugging: the plot says the duty
        ## saturated, the console says which branch decided that.
        self.consoles = list(console_of or [])
        self.events = []          ## (time, label), drawn as vertical markers

    def __repr__(self):
        return '<Telemetry %s channels=%d samples=%d>' % (
            self.name, len(self.channels), len(self.times))

    # -- declaring what to record -------------------------------------------

    def watch(self, name, read, group=None, unit='', scale=1.0):
        """Record an arbitrary signal. `read` is called once per sample."""
        channel = Channel(name, read, group=group, unit=unit, scale=scale)
        self.channels.append(channel)
        return channel

    def watch_robot(self, robot, prefix='', include=('pose', 'motion', 'contact')):
        """
        Record the standard signals a robot has.

        Grouped so the panel reads as a story rather than a wall: where it went,
        how fast, and what the world did about it.
        """
        p = prefix or getattr(robot.root, 'name', 'robot')
        if 'pose' in include:
            self.watch('%s x' % p, lambda: robot.root.location.x, group='position', unit='m')
            self.watch('%s y' % p, lambda: robot.root.location.y, group='position', unit='m')
            self.watch('%s z' % p, lambda: robot.root.location.z, group='height', unit='m')
        if 'motion' in include:
            self.watch('%s speed' % p, lambda: getattr(robot.drive, 'v', 0.0),
                       group='speed', unit='m/s')
            self.watch('%s slip' % p, lambda: getattr(robot.drive, 'slip', 0.0),
                       group='slip', unit='ratio')
        if 'contact' in include:
            self.watch('%s blocked' % p, lambda: _blocked(robot),
                       group='contact', unit='0/1')
            self.watch('%s pitch' % p, lambda: _degrees(robot.root.rotation_euler.x),
                       group='attitude', unit='deg')
        return self

    def watch_board(self, board, prefix=None):
        """Record what a firmware board is commanding and being told."""
        p = prefix or board.name
        self.watch('%s duty' % p, lambda: board.motor_duty, group='duty', unit='1/1000')
        self.watch('%s encoder' % p, lambda: board.encoder, group='encoder', unit='counts')
        self.watch('%s sent' % p, lambda: board.link_stats['sent'],
                   group='link', unit='messages')
        self.watch('%s dropped' % p, lambda: board.link_stats['dropped'],
                   group='link', unit='messages')
        if board not in self.consoles:
            self.consoles.append(board)
        return self

    def watch_lidar(self, lidar, prefix=None, sectors=None):
        """
        Record what the lidar last saw.

        Reads `last_scan` rather than taking a scan of its own: a sweep is cheap
        but not free, and telemetry must not change the thing it measures by
        making the sensor fire twice a tick.
        """
        p = prefix or getattr(lidar.mount, 'name', 'lidar')
        self.watch('%s nearest' % p, lambda: _min_range(lidar),
                   group='range', unit='m')
        for label, (a, b) in (sectors or {}).items():
            self.watch('%s %s' % (p, label),
                       (lambda a=a, b=b: _sector(lidar, a, b)),
                       group='range', unit='m')
        return self

    # -- recording ----------------------------------------------------------

    def sample(self, t=None):
        """Read every channel once. Call from the tick callback."""
        self.times.append(len(self.times) if t is None else float(t))
        for channel in self.channels:
            channel.sample()
        return self

    def mark(self, label, t=None):
        """Note an event, drawn as a vertical line across every plot."""
        self.events.append((float(t if t is not None else
                                  (self.times[-1] if self.times else 0.0)), label))
        return self

    @property
    def groups(self):
        """Channel groups, in the order they were first declared."""
        order = []
        for channel in self.channels:
            if channel.group not in order:
                order.append(channel.group)
        return order

    def series(self, name):
        for channel in self.channels:
            if channel.name == name:
                return channel.values
        raise KeyError(name)

    # -- output -------------------------------------------------------------

    def to_csv(self, path):
        """Write the raw samples. Works with no matplotlib at all."""
        with open(path, 'w') as fh:
            fh.write(','.join(['t'] + [c.name for c in self.channels]) + '\n')
            for i, t in enumerate(self.times):
                row = [repr(t)] + [repr(c.values[i]) for c in self.channels]
                fh.write(','.join(row) + '\n')
        return path

    def plot(self, path, title=None, width=9.0, row_height=1.35, console_lines=12,
             dpi=110, min_span=1e-3):
        """
        Draw every group as a stacked subplot sharing one time axis, with the
        firmware console underneath.

        A shared x axis is the point: the whole question a panel answers is
        "what else was happening when this happened", and that only reads if
        every row lines up.
        """
        plt = _import_pyplot()
        if not self.times:
            raise RuntimeError('nothing recorded: call sample() during the run')

        groups = self.groups
        console_text = self._console_text(console_lines)
        rows = len(groups) + (1 if console_text else 0)
        height = max(2.0, row_height * rows)

        fig, axes = plt.subplots(rows, 1, figsize=(width, height), dpi=dpi,
                                 sharex=False)
        if rows == 1:
            axes = [axes]

        for index, group in enumerate(groups):
            ax = axes[index]
            for channel in self.channels:
                if channel.group != group:
                    continue
                ax.plot(self.times, channel.values, linewidth=1.2,
                        label=channel.name)
            ## A signal that barely moves would otherwise be auto-scaled into
            ## pure noise: a robot sitting flat at 0.15 m gets a height axis
            ## spanning 1e-4 and looks like it is bouncing. Give every plot a
            ## floor on its span so flat reads as flat.
            values = [v for c in self.channels if c.group == group for v in c.values]
            limits = span_limits(values, min_span)
            if limits:
                ax.set_ylim(*limits)

            unit = next((c.unit for c in self.channels if c.group == group), '')
            ax.set_ylabel('%s\n%s' % (group, unit) if unit else group, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(alpha=0.25, linewidth=0.5)
            ax.legend(fontsize=6, loc='upper right', framealpha=0.6)
            for when, label in self.events:
                ax.axvline(when, color='0.4', linestyle='--', linewidth=0.8)
                if index == 0:
                    ax.annotate(label, (when, ax.get_ylim()[1]), fontsize=6,
                                rotation=90, va='top', ha='right', color='0.3')
            ## Only the bottom plot gets tick labels, so the stack stays dense.
            if index < len(groups) - 1:
                ax.set_xticklabels([])
            else:
                ax.set_xlabel('simulated seconds', fontsize=8)

        if console_text:
            ax = axes[-1]
            ax.axis('off')
            ax.text(0.0, 1.0, console_text, fontsize=6, family='monospace',
                    va='top', ha='left', transform=ax.transAxes)

        fig.suptitle(title or self.name, fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        fig.savefig(path)
        plt.close(fig)
        return path

    def _console_text(self, limit):
        chunks = []
        for board in self.consoles:
            lines = getattr(board, 'lines', [])
            if not lines:
                continue
            tail = lines[-limit:]
            chunks.append('%s:' % board.name)
            chunks.extend('  ' + line for line in tail)
        return '\n'.join(chunks)


def compose(render, panel, out, gap=8, background=(255, 255, 255)):
    """
    Stack a Blender render above a telemetry panel into one image.

    Separate files are two things to look at; one file is a frame with its
    explanation attached. Needs PIL, which Blender ships.
    """
    from PIL import Image
    top = Image.open(render).convert('RGB')
    bottom = Image.open(panel).convert('RGB')
    width = max(top.width, bottom.width)

    def fit(image):
        if image.width == width:
            return image
        height = max(1, int(image.height * width / float(image.width)))
        return image.resize((width, height), Image.LANCZOS)

    top, bottom = fit(top), fit(bottom)
    canvas = Image.new('RGB', (width, top.height + gap + bottom.height), background)
    canvas.paste(top, (0, 0))
    canvas.paste(bottom, (0, top.height + gap))
    canvas.save(out)
    return out


# ---------------------------------------------------------------------------
# readers used by the standard channel sets
# ---------------------------------------------------------------------------

def span_limits(values, min_span=1e-3):
    """
    Axis limits for a signal that barely moves, or None to let matplotlib decide.

    A robot sitting flat at 0.15 m would otherwise get a height axis spanning
    1e-4 and look like it is bouncing. Giving every plot a floor on its span is
    what makes flat read as flat.
    """
    if not values:
        return None
    low, high = min(values), max(values)
    if high - low >= min_span:
        return None
    middle = (high + low) * 0.5
    return (middle - min_span, middle + min_span)


def _blocked(robot):
    info = getattr(robot.drive, 'last_contact', None)
    return 1.0 if getattr(info, 'blocked', False) else 0.0


def _degrees(radians):
    import math
    return math.degrees(_num(radians))


def _min_range(lidar):
    scan = getattr(lidar, 'last_scan', None)
    ## No scan yet, or every beam missed: report the sensor's own maximum rather
    ## than infinity, which would rescale the whole axis and hide the data.
    if scan is None:
        return lidar.range_max
    value = scan.min_range
    return lidar.range_max if value == float('inf') else value


def _sector(lidar, a, b):
    scan = getattr(lidar, 'last_scan', None)
    if scan is None:
        return lidar.range_max
    value = scan.sector(a, b)
    return lidar.range_max if value == float('inf') else value
