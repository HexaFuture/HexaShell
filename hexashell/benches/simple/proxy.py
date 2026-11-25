"""The ``robot`` object a cell sees in SIMPLE: a Unitree G1 humanoid (1.3 m tall) with two three-fingered Dex3
hands and a gaze camera on its head, in a room with tables.  Every call travels to the host process, which
runs the simulator and the perception models; results come back as plain types.

Units: metres, degrees.  Start frame: where the robot stood at the start of the episode - origin on the
ground under the pelvis, x forward, y to its left, z up; it stays fixed while the robot walks.  Pixels are
(x, y) = (column, row) from the top-left corner.  One step is 0.02 s of simulated time.
"""

from __future__ import annotations

from typing import Any, Callable

from ...robot import results as _perception  # noqa: F401  (loaded before the sandbox closes)
from ...robot.proxy import NotExecuted, RobotError  # noqa: F401  (the names cells catch)
from .. import results as _results  # noqa: F401


class Robot:
    def __init__(self, call: Callable[[str, tuple, dict], Any]):
        self._call = call

    # -- motion (uses the step budget) -----------------------------------------------------------
    def body(self, height=None, *, pitch=None, yaw=None):
        """Set the body's posture, closed on its measurement and then held: ``height`` the pelvis's height
        above the ground (0.30 to 0.79 m; standing about 0.77), ``pitch`` the torso's forward lean (0 to 60
        degrees below the horizontal), ``yaw`` the torso's turn left of the pelvis (-45 to 45).  Left out, a
        part keeps what it was last asked for.  The legs' controller sets how low the pelvis really goes (a
        note says so when it stops above what was asked).  The arms keep their joints (they move with the
        torso) unless a hand command said otherwise (``hold``).  Returns a BodyMove (.reached, .height,
        .pitch, .yaw, .steps)."""
        return self._call("body", (height,), dict(pitch=pitch, yaw=yaw))

    def walk(self, x=None, y=None, *, heading=None, by=None, turn=None):
        """Walk the pelvis to ground point (x, y) of the start frame and turn to ``heading`` (degrees in the
        start frame, 0 along +x, + to the left; left out: as now).  ``by=(forward, left)``: from where the
        pelvis is now, in its own directions; ``turn``: degrees from the heading now (+ to the left).  At most
        4 m per call.  The pelvis ends within about 5 cm of the target (it walks up to three rounds); a walk
        that makes no progress (a table's edge in the way) stops and says so.  Hands held in the world
        (``hold='world'``) stay where they are only as far as the arms reach: use hold='pelvis' to carry.
        Returns a Walk (.reached, .blocked, .at, .heading, .steps)."""
        return self._call("walk", (x, y), dict(heading=heading, by=by, turn=turn))

    def hand(self, side, xyz=None, *, by=None, facing=None, normal=None, finger=None, seconds=1.0, via=None,
             hold="torso", dry_run=False):
        """Move one palm ('left' or 'right') in straight lines to ``xyz`` (the palm's centre, start frame);
        ``by=(dx, dy, dz)`` moves from ``xyz`` (or from where the palm is now when xyz is None).
        Orientation: ``facing`` one of 'down', 'up', 'forward', 'inward' (the palm toward the robot's middle),
        'outward', in the robot's own directions now; or ``normal`` (the way the palm faces) and/or
        ``finger`` (from the wrist toward the finger roots) as start-frame directions; left out, the palm
        keeps the orientation it has.  ``via=[p1, p2, ...]``: points to pass on the way (up to 11).
        ``seconds``: each segment's time (0.4 to 8); the arm's speed limit may take longer.  ``hold`` says
        what the palm does during later body commands and walks: 'torso' (the arm keeps its joints: the palm
        moves with the torso), 'world' (it stays where it is in the room), 'pelvis' (it moves with the pelvis:
        carrying).  dry_run=True only checks whether the arm reaches (no motion, no steps).
        Returns a HandMove (.reached, .at, .err_mm, .steps, .notes)."""
        return self._call("hand", (side, xyz), dict(by=by, facing=facing, normal=normal, finger=finger,
                                                    seconds=seconds, via=via, hold=hold, dry_run=dry_run))

    def fingers(self, side, closed):
        """Close (True or 'close') or open (False or 'open') one hand's fingers; 'wide' opens them with the
        thumb swung back from the fingers (out of the way of a hand coming around an object's end; close
        afterwards as usual).  Closing curls the fingers until they touch and then squeezes; each change takes
        0.5 s (AMO) or 1 s (GR00T's controller).  Returns a Fingers (.touching: the hand's parts touching something - 'thumb', 'index', 'middle',
        'palm'; .holding: the thumb and one more part touch)."""
        return self._call("fingers", (side, closed), {})

    def gaze(self, yaw=None, pitch=None, *, at=None):
        """Turn the gaze camera: ``yaw`` left of the torso's forward (-45 to 45 degrees), ``pitch`` down from
        the torso's horizontal (0 to 60); or ``at=(x, y, z)``, a start-frame point to look at (a Spot, a
        Thing).  It turns at 120 degrees per second; the robot holds still meanwhile.  The pictures show what
        it sees after the turn.  Returns a Gaze (.yaw, .pitch, .steps)."""
        return self._call("gaze", (yaw, pitch), dict(at=at))

    def wait(self, n):
        """Hold everything still for n steps (one step = 0.02 s of simulated time), n <= 400."""
        return self._call("wait", (n,), {})

    # -- perception (free: no steps) -------------------------------------------------------------
    def find(self, cam, text=None, *, at=None, points=None, labels=None, box=None):
        """Outline objects in the gaze camera's current picture with SAM3.

        cam: 'gaze'.  text: a short noun phrase ('cracker box'); returns every instance found, best first.
        at=(x, y): the object under that pixel.  points=[(x, y), ...] with labels=[1, 0, ...] (1 = on the
        object, 0 = not on it).  box=(x0, y0, x1, y1).  Returns a list of Mask.
        """
        return self._call("find", (cam, text), dict(at=at, points=points, labels=labels, box=box))

    def measure(self, target, *, cams="auto"):
        """Measure an object in 3-D from the gaze camera's depth.

        target: words ('cracker box'), a Mask, (cam, (x, y)), or a Thing measured before (re-measured where it
        is now).  Returns a Thing (.center, .top, .size, .views).  Only the surfaces the camera sees are
        measured.  A thing a hand holds moves with that hand.
        """
        return self._call("measure", (target,), dict(cams=cams))

    def to_world(self, cam, xy, *, z=None, pair=None):
        """The point seen at pixel ``xy`` of the gaze camera, from the depth image.  z=0.8: where the pixel's
        ray meets the plane z = 0.8 instead.  Returns a Spot (.xyz, .source, .warning)."""
        return self._call("to_world", (cam, xy), dict(z=z, pair=pair))

    def to_pixel(self, xyz, cam):
        """Pixel (x, y) where point ``xyz`` appears in the gaze camera now, or None if out of view."""
        return self._call("to_pixel", (xyz, cam), {})

    # -- reading ---------------------------------------------------------------------------------
    @property
    def task(self):
        """The task: instruction, step budget (used / left) and the official limit."""
        return self._call("task", (), {})

    def state(self):
        """The robot now: pelvis position, heading and height, torso pitch and yaw; each palm's point,
        normal and finger direction, fingers open or closed, what they touch, holding; the gaze's angles; the
        step budget."""
        return self._call("state", (), {})

    def image(self, cam):
        """The current RGB picture of the gaze camera as a numpy array (400, 640, 3), uint8."""
        return self._call("image", (cam,), {})

    def depth(self, cam):
        """The current depth picture of the gaze camera (400, 640), float32: metres along the camera's axis
        (0: nothing within 10 m)."""
        return self._call("depth", (cam,), {})

    def look(self, cams="all", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False,
             note=None):
        """Attach the current picture to this cell's result, so you can see it.

        cams: 'gaze' or 'all'.  grid=True: coordinate grid with pixel labels.  marks: points to draw, each
        (x, y) or (x, y, 'label'), or 3-D points as ('world', (x, y, z), 'label').  masks: Masks or Things to
        outline.  crop=(x0, y0, x1, y1) and zoom (e.g. 3): enlarge a region; labels keep the full picture's
        pixel coordinates.  Pictures are taken at the moment of the call.
        """
        return self._call("look", (cams,), dict(grid=grid, marks=marks, masks=masks, crop=crop, zoom=zoom,
                                                tile=tile, note=note))

    def camera(self, cam):
        """Camera model: focal length (px), image centre, pose in the start frame now."""
        return self._call("camera", (cam,), {})

    @property
    def things(self):
        """Every object measured so far in this episode (latest measurement of each)."""
        return self._call("things", (), {})

    def __repr__(self) -> str:
        return ("robot: body, walk, hand, fingers, gaze, wait | find, measure, to_world, to_pixel | "
                "task, state, image, depth, look, camera, things")
