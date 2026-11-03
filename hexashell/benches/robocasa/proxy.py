"""The ``robot`` object a cell sees in RoboCasa: a one-arm mobile robot (a Panda arm with a parallel
gripper on a wheeled base) in a kitchen.  Every call travels to the host process, which runs the
simulator and the perception models; results come back as plain types.

Units: metres, degrees.  Start frame: the base's place at the start of the episode - origin on the
floor under it, x forward, y to its left, z up; it stays fixed when the base drives.  Pixels are
(x, y) = (column, row) from the top-left corner.
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
    def goto(self, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, speed=None, dry_run=False):
        """Move the grasp point (between the finger pads) to ``xyz`` with the given angles, in a
        straight line.

        xyz: a point in the start frame; ``by=(dx, dy, dz)`` moves relative to it (or to the current
        point when xyz is None).  pitch/yaw/roll (degrees) left out keep the ones last asked for:
        pitch 0 points level, 90 straight down; yaw is the heading of the pointing direction (0
        forward along +x, 90 to the left); pointing straight down, yaw turns the fingers about the
        vertical (yaw 0 closes them along y, yaw 90 along x); roll turns them about the pointing
        direction.  grip: an opening ('open', 'close' or 0..1) the gripper starts moving to as the
        motion starts.  speed: at most this many m/s (0.01 to 0.25, the most and the default), the
        turning slowed in proportion.  dry_run=True only checks reachability (no motion, no steps).
        Returns a Move (.ok, .reached, .blocked, .at, .angles, .steps).  A pose the arm cannot reach
        from where the base is raises NotExecuted (nothing moves); a dry run reports it instead.
        """
        return self._call("goto", (xyz,), dict(by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip,
                                               speed=speed, dry_run=dry_run))

    def arc(self, center, angle, *, axis=(0, 0, 1), grip=None, speed=None, dry_run=False):
        """Swing the grasp point by ``angle`` degrees around the line through ``center`` along
        ``axis`` ('x', 'y', 'z' or a direction), turning the hand with it by the same angle about
        the same axis - the path of a door handle about its hinge.  + turns counter-clockwise seen
        from the axis's tip (about 'z': seen from above, so a point ahead of ``center`` swings to
        the left).  speed as for goto, but holding something 0.25 m or more from the line (a door's
        handle) it defaults to 0.1 until arcs holding it have turned it 30 degrees about that line:
        faster, the bar slips out of the fingers as the door starts to open.  Returns a Move;
        raises NotExecuted when the arc leaves the arm's reach.  At most 180 degrees at a time.
        """
        return self._call("arc", (center, angle), dict(axis=axis, grip=grip, speed=speed, dry_run=dry_run))

    def push_arc(self, center, angle, *, axis=(0, 0, 1), push=None, press=0.03, tilt=25, dry_run=False):
        """Push a hinged part the hand touches (a lid, a head, a door's free edge) by up to ``angle``
        degrees around its hinge, the line through ``center`` along ``axis`` (as for arc).  The hand
        first goes along ``push`` (a direction into the part; None: along the hand's circle about the
        hinge, the way the angle turns) until it meets the part, then keeps pressing ``press`` m
        (0.01-0.05) past where it is, leaned ``tilt`` degrees (0-60) from ``push`` toward the way the
        part turns, the push turning with the part as the hand's track shows it turn; it presses
        harder while the part turns slowly.  Nothing needs to be held.  Returns a Push
        (.turned: degrees the hand went around the hinge, .stop: 'angle' when it got all the way,
        else 'stalled', 'slid', 'no touch', 'jammed', 'stretched', ...); raises NotExecuted when it
        did not start.
        """
        return self._call("push_arc", (center, angle),
                          dict(axis=axis, push=push, press=press, tilt=tilt, dry_run=dry_run))

    def rim_pick(self, target, *, pitch=45, inset=0.012, under=0.015, lift=0.10, outer=False, dry_run=False):
        """Pick a round open dish (a bowl, a deep plate, a pot without handles) by the near side of
        its rim: ``target`` is what measure takes (a Thing, a Mask, words, (cam, (x, y))) or a point
        (x, y, z) on it.  From the depth points around it, it finds the rim's outer edge on the side
        facing the base and its height, then comes down at ``pitch`` degrees (30-60), roll 90, onto
        a point ``inset`` m in from that edge and ``under`` m under the rim's top, pinches the wall,
        checks the fingers closed on it (if not, once more nearer the edge) and lifts ``lift`` m.
        The dish hangs level from the pinch while the hand keeps that pitch and roll: carry it so,
        and set it down by moving the hand, not by turning it.  The dish within 50 degrees of the
        base's heading and the base about 0.6 m from its middle (else it says how to drive).
        outer=True takes the outermost rim on that side, not the highest: a stack's bottom dish (a
        smaller dish set in it shows its rim higher, a little further in, and is what it would
        pinch).  dry_run=True says where it would pinch and whether the arm gets there.  Returns a RimPick
        (.holding, .width_m, .center, .radius, .top, .pinch); raises NotExecuted when it did not
        start."""
        return self._call("rim_pick", (target,),
                          dict(pitch=pitch, inset=inset, under=under, lift=lift, outer=outer, dry_run=dry_run))

    def gripper(self, opening):
        """Open or close the gripper: 'open', 'close', 'light' or 0..1 (1 = fully open, 8 cm between
        the pads).  Returns a Grip (.holding: True when the fingers stopped on something, False,
        None while they are still moving; .width_m).  Closing keeps squeezing until changed;
        'light' closes, then sets the fingers a little under what they stopped on (small soft food:
        squeezed hard, it slides out along the fingers while carried)."""
        return self._call("gripper", (opening,), {})

    def home(self):
        """Lift (if low) and bring the arm back to its start pose relative to the base.  Returns a
        Homing.  The gripper is left as it is."""
        return self._call("home", (), {})

    def wait(self, n):
        """Hold everything still for n steps (one step = 0.05 s of simulated time), n <= 400."""
        return self._call("wait", (n,), {})

    def drive(self, forward=0.0, left=0.0, turn=0.0, *, to=None, heading=None, about=None, dry_run=False):
        """Drive the base: ``forward`` / ``left`` (m) and ``turn`` (degrees, + to the left) from
        where it is, or ``to=(x, y)`` and/or ``heading=`` (degrees) in the start frame.  The arm
        keeps its pose relative to the base (it moves with it).  ``turn`` with ``about=(x, y)``
        turns the base about that floor point of the start frame instead of about itself: the base
        and the hand go round it on circles (with the hand closed on a door's handle and ``about`` at
        its hinge, the door swings with them).  dry_run=True only says where the base and the hand
        would end (and, with ``about``, which way the hand first goes): no motion, no steps.
        Returns a Drive (.reached, .blocked, .at, .heading)."""
        return self._call("drive", (), dict(forward=forward, left=left, turn=turn, to=to, heading=heading,
                                            about=about, dry_run=dry_run))

    # -- perception (free: no steps) -------------------------------------------------------------
    def find(self, cam, text=None, *, at=None, points=None, labels=None, box=None):
        """Outline objects in one camera's current picture with SAM3.

        cam: 'left', 'right' or 'wrist'.  text: a short noun phrase ('red mug'); returns every
        instance found, best first.  at=(x, y): the object under that pixel.  points=[(x, y), ...]
        with labels=[1, 0, ...] (1 = on the object, 0 = not on it).  box=(x0, y0, x1, y1).
        Returns a list of Mask.
        """
        return self._call("find", (cam, text), dict(at=at, points=points, labels=labels, box=box))

    def measure(self, target, *, cams="auto"):
        """Measure an object in 3-D from the depth of the cameras that see it.

        target: words ('red mug'), a Mask, (cam, (x, y)), a list of Masks of the same object in
        different cameras, or a Thing measured before (re-measured where it is now).  cams: 'auto'
        (every camera that sees it) or a list of names.  Returns a Thing (.center, .top, .size,
        .grip_yaw, .views).  Only the surfaces the cameras see are measured.
        """
        return self._call("measure", (target,), dict(cams=cams))

    def to_world(self, cam, xy, *, z=None, pair=None):
        """The point seen at pixel ``xy`` of camera ``cam``, from the depth image.  z=0.9: where the
        pixel's ray meets the plane z = 0.9 instead.  pair=(cam2, (x2, y2)): triangulate with the
        same point in a second camera.  Returns a Spot (.xyz, .source, .warning)."""
        return self._call("to_world", (cam, xy), dict(z=z, pair=pair))

    def to_pixel(self, xyz, cam):
        """Pixel (x, y) where point ``xyz`` appears in ``cam`` now, or None if out of view."""
        return self._call("to_pixel", (xyz, cam), {})

    # -- reading ---------------------------------------------------------------------------------
    @property
    def task(self):
        """The task: instruction, step budget (used / left), the arm's start pose."""
        return self._call("task", (), {})

    def state(self):
        """The robot now: grasp point and angles, gripper and holding, base position and heading,
        distance from the arm's start pose; the step budget."""
        return self._call("state", (), {})

    def image(self, cam):
        """The current RGB picture of a camera as a numpy array (512, 512, 3), uint8."""
        return self._call("image", (cam,), {})

    def depth(self, cam):
        """The current depth picture of a camera (512, 512), float32: metres along the camera's
        axis."""
        return self._call("depth", (cam,), {})

    def look(self, cams="all", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False,
             note=None):
        """Attach the current picture(s) to this cell's result, so you can see them.

        cams: 'left', 'right', 'wrist', a list of them, or 'all'.  grid=True: coordinate grid with
        pixel labels.  marks: points to draw, each (x, y) or (x, y, 'label'), or 3-D points as
        ('world', (x, y, z), 'label').  masks: Masks or Things to outline.  crop=(x0, y0, x1, y1) and
        zoom (e.g. 3): enlarge a region; labels keep the full picture's pixel coordinates.
        tile=True: the cameras side by side in one image.  Pictures are taken at the moment of the
        call.
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
        return ("robot: goto, arc, push_arc, gripper, home, wait, drive | find, measure, to_world, to_pixel | "
                "task, state, image, depth, look, camera, things")
