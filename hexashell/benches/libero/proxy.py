"""The ``robot`` object a cell sees in LIBERO-PRO: a Panda arm with a parallel gripper whose base is
fixed beside a table (or on the floor).  Every call travels to the host process, which runs the
simulator and the perception models; results come back as plain types.

Units: metres, degrees.  Arm frame: origin at the arm's base plate, x forward (from the arm toward
the workspace), y to its left, z up; the robot never moves, so it holds all episode.  Pixels are
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

        xyz: a point in the arm frame; ``by=(dx, dy, dz)`` moves relative to it (or to the current
        point when xyz is None).  pitch/yaw/roll (degrees) left out keep the ones last asked for:
        pitch 0 points level, 90 straight down; yaw is the heading of the pointing direction (0
        forward along +x, 90 to the left); pointing straight down, yaw turns the fingers about the
        vertical (yaw 0 closes them along y, yaw 90 along x); roll turns them about the pointing
        direction.  grip: an opening ('open', 'close' or 0..1) the gripper starts moving to as the
        motion starts.  speed: at most this many m/s (0.01 to 0.25, the most and the default), the
        turning slowed in proportion.  dry_run=True only checks reachability (no motion, no steps).
        Returns a Move (.ok, .reached, .blocked, .at, .angles, .steps).  A pose the arm cannot reach
        raises NotExecuted (nothing moves); a dry run reports it instead.
        """
        return self._call("goto", (xyz,), dict(by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip,
                                               speed=speed, dry_run=dry_run))

    def arc(self, center, angle, *, axis=(0, 0, 1), grip=None, speed=None, dry_run=False):
        """Swing the grasp point by ``angle`` degrees around the line through ``center`` along
        ``axis`` ('x', 'y', 'z' or a direction), turning the hand with it by the same angle about
        the same axis - the path of a door handle about its hinge.  + turns counter-clockwise seen
        from the axis's tip (about 'z': seen from above, so a point ahead of ``center`` swings to
        the left).  speed as for goto.  Returns a Move; raises NotExecuted when the arc leaves the
        arm's reach.  At most 180 degrees at a time.
        """
        return self._call("arc", (center, angle), dict(axis=axis, grip=grip, speed=speed, dry_run=dry_run))

    def gripper(self, opening):
        """Open or close the gripper: 'open', 'close' or 0..1 (1 = fully open, 8 cm between the
        pads).  Returns a Grip (.holding: True when the fingers stopped on something, False,
        None while they are still moving; .width_m).  Closing keeps squeezing until changed."""
        return self._call("gripper", (opening,), {})

    def home(self):
        """Lift (if low) and bring the hand back to its start pose.  Returns a Homing.  The gripper
        is left as it is."""
        return self._call("home", (), {})

    def wait(self, n):
        """Hold everything still for n steps (one step = 0.05 s of simulated time), n <= 400."""
        return self._call("wait", (n,), {})

    # -- perception (free: no steps) -------------------------------------------------------------
    def find(self, cam, text=None, *, at=None, points=None, labels=None, box=None):
        """Outline objects in one camera's current picture with SAM3.

        cam: 'agentview' or 'wrist'.  text: a short noun phrase ('black bowl'); returns every
        instance found, best first.  at=(x, y): the object under that pixel.  points=[(x, y), ...]
        with labels=[1, 0, ...] (1 = on the object, 0 = not on it).  box=(x0, y0, x1, y1).
        Returns a list of Mask.
        """
        return self._call("find", (cam, text), dict(at=at, points=points, labels=labels, box=box))

    def measure(self, target, *, cams="auto"):
        """Measure an object in 3-D from the depth of the cameras that see it.

        target: words ('black bowl'), a Mask, (cam, (x, y)), a list of Masks of the same object in
        different cameras, or a Thing measured before (re-measured where it is now).  cams: 'auto'
        (every camera that sees it) or a list of names.  Returns a Thing (.center, .top, .size,
        .grip_yaw, .views).  Only the surfaces the cameras see are measured.
        """
        return self._call("measure", (target,), dict(cams=cams))

    def to_world(self, cam, xy, *, z=None, pair=None):
        """The point seen at pixel ``xy`` of camera ``cam``, from the depth image.  z=0.05: where the
        pixel's ray meets the plane z = 0.05 instead.  pair=(cam2, (x2, y2)): triangulate with the
        same point in a second camera.  Returns a Spot (.xyz, .source, .warning)."""
        return self._call("to_world", (cam, xy), dict(z=z, pair=pair))

    def to_pixel(self, xyz, cam):
        """Pixel (x, y) where point ``xyz`` appears in ``cam`` now, or None if out of view."""
        return self._call("to_pixel", (xyz, cam), {})

    # -- reading ---------------------------------------------------------------------------------
    @property
    def task(self):
        """The task: instruction, step budget (used / left), the hand's start pose."""
        return self._call("task", (), {})

    def state(self):
        """The robot now: grasp point and angles, gripper and holding, distance from the start pose;
        the step budget."""
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

        cams: 'agentview', 'wrist', a list of them, or 'all'.  grid=True: coordinate grid with pixel
        labels.  marks: points to draw, each (x, y) or (x, y, 'label'), or 3-D points as
        ('world', (x, y, z), 'label').  masks: Masks or Things to outline.  crop=(x0, y0, x1, y1) and
        zoom (e.g. 3): enlarge a region; labels keep the full picture's pixel coordinates.
        tile=True: the cameras side by side in one image.  Pictures are taken at the moment of the
        call.
        """
        return self._call("look", (cams,), dict(grid=grid, marks=marks, masks=masks, crop=crop, zoom=zoom,
                                                tile=tile, note=note))

    def camera(self, cam):
        """Camera model: focal length (px), image centre, pose in the arm frame now."""
        return self._call("camera", (cam,), {})

    @property
    def things(self):
        """Every object measured so far in this episode (latest measurement of each)."""
        return self._call("things", (), {})

    def __repr__(self) -> str:
        return ("robot: goto, arc, gripper, home, wait | find, measure, to_world, to_pixel | "
                "task, state, image, depth, look, camera, things")
