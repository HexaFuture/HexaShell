"""The ``robot`` object a cell sees in a RoboColiseum episode (the AgiBot G2: two seven-joint arms on a
turning torso, a two-finger gripper on each): RoboDojo's robot calls, with the same names and result
types (:mod:`hexashell.robot.proxy`), plus ``waist``; the docstrings below say what differs.

Units: metres, degrees.  World frame: the robot's base - x forward (away from the robot), y to its
left, z up; it does not move during an episode.  Pixels are (x, y) = (column, row) from the top-left
corner.
"""

from __future__ import annotations

from ...robot import proxy as _dojo
from ...robot.proxy import Blocked, Dropped, NotExecuted, RobotError  # noqa: F401 - the exceptions a cell may catch


class Robot(_dojo.Robot):
    # -- motion (each costs steps) ---------------------------------------------------------------
    def goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto",
             dry_run=False, contact=False):
        """Move an arm's grasp point (between the finger pads) to ``xyz`` with the given angles.

        arm: 'left', 'right', or 'both' (both hands by the same ``by``, along straight lines, keeping
        their angles; nothing else is given with 'both'; returns a list of two Moves).
        xyz: world point; ``by=(dx, dy, dz)`` moves relative to it (or to the current point when xyz
        is None).  pitch/yaw/roll (degrees) left out keep the ones last asked for (at the start, the
        hand's own).  pitch: how far the fingers point below the horizontal (0 level, 90 straight
        down).  yaw: their heading (0 forward, +90 to the robot's left); pointing straight down, yaw
        turns the fingers about the vertical: at yaw 0 they close along y (left-right), at yaw 90
        along x.  roll: turn of the fingers about the way they point (both hands start near roll 0).
        With the gripper open and roll left out, the hand may arrive with its fingers half a turn
        round (the same grasp; the note says so).
        grip: an opening the gripper starts moving to as the motion starts (open on the approach with
        it; to let go of something held, use gripper() once there).
        path: 'auto' (a straight line of the grasp point, else a swing of the joints that keeps it
        close to that line), 'line', or 'swing' (any swing of the joints).  dry_run=True only checks
        (no motion, no steps).  Returns a Move (.reached, .blocked, .lag_mm, .at, .angles, .steps).
        A pose that cannot be reached, or no path there, raises NotExecuted (nothing moves; a dry run
        reports it instead).  A move that stalls against something stops there and raises Blocked;
        what the gripper held slipping out on the way raises Dropped.  contact=True (a press, a push,
        an insertion meant to meet resistance): no early stop and neither error.
        """
        return self._call("goto", (arm, xyz), dict(by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip,
                                                    path=path, dry_run=dry_run, contact=contact))

    def gripper(self, arm, opening):
        """Open or close a gripper: 'open', 'close' or 0..1 (1 = fully open, about 6.8 cm between the
        pads).  Returns a Grip (.opening, .width_m, .holding: True when a close stopped on something,
        False when it closed fully, None after an open).  Closing keeps squeezing until changed."""
        return self._call("gripper", (arm, opening), {})

    def home(self, arm="both"):
        """Lift a hand that is low, then swing the arm (or both) back to its start pose.  Returns a
        Homing."""
        return self._call("home", (arm,), {})

    def wait(self, n):
        """Hold everything still for n steps (one step = 1/30 s of simulated time), n <= 100."""
        return self._call("wait", (n,), {})

    def waist(self, yaw=None, *, by=None):
        """Turn the torso about the vertical: waist(yaw=degrees) to that heading (0 = the start's),
        waist(by=degrees) by that much; positive turns left.  Both arms turn with it, keeping their
        joints (the hands' world points and headings change: the text gives them).  Returns a text."""
        return self._call("waist", (), dict(yaw=yaw, by=by))

    # -- perception (free: no steps, except one to take depth pictures when the last had none) ---
    def find(self, cam="head", text=None, *, at=None, points=None, labels=None, box=None):
        """Outline objects in one camera's current picture with SAM3.

        cam: 'head', 'left_wrist' or 'right_wrist'.  text: a short noun phrase ('red cup'); returns
        every instance found, best first.  at=(x, y): the object under that pixel.
        points=[(x, y), ...] with labels=[1, 0, ...] (1 = on the object, 0 = not on it).
        box=(x0, y0, x1, y1).  Returns a list of Mask.
        """
        return self._call("find", (cam, text), dict(at=at, points=points, labels=labels, box=box))

    def measure(self, target, *, cams="auto"):
        """Measure an object in 3-D from the depth pictures of the cameras that see it.

        target: text ('red cup'), a Mask, (cam, (x, y)), a list of Masks of the same object in
        different cameras, or a Thing measured before (re-measured where it is now).
        cams: 'auto' (all cameras that see it) or a list of camera names.  Returns a Thing
        (.center, .top, .size, .grip_yaw, .views).  Only the surface the cameras see is measured:
        seen from one side, the centre lies toward the cameras by up to half the object's depth.
        """
        return self._call("measure", (target,), dict(cams=cams))

    def to_world(self, cam, xy, *, z=None, pair=None):
        """The world point seen at pixel ``xy`` of camera ``cam``.

        Default: the surface under the pixel, from the camera's depth picture.
        z=0.80: where the pixel's ray meets the plane z = 0.80.
        pair=(cam2, (x2, y2)): triangulate with the same point seen in a second camera.
        Returns a Spot (.xyz, .source, .warning).
        """
        return self._call("to_world", (cam, xy), dict(z=z, pair=pair))

    # -- reading ---------------------------------------------------------------------------------
    @property
    def task(self):
        """The task: the instruction and the start pose of each hand."""
        return self._call("task", (), {})

    def state(self):
        """Both arms now: grasp point, angles, gripper opening and holding, distance from the start
        pose, joints."""
        return self._call("state", (), {})

    def look(self, cams="head", *, grid=False, marks=None, masks=None, crop=None, zoom=None,
             tile=False, note=None):
        """Attach the current picture(s) to this cell's result, so you can see them.

        cams: a camera name or a list ('head', 'left_wrist', 'right_wrist'); 'all' = the three.
        grid=True: coordinate grid with pixel labels.  marks: points to draw, each (x, y) or
        (x, y, 'label'), or world points as ('world', (x, y, z), 'label').  masks: Masks or Things
        to outline.  crop=(x0, y0, x1, y1) and zoom (e.g. 3): enlarge a region; labels keep the
        full picture's pixel coordinates.  tile=True: the cameras side by side in one image.
        Pictures are those of the latest step.
        """
        return self._call("look", (cams,), dict(grid=grid, marks=marks, masks=masks, crop=crop, zoom=zoom,
                                                tile=tile, note=note))

    def __repr__(self) -> str:
        return ("robot: goto, gripper, home, wait, waist | find, measure, to_world, to_pixel | "
                "task, state, image, look, camera, things")
