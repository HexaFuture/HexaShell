"""The ``robot`` object a cell sees in a RoboColiseum Season II episode on an AgiBot G2 (two seven-joint
arms on a turning torso, a two-finger gripper on each: the omnipicker, or the ctek90d with its fingers on
swinging links; or a Wuji five-finger hand on each), or on another of the season's robots (the task card says
which): RoboDojo's robot calls, with the same names and result types (:mod:`hexashell.robot.proxy`), plus
``waist``, ``hand`` on five-finger hands and, where the base drives (the task card says), ``drive``; the
docstrings below say what differs.

Every motion call (goto, gripper, hand, home, wait, waist, drive, body, head) is one reply to the benchmark,
which counts them per stage of the task: a stage done in fewer motions scores more.  Looking and measuring cost
nothing.

Units: metres, degrees.  World frame: the robot's base - x forward (away from the robot), y to its
left, z up; it moves only when the base drives, and then everything measured before is stale.  Pixels are (x, y) = (column, row) from the top-left
corner.
"""

from __future__ import annotations

from ...robot import proxy as _dojo
from ...robot.proxy import Blocked, Dropped, NotExecuted, RobotError  # noqa: F401 - the exceptions a cell may catch


class Robot(_dojo.Robot):
    # -- motion (each call is one motion: one reply to the benchmark) ------------------------------
    def goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto",
             dry_run=False, contact=False, speed=None):
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
        (no motion; not counted).  Returns a Move (.reached, .blocked, .lag_mm, .at, .angles, .steps).
        A pose that cannot be reached, or no path there, raises NotExecuted (nothing moves, not
        counted; a dry run reports it instead).  The whole move is sent at once and runs to its end;
        the grasp point ending 2 cm or more short of its command (something is in the way, and the arm
        keeps pressing on it until the next motion) raises Blocked, what the gripper held slipping out
        on the way raises Dropped.  contact=True (a press, a push, an insertion meant to meet
        resistance): no Blocked error.
        """
        # speed= (m/s, the grasp point's top pace for this move) is left out of the docstring above on purpose:
        # help() shows it to every task, and only the write task's skill tells it (pen strokes)
        return self._call("goto", (arm, xyz), dict(by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip,
                                                    path=path, dry_run=dry_run, contact=contact, speed=speed))

    def gripper(self, arm, opening):
        """Open or close a gripper: 'open', 'close' or 0..1 (1 = fully open: about 6.8 cm between the
        pads on the omnipicker, 9.0 cm on the ctek90d).  Returns a Grip (.opening, .width_m, .holding:
        True when a close stopped on something, False when it closed fully, None after an open).
        Closing keeps squeezing until changed.  The ctek90d's pads move about 1.8 cm forward as they
        close (they swing on links): its fingertips reach 2.2 cm past the grasp point open, 4 cm
        closed.  On five-finger hands (the task card says) it sets a pinch's or a wrap's opening, as
        hand(arm, opening=...) does, and returns what hand() returns."""
        return self._call("gripper", (arm, opening), {})

    def hand(self, arm, shape=None, opening=None, *, fingers=None, dry_run=False):
        """Five-finger hands only (the task card says): set a hand's shape, in one motion; the arm stays.

        shape: 'flat' (every finger straight: the start), 'pinch' (the thumb's pad against the index
        finger's), 'wrap' (the four fingers curled round a bar across the palm, the thumb along the
        index), 'point' (the index straight, the others curled); None keeps the shape.
        opening: a pinch's or a wrap's, 'open', 'close' or 0..1 (a change of shape without one opens
        it fully; the task card gives the gaps).
        fingers: {finger: a curl 0..1 (0 straight, 1 curled), or its four joint angles in degrees}, the
        fingers being thumb, index, middle, ring, pinky; set on top of the shape until it changes.
        The arm's grasp point (the point goto places, and its pitch, yaw and roll) is the shape's
        working point: the middle of the palm (flat), where the pads meet (pinch), the middle of the
        curled fingers (wrap), the index fingertip (point); the result says where it is now.
        Returns a Hand (.holding: True when the holding fingers stopped on something - the pinch's
        thumb and index, at least two of the wrap's four -, False when they closed on nothing, None
        for flat and point; .touching: the fingers stopped short of where they were sent; .at,
        .angles: the grasp point).  Closed fingers keep squeezing until changed; a hold that slips
        later raises Dropped in that motion.  dry_run=True only says where the grasp point would be.
        """
        return self._call("hand", (arm, shape, opening), dict(fingers=fingers, dry_run=dry_run))

    def home(self, arm="both"):
        """Lift a hand that is low, then swing the arm (or both) back to its start pose.  Returns a
        Homing."""
        return self._call("home", (arm,), {})

    def wait(self, n):
        """Hold everything still for n steps (the task card says a step's simulated time), n <= 150."""
        return self._call("wait", (n,), {})

    def waist(self, yaw=None, *, by=None):
        """Turn the torso about the vertical: waist(yaw=degrees) to that heading (0 = the start's),
        waist(by=degrees) by that much; positive turns left.  Both arms turn with it, keeping their
        joints (the hands' world points and headings change: the text gives them).  Returns a text."""
        return self._call("waist", (), dict(yaw=yaw, by=by))

    def drive(self, forward=0.0, left=0.0, turn=0.0):
        """Where the base drives (the task card says): move it forward and left (m) and turn it (degrees, +
        to the left) from where it stands, in one motion - the arms keep their pose and ride along, so a hand
        held out meets walls and furniture first.  Nothing reads where the base got to: the text gives what
        was asked, and a note when a hand was pushed on the way.  The world frame moves with the base, so
        look, find and measure again before reaching.  Returns a text."""
        return self._call("drive", (), dict(forward=forward, left=left, turn=turn))

    def body(self, lean=None, height=None, *, dry_run=False):
        """The torso's posture (the robot whose base drives): lean it forward (degrees from upright, 0..75)
        and raise or lower the shoulders (z of the arm base, m), in one motion; either left out keeps its
        value now.  The arms keep their joints and ride along, so the hands move with the torso; a posture
        that would bring an arm near the floor on the way is refused.  dry_run=True: what it would do, no
        move.  body() without a posture: the posture and the head's now.  home('both') restores the start
        posture.  Returns a text."""
        return self._call("body", (), dict(lean=lean, height=height, dry_run=dry_run))

    def head(self, yaw=None, pitch=None):
        """Turn the head on the torso (yaw: degrees, + to the left, up to 44 either way) and tip it down
        (pitch: degrees down from the torso's line, 0..29), in one motion; the head camera moves with it,
        and the text says where it looks.  Either left out keeps its value now.  Returns a text."""
        return self._call("head", (), dict(yaw=yaw, pitch=pitch))

    # -- perception (free: no motion, except a one-step hold for a fresh picture when one is black) --
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
        """The task: the instruction, the start pose of each hand and the benchmark's own description of the task,
        step by step (a description, not the stages it judges)."""
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
        return ("robot: goto, gripper, home, wait, waist, drive | find, measure, to_world, to_pixel | "
                "task, state, image, look, camera, things")
