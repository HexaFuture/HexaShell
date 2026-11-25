"""The ``robot`` object a cell sees in DexJoCo-X: Franka Panda arms (one on the right, or a right and a left)
with five-fingered Wuji hands, fixed in front of a table, in MuJoCo.  Every call travels to the host process,
which runs the simulator and the perception models; results come back as plain types.

Units: metres, degrees.  Scene frame: z up, the robot faces +x, y to its left; it is fixed.  Pixels are (x, y) =
(column, row) from the top-left corner.  One step is 0.05 s of simulated time.

A hand: the palm's face has a centre (point 'palm') and a normal (the way the palm faces); the fingers run from
the palm along 'finger'.  What a closed hand holds sits at its grasp point (point 'grasp'), about 5 cm out from
the palm's face at the finger roots.  Fingertips (the middle of each pad) are the points 'thumb', 'index',
'middle', 'ring', 'little'.  The hands have no touch sensors: a finger that something outside the hand pushes
on presses on something (felt in its joints), and a hand whose thumb and one more finger press on something holds
it.
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
    def hand(self, side, xyz=None, *, by=None, point="palm", facing=None, normal=None, finger=None, along=None,
             look_at=None, approach=None, speed=None, dry_run=False):
        """Move one hand ('right', or 'left' in the two-arm tasks) so that its ``point`` goes in a straight line
        to ``xyz`` (scene frame; a Spot, a Thing's centre, a result's point); ``by=(dx, dy, dz)`` moves from
        ``xyz`` (or from where the point is now when xyz is None).  ``point``: 'palm' (the centre of the palm's
        face), 'grasp' (where a closed hand holds things), a fingertip ('thumb', 'index', 'middle', 'ring',
        'little') or 'camera' (the wrist camera).  Orientation, turned evenly on the way: ``facing`` one of 'down'
        (fingers forward), 'up', 'forward' (fingers down), 'inward' (the palm toward the robot's middle),
        'outward'; or ``normal`` (the way the palm faces) and/or ``finger`` (the way the fingers point) as
        scene-frame directions; or ``along``, for a power grip: the direction of a handle, pointing to where the
        thumb goes; the hand turns so that the handle lies across its fingers at the slant a fist holds it (with
        ``normal`` or ``facing`` as the palm's way about the handle); or, with point='camera', ``look_at=xyz``:
        the hand turns so that its wrist camera looks at that point (the picture's top upward, or for a view
        steeply down away from the robot, when the arm reaches that).  Left out, the hand keeps its
        orientation.  ``approach=d`` (m, up to 0.25): the hand first goes to the goal set
        back d along the palm's normal, then comes the last d in along it slowly, so the fingers do not sweep
        into what they will hold.  ``speed``: m/s (0.01 to 0.25; the task card gives the arm's default and its most).
        A motion that something holds up stops there and says so.  dry_run=True only checks whether the arm
        reaches (no motion, no steps).  Returns a DexterousMove (.reached, .at, .err_mm, .blocked, .steps,
        .notes)."""
        return self._call("hand", (side, xyz), dict(by=by, point=point, facing=facing, normal=normal, finger=finger,
                                                    along=along, look_at=look_at, approach=approach, speed=speed,
                                                    dry_run=dry_run))

    def fingers(self, side, shape=None, *, amount=1.0, thumb=None, index=None, middle=None, ring=None, little=None,
                oppose=None, spread=None, seconds=0.5):
        """Set one hand's fingers: a named ``shape`` and/or each finger's curl, moving evenly over ``seconds``
        (0.1 to 3).  Shapes: 'open' (the start: fingers straight, thumb beside them), 'flat' (all straight),
        'power' (a fist round a handle), 'wrap' (fingers round a box or a can), 'hook' (fingers bent as a hook,
        thumb in: a basket's handle), 'pinch' (thumb tip to index tip: a small thing), 'tripod' (thumb, index and
        middle tips together), 'point' (index straight, the others curled: pressing a button or a key).
        ``amount`` (0 to 1): that share of the way from open to the shape.  Per finger, 0 (straight) to 1
        (curled in): ``index``, ``middle``, ``ring``, ``little``, and ``thumb`` (its curl); ``oppose`` (0 to 1)
        swings the thumb across the palm, ``spread`` (-1 to 1) fans the four fingers out (negative: together).
        Settings override the shape for those fingers.  Fingers that something stops keep pressing toward where
        they were set (that is a grip's squeeze).  Returns a DexterousFingers (.curl, .pressing: the fingers that
        press on something, .holding: the thumb and one more finger press on something)."""
        return self._call("fingers", (side, shape), dict(amount=amount, thumb=thumb, index=index, middle=middle,
                                                         ring=ring, little=little, oppose=oppose, spread=spread,
                                                         seconds=seconds))

    def arc(self, side, center, angle, *, axis=(0.0, 0.0, 1.0), speed=None):
        """Turn one hand ``angle`` degrees about the line through ``center`` along ``axis`` ('x', 'y', 'z' or a
        direction; positive: counter-clockwise seen from the axis's tip), the hand turning with it: a door
        about its hinge, a dial about its spindle.  ``speed``: degrees per second (the task card gives the arm's
        default and its most).  Returns a DexterousMove (.turned, .reached, .blocked)."""
        return self._call("arc", (side, center, angle), dict(axis=axis, speed=speed))

    def home(self, side=None):
        """Move a hand (or, with no side, every hand at once) back to its start pose; the fingers keep their
        setting.  Returns a DexterousHome."""
        return self._call("home", (side,), {})

    def wait(self, n):
        """Hold everything still for n steps (one step = 0.05 s of simulated time), n <= 400."""
        return self._call("wait", (n,), {})

    # -- perception (free: no steps) -------------------------------------------------------------
    def find(self, cam, text=None, *, at=None, points=None, labels=None, box=None):
        """Outline objects in a camera's current picture with SAM3.

        cam: 'scene', 'right_wrist' or 'left_wrist' (the task's cameras).  text: a short noun phrase
        ('hammer'); returns every instance found, best first.  at=(x, y): the object under that pixel.
        points=[(x, y), ...] with labels=[1, 0, ...] (1 = on the object, 0 = not on it).  box=(x0, y0, x1, y1).
        Returns a list of Mask.
        """
        return self._call("find", (cam, text), dict(at=at, points=points, labels=labels, box=box))

    def measure(self, target, *, cams="auto"):
        """Measure an object in 3-D from the cameras' depth.

        target: words ('hammer'), a Mask, (cam, (x, y)), or a Thing measured before (re-measured where it is
        now).  Returns a Thing (.center, .top, .size, .views).  Only the surfaces the cameras see are measured.
        A thing a hand holds moves with that hand.
        """
        return self._call("measure", (target,), dict(cams=cams))

    def to_world(self, cam, xy, *, z=None, pair=None):
        """The point seen at pixel ``xy`` of camera ``cam``, from its depth image.  z=0.8: where the pixel's ray
        meets the plane z = 0.8 instead.  Returns a Spot (.xyz, .source, .warning)."""
        return self._call("to_world", (cam, xy), dict(z=z, pair=pair))

    def to_pixel(self, xyz, cam):
        """Pixel (x, y) where point ``xyz`` appears in camera ``cam`` now, or None if out of view."""
        return self._call("to_pixel", (xyz, cam), {})

    # -- reading ---------------------------------------------------------------------------------
    @property
    def task(self):
        """The task: instruction, step budget (used / left), the arms."""
        return self._call("task", (), {})

    def state(self):
        """Each hand now: palm point, normal and finger direction, grasp point, fingertips, the fingers' shape
        and curl, which fingers press on something, holding, the wrist camera; and the step budget."""
        return self._call("state", (), {})

    def image(self, cam):
        """The current RGB picture of a camera as a numpy array (512, 512, 3), uint8."""
        return self._call("image", (cam,), {})

    def depth(self, cam):
        """The current depth picture of a camera (512, 512), float32: metres along the camera's axis (0:
        nothing within 10 m)."""
        return self._call("depth", (cam,), {})

    def look(self, cams="all", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False,
             note=None):
        """Attach the current pictures to this cell's result, so you can see them.

        cams: a camera, a list of them, or 'all'.  grid=True: coordinate grid with pixel labels.  marks:
        points to draw, each (x, y) or (x, y, 'label'), or 3-D points as ('world', (x, y, z), 'label').  masks:
        Masks or Things to outline.  crop=(x0, y0, x1, y1) and zoom (e.g. 3): enlarge a region; labels keep the
        full picture's pixel coordinates.  tile=True: several cameras side by side in one picture.  Pictures
        are taken at the moment of the call.
        """
        return self._call("look", (cams,), dict(grid=grid, marks=marks, masks=masks, crop=crop, zoom=zoom,
                                                tile=tile, note=note))

    def camera(self, cam):
        """Camera model: focal length (px), image centre, pose in the scene frame now."""
        return self._call("camera", (cam,), {})

    @property
    def things(self):
        """Every object measured so far in this episode (latest measurement of each)."""
        return self._call("things", (), {})

    def __repr__(self) -> str:
        return ("robot: hand, fingers, arc, home, wait | find, measure, to_world, to_pixel | "
                "task, state, image, depth, look, camera, things")
