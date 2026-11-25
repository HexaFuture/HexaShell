"""The ``robot`` object a cell sees on a robot run by URAI: motion, images and pixel/world geometry.
The English docstrings below describe the URAI semantics shown by help(robot).

Positions are URAI's own world frame (metres, z up) and its TCP; the table's height, the bases and the
grippers' openings are on the task card (URAI's model of the robot).  Angles are HexaShell's pitch,
yaw and roll in degrees; pixels are (x, y) = (column, row).
"""

from __future__ import annotations

from ...robot import proxy as _dojo
from ...robot.proxy import Blocked, Dropped, NotExecuted, RobotError  # noqa: F401 - the exceptions a cell may catch


class RobotBase:
    # The calls that move one arm at a time; Robot adds both(). The FR3's robot (real/franka/proxy.py), one arm,
    # builds on this. Share transport and image helpers without inheriting the simulator's segmentation API.
    __init__ = _dojo.Robot.__init__
    to_pixel = _dojo.Robot.to_pixel
    task = _dojo.Robot.task
    image = _dojo.Robot.image
    look = _dojo.Robot.look

    def goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, hand_by=None, hand_turn=None,
             grip=None, path="auto", dry_run=False, contact=False, gentle=False):
        """Move the selected arm's URAI TCP (by default the fingertip, 142.5 mm along link6 +z; see the task card).

        arm is left/right; xyz accepts a point, Thing, Spot or ArmStatus; by is a world-frame offset in metres.
        Omitted pitch/yaw/roll values keep the previous request. At pitch 90 the tool points down;
        yaw 0/90 closes along world x/y. hand_by=(forward, left, up) in metres and hand_turn=(about forward,
        about left, about up) in degrees move relative to the hand, in the axes of the last requested
        orientation: forward is where the fingers point, left lies along the closing line, up completes them
        (world up for a level hand at roll 0). hand_by shifts the TCP along those axes; hand_turn turns the
        hand about the TCP by that rotation vector (direction = axis, length = angle, at most 170), so
        hand_turn=(a, 0, 0) rolls by a. Together, the shift is along the axes before the turn. They do not
        combine with xyz, by, pitch, yaw or roll. grip='open'/'close'/0..1 is sent at the start of the motion.
        path='line' is a strict Cartesian line; 'swing' is a joint-space curve; 'auto' tries the line first,
        then accepts a curve only within 3 cm of the line and at most 1 cm below the lower endpoint.
        URAI applies its IK, collision checks and time scaling.
        dry_run=True only plans, sends no control commands and consumes no action budget; Move.steps is
        the expected cost, and a refusal returns reached=False. An ordinary planning refusal raises
        NotExecuted; a structured empty_grasp result raises Dropped. Other execution errors retain their
        original messages. contact=True adds no harness stall check; URAI's own checks still apply.
        gentle=True caps the whole motion at the original H v1 endpoint limit of 0.5 m/s, without a faster
        middle; URAI's angular and joint limits still apply. The final pose is held without automatically
        returning home, raising the target or opening the gripper."""
        kwargs = dict(by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip, path=path, dry_run=dry_run,
                      contact=contact, gentle=gentle)
        # only when given: a call without them reaches the host as before
        kwargs.update({k: v for k, v in (("hand_by", hand_by), ("hand_turn", hand_turn)) if v is not None})
        return self._call("goto", (arm, xyz), kwargs)

    def gripper(self, arm, opening):
        """Hold the joints and change the opening: open/close/0..1, scaled by the model's maximum opening in mm.

        Uses draft -> preview -> execute and returns Grip: width_m is measured; holding is inferred from
        the commanded and measured openings. Closing on air is legal. There is no in-motion slip detection."""
        return self._call("gripper", (arm, opening), {})

    def home(self, arm="both"):
        """Plan through URAI back to the joints saved when the session opened, keeping the gripper unchanged.

        arm is left/right/both. Returns Homing."""
        return self._call("home", (arm,), {})

    def wait(self, n):
        """Hold the pose in URAI's execution loop for n*0.04 seconds, n=1..200.

        Waited.steps uses actual server-side execution time and may exceed n due to scheduling overhead."""
        return self._call("wait", (n,), {})

    def to_world(self, cam, xy, *, z=None, pair=None):
        """Recover a world point from the full K, camera pose and RGB-D. Returns Spot.

        Uses measured depth by default; missing depth gives xyz=None. z specifies a horizontal world plane;
        pair=(cam2,(u,v)) triangulates from two views."""
        return self._call("to_world", (cam, xy), dict(z=z, pair=pair))

    def state(self):
        """Read both arms' measured TCPs, pitch/yaw/roll, joints in radians, opening and inferred holding,
        offsets from their starting poses, and the action budget."""
        return self._call("state", (), {})

    def camera(self, cam="head"):
        """Read the camera's full K (fx/fy/cx/cy), image dimensions and current world pose.

        HexaShell camera coordinates look along -z, with +y towards the top of the image."""
        return self._call("camera", (cam,), {})

    def __repr__(self) -> str:
        return ("robot: goto, gripper, home, wait | to_world, to_pixel | "
                "task, state, image, look, camera")


class Robot(RobotBase):
    def both(self, left=None, right=None, *, together=False):
        """Move both arms in one URAI plan: they start at once and it costs the steps of the longer motion,
        not their sum.

        left, right: a dict of goto's arguments without dry_run, e.g. dict(xyz=(x, y, z), pitch=90),
        dict(by=(0, 0, 0.1)) or dict(hand_by=(0.02, 0, 0)), or dict(grip='close') for only its gripper;
        leave one out to move one arm.
        together=True: both also end in the same step, the arm that would end first slowed to the other - for
        two hands on one thing, or two ends of cloth; when it cannot be slowed so (a turn sets its pace),
        NotExecuted.
        Each arm is first planned alone, as goto's dry run (path='auto' chooses its path there); then URAI
        plans the two together, the arms' clearance checked at every sample. If either cannot run,
        NotExecuted: nothing moves, no steps. Returns a Both (.left, .right: Move or Grip, .steps). One URAI
        execution runs both arms: whatever stops it stops both, and an empty grasp raises Dropped as goto
        does."""
        return self._call("both", (), dict(left=left, right=right, together=together))

    def __repr__(self) -> str:
        return ("robot: goto, gripper, both, home, wait | to_world, to_pixel | "
                "task, state, image, look, camera")
