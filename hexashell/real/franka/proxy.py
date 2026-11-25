"""The ``robot`` object a cell sees on the FR3: one arm, so no arm argument; two cameras, ``external`` and ``wrist``.

Positions are the rig's world frame (metres, z up) and Franka's TCP between the finger tips; the table's height and the
hand's opening are on the task card.  Angles are HexaShell's pitch, yaw and roll in degrees; pixels are
(x, y) = (column, row).
"""

from __future__ import annotations

from ...robot.proxy import Blocked, Dropped, NotExecuted, RobotError  # noqa: F401 - the exceptions a cell may catch
from ..urai.proxy import RobotBase

ARM = "arm"


class Robot(RobotBase):
    def goto(self, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto", dry_run=False,
             contact=False, gentle=False):
        """Move the TCP to ``xyz`` (a point, Spot or ArmStatus; ``by`` is a world-frame offset in metres).

        Omitted pitch/yaw/roll values keep the previous request.  At pitch 90 the hand points down; yaw 0/90 closes
        along world x/y.  grip='open'/'close'/0..1 is done before the motion starts.
        path='line' is a strict Cartesian line; 'swing' is a joint-space curve; 'auto' tries the line first, then
        accepts a curve only within 3 cm of the line and at most 1 cm below the lower endpoint.  dry_run=True only
        plans, sends no command and consumes no budget; a refusal then returns reached=False, and otherwise raises
        NotExecuted.  gentle=True caps the motion at 0.03 m/s.  contact=True adds no stall check.  The final pose is
        held."""
        return self._call("goto", (ARM, xyz), dict(by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip, path=path,
                                                   dry_run=dry_run, contact=contact, gentle=gentle))

    def gripper(self, opening):
        """Change the opening: open/close/0..1 of the hand's maximum.  Closing stops at what it meets.

        Returns Grip: width_m is measured; holding is inferred from the commanded and measured openings."""
        return self._call("gripper", (ARM, opening), {})

    def home(self):
        """Go back to the joints saved when the session opened, keeping the gripper.  Returns Homing."""
        return self._call("home", (ARM,), {})

    def to_pixel(self, xyz, cam="external"):
        """Pixel (x, y) where world point ``xyz`` appears in ``cam``, or None if out of view."""
        return self._call("to_pixel", (xyz, cam), {})

    def image(self, cam="external"):
        """The current RGB picture of a camera as a numpy array (H, W, 3), uint8."""
        return self._call("image", (cam,), {})

    def camera(self, cam="external"):
        """The camera's full K (fx/fy/cx/cy), image size and current world pose.

        HexaShell camera coordinates look along -z, with +y towards the top of the image."""
        return self._call("camera", (cam,), {})

    def look(self, cams="external", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False, note=None):
        """Attach the current picture(s) of ``cams`` ('external', 'wrist', a list or 'all') to this cell's result.

        grid draws a pixel grid; marks=[('world', (x, y, z), 'label'), ...] marks points; crop=(x0, y0, x1, y1) keeps the
        original pixel coordinates; tile puts the pictures side by side."""
        return self._call("look", (cams,), dict(grid=grid, marks=marks, masks=masks, crop=crop, zoom=zoom, tile=tile,
                                                note=note))

    def state(self):
        """The measured TCP, pitch/yaw/roll, joints in radians, opening and inferred holding, the offset from the
        starting pose, and the action budget."""
        return self._call("state", (), {})

    def __repr__(self) -> str:
        return ("robot: goto, gripper, home, wait | to_world, to_pixel | "
                "task, state, image, look, camera")
