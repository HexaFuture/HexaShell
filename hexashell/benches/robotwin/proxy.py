"""The ``robot`` object a cell sees in a RoboTwin 2.0 episode: RoboDojo's robot calls, with the same
names, arguments and results (:mod:`hexashell.robot.proxy`); the docstrings below say what differs.

Units: metres, degrees.  World frame: x to the robot's right, y away from the robot, z up; the
table top's height is on the task card (about 0.71-0.74).  Pixels are (x, y) = (column, row) from
the top-left corner.
"""

from __future__ import annotations

from ...robot import proxy as _dojo
from ...robot.proxy import NotExecuted, RobotError  # noqa: F401 - the exceptions a cell may catch


class Robot(_dojo.Robot):
    def gripper(self, arm, opening):
        """Open or close a gripper: 'open', 'close' or 0..1 (1 = fully open, ~9 cm between the pads).
        Returns a Grip (.opening, .width_m, .holding: True, False, or None when the fingers did not
        close).  Closing keeps squeezing until changed."""
        return self._call("gripper", (arm, opening), {})

    def wait(self, n):
        """Hold everything still for n steps (one step = 0.2 s of simulated time), n <= 200."""
        return self._call("wait", (n,), {})

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
