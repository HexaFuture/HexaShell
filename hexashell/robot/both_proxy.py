"""RoboDojo's ``robot`` with ``both``: the kernel's proxy module when the setting ``two_hands`` is on."""

from __future__ import annotations

from . import proxy


class Robot(proxy.Robot):
    __doc__ = proxy.Robot.__doc__

    def both(self, left=None, right=None, *, together=False):
        """Move both arms in the same steps: it costs the steps of the longer motion, not their sum.

        left, right: a dict of goto's arguments, e.g. dict(xyz=(x, y, z), pitch=90) or dict(by=(0, 0, 0.1)),
        or dict(grip='close') for only its gripper; leave one out to move one arm.
        together=True: both start and end in the same step, the shorter stretched to the longer - for two
        hands on one thing, or two ends of cloth.
        Returns a Both (.left, .right: Move or Grip, .steps).  If either cannot run, or the arms would come
        closer than 8 cm on the way, NotExecuted: nothing moves, no steps.  Either arm stalling or dropping
        what it holds stops both, and raises Blocked or Dropped as goto does.
        """
        return self._call("both", (), dict(left=left, right=right, together=together))
