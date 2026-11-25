"""One episode on the FR3: URAI's session and host, with one arm, its cameras and its words."""

from __future__ import annotations

from ...errors import Refused
from ..urai import session as urai_session
from ..urai.host import PUBLIC, UraiHost
from ..urai.session import UraiSession
from . import texts

#: HexaShell's camera names -> the service's
CAMERAS = {"external": "external", "wrist": "wrist"}


class FrankaHost(UraiHost):
    cameras = CAMERAS
    plan_camera = "external"
    texts = texts
    #: gentle=True: slow enough to see and to stop for a contact (m/s)
    gentle_speed_m_s = 0.03
    #: one arm: no both
    public = tuple(name for name in PUBLIC if name != "both")

    def check_cam(self, cam) -> str:
        if cam not in self.cameras:
            raise Refused(f"camera must be one of {', '.join(self.cameras)}, not {cam!r}")
        return cam


class FrankaSession(UraiSession):
    bench = "franka"
    proxy = "hexashell.real.franka.proxy"
    host_class = FrankaHost
    texts = texts


def main(argv: list[str] | None = None) -> int:
    return urai_session.main(argv, prog="python -m hexashell.real.franka", service="FR3", env="FRANKA",
                             session_class=FrankaSession)
