"""The service over HTTP, with the standard library, so the robot's machine needs no web framework.

    python -m hexashell.real.franka.server --sim
    python -m hexashell.real.franka.server --rig rig.json [--allow-motion] [--host 127.0.0.1] [--port 7861]

Requests other than loopback's need ``Authorization: Bearer <FRANKA_TOKEN>``; a service bound elsewhere must have one.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from .service import Conflict, Forbidden, FrankaService

CAMERA_IMAGE = re.compile(r"/api/cameras/([A-Za-z0-9_-]+)/image\.jpg")


def make_handler(service: FrankaService, token: str | None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args) -> None:
            pass

        def _send(self, status: int, body: bytes, kind: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, value) -> None:
            self._send(status, json.dumps(value).encode(), "application/json")

        def _body(self) -> dict:
            length = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(length)) if length else {}

        def _answer(self, route) -> None:
            body = self._body()
            if token is not None and self.headers.get("Authorization") != f"Bearer {token}":
                self._json(401, {"detail": "a bearer token is required"})
                return
            url = urlsplit(self.path)
            try:
                route(url.path, parse_qs(url.query), body)
            except Conflict as exc:
                self._json(409, {"detail": str(exc)})
            except Forbidden as exc:
                self._json(403, {"detail": str(exc)})
            except (KeyError, ValueError, TypeError) as exc:
                self._json(400, {"detail": f"{type(exc).__name__}: {exc}"})
            except Exception as exc:  # noqa: BLE001 - the caller must learn why, and the service must stay up
                self._json(500, {"detail": f"{type(exc).__name__}: {exc}"})

        def _get(self, path, query, body) -> None:
            image = CAMERA_IMAGE.fullmatch(path)
            if path == "/api/state":
                self._json(200, service.state())
            elif path == "/api/harness/model":
                self._json(200, service.model())
            elif image or path == "/api/live.jpg":
                camera = image.group(1) if image else service.rig.plan_camera
                self._send(200, service.jpeg(camera), "image/jpeg")
            else:
                self._json(404, {"detail": f"no {path}"})

        def _post(self, path, query, body) -> None:
            if path == "/api/harness/observe":
                self._json(200, service.observe(body["camera"]))
            elif path == "/api/preview":
                self._json(200, service.preview(body, full=query.get("trajectory") == ["full"]))
            elif path == "/api/execute":
                self._json(200, service.execute(body))
            elif path == "/api/cancel":
                self._json(200, service.cancel())
            elif path == "/api/recover":
                self._json(200, service.recover())
            else:
                self._json(404, {"detail": f"no {path}"})

        def _put(self, path, query, body) -> None:
            if path == "/api/draft":
                self._json(200, service.put_draft(body))
            else:
                self._json(404, {"detail": f"no {path}"})

        def do_GET(self) -> None:  # noqa: N802 - the name http.server calls
            self._answer(self._get)

        def do_POST(self) -> None:  # noqa: N802
            self._answer(self._post)

        def do_PUT(self) -> None:  # noqa: N802
            self._answer(self._put)

    return Handler


def serve(service: FrankaService, host: str = "127.0.0.1", port: int = 7861, token: str | None = None):
    """A server not yet running; ``serve_forever`` runs it.  Port 0 takes a free one (``server_port``)."""
    if not ipaddress.ip_address(host).is_loopback and token is None:
        raise ValueError("a service bound beyond loopback needs a token (FRANKA_TOKEN)")
    return ThreadingHTTPServer((host, port), make_handler(service, token))


def build_service(*, rig_path: str | None, sim: bool, allow_motion: bool) -> FrankaService:
    if sim:
        from .simulated import SimulatedCamera, SimulatedDriver, simulated_rig

        rig = simulated_rig()
        driver = SimulatedDriver(rig.arm())
        cameras = {name: SimulatedCamera(mount, driver) for name, mount in rig.cameras.items()}
        return FrankaService(rig, driver, cameras, allow_motion=True)
    from .cameras import RealSenseCamera
    from .driver import FrankyDriver
    from .rig import load_rig

    rig = load_rig(rig_path)
    driver = FrankyDriver(rig.robot_ip, realtime=rig.realtime, dynamics_factor=rig.dynamics_factor,
                          gripper_speed_m_s=rig.gripper_speed_m_s, grasp_force_n=rig.grasp_force_n)
    cameras = {name: RealSenseCamera(mount) for name, mount in rig.cameras.items()}
    return FrankaService(rig, driver, cameras, allow_motion=allow_motion)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.real.franka.server", description=__doc__.split("\n")[0])
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--rig", help="The cell's JSON file (see hexashell.real.franka.rig)")
    source.add_argument("--sim", action="store_true", help="The kinematic simulator instead of a robot")
    ap.add_argument("--allow-motion", action="store_true",
                    help="Let the real robot move; without it the service only reads and plans")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7861)
    args = ap.parse_args(argv)
    server = serve(build_service(rig_path=args.rig, sim=args.sim, allow_motion=args.allow_motion),
                   args.host, args.port, os.environ.get("FRANKA_TOKEN"))
    print(f"serving on {args.host}:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
