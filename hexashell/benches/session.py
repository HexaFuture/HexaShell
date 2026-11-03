"""The episode side of a benchmark whose simulator runs in a server process.

:class:`SimServer` starts ``python -m hexashell.benches.<bench>.server <fd>`` with the benchmark's
own Python (its packages and numpy differ from HexaShell's), passes it one end of a socket pair and
exchanges :mod:`.wire` messages with it; or, given an ``address``, it connects to a warm server
(:mod:`.daemon`) that is already running.  :class:`BenchSession` is the part of a
:class:`~hexashell.runtime.session.Session` all such benchmarks share: the server, the episode
status it reports with every reply, and the verdict.  :class:`ArmSession` adds what the one-arm
benchmarks share: the episode opened on the server, an :class:`~hexashell.benches.armhost.ArmHost`
answering the robot calls, the perception, the pictures and the per-step records.
"""

from __future__ import annotations

import os
import shlex
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from ..runtime.session import Attach, Session
from . import wire

ROOT = Path(__file__).resolve().parents[2]


class SimError(RuntimeError):
    """The simulator server failed a request or stopped; ``status`` is the episode status it sent with
    the failure, when it sent one."""

    def __init__(self, text: str, status: dict | None = None):
        super().__init__(text)
        self.status = status


class SimRefused(SimError):
    """The server would not carry out a request (nothing moved); the message says why."""


def read_env_file(path: str | os.PathLike | None) -> dict[str, str]:
    """``export NAME=value`` lines of a shell file (no expansion beyond ``$HOME``)."""
    out: dict[str, str] = {}
    if not path:
        return out
    for line in Path(path).expanduser().read_text().splitlines():
        line = line.strip()
        if line.startswith("export "):
            line = line[len("export "):]
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        parts = shlex.split(value, comments=True) if value else [""]  # an inline comment may hold a quote
        out[name.strip()] = os.path.expandvars(parts[0] if parts else "")
    return out


def check_module(module: str) -> str:
    if not module.startswith("hexashell.benches.") or not all(p.isidentifier() for p in module.split(".")):
        raise ValueError(f"not a bench server module: {module!r}")
    return module


def server_env(env_file: str | None, egl_device: str | None) -> dict[str, str]:
    """The environment a simulator server runs in: this one without Python settings and credentials,
    plus the benchmark's ``export`` lines, HexaShell on the path and the GPU that renders."""
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PYTHON", "VIRTUAL_ENV", "CONDA"))}
    for key in [k for k in env if "KEY" in k.upper() or "TOKEN" in k.upper() or "SECRET" in k.upper()]:
        env.pop(key)  # the simulator needs no credentials
    env.update(read_env_file(env_file))
    env["PYTHONPATH"] = str(ROOT)
    env["PYTHONUNBUFFERED"] = "1"
    if egl_device not in (None, ""):
        env["HEXA_EGL_DEVICE"] = str(egl_device)
    return env


class SimServer:
    #: a warm server: how long to wait for it to listen, and for it to take this episode (s)
    connect_s = 60.0
    hello_s = 600.0

    def __init__(self, module: str, python: str, *, env_file: str | None = None, egl_device: str | None = None,
                 log_path: Path | None = None, timeout_s: float = 900.0, address: str | None = None):
        self.module = check_module(module)
        self.python = python
        self.env_file = env_file
        self.egl_device = egl_device
        self.log_path = log_path
        self.timeout_s = timeout_s
        #: the unix socket of a warm server to use instead of starting one
        self.address = address
        #: what the warm server said when it took the episode (its pid, how many episodes it has served)
        self.warm: dict | None = None
        self.proc: subprocess.Popen | None = None
        self.sock: socket.socket | None = None
        self.lock = threading.Lock()
        self.calls = 0
        self.seconds = 0.0

    def start(self) -> None:
        if self.address:
            self._connect()
            return
        ours, theirs = socket.socketpair()
        env = server_env(self.env_file, self.egl_device)
        log = open(self.log_path, "ab") if self.log_path else subprocess.DEVNULL
        workdir = self.log_path.parent if self.log_path else ROOT
        try:
            self.proc = subprocess.Popen([self.python, "-m", self.module, str(theirs.fileno())],
                                         pass_fds=(theirs.fileno(),), env=env, cwd=str(workdir),
                                         stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        finally:
            theirs.close()
            if log is not subprocess.DEVNULL:
                log.close()
        ours.settimeout(self.timeout_s)
        self.sock = ours

    def _connect(self) -> None:
        """Take a warm server's next episode: its output goes to this episode's log meanwhile."""
        where = f"the warm simulator server at {self.address}"
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline = time.monotonic() + self.connect_s
        try:
            while True:
                try:
                    sock.connect(str(self.address))
                    break
                except (FileNotFoundError, ConnectionRefusedError) as exc:  # not listening yet
                    if time.monotonic() > deadline:
                        raise SimError(f"{where}: {exc}") from None
                    time.sleep(1.0)
            sock.settimeout(self.hello_s)
            log = str(Path(self.log_path).resolve()) if self.log_path else None
            wire.send(sock, {"op": "hello", "args": {"module": self.module, "log": log}})
            reply = wire.recv(sock)
        except SimError:
            sock.close()
            raise
        except (wire.WireError, OSError) as exc:  # socket.timeout is an OSError
            sock.close()
            raise SimError(f"{where}: {exc}") from None
        if not isinstance(reply, dict) or not reply.get("ok"):
            sock.close()
            raise SimError(f"{where}: {reply.get('error') if isinstance(reply, dict) else 'unreadable reply'}")
        self.warm = {k: reply[k] for k in ("pid", "episode") if k in reply}
        sock.settimeout(self.timeout_s)
        self.sock = sock

    def call(self, op: str, **args: Any) -> dict:
        """One request; returns the reply.  Raises SimRefused / SimError."""
        if self.sock is None:
            raise SimError("the simulator server is not running")
        t0 = time.monotonic()
        with self.lock:
            try:
                wire.send(self.sock, {"op": op, "args": args})
                reply = wire.recv(self.sock)
            except (wire.WireError, socket.timeout, OSError) as exc:
                code = self.proc.poll() if self.proc else None
                raise SimError(f"simulator server: {exc}" + (f" (exited with {code})" if code is not None else "")) \
                    from None
        self.calls += 1
        self.seconds += time.monotonic() - t0
        if not isinstance(reply, dict):
            raise SimError("simulator server: unreadable reply")
        if not reply.get("ok"):
            text = str(reply.get("error") or "failed")
            status = reply.get("status") if isinstance(reply.get("status"), dict) else None
            if reply.get("kind") == "refused":
                raise SimRefused(text, status)
            raise SimError(f"simulator server: {text}\n{reply.get('trace', '')}".rstrip(), status)
        return reply

    def close(self) -> None:
        if self.sock is not None:
            try:
                if self.address or (self.proc is not None and self.proc.poll() is None):
                    self.sock.settimeout(120.0 if self.address else 30.0)
                    wire.send(self.sock, {"op": "close", "args": {}})
                    wire.recv(self.sock)
            except Exception:  # noqa: BLE001
                pass
            self.sock.close()
            self.sock = None
        if self.proc is not None:
            try:
                self.proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=10)
            self.proc = None


class BenchSession(Session):
    """The server, the status it reports and the verdict; subclasses add the robot and the card."""

    server_module = ""
    #: how long one request to the server may take (s): the first ones import, build the scene and
    #: render the first pictures
    server_timeout_s = 900.0
    #: a warm server's unix socket (:mod:`.daemon`), when the episode should use one
    sim_socket: str | None = None

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None):
        super().__init__(info)
        self.python = python
        self.env_file = env_file
        self.egl_device = egl_device
        self.server: SimServer | None = None
        self.status: dict[str, Any] = {}
        self.finished = False
        #: why the simulator stopped working, when it did (the episode is then over)
        self.failure: str | None = None

    def start_server(self, log_path: Path | None) -> None:
        self.server = SimServer(self.server_module, self.python, env_file=self.env_file, egl_device=self.egl_device,
                                log_path=log_path, timeout_s=self.server_timeout_s, address=self.sim_socket)
        self.server.start()

    def server_facts(self) -> dict[str, Any]:
        """For the record of the episode's opening: the warm server that runs it, if one does."""
        warm = getattr(self.server, "warm", None)
        return {"warm_server": warm} if warm else {}

    def call(self, op: str, **args: Any) -> dict:
        try:
            reply = self.server.call(op, **args)
        except SimError as exc:
            if exc.status and op != "open":  # a failed open may report the server's previous episode
                self.status = dict(exc.status)
            raise
        if reply.get("status"):
            self.status = dict(reply["status"])
        return reply

    def fail(self, text: str) -> None:
        """The simulator failed: the episode is over, the scene judged as last reported."""
        self.failure = text
        self.finished = True
        self.errors.append(f"simulator: {text}")

    # -- Session ---------------------------------------------------------------------------------
    @property
    def over(self) -> bool:
        return self.finished or bool(self.status.get("over"))

    @property
    def step(self) -> int | None:
        return self.status.get("step")

    @property
    def step_limit(self) -> int | None:
        return self.status.get("horizon")

    @property
    def verdict(self) -> dict | None:
        if not self.status:
            return None
        verdict = {"success": bool(self.status.get("success")), "step": self.status.get("step"),
                   "step_limit": self.status.get("horizon"), "finished": self.finished}
        if self.failure:
            verdict["failure"] = self.failure
        return verdict

    def conclude(self, timeout_s: float) -> dict | None:
        self.finished = True  # nothing moves after this; the scene is judged as it is
        return self.verdict

    def close(self) -> None:
        if self.server is not None:
            self.server.close()
            self.server = None


class ArmSession(BenchSession):
    """One episode of a one-arm benchmark.  Subclasses set ``host_class`` (an ArmHost) and
    ``server_module`` and give ``open_args`` (what the server's open takes besides the picture size),
    the names, the card and the result fields."""

    host_class: type | None = None

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None, size: int = 512):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device)
        self.size = int(size)
        self.rec = None
        self.host = None
        self.facts: dict[str, Any] = {}
        self.first: dict[str, np.ndarray] = {}

    def open_args(self) -> dict[str, Any]:
        raise NotImplementedError

    @property
    def cams(self) -> tuple[str, ...]:
        return tuple(self.host_class.cams)

    def check_cam(self, cam) -> str:
        from .armhost import camera_checker

        return camera_checker(self.host_class.cams, self.host_class.aliases)(cam)

    # -- lifetime ------------------------------------------------------------------------------
    def open(self, settings, rec, attach: Attach) -> None:
        self.rec = rec
        self.start_server(Path(rec.dir) / "server.log")
        opened = self.call("open", size=self.size, **self.open_args())
        self.facts = dict(opened["facts"])
        rec.event({"kind": "sim_open", **{k: v for k, v in opened.items() if k not in ("ok", "status")},
                   **self.server_facts()})
        scene = self._scene(settings)
        self.host = self.host_class(self, opened, scene=scene, log=rec.event, attach=attach, on_rows=self._rows)
        self.perception = scene is not None
        self.first = self.host.pictures()

    def _scene(self, settings):
        if not settings.sam3_url:
            return None
        try:
            from ..vision.rgbd import DepthScene

            return DepthScene(settings.sam3_url, cams=list(self.cams), check_cam=self.check_cam, log=self.rec.event)
        except Exception as exc:  # noqa: BLE001 - run without perception, and say so
            self.errors.append(f"perception unavailable: {exc}")
            self.rec.event({"kind": "perception_unavailable", "error": str(exc)})
            return None

    def close(self) -> None:
        if self.host is not None and self.host.scene is not None:
            try:
                self.host.scene.close()
            except Exception:  # noqa: BLE001
                pass
        super().close()

    # -- during the episode ----------------------------------------------------------------------
    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        return self.host.dispatch(name, args, kwargs)

    @property
    def instruction(self) -> str | None:
        return self.facts.get("instruction")

    def over_reason(self) -> str:
        from .armhost import describe

        return describe(self.status, finished=self.finished, failure=self.failure)

    def over_text(self) -> str:
        return f"The episode is over: {self.over_reason()}."

    def frames(self) -> dict[str, np.ndarray]:
        if self.failure or self.host is None:
            return {}
        try:
            return self.host.pictures()
        except Exception as exc:  # noqa: BLE001 - a record, not worth the episode
            self.rec.event({"kind": "frames_failed", "error": str(exc)})
            return {}

    def start_pictures(self) -> list[tuple[str, np.ndarray]]:
        return [(cam, self.first[cam]) for cam in self.cams if cam in self.first]

    # -- records ---------------------------------------------------------------------------------
    def row_extra(self, rest: np.ndarray) -> dict[str, Any]:
        """Fields of a state record from what the server adds to a log row."""
        return {}

    def _rows(self, rows: np.ndarray) -> None:
        for row in np.asarray(rows, float):
            self.rec.state({"step": int(row[0]), "tcp": [round(v, 5) for v in row[1:4]],
                            "rotvec": [round(v, 5) for v in row[4:7]], "width": round(float(row[7]), 5),
                            **self.row_extra(row[8:])})
