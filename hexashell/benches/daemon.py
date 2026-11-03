"""A warm simulator server: one process serves one episode after another.

    python -m hexashell.benches.daemon MODULE SOCKET [--max-episodes N] [--idle-s S]

Runs in the benchmark's own Python environment.  ``MODULE`` is a bench server module
(``hexashell.benches.robotwin.server``, ...): its ``prepare()`` runs once (the GPU to render on, from
``$HEXA_EGL_DEVICE``), then one ``Server()`` answers every episode.  Starting a simulator is what
costs - the imports, the first scene, the first ray-traced picture: many minutes on a busy machine -
and RoboTwin's own evaluation also plays its episodes one after another in one process.

Each connection on the unix socket ``SOCKET`` is one episode (:class:`~hexashell.benches.session.SimServer`
with ``address``): a ``hello`` naming the server module and the episode's log file (this process's
output goes there until the episode ends), then the episode's requests, answered as
:func:`hexashell.benches.armserver.serve` answers them, until ``close``.  An episode whose side goes
away without ``close`` is closed here.  One connection at a time; after ``N`` episodes, or ``S``
seconds without one, the process ends (a runner starts a fresh one when it needs one).  Started as
:func:`hexashell.benches.session.server_env` says (the benchmark's Python, its ``export`` lines, the GPU
in ``$HEXA_EGL_DEVICE``), e.g. by ``scripts/bench_pool.py``.
"""

from __future__ import annotations

import argparse
import contextlib
import importlib
import os
import socket
import stat
import sys
import time

from . import wire
from .armserver import serve


@contextlib.contextmanager
def output_to(path: str | None):
    """This process's stdout and stderr (file descriptors 1 and 2) go to ``path`` meanwhile."""
    if not path:
        yield
        return
    sys.stdout.flush()
    sys.stderr.flush()
    saved = os.dup(1), os.dup(2)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o660)
    try:
        os.dup2(fd, 1)
        os.dup2(fd, 2)
        yield
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os.dup2(saved[0], 1)
        os.dup2(saved[1], 2)
        for f in (fd, *saved):
            os.close(f)


def listen(path: str) -> socket.socket:
    """A unix socket at ``path`` that only this user can connect to (a stale one is replaced)."""
    with contextlib.suppress(FileNotFoundError):
        if stat.S_ISSOCK(os.lstat(path).st_mode):
            os.unlink(path)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    old = os.umask(0o077)
    try:
        sock.bind(path)
    finally:
        os.umask(old)
    sock.listen(4)
    return sock


def episode(conn: socket.socket, server, module: str, number: int) -> str | None:
    """Serve one connection; returns how it ended (for this process's log), None when it was no episode."""
    conn.settimeout(60.0)
    try:
        hello = wire.recv(conn)
    except (wire.WireError, OSError) as exc:
        print(f"{time.strftime('%F %T')} a connection without a hello ({exc})", flush=True)
        return None
    conn.settimeout(None)
    args = (hello.get("args") or {}) if isinstance(hello, dict) else {}
    if not isinstance(hello, dict) or hello.get("op") != "hello" or args.get("module") != module:
        with contextlib.suppress(wire.WireError, OSError):
            wire.send(conn, {"ok": False, "error": f"this is a warm server of {module} (pid {os.getpid()}); "
                                                   f"it was asked for {args.get('module')!r}"})
        print(f"{time.strftime('%F %T')} refused a hello for {args.get('module')!r}", flush=True)
        return None
    log = args.get("log")
    with output_to(str(log) if log else None):
        print(f"[warm server pid {os.getpid()}: episode {number} of this process]", flush=True)
        try:
            wire.send(conn, {"ok": True, "pid": os.getpid(), "episode": number, "status": server.status()})
        except wire.WireError as exc:
            return f"gone before the first request ({exc})"
        closed = serve(conn, server)
        if not closed:  # the episode's side went away: free the scene for the next one
            with contextlib.suppress(Exception):
                server.handle("close", {})
        sys.stdout.flush()
    return "closed" if closed else "the episode's side went away; closed here"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.daemon", description=__doc__.splitlines()[0])
    ap.add_argument("module")
    ap.add_argument("socket")
    ap.add_argument("--max-episodes", type=int, default=0, help="end after this many episodes (0: never)")
    ap.add_argument("--idle-s", type=float, default=0.0, help="end after this long without an episode (0: never)")
    args = ap.parse_args(argv)
    if not args.module.startswith("hexashell.benches.") or not all(p.isidentifier() for p in args.module.split(".")):
        ap.error(f"not a bench server module: {args.module!r}")
    t0 = time.time()
    mod = importlib.import_module(args.module)
    prepare = getattr(mod, "prepare", None)
    if prepare is not None:
        prepare()
    server = mod.Server()
    server.warm = True
    listener = listen(args.socket)
    listener.settimeout(args.idle_s if args.idle_s > 0 else None)
    print(f"{time.strftime('%F %T')} warm server {args.module} pid {os.getpid()} ready on {args.socket} "
          f"after {time.time() - t0:.1f} s", flush=True)
    number = 0
    try:
        while not args.max_episodes or number < args.max_episodes:
            try:
                conn, _ = listener.accept()
            except socket.timeout:
                print(f"{time.strftime('%F %T')} no episode for {args.idle_s:.0f} s: ending", flush=True)
                break
            t1 = time.time()
            with conn:
                how = episode(conn, server, args.module, number + 1)
            if how is not None:
                number += 1
                print(f"{time.strftime('%F %T')} episode {number}: {how} after {time.time() - t1:.0f} s",
                      flush=True)
    finally:
        listener.close()
        with contextlib.suppress(OSError):
            os.unlink(args.socket)
    return 0


if __name__ == "__main__":
    sys.exit(main())
