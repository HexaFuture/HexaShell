"""A server for testing :class:`hexashell.benches.session.SimServer`: ``python -m hexashell.benches.echo <fd>``.

Operations: ``echo`` returns its arguments, ``refuse`` and ``crash`` fail the two ways a bench server
can, ``exit`` stops without answering, ``close`` ends.  Every reply carries a ``status`` that counts
the requests, as a bench server's episode status would.  :class:`Server` answers the same way as a
bench server module's ``Server`` does, for :mod:`hexashell.benches.daemon` (a warm server).
"""

from __future__ import annotations

import os
import socket
import sys

from . import wire


def serve(sock: socket.socket) -> None:
    count = 0
    while True:
        try:
            request = wire.recv(sock)
        except wire.WireError:
            return
        op, args = request.get("op"), request.get("args") or {}
        count += 1
        if op == "exit":
            os._exit(3)
        if op == "echo":
            reply = {"ok": True, "args": args, "pid": os.getpid()}
        elif op == "refuse":
            reply = {"ok": False, "kind": "refused", "error": "not like that"}
        elif op == "crash":
            reply = {"ok": False, "kind": "crash", "error": "ZeroDivisionError: division by zero",
                     "trace": "Traceback (most recent call last): ..."}
        else:
            reply = {"ok": True}
        reply["status"] = {"step": count, "horizon": 100, "success": False, "over": False}
        wire.send(sock, reply)
        if op == "close":
            return


PREPARED = []


def prepare() -> None:
    PREPARED.append(os.environ.get("HEXA_EGL_DEVICE"))


class Server:
    """The same operations, answered through :func:`hexashell.benches.armserver.serve`; ``echo`` also
    says what ``prepare`` saw and how many episodes this server has closed."""

    def __init__(self):
        self.warm = False
        self.count = 0
        self.closed = 0
        self.won = False

    def handle(self, op: str, args: dict) -> dict:
        from .armserver import Refused

        self.count += 1
        if op == "exit":
            os._exit(3)
        if op == "echo":
            print(f"echo {args}", flush=True)
            return {"args": args, "pid": os.getpid(), "prepared": PREPARED, "warm": self.warm, "closed": self.closed}
        if op == "refuse":
            raise Refused("not like that")
        if op == "crash":
            return {"x": 1 / 0}
        if op == "win":  # the episode's task is done (the status says success until a new one opens)
            self.won = True
        if op == "open":  # a scene that cannot be built keeps the last one, as a warm bench server does
            if args.get("fail"):
                raise RuntimeError("could not build the scene")
            self.won = False
        if op == "close":
            self.closed += 1
        return {}

    def status(self) -> dict:
        return {"step": self.count, "horizon": 100, "success": self.won, "over": False}


if __name__ == "__main__":
    serve(socket.socket(fileno=int(sys.argv[1])))
