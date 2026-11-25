"""A local stand-in for the benchmark's VLM judge.

One task (manipulation, scoop_popcorn) is judged by a vision-language model: every so often its checker sends
camera pictures with the task's description to an OpenAI-style ``chat/completions`` endpoint and reads a
score from the answer (the simulation waits for it).  The official judge is qwen3-vl-plus.  Here the
benchmark process holds only a dummy key and points at a local endpoint (:mod:`.sim`); a :class:`Judge`
listens there and passes each request on to the gateway of the agent's settings, with its key and the judge
model below (gpt-6-astra, low reasoning).  The judge's prompt and pictures go through unchanged; the answer
comes back as one non-streamed chat completion.  It is not the official judge, so local scores of that task
are a reference only.

:mod:`.run` starts one per run of a VLM-judged config, on a free port, and stops it with the run.  For a
test: ``python -m hexashell.benches.robocoliseum1.judge --port 18997``.

Each request is logged (time, pictures, seconds, tokens, the answer; never a key, never the pictures).  The
benchmark's own client logs the key it holds - the dummy one.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from ...runtime.llm import Gateway, GatewayError

MODEL = "gpt-6-astra"
EFFORT = "low"
#: seconds one judgement may take
JUDGE_S = 300.0


class Judge:
    """One forwarder on ``127.0.0.1:<port>`` (0: a free one), serving from a thread once started."""

    def __init__(self, settings, *, model: str = MODEL, effort: str | None = EFFORT, port: int = 0,
                 log_path: str | Path | None = None):
        self.settings = settings
        self.model = model
        self.effort = effort
        self.log_path = Path(log_path).expanduser() if log_path else None
        self._lock = threading.Lock()
        self.requests = 0
        self.failures = 0
        self._server = ThreadingHTTPServer(("127.0.0.1", int(port)), _Handler)
        self._server.daemon_threads = True
        self._server.judge = self
        self.port = int(self._server.server_address[1])
        self.url = f"http://127.0.0.1:{self.port}/v1"
        self._thread: threading.Thread | None = None

    def log(self, record: dict) -> None:
        if self.log_path is None:
            return
        with self._lock:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps({"t": round(time.time(), 3), **record}, ensure_ascii=False, default=str) + "\n")

    def start(self) -> "Judge":
        self._thread = threading.Thread(target=self._server.serve_forever, name="judge", daemon=True)
        self._thread.start()
        self.log({"kind": "start", "port": self.port, "model": self.model, "effort": self.effort})
        return self

    def stop(self) -> None:
        if self._thread is not None:
            self._server.shutdown()
            self._thread.join(timeout=10)
        self._server.server_close()
        self.log({"kind": "stop", "requests": self.requests, "failures": self.failures})

    def answer(self, body: dict) -> tuple[int, dict]:
        messages = body.get("messages") or []
        pictures = sum(1 for m in messages if isinstance(m, dict) and isinstance(m.get("content"), list)
                       for part in m["content"] if isinstance(part, dict) and part.get("type") == "image_url")
        s = self.settings
        gateway = Gateway(s.api_base, s.api_key(), self.model, reasoning_effort=self.effort or None,
                          max_tokens=int(body.get("max_tokens") or 8000), stream=True, retries=6,
                          ca_file=getattr(s, "ca_file", None) or None)
        t0 = time.monotonic()
        with self._lock:
            self.requests += 1
        try:
            reply = gateway.ask(messages, [], deadline=time.monotonic() + JUDGE_S)
        except GatewayError as exc:
            with self._lock:
                self.failures += 1
            self.log({"kind": "judge", "pictures": pictures, "error": str(exc)[:500],
                      "seconds": round(time.monotonic() - t0, 1)})
            return 502, {"error": {"message": f"judge gateway: {exc}"[:500], "type": "upstream"}}
        finally:
            gateway.close()
        u = reply.usage
        self.log({"kind": "judge", "pictures": pictures, "seconds": round(time.monotonic() - t0, 1), "usage": u,
                  "finish": reply.finish_reason, "answer": reply.text})
        return 200, {"id": "chatcmpl-" + uuid.uuid4().hex, "object": "chat.completion", "created": int(time.time()),
                     "model": self.model,
                     "choices": [{"index": 0, "message": {"role": "assistant", "content": reply.text},
                                  "finish_reason": reply.finish_reason or "stop"}],
                     "usage": {"prompt_tokens": u.get("prompt", 0), "completion_tokens": u.get("completion", 0),
                               "total_tokens": u.get("prompt", 0) + u.get("completion", 0)}}


class _Handler(BaseHTTPRequestHandler):
    def _send(self, status: int, data: dict) -> None:
        raw = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_POST(self):  # noqa: N802 - http.server's name
        if not self.path.rstrip("/").endswith("/chat/completions"):
            return self._send(404, {"error": {"message": "only chat/completions here"}})
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        except ValueError:
            return self._send(400, {"error": {"message": "not JSON"}})
        if not isinstance(body, dict) or body.get("stream"):
            return self._send(400, {"error": {"message": "one non-streamed chat completion per request"}})
        status, data = self.server.judge.answer(body)
        self._send(status, data)

    def do_GET(self):  # noqa: N802
        if self.path.rstrip("/").endswith("/models"):
            return self._send(200, {"object": "list", "data": [{"id": self.server.judge.model, "object": "model"}]})
        self._send(404, {"error": {"message": "not here"}})

    def log_message(self, fmt, *a):  # quiet: requests are logged by Judge.answer
        pass


def main(argv: list[str] | None = None) -> int:
    from .run import ROOT, settings_for

    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.judge",
                                 description="A local stand-in for the benchmark's VLM judge (for a test).")
    ap.add_argument("--port", type=int, default=18997)
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--effort", default=EFFORT, help="reasoning effort of the judge model")
    ap.add_argument("--settings", default=str(ROOT / "config" / "benches.yml"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--log", default=None, help="request log (JSON lines)")
    args = ap.parse_args(argv)

    judge = Judge(settings_for("manip", args.settings, args.set), model=args.model, effort=args.effort,
                  port=args.port, log_path=args.log).start()
    print(f"judge on {judge.url} -> {args.model} ({args.effort})", flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        judge.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
