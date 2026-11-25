"""The RoboColiseum challenge platform's HTTP API (https://robocoliseum.ai), as its official skills describe it:
``python -m hexashell.benches.robocoliseum1.challenge <command>``.

Read-only: ``me`` (the account: name, model name and whether it is approved, the parallelism cap), ``quota``
(today's submissions), ``jobs`` (status and progress of the latest jobs), ``result [--job ID]`` (the scores),
``log [--job ID]`` (a failed job's newest failed case), ``watch [--job ID]`` (a status line every 30 s until
the job is completed or failed).

Changing something: ``login`` (email and password from ``--cred FILE``, a mode-600 file of two lines
``EMAIL=...`` / ``PASSWORD=...``, else asked for; the token goes into the state file) and ``submit --board B
--name N [--description D] [--paper-link URL]``, which uses one of the day's submissions: without ``--yes``
it only shows what it would send (after checking the quota, that the model name is approved and that the
state file's previous job is over); with ``--yes`` it posts the job and writes its handles to the state file
(``~/.simubotix-challenge.env``, as the official skills do, mode 600) and to a copy for the board
(``~/.hexashell/robocoliseum1/jobs/<board>.env``), which :mod:`.tunnel` reads with ``--state``.

Never printed: the password, the tokens, the job's UUID (the tunnel's address is shown by host only).  No
proxy is used.
"""

from __future__ import annotations

import argparse
import getpass
import json
import os
import shlex
import ssl
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

BASE_URL = os.environ.get("ROBOCOLISEUM_BASE_URL", "https://robocoliseum.ai")
STATE = Path.home() / ".simubotix-challenge.env"
JOBS = Path.home() / ".hexashell" / "robocoliseum1" / "jobs"
BOARDS = ("instruction", "spatial", "manip", "robust")
JOB_KEYS = ("JOB_ID", "JOB_UUID", "JOB_TOKEN", "PARALLELISM", "TUNNEL_ENDPOINT", "JOB_BOARD")
TERMINAL = ("completed", "failed")
CA_FILES = ("/etc/ssl/certs/ca-certificates.crt", "/etc/pki/tls/certs/ca-bundle.crt")


def tls_context() -> ssl.SSLContext:
    """Certificates checked against the system's CA bundle (else certifi's): the Python base of our envs was
    built elsewhere and its own CA path does not exist here, so its default context trusts nothing."""
    for path in (os.environ.get("HEXASHELL_CA_FILE"), *CA_FILES):
        if path and Path(path).is_file():
            return ssl.create_default_context(cafile=path)
    try:
        import certifi

        return ssl.create_default_context(cafile=certifi.where())
    except ImportError:
        return ssl.create_default_context()


OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}), urllib.request.HTTPSHandler(context=tls_context()))


# -- the state file ----------------------------------------------------------------------------------
def read_env(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line or line.lstrip().startswith("#"):
            continue
        key, _, value = line.strip().removeprefix("export ").partition("=")
        try:
            parts = shlex.split(value)
        except ValueError:
            parts = [value]
        out[key.strip()] = parts[0] if parts else ""
    return out


def write_env(path: Path, values: dict[str, str], drop: tuple[str, ...] = ()) -> None:
    """Set ``values`` in the KEY=VALUE file (others kept, ``drop`` removed), mode 600, replaced atomically."""
    path.parent.mkdir(parents=True, exist_ok=True)
    old = read_env(path)
    for k in drop:
        old.pop(k, None)
    old.update({k: str(v) for k, v in values.items()})
    tmp = path.with_name(path.name + ".new")
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        for k, v in old.items():
            f.write(f"{k}={shlex.quote(v)}\n")
    os.chmod(tmp, 0o600)
    tmp.replace(path)


# -- HTTP --------------------------------------------------------------------------------------------
class ApiError(RuntimeError):
    def __init__(self, status: int, body: str):
        super().__init__(f"HTTP {status}: {body[:500]}")
        self.status = status
        self.body = body


def call(method: str, path: str, token: str | None = None, body: dict | None = None, timeout: float = 30.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE_URL + path, data=data, method=method)
    req.add_header("Accept", "application/json")
    if data is not None:
        req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    try:
        with OPENER.open(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        raise ApiError(exc.code, exc.read().decode("utf-8", "replace")) from None
    try:
        return json.loads(raw) if raw.strip() else {}
    except ValueError:
        return {"_text": raw[:2000]}


def token_of(state: dict[str, str]) -> str:
    tok = state.get("CHALLENGE_TOKEN", "")
    if not tok:
        raise SystemExit(f"no CHALLENGE_TOKEN in {STATE}: run `login` first")
    return tok


def scrub(obj, keys=("token", "job_token", "uuid", "email", "password", "tunnel_endpoint")):
    """A copy of a response without credentials and ids we do not show."""
    if isinstance(obj, dict):
        return {k: ("<hidden>" if k.lower() in keys else scrub(v, keys)) for k, v in obj.items()}
    if isinstance(obj, list):
        return [scrub(v, keys) for v in obj]
    return obj


# -- commands ----------------------------------------------------------------------------------------
def cmd_login(args, state) -> int:
    email = password = None
    if args.cred:
        path = Path(args.cred).expanduser()
        if path.stat().st_mode & 0o077:
            raise SystemExit(f"{path} is readable by others: chmod 600 it")
        cred = read_env(path)
        email, password = cred.get("EMAIL"), cred.get("PASSWORD")
    if not email:
        email = input("email: ").strip()
    if not password:
        password = getpass.getpass("password: ")
    resp = call("POST", "/api/challenge/login", body={"email": email, "password": password})
    token = resp.get("token") if isinstance(resp, dict) else None
    if resp.get("status") != "ok" or not token:
        print("login failed:", json.dumps(scrub(resp), ensure_ascii=False))
        return 1
    write_env(STATE, {"CHALLENGE_TOKEN": token})
    print(f"logged in; CHALLENGE_TOKEN (length {len(token)}) written to {STATE}")
    return 0


def user_info(token: str) -> dict:
    return call("GET", "/api/challenge/current-user-info", token)


def cmd_me(args, state) -> int:
    info = user_info(token_of(state))
    shown = {k: info.get(k) for k in ("name", "model_name", "model_name_approved", "pending_change",
                                      "max_concurrent_cases") if k in info}
    shown["other_fields"] = sorted(k for k in info if k not in shown)
    print(json.dumps(scrub(shown), ensure_ascii=False, indent=1))
    return 0


def cmd_quota(args, state) -> int:
    print(json.dumps(call("GET", "/api/challenge/submission/quota", token_of(state)), ensure_ascii=False))
    return 0


def jobs(token: str, n: int = 20) -> list[dict]:
    resp = call("GET", f"/api/challenge/jobs?page=1&per-page={int(n)}&sort-by=id:desc", token)
    return resp.get("items") or []


def job_line(j: dict) -> str:
    p = j.get("progress") or {}
    return (f"id {j.get('id')}  {j.get('board')}  {j.get('name')!r}  status {j.get('status')} "
            f"{j.get('detailed_status')}  progress {p.get('done')}/{p.get('total')}  score {j.get('score')}  "
            f"created {j.get('created_at')}")


def cmd_jobs(args, state) -> int:
    for j in jobs(token_of(state), args.n):
        print(job_line(j))
    return 0


def job_id(args, state) -> int:
    jid = args.job or state.get("JOB_ID")
    if not jid:
        raise SystemExit("no --job and no JOB_ID in the state file")
    return int(jid)


def cmd_result(args, state) -> int:
    jid = job_id(args, state)
    resp = call("GET", f"/api/challenge/job/{jid}/result", token_of(state))
    if args.save:
        out = Path(args.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(resp, ensure_ascii=False, indent=1) + "\n")
    tasks = resp.get("tasks") or {}
    for name, t in sorted(tasks.items()):
        steps = ", ".join(f"{s:.1f}" for s in t.get("score") or [])
        print(f"{name:34s} total {t.get('total')}   steps [{steps}]")
    print(f"total {resp.get('total')}  ({len(tasks)} tasks)")
    return 0


def cmd_log(args, state) -> int:
    jid = job_id(args, state)
    resp = call("GET", f"/api/challenge/job/{jid}/log", token_of(state))
    print(json.dumps({k: resp.get(k) for k in ("failure_reason", "status", "case_id", "task_id", "run_instance_id",
                                               "created_at")}, ensure_ascii=False))
    res = resp.get("result") or {}
    print("result keys:", sorted(res))
    if isinstance(res.get("error"), str):
        print("result.error:", res["error"][:2000])
    url = res.get("stdout")
    if isinstance(url, str) and url.startswith("http"):
        if args.save:
            with OPENER.open(url, timeout=60) as r:
                Path(args.save).write_bytes(r.read())
            print(f"stdout saved to {args.save} (untrusted text)")
        else:
            print("stdout: a presigned link (1 h); --save FILE to fetch it")
    elif url is not None:
        print("stdout:", str(url)[:2000])
    return 0


def cmd_watch(args, state) -> int:
    jid = job_id(args, state)
    last = None
    while True:
        try:
            found = [j for j in jobs(token_of(read_env(STATE)), 100) if j.get("id") == jid]
        except Exception as exc:  # noqa: BLE001 - a passing failure
            print(time.strftime("%H:%M:%S"), f"poll failed: {exc}"[:200], flush=True)
            time.sleep(args.every)
            continue
        if not found:
            print(time.strftime("%H:%M:%S"), f"job {jid} not in the latest 100", flush=True)
            return 1
        line = job_line(found[0])
        if line != last or args.every_line:
            print(time.strftime("%H:%M:%S"), line, flush=True)
            last = line
        if found[0].get("detailed_status") in TERMINAL:
            return 0
        time.sleep(args.every)


def cmd_submit(args, state) -> int:
    token = token_of(state)
    quota = call("GET", "/api/challenge/submission/quota", token)
    info = user_info(token)
    print("quota now:", json.dumps(quota, ensure_ascii=False))
    print(f"model name {info.get('model_name')!r}, approved {info.get('model_name_approved')}, "
          f"pending change {info.get('pending_change')}, max concurrent cases {info.get('max_concurrent_cases')}")
    problems = []
    if not info.get("model_name_approved"):
        problems.append("the account's model name is not approved")
    if isinstance(quota, dict) and quota.get("remaining") is not None and int(quota["remaining"]) <= 0:
        problems.append("no submissions left today")
    prev = state.get("JOB_ID")
    if prev:
        found = [j for j in jobs(token, 100) if str(j.get("id")) == str(prev)]
        status = found[0].get("detailed_status") if found else "not found"
        print(f"the state file's previous job: id {prev}, {status}")
        if status not in TERMINAL and status != "not found":
            problems.append(f"the previous job (id {prev}) is still {status}")
    body = {"name": args.name, "config": {"board": args.board}}
    if args.description:
        body["config"]["description"] = args.description
    if args.paper_link:
        body["config"]["paper_link"] = args.paper_link
    print("body:", json.dumps(body, ensure_ascii=False))
    if problems:
        print("NOT submitting:", "; ".join(problems))
        return 1
    if not args.yes:
        print("dry run: nothing sent (add --yes to submit; it uses one of today's submissions)")
        return 0
    resp = call("POST", "/api/challenge/job", token, body)
    needed = ("id", "uuid", "parallelism")
    if not all(resp.get(k) is not None for k in needed):
        print("unexpected answer:", json.dumps(scrub(resp), ensure_ascii=False))
        return 1
    endpoint = resp.get("tunnel_endpoint") or ""
    values = {"JOB_ID": resp["id"], "JOB_UUID": resp["uuid"], "PARALLELISM": resp["parallelism"],
              "TUNNEL_ENDPOINT": endpoint, "JOB_BOARD": args.board}
    if resp.get("job_token"):
        values["JOB_TOKEN"] = resp["job_token"]
    write_env(STATE, values, drop=JOB_KEYS)
    board_file = JOBS / f"{args.board}.env"
    write_env(board_file, {"CHALLENGE_TOKEN": token, **values}, drop=JOB_KEYS)
    os.chmod(JOBS, 0o700)
    host = endpoint.split("://", 1)[-1].split("/", 1)[0] if endpoint else "(none: the fixed default)"
    print(f"submitted: job id {resp['id']}, board {args.board}, parallelism {resp['parallelism']}, "
          f"tunnel host {host}, job token {'yes' if resp.get('job_token') else 'no (login token used)'}")
    print(f"handles written to {STATE} and {board_file}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.challenge",
                                 description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("login")
    p.add_argument("--cred", default=None)
    sub.add_parser("me")
    sub.add_parser("quota")
    p = sub.add_parser("jobs")
    p.add_argument("--n", type=int, default=20)
    for name in ("result", "log"):
        p = sub.add_parser(name)
        p.add_argument("--job", type=int, default=None)
        p.add_argument("--save", default=None)
    p = sub.add_parser("watch")
    p.add_argument("--job", type=int, default=None)
    p.add_argument("--every", type=float, default=30.0)
    p.add_argument("--every-line", action="store_true")
    p = sub.add_parser("submit")
    p.add_argument("--board", required=True, choices=BOARDS)
    p.add_argument("--name", required=True)
    p.add_argument("--description", default=None)
    p.add_argument("--paper-link", default=None)
    p.add_argument("--yes", action="store_true")
    args = ap.parse_args(argv)
    state = read_env(STATE)
    if STATE.exists() and STATE.stat().st_mode & 0o077:
        raise SystemExit(f"{STATE} is readable by others: chmod 600 it")
    fn = {"login": cmd_login, "me": cmd_me, "quota": cmd_quota, "jobs": cmd_jobs, "result": cmd_result,
          "log": cmd_log, "watch": cmd_watch, "submit": cmd_submit}[args.cmd]
    try:
        return fn(args, state)
    except ApiError as exc:
        body = exc.body
        try:
            body = json.dumps(scrub(json.loads(body)), ensure_ascii=False)
        except ValueError:
            pass
        print(f"HTTP {exc.status}: {body[:1000]}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
