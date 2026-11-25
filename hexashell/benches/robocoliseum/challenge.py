"""The RoboColiseum platform's HTTP API for Season II (https://robocoliseum.ai), as its official Season II skills
describe it: ``python -m hexashell.benches.robocoliseum.challenge <command>``.

Read-only: ``me`` (the account: name, model name and whether it is approved, the model type, the parallelism
cap), ``quota`` (today's submissions), ``jobs`` (status, progress and the four boards' scores of the latest Season
II jobs), ``result [--job ID]`` (the scores), ``log [--job ID]`` (a failed job's newest failed case), ``watch [--job
ID]`` (a status line every 30 s until the job is completed or failed), ``ranking`` (the public leaderboard of
Season II, no login).

Changing something: ``login`` (email and password from ``--cred FILE``, a mode-600 file of two lines
``EMAIL=...`` / ``PASSWORD=...``, else asked for; the token goes into the state file) and ``submit --name N
[--description D]``, which uses one of the day's submissions: without ``--yes`` it only shows what it would send
(after checking the quota, that the model name is approved and its type set, and that the previous Season II job
is over, unless ``--leave-previous-open``: contestants cannot stop a job, so an abandoned one stays open and its
job file is kept as ``v2_job<id>.env``); with ``--yes`` it posts the job on the board ``v2`` (one job plays the cases of all four boards) and
writes its handles to the state file (``~/.simubotix-challenge.env``, as the official skills do, mode 600) and to
the Season II job file ``~/.hexashell/robocoliseum/jobs/v2.env``, which :mod:`.tunnel` reads by default.

Never printed: the password, the tokens, the job's UUID, the tunnel's address.  No proxy is used.
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
import urllib.parse
import urllib.request
from pathlib import Path

BASE_URL = os.environ.get("ROBOCOLISEUM_BASE_URL", "https://robocoliseum.ai")
STATE = Path.home() / ".simubotix-challenge.env"
JOBS = Path.home() / ".hexashell" / "robocoliseum" / "jobs"
#: Season II's job board (config.board of a submission) and season number (the jobs list's filter)
JOB_BOARD = "v2"
SEASON = "2"
#: the sub-boards a Season II job is scored on
BOARDS = ("finemanip", "crsemb", "wbc", "dexmanip")
MODEL_TYPES = ("policy", "agent")
JOB_KEYS = ("JOB_ID", "JOB_UUID", "JOB_TOKEN", "PARALLELISM", "TUNNEL_ENDPOINT", "JOB_BOARD", "JOB_SEASON")
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


def job_file() -> Path:
    return JOBS / f"{JOB_BOARD}.env"


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


#: response fields never shown (credentials, ids, addresses, presigned links)
HIDDEN = ("token", "job_token", "challenge_token", "uuid", "user_uid", "email", "password", "tunnel_endpoint", "url",
          "video_url", "stdout")


def scrub(obj, keys=HIDDEN):
    """A copy of a response without credentials, ids and links we do not show."""
    if isinstance(obj, dict):
        return {k: ("<hidden>" if str(k).lower() in keys else scrub(v, keys)) for k, v in obj.items()}
    if isinstance(obj, list):
        return [scrub(v, keys) for v in obj]
    return obj


def endpoint_host(endpoint: str) -> str:
    """What may be shown of a tunnel endpoint: its scheme, never the address."""
    if not endpoint:
        return "none"
    scheme = endpoint.split("://", 1)[0] if "://" in endpoint else "?"
    return f"set ({scheme})"


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
    if not isinstance(resp, dict) or resp.get("status") != "ok" or not token:
        print("login failed:", json.dumps(scrub(resp), ensure_ascii=False))
        return 1
    write_env(STATE, {"CHALLENGE_TOKEN": token})
    print(f"logged in; CHALLENGE_TOKEN (length {len(token)}) written to {STATE}")
    return 0


def user_info(token: str) -> dict:
    return call("GET", "/api/challenge/current-user-info", token)


ME_FIELDS = ("name", "model_name", "model_name_approved", "model_type", "pending_change", "max_concurrent_cases")


def cmd_me(args, state) -> int:
    info = user_info(token_of(state))
    shown = {k: info.get(k) for k in ME_FIELDS if k in info}
    shown["other_fields"] = sorted(k for k in info if k not in shown)
    print(json.dumps(scrub(shown), ensure_ascii=False, indent=1))
    return 0


def cmd_quota(args, state) -> int:
    print(json.dumps(scrub(call("GET", "/api/challenge/submission/quota", token_of(state))), ensure_ascii=False))
    return 0


def jobs(token: str, n: int = 20) -> list[dict]:
    resp = call("GET", f"/api/challenge/jobs?season={SEASON}&page=1&per-page={int(n)}&sort-by=id:desc", token)
    return (resp.get("items") if isinstance(resp, dict) else None) or []


def job_line(j: dict) -> str:
    p = j.get("progress") or {}
    boards = " ".join(f"{b.get('board')} {b.get('score')}" for b in j.get("boards") or [] if isinstance(b, dict))
    review = j.get("review_status")
    return (f"id {j.get('id')}  {j.get('board')}  {j.get('name')!r}  status {j.get('status')} "
            f"{j.get('detailed_status')}  progress {p.get('done')}/{p.get('total')}  score {j.get('score')}"
            f"{f'  [{boards}]' if boards else ''}{f'  review {review}' if review else ''}  created {j.get('created_at')}")


def cmd_jobs(args, state) -> int:
    for j in jobs(token_of(state), args.n):
        print(job_line(j))
    return 0


def last_job(state: dict[str, str]) -> str | None:
    """The JOB_ID of our last Season II job: the job file's, else the state file's when it is of Season II."""
    return read_env(job_file()).get("JOB_ID") or (state.get("JOB_ID") if state.get("JOB_SEASON") == SEASON else None)


def job_id(args, state) -> int:
    jid = args.job or last_job(state)
    if not jid:
        raise SystemExit(f"no --job and no JOB_ID in {job_file()}")
    return int(jid)


def cmd_result(args, state) -> int:
    jid = job_id(args, state)
    resp = call("GET", f"/api/challenge/job/{jid}/result", token_of(state))
    if args.save:
        out = Path(args.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(scrub(resp), ensure_ascii=False, indent=1) + "\n")
    tasks = resp.get("tasks") if isinstance(resp, dict) else None
    if isinstance(tasks, dict):
        for name, t in sorted(tasks.items()):
            if isinstance(t, dict):
                steps = t.get("score")
                steps = (", ".join(f"{s:.1f}" if isinstance(s, (int, float)) else str(s) for s in steps)
                         if isinstance(steps, list) else steps)
                print(f"{name:48s} total {t.get('total')}   score [{steps}]")
            else:
                print(f"{name:48s} {json.dumps(scrub(t), ensure_ascii=False)[:200]}")
        print(f"total {resp.get('total')}  ({len(tasks)} tasks)")
    else:
        print(json.dumps(scrub(resp), ensure_ascii=False)[:4000])
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
            print(time.strftime("%H:%M:%S"), f"poll failed: {type(exc).__name__}", flush=True)
            time.sleep(args.every)
            continue
        if not found:
            print(time.strftime("%H:%M:%S"), f"job {jid} not in the latest 100 of season {SEASON}", flush=True)
            return 1
        line = job_line(found[0])
        if line != last or args.every_line:
            print(time.strftime("%H:%M:%S"), line, flush=True)
            last = line
        if found[0].get("detailed_status") in TERMINAL:
            return 0
        time.sleep(args.every)


def cmd_ranking(args, state) -> int:
    """The public leaderboard of Season II (no token)."""
    query = urllib.parse.urlencode({"model_type": args.model_type, "page": args.page, "per-page": args.n})
    resp = call("GET", f"/api/challenge/leaderboard/{JOB_BOARD}?{query}")
    items = (resp.get("items") if isinstance(resp, dict) else None) or []
    for i, row in enumerate(items, 1 + (args.page - 1) * args.n):
        row = scrub(row) if isinstance(row, dict) else {"row": row}
        shown = {k: row.get(k) for k in ("model_name", "name", "model_type", "score", "boards") if k in row}
        print(i, json.dumps(shown or {"fields": sorted(row)}, ensure_ascii=False)[:400])
    if not items:
        print(json.dumps(scrub(resp), ensure_ascii=False)[:2000])
    return 0


def submit_problems(quota, info, previous: str | None, leave_previous_open: bool = False) -> list[str]:
    """Why a submission should not go now (its checks; ``previous``: the last Season II job's status, if any;
    ``leave_previous_open``: a job that is not over is no reason, since contestants cannot stop a job, neither the
    API nor the site has a stop call)."""
    problems = []
    if not info.get("model_name_approved"):
        problems.append("the account's model name is not approved")
    if info.get("model_type") not in MODEL_TYPES:
        problems.append(f"the account's model type is {info.get('model_type')!r}, not one of {', '.join(MODEL_TYPES)}")
    if isinstance(quota, dict) and quota.get("remaining") is not None and int(quota["remaining"]) <= 0:
        problems.append("no submissions left today")
    if previous is not None and previous not in TERMINAL and previous != "not found" and not leave_previous_open:
        problems.append(f"the previous Season II job is still {previous} (--leave-previous-open submits anyway)")
    return problems


def keep_previous_job_file(prev: str | None) -> Path | None:
    """A copy of the job file of the job ``prev`` (``v2_job<id>.env``, mode 600) before a new submission overwrites
    it, so that job can still be resumed; None when there is no such file."""
    src = job_file()
    if not prev or not src.exists() or read_env(src).get("JOB_ID") != str(prev):
        return None
    dst = JOBS / f"{JOB_BOARD}_job{prev}.env"
    tmp = dst.with_name(dst.name + ".new")
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(src.read_text(encoding="utf-8"))
    os.chmod(tmp, 0o600)
    tmp.replace(dst)
    return dst


def submit_body(name: str, description: str | None = None, paper_link: str | None = None) -> dict:
    """The job's body; ``paper_link`` (the site's "Open Source URL") is optional, the official skill says v2 does not
    use it, and it is sent when given."""
    body = {"name": name, "config": {"board": JOB_BOARD}}
    if description:
        body["config"]["description"] = description
    if paper_link:
        body["config"]["paper_link"] = paper_link
    return body


def cmd_submit(args, state) -> int:
    token = token_of(state)
    quota = call("GET", "/api/challenge/submission/quota", token)
    info = user_info(token)
    print("quota now:", json.dumps(scrub(quota), ensure_ascii=False))
    print(f"model name {info.get('model_name')!r}, approved {info.get('model_name_approved')}, type "
          f"{info.get('model_type')!r}, pending change {info.get('pending_change')}, max concurrent cases "
          f"{info.get('max_concurrent_cases')}")
    prev = last_job(state)
    status = None
    latest = jobs(token, 100)
    if prev:
        found = [j for j in latest if str(j.get("id")) == str(prev)]
        status = found[0].get("detailed_status") if found else "not found"
        print(f"the previous Season II job: id {prev}, {status}")
    running = [j for j in latest if j.get("detailed_status") not in TERMINAL and str(j.get("id")) != str(prev)]
    for j in running[:5]:
        print("note: another Season II job not over:", job_line(j))
    body = submit_body(args.name, args.description, args.paper_link)
    print("body:", json.dumps(body, ensure_ascii=False))
    problems = submit_problems(quota, info, status, args.leave_previous_open)
    if problems:
        print("NOT submitting:", "; ".join(problems))
        return 1
    left_open = args.leave_previous_open and status is not None and status not in TERMINAL and status != "not found"
    if left_open:
        print(f"the previous job {prev} stays {status} (contestants cannot stop a job); its job file is kept as "
              f"{JOB_BOARD}_job{prev}.env")
    if not args.yes:
        print("dry run: nothing sent (add --yes to submit; it uses one of today's submissions)")
        return 0
    if left_open:
        keep_previous_job_file(prev)
    resp = call("POST", "/api/challenge/job", token, body)
    needed = ("id", "uuid", "parallelism")
    if not isinstance(resp, dict) or not all(resp.get(k) is not None for k in needed):
        print("unexpected answer:", json.dumps(scrub(resp), ensure_ascii=False))
        return 1
    endpoint = resp.get("tunnel_endpoint") or ""
    if not endpoint:
        got = call("GET", "/api/challenge/tunnel/endpoint", token)
        endpoint = (got.get("url") if isinstance(got, dict) else None) or ""
    values = {"JOB_ID": resp["id"], "JOB_UUID": resp["uuid"], "PARALLELISM": resp["parallelism"],
              "TUNNEL_ENDPOINT": endpoint, "JOB_BOARD": str(resp.get("board") or JOB_BOARD), "JOB_SEASON": SEASON}
    if resp.get("job_token"):
        values["JOB_TOKEN"] = resp["job_token"]
    write_env(STATE, values, drop=JOB_KEYS)
    write_env(job_file(), {"CHALLENGE_TOKEN": token, **values}, drop=JOB_KEYS)
    os.chmod(JOBS, 0o700)
    print(f"submitted: job id {resp['id']}, board {values['JOB_BOARD']}, parallelism {resp['parallelism']}, "
          f"tunnel endpoint {endpoint_host(endpoint)}, job token {'yes' if resp.get('job_token') else 'no (login token used)'}")
    print(f"handles written to {STATE} and {job_file()}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum.challenge",
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
    p = sub.add_parser("ranking")
    p.add_argument("--model-type", choices=(*MODEL_TYPES, "all"), default="all")
    p.add_argument("--page", type=int, default=1)
    p.add_argument("--n", type=int, default=20)
    p = sub.add_parser("submit")
    p.add_argument("--name", required=True)
    p.add_argument("--description", default=None)
    p.add_argument("--paper-link", default=None, help="a URL of at most 512 characters")
    p.add_argument("--leave-previous-open", action="store_true",
                   help="submit although the previous job is not over (it stays open; its job file is kept)")
    p.add_argument("--yes", action="store_true")
    args = ap.parse_args(argv)
    state = read_env(STATE)
    if STATE.exists() and STATE.stat().st_mode & 0o077:
        raise SystemExit(f"{STATE} is readable by others: chmod 600 it")
    fn = {"login": cmd_login, "me": cmd_me, "quota": cmd_quota, "jobs": cmd_jobs, "result": cmd_result,
          "log": cmd_log, "watch": cmd_watch, "ranking": cmd_ranking, "submit": cmd_submit}[args.cmd]
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
