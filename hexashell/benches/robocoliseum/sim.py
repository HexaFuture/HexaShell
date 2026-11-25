"""Starting the benchmark: ``geniesim benchmark run`` of genie_sim 3.6 inside one of our containers.

A container (image ``geniesim4-local:3.6.0``, started by ``scripts/setup/geniesim36_container.sh``) has one GPU,
the host network, the source tree at /workspace and the assets at /geniesim_assets; it idles until we run
something in it with ``docker exec``.  A run is one ``geniesim benchmark run <config>`` in a login shell as
``isaac-sim`` (the image's profile sets PATH, the ROS middleware and CycloneDDS), with:

- its own result root ``GENIESIM_BENCHMARK_OUTPUT_ROOT`` = /workspace/output/hexashell/<run id> (on the host
  ``protocol.SOURCE/output/hexashell/<run id>``): the results land in ``<root>/<scene>/<sub task>/evaluate_ret_00.json``
  and the Isaac runner keeps its generated task files there too, so runs on one machine do not share files;
- its own ROS domain (the Newton and libuipc engines refuse a domain that already has one);
- robot initialization allowed ``INIT_TIMEOUT_S`` (``scripts/patches/geniesim36-initialization-timeout.patch``
  lets api_core.py read ``GENIESIM_ROBOT_INITIALIZATION_TIMEOUT_SECONDS``; upstream 300 s, too short while a machine
  builds its Kit cache), and the main-loop watchdog raised to ``WATCHDOG_S`` for every run: the NVIDIA shader cache in the Kit
  cache is one file per running process, so a run that starts while another run of the machine holds the warm
  file gets a file of its own and compiles its shaders again (5 minutes and more in its first step);
- a dummy model key and a closed local endpoint for anything that would ask a language model (no checker of
  Season II does).

One instance alone, by runtime (:func:`commands`): Isaac ``--benchmark.instance_ids=i``; a cloth task (libuipc)
our copy of its config with only instance i (:func:`write_cloth_config`, in the run's result root: the runner
finds the source tree above it), naming our copy of its scene whose CPU barrier waits :data:`SYNC_TIMEOUT_S`
instead of 0.5 s; wok_flip (Newton) the scored layout i from a bundle of the config's seed
(``generate-layouts`` + ``run-layouts --layout-ids i``), our own layouts ``--benchmark.num_instances=1`` with
their seed; the Rubik's cube (MuJoCo) ``--benchmark.num_instances=1`` with the seed that puts the scene first
(:data:`.protocol.RUBIK_SEEDS`; our scenes also name our archive).  The watchdog flag is Isaac's alone: the other
runners refuse flags they do not know.

The run is judged by its result file, never by its exit code (a run cut short can exit 0).  Stopping: the docker
client does not stop what it started inside the container, so the run writes the id of its process group to
``<root>/pid`` and :func:`stop` signals that group inside the container.  Some of what a run starts leaves that
group (the Newton runner starts ROS, the renderer and ffmpeg in sessions of their own; the MuJoCo renderer's
worker may outlive its runner and hold the Kit cache): :func:`end_leftovers` ends, at the end of every run,
whatever in the container still has the run's result root in its environment.
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import time
from pathlib import Path

from . import protocol as P

CONTAINER_PREFIX = "hexashell-geniesim-"
RUN_CONTAINER = P.RUN_CONTAINER
#: seconds robot initialization may take (the first Isaac task on a machine builds the Kit cache in it)
INIT_TIMEOUT_S = 1800
#: Isaac's main-loop watchdog for every run (upstream 300 s; a run's first step may compile its shaders)
WATCHDOG_S = 1800
KIT_CACHE = P.SOURCE / ".kit" / "cache"
#: how long run-layouts waits for its engine (upstream 600 s; a first start compiles the Warp kernels, about
#: 9.5 min without our shared cache)
LAYOUT_STARTUP_S = 1500
#: the Warp kernel cache of Newton runs, shared by our containers of a machine (the image keeps it in each
#: container's /tmp)
WARP_CACHE = f"{P.CONTAINER_SOURCE}/.warp/cache"
#: where a run's results go, inside the container and on the host
OUTPUT = "output/hexashell"
#: the cloth tasks' bringup scenes (the libuipc engine's settings), relative to the source tree
BRINGUP = "source/geniesim_ros/src/ros_ws/src/genie_sim_bringup/config"
#: seconds the libuipc engine's CPU barrier may wait for one exchange (upstream 0.5 s in the three cloth scenes; a
#: wall-clock watchdog only: every exchange's step count and simulated time are checked exactly).  At 0.5 s
#: runs ended invalid ("CPU barrier sequence N exceeded 0.5s") even with one libuipc run on the machine; our scene
#: copy waits longer.
SYNC_TIMEOUT_S = 10.0
#: environment variables set for every run (dummy model endpoint: nothing listens there)
FIXED_ENV = {
    "SIM_REPO_ROOT": P.CONTAINER_SOURCE, "SIM_ASSETS": P.CONTAINER_ASSETS, "GENIESIM_ASSETS_PATH": P.CONTAINER_ASSETS,
    "API_KEY": "sk-local-dummy", "BASE_URL": "http://127.0.0.1:18998/v1", "VL_MODEL": "local-vlm",
    "MODEL": "local-llm", "NO_PROXY": "127.0.0.1,localhost", "no_proxy": "127.0.0.1,localhost",
    "WARP_CACHE_PATH": WARP_CACHE,
}
#: upstream checker traces passed on into a run when set in our environment (developer logs in bench.log only; the
#: model never sees them): GENIESIM_POUR_TRACE=1 makes PourVolume log its particle counts at every check
TRACE_ENV = ("GENIESIM_POUR_TRACE",)


def container_for(gpu: int | str) -> str:
    return f"{CONTAINER_PREFIX}gpu{int(gpu)}"


def _docker(*args: str, timeout: float = 60) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], capture_output=True, text=True, timeout=timeout)


def running(name: str) -> bool:
    r = _docker("inspect", "-f", "{{.State.Running}}", name)
    return r.returncode == 0 and r.stdout.strip() == "true"


def ensure(name: str, gpu: int | str, log_path: Path | None = None) -> None:
    """Start the container unless it is running (blocks until its entrypoint is ready, up to 10 min)."""
    if not name.startswith(CONTAINER_PREFIX):
        raise ValueError(f"our containers are named {CONTAINER_PREFIX}*, not {name!r}")
    if running(name):
        return
    if _docker("inspect", name).returncode == 0:
        raise RuntimeError(f"container {name} exists but is not running; look at it before removing it")
    out = open(log_path, "ab") if log_path else subprocess.DEVNULL
    try:
        r = subprocess.run([str(RUN_CONTAINER), name, str(int(gpu))], stdout=out, stderr=subprocess.STDOUT,
                           timeout=900)
    finally:
        if log_path:
            out.close()
    if r.returncode != 0:
        raise RuntimeError(f"{RUN_CONTAINER.name} {name} {gpu} failed ({r.returncode})")


def cold() -> bool:
    """Whether this machine's Kit cache is still empty (the next Isaac start builds it)."""
    return not KIT_CACHE.is_dir() or not any(KIT_CACHE.iterdir())


def result_root(run_id: str) -> tuple[str, Path]:
    """A run's result root inside the container and on the host."""
    if not run_id or "/" in run_id or run_id in (".", ".."):
        raise ValueError(f"bad run id {run_id!r}")
    return f"{P.CONTAINER_SOURCE}/{OUTPUT}/{run_id}", P.SOURCE / OUTPUT / run_id


def instance_args(case: P.Case) -> list[str]:
    """The arguments of ``geniesim benchmark run`` that pick the case (Isaac, our wok_flip layouts, the Rubik's
    cube; cloth tasks pick theirs by our copy of the config, scored wok_flip layouts by ``run-layouts``)."""
    t = P.task(case.config)
    if t.runtime == "isaac":
        if not case.official:
            raise ValueError(f"Isaac tasks have official instances only: {case.key}")
        return [f"--benchmark.instance_ids={case.instance}"]
    if t.runtime == "libuipc":
        if not case.official:
            raise ValueError(f"cloth tasks have official instances only: {case.key}")
        return ["--benchmark.num_instances=1"]
    if t.runtime == "newton":
        if case.official:
            raise ValueError(f"a scored wok_flip layout runs from the layout bundle (run-layouts): {case.key}")
        return ["--benchmark.num_instances=1", f"--benchmark.seed={case.seed}"]
    if t.runtime == "mujoco":
        if case.official:
            return ["--benchmark.num_instances=1", f"--benchmark.seed={P.RUBIK_SEEDS[case.instance]}"]
        if case.archive != P.RUBIK_OWN_ARCHIVE or (case.instance, case.seed) not in [
                pair for pairs in P.RUBIK_OWN_SCENES.values() for pair in pairs]:
            raise ValueError(f"not one of our Rubik's cube scenes: {case.key}")
        return [f"--rubik.bundled_scene_archive={case.archive}", "--benchmark.num_instances=1",
                f"--benchmark.seed={case.seed}"]
    raise ValueError(f"unknown runtime {t.runtime!r}")


def cloth_config(case: P.Case, run_id: str) -> tuple[str, Path]:
    """Our copy of a cloth task's config for one instance: its path inside the container and on the host."""
    inside, host = result_root(run_id)
    name = f"{case.config}_i{case.instance:02d}.yaml"
    return f"{inside}/{name}", host / name


def write_cloth_scene(name: str, path: Path, sync_timeout_s: float = SYNC_TIMEOUT_S) -> str:
    """Write our copy of a cloth task's bringup scene with the CPU barrier's timeout raised (:data:`SYNC_TIMEOUT_S`):
    the original's text with that one value changed, everything else as it is.  Returns the text."""
    import re

    text = (P.SOURCE / BRINGUP / name).read_text()
    pattern = re.compile(r"^(\s+sync_timeout_s:\s*)(\S+)[ \t]*$", re.M)
    found = pattern.findall(text)
    if len(found) != 1:
        raise ValueError(f"{name}: {len(found)} sync_timeout_s lines, expected 1")
    if float(found[0][1]) > sync_timeout_s:
        raise ValueError(f"{name}: sync_timeout_s {found[0][1]} is already above {sync_timeout_s:g}")
    text = pattern.sub(lambda m: f"{m.group(1)}{sync_timeout_s:g}", text)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return text


def write_cloth_config(case: P.Case, path: Path, source: Path | None = None, *, inside: str | None = None) -> dict:
    """Write our copy of a cloth task's config that plays instance i alone: only its base offset, its cloth
    material first (episode e gets material e mod M: here episode 0), one instance, the cameras file by its path
    inside the container (the original names it relative to the config's folder), and the evaluation scene by
    the path of our copy of it next to the config (:func:`write_cloth_scene`; ``inside``: that folder's path
    inside the container, by default the host's).  Reads the benchmark's config as data."""
    import os

    import yaml

    t = P.task(case.config)
    if t.runtime != "libuipc" or not case.official or not 0 <= case.instance < t.instances:
        raise ValueError(f"not a cloth instance: {case.key}")
    source = source or P.PKG / "config" / f"{t.config}.yaml"
    config = yaml.safe_load(source.read_text())
    lib = config["libuipc"]
    offsets = lib["instance_base_xy_offsets"]
    if len(offsets) != t.instances or config["benchmark"]["num_instances"] != t.instances:
        raise ValueError(f"{source.name}: {len(offsets)} offsets for {config['benchmark']['num_instances']} "
                         f"instances, expected {t.instances}")
    lib["instance_base_xy_offsets"] = [offsets[case.instance]]
    config["benchmark"]["num_instances"] = 1
    cameras = Path(os.path.normpath(source.parent / lib["cameras"]))
    lib["cameras"] = f"{P.CONTAINER_SOURCE}/{cameras.relative_to(P.SOURCE).as_posix()}"
    material = config["generalization"]["material"]
    allowed = list(material["allowed_materials"])
    k = case.instance % len(allowed)
    material["allowed_materials"] = allowed[k:] + allowed[:k]
    scene = f"{case.config}_i{case.instance:02d}_scene.yaml"
    write_cloth_scene(lib["evaluation_scene"], path.parent / scene)
    lib["evaluation_scene"] = f"{inside or path.parent.as_posix()}/{scene}"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"# {t.config} instance {case.instance} alone (HexaShell, from {source.name})\n"
                    + yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
    return config


def commands(case: P.Case, port: int, run_id: str, *, watchdog_s: float | None = None,
             extra: list[str] = ()) -> list[list[str]]:
    """The commands of one run, in order (each must succeed before the next)."""
    t = P.task(case.config)
    inside, _ = result_root(run_id)
    host = f"--infer-host=127.0.0.1:{int(port)}"
    if t.runtime == "newton" and case.official:
        if not 0 <= case.instance < P.WOK_LAYOUTS:
            raise ValueError(f"wok_flip has scored layouts 0..{P.WOK_LAYOUTS - 1}: {case.key}")
        bundle = f"{inside}/wok_layouts"
        return [["geniesim", "benchmark", "generate-layouts", f"--output-dir={bundle}", f"--count={P.WOK_LAYOUTS}",
                 f"--seed={t.seed}"],
                ["geniesim", "benchmark", "run-layouts", f"--bundle={bundle}", f"--layout-ids={case.instance}", host,
                 f"--output-dir={inside}/{t.scene}/{t.sub_task}", "--benchmark.num_episode=1",
                 f"--startup-timeout-s={int(LAYOUT_STARTUP_S)}", *extra]]
    config = cloth_config(case, run_id)[0] if t.runtime == "libuipc" else case.config
    args = ["geniesim", "benchmark", "run", config, host, "--benchmark.num_episode=1", *instance_args(case)]
    if watchdog_s and t.runtime == "isaac":
        args.append(f"--app.main_loop_stall_timeout_s={int(watchdog_s)}")
    return [args + list(extra)]


def command(case: P.Case, port: int, *, watchdog_s: float | None = None, extra: list[str] = (),
            run_id: str = "run") -> list[str]:
    """The command of a run that is one command (all but a scored wok_flip layout)."""
    found = commands(case, port, run_id, watchdog_s=watchdog_s, extra=extra)
    if len(found) != 1:
        raise ValueError(f"{case.key} runs {len(found)} commands")
    return found[0]


def script(case: P.Case, port: int, run_id: str, domain: int, *, watchdog_s: float | None = None,
           extra: list[str] = ()) -> str:
    """The login-shell script of one run (see the module's notes)."""
    inside, _ = result_root(run_id)
    env = {**FIXED_ENV, **{k: os.environ[k] for k in TRACE_ENV if os.environ.get(k)}, "ROS_DOMAIN_ID": str(int(domain)),
           "GENIESIM_ROBOT_INITIALIZATION_TIMEOUT_SECONDS": str(INIT_TIMEOUT_S),
           "GENIESIM_BENCHMARK_OUTPUT_ROOT": inside}
    exports = " ".join(f"{k}={shlex.quote(v)}" for k, v in env.items())
    steps = [" ".join(shlex.quote(a) for a in c) for c in commands(case, port, run_id, watchdog_s=watchdog_s,
                                                                   extra=extra)]
    run = " && ".join(steps[:-1] + [f"exec {steps[-1]}"])
    pid = shlex.quote(f"{inside}/pid")
    return (f"cd {P.CONTAINER_SOURCE} && mkdir -p {shlex.quote(inside)} {shlex.quote(WARP_CACHE)} && export {exports} && "
            f"source /opt/ros/jazzy/setup.bash && {{ [ ! -f devel/setup.bash ] || source devel/setup.bash; }} && "
            f"exec setsid --wait sh -c {shlex.quote(f'echo $$ > {pid}; {run}')}")


def launch(name: str, case: P.Case, port: int, run_id: str, domain: int, log_path: Path, *,
           watchdog_s: float | None = None, extra: list[str] = ()) -> subprocess.Popen:
    """Start one run in the container; its output goes to ``log_path``.  Writes ``<host root>/launch.json``."""
    _, host = result_root(run_id)
    host.mkdir(parents=True, exist_ok=True)
    if P.task(case.config).runtime == "libuipc":
        config_inside, config_host = cloth_config(case, run_id)
        write_cloth_config(case, config_host, inside=config_inside.rsplit("/", 1)[0])
    text = script(case, port, run_id, domain, watchdog_s=watchdog_s, extra=extra)
    (host / "launch.json").write_text(json.dumps({"container": name, "case": case.key, "port": port,
                                                   "domain": domain, "script": text}, indent=1) + "\n")
    log = open(log_path, "ab")
    try:
        return subprocess.Popen(["docker", "exec", "-u", "isaac-sim", "-w", P.CONTAINER_SOURCE, name, "bash", "-lc",
                                 text], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
    finally:
        log.close()


def stop(name: str, run_id: str, signal: str = "TERM") -> bool:
    """Signal the run's process group inside the container; False when there is no pid to signal."""
    if signal not in ("TERM", "INT", "KILL"):
        raise ValueError(f"signal {signal!r}")
    _, host = result_root(run_id)
    try:
        pgid = int((host / "pid").read_text().strip())
    except (OSError, ValueError):
        return False
    if pgid <= 1:
        return False
    # the container's sh is dash: its kill takes a negative pid for a group, but not "--"
    r = _docker("exec", "-u", "isaac-sim", name, "sh", "-c", f"kill -{signal} -{pgid}")
    return r.returncode == 0


def leftovers(name: str, run_id: str) -> list[int] | None:
    """Pids in the container whose environment holds this run's result root: its processes and whatever they
    started, also outside its process group (None when the container did not answer)."""
    inside, _ = result_root(run_id)
    probe = 'for p in /proc/[0-9]*; do grep -qszxF -e "$1" "$p/environ" && echo "${p#/proc/}"; done; true'
    try:
        r = _docker("exec", "-u", "isaac-sim", name, "sh", "-c", probe, "sh",
                    f"GENIESIM_BENCHMARK_OUTPUT_ROOT={inside}")
    except subprocess.TimeoutExpired:
        return None
    if r.returncode != 0:
        return None
    return sorted({int(w) for w in r.stdout.split() if w.isdigit() and int(w) > 1})


def _signal_pids(name: str, signal: str, pids: list[int]) -> None:
    _docker("exec", "-u", "isaac-sim", name, "sh", "-c", f'kill -{signal} "$@" 2>/dev/null; true', "sh",
            *(str(p) for p in pids))


def end_leftovers(name: str, run_id: str, wait_s: float = 20.0, poll_s: float = 2.0) -> dict:
    """TERM what :func:`leftovers` finds, then KILL what is still there after ``wait_s``.  Returns the pids per
    signal and those still alive after the KILL (empty lists when nothing was left)."""
    out = {"term": [], "kill": [], "alive": []}
    pids = leftovers(name, run_id)
    if not pids:
        return out
    out["term"] = pids
    _signal_pids(name, "TERM", pids)
    deadline = time.monotonic() + wait_s
    while pids and time.monotonic() < deadline:
        time.sleep(poll_s)
        pids = leftovers(name, run_id)
    if pids:
        out["kill"] = pids
        _signal_pids(name, "KILL", pids)
        time.sleep(poll_s)
        out["alive"] = leftovers(name, run_id) or []
    return out


def results(run_id: str, case: P.Case) -> list[Path]:
    """The run's result files (``evaluate_ret_00.json`` first), on the host: the top level of the task's folder
    only (Newton keeps copies of its attempts below it)."""
    t = P.task(case.config)
    _, host = result_root(run_id)
    return sorted((host / t.scene / t.sub_task).glob("evaluate_ret_*.json"))


def scored(path: Path) -> bool:
    """Whether a result file has an episode's result (Newton writes its file empty when it starts)."""
    try:
        return bool(json.loads(path.read_text()).get("details"))
    except (OSError, ValueError, AttributeError):
        return False
