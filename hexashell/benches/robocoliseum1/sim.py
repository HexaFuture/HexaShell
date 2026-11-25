"""Starting the benchmark: genie_sim v3.5.0's ``app.py`` natively (no docker), in its own Python.

The environment mirrors ``docker/entrypoint.sh`` of genie_sim v3.5.0: the ROS 2 jazzy bridge libraries of
the Isaac Sim 5.1 pip install, cyclonedds with localhost-only discovery, per-run Kit token folders.

Isolation: the benchmark writes its task folder (``benchmark/saved_task/<task>``, deleted at the end) and its
results (``$SIM_REPO_ROOT/output/benchmark/...``) under ``SIM_REPO_ROOT``, so two runs of one task would
clash.  Each run gets its own ``SIM_REPO_ROOT = <run dir>/root``: links to the read-only source, except
``benchmark/saved_task`` and ``output/``, which are its own, and ``benchmark/config``, which is the native
copy whose scene files have the docker asset prefix replaced.  The code itself is imported from the real
tree.

Judge keys: the VLM / LLM evaluators (scoop_popcorn's) read ``API_KEY`` / ``BASE_URL`` / ``VL_MODEL`` /
``MODEL`` from the environment before falling back to the benchmark's own configuration file.  So these are
always set: a dummy key and a local endpoint - the run's own :class:`.judge.Judge` for the VLM-judged configs
(:mod:`.run` starts it), else ``GENIESIM_JUDGE_URL``, where nothing need listen.  No credential reaches the process.

Tools: the configs that record the cameras (``record: true``, the pick_specific_object ones) pipe them into
``ffmpeg``, which the docker image has on its PATH; here each run's ``bin/`` holds a link to the static ffmpeg
that ships with the Isaac Sim pip install (imageio-ffmpeg) and comes first on the run's PATH.

Paths default to places inside the HexaShell checkout (``protocol.GENIESIM`` = ``.bench-data/geniesim-3.5``, or
``HEXASHELL_GENIESIM35``) and can be set one by one in the environment: ``GENIESIM_ROOT`` (the genie_sim checkout),
``GENIESIM_PYTHON`` (a Python 3.11 with Isaac Sim 5.1 and the benchmark's requirements), ``ISAACSIM_SITE`` (its Isaac
Sim 5.1 site-packages), ``GENIESIM_ASSETS``, ``GENIESIM_NATIVE_CONFIG``.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from . import protocol as P

REL = "source/geniesim_benchmark/src/geniesim_benchmark"
PYTHON = os.environ.get("GENIESIM_PYTHON") or str(P.ROOT / ".venvs" / "bench-geniesim35" / "bin" / "python")
ISAAC_SITE = Path(os.environ.get("ISAACSIM_SITE")
                  or Path(PYTHON).parents[1] / "lib" / "python3.11" / "site-packages")
ASSETS = os.environ.get("GENIESIM_ASSETS") or str(P.GENIESIM / "GenieSimAssets")
#: where a judge forwarder may listen; by default a closed local port (the judge then fails and scores 0)
JUDGE_URL = os.environ.get("GENIESIM_JUDGE_URL", "http://127.0.0.1:18998/v1")
#: environment variables never passed on (display, proxies, Python settings, anything like a credential)
DROP = ("DISPLAY", "XAUTHORITY", "HTTPS_PROXY", "HTTP_PROXY", "https_proxy", "http_proxy", "ALL_PROXY", "all_proxy",
        "PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "CONDA_PREFIX", "CONDA_DEFAULT_ENV")


def package() -> Path:
    return P.PKG


def overlay(run_dir: Path) -> Path:
    """The run's SIM_REPO_ROOT (see the module's notes); returns it."""
    pkg = package()
    native = P.NATIVE_CONFIG
    if not (native / "NATIVE_COPY.txt").exists():
        raise FileNotFoundError(f"no native benchmark config at {native}")
    root = Path(run_dir) / "root"
    op = root / REL
    (op / "benchmark" / "saved_task").mkdir(parents=True, exist_ok=True)
    (root / "output").mkdir(parents=True, exist_ok=True)
    for entry in pkg.iterdir():
        if entry.name == "benchmark":
            continue
        link = op / entry.name
        if not link.is_symlink() and not link.exists():
            link.symlink_to(entry)
    for entry in (pkg / "benchmark").iterdir():
        if entry.name == "saved_task":
            continue
        link = op / "benchmark" / entry.name
        target = native if entry.name == "config" else entry
        if not link.is_symlink() and not link.exists():
            link.symlink_to(target)
    return root


def ffmpeg() -> Path | None:
    """The static ffmpeg of the Isaac Sim pip install, if there is one."""
    found = sorted((ISAAC_SITE / "imageio_ffmpeg" / "binaries").glob("ffmpeg-linux-*"))
    return found[-1] if found else None


def environment(run_dir: Path, gpu: str, domain: int, judge_url: str | None = None) -> dict[str, str]:
    run_dir = Path(run_dir)
    bridge = ISAAC_SITE / "isaacsim" / "exts" / "isaacsim.ros2.bridge" / "jazzy"
    env = {k: v for k, v in os.environ.items()
           if k not in DROP and not k.startswith(("PYTHON", "CONDA"))
           and not any(w in k.upper() for w in ("KEY", "TOKEN", "SECRET", "PASSWORD"))}
    lib = env.get("LD_LIBRARY_PATH", "")
    env.update({
        "API_KEY": "sk-local-dummy", "BASE_URL": judge_url or JUDGE_URL, "VL_MODEL": "qwen3-vl-plus", "MODEL": "local-llm",
        "NO_PROXY": "127.0.0.1,localhost", "no_proxy": "127.0.0.1,localhost",
        "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y", "PRIVACY_CONSENT": "Y",
        "CUDA_VISIBLE_DEVICES": str(gpu),
        "SIM_REPO_ROOT": str(run_dir / "root"), "SIM_ASSETS": ASSETS, "GENIESIM_ASSETS_PATH": ASSETS,
        "GENIESIM_KIT_RUNTIME_DIR": str(run_dir / "kit"), "GENIESIM_OMNI_DOCUMENTS_DIR": str(run_dir / "documents"),
        "WARP_CACHE_PATH": str(run_dir / "warp"), "XDG_RUNTIME_DIR": str(run_dir / "xdg"),
        "ROS_DISTRO": "jazzy", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "ROS_VERSION": "2",
        "ROS_PYTHON_VERSION": "3", "ROS_LOCALHOST_ONLY": "1", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "ROS_DOMAIN_ID": str(int(domain)),
        "LD_LIBRARY_PATH": (lib + ":" if lib else "") + str(bridge / "lib"),
        "PYTHONPATH": str(bridge / "rclpy"),
        "PATH": str(run_dir / "bin") + ":" + env.get("PATH", "/usr/bin:/bin"),
    })
    return env


def launch(run_dir: Path, gpu: str, config: str, overrides: list[str], *, domain: int,
           log_path: Path | None = None, judge_url: str | None = None) -> subprocess.Popen:
    """Start ``app.py`` on one benchmark config (``g2op_...``, no .yaml) with ``--benchmark.<k>=<v>``
    overrides; its output goes to ``log_path`` (default ``<run dir>/bench.log``)."""
    run_dir = Path(run_dir).resolve()
    yaml = package() / "config" / f"{config}.yaml"
    if not yaml.exists():
        raise FileNotFoundError(f"no such benchmark config: {yaml}")
    for sub in ("kit", "documents", "warp", "xdg", "bin"):
        (run_dir / sub).mkdir(parents=True, exist_ok=True)
    os.chmod(run_dir / "xdg", 0o700)
    tool, link = ffmpeg(), run_dir / "bin" / "ffmpeg"
    if tool is not None and not link.is_symlink() and not link.exists():
        link.symlink_to(tool)
    overlay(run_dir)
    log = open(log_path or run_dir / "bench.log", "ab")
    cmd = [PYTHON, str(package() / "app" / "app.py"), "--config", str(yaml), "--app.headless=true", *overrides]
    try:
        return subprocess.Popen(cmd, cwd=run_dir, env=environment(run_dir, gpu, domain, judge_url), stdout=log,
                                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    finally:
        log.close()
