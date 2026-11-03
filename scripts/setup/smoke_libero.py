#!/usr/bin/env python3
"""Headless smoke test for the LIBERO-PRO simulator venv ("bench-libero").

Each case builds one LIBERO-PRO env the way RPent does (RPent
robots/libero/env_server.py make_env -> RLinf LiberoEnv). It then checks:
  * rendering: RGB + depth for agentview and the wrist camera
  * camera geometry: K from fovy, cam->world extrinsics, depth in metres, and
    back-projection checked against MuJoCo ray casts and a sim body position
  * OSC_POSE stepping: action format and limits, EE pose and gripper per step,
    base frame vs world frame
  * success check, horizon / max-episode-steps behaviour
  * timing and GPU memory

How RPent builds an episode (seed -> init state):
    trials = len(suite.get_task_init_states(task)); trial = seed % trials
    env = OffScreenRenderEnv(bddl_file_name=..., camera_heights=256,
                             camera_widths=256, camera_depths=True,
                             horizon=max_episode_steps)
    env.seed(seed); env.reset(); env.set_init_state(init_states[trial])
    15 x env.step([0, 0, 0, 0, 0, 0, -1])   # settle with the gripper open

Usage (env vars from the env.sh setup_libero.sh writes, by default .bench-data/libero-pro/env.sh):
    python smoke_libero.py [--gpu N] [--out DIR]   # every case, one subprocess each
    python smoke_libero.py --case object_swap      # a single case in this process
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

# MuJoCo GL backend must be chosen before mujoco/robosuite are imported.
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
assert "mujoco" not in sys.modules

import numpy as np  # noqa: E402

# name -> (suite, task_id, seed)
CASES = {
    "object_swap": ("libero_object_swap", 2, 0),  # RPent's own e2e case
    "goal_swap_drawer": ("libero_goal_swap", 0, 0),  # drawer (and a stove in the scene)
    "10_task_drawer": ("libero_10_task", 3, 7),  # libero_10 kitchen drawer, seed 7 -> trial 7
}
SETTLE_STEPS = 15  # RLinf LiberoEnv.reset(): 15 zero actions with gripper -1
CAM_HW = 256  # RPent init_params camera_heights/widths
CAMERAS = ("agentview", "robot0_eye_in_hand")
OPEN = np.array([0, 0, 0, 0, 0, 0, -1.0])
np.set_printoptions(precision=4, suppress=True, linewidth=160)


def fmt(a, p=4):
    return np.array2string(np.asarray(a, dtype=float), precision=p, suppress_small=True, separator=",")


# --------------------------------------------------------------------------
# GPU / EGL selection
# --------------------------------------------------------------------------
def norm_pci(bus: str) -> str:
    """'00000000:61:00.0' -> '0000:61:00.0' (sysfs spelling)."""
    bus = bus.strip().lower()
    dom, rest = bus.split(":", 1)
    return "%04x:%s" % (int(dom, 16), rest)


def nvsmi_gpus():
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,pci.bus_id,memory.used,memory.free,name",
         "--format=csv,noheader,nounits"],
        capture_output=True, text=True, check=True).stdout
    gpus = []
    for line in out.strip().splitlines():
        idx, bus, used, free, name = [x.strip() for x in line.split(",", 4)]
        gpus.append({"index": int(idx), "pci": norm_pci(bus), "used_mib": int(used),
                     "free_mib": int(free), "name": name})
    return gpus


def egl_device_table():
    """One row per EGL device: CUDA ordinal (EGL_NV_device_cuda, what RPent's
    rpent/utils/egl.py uses) and the PCI address (EGL_EXT_device_drm -> sysfs)."""
    from mujoco.egl import egl_ext as EGL
    from OpenGL import EGL as GLEGL
    from OpenGL.EGL.EXT.device_base import eglQueryDeviceStringEXT

    proc = GLEGL.eglGetProcAddress("eglQueryDeviceAttribEXT")
    query = ctypes.cast(int(proc), ctypes.CFUNCTYPE(
        ctypes.c_uint, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_ssize_t)))
    rows = []
    for egl_index, dev in enumerate(EGL.eglQueryDevicesEXT()):
        ext = (eglQueryDeviceStringEXT(dev, 0x3055) or b"").decode(errors="replace")
        cuda = drm = pci = None
        if "EGL_NV_device_cuda" in ext:
            val = ctypes.c_ssize_t(-1)
            addr = ctypes.cast(dev, ctypes.c_void_p).value
            if query(ctypes.c_void_p(addr), 0x323A, ctypes.byref(val)) == 1 and val.value >= 0:
                cuda = int(val.value)
        if "EGL_EXT_device_drm" in ext:
            try:
                s = eglQueryDeviceStringEXT(dev, 0x3233)  # EGL_DRM_DEVICE_FILE_EXT
            except Exception:  # noqa: BLE001 - EGL_BAD_PARAMETER when this user may not open the node (H800)
                s = None
            if s:
                drm = s.decode() if isinstance(s, bytes) else str(s)
                sysdev = os.path.realpath(f"/sys/class/drm/{os.path.basename(drm)}/device")
                pci = os.path.basename(sysdev).lower()
        rows.append({"egl": egl_index, "cuda": cuda, "drm": drm, "pci": pci})
    return rows


def select_render_device(gpu_index):
    """Pin MuJoCo EGL rendering to nvidia-smi GPU `gpu_index` (most free memory if None).

    Same approach as RPent env_server --cuda-device: CUDA_VISIBLE_DEVICES is cleared
    (robosuite asserts MUJOCO_EGL_DEVICE_ID is a substring of it) and
    MUJOCO_EGL_DEVICE_ID is set to the matching EGL device index."""
    gpus = nvsmi_gpus()
    if gpu_index is None:
        gpu_index = max(gpus, key=lambda g: g["free_mib"])["index"]
    gpu = next(g for g in gpus if g["index"] == gpu_index)
    os.environ.pop("CUDA_VISIBLE_DEVICES", None)
    table = egl_device_table()
    # Only NVIDIA EGL devices (those with EGL_NV_device_cuda) are usable: Mesa can list the same
    # card again as an extra EGL device (same DRM node / PCI address) that fails eglInitialize.
    nvidia = [r for r in table if r["cuda"] is not None]
    by_cuda = {r["cuda"]: r["egl"] for r in nvidia}
    by_pci = {}
    for r in nvidia:
        if r["pci"] is not None:
            by_pci.setdefault(r["pci"], r["egl"])
    egl_pci = by_pci.get(gpu["pci"])
    egl_cuda = by_cuda.get(gpu_index)  # RPent: assumes CUDA ordinal == nvidia-smi index
    egl = egl_pci if egl_pci is not None else egl_cuda
    if egl is None:
        raise RuntimeError(f"no EGL device for GPU {gpu_index} (table={table})")
    os.environ["MUJOCO_EGL_DEVICE_ID"] = str(egl)
    return {"gpu_index": gpu_index, "gpu_pci": gpu["pci"], "gpu_free_mib_before": gpu["free_mib"],
            "egl_index": egl, "egl_by_pci": egl_pci, "egl_by_cuda_ordinal": egl_cuda,
            "egl_table": table}


def gpu_mem_of_pid(pid):
    root = ET.fromstring(subprocess.run(["nvidia-smi", "-q", "-x"], capture_output=True,
                                        text=True, check=True).stdout)
    res = []
    for gpu in root.findall("gpu"):
        for p in gpu.findall("processes/process_info"):
            if p.findtext("pid") == str(pid):
                res.append({"pci": norm_pci(gpu.findtext("pci/pci_bus_id")),
                            "type": p.findtext("type"), "used_memory": p.findtext("used_memory")})
    return res


# --------------------------------------------------------------------------
# Env construction (RPent / RLinf semantics)
# --------------------------------------------------------------------------
def build_env(suite_name, task_id, seed, max_episode_steps):
    import liberopro.liberopro.benchmark as bench
    from liberopro.liberopro import get_libero_path
    from liberopro.liberopro.envs import OffScreenRenderEnv

    suite = bench.get_benchmark(suite_name)()
    init_states = suite.get_task_init_states(task_id)
    trials = len(init_states)
    if trials == 0:
        raise RuntimeError(f"{suite_name} task {task_id} has 0 init states "
                           "(RPent would fail with seed % 0)")
    first_id = sum(len(suite.get_task_init_states(t)) for t in range(task_id))
    reset_id = first_id + (seed % trials)  # RPent make_env -> specific_reset_id
    trial_id = reset_id - first_id  # RLinf maps the reset id back to (task, trial)
    task = suite.get_task(task_id)
    bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    kwargs = dict(bddl_file_name=bddl, camera_heights=CAM_HW, camera_widths=CAM_HW,
                  camera_depths=True, horizon=max_episode_steps)
    if os.environ.get("LIBERO_ROBOT_BASE"):
        kwargs["robots"] = [os.environ["LIBERO_ROBOT_BASE"]]
    t0 = time.perf_counter()
    env = OffScreenRenderEnv(**kwargs)
    env.seed(seed)  # RLinf env_fn
    t_build = time.perf_counter() - t0
    info = {"suite": suite_name, "task_id": task_id, "seed": seed, "trials": trials,
            "reset_state_id": reset_id, "trial_id": trial_id, "language": task.language,
            "bddl": bddl, "env_kwargs": {k: v for k, v in kwargs.items() if k != "bddl_file_name"},
            "t_build_s": t_build}
    return env, init_states[trial_id], info


def rlinf_reset(env, seed, init_state):
    """RLinf LiberoEnv.reset(): seed, reset, set_init_state, then 15 settle steps."""
    t = {}
    t0 = time.perf_counter()
    env.seed(seed)  # LiberoEnv._reconfigure: env.seed(self.seed * num_envs)
    env.reset()
    t["env_reset_s"] = time.perf_counter() - t0
    t1 = time.perf_counter()
    env.set_init_state(init_state)
    t["set_init_state_s"] = time.perf_counter() - t1
    t2 = time.perf_counter()
    for _ in range(SETTLE_STEPS):
        obs, _r, _d, _i = env.step(OPEN)
    t["settle_s"] = time.perf_counter() - t2
    t["total_s"] = time.perf_counter() - t0
    return obs, t


# --------------------------------------------------------------------------
# Camera geometry
# --------------------------------------------------------------------------
def camera_meta(sim, cam, h, w):
    """Exactly RLinf venv.py 'get_camera_meta' (what RPent's tools consume)."""
    from robosuite.utils import camera_utils as cu
    K = cu.get_camera_intrinsic_matrix(sim, cam, h, w)
    E = cu.get_camera_extrinsic_matrix(sim, cam)  # cam->world, OpenCV axes (x right, y down, z fwd)
    extent = float(sim.model.stat.extent)
    return {"K": K, "E": E, "near": float(sim.model.vis.map.znear) * extent,
            "far": float(sim.model.vis.map.zfar) * extent,
            "fovy_deg": float(sim.model.cam_fovy[sim.model.camera_name2id(cam)])}


def depth_to_m(d01, near, far):
    return near / (1.0 - d01 * (1.0 - near / far))


def pixel_dirs(K, E, rows, cols, off):
    """World-frame ray directions scaled so that camera-z = 1 (so t == depth)."""
    x = (cols + off - K[0, 2]) / K[0, 0]
    y = (rows + off - K[1, 2]) / K[1, 1]
    d_cam = np.stack([x, y, np.ones_like(x, dtype=float)], -1)
    return d_cam @ E[:3, :3].T


def raycast(sim, geomgroup, origin, dirs):
    import mujoco
    m, d = sim.model._model, sim.data._data
    gid = np.zeros(1, dtype=np.int32)
    ts = np.full(len(dirs), -1.0)
    geoms = np.full(len(dirs), -1, dtype=int)
    for i, v in enumerate(dirs):
        ts[i] = mujoco.mj_ray(m, d, np.asarray(origin, np.float64), np.asarray(v, np.float64),
                              geomgroup, 1, -1, gid)
        geoms[i] = int(gid[0])
    return ts, geoms


def grid_check(sim, meta, depth_m, geomgroup, n=24):
    """Depth-buffer back-projection vs MuJoCo ray cast through the same pixel."""
    h, w = depth_m.shape
    rr, cc = np.meshgrid(np.linspace(4, h - 5, n).astype(int), np.linspace(4, w - 5, n).astype(int),
                         indexing="ij")
    rows, cols = rr.ravel(), cc.ravel()
    out = {}
    for label, off in (("pixel_center_+0.5", 0.5), ("pixel_corner_RPent", 0.0)):
        dirs = pixel_dirs(meta["K"], meta["E"], rows, cols, off)
        t, _g = raycast(sim, geomgroup, meta["E"][:3, 3], dirs)
        z = depth_m[rows, cols]
        ok = (t > 0) & (z < 0.999 * meta["far"])
        err = np.abs(z[ok] - t[ok]) * np.linalg.norm(dirs[ok], axis=1) * 1000.0  # 3D mm
        out[label] = {"n": int(ok.sum()), "median_mm": float(np.median(err)),
                      "p90_mm": float(np.percentile(err, 90)), "frac_lt_5mm": float(np.mean(err < 5.0))}
    return out


def body_root_name(model, body_id):
    names = []
    while body_id > 0:
        names.append(model.body(body_id).name)
        body_id = int(model.body_parentid[body_id])
    return names  # [self, parent, ..., top-level body]


def body_check(sim, meta, depth_m, seg, body_name):
    """Project a body's origin into the image, back-project the depth there."""
    bid = sim.model.body_name2id(body_name)
    return point_check(sim, meta, depth_m, seg, body_name, np.array(sim.data.body_xpos[bid]))


def site_check(sim, meta, depth_m, seg, site_name):
    """Same as body_check for a site (LIBERO regions such as 'wooden_cabinet_1_middle_region')."""
    sid = sim.model.site_name2id(site_name)
    return point_check(sim, meta, depth_m, seg, "site:" + site_name, np.array(sim.data.site_xpos[sid]))


def point_check(sim, meta, depth_m, seg, body_name, p):
    """Project world point p into the image and back-project the depth at that pixel."""
    model = sim.model._model
    K, E = meta["K"], meta["E"]
    Einv = np.linalg.inv(E)
    pc = Einv[:3, :3] @ p + Einv[:3, 3]
    u = K[0, 0] * pc[0] / pc[2] + K[0, 2]
    v = K[1, 1] * pc[1] / pc[2] + K[1, 2]
    row, col = int(np.floor(v)), int(np.floor(u))
    h, w = depth_m.shape
    if not (0 <= row < h and 0 <= col < w and pc[2] > 0):
        return {"body": body_name, "visible": False, "u": float(u), "v": float(v)}
    o = E[:3, 3]
    d = pixel_dirs(K, E, np.array([row]), np.array([col]), 0.5)[0]
    z = float(depth_m[row, col])
    P = o + d * z
    n_hat = (p - o) / np.linalg.norm(p - o)
    lateral = np.linalg.norm((P - o) - np.dot(P - o, n_hat) * n_hat)
    t, g = raycast(sim, meta.get("geomgroup"), o, (p - o)[None] / pc[2])
    H = o + (p - o) / pc[2] * t[0]
    objtype, objid = int(seg[row, col, 0]), int(seg[row, col, 1])
    seg_body = None
    import mujoco
    if objtype == int(mujoco.mjtObj.mjOBJ_GEOM) and objid >= 0:
        seg_body = body_root_name(model, int(model.geom_bodyid[objid]))
    ray_body = body_root_name(model, int(model.geom_bodyid[g[0]])) if g[0] >= 0 else None
    return {"body": body_name, "visible": True, "body_pos": p.tolist(), "pixel_rc": [row, col],
            "uv": [float(u), float(v)], "depth_m": z, "backprojected": P.tolist(),
            "raycast_hit_through_body": H.tolist(),
            "dist_backproj_to_raycast_mm": float(np.linalg.norm(P - H) * 1000),
            "dist_backproj_to_body_origin_mm": float(np.linalg.norm(P - p) * 1000),
            "lateral_offset_from_body_ray_mm": float(lateral * 1000),
            "segmentation_body_chain": seg_body, "raycast_body_chain": ray_body}


def fixture_poses(rs):
    """Fixture root-body pose from the *model* (fixtures have no joints, so they are not in the sim state)."""
    out = {}
    for k, f in rs.fixtures_dict.items():
        bid = rs.sim.model.body_name2id(f.root_body)
        out[k] = np.concatenate([rs.sim.model.body_pos[bid], rs.sim.model.body_quat[bid]])
    return out


def save_png(path, arr):
    from PIL import Image
    Image.fromarray(arr).save(path)


def depth_vis(depth_m, lo=None, hi=None):
    lo = np.percentile(depth_m, 1) if lo is None else lo
    hi = np.percentile(depth_m, 99) if hi is None else hi
    x = np.clip((depth_m - lo) / max(hi - lo, 1e-6), 0, 1)
    return (255 * (1 - x)).astype(np.uint8)


# --------------------------------------------------------------------------
# One case
# --------------------------------------------------------------------------
def run_case(name, args):
    import mujoco
    suite_name, task_id, seed = CASES[name]
    res = {"case": name, "pid": os.getpid()}
    res["render_device"] = select_render_device(args.gpu)
    print(f"\n==================== case {name}: {suite_name} task {task_id} seed {seed} ====================")
    print(f"[gpu] nvidia-smi GPU {res['render_device']['gpu_index']} ({res['render_device']['gpu_pci']}) -> "
          f"MUJOCO_EGL_DEVICE_ID={os.environ['MUJOCO_EGL_DEVICE_ID']} "
          f"(by PCI {res['render_device']['egl_by_pci']}, by CUDA ordinal {res['render_device']['egl_by_cuda_ordinal']})")

    t0 = time.perf_counter()
    import robosuite  # noqa: F401
    import liberopro.liberopro.benchmark  # noqa: F401  (imports torch, CPU only)
    res["t_import_s"] = time.perf_counter() - t0

    env, init_state, info = build_env(suite_name, task_id, seed, args.max_episode_steps)
    res.update(info)
    obs, t_reset = rlinf_reset(env, seed, init_state)
    res["t_first_reset"] = t_reset
    state0 = env.get_sim_state().copy()
    img0 = obs["agentview_image"].copy()
    rs, sim = env.env, env.sim
    robot = rs.robots[0]
    print(f"[task] {info['language']!r}  trials={info['trials']} reset_state_id={info['reset_state_id']} "
          f"trial={info['trial_id']}  init_state_dim={len(init_state)}")
    print(f"[task] bddl={info['bddl']}")
    print(f"[time] imports {res['t_import_s']:.2f}s  build {info['t_build_s']:.2f}s  first reset "
          f"{t_reset['total_s']:.2f}s (env.reset {t_reset['env_reset_s']:.2f}s, set_init_state "
          f"{t_reset['set_init_state_s']:.3f}s, {SETTLE_STEPS} settle steps {t_reset['settle_s']:.2f}s)")

    # ---- robot / controller / action space -----------------------------------
    ctrl = robot.part_controllers["right"]
    base_pos, base_R = robot.composite_controller.get_controller_base_pose("right")
    root_bid = sim.model.body_name2id(robot.robot_model.root_body)
    low, high = rs.action_spec
    gripper = robot.gripper["right"]
    res["robot"] = {"class": type(robot.robot_model).__name__, "root_body": robot.robot_model.root_body,
                    "root_pos": sim.data.body_xpos[root_bid].tolist(),
                    "controller_origin_pos": base_pos.tolist(), "controller_origin_R": base_R.tolist(),
                    "composite": type(robot.composite_controller).__name__, "arm_ctrl": type(ctrl).__name__,
                    "input_type": ctrl.input_type, "input_ref_frame": ctrl.input_ref_frame,
                    "goal_update_mode": getattr(ctrl, "_goal_update_mode", None),
                    "input_min": ctrl.input_min.tolist(), "input_max": ctrl.input_max.tolist(),
                    "output_min": ctrl.output_min.tolist(), "output_max": ctrl.output_max.tolist(),
                    "kp": ctrl.kp.tolist(), "gripper": type(gripper).__name__, "gripper_speed": gripper.speed,
                    "action_dim": rs.action_dim, "action_low": low.tolist(), "action_high": high.tolist(),
                    "control_freq": rs.control_freq, "model_timestep": float(sim.model.opt.timestep),
                    "horizon": rs.horizon, "ignore_done": rs.ignore_done, "hard_reset": rs.hard_reset}
    r = res["robot"]
    print(f"[robot] {r['class']} root_body={r['root_body']} root_pos={fmt(r['root_pos'])}")
    print(f"[robot] OSC origin (site robot0_right_center) pos={fmt(base_pos)} R=\n{fmt(base_R, 3)}")
    print(f"[action] dim={r['action_dim']} low={fmt(low, 2)} high={fmt(high, 2)}")
    print(f"[action] {r['composite']} / {r['arm_ctrl']} input_type={r['input_type']} ref_frame={r['input_ref_frame']} "
          f"goal_update={r['goal_update_mode']} input [-1,1] -> output_max={fmt(r['output_max'], 3)} "
          f"(m, m, m, rad, rad, rad) kp={fmt(r['kp'], 0)}")
    print(f"[action] gripper {r['gripper']} speed={r['gripper_speed']} (a[6]: -1 open, +1 close, integrated)")
    print(f"[timing] control_freq={r['control_freq']} Hz, physics dt={r['model_timestep']} s -> "
          f"{round(1.0 / (r['control_freq'] * r['model_timestep']))} substeps/step; horizon={r['horizon']} "
          f"ignore_done={r['ignore_done']} hard_reset={r['hard_reset']}")

    # ---- observations / objects ---------------------------------------------
    res["obs_keys"] = {k: [list(np.shape(v)), str(np.asarray(v).dtype)] for k, v in obs.items()}
    print("[obs] " + ", ".join(f"{k}{tuple(np.shape(v))}" for k, v in obs.items()))
    objs = {k: {"class": type(v).__name__, "root_body": v.root_body,
                "pos": sim.data.body_xpos[rs.obj_body_id[k]].round(4).tolist()}
            for k, v in rs.objects_dict.items()}
    fixtures = {k: {"class": type(v).__name__, "root_body": v.root_body,
                    "pos": sim.data.body_xpos[rs.obj_body_id[k]].round(4).tolist()}
                for k, v in rs.fixtures_dict.items()}
    res["objects"], res["fixtures"] = objs, fixtures
    res["obj_of_interest"] = list(rs.obj_of_interest)
    res["goal_state"] = rs.parsed_problem["goal_state"]
    res["object_states"] = sorted(rs.object_states_dict.keys())
    print(f"[objects] {objs}")
    print(f"[fixtures] {fixtures}")
    print(f"[goal] obj_of_interest={res['obj_of_interest']} goal_state={res['goal_state']}")

    # ---- cameras --------------------------------------------------------------
    geomgroup = np.array(sim._render_context_offscreen.vopt.geomgroup, dtype=np.uint8)
    res["render_geomgroup"] = geomgroup.tolist()
    res["cameras"] = {}
    os.makedirs(args.out, exist_ok=True)
    for cam in CAMERAS:
        meta = camera_meta(sim, cam, CAM_HW, CAM_HW)
        meta["geomgroup"] = geomgroup
        raw_rgb, raw_d = sim.render(width=CAM_HW, height=CAM_HW, camera_name=cam, depth=True)  # RPent render_camera
        same_as_obs = (np.array_equal(raw_rgb, obs[f"{cam}_image"]),
                       float(np.abs(raw_d - obs[f"{cam}_depth"][..., 0]).max()))
        rgb = obs[f"{cam}_image"][::-1]  # bottom-up OpenGL buffer -> top-down (matches K/E)
        d01 = obs[f"{cam}_depth"][::-1, :, 0]
        depth_m = depth_to_m(d01, meta["near"], meta["far"])
        grid = grid_check(sim, meta, depth_m, geomgroup)
        grid_unflipped = grid_check(sim, meta, depth_to_m(obs[f"{cam}_depth"][:, :, 0], meta["near"],
                                                           meta["far"]), geomgroup)
        seg = sim.render(width=CAM_HW, height=CAM_HW, camera_name=cam, segmentation=True)[::-1]
        cres = {"K": meta["K"].tolist(), "E_cam2world": meta["E"].tolist(), "fovy_deg": meta["fovy_deg"],
                "near_m": meta["near"], "far_m": meta["far"], "rgb_dtype": str(rgb.dtype),
                "depth_raw_range": [float(d01.min()), float(d01.max())],
                "depth_m_range": [float(depth_m.min()), float(depth_m.max())],
                "sim_render_equals_obs(rgb_equal, depth_maxdiff)": same_as_obs,
                "grid_check_flipped": grid, "grid_check_NOT_flipped": grid_unflipped}
        if cam == "agentview":
            checks = []
            for oname in dict.fromkeys(rs.obj_of_interest):  # may repeat names
                if oname in rs.objects_dict or oname in rs.fixtures_dict:
                    obj = rs.objects_dict.get(oname) or rs.fixtures_dict[oname]
                    checks.append(body_check(sim, meta, depth_m, seg, obj.root_body))
                elif oname in rs.object_sites_dict:  # a region site; also check its parent object
                    checks.append(site_check(sim, meta, depth_m, seg, oname))
                    parent = rs.get_object(rs.object_sites_dict[oname].parent_name)
                    if parent is not None and hasattr(parent, "root_body"):
                        checks.append(body_check(sim, meta, depth_m, seg, parent.root_body))
            cres["body_checks"] = checks
            # mark the projected pixels to eyeball the flip convention
            marked = rgb.copy()
            for c in checks:
                if c.get("visible"):
                    rr, cc = c["pixel_rc"]
                    marked[max(rr - 4, 0):rr + 5, cc] = (255, 0, 0)
                    marked[rr, max(cc - 4, 0):cc + 5] = (255, 0, 0)
            save_png(os.path.join(args.out, f"{name}_{cam}_rgb_marked.png"), marked)
        save_png(os.path.join(args.out, f"{name}_{cam}_rgb.png"), rgb)
        save_png(os.path.join(args.out, f"{name}_{cam}_depth.png"), depth_vis(depth_m))
        res["cameras"][cam] = cres
        print(f"[cam {cam}] fovy={meta['fovy_deg']:.1f}deg K=\n{fmt(meta['K'], 2)}\n  E(cam->world)=\n{fmt(meta['E'], 4)}")
        print(f"  near={meta['near']:.4f} far={meta['far']:.2f} raw depth [{d01.min():.4f},{d01.max():.4f}] -> "
              f"metres [{depth_m.min():.3f},{depth_m.max():.3f}]  sim.render==obs: {same_as_obs}")
        print(f"  grid back-projection vs mj_ray (flipped [::-1]): {grid}")
        print(f"  same, but image NOT flipped (wrong):             {grid_unflipped}")
        for c in cres.get("body_checks", []):
            print(f"  body check: {c}")

    # ---- OSC stepping -----------------------------------------------------------
    plan = ([("+x", [1, 0, 0, 0, 0, 0, -1])] * 8 + [("+y", [0, 1, 0, 0, 0, 0, -1])] * 4
            + [("+yaw", [0, 0, 0, 0, 0, 0.5, -1])] * 2 + [("close", [0, 0, 0, 0, 0, 0, 1])] * 6)
    plan = plan[:args.steps]
    from robosuite.utils.transform_utils import quat2mat
    steps, prev = [], np.array(obs["robot0_eef_pos"])
    start = prev.copy()
    print("[osc] step  phase  action                         eef_pos(world)            d_world(mm)"
          "            d_base(mm)             yaw   pitch  grip_qpos        open(mm) succ done  ms")
    for i, (phase, a) in enumerate(plan, 1):
        t0 = time.perf_counter()
        obs, rew, done, _info = env.step(np.array(a, dtype=float))
        dt = time.perf_counter() - t0
        succ = bool(env.check_success())
        pos = np.array(obs["robot0_eef_pos"])
        dw = (pos - prev) * 1000
        db = base_R.T @ dw
        R = quat2mat(obs["robot0_eef_quat"])
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        pitch = math.degrees(math.atan2(R[1, 2], -R[2, 2]))
        gq = np.array(obs["robot0_gripper_qpos"])
        opening = float(abs(gq[0]) + abs(gq[1]))
        # robosuite samples observables inside the substep loop, not after it: obs lags the sim by >= 1 substep
        lag = float(np.linalg.norm(pos - sim.data.site_xpos[robot.eef_site_id["right"]]) * 1000)
        steps.append({"i": i, "phase": phase, "action": a, "eef_pos": pos.tolist(), "obs_vs_sim_eef_mm": lag,
                      "eef_quat_xyzw": np.asarray(obs["robot0_eef_quat"]).tolist(), "d_world_mm": dw.tolist(),
                      "d_base_mm": db.tolist(), "yaw_deg": yaw, "pitch_deg": pitch, "gripper_qpos": gq.tolist(),
                      "opening_m": opening, "success": succ, "done": bool(done), "reward": float(rew),
                      "wall_ms": dt * 1000})
        print(f"[osc] {i:4d}  {phase:5s} {fmt(a, 1):30s} {fmt(pos):25s} {fmt(dw, 1):22s} {fmt(db, 1):22s} "
              f"{yaw:6.1f} {pitch:6.1f} {fmt(gq, 4):16s} {opening * 1000:6.1f}   {int(succ)}    {int(done)}  {dt * 1000:5.1f}")
        prev = pos
    res["osc_steps"] = steps
    lags = [s["obs_vs_sim_eef_mm"] for s in steps]
    res["obs_lag_eef_mm_max"] = max(lags)
    print(f"[osc] obs['robot0_eef_pos'] vs end-of-step sim eef (site_xpos): max {max(lags):.2f} mm "
          f"(obs is sampled at substep 24/25; rs._get_observations(force_update=True) gives end-of-step values)")
    xs = [s for s in steps if s["phase"] == "+x"]
    if xs:
        tot = np.array(xs[-1]["eef_pos"]) - start
        res["plus_x_total_world_mm"] = (tot * 1000).tolist()
        res["plus_x_total_base_mm"] = (base_R.T @ tot * 1000).tolist()
        print(f"[osc] +x phase total displacement: world {fmt(tot * 1000, 1)} mm, base {fmt(base_R.T @ tot * 1000, 1)} mm")
    ms = np.array([s["wall_ms"] for s in steps])
    res["step_ms_with_cams"] = {"mean": float(ms.mean()), "median": float(np.median(ms)), "max": float(ms.max())}

    # step cost without camera observables
    cam_obs = [f"{c}_image" for c in CAMERAS] + [f"{c}_depth" for c in CAMERAS]
    for k in cam_obs:
        rs.modify_observable(k, "enabled", False)
    t_noc = []
    for _ in range(10):
        t0 = time.perf_counter()
        env.step(OPEN)
        t_noc.append(time.perf_counter() - t0)
    for k in cam_obs:
        rs.modify_observable(k, "enabled", True)
    res["step_ms_no_cams"] = {"mean": float(np.mean(t_noc) * 1000), "median": float(np.median(t_noc) * 1000)}
    print(f"[timing] env.step with 2x RGB-D {CAM_HW}px: mean {ms.mean():.1f} ms (median {np.median(ms):.1f}); "
          f"without camera obs: mean {np.mean(t_noc) * 1000:.1f} ms")

    # hi-res render as used by RPent render_camera (default 1024x1024)
    t_hr = []
    for _ in range(3):
        t0 = time.perf_counter()
        hr_rgb, hr_d = sim.render(width=1024, height=1024, camera_name="agentview", depth=True)
        t_hr.append(time.perf_counter() - t0)
    res["t_render_1024_rgbd_s"] = t_hr
    save_png(os.path.join(args.out, f"{name}_agentview_1024.png"), hr_rgb[::-1])
    t0 = time.perf_counter()
    sim.render(width=CAM_HW, height=CAM_HW, camera_name="agentview", depth=True)
    res["t_render_256_rgbd_s"] = time.perf_counter() - t0
    print(f"[render] 1024x1024 RGB-D agentview: {', '.join(f'{t * 1000:.0f}' for t in t_hr)} ms (3 calls), "
          f"shape {hr_rgb.shape} {hr_d.shape}; 256x256 RGB-D: {res['t_render_256_rgbd_s'] * 1000:.1f} ms")

    # ---- horizon / max episode steps --------------------------------------------
    saved_h = rs.horizon
    rs.horizon = rs.timestep + 2
    hz = []
    for k in range(3):
        try:
            _o, _r, d, _ = env.step(OPEN)
            hz.append(f"step {k + 1}: ok, returned done={d}, robosuite env.done={rs.done}, timestep={rs.timestep}")
        except ValueError as e:
            hz.append(f"step {k + 1}: ValueError: {e}")
    rs.horizon = saved_h
    res["horizon_demo"] = hz
    print(f"[horizon] horizon set to timestep+2: {hz}")

    # ---- second reset: timing + determinism ---------------------------------------
    # LIBERO re-samples fixture placement (sim.model.body_pos of cabinets, stoves, racks, ...) on every
    # env.reset() from the global np.random state, which nothing seeds (robosuite 1.5 broke env.seed()).
    # The init state (qpos/qvel) does not cover it, so only np.random.seed() before env.reset() reproduces it.
    fix0 = fixture_poses(rs)
    obs2, t_reset2 = rlinf_reset(env, seed, init_state)  # as RPent: global np.random not seeded
    state1 = env.get_sim_state()
    fix1 = fixture_poses(rs)
    res["t_second_reset"] = t_reset2
    det = {"state_maxabs": float(np.abs(state1 - state0).max()),
           "agentview_img_maxabs": int(np.abs(obs2["agentview_image"].astype(int) - img0.astype(int)).max()),
           "fixture_pose_maxabs_mm": max([float(np.abs(fix1[k] - fix0[k]).max()) * 1000 for k in fix0], default=0.0)}
    seeded = []
    for _ in range(2):
        np.random.seed(seed)
        o, _t = rlinf_reset(env, seed, init_state)
        seeded.append((env.get_sim_state(), o["agentview_image"].astype(int), fixture_poses(rs)))
    (sa, ia, fa), (sb, ib, fb) = seeded
    det["with_np_random_seed"] = {"state_maxabs": float(np.abs(sa - sb).max()),
                                  "agentview_img_maxabs": int(np.abs(ia - ib).max()),
                                  "fixture_pose_maxabs_mm": max([float(np.abs(fa[k] - fb[k]).max()) * 1000
                                                                 for k in fa], default=0.0)}
    res["reset_determinism"] = det
    print(f"[reset] second reset {t_reset2['total_s']:.2f}s (env.reset {t_reset2['env_reset_s']:.2f}s); "
          f"determinism vs first reset (unseeded np.random, as RPent): {det}")
    print(f"[reset] fixture poses (sim.model.body_pos) first/second reset: "
          + "; ".join(f"{k} {fmt(fix0[k][:3], 4)} / {fmt(fix1[k][:3], 4)}" for k in fix0))

    res["gpu_mem"] = gpu_mem_of_pid(os.getpid())
    print(f"[gpu] this process: {res['gpu_mem']}")
    env.close()
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", choices=sorted(CASES), default=None)
    ap.add_argument("--cases", default=",".join(CASES))
    ap.add_argument("--gpu", type=int, default=None, help="nvidia-smi index (default: most free memory)")
    ap.add_argument("--steps", type=int, default=20)
    ap.add_argument("--max-episode-steps", type=int, default=10000)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "..", "libero-pro", "smoke-out"))
    args = ap.parse_args()
    args.out = os.path.abspath(args.out)
    os.makedirs(args.out, exist_ok=True)
    if args.case:
        res = run_case(args.case, args)
        with open(os.path.join(args.out, f"{args.case}.json"), "w") as f:
            json.dump(res, f, indent=1, default=str)
        return
    gpu = args.gpu
    if gpu is None:
        gpu = max(nvsmi_gpus(), key=lambda g: g["free_mib"])["index"]
    summary = []
    for name in args.cases.split(","):
        t0 = time.perf_counter()
        cmd = [sys.executable, os.path.abspath(__file__), "--case", name, "--gpu", str(gpu),
               "--steps", str(args.steps), "--max-episode-steps", str(args.max_episode_steps), "--out", args.out]
        rc = subprocess.call(cmd)
        wall = time.perf_counter() - t0
        if rc != 0:
            print(f"!! case {name} failed with exit code {rc}")
            summary.append((name, "FAILED", rc))
            continue
        with open(os.path.join(args.out, f"{name}.json")) as f:
            r = json.load(f)
        summary.append((name, r, wall))
    print("\n==================== summary ====================")
    ok = True
    for name, r, wall in summary:
        if r == "FAILED":
            ok = False
            print(f"{name}: FAILED")
            continue
        g = r["cameras"]["agentview"]["grid_check_flipped"]["pixel_center_+0.5"]
        gw = r["cameras"]["robot0_eye_in_hand"]["grid_check_flipped"]["pixel_center_+0.5"]
        bc = [(c["body"], round(c.get("lateral_offset_from_body_ray_mm", float("nan")), 1),
               round(c.get("dist_backproj_to_raycast_mm", float("nan")), 1),
               round(c.get("dist_backproj_to_body_origin_mm", float("nan")), 1))
              for c in r["cameras"]["agentview"].get("body_checks", [])]
        print(f"{name}: {r['suite']} t{r['task_id']} s{r['seed']} (trial {r['trial_id']}) robot={r['robot']['class']} "
              f"process wall {wall:.1f}s | import {r['t_import_s']:.1f}s build {r['t_build_s']:.1f}s "
              f"reset {r['t_first_reset']['total_s']:.2f}s/{r['t_second_reset']['total_s']:.2f}s | "
              f"step {r['step_ms_with_cams']['mean']:.1f}ms (no cams {r['step_ms_no_cams']['mean']:.1f}ms) | "
              f"gpu {r['gpu_mem']}")
        print(f"   agentview grid med {g['median_mm']:.2f}mm <5mm {g['frac_lt_5mm']:.2f}; wrist grid med "
              f"{gw['median_mm']:.2f}mm <5mm {gw['frac_lt_5mm']:.2f}; body (lateral, vs-raycast, vs-origin mm) {bc}; "
              f"+x world mm {fmt(r.get('plus_x_total_world_mm', []), 1)}; determinism {r['reset_determinism']}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
