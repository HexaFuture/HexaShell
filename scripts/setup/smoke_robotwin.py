#!/usr/bin/env python3
"""smoke_robotwin.py -- headless smoke test of the standalone RoboTwin 2.0 simulator env.

Same simulator stack as RPent: rlinf-robotwin-runtime 0.1.1 (SAPIEN 3.0.0b1), cuRobo v0.7.8,
task config demo_randomized (aloha-agilex, D435 320x240), planner_backend="curobo", step limit 10000.

    .venvs/bench-robotwin/bin/python scripts/setup/smoke_robotwin.py [--gpu N] [--out DIR]
        [--planner curobo|mplib] [--skip-expert]

Checks (results printed and written to <out>/smoke_<planner>.json, images to <out>/*.png):
  1. build adjust_bottle (seed 100000) through the runtime's VectorEnv (= what RLinf/RPent use) and
     beat_block_hammer (seed 100000) by constructing the task class directly (no VectorEnv)
  2. head / left / right camera RGB + depth (metres), intrinsics, extrinsics, world pose; wrist cams move;
     back-projection of pixels vs known 3-D points (block top face, table plane)
  3. joint positions, EE / TCP poses (world frame), gripper values of both arms
  4. cuRobo plan + execution of a left-arm motion ~7 cm up (RPent move_to style: 25 qpos take_action steps)
     and back down (one 16-d 'ee' take_action), gripper close/open; step + physics-step accounting, timings
  5. success check (check_success / eval_success), expert demo (play_once) -> success
  6. GPU memory of this process, build/reset times, seed reproducibility, depth on/off scene invariance,
     VectorEnv vs direct construction equivalence
"""
from __future__ import annotations

import argparse
import copy
import faulthandler
import gc
import importlib
import json
import os
import signal
import subprocess
import sys
import time
import traceback

# ----------------------------------------------------------------------------- paths / settings
DATA = os.environ.get("ROBOTWIN_DATA", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                                    ".bench-data", "robotwin"))  # setup_robotwin.sh's DATA
ASSETS_ROOT = os.environ.get("ROBOTWIN_ASSETS_ROOT", os.path.join(DATA, "assets"))
OUT_DEFAULT = os.path.join(DATA, "smoke")
VK_ICD_CANDIDATES = ("/etc/vulkan/icd.d/nvidia_icd.json", "/usr/share/vulkan/icd.d/nvidia_icd.json")
TASK_CONFIG = "demo_randomized"
STEP_LIM = 10000
# first verified seed + published instruction per task, from RPent eval/demo_randomized.json
CASES = {
    "adjust_bottle": (100000, "Pick the Coca-Cola bottle upright from the table"),
    "beat_block_hammer": (100000, "Catch the handheld medium claw hammer and use it on the block."),
}
OTHER_SEED = 100002  # another verified adjust_bottle seed (sanity check: different scene)


def gpu_status() -> list:
    """[(index, memory.free MiB, memory.used MiB, utilization %)] for all GPUs."""
    out = subprocess.run(["nvidia-smi", "--query-gpu=index,memory.free,memory.used,utilization.gpu",
                          "--format=csv,noheader,nounits"], capture_output=True, text=True, check=True,
                         timeout=180).stdout
    return [tuple(int(x) for x in line.split(",")) for line in out.strip().splitlines()]


def pick_gpu() -> int:
    return max(gpu_status(), key=lambda r: r[1])[0]


def configure_process_env(gpu: int) -> None:
    """Must run before torch / sapien / robotwin are imported."""
    os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu)
    if not os.environ.get("VK_ICD_FILENAMES"):
        for cand in VK_ICD_CANDIDATES:
            if os.path.isfile(cand):
                os.environ["VK_ICD_FILENAMES"] = cand
                break
    os.environ["ROBOTWIN_ASSETS_ROOT"] = ASSETS_ROOT      # read by VectorEnv (validate_root)
    os.environ["ROBOTWIN_ASSETS_PATH"] = ASSETS_ROOT      # RPent sets this too
    os.environ["ASSETS_PATH"] = ASSETS_ROOT               # native task helpers read it (VectorEnv sets it as well)


# ----------------------------------------------------------------------------- small helpers
def jsonable(x):
    import numpy as np
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return jsonable(x.tolist())
    if isinstance(x, (np.floating,)):
        return float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, float):
        return round(x, 6)
    return x


def gpu_mem_mib(pid: int) -> dict:
    """Per-process GPU memory (MiB) from `nvidia-smi` (compute + graphics rows), keyed by physical GPU index."""
    res = {}
    try:
        out = subprocess.run(["nvidia-smi"], capture_output=True, text=True, timeout=180).stdout
        for line in out.splitlines():
            parts = line.replace("|", " ").split()
            # |    5   N/A  N/A   123456    C+G   python   1234MiB |
            if len(parts) >= 7 and parts[0].isdigit() and parts[3] == str(pid) and parts[-1].endswith("MiB"):
                res[f"gpu{parts[0]}:{parts[4]}"] = int(parts[-1][:-3])
    except Exception as exc:  # pragma: no cover
        res["error"] = repr(exc)
    return res


class Timer:
    def __init__(self):
        self.t = time.perf_counter()

    def __call__(self) -> float:
        return round(time.perf_counter() - self.t, 3)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gpu", type=int, default=None, help="physical GPU index (default: most free memory)")
    ap.add_argument("--out", default=OUT_DEFAULT)
    ap.add_argument("--planner", default="curobo", choices=["curobo", "mplib"])
    ap.add_argument("--skip-expert", action="store_true", help="skip the play_once expert demo")
    args = ap.parse_args()

    # `kill -USR1 <pid>` dumps all Python thread stacks into the log (the native renderer can stall when the
    # shared GPU runs out of memory; ptrace/py-spy are not available to normal users on this host)
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    gpus = gpu_status()
    gpu = max(gpus, key=lambda r: r[1])[0] if args.gpu is None else args.gpu
    configure_process_env(gpu)
    os.makedirs(args.out, exist_ok=True)
    os.chdir(args.out)  # the native code may write relative ./data paths; keep them in the output dir
    report: dict = {"gpu": gpu, "planner_backend": args.planner, "pid": os.getpid(),
                    "gpu_status_at_start(idx,free,used,util)": gpus,
                    "env": {k: os.environ.get(k) for k in ("CUDA_VISIBLE_DEVICES", "VK_ICD_FILENAMES",
                                                           "ROBOTWIN_ASSETS_ROOT")}}
    print("pid", os.getpid(), "gpu", gpu, "gpu status (idx, free MiB, used MiB, util %):", gpus, flush=True)
    T = Timer()
    T_START = time.perf_counter()

    def stamp(msg: str) -> None:
        print(f"[{time.perf_counter() - T_START:8.1f}s] {msg}", flush=True)

    import numpy as np
    import torch
    import transforms3d as t3d
    import sapien
    import robotwin
    from robotwin.config import load_task_config
    from robotwin.envs.vector_env import VectorEnv
    from robotwin.envs._GLOBAL_CONFIGS import GRASP_DIRECTION_DIC
    import importlib.metadata as md

    report["versions"] = {p: md.version(p) for p in ("rlinf-robotwin-runtime", "sapien", "mplib", "torch", "numpy",
                                                     "warp-lang", "toppra", "transforms3d", "trimesh")}
    try:
        report["versions"]["nvidia_curobo"] = md.version("nvidia_curobo")
    except md.PackageNotFoundError:
        report["versions"]["nvidia_curobo"] = None
    report["import_s"] = T()
    print("imports", report["versions"], flush=True)

    out_json = os.path.join(args.out, f"smoke_{args.planner}.json")

    def save_report(final: bool = False) -> None:
        """Written at every section start too, so a stalled / crashed run still leaves its partial results."""
        report["complete"] = final
        try:
            with open(out_json + ".tmp", "w") as f:
                json.dump(jsonable(report), f, indent=1, default=repr)
            os.replace(out_json + ".tmp", out_json)
        except Exception as exc:  # pragma: no cover
            print("could not write report:", repr(exc), flush=True)

    def section(name):
        save_report()
        print(f"\n======== {name} ======== [t={time.perf_counter() - T_START:.1f}s]", flush=True)

    # ------------------------------------------------------------------ helpers that need numpy / the task
    def native_task_config(task_name: str) -> dict:
        cfg = load_task_config(TASK_CONFIG)          # packaged demo_randomized.yml (plain dict)
        cfg.update(task_name=task_name, task_config=TASK_CONFIG, step_lim=STEP_LIM,
                   planner_backend=args.planner, eval_video_log=False, render_freq=0)
        return cfg

    def scene_objects(task) -> list:
        objs = []
        for ent in task.scene.get_all_actors():
            p = ent.get_pose()
            objs.append((ent.get_name(), np.round(p.p, 5).tolist(), np.round(p.q, 5).tolist()))
        objs.sort()
        return objs

    def compare_objects(a: list, b: list) -> dict:
        if [o[0] for o in a] != [o[0] for o in b]:
            return {"same_names": False, "names_a": [o[0] for o in a], "names_b": [o[0] for o in b]}
        dp = max((float(np.abs(np.array(x[1]) - np.array(y[1])).max()) for x, y in zip(a, b)), default=0.0)
        dq = max((float(min(np.abs(np.array(x[2]) - np.array(y[2])).max(),
                            np.abs(np.array(x[2]) + np.array(y[2])).max())) for x, y in zip(a, b)), default=0.0)
        return {"same_names": True, "n": len(a), "max_dpos_m": dp, "max_dquat": dq}

    def cameras(task) -> dict:
        cams = task.cameras
        head = cams.static_camera_list[cams.static_camera_name.index("head_camera")]
        return {"head": head, "left_wrist": cams.left_camera, "right_wrist": cams.right_camera}

    def depth_m(camera) -> np.ndarray:
        """RPent's depth: -z of the camera-space 'Position' buffer (OpenGL camera frame), metres; NaN = no hit."""
        pos = camera.get_picture("Position")
        d = (-pos[..., 2]).astype(np.float32)
        d[pos[..., 3] >= 1] = np.nan
        return d

    def cam_meta(camera) -> dict:
        return {"intrinsic_K": camera.get_intrinsic_matrix(), "extrinsic_cv": camera.get_extrinsic_matrix(),
                "cam2world_gl": camera.get_model_matrix(), "width": camera.get_width(),
                "height": camera.get_height(), "fovy_deg": float(np.rad2deg(camera.fovy)),
                "near": camera.get_near(), "far": camera.get_far(),
                "world_pose_p_q(wxyz)": [camera.get_global_pose().p, camera.get_global_pose().q],
                "picture_names": camera.get_picture_names()}

    def project(pw, K, E):
        pc = E[:3, :3] @ pw + E[:3, 3]                   # world -> OpenCV camera (x right, y down, z fwd)
        return K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2], pc[2]

    def backproject(u, v, d, K, M):
        """RPent toolkit formula: pixel (u=col, v=row) + depth (m) -> world, via cam2world_gl (OpenGL)."""
        pc = np.array([(u - K[0, 2]) * d / K[0, 0], -(v - K[1, 2]) * d / K[1, 1], -d])
        return M[:3, :3] @ pc + M[:3, 3]

    def arm_state(task) -> dict:
        r = task.robot
        out = {}
        for arm in ("left", "right"):
            js = getattr(r, f"get_{arm}_arm_jointState")()          # 6 joint drive targets + commanded gripper
            real = getattr(r, f"get_{arm}_arm_real_jointState")()   # 6 measured joint positions + commanded gripper
            ent = getattr(r, f"{arm}_entity")
            gj = getattr(r, f"{arm}_gripper")[0][0]                 # base finger joint (fl/fr_joint7)
            scale = getattr(r, f"{arm}_gripper_scale")
            q_f = float(ent.get_qpos()[ent.get_active_joints().index(gj)])
            out[arm] = {
                "joint_targets": js[:6], "joint_qpos": real[:6], "gripper_cmd": js[6],
                "gripper_measured": (q_f - scale[0]) / (scale[1] - scale[0]),
                "ee_pose": getattr(r, f"get_{arm}_ee_pose")(), "tcp_pose": getattr(r, f"get_{arm}_tcp_pose")(),
            }
        return out

    class StepCounter:
        """Counts physics steps by wrapping scene.step (sapien.Scene is a python subclass, so this works)."""

        def __init__(self, task):
            self.n = 0
            self.scene = task.scene
            orig = task.scene.step

            def step():
                self.n += 1
                orig()
            task.scene.step = step

    def save_views(task, tag: str) -> dict:
        import cv2
        obs = task.get_obs()
        info = {}
        for name, cam in cameras(task).items():
            rgb = obs["observation"][{"head": "head_camera", "left_wrist": "left_camera",
                                      "right_wrist": "right_camera"}[name]]["rgb"]
            d = depth_m(cam)
            cv2.imwrite(f"{tag}_{name}_rgb.png", rgb[..., ::-1])
            dn = np.nan_to_num((d - np.nanmin(d)) / max(np.nanmax(d) - np.nanmin(d), 1e-6), nan=1.0)
            cv2.imwrite(f"{tag}_{name}_depth.png", (dn * 255).astype(np.uint8))
            info[name] = {"rgb_shape": list(rgb.shape), "rgb_dtype": str(rgb.dtype),
                          "depth_shape": list(d.shape), "depth_dtype": str(d.dtype),
                          "depth_min_max_m": [float(np.nanmin(d)), float(np.nanmax(d))],
                          "depth_nan_frac": float(np.isnan(d).mean())}
        return info

    def table_plane_check(task, cam, table_z) -> dict:
        """Project a grid of table-top points, read depth there, back-project, compare z with the table height."""
        K, E, M = cam.get_intrinsic_matrix(), cam.get_extrinsic_matrix(), cam.get_model_matrix()
        d = depth_m(cam)
        errs = []
        for x in np.linspace(-0.3, 0.3, 7):
            for y in np.linspace(-0.2, 0.15, 6):
                u, v, z = project(np.array([x, y, table_z]), K, E)
                ui, vi = int(round(u)), int(round(v))
                if z <= 0 or not (0 <= ui < d.shape[1] and 0 <= vi < d.shape[0]) or np.isnan(d[vi, ui]):
                    continue
                w = backproject(ui, vi, float(d[vi, ui]), K, M)
                errs.append(float(w[2] - table_z))
        errs = np.array(errs)
        if errs.size == 0:
            return {"n": 0}
        return {"n": int(errs.size), "median_abs_dz_m": float(np.median(np.abs(errs))),
                "frac_within_1cm": float(np.mean(np.abs(errs) < 0.01))}

    # ================================================================== 1. adjust_bottle via VectorEnv
    task_name = "adjust_bottle"
    seed, published = CASES[task_name]
    section(f"1. {task_name} seed {seed} via robotwin.envs.vector_env.VectorEnv (RLinf/RPent path)")
    stamp("constructing VectorEnv")
    t = Timer()
    venv = VectorEnv(task_config=native_task_config(task_name), n_envs=1, env_seeds=[seed])
    build_s = t()
    sub = venv.envs[0]

    def cuda_mem() -> dict:
        return {"torch_allocated_MiB": round(torch.cuda.memory_allocated() / 2**20, 1),
                "torch_reserved_MiB": round(torch.cuda.memory_reserved() / 2**20, 1),
                "torch_max_allocated_MiB": round(torch.cuda.max_memory_allocated() / 2**20, 1)}

    def reset_vectorenv(seed_: int) -> tuple:
        """VectorEnv.reset (the RLinf/RPent call). Its future has a hard 120 s timeout; on a busy shared GPU the
        reset can take longer and then keeps running in the pool thread -> wait for it on the SubEnv locks."""
        gc.collect()
        t_ = Timer()
        try:
            venv.reset(env_idx=[0], env_seeds=[seed_])
            return t_(), False
        except RuntimeError as exc:
            if not isinstance(exc.__context__, TimeoutError):
                raise
            with sub.global_lock:
                with sub.lock:
                    pass
            return t_(), True

    reset_log = []

    def reset_sync(seed_: int, collect: bool = True) -> float:
        """What VectorEnv.reset runs in its pool thread (SubEnv.reset), called directly: no 120 s timeout.
        collect=True first runs gc.collect(): the previous episode's cuRobo MotionGens sit in reference cycles,
        so without it a reset holds old + new planners (~2x torch memory) and can OOM on a shared GPU.
        (No torch.cuda.empty_cache(): the freed blocks stay in torch's cache and are reused by the new planners.)"""
        if collect:
            gc.collect()
        t_ = Timer()
        sub.reset(env_seed=seed_)
        dt = t_()
        reset_log.append({"seed": seed_, "gc_before": collect, "reset_s": dt, **cuda_mem()})
        return dt

    stamp("VectorEnv.reset")
    reset_s, reset_timed_out = reset_vectorenv(seed)
    task = sub.task
    print(f"VectorEnv construction (prewalk setup_demo + close) {build_s}s, reset {reset_s}s", flush=True)
    rep1 = {"construct_s": build_s, "reset_s": reset_s, "vectorenv_reset_hit_120s_timeout": reset_timed_out,
            "cuda_mem_after_reset": cuda_mem(), "actual_seed(task.ep_num)": task.ep_num,
            "task.step_lim": task.step_lim, "native_instruction": sub.instruction, "published_instruction": published,
            "episode_info": sub.episode_info_list[0], "gpu_mem_after_reset": gpu_mem_mib(os.getpid()),
            "torch_max_alloc_MiB": torch.cuda.max_memory_allocated() / 2**20}
    stamp("first get_obs (first ray-tracing render of this process)")
    t = Timer()
    vobs = venv.get_obs()[0]
    rep1["first_get_obs_s"] = t()
    stamp(f"first get_obs done in {rep1['first_get_obs_s']}s")
    rep1["vectorenv_obs_keys"] = {k: (list(v.shape) if hasattr(v, "shape") else type(v).__name__) for k, v in vobs.items()}
    t = Timer()
    obs = task.get_obs()
    rep1["get_obs_s"] = t()
    rep1["native_obs_keys"] = {k: (list(v.keys()) if isinstance(v, dict) else type(v).__name__) for k, v in obs.items()}
    rep1["native_obs_camera_keys"] = {k: list(v.keys()) for k, v in obs["observation"].items()}
    rep1["joint_action_vector"] = obs["joint_action"]["vector"]
    rep1["endpose"] = obs["endpose"]
    report["adjust_bottle_vectorenv"] = rep1
    print(json.dumps(jsonable(rep1), indent=1)[:4000], flush=True)

    # ------------------------------------------------------------------ 2. cameras
    section("2. cameras: rgb, depth (m), intrinsics/extrinsics, back-projection")
    cams = cameras(task)
    task.get_obs()
    t = Timer()
    for cam in cams.values():
        depth_m(cam)
    report["depth_fetch_3cams_s"] = t()
    report["camera_meta"] = {n: cam_meta(c) for n, c in cams.items()}
    report["views_adjust_bottle"] = save_views(task, "adjust_bottle")
    table_z = 0.74 + float(task.table_z_bias)
    report["table_top_z"] = table_z
    report["table_plane_check"] = {n: table_plane_check(task, c, table_z) for n, c in cams.items()}
    # bottle centre seen from the head camera: the depth hit is the bottle surface in front of the centre
    K, E, M = (cams["head"].get_intrinsic_matrix(), cams["head"].get_extrinsic_matrix(),
               cams["head"].get_model_matrix())
    bp = task.bottle.get_pose().p
    u, v, zc = project(bp, K, E)
    d = depth_m(cams["head"])
    ui, vi = int(round(u)), int(round(v))
    w = backproject(ui, vi, float(d[vi, ui]), K, M)
    report["bottle_backprojection"] = {"bottle_pose_p": bp, "pixel_uv": [ui, vi], "depth_at_pixel_m": float(d[vi, ui]),
                                       "camera_z_of_centre_m": float(zc), "backprojected_world": w,
                                       "dist_to_centre_m": float(np.linalg.norm(w - bp))}
    print(json.dumps(jsonable({k: report[k] for k in ("camera_meta", "views_adjust_bottle", "table_plane_check",
                                                      "bottle_backprojection")}), indent=1)[:6000], flush=True)

    # ------------------------------------------------------------------ 3. robot state
    section("3. robot state (world frame; quaternions are [w, x, y, z])")
    st0 = arm_state(task)
    report["robot_state_after_reset"] = st0
    axes = {}
    for arm in ("left", "right"):
        R = t3d.quaternions.quat2mat(st0[arm]["ee_pose"][3:])
        axes[arm] = {"ee_x_axis(approach)": R[:, 0], "ee_y_axis": R[:, 1], "ee_z_axis": R[:, 2],
                     "tcp_minus_ee": np.array(st0[arm]["tcp_pose"][:3]) - np.array(st0[arm]["ee_pose"][:3])}
    report["ee_axes_after_reset"] = axes
    report["top_down_quat_wxyz"] = GRASP_DIRECTION_DIC["top_down"]
    report["top_down_R"] = t3d.quaternions.quat2mat(GRASP_DIRECTION_DIC["top_down"])
    root = task.robot.left_entity.get_root_pose()
    report["robot_root_pose"] = [root.p, root.q]
    print(json.dumps(jsonable({k: report[k] for k in ("robot_state_after_reset", "ee_axes_after_reset",
                                                      "top_down_quat_wxyz", "top_down_R", "robot_root_pose")}),
                     indent=1), flush=True)

    # ------------------------------------------------------------------ 5a. success flags right after reset
    report["success_after_reset"] = {"check_success()": bool(task.check_success()),
                                     "eval_success": bool(getattr(task, "eval_success", False)),
                                     "take_action_cnt": task.take_action_cnt}

    # ------------------------------------------------------------------ 4. planning + execution
    section(f"4. {args.planner} plan + execute: left arm 7 cm up (25 qpos steps), back down (1 ee step), gripper")
    counter = StepCounter(task)
    r = task.robot
    ee0 = np.array(r.get_left_ee_pose())
    target = ee0.copy()
    target[2] += 0.07
    left_cam_before = cams["left_wrist"].get_model_matrix().copy()
    plan_times = []
    for _ in range(3):  # the first call may include lazy CUDA-graph / kernel warm-up
        t = Timer()
        res = r.left_plan_path(target.tolist())
        plan_times.append(t())
    rep4 = {"plan_status": res["status"], "plan_s": plan_times}
    if res["status"] == "Success":
        path = np.asarray(res["position"])
        rep4["plan_waypoints(1/250s)"] = int(path.shape[0])
        idx = np.linspace(0, path.shape[0] - 1, 25).astype(int)
        cnt0, n0 = task.take_action_cnt, counter.n
        t = Timer()
        per = []
        for i in idx:
            a = np.array(r.get_left_arm_jointState() + r.get_right_arm_jointState(), dtype=float)
            a[:6] = path[i]
            t1 = Timer()
            task.take_action(a, action_type="qpos")
            per.append(t1())
        rep4["qpos_exec"] = {"take_action_steps": task.take_action_cnt - cnt0, "physics_steps": counter.n - n0,
                             "wall_s": t(), "per_take_action_s": [min(per), float(np.median(per)), max(per)],
                             "ee_err_m": float(np.linalg.norm(np.array(r.get_left_ee_pose()[:3]) - target[:3]))}
        task.get_obs()
        left_cam_after = cams["left_wrist"].get_model_matrix()
        rep4["left_wrist_cam_moved_m"] = float(np.linalg.norm(left_cam_after[:3, 3] - left_cam_before[:3, 3]))
        # back down with ONE 16-d ee action: [L xyz wxyz, L grip, R xyz wxyz, R grip]
        a16 = np.concatenate([ee0, [r.get_left_gripper_val()], r.get_right_ee_pose(), [r.get_right_gripper_val()]])
        cnt0, n0 = task.take_action_cnt, counter.n
        t = Timer()
        task.take_action(a16, action_type="ee")
        rep4["ee_exec"] = {"take_action_steps": task.take_action_cnt - cnt0, "physics_steps": counter.n - n0,
                           "wall_s(incl. 2 plans)": t(),
                           "ee_err_m": float(np.linalg.norm(np.array(r.get_left_ee_pose()[:3]) - ee0[:3]))}
    # gripper: RPent set_gripper = 10 interpolated qpos actions
    grip = {}
    for name, goal in (("close", 0.0), ("open", 1.0)):
        cnt0, n0 = task.take_action_cnt, counter.n
        g0 = r.get_left_gripper_val()
        t = Timer()
        for g in np.linspace(g0, goal, 11)[1:]:
            a = np.array(r.get_left_arm_jointState() + r.get_right_arm_jointState(), dtype=float)
            a[6] = g
            task.take_action(a, action_type="qpos")
        s = arm_state(task)["left"]
        grip[name] = {"take_action_steps": task.take_action_cnt - cnt0, "physics_steps": counter.n - n0,
                      "wall_s": t(), "gripper_cmd": s["gripper_cmd"], "gripper_measured": s["gripper_measured"]}
    rep4["gripper"] = grip
    # raw per-physics-step cost
    t = Timer()
    for _ in range(100):
        task.scene.step()
    rep4["scene.step_ms"] = round(t() * 10, 3)
    t = Timer()
    for _ in range(100):
        task.scene.step()
        task._update_render()
        task.check_success()
    rep4["step+update_render+check_success_ms"] = round(t() * 10, 3)
    t = Timer()
    for _ in range(5):
        task.get_obs()
    rep4["get_obs_ms"] = round(t() * 200, 1)
    rep4["take_action_cnt_total"] = task.take_action_cnt
    rep4["gpu_mem_after_motion"] = gpu_mem_mib(os.getpid())
    report["motion"] = rep4
    print(json.dumps(jsonable(rep4), indent=1), flush=True)

    # ------------------------------------------------------------------ 6. reproducibility (VectorEnv resets)
    section("6. reset reproducibility / depth on-off / other seed (SubEnv.reset, synchronous)")
    reset_sync(seed)
    objs_a = scene_objects(sub.task)
    rgb_a = sub.task.get_obs()["observation"]["head_camera"]["rgb"].astype(np.int16)
    qpos_a = sub.task.robot.left_entity.get_qpos()
    instr_a = sub.instruction
    reset2_s = reset_sync(seed, collect=False)   # measure the plain SubEnv.reset memory behaviour once
    objs_b = scene_objects(sub.task)
    rgb_b = sub.task.get_obs()["observation"]["head_camera"]["rgb"].astype(np.int16)
    rep6 = {"reset_s": reset2_s, "same_seed": compare_objects(objs_a, objs_b),
            "same_seed_head_rgb_max_absdiff": int(np.abs(rgb_a - rgb_b).max()),
            "same_seed_head_rgb_mean_absdiff": float(np.abs(rgb_a - rgb_b).mean()),
            "same_seed_robot_qpos_maxdiff": float(np.abs(qpos_a - sub.task.robot.left_entity.get_qpos()).max()),
            "native_instruction_reset_a": instr_a, "native_instruction_reset_b": sub.instruction,
            "n_scene_actors": len(objs_a), "scene_actors_sample": objs_a[:40]}
    venv.args["data_type"]["depth"] = True          # SubEnv.args is the same dict
    reset_sync(seed, collect=True)
    objs_c = scene_objects(sub.task)
    obs_c = sub.task.get_obs()
    rgb_c = obs_c["observation"]["head_camera"]["rgb"].astype(np.int16)
    nd = obs_c["observation"]["head_camera"].get("depth")
    mine = depth_m(cameras(sub.task)["head"])
    rep6["depth_on"] = {"objects_vs_depth_off": compare_objects(objs_a, objs_c),
                        "head_rgb_max_absdiff_vs_depth_off": int(np.abs(rgb_a - rgb_c).max()),
                        "native_depth_dtype_unit": f"{nd.dtype} millimetres" if nd is not None else None,
                        "native_mm_vs_position_m_maxdiff_mm": (float(np.nanmax(np.abs(nd - np.nan_to_num(mine) * 1000)))
                                                               if nd is not None else None)}
    venv.args["data_type"]["depth"] = False
    reset_sync(OTHER_SEED, collect=True)
    rep6["other_seed"] = {"seed": OTHER_SEED, "actual_seed": sub.task.ep_num,
                          "vs_seed_a": compare_objects(objs_a, scene_objects(sub.task))}
    rep6["reset_log"] = reset_log
    report["reproducibility"] = rep6
    print(json.dumps(jsonable({k: v for k, v in rep6.items() if k != "scene_actors_sample"}), indent=1), flush=True)

    # ------------------------------------------------------------------ 5b. expert demo -> success
    if not args.skip_expert:
        section("5. expert demo (task.play_once) -> check_success")
        reset_sync(seed, collect=True)
        tk = sub.task
        t = Timer()
        try:
            tk.play_once()
            ok = {"plan_success": bool(tk.plan_success), "check_success()": bool(tk.check_success())}
        except Exception as exc:
            ok = {"error": repr(exc), "trace": traceback.format_exc()[-2000:]}
        ok["wall_s"] = t()
        ok["take_action_cnt(not used by play_once)"] = tk.take_action_cnt
        report["expert_demo"] = ok
        print(json.dumps(jsonable(ok), indent=1), flush=True)

    report["gpu_mem_peak_sample"] = gpu_mem_mib(os.getpid())
    report["cuda_mem_before_close"] = cuda_mem()
    report["reset_log"] = reset_log
    venv.close(clear_cache=True)

    # ================================================================== 1b. direct construction (no VectorEnv)
    def direct_task(name: str, seed_: int, base_args: dict):
        tk = getattr(importlib.import_module(f"robotwin.envs.{name}"), name)()
        a = copy.deepcopy(base_args)
        a["task_name"] = name
        a["save_path"] = f"./data/{name}_reward"
        a["instruction"] = CASES.get(name, (None, None))[1]
        gc.collect()
        t_ = Timer()
        tk.setup_demo(now_ep_num=seed_, seed=seed_, **a)
        tk.step_lim = STEP_LIM
        return tk, t_()

    section("1b. direct construction: adjust_bottle (compare with VectorEnv) + beat_block_hammer")
    base = copy.deepcopy(venv.args)   # exactly the args VectorEnv derived (embodiment files, cameras, eval_mode, ...)
    base["data_type"]["depth"] = False
    tk, s_ = direct_task("adjust_bottle", seed, base)
    report["direct_adjust_bottle"] = {"setup_demo_s": s_, "vs_vectorenv_same_seed": compare_objects(objs_a, scene_objects(tk))}
    rgb_d = tk.get_obs()["observation"]["head_camera"]["rgb"].astype(np.int16)
    report["direct_adjust_bottle"]["head_rgb_max_absdiff_vs_vectorenv"] = int(np.abs(rgb_a - rgb_d).max())
    tk.close_env(clear_cache=True)
    print(json.dumps(jsonable(report["direct_adjust_bottle"]), indent=1), flush=True)

    tk, s_ = direct_task("beat_block_hammer", CASES["beat_block_hammer"][0], base)
    rep7 = {"setup_demo_s": s_, "actual_seed": tk.ep_num, "check_success()": bool(tk.check_success())}
    rep7["views"] = save_views(tk, "beat_block_hammer")
    cams = cameras(tk)
    blk = tk.block.get_pose()
    top = blk.p + t3d.quaternions.quat2mat(blk.q) @ np.array([0, 0, 0.025])   # top-face centre (half size 2.5 cm)
    bp_checks = {}
    for name, cam in cams.items():
        K, E, M = cam.get_intrinsic_matrix(), cam.get_extrinsic_matrix(), cam.get_model_matrix()
        u, v, zc = project(top, K, E)
        ui, vi = int(round(u)), int(round(v))
        d = depth_m(cam)
        if zc <= 0 or not (0 <= ui < d.shape[1] and 0 <= vi < d.shape[0]) or np.isnan(d[vi, ui]):
            bp_checks[name] = {"visible": False, "uv": [u, v]}
            continue
        w = backproject(ui, vi, float(d[vi, ui]), K, M)
        bp_checks[name] = {"visible": True, "pixel_uv": [ui, vi], "depth_m": float(d[vi, ui]),
                           "expected_depth_m": float(zc), "backprojected": w, "block_top_centre": top,
                           "err_m": float(np.linalg.norm(w - top))}
    rep7["block_top_backprojection"] = bp_checks
    rep7["table_plane_check"] = {n: table_plane_check(tk, c, 0.74 + float(tk.table_z_bias)) for n, c in cams.items()}
    rep7["objects"] = {"hammer": [tk.hammer.get_pose().p, tk.hammer.get_pose().q], "block": [blk.p, blk.q],
                       "hammer_functional_point0": tk.hammer.get_functional_point(0, "pose").p,
                       "block_functional_point1": tk.block.get_functional_point(1, "pose").p}
    rep7["gpu_mem"] = gpu_mem_mib(os.getpid())
    report["beat_block_hammer_direct"] = rep7
    print(json.dumps(jsonable(rep7), indent=1), flush=True)
    tk.close_env(clear_cache=True)

    report["total_s"] = T()
    report["gpu_status_at_end(idx,free,used,util)"] = gpu_status()
    save_report(final=True)
    print(f"\nSMOKE DONE in {report['total_s']}s -> {out_json}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
