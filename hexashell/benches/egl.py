"""Which EGL device a MuJoCo simulator renders on.

MuJoCo (through robosuite) takes ``MUJOCO_EGL_DEVICE_ID`` as a position in EGL's own device list,
which need not follow nvidia-smi's numbering, and Mesa may list a card a second time.  ``pin(gpu)``
takes an nvidia-smi index and points MuJoCo at the NVIDIA EGL device of the same GPU: the same UUID,
else the same PCI bus (the DRM node, which a user outside the video group may not open), else the
same CUDA ordinal.  Runs in a benchmark's own environment (needs ``mujoco`` and PyOpenGL), before the
simulator starts.
"""

from __future__ import annotations

import ctypes
import os
import subprocess
import uuid

EGL_EXTENSIONS = 0x3055
EGL_DRM_DEVICE_FILE_EXT = 0x3233
EGL_CUDA_DEVICE_NV = 0x323A
EGL_DEVICE_UUID_EXT = 0x335C


def _bus(text: str) -> str:
    """'00000000:61:00.0' -> '0000:61:00.0', as sysfs spells it."""
    domain, rest = text.strip().lower().split(":", 1)
    return f"{int(domain, 16):04x}:{rest}"


def _uuid(text: str) -> str:
    """'GPU-fbf5c7ef-2c82-...' -> 'fbf5c7ef-2c82-...'."""
    text = text.strip().lower()
    return text[4:] if text.startswith("gpu-") else text


def gpus() -> dict[int, dict]:
    """nvidia-smi's GPU index -> its PCI bus and UUID (one call: nvidia-smi can take seconds to answer)."""
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=index,pci.bus_id,uuid", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=120, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return {}
    found = {}
    for line in out.strip().splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) == 3 and parts[0].isdigit():
            found[int(parts[0])] = {"bus": _bus(parts[1]) if parts[1] else None,
                                    "uuid": _uuid(parts[2]) if parts[2] else None}
    return found


def devices() -> list[dict]:
    """EGL's devices in its order: whether each is NVIDIA's, its CUDA ordinal, UUID and PCI bus."""
    from mujoco.egl import egl_ext as EGL
    from OpenGL import EGL as GL
    from OpenGL.EGL.EXT.device_base import eglQueryDeviceStringEXT

    def proc(name: str, *args):
        address = GL.eglGetProcAddress(name)
        return ctypes.CFUNCTYPE(ctypes.c_uint, *args)(int(address)) if address else None

    attrib = proc("eglQueryDeviceAttribEXT", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_ssize_t))
    binary = proc("eglQueryDeviceBinaryEXT", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                  ctypes.POINTER(ctypes.c_int))
    rows = []
    for device in EGL.eglQueryDevicesEXT():
        handle = ctypes.cast(device, ctypes.c_void_p)
        try:
            extensions = set((eglQueryDeviceStringEXT(device, EGL_EXTENSIONS) or b"").decode(errors="replace").split())
        except Exception:  # noqa: BLE001 - PyOpenGL raises EGLError; the device stays unknown
            extensions = set()
        row = {"nvidia": "EGL_NV_device_cuda" in extensions, "cuda": None, "uuid": None, "bus": None}
        if row["nvidia"] and attrib is not None:
            value = ctypes.c_ssize_t(-1)
            if attrib(handle, EGL_CUDA_DEVICE_NV, ctypes.byref(value)) == 1:
                row["cuda"] = int(value.value) if value.value >= 0 else None
        if "EGL_EXT_device_persistent_id" in extensions and binary is not None:
            raw, size = (ctypes.c_ubyte * 16)(), ctypes.c_int(0)
            if binary(handle, EGL_DEVICE_UUID_EXT, 16, raw, ctypes.byref(size)) == 1 and size.value == 16:
                row["uuid"] = str(uuid.UUID(bytes=bytes(raw)))
        if "EGL_EXT_device_drm" in extensions:
            try:
                node = eglQueryDeviceStringEXT(device, EGL_DRM_DEVICE_FILE_EXT)
            except Exception:  # noqa: BLE001 - EGL_BAD_PARAMETER when this user may not open the node
                node = None
            if node:
                node = node.decode() if isinstance(node, bytes) else str(node)
                row["bus"] = os.path.basename(os.path.realpath(f"/sys/class/drm/{os.path.basename(node)}/device"))
        rows.append(row)
    return rows


def egl_index(gpu: int, rows: list[dict] | None = None, smi: dict[int, dict] | None = None) -> int:
    """EGL's index of nvidia-smi's GPU ``gpu``: by UUID, else PCI bus, else CUDA ordinal, else ``gpu``."""
    rows = devices() if rows is None else rows
    want = (gpus() if smi is None else smi).get(int(gpu), {})
    for key in ("uuid", "bus"):
        if want.get(key):
            for i, row in enumerate(rows):
                if row["nvidia"] and row.get(key) == want[key]:
                    return i
    for i, row in enumerate(rows):
        if row["nvidia"] and row["cuda"] == gpu:
            return i
    return gpu


def pin(gpu: int | str | None) -> int | None:
    """Render on nvidia-smi's GPU ``gpu`` (nothing changes for None or '')."""
    if gpu in (None, ""):
        return None
    index = egl_index(int(gpu))
    os.environ.pop("CUDA_VISIBLE_DEVICES", None)  # robosuite wants MUJOCO_EGL_DEVICE_ID listed in it when set
    os.environ["MUJOCO_EGL_DEVICE_ID"] = str(index)
    return index
