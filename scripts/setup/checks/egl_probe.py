"""List EGL devices: extensions, CUDA ordinal, DRM node (each query guarded)."""
import ctypes
import os

from mujoco.egl import egl_ext as EGL
from OpenGL import EGL as GL
from OpenGL.EGL.EXT.device_base import eglQueryDeviceStringEXT

attrib = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_ssize_t))(
    int(GL.eglGetProcAddress("eglQueryDeviceAttribEXT")))
for i, device in enumerate(EGL.eglQueryDevicesEXT()):
    try:
        ext = (eglQueryDeviceStringEXT(device, 0x3055) or b"").decode()
    except Exception as e:
        ext = f"ERR {e.__class__.__name__}"
    v = ctypes.c_ssize_t(-1)
    ok = attrib(ctypes.cast(device, ctypes.c_void_p), 0x323A, ctypes.byref(v))
    try:
        node = eglQueryDeviceStringEXT(device, 0x3233)
    except Exception as e:
        node = f"ERR {getattr(e, 'err', e)}"
    print(i, "cuda", (int(v.value) if ok == 1 else None), "drm", node, "|", ext)
print("dev/dri:", sorted(os.listdir("/dev/dri")) if os.path.isdir("/dev/dri") else "absent")
