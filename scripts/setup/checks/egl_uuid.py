"""EGL devices' UUIDs (EGL_EXT_device_persistent_id) next to nvidia-smi's."""
import ctypes
import subprocess
import uuid

from mujoco.egl import egl_ext as EGL
from OpenGL import EGL as GL

query = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                         ctypes.POINTER(ctypes.c_int))(int(GL.eglGetProcAddress("eglQueryDeviceBinaryEXT")))
attrib = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_ssize_t))(
    int(GL.eglGetProcAddress("eglQueryDeviceAttribEXT")))
for i, device in enumerate(EGL.eglQueryDevicesEXT()):
    buf = (ctypes.c_ubyte * 16)()
    size = ctypes.c_int(0)
    ok = query(ctypes.cast(device, ctypes.c_void_p), 0x335C, 16, buf, ctypes.byref(size))
    v = ctypes.c_ssize_t(-1)
    attrib(ctypes.cast(device, ctypes.c_void_p), 0x323A, ctypes.byref(v))
    print(i, "cuda", v.value, "uuid", (str(uuid.UUID(bytes=bytes(buf))) if ok == 1 and size.value == 16 else f"none ok={ok} size={size.value}"))
print(subprocess.run(["nvidia-smi", "--query-gpu=index,uuid", "--format=csv,noheader"], capture_output=True, text=True).stdout)
