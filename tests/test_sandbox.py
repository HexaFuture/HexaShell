"""The cell sandbox holds: no files outside the workspace, no network, no processes, no signals.

Run: ``python tests/test_sandbox.py`` in the policy environment.
"""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# RoboDojo (where the evaluators live), whose files the cell must not read; checked when it is set
ROBODOJO = os.environ.get("ROBODOJO_ROOT", "")
sys.path.insert(0, str(ROOT))

from hexashell.kernel.host import Kernel  # noqa: E402

ESCAPES = {
    # each prints "BLOCKED" when the sandbox holds
    "tcp socket": "c = np.ctypeslib.ctypes; libc = c.CDLL(None, use_errno=True)\n"
                  "print('BLOCKED' if libc.socket(2, 1, 0) < 0 else 'OPEN')",
    "fork": "c = np.ctypeslib.ctypes; libc = c.CDLL(None)\npid = libc.fork()\n"
            "print('BLOCKED' if pid < 0 else ('OPEN' if pid > 0 else libc._exit(0)))",
    "exec": "c = np.ctypeslib.ctypes; libc = c.CDLL(None)\n"
            "print('BLOCKED' if libc.system(b'/bin/true') != 0 else 'OPEN')",
    "signal parent": "c = np.ctypeslib.ctypes; libc = c.CDLL(None)\n"
                     "print('BLOCKED' if libc.kill(libc.getppid(), 0) != 0 else 'OPEN')",
    "read outside": "c = np.ctypeslib.ctypes; libc = c.CDLL(None)\n"
                    f"print('BLOCKED' if libc.open(b'{ROOT}/NOTICE', 0) < 0 else 'OPEN')",
    "proc environ": "c = np.ctypeslib.ctypes; libc = c.CDLL(None)\n"
                    "print('BLOCKED' if libc.open(b'/proc/%d/environ' % libc.getppid(), 0) < 0 else 'OPEN')",
}
if ROBODOJO and Path(ROBODOJO, "README.md").is_file():
    ESCAPES["read evaluator"] = ("c = np.ctypeslib.ctypes; libc = c.CDLL(None)\n"
                                 f"print('BLOCKED' if libc.open(b'{ROBODOJO}/README.md', 0) < 0 else 'OPEN')")

WORKS = {
    "unix socket": "c = np.ctypeslib.ctypes; libc = c.CDLL(None)\nprint('OK' if libc.socket(1, 1, 0) >= 0 else 'NO')",
    "threads (BLAS)": "a = np.ones((800, 800)); print('OK' if float((a @ a)[0, 0]) == 800.0 else 'NO')",
    "threads (cv2)": "import cv2\nimg = np.zeros((2000, 2000, 3), np.uint8)\n"
                     "print('OK' if cv2.GaussianBlur(img, (31, 31), 0).shape == img.shape else 'NO')",
    "workspace file": "open('x.txt', 'w').write('hi'); print('OK' if open('x.txt').read() == 'hi' else 'NO')",
}


def test_sandbox_holds():
    work = Path(tempfile.mkdtemp(prefix="hexashell-sandbox-")) / "workspace"
    kernel = Kernel(work, lambda name, args, kwargs: None, cpu_s=20, idle_s=30)
    kernel.start()
    try:
        print("sandbox:", kernel.sandbox_note)
        bad = []
        for name, code in ESCAPES.items():
            r = kernel.run(code)
            text = (r.output + (r.error or "")).strip()
            print(f"  {name:16s} {text.splitlines()[-1] if text else '?'}")
            if "BLOCKED" not in text:
                bad.append(name)
        for name, code in WORKS.items():
            r = kernel.run(code)
            text = (r.output + (r.error or "")).strip()
            print(f"  {name:16s} {text.splitlines()[-1] if text else '?'}")
            if "OK" not in text:
                bad.append(name)
        assert not bad, bad
    finally:
        kernel.close()


if __name__ == "__main__":
    test_sandbox_holds()
    print("ok")
