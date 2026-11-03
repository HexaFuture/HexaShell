"""Messages between HexaShell and a simulator server: a JSON header followed by raw array bytes.

The two processes run different Python and numpy versions, so nothing is pickled.  A message is

    8-byte big-endian length of the header | header (UTF-8 JSON) | the blobs, back to back

Arrays (and bytes) in the payload are replaced in the header by ``{"__blob__": i}`` and listed
in ``header["blobs"]`` with their dtype and shape.  Tuples arrive as lists; numpy scalars as
Python numbers.  Only plain data crosses: either side can read what the other sends without
running any of its code.

Imports nothing but the standard library and numpy (the servers import it too).
"""

from __future__ import annotations

import json
import socket
import struct
from typing import Any

import numpy as np

#: The largest message either side accepts (bytes).
MAX_MESSAGE = 1 << 30
#: dtypes an array may have.
DTYPES = {"bool", "uint8", "int8", "uint16", "int16", "uint32", "int32", "int64", "float16", "float32",
          "float64"}


class WireError(RuntimeError):
    """A message that cannot be read, or a connection that broke."""


def encode(payload: Any) -> bytes:
    blobs: list[tuple[dict, bytes]] = []

    def plain(value):
        if isinstance(value, np.ndarray):
            arr = np.ascontiguousarray(value)
            if arr.dtype.name not in DTYPES:
                raise WireError(f"arrays of {arr.dtype} cannot be sent")
            blobs.append(({"dtype": arr.dtype.name, "shape": list(arr.shape)}, arr.tobytes()))
            return {"__blob__": len(blobs) - 1}
        if isinstance(value, (bytes, bytearray, memoryview)):
            blobs.append(({"dtype": "bytes"}, bytes(value)))
            return {"__blob__": len(blobs) - 1}
        if isinstance(value, dict):
            return {str(k): plain(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [plain(v) for v in value]
        if isinstance(value, (bool, np.bool_)):
            return bool(value)
        if isinstance(value, (int, np.integer)):
            return int(value)
        if isinstance(value, (float, np.floating)):
            return float(value)
        if value is None or isinstance(value, str):
            return value
        raise WireError(f"{type(value).__name__} cannot be sent")

    body = plain(payload)
    header = json.dumps({"body": body, "blobs": [meta | {"nbytes": len(data)} for meta, data in blobs]},
                        allow_nan=True, separators=(",", ":")).encode("utf-8")
    return b"".join([struct.pack(">Q", len(header)), header] + [data for _, data in blobs])


def decode(header: dict, blobs: list[bytes]) -> Any:
    metas = header.get("blobs") or []
    if len(metas) != len(blobs):
        raise WireError("blob count does not match the header")
    values = []
    for meta, data in zip(metas, blobs):
        kind = meta.get("dtype")
        if kind == "bytes":
            values.append(data)
            continue
        if kind not in DTYPES:
            raise WireError(f"unknown dtype {kind!r}")
        shape = tuple(int(n) for n in meta.get("shape") or ())
        arr = np.frombuffer(data, dtype=np.dtype(kind))
        if arr.size != int(np.prod(shape, dtype=np.int64)):
            raise WireError(f"blob of {arr.size} {kind} does not fill shape {shape}")
        values.append(arr.reshape(shape).copy())

    def back(value):
        if isinstance(value, dict):
            if set(value) == {"__blob__"}:
                index = value["__blob__"]
                if not isinstance(index, int) or not 0 <= index < len(values):
                    raise WireError(f"bad blob reference {index!r}")
                return values[index]
            return {k: back(v) for k, v in value.items()}
        if isinstance(value, list):
            return [back(v) for v in value]
        return value

    return back(header.get("body"))


def _read_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise WireError("the connection closed")
        got += k
    return bytes(buf)


def send(sock: socket.socket, payload: Any) -> None:
    data = encode(payload)
    if len(data) > MAX_MESSAGE:
        raise WireError(f"message of {len(data)} bytes is too large")
    try:
        sock.sendall(data)
    except OSError as exc:
        raise WireError(f"send failed: {exc}") from exc


def recv(sock: socket.socket) -> Any:
    try:
        (size,) = struct.unpack(">Q", _read_exact(sock, 8))
        if size > MAX_MESSAGE:
            raise WireError(f"header of {size} bytes is too large")
        header = json.loads(_read_exact(sock, size).decode("utf-8"))
        if not isinstance(header, dict):
            raise WireError("the header is not an object")
        metas = header.get("blobs") or []
        total = sum(int(m.get("nbytes", 0)) for m in metas)
        if total > MAX_MESSAGE:
            raise WireError(f"blobs of {total} bytes are too large")
        blobs = [_read_exact(sock, int(m.get("nbytes", 0))) for m in metas]
    except OSError as exc:
        raise WireError(f"receive failed: {exc}") from exc
    except (ValueError, struct.error) as exc:
        raise WireError(f"unreadable message: {exc}") from exc
    return decode(header, blobs)
