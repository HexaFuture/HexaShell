"""Build hexashell/assets/robot_model.npz: the X5 arm's shape for the robot mask.

    python bench/build_robot_points.py [--meshes DIR] [--cell 0.003] [--out PATH] [--no-check]

Reads the arm's visual meshes (binary STL for base_link and link1..link8, glTF binary for the
wrist camera mount and the camera) and writes, per part, vertices in the frame of the link the
part moves with, plus triangles.  The camera parts are stored in link6's frame, with the URDF's
fixed joints and visual origins applied.

Per part:
  * small pieces (screws, inner parts) are dropped when they never show more than DROP_TOL
    outside the rest of the part, seen from many directions;
  * the rest is simplified by vertex clustering on a grid (each cluster at the point that best
    fits the planes around it), keeping the surface closed; the fingers keep full detail, since
    their own wrist camera sees them from a few centimetres;
  * every triangle side gets the triangle on its other side (``twins``), which lets the runtime
    draw a part from its outline alone;
  * the camera's mesh is not closed, so it is stored as points on its convex hull instead
    (``kind`` 1), drawn as the hull of their projection.

The check at the end draws each part from many directions, full meshes against what is
stored, and prints how far the outlines differ.
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hexashell.geometry import homogeneous, rpy  # noqa: E402
from hexashell.vision.robotmask import raster_triangles  # noqa: E402

#: RoboDojo's ARX X5 meshes ($ROBODOJO_ROOT: the RoboDojo checkout)
MESHES = Path(os.environ.get("ROBODOJO_ROOT", "RoboDojo")) / "Assets" / "Robots" / "x5" / "meshes"
URDF = ROOT / "hexashell" / "assets" / "X5A.urdf"
OUT = ROOT / "hexashell" / "assets" / "robot_model.npz"

MESH, HULL = 0, 1
#: part -> (link it moves with, kind, grid cell in metres: None for --cell, 0 for full detail)
PARTS = {
    "base_link": ("base_link", MESH, None),
    "link1": ("link1", MESH, None),
    "link2": ("link2", MESH, None),
    "link3": ("link3", MESH, None),
    "link4": ("link4", MESH, None),
    "link5": ("link5", MESH, None),
    "link6": ("link6", MESH, None),
    "camera_base": ("link6", MESH, None),
    "camera": ("link6", HULL, None),
    "link7": ("link7", MESH, 0.0),
    "link8": ("link8", MESH, 0.0),
}
#: A piece may be dropped when it shows at most this far (m) outside the rest of its part.
DROP_TOL = 0.0005
#: Pieces at least this share of the part's biggest piece (by volume) are always kept.
CORE_SHARE = 0.05


# -- mesh files ------------------------------------------------------------------------------------
def read_stl(path: Path) -> np.ndarray:
    """(F, 3, 3) triangle corners of a binary STL."""
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        raise ValueError(f"{path}: ASCII STL is not supported")
    count = struct.unpack("<I", data[80:84])[0]
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rows = np.frombuffer(data, dtype=rec, count=count, offset=84)
    return rows["v"].astype(np.float64)


_GLTF_TYPES = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_GLTF_WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def _node_matrix(node: dict) -> np.ndarray:
    if "matrix" in node:
        return np.array(node["matrix"], float).reshape(4, 4).T  # column-major in glTF
    m = np.eye(4)
    if "scale" in node:
        m = np.diag(list(node["scale"]) + [1.0]) @ m
    if "rotation" in node:
        x, y, z, w = node["rotation"]
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        m = homogeneous(r, (0, 0, 0)) @ m
    if "translation" in node:
        m = homogeneous(np.eye(3), node["translation"]) @ m
    return m


def read_glb(path: Path) -> np.ndarray:
    """(F, 3, 3) triangle corners of every mesh in a glTF binary, node transforms applied."""
    data = path.read_bytes()
    magic, _version, _length = struct.unpack("<4sII", data[:12])
    if magic != b"glTF":
        raise ValueError(f"{path}: not a glTF binary")
    pos = 12
    doc, blob = None, b""
    while pos < len(data):
        size, kind = struct.unpack("<I4s", data[pos:pos + 8])
        chunk = data[pos + 8:pos + 8 + size]
        if kind == b"JSON":
            doc = json.loads(chunk)
        elif kind == b"BIN\x00":
            blob = chunk
        pos += 8 + size

    def accessor(i: int) -> np.ndarray:
        acc = doc["accessors"][i]
        view = doc["bufferViews"][acc["bufferView"]]
        dtype = np.dtype(_GLTF_TYPES[acc["componentType"]])
        width = _GLTF_WIDTH[acc["type"]]
        start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
        stride = view.get("byteStride", 0) or dtype.itemsize * width
        n = acc["count"]
        raw = np.frombuffer(blob, np.uint8, count=stride * (n - 1) + dtype.itemsize * width, offset=start)
        rows = np.lib.stride_tricks.as_strided(raw, shape=(n, dtype.itemsize * width), strides=(stride, 1))
        return np.ascontiguousarray(rows).view(dtype).reshape(n, width)

    tris = []

    def walk(index: int, parent: np.ndarray) -> None:
        node = doc["nodes"][index]
        m = parent @ _node_matrix(node)
        if "mesh" in node:
            for prim in doc["meshes"][node["mesh"]]["primitives"]:
                if prim.get("mode", 4) != 4:
                    continue
                v = accessor(prim["attributes"]["POSITION"]).astype(np.float64)
                v = v @ m[:3, :3].T + m[:3, 3]
                idx = accessor(prim["indices"]).reshape(-1) if "indices" in prim else np.arange(len(v))
                tris.append(v[idx.reshape(-1, 3)])
        for child in node.get("children", []):
            walk(child, m)

    scene = doc["scenes"][doc.get("scene", 0)]
    for root in scene["nodes"]:
        walk(root, np.eye(4))
    return np.concatenate(tris)


# -- URDF frames -----------------------------------------------------------------------------------
def urdf_frames(path: Path = URDF) -> tuple[dict, dict]:
    """(joints by child link: origin 4x4, axis, type, parent), (visual origin 4x4 and mesh file by link)."""
    root = ET.parse(path).getroot()

    def origin(el) -> np.ndarray:
        if el is None:
            return np.eye(4)
        xyz = [float(v) for v in el.get("xyz", "0 0 0").split()]
        angles = [float(v) for v in el.get("rpy", "0 0 0").split()]
        return homogeneous(rpy(*angles), xyz)

    joints = {}
    for el in root.findall("joint"):
        axis = el.find("axis")
        joints[el.find("child").get("link")] = {
            "parent": el.find("parent").get("link"),
            "type": el.get("type"),
            "origin": origin(el.find("origin")),
            "axis": np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else np.zeros(3),
        }
    visuals = {}
    for link in root.findall("link"):
        vis = link.find("visual")
        mesh = None if vis is None else vis.find("geometry/mesh")
        if mesh is not None:
            visuals[link.get("name")] = (origin(vis.find("origin")), Path(mesh.get("filename")).name)
    return joints, visuals


# -- mesh helpers ----------------------------------------------------------------------------------
def weld(tris: np.ndarray, tol: float = 1e-7) -> tuple[np.ndarray, np.ndarray]:
    """Shared vertices (V, 3) and faces (F, 3) of a triangle soup."""
    v = tris.reshape(-1, 3)
    key = np.round(v / tol).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return v[first], inv.reshape(-1, 3)


def clean(verts: np.ndarray, faces: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Drop triangles with a repeated corner and unused vertices.  Nothing else is dropped, so a
    closed surface stays closed (every side has a partner running the other way)."""
    f = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 2] != faces[:, 0])]
    used, inv = np.unique(f.reshape(-1), return_inverse=True)
    return verts[used], inv.reshape(-1, 3)


def twins(faces: np.ndarray) -> np.ndarray:
    """(F, 3): for side k of each triangle (corner k to k+1), the triangle holding the same side
    run the other way (-1: none).  Each side is paired with at most one other."""
    a = faces.reshape(-1).astype(np.int64)
    b = np.roll(faces, -1, axis=1).reshape(-1).astype(np.int64)
    n = int(faces.max()) + 1
    key = a * n + b
    order = np.argsort(key, kind="stable")
    sk = key[order]
    first = np.searchsorted(sk, sk, side="left")
    rank = np.empty(len(key), np.int64)
    rank[order] = np.arange(len(key)) - first  # place among sides with the same key
    want = b * n + a
    lo = np.searchsorted(sk, want, side="left")
    hi = np.searchsorted(sk, want, side="right")
    ok = rank < hi - lo
    partner = np.where(ok, order[np.minimum(lo + rank, len(key) - 1)], -1)
    return np.where(partner >= 0, partner // 3, -1).reshape(-1, 3).astype(np.int32)


def unpaired_sides(faces: np.ndarray) -> int:
    return int((twins(faces) < 0).sum()) if len(faces) else 0


def components(nv: int, faces: np.ndarray) -> np.ndarray:
    """Connected piece of every face (labels 0..)."""
    root = np.arange(nv)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]]])
    while True:
        a, b = root[edges[:, 0]], root[edges[:, 1]]
        lo, hi = np.minimum(a, b), np.maximum(a, b)
        joined = lo < hi
        if not joined.any():
            break
        # hang the larger root under the smaller one, then point everything at its root
        np.minimum.at(root, hi[joined], lo[joined])
        while True:
            up = root[root]
            if np.array_equal(up, root):
                break
            root = up
    return np.unique(root[faces[:, 0]], return_inverse=True)[1].reshape(-1)


def volume(verts: np.ndarray, faces: np.ndarray) -> float:
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def cluster(verts: np.ndarray, faces: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Vertex clustering on a grid of ``cell`` metres, each cluster at its best-fit point."""
    origin = verts.min(axis=0) - 1e-6
    keys = np.floor((verts - origin) / cell).astype(np.int64)
    _, group, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    group = group.reshape(-1)
    k = len(counts)
    # plane quadrics of the triangles around each vertex, summed per cluster (area weighted)
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    cross = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cross, axis=1)
    n = cross / np.maximum(2.0 * area, 1e-18)[:, None]
    d = -np.einsum("ij,ij->i", n, a)
    nn = n[:, :, None] * n[:, None, :] * area[:, None, None]
    nd = n * (d * area)[:, None]
    qa = np.zeros((k, 3, 3))
    qb = np.zeros((k, 3))
    for corner in range(3):
        g = group[faces[:, corner]]
        np.add.at(qa, g, nn)
        np.add.at(qb, g, nd)
    mean = np.zeros((k, 3))
    np.add.at(mean, group, verts)
    mean /= counts[:, None]
    # minimise x'Ax + 2b'x near the mean: truncated pseudo-inverse
    u, s, vt = np.linalg.svd(qa)
    smax = np.maximum(s[:, :1], 1e-30)
    inv_s = np.where(s > 0.05 * smax, 1.0 / np.maximum(s, 1e-30), 0.0)
    rhs = -qb - np.einsum("kij,kj->ki", qa, mean)
    step = np.einsum("kji,kj->ki", vt, inv_s * np.einsum("kji,kj->ki", u, rhs))
    best = mean + step
    # stay near the cluster's own cell
    lo = origin + (np.floor((mean - origin) / cell)) * cell - 0.5 * cell
    best = np.clip(best, lo, lo + 2.0 * cell)
    return clean(best, group[faces])


def directions(n: int) -> np.ndarray:
    """Roughly even unit directions (Fibonacci sphere)."""
    k = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * k / n)
    th = np.pi * (1 + 5 ** 0.5) * k
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], axis=1)


def hull_points(verts: np.ndarray, n: int = 2000) -> np.ndarray:
    """Vertices that are farthest out in at least one of n directions (on the convex hull)."""
    return verts[np.unique(np.argmax(verts @ directions(n).T, axis=0))]


# -- drawing from many directions --------------------------------------------------------------------
class Views:
    """Orthographic outlines of a reference mesh from ``n`` directions, ``px`` metres a pixel."""

    def __init__(self, verts: np.ndarray, faces: np.ndarray, n: int = 40, px: float = 0.0005):
        self.center = verts.mean(axis=0)
        self.px = px
        self.size = int(2 * np.max(np.linalg.norm(verts - self.center, axis=1)) / px) + 16
        self.rots = []
        for z in directions(n):
            a = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
            x = np.cross(a, z)
            x /= np.linalg.norm(x)
            self.rots.append(np.stack([x, np.cross(z, x), z]))
        self.sil = [self.draw(verts, faces, r) for r in self.rots]
        self.dist_out = [cv2.distanceTransform((~s).astype(np.uint8), cv2.DIST_L2, 5) for s in self.sil]

    def xy(self, pts: np.ndarray, rot: np.ndarray) -> np.ndarray:
        return ((pts - self.center) @ rot.T)[:, :2] / self.px + self.size / 2.0

    def draw(self, verts: np.ndarray, faces: np.ndarray | None, rot: np.ndarray) -> np.ndarray:
        xy = self.xy(verts, rot)
        if faces is None:  # convex hull of the points
            img = np.zeros((self.size, self.size), np.uint8)
            hull = cv2.convexHull(xy.astype(np.float32))
            cv2.fillConvexPoly(img, np.round(hull * 16).astype(np.int32), 1, cv2.LINE_8, 4)
            return img > 0
        return raster_triangles(xy[faces], self.size, self.size)

    def outside(self, verts: np.ndarray, faces: np.ndarray | None) -> float:
        """Farthest (m) the given shape shows outside the reference outline, over the views."""
        worst = 0.0
        for rot, dist in zip(self.rots, self.dist_out):
            m = self.draw(verts, faces, rot)
            if m.any():
                worst = max(worst, float(dist[m].max()))
        return worst * self.px

    def compare(self, verts: np.ndarray, faces: np.ndarray | None) -> tuple[float, float, float]:
        """(mean, worst) of the per-view largest outline offset, and the worst shown outside (m)."""
        per_view, outside = [], 0.0
        for rot, sil, dist in zip(self.rots, self.sil, self.dist_out):
            m = self.draw(verts, faces, rot)
            back = cv2.distanceTransform((~m).astype(np.uint8), cv2.DIST_L2, 5)
            out = float(dist[m].max()) if m.any() else 0.0
            lost = float(back[sil].max()) if sil.any() else 0.0
            per_view.append(max(out, lost))
            outside = max(outside, out)
        return float(np.mean(per_view)) * self.px, float(np.max(per_view)) * self.px, outside * self.px


# -- building --------------------------------------------------------------------------------------
def drop_hidden(verts: np.ndarray, faces: np.ndarray, views_n: int = 40) -> tuple[np.ndarray, np.ndarray, int]:
    """Drop pieces that never show more than DROP_TOL outside the part's big pieces."""
    label = components(len(verts), faces)
    count = label.max() + 1
    vols = np.array([abs(volume(verts, faces[label == k])) for k in range(count)])
    core = vols >= CORE_SHARE * vols.max()
    if core.all():
        return verts, faces, 0
    keep = core.copy()
    views = Views(verts, faces[core[label]], n=views_n)
    for k in np.flatnonzero(~core):
        piece = faces[label == k]
        if views.outside(verts, piece) > DROP_TOL:
            keep[k] = True
    v, f = clean(verts, faces[keep[label]])
    return v, f, int((~keep).sum())


def build(meshes: Path, cell: float, check: bool) -> dict:
    joints, visuals = urdf_frames()
    names, links, kinds = [], [], []
    all_v, all_f, all_t = [], [], []
    for part, (link, kind, part_cell) in PARTS.items():
        origin, filename = visuals[part]
        path = meshes / filename
        tris = read_stl(path) if path.suffix.lower() == ".stl" else read_glb(path)
        tris = tris @ origin[:3, :3].T + origin[:3, 3]
        # camera parts: carry them into link6's frame through their fixed joints
        frame = np.eye(4)
        name = part
        while name != link:
            j = joints[name]
            if j["type"] != "fixed":
                raise RuntimeError(f"{part}: moving joint between it and {link}")
            frame = j["origin"] @ frame
            name = j["parent"]
        tris = tris @ frame[:3, :3].T + frame[:3, 3]
        full = clean(*weld(tris))
        line = f"{part:12s} {len(full[1]):7d} triangles"
        if kind == HULL:
            v, f = hull_points(full[0]), np.zeros((0, 3), np.int64)
            line += f" -> {len(v):4d} hull points"
        else:
            v, f, dropped = drop_hidden(*full)
            size = cell if part_cell is None else part_cell
            if size > 0:
                v, f = cluster(v, f, size)
            line += f" -> {len(f):5d} ({dropped} small pieces dropped, {unpaired_sides(f)} open sides)"
        if check:
            views = Views(*full, n=40)
            mean, worst, outside = views.compare(v, None if kind == HULL else f)
            line += f"   outline offset mean {mean * 1e3:.2f} mm, worst {worst * 1e3:.2f} mm, outside {outside * 1e3:.2f} mm"
        print(line, flush=True)
        names.append(part)
        links.append(link)
        kinds.append(kind)
        all_v.append(v)
        all_f.append(f)
        all_t.append(twins(f) if len(f) else np.zeros((0, 3), np.int32))
    vc = np.array([len(v) for v in all_v])
    fc = np.array([len(f) for f in all_f])
    return {
        "part_names": np.array(names),
        "part_links": np.array(links),
        "part_kinds": np.array(kinds, np.int8),
        "vertex_start": (np.cumsum(vc) - vc).astype(np.int32),
        "vertex_count": vc.astype(np.int32),
        "face_start": (np.cumsum(fc) - fc).astype(np.int32),
        "face_count": fc.astype(np.int32),
        "vertices": np.concatenate(all_v).astype(np.float32),
        "faces": np.concatenate(all_f).astype(np.int32),
        "twins": np.concatenate(all_t).astype(np.int32),
        # the fingers: prismatic joints on link6
        "finger_links": np.array(["link7", "link8"]),
        "finger_origins": np.stack([joints[n]["origin"] for n in ("link7", "link8")]),
        "finger_axes": np.stack([joints[n]["axis"] for n in ("link7", "link8")]),
        "cell_m": np.array(cell),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--meshes", type=Path, default=MESHES)
    ap.add_argument("--cell", type=float, default=0.003, help="grid cell of the simplification (m)")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--no-check", action="store_true", help="skip the outline comparison")
    args = ap.parse_args()
    model = build(args.meshes, args.cell, not args.no_check)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, **model)
    print(f"{int(model['face_count'].sum())} triangles, {len(model['vertices'])} vertices -> {args.out} "
          f"({args.out.stat().st_size / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
