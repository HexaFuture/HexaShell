"""Measuring known solids from exact outlines.

Outlines are rasterised the way the renderer samples pixels (a pixel belongs to the object when
its centre does), so what the tests see as error is the method's own, not a segmenter's.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fakes import Box, FakeTable
from hexashell.geometry import homogeneous
from hexashell.motion import ik
from hexashell.motion.chain import arm_chain, slide
from hexashell.motion.pose import rotation_of
from hexashell.state import parse_obs
from hexashell.vision import hull
from hexashell.vision.cameras import HEAD_FOCAL, HEAD_POSE, WRIST_FOCAL, Camera
from hexashell.vision.sam3_client import Segment
from hexashell.vision.scene import Scene
from hexashell.world import GRASP_REACH, TABLE_Z


# -- pictures ------------------------------------------------------------------------------------------
def outline(cam: Camera, points: np.ndarray) -> np.ndarray:
    """Pixels whose centres fall inside the convex hull of the points' images."""
    import cv2

    px, depth = cam.project(points)
    assert np.all(depth > 0.01)
    poly = cv2.convexHull(px.astype(np.float32)).reshape(-1, 2).astype(float)
    ys, xs = np.mgrid[0:cam.height, 0:cam.width]
    n = len(poly)
    turn = sum(poly[i, 0] * poly[(i + 1) % n, 1] - poly[(i + 1) % n, 0] * poly[i, 1] for i in range(n))
    inside = np.ones(xs.shape, bool)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        inside &= math.copysign(1.0, turn) * ((x1 - x0) * (ys - y0) - (y1 - y0) * (xs - x0)) >= 0
    return inside


def head() -> Camera:
    return Camera("head", HEAD_FOCAL, HEAD_POSE.copy())


def above(x: float, y: float, z: float, name: str = "left_wrist") -> Camera:
    """A wrist camera looking straight down from (x, y, z)."""
    pose = np.eye(4)
    pose[:3, 3] = (x, y, z)
    return Camera(name, WRIST_FOCAL, pose)


def aimed(target, dist: float, tilt_deg: float, heading_deg: float, name: str = "right_wrist") -> Camera:
    """A wrist camera ``dist`` from ``target``, looking at it ``tilt_deg`` off straight down."""
    t, a = math.radians(tilt_deg), math.radians(heading_deg)
    look = np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)])
    z = -look
    x = np.cross([0.0, 0.0, 1.0], z)
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2] = x, np.cross(z, x), z
    pose[:3, 3] = np.asarray(target, float) - dist * look
    return Camera(name, WRIST_FOCAL, pose)


def sights(cams, points) -> list[hull.Sight]:
    return [hull.Sight(c.name, c, outline(c, points)) for c in cams]


def on_table(xy, size, yaw=0.0, label="block", color=(200, 40, 40)) -> Box:
    return Box(label, (xy[0], xy[1], TABLE_Z + size[2] / 2), size, yaw, color)


BOXES = [
    on_table((0.05, 0.00), (0.06, 0.04, 0.05), 25.0),
    on_table((-0.10, 0.05), (0.04, 0.04, 0.04)),
    on_table((0.12, -0.08), (0.08, 0.03, 0.03), -60.0),
]


def errors(shape: hull.Shape, box: Box) -> dict:
    length, width = sorted(box.size[:2], reverse=True)
    heading = box.yaw_deg if box.size[0] >= box.size[1] else box.yaw_deg + 90.0
    out = {"xy": 1000 * math.hypot(shape.center[0] - box.center[0], shape.center[1] - box.center[1]),
           "top": 1000 * (shape.top - (TABLE_Z + box.size[2])),
           "L": 1000 * (shape.size[0] - length), "W": 1000 * (shape.size[1] - width)}
    if length - width > 0.005:
        out["yaw"] = (shape.yaw - heading + 90.0) % 180.0 - 90.0
    return out


def near(errs: dict, xy: float, size: float, top: float, yaw: float = 3.0) -> None:
    assert errs["xy"] <= xy, errs
    assert abs(errs["L"]) <= size and abs(errs["W"]) <= size, errs
    assert abs(errs["top"]) <= top, errs
    if "yaw" in errs:
        assert abs(errs["yaw"]) <= yaw, errs


# -- several cameras -------------------------------------------------------------------------------------
@pytest.mark.parametrize("box", BOXES, ids=["turned", "cube", "long"])
def test_head_and_camera_above(box):
    cams = [head(), above(box.center[0] + 0.04, box.center[1] - 0.03, TABLE_Z + 0.30)]
    shape = hull.shape(sights(cams, box.corners()))
    assert shape.kind == "stands" and not shape.round
    assert shape.footprint_from == "left_wrist"  # the fine view from above gives the footprint
    near(errors(shape, box), xy=1.0, size=1.5, top=2.0)


@pytest.mark.parametrize("box", BOXES, ids=["turned", "cube", "long"])
def test_head_and_slanted_camera(box):
    cams = [head(), aimed((box.center[0], box.center[1], TABLE_Z + 0.02), 0.30, 35.0, 30.0)]
    shape = hull.shape(sights(cams, box.corners()))
    assert shape.kind == "stands" and shape.footprint_from == ""
    near(errors(shape, box), xy=2.0, size=5.0, top=2.5)


@pytest.mark.parametrize("box", BOXES, ids=["turned", "cube", "long"])
def test_three_cameras(box):
    target = (box.center[0], box.center[1], TABLE_Z + 0.02)
    cams = [head(), above(box.center[0] + 0.04, box.center[1] - 0.03, TABLE_Z + 0.30), aimed(target, 0.30, 35.0, 30.0)]
    near(errors(hull.shape(sights(cams, box.corners())), box), xy=1.0, size=1.0, top=1.5)


def test_cameras_looking_down_only_flag_the_height():
    box = BOXES[0]
    cams = [above(box.center[0] + 0.03, box.center[1], TABLE_Z + 0.30),
            above(box.center[0] - 0.03, box.center[1] + 0.02, TABLE_Z + 0.28, "right_wrist")]
    shape = hull.shape(sights(cams, box.corners()))
    assert any("height is uncertain" in n for n in shape.notes)


def bowl(centre, rim: float, foot: float, tall: float) -> np.ndarray:
    """Points of a bowl's outline: a frustum, widest at the rim."""
    a = np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False)
    ring = np.stack([np.cos(a), np.sin(a)], axis=1)
    centre = np.asarray(centre, float)
    return np.concatenate([np.c_[centre + rim * ring, np.full(len(a), TABLE_Z + tall)],
                           np.c_[centre + foot * ring, np.full(len(a), TABLE_Z)]])


BOWLS = [((0.02, 0.03), 0.06, 0.03, 0.05), ((-0.10, 0.00), 0.05, 0.035, 0.04), ((0.0, -0.08), 0.075, 0.04, 0.06)]


@pytest.mark.parametrize("setup", ["from above", "from both sides"])
@pytest.mark.parametrize("centre,rim,foot,tall", BOWLS, ids=["middle", "small", "big near"])
def test_bowl_hangs_from_its_rim(setup, centre, rim, foot, tall):
    target = (centre[0], centre[1], TABLE_Z + 0.03)
    if setup == "from above":  # a wrist camera over it, beside the head
        cams = [head(), above(centre[0] + 0.03, centre[1] - 0.02, TABLE_Z + 0.35)]
    else:  # the two wrist cameras from the left and the right, as at the start
        cams = [head(), aimed(target, 0.40, 50.0, 0.0, "left_wrist"), aimed(target, 0.40, 50.0, 180.0)]
    shape = hull.shape(sights(cams, bowl(centre, rim, foot, tall)))
    assert shape.kind == "hangs" and shape.round, (shape.kind, shape.notes)
    assert math.hypot(shape.center[0] - centre[0], shape.center[1] - centre[1]) <= 0.003
    assert abs(shape.top - (TABLE_Z + tall)) <= 0.004
    assert abs(shape.size[0] - 2 * rim) <= 0.004


# -- one camera -----------------------------------------------------------------------------------------
def test_one_camera_with_known_height():
    box = BOXES[0]
    shape = hull.single(sights([head()], box.corners())[0], height=box.size[2])
    assert shape.single and not shape.guessed
    errs = errors(shape, box)
    assert errs["xy"] <= 2.0 and abs(errs["top"]) <= 0.5, errs
    assert 0.0 <= errs["L"] <= 6.0 and 0.0 <= errs["W"] <= 6.0, errs  # cones overlap: never smaller


def test_one_camera_guess_is_flagged():
    box = BOXES[1]
    shape = hull.single(sights([head()], box.corners())[0])
    assert shape.ok and shape.guessed
    assert any("guess" in n for n in shape.notes)
    assert math.hypot(shape.center[0] - box.center[0], shape.center[1] - box.center[1]) <= 0.01


# -- the scene, with a stand-in for SAM3 ------------------------------------------------------------------
class OutlinesByColour:
    """SAM3 stand-in: every box of the table is an object, outlined by its colour in the picture.
    A point prompt gets the box under the point; a text prompt the boxes whose label shares a word,
    scored by the share of the prompt's words in the label."""

    def __init__(self, table: FakeTable):
        self.table = table
        self.calls = []

    def health(self) -> dict:
        return {"ok": True}

    def close(self) -> None:
        pass

    def segment(self, image, *, text=None, points=None, labels=None, box=None, max_masks=8, min_score=0.0,
                multimask=None):
        self.calls.append({"text": text, "points": points})
        out = []
        for b in self.table.boxes:
            m = np.all(image == np.array(b.color, np.uint8), axis=-1)
            if m.sum() < 30:
                continue
            if points is not None:
                x, y = points[0]
                xi, yi = int(round(x)), int(round(y))
                if not (0 <= xi < m.shape[1] and 0 <= yi < m.shape[0] and m[yi, xi]):
                    continue
            score = 0.9
            if points is None and text is not None:
                words = set(text.lower().split())
                score *= len(words & set(b.label.split())) / max(len(words), 1)
                if not score:
                    continue
            ys, xs = np.nonzero(m)
            out.append(Segment(mask=m, score=score, box=(int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())),
                               area=int(m.sum())))
        return out


class _Host:
    def __init__(self, table: FakeTable):
        self.snap = parse_obs(table.obs(), 1)
        self.driver = None


def _hover_q(arm: str, xy) -> np.ndarray:
    """Joint angles that put the gripper 25 cm over the point, pointing down."""
    chain = arm_chain(arm)
    sol, _ = ik.solve(chain, homogeneous(rotation_of(90.0, 0.0, 0.0), (xy[0], xy[1], TABLE_Z + 0.25)), np.zeros(6))
    return sol.q


@pytest.fixture
def scene_and_table():
    red = on_table((0.05, 0.02), (0.05, 0.035, 0.05), 20.0, "red block", (200, 40, 40))
    blue = on_table((-0.12, 0.08), (0.04, 0.04, 0.04), 0.0, "blue block", (40, 60, 200))
    table = FakeTable([red, blue], start_q={"left": _hover_q("left", red.center[:2]), "right": np.zeros(6)})
    scene = Scene("http://fake", client=OutlinesByColour(table))
    scene.bind(_Host(table))
    yield scene, table
    scene.close()


def test_scene_measures_a_clicked_box_and_finds_it_again(scene_and_table):
    scene, table = scene_and_table
    red = table.boxes[0]
    view = scene.view("head")
    x, y = view.camera.pixel(red.center)
    thing = scene.measure(("head", (x, y)))
    assert thing.found, thing
    assert "head" in thing.views and "left_wrist" in thing.views, thing.views
    assert math.hypot(thing.center[0] - red.center[0], thing.center[1] - red.center[1]) <= 0.002
    assert abs(thing.top - (TABLE_Z + red.size[2])) <= 0.003
    assert abs(thing.size[0] - 0.05) <= 0.003 and abs(thing.size[1] - 0.035) <= 0.003
    assert thing.on == "table"
    again = scene.measure(thing)
    assert again.found and again.id == thing.id
    assert math.hypot(again.center[0] - thing.center[0], again.center[1] - thing.center[1]) <= 0.001


def test_scene_measures_by_words(scene_and_table):
    scene, table = scene_and_table
    blue = table.boxes[1]
    thing = scene.measure("blue block")
    assert thing.found, thing
    assert math.hypot(thing.center[0] - blue.center[0], thing.center[1] - blue.center[1]) <= 0.004


class _Holding:
    """Driver stand-in: one arm holds something."""

    def __init__(self, arm: str):
        self.arm = arm

    def holding(self, arm: str) -> bool:
        return arm == self.arm

    def squeezing(self, arm: str) -> bool:
        return False


def test_what_a_gripper_holds_is_blind_in_its_own_wrist_camera(scene_and_table):
    # Say the left gripper's load hides the red block from the left wrist camera: that camera must
    # then neither count as seeing the block nor measure the load itself.
    scene, table = scene_and_table
    red = table.boxes[0]
    scene.host.driver = _Holding("left")
    scene._held_region = lambda arm, view: np.all(view.image == np.array(red.color, np.uint8), axis=-1)
    views = scene.views()
    assert views["head"].held is None and views["right_wrist"].held is None
    wrist = views["left_wrist"]
    assert wrist.held.any() and np.all(wrist.blind[wrist.held]) and np.all(wrist.blind[wrist.robot])
    x, y = views["head"].camera.pixel(red.center)
    thing = scene.measure(("head", (x, y)))
    assert thing.found and "left_wrist" not in thing.views, thing
    wx, wy = wrist.camera.pixel(red.center)
    there = scene.measure(("left_wrist", (wx, wy)))
    assert not there.found and "what the left gripper holds" in there.why_missing, there


def test_the_load_is_outlined_from_a_click_at_the_grasp_point():
    q = _hover_q("left", (0.05, 0.02))
    grasp = slide(arm_chain("left").flange(q), GRASP_REACH)[:3, 3]
    load = Box("green block", tuple(float(v) for v in grasp), (0.03, 0.03, 0.03), 0.0, (40, 200, 40))
    table = FakeTable([load], start_q={"left": q, "right": np.zeros(6)})
    scene = Scene("http://fake", client=OutlinesByColour(table))
    host = _Host(table)
    host.driver = _Holding("left")
    scene.bind(host)
    try:
        view = scene.views()["left_wrist"]
        shown = np.all(view.image == np.array(load.color, np.uint8), axis=-1)
        assert shown.sum() > 1000
        held = view.held
        assert held[shown].all() and held.sum() < 1.2 * shown.sum()
    finally:
        scene.close()


class TableToo(OutlinesByColour):
    """... and the words "floor" outline every pixel that shows no box."""

    def segment(self, image, *, text=None, **kw):
        if text != "floor":
            return super().segment(image, text=text, **kw)
        self.calls.append({"text": text, "points": None})
        boxes = np.zeros(image.shape[:2], bool)
        for b in self.table.boxes:
            boxes |= np.all(image == np.array(b.color, np.uint8), axis=-1)
        return [Segment(mask=~boxes, score=0.8, box=(0, 0, image.shape[1] - 1, image.shape[0] - 1),
                        area=int((~boxes).sum()))]


def test_to_world_takes_bare_table_as_the_table_top(scene_and_table):
    scene, table = scene_and_table
    scene.sam = TableToo(table)
    red = table.boxes[0]
    head = scene.view("head").camera
    floor_point = np.array([0.20, 0.15, TABLE_Z])
    spot = scene.surface_spot("head", head.pixel(floor_point))
    assert "bare table" in spot.source and np.linalg.norm(np.array(spot.xyz) - floor_point) < 0.002, spot
    top = np.array([red.center[0], red.center[1], TABLE_Z + red.size[2]])
    on_red = scene.surface_spot("head", head.pixel(top))
    assert "bare table" not in on_red.source and abs(on_red.xyz[2] - top[2]) < 0.006, on_red


def test_to_world_flags_a_pixel_on_the_robot_in_the_picture_it_was_picked_from(scene_and_table):
    # The left arm hangs over the red block, its hand in the head picture.  A pixel on the hand is
    # flagged, one on the blue block is not, and what counts is the picture the pixel was picked from
    # (the host's `shown`), not the current one.
    from types import SimpleNamespace

    from hexashell.vision.spots import to_world

    scene, table = scene_and_table
    now = scene.host.snap
    head = scene.view("head")
    hand = now.arms["left"].flange[:3, 3]
    on_hand = head.camera.pixel(hand)
    assert head.robot[int(round(on_hand[1])), int(round(on_hand[0]))]
    blue = table.boxes[1]
    on_blue = head.camera.pixel(np.array(blue.center) + [0.0, 0.0, blue.size[2] / 2])
    host = SimpleNamespace(snap=now, scene=None)
    flagged = to_world(host, "head", on_hand, z=hand[2])
    assert np.linalg.norm(np.array(flagged.xyz) - hand) < 0.002, flagged
    assert "robot's own arm or gripper in the head picture" in flagged.warning, flagged
    assert to_world(host, "head", on_blue, z=TABLE_Z + blue.size[2]).warning is None
    paired = to_world(host, "left_wrist", (320, 240), pair=("head", on_hand))
    assert "robot's own arm or gripper in the head picture" in paired.warning, paired
    # the same pixel picked from a picture with the arm still down is fine, though the hand is there now
    table.q["left"] = np.zeros(6)
    earlier = parse_obs(table.obs(), 0)
    assert to_world(SimpleNamespace(snap=now, scene=None, shown={"head": earlier}), "head", on_hand,
                    z=hand[2]).warning is None
    # ... and picked from the picture with the hand there, it is flagged though the hand has gone since
    gone = to_world(SimpleNamespace(snap=earlier, scene=None, shown={"head": now}), "head", on_hand, z=hand[2])
    assert "robot's own arm or gripper" in gone.warning, gone


class _Bowl(Box):
    """A bowl for FakeTable: a frustum, widest at the rim (drawn as its convex outline)."""

    def __init__(self, centre, rim, foot, tall, color=(200, 40, 40)):
        super().__init__("red bowl", (centre[0], centre[1], TABLE_Z + tall / 2), (2 * rim, 2 * rim, tall), 0.0, color)
        self.points = bowl(centre, rim, foot, tall)

    def corners(self) -> np.ndarray:
        return self.points


def test_a_thing_widest_at_the_top_says_how_its_top_errs():
    centre, rim, foot, tall = BOWLS[0]
    table = FakeTable([_Bowl(centre, rim, foot, tall)],
                      start_q={"left": _hover_q("left", centre), "right": np.zeros(6)})
    scene = Scene("http://fake", client=OutlinesByColour(table))
    scene.bind(_Host(table))
    try:
        camera = scene.view("head").camera
        thing = scene.measure(("head", camera.pixel((centre[0], centre[1], TABLE_Z + tall))))
        assert thing.found and abs(thing.top - (TABLE_Z + tall)) <= 0.005, thing
        note = [n for n in thing.notes if n.startswith("widest at the top")]
        assert len(note) == 1 and "a tall can or bottle" in note[0] and "the lowest top is closest" in note[0], thing
    finally:
        scene.close()


def test_the_log_keeps_the_best_weak_outlines_of_an_empty_find(scene_and_table):
    scene, table = scene_and_table
    events = []
    scene.log = events.append
    assert not scene.find("head", "red round shiny ball")  # one word of four: scored 0.225, under the cut
    sam = [e for e in events if e["kind"] == "sam3"]
    assert len(sam) == 1 and sam[0]["found"] == 1, sam
    (score, box, area), = sam[0]["top"]
    assert score == pytest.approx(0.225) and area > 0
    x, y = scene.view("head").camera.pixel(table.boxes[0].center)
    assert box[0] <= x <= box[2] and box[1] <= y <= box[3]


def _flared_bowl(rng, centre, z0, r_top=0.13, r_bot=0.06, tall=0.07, wall=0.006, seen_from=(-1.0, 0.0)):
    """Depth points of a bowl on a counter as cameras on the ``seen_from`` side see it: the outer face of the
    near half, the inner face of the far half, the rim's top, the inside floor; and the counter around it."""
    t, s = rng.uniform(0, 2 * np.pi, 6000), rng.uniform(0, 1, 6000)
    d = np.c_[np.cos(t), np.sin(t)]
    r = r_bot + (r_top - r_bot) * s + np.where(d @ np.asarray(seen_from) > 0, wall, 0.0)
    faces = np.c_[centre[0] + d[:, 0] * r, centre[1] + d[:, 1] * r, z0 + tall * s]
    t, rr = rng.uniform(0, 2 * np.pi, 800), rng.uniform(r_top, r_top + wall, 800)
    top = np.c_[centre[0] + rr * np.cos(t), centre[1] + rr * np.sin(t), np.full(800, z0 + tall)]
    t, rr = rng.uniform(0, 2 * np.pi, 1500), r_bot * np.sqrt(rng.uniform(0, 1, 1500))
    floor = np.c_[centre[0] + rr * np.cos(t), centre[1] + rr * np.sin(t), np.full(1500, z0 + 0.008)]
    counter = np.c_[rng.uniform(-0.4, 0.4, (20000, 2)) + centre, np.full(20000, z0)]
    counter = counter[np.hypot(counter[:, 0] - centre[0], counter[:, 1] - centre[1]) > r_top + wall + 0.005]
    return np.vstack([faces, top, floor, counter])


def test_rim_of_a_bowl_from_its_depth_points():
    from hexashell.vision.rim import rim_of

    rng = np.random.default_rng(0)
    z0, tall, outer = 0.92, 0.07, 0.136
    pts = _flared_bowl(rng, (1.0, 0.0), z0, tall=tall)
    beside = np.c_[rng.uniform(1.0, 1.2, 3000), rng.uniform(0.17, 0.35, 3000), rng.uniform(z0, z0 + 0.35, 3000)]  # a box
    pts = np.vstack([pts, beside]) + rng.normal(0, 0.001, (len(pts) + len(beside), 3))
    # from the middle at its top, a point off the middle, a point on the near rim; the hand's side ahead of it
    for p in [(1.0, 0.0, z0 + tall), (1.03, 0.03, z0 + 0.05), (1.0 - outer + 0.004, 0.0, z0 + tall)]:
        rim, why = rim_of(pts, p, (0.4, 0.0))
        assert rim is not None, why
        assert np.allclose(rim.center, (1.0, 0.0), atol=0.004) and abs(rim.radius - outer) < 0.008, rim
        assert abs(rim.top - (z0 + tall)) < 0.004 and abs(rim.on_z - z0) < 0.002, rim
        assert abs(rim.edge[0] - (1.0 - outer)) < 0.004 and abs(rim.edge[1]) < 0.004, rim
        assert np.allclose(rim.toward, (1.0, 0.0), atol=0.01), rim
    # the hand coming from its right: the near side is the right one
    rim, _ = rim_of(pts, (1.0, 0.0, z0 + tall), (1.0, -0.6))
    assert abs(rim.edge[1] + outer) < 0.006 and np.allclose(rim.toward, (0.0, 1.0), atol=0.01), rim
    # the box alone is too tall; a flat plate too low; nothing there
    rim, why = rim_of(np.vstack([pts[pts[:, 2] < z0 + 0.003], beside]), (1.1, 0.26, z0 + 0.3), (0.4, 0.0))
    assert rim is None and why == "only something taller than a dish there", why
    flat = np.c_[rng.uniform(0.9, 1.1, 3000), rng.uniform(-0.1, 0.1, 3000), np.full(3000, z0 + 0.014)]
    rim, why = rim_of(np.vstack([pts[pts[:, 2] < z0 + 0.003], flat]), (1.0, 0.0, z0 + 0.014), (0.4, 0.0))
    assert rim is None and why.startswith("its near rim stands 1.") and why.endswith("cm over the surface: not a dish"), why
    rim, why = rim_of(np.zeros((0, 3)), (1.0, 0.0, 1.0), (0.4, 0.0))
    assert rim is None and why.startswith("no surface found under it"), why


def test_rim_of_a_bowl_with_something_joined_to_it():
    """A low tray touching the bowl, a second bowl touching it: their points join the bowl's part but fall off the
    circle of its outline, so neither the middle nor the near edge moves (taken as the part's extent, both did by
    6-8 cm)."""
    from hexashell.vision.rim import rim_of

    rng = np.random.default_rng(1)
    z0, outer = 0.92, 0.136
    bowl = _flared_bowl(rng, (1.0, 0.0), z0)
    tray = np.c_[rng.uniform(0.95, 1.15, 4000), rng.uniform(0.14, 0.29, 4000), np.full(4000, z0 + 0.03)]
    machine = np.c_[rng.uniform(1.12, 1.3, 4000), rng.uniform(0.14, 0.34, 4000), rng.uniform(z0, z0 + 0.35, 4000)]
    pts = np.vstack([bowl, tray, machine]) + rng.normal(0, 0.001, (len(bowl) + len(tray) + len(machine), 3))
    for p in [(1.0, 0.0, z0 + 0.07), (1.03, 0.04, z0 + 0.05), (1.0 - outer + 0.004, 0.0, z0 + 0.07)]:
        rim, why = rim_of(pts, p, (0.4, 0.0))
        assert rim is not None, why
        assert np.allclose(rim.center, (1.0, 0.0), atol=0.005) and abs(rim.radius - outer) < 0.005, rim
        assert abs(rim.edge[0] - (1.0 - outer)) < 0.004 and abs(rim.edge[1]) < 0.004, rim
    small = _flared_bowl(rng, (1.0, -0.235), z0, r_top=0.09, r_bot=0.04, tall=0.05)
    pts = np.vstack([bowl, small[small[:, 2] > z0 + 0.003]])
    rim, why = rim_of(pts, (1.0, 0.0, z0 + 0.07), (0.4, 0.0))
    assert np.allclose(rim.center, (1.0, 0.0), atol=0.005) and abs(rim.radius - outer) < 0.005, rim
    rim, why = rim_of(pts, (1.0, -0.235, z0 + 0.05), (0.4, -0.235))
    assert np.allclose(rim.center, (1.0, -0.235), atol=0.005) and abs(rim.radius - 0.096) < 0.005, rim
    assert abs(rim.edge[0] - 0.904) < 0.004 and abs(rim.top - (z0 + 0.05)) < 0.004, rim


def test_rim_of_a_stack_takes_the_outer_rim_when_asked():
    """A smaller bowl set in a larger one stands 2.5 cm up in it, its rim 2.5 cm higher and 2 cm further in than the
    larger one's: the highest near rim is the smaller bowl's (a pinch there lifts it alone); outer=True takes the
    larger one's.  Both ways the outline is the larger bowl's."""
    from hexashell.vision.rim import rim_of

    rng = np.random.default_rng(2)
    z0 = 0.92
    big = _flared_bowl(rng, (1.0, 0.0), z0, r_top=0.10, r_bot=0.05, tall=0.07)
    small = _flared_bowl(rng, (1.0, 0.0), z0 + 0.025, r_top=0.08, r_bot=0.04, tall=0.07)
    small = small[small[:, 2] > z0 + 0.028]  # without its counter
    pts = np.vstack([big, small]) + rng.normal(0, 0.001, (len(big) + len(small), 3))
    for p in [(1.0, 0.0, z0 + 0.095), (1.02, 0.02, z0 + 0.06)]:
        rim, why = rim_of(pts, p, (0.4, 0.0))
        assert rim is not None, why
        assert np.allclose(rim.center, (1.0, 0.0), atol=0.005) and abs(rim.radius - 0.106) < 0.006, rim
        assert abs(rim.top - (z0 + 0.095)) < 0.004 and abs(rim.edge[0] - (1.0 - 0.086)) < 0.004, rim
        rim, why = rim_of(pts, p, (0.4, 0.0), outer=True)
        assert rim is not None, why
        assert np.allclose(rim.center, (1.0, 0.0), atol=0.005) and abs(rim.radius - 0.106) < 0.006, rim
        assert abs(rim.top - (z0 + 0.07)) < 0.004 and abs(rim.edge[0] - (1.0 - 0.106)) < 0.004, rim
        assert abs(rim.on_z - z0) < 0.002, rim
    # one bowl: outer changes nothing
    one = big + rng.normal(0, 0.001, big.shape)
    a, _ = rim_of(one, (1.0, 0.0, z0 + 0.07), (0.4, 0.0))
    b, _ = rim_of(one, (1.0, 0.0, z0 + 0.07), (0.4, 0.0), outer=True)
    assert abs(a.top - b.top) < 0.002 and abs(a.edge[0] - b.edge[0]) < 0.002, (a, b)
