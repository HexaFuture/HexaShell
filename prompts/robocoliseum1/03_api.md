# 3. The `robot` API

Cameras are named 'head', 'left_wrist' and 'right_wrist' ('left' and 'right' work too). Arms are
'left' and 'right'.

Motion - uses steps; each call returns when the motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.goto(arm, xyz, pitch=, yaw=, roll=, grip=, path='auto')` | moves the grasp point to `xyz` with those angles (`path`: 'auto' a straight line, else a joint swing close to it; 'line'; 'swing' any curve); `grip` ('open', 'close' or 0..1) starts as the move starts: open on the approach with it, but let go of a held object with `gripper` once there | Move |
| `robot.goto(arm, by=(dx, dy, dz))` | moves relative to where it is (with `xyz` too: relative to `xyz`) | Move |
| `robot.goto('both', by=(dx, dy, dz))` | both hands together by the same offset, along straight lines, keeping their angles (a load held in both hands); nothing else is given with 'both' | [Move, Move] |
| `robot.goto(..., dry_run=True)` | only checks: reachable or not, path, steps; nothing moves | Move |
| `robot.goto(..., contact=True)` | for a press, push or pull meant to meet resistance: it is not stopped early and raises neither `Blocked` nor `Dropped` | Move |
| `robot.gripper(arm, opening)` | 'open', 'close' or 0..1 (1 = about 6.8 cm between the pads) | Grip |
| `robot.home(arm='both')` | lifts (if low) and swings back to the start pose; grippers unchanged; with 'both' the waist turns back to the start too | Homing |
| `robot.wait(n)` | holds everything still for n steps (n <= 100) | Waited |
| `robot.waist(yaw=)`, `robot.waist(by=)` | turns the torso to that heading (0 = the start's) or by that much, + to the left; both arms and the head camera turn with it, keeping their joints | text |

Seeing and measuring - no steps used (a measurement may take one held step for depth pictures):

| call | what it does | returns |
|---|---|---|
| `robot.look(cams='head', grid=, marks=, masks=, crop=, zoom=, tile=)` | attaches the current picture(s) to this call's result (see section 4) | Picture |
| `robot.find(cam, 'red cup')` | outlines every instance the words describe, best first | list of Mask |
| `robot.find(cam, at=(x, y))` | outlines the object under that pixel | list of Mask |
| `robot.find(cam, points=[(x, y), ...], labels=[1, 0, ...])`, `box=(x0, y0, x1, y1)` | outline from several points (1 = on the object, 0 = not on it) or a box | list of Mask |
| `robot.measure(target)` | 3-D position, top, size and grasp angle of one object from the depth pictures; `target` is words, a Mask, `(cam, (x, y))`, a list of Masks of the same object in different cameras, or a Thing measured before | Thing |
| `robot.to_world(cam, (x, y))` | the point seen at a pixel, from the depth picture; `z=` or `pair=(cam2, (x2, y2))` (section 4) | Spot |
| `robot.to_pixel(xyz, cam)` | where a world point appears in a camera's picture now, or None | (x, y) |

Reading - no steps used:

| call | what it gives |
|---|---|
| `robot.task` | the instruction and the start poses |
| `robot.state()` | both arms: grasp point, angles, gripper, holding, distance from the start pose. One arm: `robot.state().left` - `.at`, `.angles`, `.gripper`, `.holding` |
| `robot.image(cam)` | the current picture as an RGB numpy array ((400, 640, 3) for the head, (528, 640, 3) for a wrist) |
| `robot.camera(cam)` | a camera's focal length, centre and pose now |
| `robot.things` | every object measured so far (the latest measurement of each) |

Results print as one line or a few, with problems first. Their fields can be used in code:

```
Move left: at (0.550, 0.150, 0.820) pitch 90 yaw 0 roll 0  ±1 mm 0°  16 steps (line)
```
`.ok` (reached, not blocked, nothing slipped out), `.reached`, `.blocked`, `.lag_mm` (how far it
ended behind its command), `.dropped`, `.at`, `.angles`, `.steps`. A pose that cannot be reached
raises `NotExecuted` (`Move left: NOT executed` with the reason and `reachable instead: ...`):
nothing moved, and the rest of the cell does not run. A move that stalls against something, or ends
well behind its command, stops there and raises `Blocked` (`Move left: BLOCKED — at ...`): it is
touching something - an object, the table or the other arm; the rest of the cell does not run, so
you can look. What the gripper holds slipping out stops the motion and raises `Dropped`.
`short of target`: it ended a little off the target without being stopped (no error raised).

```
Grip right: closed on something ~3.0 cm wide (holding)  6 steps
```
`.holding` (True: the fingers stopped on something; False: closed fully - nothing between them, or
something very thin; None after an open), `.width_m` (the gap between the pads; on something held it
reads lower than the object's own size, by up to about 1.5 cm), `.opening`.

```
Home: left home, right home  23 steps
Waited 40 steps (1.33 s)
Waist: yaw 20° from the start's (+20°), 5 steps; the arms turned with it: left grasp point now (...); right grasp point now (...)
```

```
Thing#1 'red block'  center (0.752, -0.049, 0.758)  top 0.778  size 0.048×0.041×0.040  grip_yaw -84°  on z 0.738  3 views (head, left_wrist, right_wrist)
```
`.center` (x, y, z: the middle of what the cameras see of it), `.top` (its highest point), `.size`
(length, width, height), `.grip_yaw` (the yaw that closes down-pointing fingers across its narrow
side - for a square block, across a pair of its faces, and a note says so; None when round: any
yaw), `.on_z` (the height of its lowest visible point - about the surface
it stands on), `.views`, `.masks`. A warning comes first when there is one, notes follow. When the
object is not found it prints `NOT FOUND` with the reason, and `.center` raises NotFound.

```
Mask#7 head 'red block' score 0.91  centroid (412, 233)  box (391, 210)-(433, 258)  1520 px
Spot (0.604, 0.031, 0.731) from head (320, 300) on the depth picture  (~2.1 mm per pixel)
```

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotExecuted` (a goto that cannot be done; nothing moved), `Blocked`
and `Dropped` (a goto that ran and was stopped; the message is the Move), `NotFound`, `RobotError`
(`NotExecuted`, `Blocked` and `Dropped` are ones); the names are defined in the session. Catch them
in code when you have a plan B, or check a doubtful pose first with `dry_run=True`, which reports
instead of raising.
