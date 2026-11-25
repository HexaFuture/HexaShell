# 3. The `robot` API

Cameras are named 'head', 'left_wrist' and 'right_wrist' ('left' and 'right' work too). Arms are
'left' and 'right'.

Motion - each call is one motion (one reply to the benchmark) and uses steps; it returns when the
motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.goto(arm, xyz, pitch=, yaw=, roll=, grip=, path='auto')` | moves the grasp point to `xyz` with those angles (`path`: 'auto' a straight line, else a joint swing close to it; 'line'; 'swing' any curve); `grip` ('open', 'close' or 0..1) starts as the move starts: open on the approach with it, but let go of a held object with `gripper` once there | Move |
| `robot.goto(arm, by=(dx, dy, dz))` | moves relative to where it is (with `xyz` too: relative to `xyz`) | Move |
| `robot.goto('both', by=(dx, dy, dz))` | both hands together by the same offset, along straight lines, keeping their angles (a load held in both hands); nothing else is given with 'both' | [Move, Move] |
| `robot.goto(..., dry_run=True)` | only checks: reachable or not, path, steps (not whether something is in the way); nothing moves, not a motion | Move |
| `robot.goto(..., contact=True)` | for a press, push or pull meant to meet resistance: it raises no `Blocked` | Move |
| `robot.gripper(arm, opening)` | 'open', 'close' or 0..1 (1 = fully open; the task card gives the gap at each opening); on five-finger hands the opening of a pinch or a wrap | Grip (Hand) |
| `robot.hand(arm, shape, opening, fingers=)` | where the task card lists five-finger hands: sets a hand's shape - 'flat', 'pinch', 'wrap' or 'point' -, a pinch's or a wrap's opening (as `gripper`), and single fingers (`fingers={'thumb': 0..1 curl, or its four joint angles in degrees}`); the arm stays, and the grasp point `goto` places becomes the shape's working point (the task card gives each) | Hand |
| `robot.home(arm='both')` | lifts (if low) and swings back to the start pose; grippers unchanged; with 'both' the waist (where the torso turns) turns back to the start too, and the torso's posture (`robot.body`) returns to the start's; the head stays | Homing |
| `robot.wait(n)` | holds everything still for n steps (n <= 150) | Waited |
| `robot.waist(yaw=)`, `robot.waist(by=)` | where the torso turns (the task card says): turns it to that heading (0 = the start's) or by that much, + to the left; both arms and the head camera turn with it, keeping their joints | text |
| `robot.drive(forward=, left=, turn=)` | where the base drives (the task card says): moves it forward and to the left (m) and turns it (degrees, + to the left) from where it stands, at most 1.5 m and 180 degrees a call; the arms keep their pose and ride along; the world frame moves with the base; the reply gives the move the head pictures before and after read | text |
| `robot.body(lean=, height=)` | where the task card lists the torso: leans it forward (degrees from upright) and raises or lowers the shoulders (z, m) in one motion, either left out kept; the arms keep their joints and ride along, so the hands move with the torso; refused if an arm would come near the floor on the way; `dry_run=True` only checks; `robot.body()` reads the posture and the head's | text |
| `robot.head(yaw=, pitch=)` | where the task card lists the head: turns it on the torso (+ to the left) and tips it down from the torso's line; the head camera moves with it | text |

Seeing and measuring - not motions, no steps used (a black picture makes them hold one step for a
new one):

| call | what it does | returns |
|---|---|---|
| `robot.look(cams='head', grid=, marks=, masks=, crop=, zoom=, tile=)` | attaches the latest picture(s) - from the end of the last motion - to this call's result (see section 4) | Picture |
| `robot.find(cam, 'red cup')` | outlines every instance the words describe, best first | list of Mask |
| `robot.find(cam, at=(x, y))` | outlines the object under that pixel | list of Mask |
| `robot.find(cam, points=[(x, y), ...], labels=[1, 0, ...])`, `box=(x0, y0, x1, y1)` | outline from several points (1 = on the object, 0 = not on it) or a box | list of Mask |
| `robot.measure(target)` | 3-D position, top, size and grasp angle of one object from the depth pictures; `target` is words, a Mask, `(cam, (x, y))`, a list of Masks of the same object in different cameras, or a Thing measured before | Thing |
| `robot.to_world(cam, (x, y))` | the point seen at a pixel, from the depth picture; `z=` or `pair=(cam2, (x2, y2))` (section 4) | Spot |
| `robot.to_pixel(xyz, cam)` | where a world point appears in a camera's picture now, or None | (x, y) |

Reading - no steps used:

| call | what it gives |
|---|---|
| `robot.task` | the instruction, the start poses and the benchmark's own description of the task, step by step |
| `robot.state()` | both arms: grasp point, angles, gripper, holding, distance from the start pose. One arm: `robot.state().left` - `.at`, `.angles`, `.gripper`, `.holding` |
| `robot.image(cam)` | the latest picture as an RGB numpy array (height, width, 3) |
| `robot.camera(cam)` | a camera's focal length, centre and pose now |
| `robot.things` | every object measured so far (the latest measurement of each) |

Results print as one line or a few, with problems first. Their fields can be used in code:

```
Move left: at (0.550, 0.150, 0.820) pitch 90 yaw 0 roll 0  ±1 mm 0°  14 steps (line)
```
`.ok` (reached, not blocked, nothing slipped out), `.reached`, `.blocked`, `.lag_mm` (how far it
ended behind its command), `.dropped`, `.at`, `.angles`, `.steps`. A pose that cannot be reached
raises `NotExecuted` (`Move left: NOT executed` with the reason and `reachable instead: ...`):
nothing moved, it is not a motion, and the rest of the cell does not run. A move is sent whole and
runs to its end; one that ends 2 cm or more behind its command raises `Blocked`
(`Move left: BLOCKED — at ...`): it is pressing on something - an object, the table, the other arm
or the robot's own body - and keeps pressing until the next motion; the rest of the cell does not
run, so you can look.
What the gripper held slipping out on the way raises `Dropped`. `short of target`: it ended off the
target by the distance the line gives (no error raised); a note says when something stopped it short
(after `contact=True` a push that met what it pushes on ends so, and keeps pressing on it).

```
Grip right: closed on something ~3.0 cm wide (holding)  40 steps
```
`.holding` (True: the fingers stopped on something; False: closed fully - nothing between them, or
something very thin; None after an open), `.width_m` (the gap between the pads; on something held it
reads lower than the object's own size, by up to about 1.5 cm), `.opening`.

```
Hand right: pinch, opening 0.00 (it was flat)  24 steps
```
Five-finger hands: `.holding` (True: the holding fingers stopped on something - a pinch's thumb and
index, at least two of a wrap's four; False: they closed on nothing; None for flat and point),
`.touching` (fingers stopped short of where they were sent), `.shape`, `.opening`, `.at` and
`.angles` (the grasp point now).

```
Home: left home, right home  45 steps
Waited 30 steps (1.00 s)
Waist: yaw 20° from the start's (+20°), 32 steps; the arms turned with it: left grasp point now (...); right grasp point now (...)
Drove: asked forward 0.30 m, left 0.00 m, turned +0°; the head pictures before and after read forward 0.29 m, left 0.01 m, turned +0° (212 of 380 matched points agree), 60 steps. The world of the robot calls moved with the base (...)
```
A drive's reply reads the base's move from the still things the head camera saw before and after
it, with a note when the base hardly moved or ended away from where it was sent; after a large
turn the pictures share too little and the reply gives what was asked - look then.
After a drive, a Thing measured before it is refused by `goto` (measure it again; `measure` of that
Thing looks for it where the drive - as read, else as asked - should have put it); `to_world` warns
of a pixel from a picture seen before it.

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
A Spot is a point: `.xyz` (also `spot[0]`, `x, y, z = spot`, or pass the Spot itself as a target),
`.mm_per_px`, `.gap_mm` (with `pair=`).

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotExecuted` (a goto that cannot be done; nothing moved), `Blocked`
and `Dropped` (a goto that ran into trouble; the message is the Move), `NotFound`, `RobotError`
(`NotExecuted`, `Blocked` and `Dropped` are ones); the names are defined in the session. Catch them
in code when you have a plan B, or check a doubtful pose first with `dry_run=True`, which reports
instead of raising.
