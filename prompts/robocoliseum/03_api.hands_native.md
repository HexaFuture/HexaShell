# 3. The `robot` API

Cameras are named 'head', 'left_wrist' and 'right_wrist' ('left' and 'right' work too). Arms are
'left' and 'right'.

Motion - each call is one motion (one reply to the benchmark) and uses steps of the episode's span; it
returns when the motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.goto(arm, xyz, pitch=, yaw=, roll=, grip=, path='auto')` | moves the grasp point to `xyz` with those angles (`path`: 'auto' a straight line, else a joint swing close to it; 'line'; 'swing' any curve); `grip` ('open', 'close' or 0..1) starts as the move starts: open on the approach with it, but let go of a held object with `gripper` once there | Move |
| `robot.goto(arm, by=(dx, dy, dz))` | moves relative to where it is (with `xyz` too: relative to `xyz`) | Move |
| `robot.goto('both', by=(dx, dy, dz))` | both hands together by the same offset, along straight lines, keeping their angles (a load held in both hands); nothing else is given with 'both' | [Move, Move] |
| `robot.goto(..., dry_run=True)` | only checks: reachable or not, path, steps (not whether something is in the way); nothing moves, not a motion | Move |
| `robot.goto(..., contact=True)` | for a press, push or pull meant to meet resistance: it raises no `Blocked` | Move |
| `robot.gripper(arm, opening)` | 'open', 'close' or 0..1 (1 = fully open; the task card gives the gap at each opening); on five-finger hands the opening of a pinch or a wrap | Grip (Hand) |
| `robot.hand(arm, shape, opening, fingers=)` | where the task card lists five-finger hands: sets a hand's shape - 'flat', 'pinch', 'wrap' or 'point' -, a pinch's or a wrap's opening (as `gripper`), and single fingers (`fingers={'thumb': 0..1 curl, or its four joint angles in degrees}`); the arm stays, and the grasp point `goto` places becomes the shape's working point (the task card gives each) | Hand |
| `robot.home(arm='both')` | lifts (if low) and swings back to the start pose; grippers unchanged | Homing |
| `robot.wait(n)` | holds everything still for n steps (n <= 150) | Waited |

Seeing and placing points - not motions, no steps used (a black picture makes them hold one step for
a new one):

| call | what it does | returns |
|---|---|---|
| `robot.look(cams='head', grid=, marks=, masks=, crop=, zoom=, tile=)` | attaches the latest picture(s) - from the end of the last motion - to this call's result (see section 4) | Picture |
| `robot.find(cam, 'red cup')` | outlines every instance the words describe, best first | list of Mask |
| `robot.find(cam, at=(x, y))` | outlines the object under that pixel | list of Mask |
| `robot.find(cam, points=[(x, y), ...], labels=[1, 0, ...])`, `box=(x0, y0, x1, y1)` | outline from several points (1 = on the object, 0 = not on it) or a box | list of Mask |
| `robot.to_world(cam, (x, y), z=h)` | the point seen at a pixel, where its ray meets the height h (a height you know; section 4) | Spot |
| `robot.to_world(cam, (x, y), pair=(cam2, (x2, y2)))` | the point seen at a pixel in each of two pictures (section 4) | Spot |
| `robot.measure(target)` | not available in these tasks: it reads the depth pictures, which these cameras do not give; it raises ValueError | - |
| `robot.to_pixel(xyz, cam)` | where a world point appears in a camera's picture now, or None | (x, y) |

Reading - no steps used:

| call | what it gives |
|---|---|
| `robot.task` | the instruction, the span (its steps used and left), the start poses and the benchmark's own description of the task, step by step |
| `robot.state()` | the steps used and left of the span; both arms: grasp point, angles, gripper, holding, distance from the start pose. One arm: `robot.state().left` - `.at`, `.angles`, `.gripper`, `.holding` |
| `robot.image(cam)` | the latest picture as an RGB numpy array (height, width, 3) |
| `robot.camera(cam)` | a camera's focal length, centre and pose now |

Results print as one line or a few, with problems first; a motion's first line ends with the steps
left of the episode's span. Their fields can be used in code:

```
Move left: at (0.550, 0.150, 0.820) pitch 90 yaw 0 roll 0  ±1 mm 0°  14 steps (line)  [486 steps left]
```
`.ok` (reached, not blocked, nothing slipped out), `.reached`, `.blocked`, `.lag_mm` (how far it
ended behind its command), `.dropped`, `.at`, `.angles`, `.steps`, `.steps_left`. A pose that cannot
be reached raises `NotExecuted` (`Move left: NOT executed` with the reason and `reachable instead: ...`):
nothing moved, it is not a motion, and the rest of the cell does not run. A move is sent whole and
runs to its end; one that ends 2 cm or more behind its command raises `Blocked`
(`Move left: BLOCKED — at ...`): it is pressing on something - an object, the table, the other arm
or the robot's own body - and keeps pressing until the next motion; the rest of the cell does not
run, so you can look.
What the gripper held slipping out on the way raises `Dropped`. `short of target`: it ended off the
target by the distance the line gives (no error raised); a note says when something stopped it short
(after `contact=True` a push that met what it pushes on ends so, and keeps pressing on it).

```
Grip right: closed on something ~3.0 cm wide (holding)  40 steps  [446 steps left]
```
`.holding` (True: the fingers stopped on something; False: closed fully - nothing between them, or
something very thin; None after an open), `.width_m` (the gap between the pads; on something held it
reads lower than the object's own size, by up to about 1.5 cm), `.opening`.

```
Hand right: pinch, opening 0.00 (it was flat)  24 steps  [422 steps left]
```
Five-finger hands: `.holding` (True: the holding fingers stopped on something - a pinch's thumb and
index, at least two of a wrap's four; False: they closed on nothing; None for flat and point),
`.touching` (fingers stopped short of where they were sent), `.shape`, `.opening`, `.at` and
`.angles` (the grasp point now). In some tasks the fingers' readings cannot tell what they meet: the
result then says "touching or holding cannot be told here", `.holding` is None and `.touching` empty.

```
Home: left home, right home  45 steps  [377 steps left]
Waited 30 steps (1.00 s)  [347 steps left]
```

```
Mask#7 head 'red block' score 0.91  centroid (412, 233)  box (391, 210)-(433, 258)  1520 px
Spot (0.507, 0.031, 0.800) from head (320, 300) on plane z 0.800  (~2.5 mm per pixel)
Spot (0.790, -0.327, 0.948) from right_wrist (479, 359) on two views (right_wrist + head)  rays 0.1 mm apart, 65° between them  (~2.8 mm per pixel)
```
A Spot is a point: `.xyz` (also `spot[0]`, `x, y, z = spot`, or pass the Spot itself as a target),
`.mm_per_px`, `.gap_mm` (with `pair=`).

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotExecuted` (a goto that cannot be done; nothing moved), `Blocked`
and `Dropped` (a goto that ran into trouble; the message is the Move), `NotFound`, `RobotError`
(`NotExecuted`, `Blocked` and `Dropped` are ones); the names are defined in the session. Catch them
in code when you have a plan B, or check a doubtful pose first with `dry_run=True`, which reports
instead of raising.
