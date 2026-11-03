# 3. The `robot` API

Cameras are named 'head', 'left_wrist' and 'right_wrist' ('left' and 'right' work too). Arms are
'left' and 'right'.

Motion - uses steps; each call returns when the motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.goto(arm, xyz, pitch=, yaw=, roll=, grip=, path='auto')` | moves the grasp point to `xyz` with those angles (`path`: 'auto' a straight line, else a joint swing close to it; 'line'; 'swing' any curve); `grip` ('open', 'close' or 0..1) starts as the move starts: open on the approach with it, but let go of a held object with `gripper` once there | Move |
| `robot.goto(arm, by=(dx, dy, dz))` | moves relative to where it is (with `xyz` too: relative to `xyz`) | Move |
| `robot.goto(..., dry_run=True)` | only checks: reachable or not, path, steps, and whether it comes next to the other arm; nothing moves | Move |
| `robot.gripper(arm, opening)` | 'open', 'close' or 0..1 (1 = 9 cm between the pads) | Grip |
| `robot.home(arm='both')` | lifts (if low) and swings back to the start pose; grippers unchanged | Homing |
| `robot.wait(n)` | holds everything still for n steps (n <= 200) | Waited |

Seeing and measuring - no steps used:

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
| `robot.task` | instruction, steps used and left, start poses |
| `robot.state()` | both arms: grasp point, angles, gripper, holding, distance from the start pose; steps. One arm: `robot.state().left` - `.at`, `.angles`, `.gripper`, `.holding` |
| `robot.image(cam)` | the current picture as an RGB numpy array (240, 320, 3) |
| `robot.camera(cam)` | a camera's focal length, centre and pose now |
| `robot.things` | every object measured so far (the latest measurement of each) |

Results print as one line or a few, with problems first. Their fields can be used in code:

```
Move left: at (-0.148, -0.003, 0.858) pitch 90 yaw -1 roll 0  ±1 mm 0°  14 steps (line)  [9906 steps left]
```
`.ok` (reached and not blocked), `.reached`, `.blocked`, `.at`, `.angles`, `.steps`. A pose that
cannot be reached raises `NotExecuted` (`Move left: NOT executed` with the reason and `reachable
instead: ...`): nothing moved, and the rest of the cell does not run. `BLOCKED`: the arm stopped
short of the target - it is touching something (the note says so when that is the other arm).

```
Grip right: closed on something ~3.4 cm wide (holding)  6 steps  [9890 steps left]
```
`.holding` (True: the fingers stopped on something; False: open, or closed fully; None: they
stayed about fully open - blocked), `.width_m`, `.opening`.

```
Thing#3 'red cup'  center (-0.151, 0.001, 0.770)  top 0.806  size 0.071×0.069×0.071 round  grip_yaw any  on z 0.735  2 views (head, left_wrist)
```
`.center` (x, y, z: the middle of what the cameras see of it), `.top` (its highest point), `.size`
(length, width, height), `.grip_yaw` (the yaw that closes down-pointing fingers across its narrow
side; None when round: any yaw), `.on_z` (the height of its lowest visible point - about the surface
it stands on; `.on` is the same as text), `.views`, `.masks`. A warning comes first when there is
one, notes follow. When the object is not found it prints `NOT FOUND` with the reason, and `.center`
raises NotFound.

```
Mask#7 head 'red cup' score 0.91  centroid (112, 133)  box (101, 120)-(123, 146)  420 px
Spot (-0.151, 0.001, 0.806) from head (112, 125) on the depth picture  (~2.4 mm per pixel)
```

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotExecuted` (a goto that cannot be done; nothing moved), `NotFound`,
`RobotError` (`NotExecuted` is one); the names are defined in the session. Catch them in code when
you have a plan B, or check a doubtful pose first with `dry_run=True`, which reports instead of
raising.
