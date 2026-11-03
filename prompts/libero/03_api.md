# 3. The `robot` API

Cameras are named 'agentview' and 'wrist'. There is one arm, so motions name no arm.

Motion - uses steps; each call returns when the motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.goto(xyz, pitch=, yaw=, roll=, grip=)` | moves the grasp point to `xyz` with those angles in a straight line, turning the hand on the way; `grip` ('open', 'close' or 0..1) starts as the move starts: open on the approach with it, but let go of a held object with `gripper` once there | Move |
| `robot.goto(by=(dx, dy, dz))` | moves relative to where it is (with `xyz` too: relative to `xyz`) | Move |
| `robot.goto(..., dry_run=True)` | only checks whether the pose is reachable (not whether something is in the way); nothing moves | Move |
| `robot.goto(..., speed=0.05)` | the same at most that many m/s (0.01 to 0.25, the most and the default), turning slower in proportion; `arc` takes it too | Move |
| `robot.arc(center, angle, axis='z')` | swings the grasp point by `angle` degrees around the line through `center` along `axis` ('x', 'y', 'z', '-z' or a direction), turning the hand with it by the same angle about the same axis (about 'y' that changes the pitch): the path of a handle about its hinge; + is counter-clockwise seen from the axis's tip (about 'z': seen from above, so a point ahead of `center` swings to the left); at most 180 at a time | Move |
| `robot.gripper(opening)` | 'open', 'close' or 0..1 (1 = 8 cm between the pads, 0.5 = 4 cm) | Grip |
| `robot.home()` | lifts (if low) and brings the hand back to its start pose; the gripper is left as it is | Homing |
| `robot.wait(n)` | holds everything still for n steps (n <= 400) | Waited |

Seeing and measuring - no steps used:

| call | what it does | returns |
|---|---|---|
| `robot.look(cams='all', grid=, marks=, masks=, crop=, zoom=, tile=)` | attaches the current picture(s) to this call's result (see section 4) | Picture |
| `robot.find(cam, 'black bowl')` | outlines every instance the words describe, best first | list of Mask |
| `robot.find(cam, at=(x, y))` | outlines the object under that pixel | list of Mask |
| `robot.find(cam, points=[(x, y), ...], labels=[1, 0, ...])`, `box=(x0, y0, x1, y1)` | outline from several points (1 = on the object, 0 = not on it) or a box | list of Mask |
| `robot.measure(target)` | 3-D position, top, size and grasp angle of one object from the depth pictures; `target` is words, a Mask, `(cam, (x, y))`, a list of Masks of the same object in different cameras, or a Thing measured before | Thing |
| `robot.to_world(cam, (x, y))` | the point seen at a pixel, from the depth picture; `z=` or `pair=(cam2, (x2, y2))` (section 4) | Spot |
| `robot.to_pixel(xyz, cam)` | where a point appears in a camera's picture now, or None | (x, y) |

Reading - no steps used:

| call | what it gives |
|---|---|
| `robot.task` | instruction, steps used and left, the start pose |
| `robot.state()` | grasp point, angles, gripper, holding, distance from the start pose; steps. Fields: `.at`, `.angles`, `.gripper`, `.holding` |
| `robot.image(cam)` | the current picture as an RGB numpy array (512, 512, 3) |
| `robot.depth(cam)` | the current depth picture (512, 512): metres along the camera's axis |
| `robot.camera(cam)` | a camera's focal length, centre and pose now |
| `robot.things` | every object measured so far (the latest measurement of each) |

Results print as one line or a few, with problems first. Their fields can be used in code:

```
Move: at (0.512, -0.083, 0.081) pitch 90 yaw 12 roll 0  ±2 mm 1°  18 steps  [262 steps left]
```
`.ok` (reached and not blocked), `.reached`, `.blocked`, `.at`, `.angles`, `.steps`. A pose that
cannot be reached raises `NotExecuted` (`Move: NOT executed` with the reason): nothing moved, and
the rest of the cell does not run. `BLOCKED`: the hand stopped short - it is pressing on something.
`short of target`: it ended farther from the target than usual.

```
Grip: closed on something ~3.4 cm wide (holding)  12 steps  [250 steps left]
```
`.holding` (True: the fingers stopped on something; False: open, or closed fully; None: still
moving), `.width_m`, `.opening`.

```
Thing#3 'black bowl'  center (0.612, 0.083, 0.021)  top 0.047  size 0.131×0.128×0.052  grip_yaw None  on z -0.011  2 views (agentview, wrist)
```
`.center` (x, y, z: the middle of what the cameras see of it), `.top` (its highest point), `.size`
(length, width, height), `.grip_yaw` (the yaw that closes down-pointing fingers across its narrow
side; None when round: any yaw), `.on_z` (the height of its lowest visible point - about the surface
it stands on; `.on` is the same as text), `.views`, `.masks`. A warning comes first when there is
one. When the object is not found it prints `NOT FOUND` with the reason, and `.center` raises
NotFound.

```
Mask#7 agentview 'black bowl' score 0.91  centroid (312, 233)  box (291, 210)-(333, 258)  1520 px
Spot (0.612, 0.083, 0.047) from agentview (312, 220) on the depth picture  (~1.9 mm per pixel)
```

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotExecuted` (a motion that cannot be done; nothing moved),
`NotFound`, `RobotError` (`NotExecuted` is one); the names are defined in the session. Catch them
in code when you have a plan B, or check a doubtful pose first with `dry_run=True`, which reports
instead of raising.
