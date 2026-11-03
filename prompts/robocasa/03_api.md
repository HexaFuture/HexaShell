# 3. The `robot` API

Cameras are named 'left', 'right' and 'wrist'. There is one arm, so motions name no arm.

Motion - uses steps; each call returns when the motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.goto(xyz, pitch=, yaw=, roll=, grip=)` | moves the grasp point to `xyz` with those angles in a straight line, turning the hand on the way; `grip` ('open', 'close' or 0..1) starts as the move starts: open on the approach with it, but let go of a held object with `gripper` once there | Move |
| `robot.goto(by=(dx, dy, dz))` | moves relative to where it is (with `xyz` too: relative to `xyz`) | Move |
| `robot.goto(..., dry_run=True)` | only checks whether the pose is reachable from where the base is (not whether something is in the way); nothing moves | Move |
| `robot.goto(..., speed=0.05)` | the same at most that many m/s (0.01 to 0.25, the most and the default), turning slower in proportion; `arc` takes it too (holding something 0.25 m or more from its line, a door's handle, an arc defaults to 0.1 until arcs holding it have turned the door 30 degrees: faster, the bar slips out of the fingers as the door starts to open) | Move |
| `robot.arc(center, angle, axis='z')` | swings the grasp point by `angle` degrees around the line through `center` along `axis` ('x', 'y', 'z', '-z' or a direction), turning the hand with it by the same angle about the same axis (about 'y' that changes the pitch): the path of a handle about its hinge; + is counter-clockwise seen from the axis's tip (about 'z': seen from above, so a point ahead of `center` swings to the left); at most 180 at a time | Move |
| `robot.push_arc(center, angle, axis='z', push=None, press=0.03, tilt=25)` | pushes a hinged part the hand touches (a lid, a head, a free edge) up to `angle` degrees around its hinge (`center`, `axis` and the sign as for `arc`) without holding it: the hand goes along `push` (a direction into the part; None: along its circle about the hinge) until it meets the part, then keeps pressing `press` m (0.01-0.05) past itself, leaned `tilt` degrees from `push` toward the way the part turns, the push and the hand turning as the part turns, harder while it turns slowly; `.turned` is how far the hand went around, `.stop` why it stopped ('angle' = all the way) | Push |
| `robot.rim_pick(target, pitch=45)` | picks a round open dish (a bowl, a deep plate, a pot without handles) by the near side of its rim: `target` as for `measure`, or a point on it. From the depth around it, it finds the rim's outer edge on the side facing the base, comes down at `pitch` degrees (30-60) with roll 90 onto a point just inside that edge and under the rim's top, pinches the wall (once more nearer the edge if the fingers close on nothing) and lifts 0.1 m (`lift=`). The dish hangs level while the hand keeps that pitch and roll: carry it so, and set it down by moving the hand, not by turning it. It wants the dish within 50 degrees of the base's heading, its middle about 0.6 m away (else it says how to drive); `dry_run=True` only says where it would pinch | RimPick |
| `robot.gripper(opening)` | 'open', 'close', 'light' (closes, then holds lightly: small soft food only, see "The gripper" in section 5) or 0..1 (1 = 8 cm between the pads, 0.5 = 4 cm) | Grip |
| `robot.home()` | lifts (if low) and brings the arm back to its start pose relative to the base; the gripper is left as it is | Homing |
| `robot.wait(n)` | holds everything still for n steps (n <= 400) | Waited |
| `robot.drive(forward, left, turn)` | drives the base from where it is: metres forward and to its left, degrees of turn (+ to the left); the arm rides along, keeping its pose relative to the base | Drive |
| `robot.drive(to=(x, y), heading=)` | drives the base to a floor point and/or heading of the start frame (at most 4 m per call) | Drive |
| `robot.drive(turn=, about=(x, y))` | turns the base by `turn` degrees (+ to the left) about that floor point of the start frame instead of about itself: the base and the hand go round it on circles, the arm keeping its pose relative to the base. With the hand closed on a door's handle and `about` at its hinge, the door swings with them. `dry_run=True`: where the base and the hand would end and which way the hand first goes, nothing moves | Drive |

Seeing and measuring - no steps used:

| call | what it does | returns |
|---|---|---|
| `robot.look(cams='all', grid=, marks=, masks=, crop=, zoom=, tile=)` | attaches the current picture(s) to this call's result (see section 4) | Picture |
| `robot.find(cam, 'red mug')` | outlines every instance the words describe, best first | list of Mask |
| `robot.find(cam, at=(x, y))` | outlines the object under that pixel | list of Mask |
| `robot.find(cam, points=[(x, y), ...], labels=[1, 0, ...])`, `box=(x0, y0, x1, y1)` | outline from several points (1 = on the object, 0 = not on it) or a box | list of Mask |
| `robot.measure(target)` | 3-D position, top, size and grasp angle of one object from the depth pictures; `target` is words, a Mask, `(cam, (x, y))`, a list of Masks of the same object in different cameras, or a Thing measured before | Thing |
| `robot.to_world(cam, (x, y))` | the point seen at a pixel, from the depth picture; `z=` or `pair=(cam2, (x2, y2))` (section 4) | Spot |
| `robot.to_pixel(xyz, cam)` | where a point appears in a camera's picture now, or None | (x, y) |

Reading - no steps used:

| call | what it gives |
|---|---|
| `robot.task` | instruction, steps used and left, the start pose |
| `robot.state()` | grasp point, angles, gripper, holding, distance from the start pose; base position and heading; steps. Fields: `.at`, `.angles`, `.gripper`, `.holding`, `.base`, `.heading` |
| `robot.image(cam)` | the current picture as an RGB numpy array (512, 512, 3) |
| `robot.depth(cam)` | the current depth picture (512, 512): metres along the camera's axis |
| `robot.camera(cam)` | a camera's focal length, centre and pose now |
| `robot.things` | every object measured so far (the latest measurement of each) |

Results print as one line or a few, with problems first. Their fields can be used in code:

```
Move: at (0.612, -0.083, 0.981) pitch 90 yaw 12 roll 0  ±2 mm 1°  18 steps  [842 steps left]
```
`.ok` (reached and not blocked), `.reached`, `.blocked`, `.at`, `.angles`, `.steps`. A pose that
cannot be reached from where the base is raises `NotExecuted` (`Move: NOT executed` with the
reason): nothing moved, and the rest of the cell does not run. `BLOCKED`: the hand stopped short -
it is pressing on something. `short of target`: it ended farther from the target than usual.

```
Grip: closed on something ~3.4 cm wide (holding)  12 steps  [830 steps left]
```
`.holding` (True: the fingers stopped on something; False: open, or closed fully; None: still
moving), `.width_m`, `.opening`.

```
Drive: base at (0.400, 0.000) heading 0  31 steps  [799 steps left]
```
`.reached`, `.blocked`, `.at` (x, y), `.heading`.

```
Thing#3 'red mug'  center (0.712, -0.083, 0.951)  top 0.995  size 0.101×0.083×0.088  grip_yaw 12°  on z 0.907  2 views (left, right)
```
`.center` (x, y, z: the middle of what the cameras see of it), `.top` (its highest point), `.size`
(length, width, height), `.grip_yaw` (the yaw that closes down-pointing fingers across its narrow
side; None when round: any yaw), `.on_z` (the height of its lowest visible point - about the surface
it stands on; `.on` is the same as text), `.views`, `.masks`. A warning comes first when there is
one. When the object is not found it prints `NOT FOUND` with the reason, and `.center` raises
NotFound.

```
Mask#7 left 'red mug' score 0.91  centroid (412, 233)  box (391, 210)-(433, 258)  1520 px
Spot (0.712, -0.083, 0.995) from left (412, 220) on the depth picture  (~1.6 mm per pixel)
```

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotExecuted` (a motion that cannot be done; nothing moved),
`NotFound`, `RobotError` (`NotExecuted` is one); the names are defined in the session. Catch them
in code when you have a plan B, or check a doubtful pose first with `dry_run=True`, which reports
instead of raising.
