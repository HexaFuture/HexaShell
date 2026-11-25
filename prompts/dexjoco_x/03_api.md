# 3. The `robot` API

Hands are named 'right' and 'left' (only those of the task's arms). Cameras are named 'scene',
'right_wrist' and 'left_wrist' ('wrist' is the right one).

Motion - uses steps; each call returns when the motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.hand(side, xyz, point='palm', facing=)` | moves the hand's `point` in a straight line to `xyz`, turning the hand evenly on the way: `facing` 'down', 'up', 'forward', 'inward' or 'outward', or `normal=` / `finger=` directions (left out: the orientation it has) | Move |
| `robot.hand(side, by=(dx, dy, dz))` | moves relative to where the point is (with `xyz` too: relative to `xyz`) | Move |
| `robot.hand(side, xyz, point='grasp', along=(ax, ay, az), facing=)` | for a power grip: turns the hand so that a handle running along `along` lies across the fingers at the slant a fist holds it; `along` points toward the thumb's side; `facing` / `normal` give the palm's way about the handle | Move |
| `robot.hand(..., approach=0.08)` | goes first to the goal set back that far along the palm's normal, then comes the last part straight in along it, slowly (6 cm/s), so the fingers do not sweep into what they will hold (up to 0.25) | Move |
| `robot.hand(side, xyz, point='camera', look_at=target)` | puts the wrist camera at `xyz` looking at `target` (at least 10 cm away); the hand turns as the arm reaches best, and the note says which way the picture's top points | Move |
| `robot.hand(..., speed=0.08)` | the point's speed in m/s (the task card gives the default and the most) | Move |
| `robot.hand(..., dry_run=True)` | only checks whether the arm can make the motion (its joints along the way, not whether something is in the way): reachable, at the edge of reach, or NOT reachable; nothing moves, no steps | Move |
| `robot.fingers(side, shape, seconds=0.5)` | sets the fingers to a named shape (below), moving evenly over `seconds` (0.1-3) | Fingers |
| `robot.fingers(side, shape, amount=0.5)` | that share of the way from open to the shape | Fingers |
| `robot.fingers(side, index=0.8, thumb=0.3, oppose=0.5, spread=0.2)` | per finger: `thumb`, `index`, `middle`, `ring`, `little` each 0 (straight) to 1 (curled in); `oppose` (0-1) swings the thumb across the palm; `spread` (-1 to 1) fans the four fingers out (negative: together); with a shape, these override it for those fingers | Fingers |
| `robot.arc(side, center, angle, axis='z', speed=)` | turns the hand `angle` degrees about the line through `center` along `axis` ('x', 'y', 'z' or a direction), the hand turning with it: a door about its hinge, a lid, a dial; + is counter-clockwise seen from the axis's tip (about 'z': seen from above); at most 180 at a time; `speed` in degrees per second | Move |
| `robot.home(side)` | moves a hand back to its start pose (with no side: every hand at once); the fingers keep their setting | Home |
| `robot.wait(n)` | holds everything still for n steps (n <= 400) | Waited |

The shapes, as the benchmark's demonstrations closed the hand:

| shape | what it is | used for |
|---|---|---|
| 'open' | the start: fingers straight, thumb beside them | letting go, approaching |
| 'flat' | all fingers straight | pushing, a flat support |
| 'power' | a fist round a handle | the hammer's handle |
| 'wrap' | the fingers round a box or a can, the thumb opposite | a box, a can, a bottle, a camera |
| 'hook' | the fingers bent as a hook, the thumb in | a basket's or a bag's handle |
| 'pinch' | thumb tip to index tip | a thin thing: a disk's rim |
| 'tripod' | thumb, index and middle tips together | a small thing: a block, a ring, food (the hand tilted, section 5) |
| 'point' | the index straight, the others curled | pressing a button or a key with the index tip |

Seeing and measuring - no steps used:

| call | what it does | returns |
|---|---|---|
| `robot.look(cams='all', grid=, marks=, masks=, crop=, zoom=, tile=)` | attaches the current picture(s) to this call's result (see section 4) | Picture |
| `robot.find(cam, 'red box')` | outlines every instance the words describe, best first | list of Mask |
| `robot.find(cam, at=(x, y))` | outlines the object under that pixel | list of Mask |
| `robot.find(cam, points=[(x, y), ...], labels=[1, 0, ...])`, `box=(x0, y0, x1, y1)` | outline from several points (1 = on the object, 0 = not on it) or a box | list of Mask |
| `robot.measure(target)` | 3-D position, top, size and long side of one object from the depth pictures; `target` is words, a Mask, `(cam, (x, y))`, or a Thing measured before (measured again where it is now) | Thing |
| `robot.to_world(cam, (x, y))` | the surface point seen at a pixel, from the depth picture; `z=0.95`: where the pixel's ray meets the plane z = 0.95 | Spot |
| `robot.to_pixel(xyz, cam)` | where a point appears in a camera's picture now, or None | (x, y) |

Reading - no steps used:

| call | what it gives |
|---|---|
| `robot.task` | the instruction, the step budget (used, left), the arms |
| `robot.state()` | each hand: palm (point, normal, finger direction), grasp point, fingertips, the fingers' shape and curl, which fingers press on something, holding, the wrist camera; the budget. `robot.state().right.palm`, `.grasp`, `.tips['index']`, `.holding`, `.pressing`, `.camera` |
| `robot.image(cam)` | the current picture as an RGB numpy array (512, 512, 3) |
| `robot.depth(cam)` | the current depth picture (512, 512): metres along the camera's axis, 0 where nothing is within 10 m |
| `robot.camera(cam)` | a camera's focal length, centre and pose now |
| `robot.things` | every object measured so far (the latest measurement of each) |

Results print as one line or a few, with problems first. Their fields can be used in code:

```
Hand right: grasp at (-0.322, -0.489, 1.001)  ±0 mm 0°  135 steps  [776 steps left]
```
`.ok` (reached and not blocked), `.reached` (within 1 cm and 5 degrees), `.at`, `.err_mm`,
`.err_deg`, `.blocked`, `.steps`, `.notes`.

```
Hand right: BLOCKED - palm at (-0.218, 0.034, 1.091), 26 mm / 0° from (-0.212, 0.039, 1.067)  16 steps  [612 steps left]
  blocked: it stopped 2.6 cm short (something holds it up)
  note: the motion stopped there: something is in the hand's way or it presses on something - look before going on
```
The hand stopped where something held it up and stays there. Other reasons: the arm has a joint at
the end of its travel (the note names it), or the goal is out of the arm's reach with the hand held
that way. A motion meant to end against something (a strike, a press) reports BLOCKED as well.

```
Hand right (dry run): NOT reachable - the palm would end 33 mm / 8° from (-0.100, -0.200, 1.100)
  note: the arm would end with joint 2 (the shoulder) and joint 6 (the wrist) at the end of its travel: that pose is out of the arm's reach with the hand held this way - another orientation of the hand, or a point nearer the arm's base, reaches easier
Hand right: camera at (-0.207, -0.421, 1.486)  ±2 mm 0°  79 steps  [911 steps left]
  note: the right_wrist camera looks at (-0.206, -0.422, 1.036) from 0.45 m; the top of its picture is toward +y (the robot's left)
```

Touch notes come after a motion when the fingers felt something (section 2): "the right hand presses
on something now (index, middle)", "on the way something pushed on the right hand's fingers (index):
the hand touched something - it may have pushed or tipped it", and for a hand that held something:
"... what it held has slipped out or is slipping".

```
Fingers right: power, holding (pressing on something: thumb, middle)  17 steps  [759 steps left]
  curl: thumb 0.00 index 0.37 middle 0.41 ring 0.48 little 0.44
```
`.holding` (True: the thumb and one more finger press on something), `.pressing`, `.curl`
(measured), `.asked` (set). Closing on nothing says so: "the fingers closed without pressing on
anything: the hand holds nothing".

```
Arc right: turned 30° of 30°, palm at (-0.348, -0.000, 1.472)  24 steps  [966 steps left]
Home: right home  65 steps  [822 steps left]
```
Arc: `.turned` (degrees), `.ok`, `.blocked`. Home: `.ok`.

```
Thing#5 'yellow wooden handle'  center (-0.362, -0.515, 1.014)  top 1.044  size 0.211×0.045×0.060  grip_yaw 33°  on z 0.984  1 view (right_wrist)
  note: seen by right_wrist only: its far side is hidden, so the centre may lie up to half its depth toward that camera
```
`.center` (x, y, z: the middle of what the cameras see of it - of a thing seen from above, the
middle of its upper surface), `.top` (its highest point), `.size` (length, width, height), `.grip_yaw`
(degrees; None for a round thing: the direction (cos, sin) of the long side of its outline seen from
above, either way along it), `.on_z` (the height of its lowest visible point - about the surface it
stands on), `.views`, `.masks`. A warning comes first when there is one. When the object is not
found it prints `NOT FOUND` with the reason, and `.center` raises NotFound.

```
Mask#3 scene 'grey metal block' score 0.65  centroid (94, 206)  box (75, 170)-(112, 244)  2021 px
Spot (-0.035, 0.089, 1.076) from scene (294, 217) on the depth picture  (~2.3 mm per pixel)
```

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotFound`, `RobotError`; the names are defined in the session. Catch
them in code when you have a plan B, or check a doubtful pose first with `dry_run=True`.
