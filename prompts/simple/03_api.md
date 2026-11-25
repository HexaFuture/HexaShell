# 3. The `robot` API

The camera is named 'gaze' ('head' works too). Hands are named 'left' and 'right'.

Motion - uses steps; each call returns when the motion is done:

| call | what it does | returns |
|---|---|---|
| `robot.body(height, pitch=, yaw=)` | sets the pelvis's height above the ground (0.30-0.79 m), the torso's forward lean `pitch` (0-60) and its turn `yaw` left of the pelvis (-45 to 45); left out, a part keeps what it was last asked for; closed on the measurement and then held | BodyMove |
| `robot.walk(x, y, heading=)` | walks the pelvis to ground point (x, y) and turns to `heading` (start frame; left out: as now); at most 4 m per call | Walk |
| `robot.walk(by=(forward, left), turn=)` | walks from where the pelvis is, in its own directions now; `turn` degrees from the heading now (+ left) | Walk |
| `robot.hand(side, xyz, facing=, seconds=, via=, hold=)` | moves one palm in straight lines to `xyz`, through the `via` points first (up to 11); `facing` 'down', 'up', 'forward', 'inward' or 'outward', or `normal=` / `finger=` start-frame directions (left out: the orientation it has); `seconds` per segment (0.4-8, default 1; the arm's speed limit may take longer) | HandMove |
| `robot.hand(side, by=(dx, dy, dz))` | moves relative to where the palm is (with `xyz` too: relative to `xyz`) | HandMove |
| `robot.hand(..., dry_run=True)` | only checks whether the arm reaches from the body's pose now (not whether something is in the way); nothing moves | HandMove |
| `robot.fingers(side, True)` / `robot.fingers(side, False)` / `robot.fingers(side, 'wide')` | closes ('close') or opens ('open') one hand's fingers; 'wide' opens them with the thumb swung back; 0.5 s (AMO) or 1 s (GR00T's controller) | Fingers |
| `robot.gaze(yaw, pitch)` / `robot.gaze(at=xyz)` | turns the camera (yaw -45 to 45, pitch 0 to 60, relative to the torso), or points it at a start-frame point (a Spot, a Thing); 120 degrees per second | Gaze |
| `robot.wait(n)` | holds everything still for n steps (n <= 400) | Waited |

`hold` (hand): what the palm does in later body commands and walks. 'torso' (the default): the arm
keeps its joints, so the palm moves with the torso (lean forward and it goes forward and down).
'world': it stays where it is in the room, as far as the arm reaches. 'pelvis': it moves with the
pelvis - carrying while walking or turning.

Perception - free, uses no steps:

| call | what it does | returns |
|---|---|---|
| `robot.find('gaze', 'red box')` | outlines in the current picture with SAM3: words (every instance, best first), `at=(x, y)` (the object under that pixel), `points=`/`labels=`, `box=` | list of Mask |
| `robot.measure(target)` | measures an object in 3-D from the depth picture: words, a Mask, ('gaze', (x, y)) or a Thing measured before (measured again where it is now) | Thing |
| `robot.to_world('gaze', (x, y))` | the surface point at a pixel, from the depth picture; `z=0.8`: where the pixel's ray meets the plane z = 0.8 | Spot |
| `robot.to_pixel(xyz, 'gaze')` | where a point appears in the picture now (None: out of view) | (x, y) |
| `robot.look('gaze', grid=, marks=, masks=, crop=, zoom=)` | attaches the current picture to the result so you can see it (see section 4) | Picture |

Reading - free:

| call | what it gives |
|---|---|
| `robot.task` | the instruction, the step budget (used, left) and the official limit |
| `robot.state()` | the pelvis (position, heading, height), the torso's pitch and yaw, each palm (point, normal, finger direction, fingers open or closed, what they touch, holding, hold), the gaze, the budget |
| `robot.image('gaze')` | the current picture as an RGB numpy array (400, 640, 3) |
| `robot.depth('gaze')` | the current depth picture (400, 640): metres along the camera's axis, 0 where nothing is within 10 m |
| `robot.camera('gaze')` | the camera's focal length, centre and pose now |
| `robot.things` | every object measured so far (the latest measurement of each) |

Results print as one line or a few, with problems first. Their fields can be used in code:

```
Body: pelvis 0.551 m high, torso pitch 25 yaw 0  73 steps  [2101 steps left]
```
`.reached`, `.height`, `.pitch`, `.yaw`, `.steps`. `short of target` and a note when the controller
could not get there (the legs' lowest pelvis, for one).

```
Walk: pelvis at (0.312, -0.041) heading 0, 32 mm from the target  118 steps  [1983 steps left]
```
`.reached` (within 5 cm), `.blocked` (it stopped: something is in the way), `.at` (x, y), `.heading`,
`.err_mm` (how far from the target; shown from 10 mm).

```
Hand right: palm at (0.552, -0.118, 0.851)  ±6 mm, normal 2° off  hold torso  41 steps  [1942 steps left]
```
`.ok` / `.reached` (within 2 cm), `.at`, `.err_mm`, `.steps`, `.notes` (a joint limit: the arm gets no
farther from this body pose; the hand touches something, or touched something on the way: it may have
pushed or tipped it). Body moves, walks and waits carry the same touch notes.

```
Fingers right: closed, holding (touching: thumb, index, middle)  50 steps  [1892 steps left]
```
`.holding` (True: the thumb and one more part touch what is in the hand; None while open),
`.touching`, `.closed`.

```
Thing#3 'cracker box'  center (0.612, -0.083, 0.851)  top 0.951  size 0.161×0.062×0.201  ...  1 view (gaze)
```
`.center` (x, y, z: the middle of what the camera sees of it), `.top` (its highest point), `.size`
(length, width, height), `.on_z` (the height of its lowest visible point - about the surface it
stands on), `.views`, `.masks`. A warning comes first when there is one. When the object is not found
it prints `NOT FOUND` with the reason, and `.center` raises NotFound. `.grip_yaw` (degrees; None for
a round thing) gives the long side of its outline seen from above: the direction (cos, sin) of it in
the start frame (either way along the side); the short side is across it.

```
Mask#7 gaze 'cracker box' score 0.91  centroid (412, 233)  box (391, 210)-(433, 258)  1520 px
Spot (0.712, -0.083, 0.795) from gaze (412, 220) on the depth picture  (~1.6 mm per pixel)
```

Errors are Python exceptions with a message saying what to change: `ValueError` (bad or impossible
arguments; nothing happened), `NotFound`, `RobotError`; the names are defined in the session. Catch
them in code when you have a plan B, or check a doubtful reach first with `dry_run=True`.
