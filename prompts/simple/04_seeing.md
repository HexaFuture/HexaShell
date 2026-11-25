# 4. Seeing and measuring

The gaze camera gives a colour and a depth picture. An outline's pixels, with their depth, are points
on the surface the camera sees, so a measurement needs no guessing of heights - but only the surfaces
the camera sees are measured, and it sees only where it points.

A good order of work:
1. **Point the camera.** `robot.gaze(at=xyz)` turns it toward a point you know or guess (the table's
   middle, a measured Thing); `robot.gaze(yaw, pitch)` by angles. Nearby things on a table are seen
   best looking 40-60 degrees down; to look across the room, look less far down and turn the yaw.
   The camera turns with the torso: after a lean or a torso turn, look again.
2. **Look.** `robot.look('gaze')`; `grid=True` draws a pixel grid. To read a pixel you will point at,
   enlarge the region around it: `robot.look('gaze', crop=(x0, y0, x1, y1), zoom=3)`; the labels on an
   enlarged picture are still full-picture pixels, so pass what you read as it is.
3. **Find.** `robot.find('gaze', 'cracker box')` returns every instance, best first. The words find by
   appearance: "left", "second" or "nearest" do not pick an instance - choose among the masks by their
   centroids, or point at the one you mean: `robot.find('gaze', at=(412, 233))`. A name that finds
   nothing or something else may be one `find` does not know: say what it looks like instead
   (`'red rectangular box'`, `'grey basket'`) or point at it. Check outlines with
   `robot.look('gaze', masks=[m1, m2])` before acting on them.
4. **Measure.** `robot.measure('cracker box')` or `robot.measure(mask)`. Seen from one side, the far
   half of an object is hidden: its centre comes out toward the camera by up to half its depth, and
   its size along the line of sight is too short. Known sizes help: a box's three sides are fixed, so
   work out its middle from the faces you see.
5. **Measure again from close by** before a grasp: walk up, point the camera at the object and
   `robot.measure(thing)`.
6. **Check before committing** a motion near objects: draw the target -
   `robot.look('gaze', marks=[('world', xyz, 'target')])` - or ask `robot.to_pixel(xyz, 'gaze')`.

Good to know:
- Everything reads the CURRENT picture. After anything moved (the robot, the camera, an object),
  positions from before may be stale: `robot.measure(thing)` measures the object again near where it
  was.
- Positions are in the start frame and stay right while the robot walks: a Thing measured before a
  walk keeps its coordinates as long as nothing moved it.
- The robot's own pixels (hands, arms) are left out of measurements. An object behind or under a hand
  is partly hidden: move the hand aside before measuring there. A held object measures badly; work out
  where it is from the palm (it moves with the hand that holds it).
- Read the ⚠ and notes of every Thing before using it: few depth points, other matches.
- A mask is the best match for the words, not proof of identity: a low score, an odd place or a size
  that does not fit means it found something else.
- `robot.to_world('gaze', (x, y))`: `mm per pixel` is how far a one-pixel slip moves the point; near an
  edge the depth may belong to what lies behind. With `z=0.75` it is where the pixel's ray meets the
  plane z = 0.75 (use it when you know the height, a table's top for one).
- `robot.image('gaze')` and `robot.depth('gaze')` give the raw pictures for your own analysis (colours,
  cv2, depth edges); `robot.camera('gaze')` gives the camera model to turn pixels and depths into
  points yourself.
