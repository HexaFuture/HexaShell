# 4. Seeing and measuring

Every camera gives a colour and a depth picture. An outline's pixels, with their depth, are points
on the surface the camera sees, so a measurement needs no guessing of heights - but only the
surfaces the cameras see are measured.

A good order of work:
1. **Look.** `robot.look('all', tile=True)` puts the three cameras side by side in one picture.
   `robot.look('left', grid=True)` draws a pixel grid. To read a pixel you will point at, enlarge
   the region around it: `robot.look('left', crop=(x0, y0, x1, y1), zoom=3)`; the labels on an
   enlarged picture are still full-picture pixels, so pass what you read as it is.
2. **Find.** `robot.find('left', 'red mug')` returns every instance, best first. The words find by
   appearance: "left", "second" or "nearest" do not pick an instance - choose among the masks by
   their centroids, or point at the one you mean: `robot.find('left', at=(412, 233))`. Points
   (`labels` 1 on it, 0 not on it) or a `box` separate touching things. A name that finds nothing
   or something else may be one `find` does not know: say what it looks like instead
   (`'brown oval food'`, `'clear plastic box'`) or point at it. Check outlines with
   `robot.look('left', masks=[m1, m2])` before acting on them.
3. **Measure.** `robot.measure('red mug')` outlines it in every camera that sees it and merges
   their points; `robot.measure(mask)` (or `('left', (x, y))`) starts from one outline and finds
   the same object in the other cameras. Words measure the best match and list the other matches
   with their positions: measure the one you mean by a Mask or a pixel.
4. **Measure again from close by** when it matters: before a grasp, and whenever the Thing says it
   was seen by one camera only. Seen from one side, the far half of an object is hidden, so its
   centre comes out toward the camera by up to half its depth and its footprint is too short. Bring
   the wrist camera 25-35 cm above it, pointing down (or in front of it, pointing at it), and
   `robot.measure(thing)`: the wrist view shows its top and outline.
5. **Check before committing** a motion near objects and fixtures: draw the target -
   `robot.look('wrist', marks=[('world', xyz, 'target')])` - or ask `robot.to_pixel(xyz, cam)`.

Fixtures (doors, handles, drawers, knobs, buttons, the counter) are measured the same way: name the
part (`'fridge door handle'`, `'stove knob'`) or point at it. `robot.to_world(cam, (x, y))` gives
the surface point at a pixel directly from the depth picture: handy for a handle, the edge of a
door, a spot on the counter or inside a cabinet.

Good to know:
- Everything reads the CURRENT pictures. After anything moved, positions from before are stale:
  `robot.measure(thing)` measures the object again near where it was.
- The left and right cameras ride on the base: after a drive they show the scene from the new
  place, and what was in view may be out of it. Positions stay in the start frame, so a Thing
  measured before the drive keeps its coordinates as long as nothing moved it.
- The robot's own pixels are left out of measurements. An object behind or under the arm is
  partly hidden: lift the arm or move it aside (or `robot.home()`) before measuring there. A held
  object measures badly; work out where it is from the gripper.
- Read the ⚠ and notes of every Thing before using it: few depth points, one view, other matches.
- See-through things (clear plastic boxes and lids, glass) do not show in the depth picture: their
  pixels carry the depth of what lies behind or under them, so they measure flat, at the height of
  the surface they stand on, and the Thing warns of it. Judge their walls and rim from the colour
  pictures: `robot.to_world(cam, (x, y), pair=(cam2, (x2, y2)))` triangulates a corner seen by two
  cameras.
- A mask is the best match for the words, not proof of identity: a low score, an odd place or a
  size that does not fit means it found something else. Kitchens hold many look-alike things
  (several cabinets, doors, drawers, bowls): check with `robot.look(cam, masks=[...])` that the
  outline is the one the instruction means.
- `robot.to_world(cam, (x, y))`: `mm per pixel` is how far a one-pixel slip moves the point; near
  an edge the depth may belong to what lies behind. With `z=0.95` it is where the pixel's ray
  meets the plane z = 0.95 (use it when you know the height). With `pair=(cam2, (x2, y2))` - the
  same point seen in a second camera - it triangulates and reports how far apart the two rays pass:
  more than about 5 mm means the two pixels are not the same point.
- `robot.image(cam)` and `robot.depth(cam)` give the raw pictures for your own analysis (colours,
  cv2, depth edges).
