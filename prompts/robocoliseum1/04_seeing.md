# 4. Seeing and measuring

Every camera gives a colour and a depth picture. An outline's pixels, with their depth, are points
on the surface the camera sees, so a measurement needs no guessing of heights - but only the
surfaces the cameras see are measured.

A good order of work:
1. **Look.** `robot.look('all', tile=True)` puts the three cameras side by side in one picture.
   `robot.look('head', grid=True)` draws a pixel grid. To read a pixel you will point at, enlarge
   the region around it: `robot.look('head', crop=(x0, y0, x1, y1), zoom=3)`; the labels on an
   enlarged picture are still full-picture pixels, so pass what you read as it is.
2. **Find.** `robot.find('head', 'red block')` returns every instance, best first. The words find by
   appearance: "left", "second" or "nearest" do not pick an instance - choose among the masks by
   their centroids, or point at the one you mean: `robot.find('head', at=(412, 233))`. Points
   (`labels` 1 on it, 0 not on it) or a `box` separate touching things. Check outlines with
   `robot.look('head', masks=[m1, m2])` before acting on them.
3. **Measure.** `robot.measure('red block')` outlines it in every camera that sees it and merges
   their points; `robot.measure(mask)` (or `('head', (x, y))`) starts from one outline and finds the
   same object in the other cameras. Words measure the best match and list the other matches with
   their positions: measure the one you mean by a Mask or a pixel. Measure every object the task
   involves before either arm moves: it is free, and nothing is covered yet.
4. **Measure again from close by** when it matters: before grasping something whose far side the
   cameras cannot see, and whenever the Thing says it was seen by one camera only. Seen from one
   side, the far half of an object is hidden, so its centre comes out toward the camera by up to
   half its depth and its footprint is too short. The pick-up's first pose (section 5) holds a wrist
   camera over the object: `robot.measure(thing)` there sees its top and outline. For something
   tall seen from the side, point the hand at it level from 20-30 cm away instead. Small low objects
   seen by two or three cameras at the start usually need no second measurement.
5. **Check before committing** a motion near objects: draw the target -
   `robot.look('head', marks=[('world', xyz, 'target')])` - or ask `robot.to_pixel(xyz, cam)`.

Good to know:
- Everything reads the CURRENT pictures. After anything moved, positions from before are stale:
  `robot.measure(thing)` measures the object again near where it was.
- The head camera sees the work area from above and in front of the torso; when the waist turns,
  it turns too. At the start the wrist cameras look forward and inward over the table.
- The robot's own pixels are left out of measurements. An object under or behind an arm is partly
  hidden from the head camera: lift the arm and move it aside before measuring there. A held object
  measures badly; work out where it is from the gripper. What a gripper holds hides what lies under
  it from that arm's wrist camera: measure where a held object will go before the grasp, or with
  the other arm's wrist camera.
- Read the ⚠ and notes of every Thing before using it: few depth points, one view, other matches.
  Shiny, dark or transparent surfaces can give holes in the depth picture.
- A mask is the best match for the words, not proof of identity: a low score, an odd place or a
  size that does not fit means it found something else. Scenes often hold several objects of a
  kind (blocks of several colours, numbers or shapes; several fruits, bottles or packages): check
  with `robot.look(cam, masks=[...])` that the outline is the one the instruction means. Where the
  instruction picks an object by a relation ("the one on the left", "the one behind the cup",
  "the biggest"), work it out from the measured positions and sizes. Left is +y, the left side of
  the head picture; behind / farther is +x.
- `robot.to_world(cam, (x, y))` gives the surface point under a pixel from the depth picture: handy
  for the edge of something, a handle, a spot on the table or inside a container. `mm per pixel` is
  how far a one-pixel slip moves the point; near an edge the depth may belong to what lies behind.
  With `z=0.80` it is where the pixel's ray meets the plane z = 0.80 (use it when you know the
  height). With `pair=(cam2, (x2, y2))` - the same point seen in a second camera - it triangulates
  and reports how far apart the two rays pass: more than about 5 mm means the two pixels are not
  the same point.
- Some episodes are harder to see: blurred, noisy, smudged, dark or over-bright pictures, unusual
  lighting or backgrounds, a camera mounted a little differently, an instruction worded in another
  way. Check what a mask found with a picture, compare two cameras where they disagree, and trust a
  depth measurement over a guess from the colour picture. Now and then a camera's picture comes
  black (that camera missed that moment): looking and measuring then hold a step or two for a new
  one, and a camera whose picture stays black is left out, so look again or use another camera.
- `robot.image(cam)` gives the raw picture for your own analysis (colours, cv2).
