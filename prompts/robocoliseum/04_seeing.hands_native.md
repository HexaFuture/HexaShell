# 4. Seeing and placing points

In these tasks the cameras give colour pictures only, no depth pictures: a pixel gives a direction
from its camera (a ray), not how far away the surface is. A point in the world comes from a pixel
and one more fact - a height you know, or the same point seen in a second picture - so
`robot.measure()` does not work here, and `robot.to_world` with `z=` or `pair=` does.

A good order of work:
1. **Look.** `robot.look('all', tile=True)` puts the three cameras side by side in one picture.
   `robot.look('head', grid=True)` draws a pixel grid. To read a pixel you will point at, enlarge
   the region around it: `robot.look('head', crop=(x0, y0, x1, y1), zoom=3)`; the labels on an
   enlarged picture are still full-picture pixels, so pass what you read as it is.
2. **Find.** `robot.find('head', 'red block')` returns every instance, best first. The words find by
   appearance: "left", "second" or "nearest" do not pick an instance - choose among the masks by
   their centroids, or point at the one you mean: `robot.find('head', at=(412, 233))`. Points
   (`labels` 1 on it, 0 not on it) or a `box` separate touching things. Check outlines with
   `robot.look('head', masks=[m1, m2])` before acting on them. A Mask's centroid and box are pixels.
3. **Place.** Turn the pixels you need into points:
   - `robot.to_world(cam, (x, y), z=h)` is where the pixel's ray meets the plane z = h: the point
     itself when you know its height. The table's top is such a height: read it once from a mark on
     the table seen in two pictures (below), or from a hand's own stop on it - a pinch pointing
     straight down, lowered with `contact=True` until it stops: the Move's height is then about the
     table's (a motion or two, but sure). Not the point shape's stop: it read 3-10 cm low or was
     pushed aside. A thing's top is the table's height plus the thing's own height; something thin
     lying flat (a cloth, a card) is at the table's height.
   - `robot.to_world(cam, (x, y), pair=(cam2, (x2, y2)))` - the same point seen in a second camera
     (a wrist picture and the head's, or both wrists') - is where the two rays meet; it reports how
     far apart they pass: more than about 5 mm means the two pixels are not the same point. Pick a
     point both pictures show sharply - a corner, the end of an edge, a mark -, not the middle of
     something that looks different from each camera.
   Work out every point the task needs before either arm moves: it is free, and nothing is covered
   yet.
4. **Look again from close by** when it matters: a wrist camera held over or before an object sees
   its outline and top far better than the head camera does, and its pixel with the head's (`pair=`)
   or with a known height (`z=`) gives the point again. The pick-up's first pose (section 5) holds a
   wrist camera close to the object for this.
5. **Check before committing** a motion near objects: draw the target -
   `robot.look('head', marks=[('world', xyz, 'target')])` - or ask `robot.to_pixel(xyz, cam)`.

Good to know:
- Everything reads the LATEST pictures: those the benchmark sent at the end of the last motion.
  Whatever moves on its own (something rolling or falling, a cloth settling) has moved on since;
  `robot.wait(1)` (a motion of one step) brings new ones. After anything moved, points from before
  are stale: find the object again and work out its point anew.
- The head camera sees the work area from above and in front of the robot. At the start the wrist
  cameras look forward or inward over the table.
- An object under or behind an arm is partly hidden from the head camera: lift the arm and move it
  aside before looking there. What a hand holds hides what lies under it from that arm's wrist
  camera: work out where a held object will go before the grasp, or with the other arm's wrist
  camera.
- A point from `z=` is only as good as the height you give: a height that is off moves the point
  along the pixel's ray - little for a ray that looks steeply down (the head camera's), more for
  one that looks across the table (a wrist camera looking forward). `mm per pixel` is how far a
  one-pixel slip moves the point.
- A mask is the best match for the words, not proof of identity: a low score, an odd place or a
  size that does not fit means it found something else. Scenes often hold several objects of a
  kind: check with `robot.look(cam, masks=[...])` that the outline is the one the instruction
  means. Where the instruction picks an object by a relation ("the one on the left", "the one
  behind the cup", "the biggest"), work it out from their points and outlines. Left is +y, the
  left side of the head picture; behind / farther is +x, higher up the head picture.
- Some episodes are harder to see: blurred, noisy, smudged, dark or over-bright pictures, unusual
  lighting or backgrounds, an instruction worded in another way. Check what a mask found with a
  picture, and compare two cameras where they disagree. Now and then a camera's picture comes
  black (that camera missed that moment): looking then holds a step or two for a new one (each
  hold is a motion), and a camera whose picture stays black is left out, so look again or use
  another camera.
- `robot.image(cam)` gives the raw picture for your own analysis (colours, cv2).
