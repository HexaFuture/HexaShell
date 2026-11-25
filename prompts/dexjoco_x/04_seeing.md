# 4. Seeing and measuring

Every camera gives a colour and a depth picture. An outline's pixels, with their depth, are points
on the surface the camera sees, so a measurement needs no guessing of heights - but only the
surfaces the cameras see are measured.

A good order of work:
1. **Look.** `robot.look('all', tile=True)` puts the cameras side by side in one picture.
   `robot.look('scene', grid=True)` draws a pixel grid. To read a pixel you will point at, enlarge
   the region around it: `robot.look('scene', crop=(x0, y0, x1, y1), zoom=3)`; the labels on an
   enlarged picture are still full-picture pixels, so pass what you read as it is.
2. **Find.** `robot.find('scene', 'red box')` returns every instance, best first. The words find by
   appearance, and a name `find` does not know finds nothing or something else: say what it looks
   like instead (the hammer of the hammer task is found as 'tool' or 'mallet', its parts as 'grey
   metal hammer head' and 'yellow wooden handle'), or point at it:
   `robot.find('right_wrist', at=(240, 270))`. "left", "second" or "nearest" do not pick an
   instance: choose among the masks by their centroids. Points (`labels` 1 on it, 0 not on it) or a
   `box` separate touching things. Check outlines with `robot.look(cam, masks=[m1, m2])` before
   acting on them.
3. **Measure.** `robot.measure('red box')` outlines it in every camera that sees it and merges their
   points; `robot.measure(mask)` (or `(cam, (x, y))`) starts from one outline and finds the same
   object in the other cameras. Words measure the best match and list the other matches with their
   positions: measure the one you mean by a Mask or a pixel.
4. **Look from close by** before a grasp, a press or anything that needs a centimetre: the scene
   camera stands 1.5-2 m away, and its view hides the sides facing away from it. Bring a wrist
   camera over the object: `robot.hand('right', (x, y, z + 0.45), point='camera', look_at=(x, y,
   z))` looks straight down on it from 45 cm (the picture is then about 50 cm across, 1 mm per
   pixel); `look_at` from the side or from behind at 45 degrees shows a side face. Then look,
   find and `robot.measure(thing)` again. Such a view costs 50-80 steps: plan one per object that
   needs it, and keep the hand there while you work out the next steps.
5. **Check before committing** a motion near objects: draw the target -
   `robot.look('right_wrist', marks=[('world', xyz, 'target')])` - or ask `robot.to_pixel(xyz, cam)`.

A wrist camera's view: the camera sits 23.5 cm behind the palm and looks 30 degrees off the fingers'
line, so the fingers stick out toward what it looks at: looking straight down from 40 cm, the
fingertips hang about 8 cm above the height of the point looked at and 18 cm from it, toward the
top of the picture (from 45 cm, 13 cm above). Keep looking-at distances of 40-50 cm over a table, and more when tall things
stand near the point. The note of a `look_at` move says which way the top of the picture points in
the scene: read directions in the picture from it, not from habit.

Good to know:
- Everything reads the CURRENT pictures. After anything moved (a hand, a camera, an object),
  positions from before may be stale: `robot.measure(thing)` measures the object again near where it
  was.
- A Thing's `.center` is the middle of the surface the cameras see. Seen from above, that is the
  middle of its upper surface: the axis of a bar or a handle lies about half its thickness lower, the
  middle of a box half its height lower (`.top` and `.on_z` give the heights). For a grasp at the
  middle of a thing, work out its middle from what you know of its shape, not from `.center` alone.
- The robot's own pixels (arms, hands) are left out of measurements. An object behind or under a hand
  or an arm is partly hidden: move the hand aside (or `robot.home(side)`) before measuring there. A
  held object measures badly; work out where it is from the hand (it moves with the grasp point).
- Read the ⚠ and notes of every Thing before using it: few depth points, one view, other matches.
- A mask is the best match for the words, not proof of identity: a low score, an odd place or a size
  that does not fit means it found something else.
- `robot.to_world(cam, (x, y))`: `mm per pixel` is how far a one-pixel slip moves the point; near an
  edge the depth may belong to what lies behind. With `z=0.95` it is where the pixel's ray meets the
  plane z = 0.95 (use it when you know the height, a table's top for one).
- `robot.image(cam)` and `robot.depth(cam)` give the raw pictures for your own analysis (colours,
  cv2, depth edges); `robot.camera(cam)` gives the camera model to turn pixels and depths into points
  yourself.
