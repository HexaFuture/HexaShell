# 4. Seeing and measuring

There are no depth pictures: 3-D positions come from the colour pictures and the camera models. An
object is placed where its outlines in the cameras that see it meet, a point on a surface where the
same patch of it shows in both pictures - so a measurement wants both cameras to see the object.

A good order of work:
1. **Look.** `robot.look('all', tile=True)` puts both cameras side by side in one picture.
   `robot.look('agentview', grid=True)` draws a pixel grid. To read a pixel you will point at,
   enlarge the region around it: `robot.look('agentview', crop=(x0, y0, x1, y1), zoom=3)`; the
   labels on an enlarged picture are still full-picture pixels, so pass what you read as it is.
2. **Find.** `robot.find('agentview', 'black bowl')` returns every instance, best first. The words
   find by appearance: "left", "second" or "nearest" do not pick an instance - choose among the
   masks by their centroids, or point at the one you mean: `robot.find('agentview', at=(312, 233))`.
   Points (`labels` 1 on it, 0 not on it) or a `box` separate touching things. Check outlines with
   `robot.look('agentview', masks=[m1, m2])` before acting on them. A click can outline only part of
   an object (a handle, a lid): `robot.find(cam, at=(x, y))` lists the outlines offered there -
   measure the Mask that covers all of it.
3. **Measure.** `robot.measure(mask)` (or words, or `('agentview', (x, y))`) finds the same object
   in the other camera and intersects what they see. With 1 view the height - and so the position
   along the camera's line of sight - is a guess, and the warning says where to hold the wrist
   camera. Words measure the best match and list the other matches as Masks: measure the Mask you
   mean. Before the arm moves, measure each object the task involves: nothing is covered yet, and
   one later look from above can serve several of them.
4. **Measure again from above** when it matters: before a grasp, before placing on or into an
   object, and whenever the Thing warns of 1 view. Hold the wrist camera over the object and
   `robot.measure(thing)`: its view, with agentview's, gives the top and the footprint. A Thing
   that needs the view gives the exact command in its warning - a pose the arm reaches, with the
   object in the wrist picture and as little of it as possible hidden by the arm from agentview;
   for a big object (a basket, a tray) it is farther, so that all of its top fits in the wrist
   picture. Otherwise: the hand pointing straight down, its grasp point 10-15 cm above the object's
   top (the wrist camera rides about 10 cm behind the grasp point and 5 cm to one side, so it is
   then 20-25 cm above it, looking down at it); if the arm then hides the object from agentview, the
   Thing says so. What the gripper holds hides what lies under it from the wrist camera: measure
   where a held object will go before the grasp. A Thing measured from above needs no second look
   unless something moved it, and an object you set down lies about where you released it (shifted
   by where it hung in the gripper): aim there and check with a picture.
5. **Check before committing** a motion near objects and fixtures: draw the target -
   `robot.look('wrist', marks=[('world', xyz, 'target')])` - or ask `robot.to_pixel(xyz, cam)`.

Fixtures (drawers, doors, handles, knobs, racks, the stove) are measured the same way: name the
part (`'drawer handle'`, `'stove knob'`) or point at it. For one point on a fixture - a handle, the
edge of a door - give `robot.to_world` the same point in the other picture too (`pair=`, below).

Good to know:
- Everything reads the CURRENT pictures. After anything moved, positions from before are stale:
  `robot.measure(thing)` re-finds the object near where it was and measures it again.
- agentview sees the whole workspace from about a metre away: small things are a few dozen pixels
  wide there, and their sides facing the robot are hidden. The wrist camera gives the close view.
- The robot's own pixels, where the robot's model of itself puts them, are left out of
  measurements. The arm over the table hides from agentview what is behind and under it: lift it
  and move it aside (or `robot.home()`) before measuring there. A held object measures badly, and
  the Thing says so; work out where it is from the gripper.
- Read the ⚠ of every Thing before using it. `1 view: height guessed` or `height unknown`: go to
  the pose it gives, then measure again. `partly hidden ... move the arm out of the way`: the arm
  covers part of it in a picture. `... only part of it: it may stand 150 mm tall at (x, y) ... not
  lie low`: a standing object may have been read as a low one - measure it again from the pose it
  gives before grasping it or placing on it. `the cameras agree only roughly`: the outlines may
  belong to different objects - check with `robot.look('all', masks=[thing], tile=True)`.
- `on` / `on_z` say what an object rests on: `table` (or `floor`), `Thing#2` (stacked), `held by
  the gripper`. Hollow objects (bowls, mugs, baskets) give their rim as `top`; `round` marks a round
  footprint.
- A mask is the best match for the words, not proof of identity: a low score, an odd place or a
  size that does not fit means it found something else. Scenes often hold look-alike things (two
  bowls, two mugs, several boxes): check with `robot.look(cam, masks=[...])` that the outline is
  the one the instruction means. Where the instruction tells them apart by a relation ("the bowl
  next to the plate", "the left plate"), work the relation out from the measured positions
  (section 2 says which way left and right are meant).
- `robot.to_world(cam, (x, y))` gives the point on the surface under a pixel: on the table top where
  the picture shows bare table, else matched between the two pictures where the surface has
  texture, else on the measured object outlined there - the Spot says which, warns when the last two
  disagree and lists the other under `also:`. On an object it is where the pixel's ray enters the
  object's measured box, often 1-5 cm off along the ray: for where an object is, use
  `robot.measure`; for one point on it, `pair=`. A thin flat object can pass for bare table: for a
  point on one, give `z=` the table height plus its thickness. `mm per pixel` is how far a one-pixel
  slip moves the point. With `z=0.05` it is where the pixel's ray meets the plane z = 0.05 (use it
  when you know the height: a rim, the table top from the task card). With `pair=(cam2, (x2, y2))`
  - the same point seen in a second camera - it triangulates and reports how far apart the two rays
  pass: more than about 5 mm means the two pixels are not the same point.
- `robot.image(cam)` gives the raw picture for your own analysis (colours, cv2).

When the robot's models may be off (section 1): measurements turn pixels into the arm frame through
the camera models, and motions go by the robot's readings of its arm. Check both once, early, where
the hand is in view: `robot.look(cams, marks=[('world', robot.state().at, 'hand')])` should put the
mark between the finger pads in each picture, and `robot.to_pixel` of a measured object should land
on it in both pictures. When they disagree by more than a centimetre or two, trust what the
pictures show over the numbers: work out the offset from where the hand and the objects appear,
correct your targets by it, and check again from close by with the wrist camera before a grasp or a
placement.
