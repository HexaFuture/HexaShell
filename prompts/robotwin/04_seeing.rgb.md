# 4. Seeing and measuring

There are no depth pictures: 3-D positions come from the colour pictures and the camera models. An
object is placed where its outlines in the cameras that see it meet, a point on a surface where the
same patch of it shows in two pictures - so a measurement wants two cameras that see the object,
usually the head camera and a wrist camera.

A good order of work:
1. **Look.** `robot.look('all', tile=True)` puts the three cameras side by side in one picture.
   `robot.look('head', grid=True)` draws a pixel grid. The pictures are small (320 x 240): to read
   a pixel you will point at, enlarge the region around it:
   `robot.look('head', crop=(x0, y0, x1, y1), zoom=3)`; the labels on an enlarged picture are still
   full-picture pixels, so pass what you read as it is.
2. **Find.** `robot.find('head', 'red cup')` returns every instance, best first. The words find by
   appearance: "left", "second" or "nearest" do not pick an instance - choose among the masks by
   their centroids, or point at the one you mean: `robot.find('head', at=(112, 133))`. Points
   (`labels` 1 on it, 0 not on it) or a `box` separate touching things. Check outlines with
   `robot.look('head', masks=[m1, m2])` before acting on them. A click can outline only part of a
   long object (a handle): `robot.find(cam, at=(x, y))` lists the outlines offered there - measure
   the Mask that covers all of it.
3. **Measure.** `robot.measure(mask)` (or words, or `('head', (x, y))`) finds the same object in
   the other cameras and intersects what they see. With 1 view the height - and so the position
   along the camera's line of sight - is a guess, and the warning says where to hold a wrist
   camera. Words measure the best match and list the other matches as Masks: measure the Mask you
   mean. Before either arm moves, measure each object the task involves: nothing is covered yet,
   and one later look from above can serve several of them.
4. **Measure again from above** when it matters: before a grasp, before placing on or into an
   object, and whenever the Thing warns of 1 view. Hold a wrist camera over the object and
   `robot.measure(thing)`: its view, with the head camera's, gives the top and the footprint. A
   Thing that needs the view gives the exact command in its warning - a pose the arm reaches, with
   the object in the wrist picture and as little of it as possible hidden by the arm from the head
   camera; for a big object (a box, a tray) it is farther, so that all of its top fits in the wrist
   picture. Otherwise: the hand pointing straight down, its grasp point 10-15 cm above the object's
   top (the wrist camera is then about 20 cm above it, looking down at it); if the arm then hides
   the object from the head camera, the Thing says so. What a gripper holds hides what lies under it
   from that arm's own wrist camera: measure where a held object will go before the grasp, or with
   the other wrist camera. A Thing measured from above needs no second look unless something moved
   it, and an object you set down lies about where you released it (shifted by where it hung in the
   gripper): aim there and check with a picture.
5. **Check before committing** a motion near objects: draw the target -
   `robot.look('head', marks=[('world', xyz, 'target')])` - or ask `robot.to_pixel(xyz, cam)`.

Good to know:
- Everything reads the CURRENT pictures. After anything moved, positions from before are stale:
  `robot.measure(thing)` re-finds the object near where it was and measures it again.
- The head camera sees the whole table from above and behind the arms: small things are only 15-40
  pixels wide there, and their sides facing away from the robot are hidden. The wrist cameras give
  the close views. At the start they look forward along the table, over what is near the robot.
- The robot's own pixels are left out of measurements. An arm over the table hides from the head
  camera what is behind and under it: lift it and move it aside before measuring there. A held
  object measures badly, and the Thing says so; work out where it is from the gripper.
- Read the ⚠ of every Thing before using it. `1 view: height guessed` or `height unknown`: go to
  the pose it gives, then measure again. `partly hidden ... move the left arm out of the way`: the
  arm covers part of it in a picture. `... only part of it: it may stand 150 mm tall at (x, y) ...
  not lie low`: a standing object may have been read as a low one - measure it again from the pose
  it gives before grasping it or placing on it. `the cameras agree only roughly`: the outlines may
  belong to different objects - check with `robot.look('all', masks=[thing], tile=True)`.
- `on` / `on_z` say what an object rests on: `table`, `Thing#2` (stacked), `held by left`. Hollow
  objects (bowls, cups) give their rim as `top`; `round` marks a round footprint.
- A mask is the best match for the words, not proof of identity: a low score, an odd place or a
  size that does not fit means it found something else. Scenes often hold several objects of a
  kind (blocks of different colours, two bottles, several cups): check with
  `robot.look(cam, masks=[...])` that the outline is the one the instruction means. Left and right
  in the instructions are the robot's: left is -x, the left side of the head picture; where the
  instruction picks an object by a relation ("the bottle on the left"), work it out from the
  measured positions.
- `robot.to_world(cam, (x, y))` gives the point on the surface under a pixel: on the table top where
  the picture shows bare table, else matched between two cameras' pictures where the surface has
  texture, else on the measured object outlined there - the Spot says which, warns when the last two
  disagree and lists the other under `also:`. On an object it is where the pixel's ray enters the
  object's measured box, often 1-5 cm off along the ray: for where an object is, use
  `robot.measure`; for one point on it, `pair=`. A thin flat object can pass for bare table: for a
  point on one, give `z=` the table height plus its thickness. `mm per pixel` is how far a one-pixel
  slip moves the point. With `z=0.80` it is where the pixel's ray meets the plane z = 0.80 (use it
  when you know the height: a rim, a slot, the table top from the task card). With
  `pair=(cam2, (x2, y2))` - the same point seen in a second camera - it triangulates and reports how
  far apart the two rays pass: more than about 5 mm means the two pixels are not the same point.
- `robot.image(cam)` gives the raw picture for your own analysis (colours, cv2).
