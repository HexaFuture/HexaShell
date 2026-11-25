- Words often found nothing for these small parts (coin, gear, plug, straw, cup, phone; L0-L15):
  point at a pixel in the head picture, or outline the part with a box (`find(cam, box=...)`).
  'pen' was found by words 16/16, 'pen cap' 14/16 (not in V6; pen cap L0-L15, V1-V16).
- A depth reading into a hole or a hollow comes back too far (the lid's hole, the hollow shaft:
  1-4 cm; straw, bearing L0-L15): take a hole or a rim with `to_world(cam, px, z=...)`, the
  height measured on the surface beside it. Pick the hole itself in a zoomed head picture: from
  a wrist camera looking along the lid from afar a hole came out 1.9 cm off (straw V11 t15).
- `measure` at a pixel on a thin part can return the table (about 0.9 x 0.55 m) or the tray it
  stands in (coin L15 t24, t30; building block V1, V6, V11, V16): check the size. On a large flat
  part from straight above it can latch onto a small mark on it (the lid's 3 cm logo: phone L0
  t10, V1) or call it "flat or see-through" (phone L10 t16, L15 t8, V6, V16): box the whole
  part, or `to_world` its corners.
- Measure a part before an arm covers it: a head pixel under the robot's raised arm returns the
  arm (pen cap L0 t4; phone L0 t2, L15 t6), and `measure(thing)` again after the other hand
  lifted a part returned the held part (pen cap L15 t4). Then use a pixel in a wrist picture.
- Parts move before the grasp: an untouched plug moved 3-13 cm early on (ethernet V6, V11, V16),
  a lid 1.5-3 cm as the fingers came to it (phone V11 t16, V16 t16): measure it again from close
  by just before closing.
- Place by the part, not the hand, for a part slid onto a post: hand goal = target + (grasp point
  - part centre at the grasp). Aiming the grasp point at the target jammed (bearing L0, L10);
  keeping the offset slid on (bearing L5-L15, V; 11/11). A wide lid set down by its grasp offset
  landed 1.4 cm off on the rim; with the hand straight over the base's centre it went on (phone
  V16).
- Head-measured centres and headings were good to a few mm and degrees, heights not: from the
  head they read 0.6-3 cm more than from a wrist camera 10 cm above (building block, phone V1-V16).
  Parts with a few mm to spare landed on a wall (phone L10, L15 vs L5): before letting go, look
  straight down with the holding wrist camera and align by the picture (untested).
- A part pressed into a tight holder could not be grasped again: `Dropped` 8/8 (phone V1, V16).
  Get the place right before pressing.
- Insertions need millimetres: measure the held part's tip and the hole in the same picture, from
  close by, in the pose you will push from, and correct before the push (straw L0, building
  block L15, pen cap L15; pen cap V11, V16 went on at once, the unmeasured pushes 0/3 V1-V11).
  The same aim still left a cap short 3 of 5 times (pen cap L0, L10), and a reading within 1 mm
  still met the lid 5 times (straw L15): after a miss, look at the opening beside the tip in a
  zoomed picture and move toward its uncovered part (untested).
- Release once the held part's lower end is in the opening, the hand pointing down: 9 of 10
  (building block L5-L15, V; coin L0, L5). Level releases from above the mouth went in 3 of 8
  (building block L0, L10, V). Release only with the part seen over the opening: hidden by the
  fingers, a coin landed 4 cm off (coin V16 t17-t21).
- Steer the held part's tip, not the grasp point: it sits ahead of and below the grasp point,
  depending on how it is held; locate it in a wrist picture after the lift (straw L0, L15;
  pen cap L0-L15; charging V1-V16); it keeps that offset while the pitch stays. `to_world` on a
  held part may say "that pixel shows the robot" and still read right (charging V1, V16); on the
  network plug's front it read 7-9 mm high (ethernet V1, V6): check it against the pick height.
- A press with `contact=True` pushes what the target stands on: the straw's cup moved 6-12 cm
  over five or six presses (straw L5, L10), stopped or blocked moves shoved the bearing's fixture
  1.7-8.5 cm (bearing L10, V1, V6, V11, V16): measure the target again after each press.
- What a contact press says does not tell in from out. A straw press that met the lid ended
  "short of target" by 1.7-5.5 cm with the hand tipped 2-9 degrees, one that went in reached its
  target or stopped about 1 cm short (straw V1-V16, about 30 presses). Pushes into a socket ended
  "short of target" or "something stopped it" a few mm to 2 cm short whether in or out
  (charging V1; ethernet L0, L10, V1-V16); a gear's stop said nothing either way, only a wrist
  measure of its height did (bearing V6, V16); a part stuck while the fingers slid on said
  `Dropped` (bearing L15 t11). Look before lifting or waiting; a stuck part: grasp it again low.
- Moves made while a held part pressed on a face (a sideways shift, a height change, even a 5 cm
  straight back-off) lost it 5 times (ethernet L15, V6, V16): settle the aim before the push.
- Keep a holding hand's pitch steady: large pitch or yaw swings while holding dropped parts (pen
  cap, 4 times) or turned them in the pinch (straw 2/2); a level turn after a grasp from above
  tilted the star about 40 degrees (building block L0 t21); pushes at pitch 35-63 after a miss
  never put the cap on (pen cap V6, V11; 0/3). Tilted releases never went in (4/4).
- "holding" at a width far above the part's size was no hold: 7.6 cm on a 3 cm plug was the
  fingertips on the table (charging L0 t11); 5.3-7.0 cm on the network plug and 5.5 cm on the
  phone were lost at the lift (ethernet L5 t10, V6, V16; charging L10 t10). A close clearly wider
  than earlier grasps of the same part was a crooked grasp (building block L10 t15): open and
  grasp again.
- SUCCESS arrives as an error, "EpisodeOver: ... SUCCESS ...", in the same turn as the part went
  in or one or two turns later (ethernet L0 t16, L10 t25, V1 t21): keep holding until then (L10
  t15 opened too early).
- A part can be in place and still need time untouched, and no result says so: seated gears
  touched again were lost (bearing V1, V6). After placing, measure it once; if it is right, leave
  it alone.
