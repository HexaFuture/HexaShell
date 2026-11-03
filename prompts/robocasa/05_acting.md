# 5. Moving and handling things

How motions run:
- `goto` moves the grasp point in a straight line at up to 25 cm/s (or the `speed` asked for),
  turning the hand on the way. A straight line can sweep through things: near objects and fixtures
  keep moves short, and to cross over something, lift first (each leg a straight line). `home()`
  lifts first when low, then swings the arm's joints back to where they started (the hand's way
  curves): near an open door or drawer, first move the hand clear of it, or it may push it shut.
- The arm cannot work stretched out straight. A pose it would hold with the elbow nearly straight,
  or a straight line passing through such poses, is refused as out of reach (a long way out while
  the hand turns a lot swings the wrist out: go in two moves). A Move, a drive, a grip or a wait in
  which the elbow straightens that far anyway stops, and the arm goes back part of the way it came,
  which bends the elbow, with a note: drive the base closer. While the arm is stretched that far,
  only `home()` moves it.
- Pressing hard on something with the elbow open wider than a right angle (the hand, the wrist
  or the forearm against a fixture) would snap the elbow straight: such a motion, or a drive,
  stops at once and the arm pulls the hand 5 cm back toward the shoulder, with a note. Look what
  it touched; work close, with the elbow well bent.
- A pose that cannot be reached is not executed: the result says why (too far from the shoulder,
  or how close the hand would get). Drive the base closer or turn it toward the target, or ask for
  another pitch or height, rather than resending the same numbers. When the arm gets there from
  home but not from its pose now (a joint at the end of its travel, the wrist turned over), the
  result says so: `robot.home()` first, then the same move.
- `BLOCKED` means the hand is pressing on something - an object, the counter, a door. Look before
  going on: something may have been pushed or knocked over. A Move that stopped short "and was not
  getting closer" touches something too, or the arm is at the edge of what it can do there. With
  something in the hand, do not open the fingers there: it drops where it is. Back off or lift
  first, look, and come in another way.
- Low and far with a level hand is often out of reach; pointing down, or a pitch near 45 degrees,
  reaches lower. Far from the shoulder the hand is less exact: drive closer for fine work.
- Rough costs: a 10 cm move about 15-20 steps (at `speed=0.05` about 45-50), 30 cm about 35-45,
  turning the hand 90 degrees about 45, a gripper change 10-15, `home` 60-100; driving the base 1 m
  about 60-70 steps, turning it 90 degrees about 45. Every Move, Grip, Drive and Homing prints what
  it used and what is left. Go from one pose straight to the next nearby one: `home()` is for
  bringing the arm in before a drive and for clearing the cameras' view, not for between motions. It
  also turns the hand back to its start angles: a held bowl or cup tilts on the way.

The base:
- Stand where the arm reaches what you work on with room to spare: `dry_run=True` on the poses you
  will need tells you before you commit. Facing the work is better than reaching sideways.
- The arm keeps its pose relative to the base while it drives, and a held object comes along.
  Before a drive, bring the arm in (`robot.home()`, or lifted and close to the base) so it does not
  sweep into the counter or a door.
- Counters, open doors and walls stop the base (`BLOCKED`): drive in short legs around them and
  look after each. A small `forward`/`left` correction costs few steps. A Drive that says the base
  was held off its path touched something: its rear, the long part, is the usual culprit. In a
  corner between counters, first move away from the side counter, then turn.
- A turn of the base turns the yaw the arm keeps along with it.
- With the fingers on a door's handle, drive only round its hinge (`about=`, see Fixtures): a
  plain drive moves the base straight while the door turns about its hinge, and the grip comes
  off. A plain drive pulls only what moves the same way (a drawer straight out).

The gripper:
- An opening g (0..1) leaves a gap of about g × 8 cm between the pads.
- It is position-controlled: `'close'` squeezes whatever is between the pads and keeps squeezing
  while the arm and the base move. An opening wider than the object does not hold it. Carry things
  closed.
- `'light'` closes, then sets the fingers a little under what they stopped on: a light hold. Pick
  small soft food with it (vegetables, fruit, meat, sausages, a kebab, a bun, ice): squeezed hard,
  these slowly slide out along the fingers as they are carried, and a light hold keeps 2 in 3 of
  those in. Close the rest with `'close'` (cups, plates, bowls, bottles, utensils, wedges, bread
  loaves, lids, handles, heavy things): held lightly, 1 in 4 of them turn and slide out as the hand
  rises or pulls.
- After closing, `holding` and the width say what is in the fingers: holding with a width near the
  object's size is a grasp; `closed fully` is a miss (unless it is very thin - look at the wrist
  picture). A width well under its measured size across the fingers is a pinch on an edge or a
  corner, which slips on the way: open and grasp deeper before carrying it. A Move, Drive or
  Homing that says what it held slipped out means it fell on the way: look where it went; one that
  says the gap between the fingers changed means it turned or slid in them.
- A held object turns with the hand. Something long gripped from above lies across the fingers,
  along the hand's up-down direction: levelling the hand at roll 0 stands it upright; at roll 90 it
  stays lying, across the pointing direction.

Picking something up from above:
1. Go 8-10 cm above it pointing down with `yaw=thing.grip_yaw`, opening the gripper on the way
   (`grip=`, a gap at least 2 cm wider than the object), and measure it again from there.
2. Descend in a straight line to the grasp height (from the new measurement): for a thing under
   4 cm high, the height it stands on (`thing.on_z`; a move that stops short within 1 cm over it,
   on the surface, is fine); for a taller one, about half its height and at least 1.5 cm below its
   top. Closed within 0.5 cm of its top, a low thing slips out as the hand rises about half the
   time. A descent that stops short higher up has the fingers on the thing or beside it, not
   around it (its Move says so): a close there holds about 1 time in 5 at its top, 1 in 3 lower
   down. Rise 2 cm, look at the wrist picture and come down where both fingers clear it.
3. Close, check the Grip line, lift straight up 8-10 cm, and confirm with a picture that the
   object came up with the gripper.

Other grasps:
- Tall things (bottles, cans, cups) and things on a shelf or in a cabinet are often easier from the
  side: level pitch, the grasp point at about half the height, approaching along the pointing
  direction from 10-15 cm away, the gripper open.
- Long things: grip near the middle, across the long side (`grip_yaw` gives that).
- Hollow things (bowls, cups, pans) can be gripped by the wall or the rim: one finger inside, one
  outside.

Putting down and putting in:
- Carry high enough to clear everything on the way: lift, move across, then descend.
- Lower in a straight line until the object's bottom is about 1-2 cm above the surface it goes on,
  open, then lift straight up (or back out along the way in) before moving sideways. Then move the
  hand well away (section 1).
- Into a cabinet, a microwave, a fridge or onto a shelf: approach level through the opening, the
  held object below the top edge of the opening, set it down inside, open, back out straight.
- Into a sink, a bowl, a pan or a basket: release with the object over the middle of the opening.

Fixtures:
- A tall door with an upright hinge (a fridge's) opens fastest with the base: stand square in front
  of it, close the hand on the handle, then `robot.drive(turn=..., about=hinge)` (+ for a hinge on
  the right, - on the left; `dry_run=True` says which way the hand would go: back toward the base
  opens it): base, hand and door go round the hinge together, 85 degrees in one call. The hinge is
  at the door's outer edge (away from the handle), a few cm behind its front; a few cm off only
  makes the door lag or lead. Whatever stands beside the door (a wall, a counter,
  an open door) can stop the base part way (`BLOCKED`). Then let go, back the hand out and home,
  and before going to the next door back the base out square, clear of the open door: the hand
  sticks out ahead of the base and can push an open door shut.
- Doors with a hinge turn about it. With the grasp point on the handle (fingers closed on it, or
  hooked behind it), `robot.arc(hinge, angle, axis='z')` moves the hand along the door's circle
  about a vertical hinge line through `hinge`; for a door hinged at the bottom the axis is level,
  along the hinge. Check the direction first: `dry_run=True` prints where the arc ends. Swing in
  steps of 20-40 degrees and look after each; the grip slides on a handle, so re-grip when the hand
  lags behind the door, and check `holding` after closing on it before swinging. A door can also be pushed: closed gripper on its face near the free edge,
  moving along its circle.
- Drawers and sliding racks move straight: grip the handle or hook behind the front, then
  `robot.goto(by=...)` along the direction the front faces. To close, push the front back in.
- Knobs and dials turn about their own axis: point the fingers along that axis onto the knob,
  close on it, then change `roll` (`robot.goto(roll=...)` keeps the position). Small steps.
- Buttons and switches: closed gripper, straight in until 1-2 cm past where it touches, then
  straight back out.
- Levers and handles that tip (a faucet, a lid, a mixer head): push or lift them along the way
  they move, with the closed gripper, in small steps, looking after each.
