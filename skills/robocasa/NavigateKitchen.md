# NavigateKitchen
## Judging
- Done when the base stands close to the goal pose and faces about its way (both within a short
  step); it ends the moment that holds, even mid-drive (8 eps). Only the base counts.
- The goal squarely faces the floor fixture under or around the named one (counter, stove, tall
  cabinet, fridge), opposite the NAMED fixture's centre, about as close to that fixture's front as
  you start to yours (a little farther for a fridge or tall tower).
- "shelves" = open wall shelving above a counter; "stove" = a range or a cooktop; a "range hood"
  or a microwave hung over the stove: the stove below it. It is the only one of its kind, away
  from the start counter; goals lay up to ~4 m away (prior tool).
## Flow
1. Read the start pictures (no steps). `robot.drive(forward=-0.5)` (35 steps), then turn in place,
   `robot.drive(turn=90)` or -90, toward more kitchen (untested), and look; more 90-degree turns
   there were clear (prior tool, 27 of 50 backed first; 2 backings BLOCKED after 17-29 steps).
2. Name it by its look before measuring: a toaster oven was taken for the microwave and for the
   toaster (2 eps). Toaster oven: a glass door, round knobs down one side (2 eps). Microwave: a
   dark window door, a keypad strip, no knobs; built into a tall cabinet (1 ep) or hung over the
   stove, only its underside at the top of the picture (prior tool). Toaster: small, slots on
   top, no door; small ones hide in corners past a fridge or tall cabinet (prior tool).
3. `robot.measure` it in two views; `robot.to_world` two points on the front edge of the floor
   fixture under it: the front line and the heading facing it (8 eps).
4. Goal = opposite the MEASURED centre of the named fixture, 0.25 m out from that front line
   (0.35 m for a fridge, 1 ep), heading facing the line; success fired on the way (7 eps).
5. One `robot.drive(to=(x, y), heading=h)` when no counter or island lies on the straight line
   (prior tool, 46/50; 1 ep); else a straight leg down the open aisle first, then `to=` (7 eps).
## Verified parameters
- Turns of 90 degrees 36-42 steps; straight legs ~55 steps per metre; `drive(to=...)` with a
  heading over ~3 m 156; final `to=` of 0.2-0.5 m 15-51 (9 eps).
- Goals 0.24-0.30 m out from the counter's front edge succeeded (3 eps); the prior tool stood
  about a quarter metre from the cabinet face below the edge, fridges too (prior tool, 46/50).
- Wall shelves: `robot.measure` gave a centre 1.45 m high, 0.68 m wide; a 0.35 m width saw only
  part of them (1 ep); the base ended ~0.6 m short of that centre (1 ep).
## Common failures and fixes
- Look-alike: the drive to 0.25 m from a toaster oven taken for the microwave ended with no
  success; 9 more replies of nudges around it used the rest (1 ep). At a goal computed as above
  with no success the fixture is wrong: identify again and look elsewhere, no nudging (untested).
- Not in view: six 90-degree turns at two spots cost 252 of 450 steps; the toaster, in a corner
  past a fridge, was never found (1 ep). After one look each way, drive 1-2 m down the open aisle
  toward unseen counters and look again (prior tool: found it so in 2 of 4).
- Shelves: 1 of 3 succeeded; 6 base views and ~200 steps of wrist looks found only wall cabinets
  (1 ep). Look at the upper part of each wall first; tilt the wrist up after that (untested).
- Turns beside a counter BLOCKED (8 eps); 0.3 m sideways away from it first freed them (5 eps).
## Typical step costs
- Horizon 450. Successes 165-376 (16 eps); legs ~55 steps per metre; final `to=` 15-105. The
  opening above 77, 119 with a second turn; whole runs 94-447, median 212 (prior tool, 46/50).
