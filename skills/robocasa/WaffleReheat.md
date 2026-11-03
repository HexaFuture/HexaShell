# WaffleReheat
## Judging
- Done when the waffle is in the bowl, the bowl inside the oven (need not be exact) and the oven
  on, at once. Push the door shut first (step 6), then press START (step 7).
## Flow
1. Priors: the base starts close before the microwave; its door is hinged on the LEFT (- opens);
   START and STOP low right of it. Cavity floor ~0.2 m over the counter, or ~0.5 m (high unit).
2. Bowl left of the handle (hinge side): before opening, set it on a clear counter spot 0.6-0.9 m
   right of the handle, as in 4; the open door stands in the way to it and gets swept (prior tool).
3. Open (10 eps): `robot.find('right', 'microwave handle')`; hinge: `robot.to_world` on the door's
   left front edge. Base back 0.28 m; grip at pitch 0 from 13 cm in front; then
   `robot.arc(hinge, -30, axis='z')`, `robot.goto(pitch=25)`, arc -30; base 0.14 m left; re-grip
   at pitch 15, yaw -60; arc to the stop. Let go: open, then `robot.drive(forward=-0.25)` (6 eps).
   Look: fetch the bowl only with the door at its stop, else re-grip and arc it (2 eps; ~300 steps).
4. Bowl: turn the base to face it, ~0.6 m from its middle (rim_pick says how to drive);
   `robot.rim_pick(bowl)` pinches its near wall at pitch 45, roll 90 and lifts it 0.1 m, hanging
   level (30 of 30 held, scripted). Keep those angles for every move and drive while it is held;
   turned, the bowl tilts and the waffle slides out (prior tool). Before driving, draw it in: grasp
   point 0.36 m ahead of the base centre, >= 1.15 m up (prior tool). Route (both ways): back to
   0.2+ m behind the start spot (0.55+ m if the bowl is on the left, clear of the open door), then
   sideways, then forward: 79 such legs, none blocked (44 eps).
5. Insert, the bowl level ahead of the hand (prior tool): base square to the oven, its centre
   ~0.14 m right of the bowl's line; `robot.to_world` on the cavity floor and lip. Hold the bowl in
   front of the opening, its bottom 4-5 cm over the floor: grasp point z = floor + (pinch z - on_z
   of the RimPick) + 0.045. Drive in, then the hand straight in until the bowl's middle is ~0.2 m
   past the door plane (dry_run first); lower until it stops; open; hand up 4-5 cm; back out
   0.25 m. Stopped at the lip: back 6 cm, up 3.5 cm, in again; still stopped: check the door.
6. Door, pushed from outside (1 ep): base 0.25 m back, ~0.4 m left of the start spot; closed
   fingers. Round the free edge: hand ~0.15 m in front of it (pitch 45, yaw -60), left to ~9 cm past
   the door's outer face (pitch 20, yaw -75), forward to the free edge at handle height (pitch 15),
   right till it touches; `robot.arc(hinge, 40, axis='z')`, 35 more (3 eps), on to shut (untested).
7. START (3 eps): read the labels (`robot.look('right', crop=..., zoom=5)`), `robot.to_world` on
   START; closed fingers, pitch 15, yaw 0, 11 cm in front; straight in to 1.4 cm past.
## Verified parameters
- rim_pick closes on 1.1-3.4 cm; a gap narrowing up to 8 mm on the way is normal (scripted).
## Common failures and fixes
- Blind back-off-and-turn carries: held out, 8 of 30 swept it into a fridge or cabinet, 6 fell;
  drawn in, 3 fell (scripted). Turn only with ~0.7 m clear around the base on that side.
- rim_pick refused or empty twice: grasp a side wall from above instead, pitch 90, 1.5-2 cm below
  the rim, close 1.7-4 cm, keep pitch 90 and insert at pitch 20-25 (4 eps).
- Door short, bowl held (8 eps): set it down, re-open, re-fetch (700-1200 steps); 4 won, 4 lost.
- Hand moves across the open door to its outer face: BLOCKED (3 eps).
## Typical step costs
- Horizon 4050. Wins (4 eps): open 290-560, bowl 80-450, insert 160-320, START 70-180, door 420-640.
