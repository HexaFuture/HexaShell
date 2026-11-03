# SteamInMicrowave
## Judging
- Done when the vegetable is in the bowl, the bowl wholly inside, the door shut and the oven on.
## Flow
1. Priors: bowl on the counter beside the sink; door hinged on the LEFT, wide open (about
   square). Oven 1.5-4.5 m off (8 eps); lip 1.40-1.48 m above a cooktop (7 of 18 eps), else ~1.1 m.
2. Vegetable (<= 250): pitch 90, yaw = grip_yaw, height and 'light' as the acting rules say; a ~15
   cm slide toward the robot before lifting (untested): 6-7 cm slides hit the tap (5 eps); if
   BLOCKED, go 7 cm back, 20 cm up (1 ep). Let go 3-5 cm over the rim, mid-bowl (9 of 10 eps).
3. Bowl (<= 100): turn the base to face it, ~0.6 m from its middle (rim_pick says how to drive);
   `robot.rim_pick(bowl)` pinches its near wall at pitch 45, roll 90 and lifts it 0.1 m, hanging
   level (30 of 30 held, scripted). Keep those angles for every move and drive while it is held;
   turned, it tilts and the vegetable falls out (prior tool). Never home() with it (2 eps). Then
   draw it in: grasp point 0.36 m ahead of the base centre, >= 1.15 m up, angles kept (prior tool).
4. Find (<= 300): back 0.3 m before each 90-degree turn, ~0.7 m clear around the base on the
   turning side: blind back-off-and-turn carries dropped 6 of 30 held out, 3 drawn in (scripted).
   The oven: door open on its LEFT, keys on the right; a high one shows its underside at a picture's
   top. Its door juts ~0.45 m: a bowl carried past swept it shut (4 eps); pass with the hand >= 0.6
   m out, come in from the front (prior tool).
5. Insert (<= 200), the bowl level ahead of the hand (prior tool): park square, base centre ~0.14
   m right of the bowl's line, right of the open door (2 eps hit it); `robot.to_world` on the lip,
   the cavity floor and the opening's sides. Hold the bowl in front of the opening, its bottom 4-5
   cm over the floor: grasp point z = floor + (pinch z - on_z of the RimPick) + 0.045. Drive in,
   then the hand straight in until the bowl's middle is ~0.2 m past the lip (dry_run first); lower
   until it stops; open; hand up 4-5 cm; back out 0.25 m. Stopped at the lip: back 6 cm, up 3.5
   cm, in again. High oven: the same; a cooktop under it can stop the base short: set the bowl
   3-9 cm less deep, its near rim still inside the front (prior tool).
6. Door (<= 350): base 0.67-0.78 m behind the hinge line, 0-0.3 m left of the hinge; closed fingers,
   pitch 15-20, yaw = square heading - 90, 3-5 cm off the outer face, 0.50-0.55 m from the hinge
   (from the right, round the free edge 0.25 m in front of it: straight there was BLOCKED, 1 ep);
   `robot.arc(hinge, 35, axis='z')` (3 of 4 eps). Home, base 0.1-0.3 m closer, yaw square - 40 to
   -56, arc +30-35; then arc +15, or push straight in near the free edge, until BLOCKED (2 eps).
7. START (<= 100): outer bottom-row key, blue-marked when zoomed (STOP red); press 10 cm out to
   1.5 cm past, pitch 0 (10 eps, other task), or 20-35 down below 1.30 m (2 eps; prior tool). Door
   ajar: on, but no SUCCESS: push it shut, robot.wait(2), press the other key (2 eps).
## Verified parameters
- rim_pick closes on 1.1-3.4 cm; a gap narrowing up to 8 mm on the way is normal (scripted).
## Common failures and fixes
- Grasped any other way, at the opening: never drive, roll or slide it; on BLOCKED back out ~10 cm
  (9 eps lost it).
- rim_pick refused or empty twice: pitch 90 on its left or right wall, 1-1.5 cm inside the outer
  edge, 1-1.5 cm below the top, 'close', lift 15 cm (26 eps; under ~6.5 cm tall it fails: 6 eps).
## Typical step costs
- Horizon 2100. Successes ended at 1204-2011 (10 eps). Stages: vegetable 140-490, search up to 570,
  insert 150-200 turned, 385-620 re-gripped, door 400-1000, START 30-280.
