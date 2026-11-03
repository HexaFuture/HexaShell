# TurnOffStove
## Judging
- Only the named burner's knob counts. Done the moment it is turned back to about off, even with
  the fingers on it; other knobs and the pot on the stove do not matter.
- Only that burner is on, its knob well away from off. Its bar is the only burner bar turned away
  from its "0" mark; its burner shows a flame or glows red (2 eps).
- Knobs left to right: FL, RL, [centre], RR, FR (prior tool). The oven's two numbered dials are not
  burner knobs; on some stoves they end the row: one taken for FR cost ~100 steps (1 ep).
## Flow
1. Where the knobs are (20 eps): on the front of a range, ~0.8-0.9 m high and ~0.2 m ahead
   of the base (14 eps); on the top surface (4 eps); on a raised back panel, ~1.1 m high (2 eps).
2. Front knobs: off is clockwise seen from the robot, + roll (11 eps). At off the bar
   stands upright, pointer up (1 ep). Read θ, the pointer's counter-clockwise turn from 12 o'clock.
   A bar near upright, its top end leaning left (y grows to the left), is 180 + the lean; leaning
   right, 180 - the lean. 165 taken for 193: the turn BLOCKED at once (1 ep).
3. Front knobs are out of reach from the start (6 eps). Drive back and sideways AWAY from the knob
   until it is 0.38-0.42 m ahead and 0.35-0.38 m to the side; for a knob 0.2 m ahead and 0.2 m
   right: `robot.drive(forward=-0.22, left=0.17)` (12 eps). `dry_run` the grasp at roll -θ and 0.
4. Pre-pose 6-13 cm in front of the knob at its height, pitch 0-15, yaw 0, roll -θ, grip='open';
   straight in to the bar's front face (to_world on it); `robot.gripper('close')` (12 eps).
5. Turn: `robot.goto(knob_xyz, roll=-θ + 60)`, then on toward 0, at most 60 degrees per move;
   success fired on moves to roll -15 to +20 (9 eps). NOT executed: a smaller step, or pitch 35
   for the last move (1 ep; the prior tool halved its step).
6. Swap the fingers (2 eps) if roll -θ is NOT reachable or a turn is BLOCKED, or short with a joint
   at its end: open, back 7 cm, re-read θ, grasp the same bar at roll 180 - θ, turn + toward 180;
   success fired at 90-180. Three grasps near -θ failed (1 ep). Knob on the base's left, θ 150 or
   more (-θ stuck in 2 of 4 eps): grasp at -θ - 180, from roll -90 by - steps up to 90 (prior tool).
7. Top-surface knobs: from 11-18 cm above, pitch 90 at yaw = the bar's direction, down onto it,
   close, then lower the yaw (clockwise from above) in 1-2 moves; done at yaw 0 to -15 (4 eps).
8. Back-panel knobs: the base 0.73-0.77 m behind the knob and 0.29-0.39 m to its side; pitch 0,
   roll -θ, close, turn + roll up to 65 degrees per move (2 eps).
## Verified parameters
- Front, step 3's stance (12 eps): grasp rolls -45 to -170, swapped +30 to +75; pitch 15 (0 out of
  reach in 3 eps); held 1.4-5.5 cm; 1-4 turns of 8-60 steps. Top knobs held 0.9-4.4 cm (4 eps).
## Common failures and fixes
- Backing straight up 0.29-0.42 m with the knob 0.10-0.23 m to the side: level poses NOT
  reachable or short, wrist or shoulder at its limit (5 eps). Move sideways as in step 3.
- Hunting a pose by trial: 460-580 steps before a grasp, both failed (2 eps). Place the base first.
- Turning from the edge of reach ("short of target" with the shoulder at its limit, or "full force"
  stretched toward a back panel): the knob slipped out (4 turns, 3 eps). Swap the fingers (step 6);
  backing 0.10 m off step 3's stance put every later grasp at a joint limit (1 ep).
## Typical step costs
- Horizon 750. Successes: top knobs 82-160 (4 eps), front 152-453, 8 of 11 under 210, back panel
  134 and 717; failures 738-750 (3 eps). Clean front knob: drive 20-30, pre-pose 55-70, in 12-21,
  close 10, 1-3 turns of 8-60: 150-210 (7 eps). A finger swap, pre-pose to close: 85-100 (2 eps).
