# GarnishPancake
## Judging
- Done when the berry rests on the pancake near its middle, the pancake still on its plate and the
  plate on the counter (don't push them), and the hand is well away from it.
- The berry sits at the front of a fridge shelf about 1-1.5 m up, another food behind it; the plate
  by a stool at the dining counter. A berry fallen in a drive was lost (10 eps).
## Flow
1. Pancake first: back ~0.6 m (BLOCKED: 0.25 m forward), turn ~170, look; measure it at a pixel,
   keep `.center`, `.top` (11 eps). Unseen: ~10 px, 4 m off on a ledge by the stools: zoom (2 eps).
2. Which doors (prior tool): over wide drawers both, the right first; to the floor the one without a
   water/ice panel (none: the right); one tall door: it. No reach past a door under 60 deg (9 eps).
3. Door grip (prior tool, 7 eps): base square, the bar's front 0.51-0.54 m ahead; at 1.10-1.30 m,
   pitch 15-25, yaw square, roll 0: open, 9 cm back along the hand, in to 0.5 cm past its front,
   close on its upright part; one ending under 1.2 m: pitch 45, 5 cm under its end (2 eps). Pairs to
   the floor: open 0.6 (at 1 a finger hit the other bar), grip 1.00-1.09 m (15 eps; 1.15+ slid).
4. Swing `robot.drive(turn=..., about=(hx, hy))` about the hinge, ~4 cm in from the door's outer
   edge (prior tool). Hand on the bar: `x, y, _ = robot.state().at`, `c, n` = cos, sin of
   radians(`.heading`); behind `(hx-x)*c + (hy-y)*n` 0.06-0.09; `s = (hy-y)*c - (hx-x)*n`: |s|
   0.34-0.38 pairs, 0.60-0.66 wide (prior tool, 150 opens); `turn=-85 if s > 0 else 85`. A pair's
   bar slid out before 30 deg at |s| 0.46+ 5 of 13 swings, at 0.38 or less 0 of 22 (43 eps).
5. Leave: open, `robot.wait(6)`, back 11 cm level along the hand, home (prior tool, 5 eps); out
   square 0.75 m before the bars (a wide door 1.2 m: at 0.3-0.83 the hand pushed it, 2 eps), across.
6. Berry: base 0.60-0.65 m behind it, facing it (prior tool, 3 eps); pitch 45, `.top` over 1.35 m:
   35, yaw 30 if yaw 0 stops short (11 eps). Open, from 15 cm back along the hand in to 1 cm past
   `.center` at height min(`.on_z`, `.top` - 0.022): within 1 cm over `.on_z` 17 of 20 held, higher
   3 of 16 (34 eps). Under ~1.10 m: 90 if `dry_run=True` passes (1 ep).
7. Re-seat: `robot.gripper(max(0.6, g.width_m / 0.08 + 0.15))` (`g`: the close), 1.5 cm deeper
   along the hand, `robot.gripper('light')` (replays: 8 of 13 falls kept in, 0 of 14 holds lost);
   closed re-seats held 8 of 8 2-9 m (8 eps); not re-seated, 4.3-4.8 fell 7 of 15 (15 eps).
8. Lift 7 cm, back out 30 cm level, drive back ~0.4 m; then no pitch change, no home (10 eps), the
   hand ~0.28 m ahead at ~1.28 m; turns of 90 or less (164-180 shrank the hold 0.4-0.8 cm, 3 eps).
9. Place from the stools' side, round the counter's end, base 0.6-0.9 m off; at the carry pitch,
   grasp point 3-4 cm over `.top` at the pancake's `.center`; open; back and up 25 cm+ (4 eps).
## Verified parameters
- The berry closes 3.4-4.8 cm, re-seated 3.4-3.7; it measures 1.6-3.6 cm high (34 eps).
## Common failures and fixes
- Slip refusal of a drive with the berry: from over ~1.4 m to the pancake, driving on dropped it 9
  of 9 (9 eps); within ~1.1 m 2 of 3 held (3 eps). Farther: set it down on the counter beside, grasp
  again by steps 6-7 at pitch 45 (1 ep); grasped again at pitch 90 it fell 3 of 4 (4 eps).
- A swing BLOCKED within ~15 steps and 15 degrees: `s` against the sign (8 swings) or no room (7);
  repeating never helped (9 eps). The base backs `math.hypot(d, s) - d` + 8 cm (d: hinge ahead of
  its centre); less room: bar 0.42 m ahead, pitch 60 reached -85 (1 ep), or `robot.arc` by 30-40.
- Reach BLOCKED, doors open: base 0.65 m back, 0.15 m off away from the hinge, pitch 50 (1 ep).
## Typical step costs
- Horizon 2700; successes 1367-2643 (11 eps): pancake 110-240, doors 600-1350, carry 300-750.
