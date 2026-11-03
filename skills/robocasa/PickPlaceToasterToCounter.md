# PickPlaceToasterToCounter

## Judging
- Done when the slice rests on the plate (its centre over the plate) and the hand is well away
  from it, at once. Its pose and the toaster do not matter. Leave the lever alone: pushing it down
  lowers the slice into the toaster. It fired in the lift after the release (9 eps).

## Flow
1. Priors: one slice stands in a slot of a toaster at the counter's back edge, its faces left and
   right; the plate is near the counter's front edge, not far to the side of the toaster.
2. Point at the brown strip in a slot: `m = robot.find(cam, at=(x, y))`, `robot.measure(m[0])`
   (9 eps). In a side picture, check for a wall cabinet over the toaster.
3. No cabinet over it: hover 11-12 cm above the slice's top, pitch 90, yaw 0, grip='open' (3 eps);
   measure again (its centre moved 1.3-3.1 cm, 5 eps); go down to 1-1.5 cm below its top; close;
   `robot.goto(by=(0, 0, 0.15))` (0.18-0.19 were refused, 2 eps).
4. Cabinet over it: pitch 90 was BLOCKED at every height (7 eps): skip it. Start 15-18 cm in front
   of the slice at its top's height, pitch 40, yaw 0, open; move straight forward to a grasp point
   1-2 cm below its top (4 eps); close only once the move arrived.
5. Going up from a tilted grasp was BLOCKED after 2-6 cm (6 eps). Turn the hand to pitch 25 there
   (10-25 worked; 0-5 were refused, 2 eps), then rise to 8-14 cm above the slice's top (6 eps).
6. Carry above the plate's centre at that height; turn to pitch 55-90 before lowering (lowers at
   pitch 25-40 were refused, 3 eps); lower to a grasp point 11-14 cm above the plate's top
   (9 eps); `robot.gripper('open')`; `robot.goto(by=(0, 0, 0.30))`.

## Verified parameters
- Grip on the slice 1.8-2.5 cm; once clear of the slot it never slipped (9 eps).
- First reach under a cabinet: pitch 35-40 reached the slice at once (4 of 4 tries); pitch 60-70
  stopped 4-5 cm short or BLOCKED (5 of 7 tries).
- Forward drives at the start stopped on the counter after 1-6 cm (5 eps); skip them (1 ep).

## Common failures and fixes
- The failure: under a cabinet, two pitch-60 reaches stopped short, a pitch-80 descent was BLOCKED,
  and it closed there (4.1 cm); the lift wedged the fingers in the slot (gap to 1.9 cm, then they
  "opened only to" 3.2-5 cm). 16 moves BLOCKED (~260 steps) until a 6 cm sideways move and pitch
  20 freed it; later grasps slipped; out of steps (1 ep). Never close after a BLOCKED descent.
- Pitch-90 hovers and descents tried first under a cabinet cost 40-190 steps (7 eps), more with
  `robot.home()` and drives between them.

## Typical step costs
- Horizon 600. Successes 176-343 (9 eps); the failure ran out at 562.
- No cabinet (3 eps, 176-194): hover 29-35, down 16-17, close 10, lift 23-37, carry 35-37,
  lower 24-27, open 14, lift until success 14.
- Cabinet (6 eps, 260-343): pitch-90 tries 40-190, tilted reach 20-45, close 10, blocked lift
  10-15, turn and rise 20-35, carry 24-35, turn and lower 20-40, open and lift 28-53.
