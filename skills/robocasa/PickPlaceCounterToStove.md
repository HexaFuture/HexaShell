# PickPlaceCounterToStove

## Judging
- Done when the food lies in the pan near the middle of its round bowl, let go, and the hand is
  well away from it, at once. Food against the wall or on the rim fails. Which burner, the knobs
  and the plate do not matter.
- The measured 'pan' includes its handle: its centre lies several cm toward the handle; aim at
  the bowl floor's middle instead (step 2).
- Success fired on the lift or the move away after the open (10 eps).

## Flow
1. Priors: the food lies on a plate at the front of the counter beside the stove. The pan sits on
   a burner, its handle turned partly toward the robot.
2. `robot.measure('<food>')`; if NOT FOUND, point at it: `robot.measure(('left', (x, y)))`
   (3 eps). `robot.to_world` on the bowl floor's middle in 'left' and 'right': the target.
3. The food started ~0.4-0.7 m to the side (9 eps). `robot.drive(left=...)` 0.22-0.40 m toward
   it, arm at home; a drive stopped partway did no harm (3 eps). The bowl was then ~0.35-0.7 m
   ahead and up to ~0.5 m to the other side, and the arm alone carried the food there (8 eps).
4. Grasp from above with `yaw=thing.grip_yaw`. Thick food (4-11 cm): the grasp point 1.5-2.5 cm
   below the measured top (7 eps). Thin food (1-3 cm): down to its `on_z` (all held, 6 eps); 0.2-0.4
   cm under its top it slipped (1 ep). Check the Grip width; `robot.goto(by=(0, 0, 0.15))`.
5. Carry 12-18 cm above the bowl floor to above the bowl's middle; a bowl 0.72 m ahead was reached
   at pitch 70 (pitch 90 stretched, 1 ep). Lower until "short of target" (grasp point 3-6 cm over
   the floor) or the food is ~1 cm above it; `robot.gripper('open')` (9 eps).
6. `robot.goto(by=(0, 0, 0.15))`, then away; if that is BLOCKED, `robot.home()` or
   `robot.drive(forward=-0.25)` (3 eps).

## Verified parameters
- Closed widths that held to the pan: stick 2.2 cm, marshmallow 2.3, carrot 3.3, eggplant 4.2,
  mushroom 4.5, garlic 5.0, drumstick 5.5, lime 6.0, steak 6.1, tomato 6.7 (10 eps).
- A drive while holding, with the hand drawn back over the base first, kept the food (1 ep).

## Common failures and fixes
- Round food grasped 0.5-1 cm below its top: the gap shrank 4.2 -> 2.1 cm on the lift and it
  slipped out on the reach (1 ep, twice). 1.9 cm below the top it held (1 ep).
- A drive with the arm stretched out toward a far bowl was BLOCKED and dropped the food (1 ep).
- `robot.goto(by=(0, 0, 0.30))` after the open at 0.6-0.7 m reach was refused as stretched
  (4 eps); 0.15-0.17 worked.
- A lift or move away after the open BLOCKED high over the stove: retries cost 30-70 steps;
  `robot.home()` or a drive back ended it (3 eps).

## Typical step costs
- Horizon 600. 193-295 used (9 eps): drive 21-30, pre-pose 23-39, descend and close 35-37,
  lift 17-18, carry 34-77, lower 14-27, open 13-14, up 17-22. Drops and re-grasps added ~300 (1 ep).
