# ScrubCuttingBoard

## Judging
- Done when the sponge, held in the closed hand, has been rubbed across a good stretch of the
  board, then let go with the hand pulled back from it, at once. So: stroke, open, pull back.
- One 10 cm stroke was not enough (1 ep): plan 15-20 cm. Pressing in place, harder or longer,
  added nothing (1 ep). Where the sponge ends up does not matter.
- Released, hand pulled back, still running: the strokes were too short, or the hold too loose.
  Regrasp and stroke a new stretch (untested).

## Flow
1. Priors: the robot stands at the sponge, which lies on the counter beside the board; the board
   is next to the sink, aligned with the counter. Measure both. The board's long side runs along
   the heading `board.grip_yaw` (1 ep: away from the robot).
2. Grasp: `robot.goto((s.center[0], s.center[1], s.top + 0.11), pitch=90, yaw=s.grip_yaw, roll=0,
   grip='open')`, down to `s.top - 0.004`, close across its short side (~6 cm); lift 10 cm.
   A Grip note that this is near its top: keep it, the sponge held there (10 eps).
3. Over the board, sponge centre ~5 cm in from one end of the long side. Go to pitch 45 there and
   descend until the hand stops on the board ("short of target"): call that height h.
4. Stroke at pitch 45 to z = h - 0.009: one `goto` 15-20 cm along the long side, staying on the
   board; then a 6-8 cm leg back or sideways (strokes over 10 cm untested).
5. `robot.gripper('open')`, then `robot.goto(by=(-0.15, 0, 0.15))`: success should fire there.

## Verified parameters
- Sponge 9 x 5.5 x 1.0-1.3 cm. Grasp point 0.3-0.4 cm below its measured top: 6.0 cm, held (2/2).
  At or above its top: 0.8-1.5 cm "holding", slipped out on the lift (3/3) (1 ep).
- Pitch 45, target 0.9 cm below h: legs of 4.4 cm and 9.9 cm moved freely (1 ep).
- After opening, a goto 12 cm back and 17 cm up at pitch 60 was far enough (1 ep).

## Common failures and fixes
- Pitch 90 with the target 1.3 cm below the stop height: legs stalled after 1-4 cm; pitch 75
  stalled too (1 ep). Stroke at pitch 45.
- `wait(40)` while pressing, and closed fingers pushing from above (stuck after 1.5 cm), added
  nothing (1 ep).
- A 32 cm lift after release stopped ~21 cm up under the wall cabinet, and `home()` was then
  BLOCKED (1 ep). Pull back toward the robot instead.

## Typical step costs
- Horizon 1200, the shortest in this group. 1 ep (failed): grasp ~85, over the board and down ~90,
  the 4 + 10 cm stroke ~75, release and retreat ~80; the rest went on regrasps and pitch-90 pushes.
- Plan ~350: grasp 90, over the board 90, stroke 80, release 60. Aim to finish by 400.
