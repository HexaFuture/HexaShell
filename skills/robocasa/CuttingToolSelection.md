# CuttingToolSelection
## Judging
- Common sense: knife for garlic, broccoli, avocado; peeler for eggplant, carrot, cucumber, potato.
- Done when the right tool lies on the board near its middle, the other tool is not on it, and
  the hand is well away from the board, all at once.
## Flow
1. Start (10 eps): the robot is beside a part-open drawer; both tools lie in it, mostly
   under the counter top (once wholly); the board is on the counter's front edge above it.
2. Measure the board and the vegetable first, before the arm hides them (prior tool).
3. From the start spot, no drive first, pull the drawer 0.22 m further out before any grasp: the
   tools come out with it (prior tool, 28/30). Grip the top edge of ITS front (~0.89 m; once lower:
   a pull that left the tools in place had gripped another drawer), not the bar on its face:
   `robot.to_world` on that edge ~0.35 m to the drawer's side of the base (prior tool).
4. 10 cm above it, pitch 90, yaw 90 (fingers across its thickness), grip 0.6; straight down to
   1.2 cm below its top; close (~2.8 cm, holding); `robot.drive(forward=-0.22)`; open; up 0.12 m,
   6 cm toward the robot (prior tool). Way down BLOCKED: base back 0.15 m, repeat (prior tool).
5. Drive forward the pulled distance and 6 cm toward the drawer: from there the arm alone grasped
   and placed (prior tool). Refused, or the base under 0.4 m behind the spot (the hand stopped 4-7
   cm above the tool, 2 eps): base 10-20 cm back or 10 cm sideways, try again (prior tool).
6. Wrist ~1 m up over the tools; measure the right one. Knife: >=19 cm long, <=5 cm wide, flat
   blade, its handle the higher end; peeler: a head >=5 cm wide on a handle (prior tool).
7. Spot on its axis 8 cm from the knife's handle end, 5 cm from the peeler's (prior tool). From
   10 cm above, pitch 90, yaw = grip_yaw, grip 'open', straight down to `.on_z` + 0.5 cm, under
   the usual 1 cm (see below); close: 2.3-3.3 cm held (6 eps).
8. Lift, look, up to ~1 m; carry by the arm, the board within ~0.75 m of the base (prior tool).
   "The gap between the fingers changed": set it down, grasp lower; going on dropped it (2 eps).
9. Release with the tool's middle within 10 cm of the board's centre, >=1 cm off the vegetable
   (prior tool); grasp point 2.7-4.3 cm above the board's `.top`; open; up 0.3 m (9 eps).
## Verified parameters
- Close height, grasp point above `.on_z`: 1-3.1 cm, 0 of 19 reached the board (6 eps); <=0.8 cm,
  6 of 8 did, 5 of them through a base drive (7 eps).
- "The hand stopped 11-32 mm from the target" going down: closing there lost the tool 9 of 9 (4
  eps); instead back 3 cm the way it came, up, aim 3 cm further toward its middle (prior tool).
## Common failures and fixes
- Grasping a tool still under the counter: BLOCKED or "pressing on something" 11-16 times an ep,
  grips slipped, steps ran out (4 eps): pull first (step 3).
- Pitch-90 grips on the handle bar closed empty or pushed the drawer in (8 eps, 6 failed); a level
  hand rolled 90 round the bar, then `robot.drive(forward=-0.30)`, pulled it (1 ep).
- Top edge closes empty: hook a closed hand just inside the front, 4 cm below its top, same drive
  (untested). No pull at all: drag the tool 10+ cm toward the robot, regrasp mid-length (2 eps).
- Down at grip_yaw BLOCKED 7-10 cm over the knife: a drawer wall is beside it (5 eps; probe, 3).
  Turn the hand 30-45 degrees so no finger is between knife and wall: held (2 eps; probe, 3).
## Typical step costs
- Horizon 1200. Successes 307-605 (5 eps) and 1020 (1 ep). Failures ran out after 31-39 replies:
  moves 500-800 steps, drives 110-190, homes 130-340 (4 eps): lift 10 cm and move on, not home.
