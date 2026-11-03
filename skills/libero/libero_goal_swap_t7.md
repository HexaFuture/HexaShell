# libero_goal_swap_t7

## Judging
- The stove: round, about 18-19 cm, top 0.019, 0.765-0.78 ahead, 0.136-0.143 to +y (lower right
  of the agentview picture).
- The control that turned it on: the black round knob with a bar across its top, on the robot's
  side of the stove (above the stove in the agentview picture): 0.596-0.620 ahead, 0.117-0.137
  to +y, top 0.047, 7.3-8.4 cm across. Gripped and turned, it ended the episode with SUCCESS
  (19 eps).
- Not controls: the flat light-grey strip along the stove's robot-side edge (about 0.70 ahead,
  0.135 to +y, top 0.015-0.018), taken for a switch (4 eps), and the plate on the stove's -y side
  (about 0.72 ahead, 0.03 to -y), moved aside to look under it (2 eps). Neither ended anything.
- 'stove knob' found the knob in 8 of 20 episodes; otherwise point at it.

## Flow
1. Free: `k = robot.measure('stove knob')`, or point at the knob (19 eps).
2. `robot.goto((k.center[0], k.center[1], 0.13), pitch=90, yaw=0, grip='open')`, down to 0.033,
   `robot.gripper('close')`: held 2.5 cm (19 eps).
3. `robot.goto(yaw=45)` (40-60 all worked): SUCCESS came during the turn (19 eps).

## Verified parameters
- Grasp at the knob's measured centre at 0.032-0.038 and yaw 0, then a 40-60 degree turn
  (19 eps).

## Common failures and fixes
- Took the light-grey strip for the switch, then moved the plate to look for one: BLOCKED going
  down, closed on nothing, carried the plate off; ran out at 296 (1 ep). The others that went
  there first reached the knob late: SUCCESS at 145-234 (3 eps). Go to the knob first (16 eps).

## Typical step costs
- Horizon 300. Above 29-31, down 16-24, close 10, turn 20-21: 76-83 straight to the knob
  (10 eps); with a first stop above the stove's middle: 98-121 (6 eps).
