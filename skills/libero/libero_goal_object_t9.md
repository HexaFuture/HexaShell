# libero_goal_object_t9

## Judging
- The wine bottle: dark, upright, about 0.44-0.48 ahead, 0.035-0.061 to -y, top 0.145. The rack
  that counted: the wooden rack at the upper left of the agentview picture; pointed at, one view:
  0.398-0.416 ahead, 0.227-0.247 to -y, top 0.295-0.299 (two views read 0.419-0.432 ahead).
- SUCCESS came as the bottle, still upright and held, was lowered into the rack's top from above
  (6 eps); or after a level-hand slide in at 0.35-0.37 ahead (8 eps).
- Let go with the hand level at 0.388-0.396 ahead, the bottle stayed propped on the rack and did
  not count (3 eps).
- 'rack' found nothing in 15 of 16 episodes, 'wooden rack' in 3 of 4: point at it.

## Flow
1. Free: `w = robot.measure('wine bottle')` (20 eps); point at the rack in the agentview
   picture, `r = robot.measure(('agentview', (x, y)))` (18 eps).
2. Neck from above: `robot.goto((w.center[0], w.center[1], 0.22), pitch=90, yaw=0,
   grip='open')`, down to 0.115, `robot.gripper('close')`: held 1.5 cm (20 eps).
3. Keep pitch 90: straight up to 0.36-0.42 (20 eps), then over `r.center` at 0.43-0.48 (6 eps).
4. Down to 0.38 (default speed, or 0.06-0.08): SUCCESS came during that lowering (6 eps).

## Verified parameters
- The upright drop over the one-view rack measure (0.37-0.43 ahead) worked every time it was
  tried (6 eps).

## Common failures and fixes
- Hand levelled (pitch 0-20) over the rack at 0.388-0.396 ahead, let go: the bottle stuck at
  about 0.26 up, propped; pushes ran out (2 eps); a closed-hand push at pitch 75 toward -y gave
  SUCCESS at 293 (1 ep). Stay upright (6 eps); the level slide worked at 0.35-0.37 (8 eps).
- A pitch 30 / yaw 90 lay-down also worked but cost 145-181 (3 eps).

## Typical step costs
- Horizon 300. Above 11-15, down 16-19, close 10-11, up 28-35, over the rack 27-29, down 7-13:
  105-125 upright (6 eps). Level-hand successes: 147-206 (8 eps).
