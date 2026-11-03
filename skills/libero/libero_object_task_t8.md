# libero_object_task_t8

## Judging
- The 'salad dressing' that counted: the white bottle with a green cap, nearest the camera in the
  lower middle of the agentview picture: about 0.75-0.77 ahead, y about +0.03, top 0.139-0.146,
  12-13.5 cm tall. Lowered into the basket, it ended the episode with SUCCESS (19 eps).
- The orange bottle in the left part of the agentview picture (about 0.71 ahead, 0.20 to -y, top
  0.146) is not it: put into the basket it ended nothing (1 ep).
- 'salad dressing' found nothing in all 20 episodes: point at the white bottle.

## Flow
1. Free: point at the white bottle, `c = robot.measure(('agentview', (x, y)))` (19 eps);
   `b = robot.measure('basket')`: top 0.140, 0.60-0.62 ahead, 0.25-0.27 to +y (19 eps).
2. `robot.goto((c.center[0], c.center[1], 0.25), pitch=90, yaw=0, grip='open')`, down to 0.12,
   `robot.gripper('close')`: held 3.5-3.7 cm (19 eps). Straight up to 0.30.
3. Over the basket's measured centre at 0.29-0.33, down to 0.15: SUCCESS came during that
   lowering, before any opening (19 eps).

## Verified parameters
- The descent at yaw 0 reached 0.114-0.135 at 0.746-0.764 ahead, y 0.029-0.030 (19 eps).

## Common failures and fixes
- The orange bottle first (held 3.3 cm), let go over the basket: no SUCCESS; the white bottle
  came after it and the steps ran out (1 ep). Take the white bottle first (19 eps).

## Typical step costs
- Horizon 280. Above 32-59, down 19-30, close 10, up 23-49, over the basket 30-46, down 12-15:
  158-193 (19 eps).
