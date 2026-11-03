# libero_object_object_t5

## Judging
- The 'tomato sauce' that counted: the short round can with a blue-and-yellow label, 6.0-6.3 cm
  across, top 0.080, about 0.65 ahead and 0.10 to -y (left of the middle and below it in the
  agentview picture). Lowered into the basket, it ended the episode with SUCCESS (17 eps).
- The slim brown bottle with a cap, nearest the robot (about 0.40 ahead, 0.07-0.08 to -y, top
  0.112, a little left of and above the middle of the agentview picture), is not it: put into
  the basket it ended nothing (3 eps).
- 'tomato sauce' found nothing in all 20 episodes: point at the can. 'tomato sauce bottle' found
  the brown bottle, which was then taken (1 ep).

## Flow
1. Free: point at the can, `c = robot.measure(('agentview', (x, y)))`: round, 6.0-6.3 cm, top
   0.080, 0.65 ahead, 0.10 to -y (17 eps). `b = robot.measure('basket')`: top 0.140, 15-16 cm
   across, 0.60-0.62 ahead, 0.25-0.27 to +y (17 eps).
2. `robot.goto((c.center[0], c.center[1], 0.18), pitch=90, yaw=0, grip='open')`, down to 0.053,
   `robot.gripper('close')`: held 6.2 cm (17 eps). Straight up to 0.235.
3. Over the basket's measured centre at 0.25-0.30, then down to 0.095: SUCCESS came during that
   lowering, before any opening (16 eps).

## Verified parameters
- Yaw 0 straight down onto the can's measured centre reached 0.052-0.059 and held it every time
  (17 eps).

## Common failures and fixes
- The brown bottle first (held 3.0-3.6 cm), into the basket: no SUCCESS. One run then called
  finish with 22 steps left; two reached the can with 0-6 steps left (3 eps). Take the can first
  (17 eps).
- A run that lifted the bottle and set it back before taking the can ended at 278 of 280 (1 ep).
- Going first to the item in the lower middle of the agentview picture (about 0.75 ahead, y
  about +0.03) cost 36-40 steps (3 eps).

## Typical step costs
- Horizon 280. Above 31-38, down 20-26, close 10, up 24-27, over the basket 39-41, down 10-16:
  141-152 with the can first (14 eps).
