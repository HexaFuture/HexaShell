# libero_10_task_t8

## Judging
- The "left" moka pot here is the +y one (right in the agentview picture, the robot's left), the
  pot farther from the stove: pointed at there (x about 405-440, y about 285-305) such a pot
  measured 0.60-0.64 ahead, 0.21-0.25 to +y, top 0.124 (10 eps of the variant with both pots).
  Lowered onto the stove it ended the episode with SUCCESS; the pot nearer y 0, lowered there and
  let go the same way, did not (2 probes).
- All 10 learn1 runs moved the pot in the middle of the picture (0.70-0.74 ahead, 0.01-0.06 to
  +y), and none ended (10 eps).
- `robot.measure('stove')`: 0.61-0.63 ahead, 0.18-0.21 to -y, about 0.18 across, top 0.019
  (7 eps); once it gave a box at x -0.85 (1 ep): then point at the stove instead.

## Flow
1. Free: `p = robot.measure(('agentview', (x, y)))` on the +y pot, `s = robot.measure('stove')`
   (10 eps).
2. Lid knob from above: `robot.goto((p.center[0] - 0.005, p.center[1] + 0.015, 0.25), pitch=90,
   yaw=90, grip='open')`, straight down to 0.13, `robot.gripper('close')`, up to 0.25 (probe;
   from our own pointed measure of the same pot in the sibling variant too, probe).
3. Across at 0.25 to over `s.center`, then straight down toward 0.16: SUCCESS came during that
   lowering, the pot still held (probe).

## Verified parameters
- Knob grasps held 1.1-1.5 cm (18 eps in the three pot variants). Grasps across the pot's body
  closed at 7.6-8.1 cm; they slipped or wedged in 5 eps of the sibling variants, held in 4.

## Common failures and fixes
- Pointing at the left of the agentview picture, or at the middle pot, for "left" (10 eps): the
  pot that counts is the +y one (2 probes).

## Typical step costs
- Horizon 520. Over the pot 47, down 19, close 12, up 21, across 47, down 17: 163 (probe).
