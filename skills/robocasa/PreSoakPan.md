# PreSoakPan
## Judging
- Done when the water is on (the lever stick tipped well over to the robot's RIGHT), pan and
  sponge lie wholly inside the basin, handle included, and the hand is away from both, at once.
  The stick stays where a push leaves it, so pushes add up.
- Water is a thin see-through blue column under the spout: zoom on it; once it ran unseen for
  1450 steps (1 ep).
## Flow
1. Lever FIRST, from the start pose, basin empty, OPEN hand (scripted: water on in 10 of 10 eps,
   by step 182-235; pushes from the counter, pan and sponge in: 3 of 10 never turned it, 10 eps).
   Its top (x, y, z): to_world in the right AND left camera before any move; 6-9 cm further hit
   the wall: the nearer (7 eps, faucet task). Do not drive forward. Stick more than 0.20 m right
   or 0.15 m left of the hand: `robot.drive(left=...)` so that it is 0.10 m right of the hand;
   (x, y, z) stay. pitch=0, yaw=20, roll=180, grip='open': `robot.goto((x - 0.096, y - 0.058,
   z - 0.013), ...)`, then `robot.goto((x + 0.017, y - 0.017, z - 0.013), ..., speed=0.08)`;
   `robot.push_arc((x, y + 0.01, z - 0.07), 60, axis='x', push=(0.34, -0.94, 0), press=0.05,
   tilt=0)`: 'jammed', 'angle', 'stalled' stops are normal (scripted: 10 eps). Back out
   `robot.goto(by=(-0.06, -0.03, 0))`, `robot.home()`; zoom on the stream. None: the grasp-push.
2. Pan (handle toward the robot): side drive until it is in front of the hand; pitch 90, fingers
   across the handle 11-15 cm from the pan's `.center`, 1-2 cm below the handle's top; lift 0.25 m
   (8 eps). Taken at the handle's end (17-20 cm out) it tipped or took 3 placements (2 eps).
3. Keep that offset: grasp point where the whole pan's middle, handle included, comes over the
   middle of the (larger) basin, not a drainer basket (1 ep; prior tool). Lower the grasp point
   to the pan's first `.on_z` minus 0.06 (7 eps). Then open, also after "short of target" (an
   `if m.ok:` kept the pan in the hand, 1 ep).
4. Sponge: pitch 90, yaw = its `.grip_yaw`, 1-3 cm below its top; let go over bare basin floor (5
   eps), 8 cm or more in front of the spout (prior tool). Look: both flat, below the rim (8 eps).
   Still running with the stream on: take the hand away with `robot.home()` (1 ep).
## Verified parameters
- Grasp-push that finished here: gripper 0.5, pitch 25-40, yaw 0, roll 0, in level from 10 cm in
  front, grasp point 1.2-1.7 cm below the top; close (holding 0.7-1.8 cm); push with
  `robot.goto(by=(0, -0.05, -0.015))`; open, back 12 cm, re-measure the top: 1-4 cycles (7 eps).
## Common failures and fixes
- Lever last, base driven to the counter: pitch 0-45 entries BLOCKED or 6-19 cm short, holds of
  0.9-1.2 cm, pushes left it still; 1400-1920 steps gone in 3 failures (10 eps). Use step 1.
- Closing after a short or BLOCKED entry: empty, 30-50 steps lost (4 eps).
- Pan lowering stopped over 4 cm above its target: it hangs tipped; lift 1.5 cm, move it 5-10 cm
  away from the robot, lower again, open (prior tool).
## Typical step costs
- Horizon 2400. Lever by push_arc 182-235 (scripted: 10 eps); pan + sponge 440-970 (10 eps); old
  lever ways 220-650 in 6 successes, 1140 in one, 1400-1920 in failures (10 eps).
