# TurnOnElectricKettle
## Judging
- Done the moment the power switch is pressed right down to its stop; it stays on. The hand may
  stay on it.
- The travel is tiny: a slide switch moves a few mm to a few cm straight down; a hinged lever tips
  its outer end down a little. Press it all the way, to its stop.
- The lid's release button only opens the lid. Still running after a press: the switch is short
  of its stop, or the hand pressed beside it.
## Flow
1. The switch sits low on the handle side of the kettle (step 2), facing the robot's right or
   front-right. It was in reach from the start pose, no drive needed (10 eps).
2. `robot.measure('electric kettle')` for the centre kc; zoom a wrist picture on the switch and
   `robot.to_world` 3-4 pixels on its top; they agreed within 1 cm; their middle is s (10 eps).
   s is at most 5 cm over the kettle's `.on_z` (30 eps); 8.5 cm up is the handle's foot (2 eps).
3. `yaw = math.degrees(math.atan2(kc[1] - s[1], kc[0] - s[0]))`: 50-109 (10 eps).
4. Press with the gripper closed (grip='close'), pitch 45, that yaw, roll 0 (10 eps):
   - pre-pose 12 cm from s back along the yaw line, 6-10 cm above its top (22-41 steps);
   - then the grasp point above the switch's middle, 3.5 cm above its top (15-22 steps);
   - then straight down to 3.5 cm below its top: success fired within 5-12 steps.
5. Step 4's second move often stopped "short of target" ~2 cm back along the yaw line, the hand
   touching the body (5 eps). At the yaw toward the centre that is fine: go straight down from
   where it stopped to 3.5 cm below the switch's top; success in all 5.
## Verified parameters
- Closed gripper, pitch 45, yaw toward the kettle's centre, roll 0; the grasp point lowered from
  3.5 cm above the switch's top to 3.5 cm below it, over its middle or up to 2.4 cm back from it
  along the yaw line (10 eps).
## Common failures and fixes
- From the 12 cm pre-pose straight to the grasp point at yaw 88 and 109: BLOCKED with the
  wrist twisted (yaw 2-40, roll -55 to -78) (2 eps; 4 others at yaw 89-105 passed). Fix: back
  to the pre-pose lowered to 4 cm above the switch's top, then above the switch, then down
  (1 ep); or pitch 15 from 9 cm back, then 3 cm back, then pitch 25 down to 2.6 cm below its
  top (1 ep).
- The pre-pose BLOCKED by an appliance or wall within ~10 cm of the switch on the hand's side (4
  eps at yaw 110; look): yaw 45, its pre-pose (turning there BLOCKED: again), then 5.5 cm back
  along the yaw line 4 cm above the switch's top, then press to 1 cm back 2.5 cm below it (10 of 10
  replays at 40-45, 50-70 missed; prior tool). Not step 5 there: it went beside the switch (1 ep).
- "joint 4 (the elbow) fully bent": the switch is too close in: `robot.drive(forward=-0.15)`,
  press again (1 ep; prior tool: only backed agents pressed), or pitch 25 for the last cm (1 ep).
- A home and a sideways, turning or backing drive after a block: 100-150 steps, no help (3 eps).
- Pitch 90 stopped 7 cm high on the handle; pitch 0-20 stalled; 2.6 cm aside went beside it (1 ep).
- Do not hook the handle or the body: the kettle lifts off its base (untested).
## Typical step costs
- Horizon 450. Successes 57-93 (8 eps): measure and to_world 0, pre-pose 22-41, above the switch
  15-22, press 5-12. Twisted-wrist block: 141 and 310 (2 eps); the yaw change: 173, 320 (2 eps).
