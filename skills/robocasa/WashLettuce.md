# WashLettuce
## Judging
- Done once the lettuce has been under the running stream for a while, its
  middle right under the stream, below the outlet. Held, lying in the basin or in the colander
  all counted (6 eps). Water: the lever's top tipped well over to the robot's RIGHT; it stays on;
  the stream shows as a blue column.
## Flow
1. Priors: the lettuce (~10 cm) sits near the middle of a colander ~20-30 cm across (4 eps);
   the lever stick (5-8 cm) is ~0.7-0.85 m ahead of the start (6 eps). Do not drive forward.
2. Lever first, OPEN hand (scripted: water on in 10 of 10 eps). Its top (x, y, z): to_world in the
   right AND left camera before any move; a left pixel on the spout (higher, nearer) or the robot
   is no reading (12 eps); 6-11 cm further hit the wall: the nearer (6 eps). Stick more than 0.20 m
   right or 0.15 m left of the hand: `robot.drive(left=...)` so that it is 0.10 m right of the
   hand; (x, y, z) stay. pitch=0, yaw=20, roll=180, grip='open': `robot.goto((x - 0.096, y - 0.058,
   z - 0.013), ...)`, then `robot.goto((x + 0.017, y - 0.017, z - 0.013), ..., speed=0.08)`. **Push
   only if that move reached its target** ('at ... ±N mm'): water on 19 of 19; from stops 4-14 cm
   short, 0 of 5 (20 eps). `robot.push_arc((x, y + 0.01, z - 0.07), 60, axis='x', push=(0.34,
   -0.94, 0), press=0.05, tilt=0)`, whatever way it stops (19 eps); back out `robot.goto(by=(-0.06,
   -0.03, 0))`; zoom on the stream. Contact short: home, re-measure the top (zoomed; left camera
   within 1 cm), redo both moves (1 ep). Short again (untested): pitch 60, yaw 45, roll 90, open,
   9-10 cm right of the top, 0-2 cm lower; move onto the stick, close; `robot.arc(c, 40,
   axis='x')`, c 6-8 cm below the grip (2 eps, stick 0.70 m ahead, 0.13 m right).
3. Lettuce: `robot.home()`; 'Home: arm BLOCKED' after the push: `robot.goto(by=...)` 16-24 cm back,
   at its own angles (2 eps, one rinse task) or the approach's (1 ep), then home. Drive until the
   colander is 0.37-0.50 m ahead (11 eps). **Take the colander** at pitch 90, yaw 90 on the rim
   straight in front of its centre (robot's side, up to 5 cm off; to_world the rim, 1 ep), 1-2 cm
   inside it, 1-3 cm below its top; held at 2.2-4.1 cm (14 eps); check the Grip. BLOCKED 4-8 cm
   above it: up, 5-7 cm along the rim away from the lettuce, down again (2 eps); closed where it
   stopped, it came up empty or slipped (6 eps). A side-rim hold (0.5 cm, then empty) or a diagonal
   one lost the lettuce (1 ep). Pitch-90 head grasps fell (9 eps).
4. Base at its start pose (7 eps). Colander: lower it into the basin (not at its near edge:
   BLOCKED, 1 ep) with the lettuce, not the colander, over the stream's foot (to_world it on the
   basin floor; 6 cm short failed, 1 ep); slide it level 8-15 cm until the lettuce is under the
   stream, `robot.wait(30)` (4 eps). Head at pitch 45: grasp point on the axis at counter height
   (1 ep). A head dropped in the basin: closed hand 3-4 cm above the floor, slide it (1 ep).
## Verified parameters
- Rim holds survived 0.3-0.6 m drives and 42-43 cm arm moves (5 eps). The push with the stick
  measured 1.5 cm short or long still turned it 19 of 20 (scripted: 20 eps each, faucet task).
## Common failures and fixes
- Lettuce ~6 cm short of the stream's foot; a 6.5 cm forward slide BLOCKED; after letting go, four
  re-grasps came up empty or slipped (1 ep). Head lowered from above into the basin slipped (2 eps);
  20 cells of re-grasps there cost ~750 steps (1 ep). Rim-top tries closed empty (2 eps).
## Typical step costs
- Horizon 1650. Success 491-1517 (14 eps): lever 148-234 when a contact move reached the stick,
  812-1071 after short ones (2 eps); home 71-149; pick 113-207; carry and stream 63-204.
