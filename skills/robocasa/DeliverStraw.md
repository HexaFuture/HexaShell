# DeliverStraw

## Judging
- Done when the straw touches the glass cup while the hand, after the release, is well away from
  the straw. The drawer does not matter. Still running: not touching yet.
- **A straw dropped on the floor is lost** (too low to grasp): 2/2 eps ended so, ~1570 steps left.

## Flow
1. Priors: the drawer's bar is 0.15-0.3 m ahead at the start (3 eps); the straw (16-19 cm long,
   1-1.6 cm thick, some bent) lies flat in its front half. The glass cup (6-10 cm wide, 10-15 cm
   tall, tips easily) is on the dining counter on the stools' side, maybe far away.
2. Bar: measure it by pointing, `robot.measure(('left', (x, y)))`. `robot.drive(forward=-0.45,
   left=0.28)`: base 0.6-0.75 m behind it, 0.25-0.3 m to one side (3 eps).
3. **Roll by side: bar on the base's right: roll 90; on its left: roll -90** (3/3). Pre-pose
   10-30 cm in front of the bar, 3-10 cm above it, pitch 20-35, yaw 0, grip='open'; pitch 15 at
   its height 10 cm in front; onto it; close.
4. `robot.goto(by=(-0.25, 0, 0))`, `robot.drive(forward=-0.23)` holding; open; lift away with
   `robot.goto(by=(-0.05, 0, 0.2), pitch=65, roll=0)`. The straw then lay in reach (3/3).
5. Find the cup now, before taking the straw (untested): back out, turn in 90-degree steps and
   look ('glass cup', 'clear glass' or point at it); plan the legs to ~0.5 m from it; come back.
6. Straw: measure by colour ('pink straw', 'red straw': 2/2); base 0.5-0.6 m behind it. Hover
   15 cm above, pitch 90, yaw = grip_yaw; to_world on both ends and the drawer bottom. **Grasp
   point at the ends' midpoint, 1.0 cm above the drawer bottom** (`.on_z` + 0.01); want 0.8+ cm.
7. Lift 5 cm and look: the straw must stay level. Hanging or tilted: set it down and regrasp
   nearer its middle (untested). Then up 0.3 m.
8. Carry in few straight legs and few turns. Place (untested): base square to the counter, cup
   ~0.5 m ahead; pitch 90, the straw's middle over the mouth, 1-2 cm above the rim; open; up.

## Verified parameters
- Bar: pitch 15, yaw 0, roll 90 (bar on the right): 5.4 cm; roll -90 (bar on the left): 3.5-5.2
  cm; the pull halves the gap, fine (3 eps).
- Straw: grasp point 1.2 cm above the bottom: 1.0 cm, came up hanging from the fingertips, fell
  in a turn (1 ep); 1.3-1.9 cm above: 0.6 cm edge pinch, slipped on the lift (another ep).

## Common failures and fixes
- The other roll sign stopped at the wrist's limit, then home BLOCKED: 180-250 steps (3 eps).
- A back-up or turn BLOCKED right after the drawer: something behind; drive forward 0.2 m (1 ep).
- If the straw keeps slipping (untested): carry the cup (one finger inside, by its wall) to the
  counter by the drawer first; then the straw needs only a short move.

## Typical step costs
- Horizon 2550, so steps are not the limit; drops are. Lean drawer ~280 (used 478 and 555).
- Plan: drawer 300, cup search 300, straw 200, carry 400, place 150: about 1350.
