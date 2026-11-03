# KettleBoiling
## Judging
- Done when the kettle stands on a burner near its centre, THAT burner is on (its knob turned
  either way), and the hand is well away from the kettle, all at once.
## Flow
1. Priors: the kettle stands at a counter's front edge, left or right of the stove. Locate
   it before the knob: in no start picture, drive 0.65 m to each side in turn and look (3 eps).
   Beside the base on a side counter: reach it by arm; drives there were BLOCKED ~300 steps (1 ep).
2. Knob first (prior tool; 5 eps): the free front burner on the kettle's side, by the outermost
   burner knob on that side; the arm can hide part of the row: slide the base 0.2 m to see it all
   (prior tool). Put the kettle on the burner that lit (blue flame or red ring).
3. Front-panel knob k: heading 0, k 0.53-0.67 m ahead, in line or up to 0.14 m left (lit: 5 eps in
   line, 4 at 0.12-0.14 m left), e.g. `robot.drive(to=(kx - 0.60, ky - 0.12), heading=0)`. Pre-grasp
   at its height, 12 cm nearer the base: `robot.goto(k, by=(-0.12, 0, 0), pitch=45, yaw=0, roll=0,
   grip='open')` (`dry_run=True` first); `robot.goto(k)`; close (2 eps). From beyond it, straight
   above it, or often diagonally from above, the hand stopped on the stove (6 eps).
4. At pitch 45 the fingers are not along the knob's axis: `robot.arc(c, -30, axis='x')` with c =
   `robot.state().at` once closed, twice (lit 5 eps); in line, `robot.goto(roll=-90)` turned it at
   once in 5 eps, stopped early in 7 (2 had lit it). Open, back 10 cm out, look (prior tool).
   - On the cooktop by its front edge: pitch 90, yaw 0 onto it; close; `robot.goto(yaw=65)` (6 eps).
   - Raised panel: base 0.70-0.74 m behind it; pitch 30-45; close on the whole knob (2 eps).
5. Kettle: measure 'kettle handle'. A bar over the top: pitch 90, yaw = grip_yaw, grasp point
   1-2.8 cm below its top (23 eps). Only a side handle: the knob on the lid, pitch 90, yaw 0 at
   heading 0, 1-1.4 cm below its top (1 ep; prior tool); side or body grips slid on lifts (2 eps).
6. Close; up 5 cm; back 15 cm toward the base; drive sideways (13 eps) with its bottom ~10 cm above
   the grate (prior tool): 14-20 cm lifts 0.40-0.45 m out pressed on something above (2 eps).
7. Hand over the burner's centre, 0.39-0.49 m ahead of the base (5 eps): the bar or lid knob sits
   over the kettle's middle (prior tool). Lower to the burner's height + 0.01 + (grasp height -
   the kettle's `.on_z` before the pick); a stop 1-4 cm short: it touched down (9 eps).
8. Open; straight up `robot.goto(by=(0, 0, 0.15))`, back `by=(-0.15, 0, 0)`, `robot.home()` (prior
   tool); a diagonal back-off, `robot.home()` or a drive first dragged it off (3 eps). Rise BLOCKED:
   back out level kept a bar handle (3 eps), dragged a lid knob; rising 6 cm first kept it (1 ep).
## Verified parameters
- Let go 1-6 cm from the burner's centre: stayed and counted (6 eps); 9.5 cm off: toppled (1 ep).
## Common failures and fixes
- Step 2's outermost knob can be a dial (more knobs than burners: oven or clock dials sit at an
  end or mid-row). No flame or ring after two turns (a stalled or BLOCKED turn can still light it:
  look), or the gap fell 2+ cm in a turn: go inward knob by knob till one lights (1 ep; probe).
- A turn "NOT executed" or stopped at joint 5, 6 or 7: the same grasp at roll 180 (reached 3 eps,
  lit 1) or the other stance; a knob five tries at 0.12 m left missed lit at once in line (1 ep).
- Pitch-90 closes on a top grip came back empty 6 times (4 eps); then held 1.4 cm lower (1 ep),
  after `robot.drive(forward=-0.18)` (1 ep), at pitch 45 yaw 0 roll 90 or pitch 70 (2 eps).
## Typical step costs
- Horizon 1500. Successes 336-1314 (22 eps, median ~585). Knob ~120 steps on the cooktop (2 eps),
  170-730 on a panel (5 eps), 960-1140 when it never lit (2 eps); pick to let-go 255-540 (7 eps).
