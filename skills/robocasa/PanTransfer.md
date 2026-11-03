# PanTransfer

## Judging
- Done when the vegetable lies on the plate near its middle, the pan's round part stands back
  over a burner (its start burner is fine) and the hand is well away from the pan, all at once.
  Knobs and flames do not matter.
- Never touch the vegetable with the gripper, not even once: move it only by tipping the pan.
  There is one vegetable, despite "vegetables".
- Still running after the release: the hand is still near the pan, the pan is off its burner,
  or the vegetable rolled off the plate.

## Flow
1. Priors: the robot faces the stove's middle, close in front; the handle points roughly toward
   the robot; the plate is on the counter just left or right of the
   stove, at its front edge, often out of reach from the start. The plate cannot be grasped.
2. `robot.measure('pan')`, `'pan handle'`, `'plate'`; 'vegetables' was NOT FOUND: name it by its
   look, or `robot.to_world` on it.
3. Handle: pitch 90, yaw = the handle's grip_yaw, 13 cm above it; down to its centre height;
   `robot.gripper('close')`; `robot.goto(by=(0, 0, 0.19))`.
4. Drive sideways toward the plate, the pan held level; put the pan's middle over the plate's
   middle, the grasp point ~0.19 m above the plate top.
5. Pour: `to_world` on the vegetable; shift the pan until the vegetable is 3-4 cm from the plate's
   centre on the side AWAY from the tip. `axis` = the level direction from the grasp point to
   the pan's centre, `pivot = tuple(robot.state().at)`; `robot.arc(pivot, 60, axis=axis)`:
   + tips the opening to the RIGHT of that line, - to the left. Look, then arc -60 back.
6. Up 0.1 m, drive back, 13 cm over the start grasp spot with the same yaw, down to ~1 cm above
   the start grasp height (the pan lands on its burner again), open, lift 0.25 m.

## Verified parameters
- Handle 15 x 1.4 cm: pitch 90, yaw = grip_yaw (-26), at its centre height (0.2 cm below its
  top): holding 1.8 cm; it held the pan through a 0.19 m lift and a 0.55 m drive (1 ep).
- The plate was 8.6 cm out of reach from the start base; `robot.drive(left=0.55)` fixed it (1 ep).
- The vegetable sat 4 cm right of the plate's centre; `goto(by=(0, 0.075, 0))` put it 3 cm left;
  arc +60 about the grasp point (axis heading -26; 39 steps, "near a joint limit", the hand at
  pitch 30, yaw 64, roll 90) dropped it ~17 cm onto the plate; it stayed. Arc -60: 38 steps (1 ep).
- Let go 0.8 cm above the start grasp height: success fired during the 0.20 m lift (1 ep).

## Common failures and fixes
- None seen (1 ep). If the vegetable stays in the pan, tip further (untested); never toward the
  handle, where it would roll at the fingers. A round one may roll off: tip near the plate's middle.

## Typical step costs
- Horizon 1800, the shortest in this group. 1 ep took 422 in 10 replies: grasp and lift 100, drive
  41, over the plate 46, shift and pour 92, return and release 143. Aim for ~500.
