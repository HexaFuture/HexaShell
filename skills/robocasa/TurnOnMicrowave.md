# TurnOnMicrowave

## Judging
- Done when START has been pressed and the hand has then backed well away from it. A light press
  was enough; success came while backing away (11 eps).
- STOP turns it off again; leave the door as it is. The object inside does not matter.
- Still running after a straight 20 cm retreat: START was missed, or STOP was pressed.

## Flow
1. Priors: the robot stands close to the front. The buttons are flush with the front, low on the
   right-hand control panel, small (a few cm) and close together. START is left of
   STOP on most models, right of it on some, and the two are stacked on others.
2. Read the labels: `robot.look('right', crop=..., zoom=5)` on the panel's lower part. In an
   earlier batch START had a blue mark, STOP/CLEAR a red one; words found nothing (1 ep).
3. `x, y, z = robot.to_world('right', (px, py))` on START's middle: ~0.3-0.45 m ahead of the base,
   ~1.15-1.5 m high (10 eps). No drive needed (10 eps).
4. Pre-press: `robot.goto((x - 0.10, y, z), pitch=0, yaw=0, roll=0, grip='close')` (10 eps).
   If a dry run says NOT reachable, pitch 10 was reachable (1 ep). Keep the roll near the panel.
5. Take a wrist picture there and `robot.to_world('wrist', ...)` START's middle again; use that
   point. It matched within 3 mm (8 eps), or moved START 3.4-5 cm sideways (2 eps).
6. Press: `robot.goto((x + 0.015, y, z))`. "Short of target" ~1 cm before the face is normal
   (8 eps); BLOCKED 4-5.5 cm short also counted as a touch (2 eps).
7. Retreat: `robot.goto(by=(-0.20, 0, 0))`; success fired on the way (10 eps).

## Verified parameters
- Closed gripper, pitch 0, yaw 0, roll 0, square to the panel from the start pose; pre-press 10 cm
  in front; press target 1.5 cm past the face; retreat 20-22 cm straight back (10 eps).
- Pitch 10 for a pre-press that pitch 0 could not reach (1 ep).

## Common failures and fixes
- A second press with roll 90, and the turn back to roll 0 at 9 cm from the panel, caught the
  door's handle edge: the door came ajar and the hand stuck 3 cm short (1 ep).
- After a press, back off 20 cm before deciding it failed; press again only on START (1 ep).

## Typical step costs
- Horizon 450. 61-87 used (10 eps): looks and to_world 0, pre-press 27-45, re-aim 9-11 (if
  needed), press 20-35, retreat 12-14. Aim for 90 or less.
