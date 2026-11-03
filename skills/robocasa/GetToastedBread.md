# GetToastedBread

## Judging
- First toast it: press its half's lever right down to the bottom with the slice in; the toaster
  then holds the lever down for a while, till it pops up.
- Once it has popped up, the slice on the plate, well inside its rim, let go, the hand well away
  from it.
- Clue: while on, the slice sinks till its top is about level with the toaster's top; still well
  out right after the press = not started. Still running at the end: off the plate, or the hand
  still near it.

## Flow
1. Priors: the robot faces the toaster; the slice stands in a front-back slot; `measure` gave it
   1.2-2 cm thick (10 eps), 6-9 cm of it above the slot (6 eps). On a two-lever toaster press the
   lever on its half. The plate (`measure`: 17-23 cm across, 10 eps) is at the stools' edge of
   the dining counter, maybe far away.
2. Press: measure 'toaster lever' (a small tab on the front face, ~2 cm below the top, 1 ep). Use
   the fingertips from in front (untested): closed hand, pitch 45, yaw 0, grasp point ~1 cm above
   and ~1 cm in front of the tab's top; then `robot.goto(by=(0, 0, -0.07))`. Lift off; measure
   the slice: its top should drop to about the toaster's top.
3. Down: use the wait (untested): note `robot.state()`, find the plate (turn, look), come
   back with `robot.drive(to=(x, y), heading=h)`; else `robot.wait(...)` in chunks, with looks.
   If the slice is not back up afterwards, lift the tab from below (untested).
4. Grasp from in front, not from above: pitch 15, yaw 0, roll 0, grasp point ~3 cm below the
   slice's top; close; holding 2-3 cm.
5. Lift straight up 18 cm before any other move; hand in; `robot.drive(forward=-0.45)`; turn at
   most 90 degrees per drive call (untested), checking holding after each.
6. Over the plate's centre, lower until the slice's bottom is ~1 cm above it; open; up 0.3 m.

## Verified parameters
- Pitch-90 hovers 12-21 cm above the slice: BLOCKED 5 times (likely the wall cabinet above) (1 ep).
- Pitch 45, yaw 0, 2-2.4 cm below the slice's top: 2.2-2.5 cm, but the lift stopped after 6 cm
  and it slipped as the hand moved back (2/2).
- Pitch 15, yaw 0, 2.9 cm below the top of a half-raised slice: 2.9 cm (2.2 after the lift);
  held through an 18 cm lift, a 20 cm move back and `drive(forward=-0.45)` (1 ep).

## Common failures and fixes
- A press from above (closed hand, pitch 90, yaw 0) stopped 3.2 cm down, the hand's body on the
  toaster's top edge (picture); the toaster never started (1 ep). Press with the fingertips.
- `robot.drive(turn=160)` with the slice in hand dropped it on the floor, out of reach (1 ep).
- Pulling it back while still in the slot slipped it (2 times): lift clear first.

## Typical step costs
- Horizon 3000. 1 ep stuck at 1168, toaster never on. Plan ~1500: press 150, the wait (find the
  plate meanwhile), grasp and lift 150, drive 400, place 150.
