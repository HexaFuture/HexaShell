# OpenStandMixerHead

## Judging
- Open = the head tipped right back to its upper stop (most kinds about upright); the episode
  ends there. Pushed far enough (mostly past about 60 degrees) it goes on by itself; short of
  that, it falls shut when the hand leaves it (prior tool; falls: 7 eps).
- Hinge along the robot's forward axis; the free end points left (s = +1), in a few kinds right
  (s = -1): see which in the pictures. A cabinet low over the head or a machine at its free end
  stops its swing short: those kitchens do not open (scripted: 2 of 20 eps).

## Flow
1. `robot.gripper('close')`. `robot.to_world` in left and right on the head's top 3-5 cm from the
   free end (top), on the END face 2-4 cm under the top (end), on the head's back end over the
   column (back); end to back ~0.2-0.35 m. P: x of the top point, y of the end, z = top - 0.035;
   H: the same x, back + 0.02s, z = top - 0.12 (prior tool, 90 calls; scripted: 20 eps).
2. Turn the hand high up: `robot.goto((x, y, top + 0.10), pitch=0, yaw=-70*s, roll=180)`, x and y
   of `robot.state().at`: fingers 20 degrees turned toward the robot (scripted: 17 of 20 eps).
3. Stand: d = P_y + 0.03s - y of `robot.state().base`; |d| over 0.16: `robot.drive(left=d -
   0.10s)`; stopped short (BLOCKED): go on from there (scripted: 3 eps, 1 opened); no forward drive.
4. Over: `robot.goto((P_x, P_y + 0.07s, top + 0.10), pitch=0, yaw=-70*s, roll=180)`; down slowly:
   `robot.goto((P_x, P_y + 0.05s, P_z), speed=0.08)` (scripted: 20 eps).
5. `p = robot.push_arc(H, 95*s, axis='x', push=(0, -s, 0), press=0.05, tilt=35)`: it goes in to
   the face, presses, turns the hand with the head and fits the hinge; it opens and the episode
   ends (scripted: 17 of 20 eps). Do not push by hand with `goto(by=)` (10 eps: 1 opened).
6. p.stop 'no touch': nothing within 12 cm, P is off: look, measure the end face again, steps 4-5
   (untested). 'slid', 'jammed' or 'stalled' under 60: the head fell back; 200+ steps left: back
   up 5 cm along +s y, steps 4-5 (1 ep: at 56 degrees when the steps ran out).

## Verified parameters
- End face from 1-5 cm under the head's top down to 8-13 cm under it; push point 80 % up it, 1.6+
  cm under its top edge (prior tool, 90 calls). push_arc fits the hinge as the head turns: guesses
  1-12 cm off opened (scripted: 16 eps).
- press 0.05, the most: as many opened as 0.03, in 160 against 190 steps (scripted: 8 of 10 eps
  each). Roll 180 not 0, tilt 35 not 25, yaw -70s not -50s: 16/15, 17/16, 17/17 of 20 (scripted:
  20 eps each). At tilt 25 or yaw -50s the pressed hand can drive the wrist through 180 degrees to
  its limit and the head falls: agent pushes 4 of 4 at 35, 9 of 10 at -70s (replayed: 14 eps).

## Common failures and fixes
- A straight hand, yaw -90s, struck itself on the way down after the drive (BLOCKED, forearm on
  the wrist) (scripted: 3 of 20 eps); turned 20 or 40 degrees it did not (scripted: 20 eps each).
- Some heads have a thin skirt low at the front, up to 13 cm past the end face: an end measured on
  it puts P in the air and the push meets nothing (scripted: 2 eps). Measure on the face itself.

## Typical step costs
- Horizon 450. Steps 1-4: 110-155; push_arc 75-150 to open; 185-300 all told (scripted: 17 eps).
