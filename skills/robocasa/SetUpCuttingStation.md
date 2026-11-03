# SetUpCuttingStation

## Judging
- Done when the knife and the meat both lie on the board, each well inside it toward its middle,
  and the hand is well away from the knife and the board (rise 30 cm), at once.

## Flow
1. Priors: you face the open top drawer, the knife flat in it, often under the counter top; board
   and plate can be metres off, maybe behind you or higher (22 eps; past a deep counter,
   place at pitch 0-25, 2 eps). Board first: a flat 0.25-0.35 x 0.15-0.2 m rectangle,
   `robot.measure(('left', (px, py)))` (23 eps). Not in view: back 0.35 m (turning: BLOCKED, 3 eps),
   then turn 90 and look, up to 3 times, then drive (4 eps; driving to look: 450-575, 6 eps).
2. Pull the drawer out before reaching in (else 190-775 steps or the episode lost, 10 eps): grip the
   front panel's top edge, 4-5 cm below the counter top, with it 0.28-0.40 m ahead, 0.21-0.32 m to a
   side (7 of 7 in 130-230 steps, else 4 of 5 took 290-640); pitch 90, yaw 90, open, 1 cm below it,
   close, drive back 0.2 m (15 eps; prior tool). BLOCKED: re-grip, `robot.goto(by=(-0.2, 0, 0),
   speed=0.08)` (3 eps). Then open, rise 15 cm, drive (18 eps). A descent at the knife that ends at
   counter-top height: it is still under the top, pull 0.10 m more (7 eps; prior tool).
3. No such edge: point at the bar (by name: wrong ones, 3 eps), 15-18 cm below the counter top,
   0.17-0.30 m right, 0.23-0.40 m ahead; pitch 0, yaw 0, roll 90 at its height from 3 cm in front,
   3 cm in, close, then as above (7 eps).
4. Handle 0.45-0.6 m ahead (19 eps): the dark or wooden end (5 eps), or the end 3 mm higher 5 cm in
   (`robot.to_world`, 4 eps). Pitch 90, yaw = grip_yaw over its middle, down to its `on_z` (there
   or 1+ cm under its top: 11 of 11 held; off the floor, within 0.6 cm of its top: 4 of 6 fell; 15
   eps); close (Grip 2.4-4.0 cm, 22 eps); at 0.08 slide 10-12 cm to you rising 6 cm, lift at 0.10.
5. The knife's middle is 4-9 cm from the grasp toward the blade (9 eps): put the middle, not the
   hand, over the board's centre; let go 2-4 cm up (22 eps; hand 10 cm short: failed, 1 ep).
6. Meat: grasp deep (2.1-3.3 cm under its top held 3 of 3, 1.2-1.9 slid 3 of 5); last 10 cm down,
   first 8 cm up at 0.05 (else 11 of 15 fell, 16 eps); put it 6+ cm off the knife (2 eps).
7. A shrimp 4+ cm wide is curled: 'close' fell 9 of 9 at grip_yaw or up to 70 degrees off (3 eps).
   Held: `robot.gripper(0.35)` (1 ep), or the hand 90 degrees off (2 of 3, 2 eps); lift at 0.05.

## Verified parameters
- Closed within 0.5 cm of the measured top, 12 of 26 fell at once; 1-2 cm under, 1 of 23 (30 eps).
  Descend at 0.05-0.10: full speed stopped 0.4-0.8 cm high (7 eps), slow within 0.2 cm (4 eps).

## Common failures and fixes
- `robot.drive` refused as slipping, or a lift narrowing the gap 1 cm or more: set it down, measure
  again, grasp lower at the new grip_yaw (held 3 of 3); driving on lost it 2 of 4 (5 eps).
- Knife along a drawer wall: open-hand descents stop BLOCKED 6-7 cm above it (3 eps). Gripper 0.5,
  hand 90 degrees off grip_yaw, down on the wall side to the knife's top, 12 cm away at 0.05 (1 ep).

## Typical step costs
- Horizon 2400; successes 573-2309 (28 eps). Plan ~1500 (7 eps): search 0-250, pull 130-230, knife
  130-370, place 130-390, meat 360-770; out of steps (3 eps): search, stance, re-placing, shrimp.
