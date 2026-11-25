# tabletop_grasp

## Judging
- "The object" is the red chips can lying on its side across the table, just ahead and near the
  pelvis's line; the other things are only there besides it (scripted 20/20 lifted the can).
- Lifting it a few centimetres counted at once (3/3 runs that held it); the official limit is
  800 steps.

## Flow
1. `robot.measure('red can')` from the start: about 21 cm long seen from above (it is a little
   longer), its top about 0.86 m high, its centre 0.43-0.47 m ahead (4/4 runs). 'red can' found
   it in 3 layouts but not in L3 (scripted), and 'object on tabletop' outlined something 3 m away
   (L3): then point at the can in the picture, `robot.measure(('gaze', (x, y)))` (L3).
2. `robot.body(0.75, pitch=15)` (12-24 degrees): standing, every grip point was NOT reachable,
   1.4-7 cm short (4 runs). Both palms rest at the table's edge at the start: raise the right
   hand before it goes over the table.
3. End grip on the can's right end: `robot.fingers('right', 'wide')`, the right palm
   `facing='inward'` (fingers forward) 2-4 cm right of the grip point, then in to it: the palm
   about 2 cm outside the measured right end, about 2 cm nearer than the measured centre, about
   1 cm below the top (L6). The scripted grip, 20/20, had it 2.5 cm below the top (scripted).
4. `robot.fingers('right', True)`: holding with the thumb and the middle finger is enough here
   (scripted 20/20).
5. `robot.hand('right', by=(0, 0, 0.1))` to `0.12`: it counted on the way (3/3 runs that held it).

## Verified parameters
- From above it worked once: palm down 3 cm nearer than the measured centre and only 2 cm above
  the top, then closed (L4, done at step 173). Done the same way by a script it closed on
  nothing (scripted); see below.

## Common failures and fixes
- Palm down 3 cm or more above the top: closed on nothing (3 runs: L3 twice, L5), twice with the
  can rolled 4-5 cm away. Grip the end instead (step 3).
- Moving the hand from its start straight to a point over the can brushed the can on the way
  (scripted). Lift the hand first, then go over.

## Final checks
- None needed: the episode ends by itself on success (4/4 runs).

## Typical step costs
- Lean 30-50, the two hand moves 100-150, close 25, the lift a few steps: done at steps 173 and
  196 (2 runs) and 226-268 (scripted 20/20) of the official 800. A run that missed three times
  took 1241 (L3).
