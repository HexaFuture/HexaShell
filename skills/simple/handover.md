# handover

## Judging
- It counted as soon as the box, still in the left hand, met the container for a moment; letting
  go was not needed (2/2 runs, both ended on the way down to it).
- The container is the basket on the left, about 0.4 m left of the box, its rim about 7 cm lower
  than the box's top (4/4 runs). `robot.measure('basket')`; a 'container' search once outlined
  the box itself (L4) and once found nothing (L6): check its centre is to the left.

## Flow
1. `robot.measure('cracker box')`: upright on the table just ahead, its top about 0.76 m high,
   its long side across (4/4 runs). Measure the basket too.
2. `robot.body(0.65, pitch=15)`: standing, the right grip point was NOT reachable, 6-7 cm short
   (3 runs).
3. `robot.fingers('right', 'wide')`; the right palm `facing='inward'` 3 cm right of its grip point,
   then in to it: 1.5-2.5 cm outside the box's right end, level with its centre front to back,
   5-7 cm below its top. `robot.fingers('right', True)` (4/4 runs held).
4. Lift it straight up 16-19 cm in the right hand, to a palm height of about 0.87 m (4/4 runs).
   Look: if the box leans in the hand (measured 6-7 cm tall instead of 12-18; 3 runs), move the
   right palm again to where it is, `facing='inward'`: the box came level, and the left held at
   the first try (L7, L9). Against the leaning box the left missed 2-3 times (L7, L9, L10).
5. `robot.fingers('left', 'wide')`; the left palm `facing='inward'` 4-5 cm left of the box's left
   end (a `via` point up and out to the left on the way, L4), then in against that end, about as
   high as the right palm. `robot.fingers('left', True)` and check `.holding` and what touches.
6. Only with the left hand holding: `robot.fingers('right', False)`, then the right hand out
   (`by=(0, -0.13, 0.04)`; 37-51 steps). Left open where it was, it did not hold the box back
   (scripted 60/60).
7. The left hand over the basket, `seconds` 0.6-0.7: the palm 0.82-0.90 m high, 2-7 cm left of
   the basket's centre and 3-5 cm nearer than it. It counted on the way (2/2 runs).

## Verified parameters
- Closed holding with thumb, index and middle touching, the left hand kept the box when the right
  let go (3/3 runs).

## Common failures and fixes
- The left hand closed with only the thumb and one finger touching: the box fell when the right
  let go (L5, L10). Open the left again and go 1-2 cm deeper before opening the right.
- Turning the torso (`yaw=30`) with the box in the left hand: it slipped out (L3). Move the hand,
  not the body: the basket is within the left arm's reach from where the robot stands.
- Once on the table or the floor the box was never picked up again: reaching low and left the
  left wrist hit its limits (2/2 runs).

## Final checks
- None needed: the episode ends by itself on success (2/2 runs).

## Typical step costs
- Lean about 80, right grasp 120-170, lift about 40, left grasp 140-210, right off 80-110, to the
  basket 35-40: done at 638, 650 and 660 of the official 800 (3 runs). Looks and checks of
  `.holding` cost nothing; each extra left regrasp cost 100-200: done at 1036 and 1289 (L9, L7).
