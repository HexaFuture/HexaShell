# libero_10_object_t1

## Judging
- What counted: with both boxes in the pink basket the episode ended with success (17 eps).
  "cream cheese box": the flat red-and-white box marked "Cream Cheese" in the lower left of the
  agentview picture (+x, -y): 0.59-0.64 ahead, 0.17-0.22 to -y, 7.6 × 3.7 cm, top 0.035.
  "butter": the flat green box marked "Farm Fresh Butter" just right of the picture's centre, in
  its lower part (+y): 0.54-0.59 ahead, 0.03-0.08 to +y, 7.2 × 3.4 cm, top 0.035.
- 'cream cheese box' found the box in 17 episodes and nothing in 3; 'butter' found nothing (20
  eps): point at the green box.

## Flow
1. Before any motion (free): `b = robot.measure('cream cheese box')` or a point on it, `u = ...`
   a point on the butter, `robot.measure('basket')`: about 16-17 × 12-14 cm, centred 0.50-0.52
   ahead, 0.23-0.24 to +y (20 eps).
2. Box measured 0.61 ahead or farther: above it at 0.14-0.23, pitch 90, yaw 0, `grip='open'`,
   straight down to 0.026, close (4.2-4.5 cm), up to 0.25 (11 eps). Nearer: see the first item
   under failures.
3. Across at 0.25-0.30 to over 0.49-0.55 ahead, 0.25-0.27 to +y, lower toward 0.12, open where
   it stops, up to 0.25 (15 eps).
4. Butter: above it at 0.14-0.23, straight down to 0.026 at pitch 90, yaw 0, close, up to 0.25,
   across to over the basket again, lower: the episode ended with success there (16 eps).

## Verified parameters
- The butter's close printed 3.9-8.0 cm "holding"; on the lift the gap went to 3.9 cm and it
  stayed held (17 eps).
- Held over the basket, the box or the butter stopped going down at 0.119-0.132, BLOCKED or
  "short of target" (15 eps); opening right there dropped it in (8 eps).

## Common failures and fixes
- Box measured nearer than 0.61 ahead: the pitch-90 descent stopped BLOCKED at 0.09-0.10 (once
  0.146) with the hand pushed about 2 cm to +x (6 of 9 eps; none of 11 farther out); a close
  there found nothing (4 eps). Fix: hover about 4 cm to +x of its centre at 0.13-0.18, then down
  to 0.025 at a point 2-3 cm to +x of it: reached 0.026-0.028, closed 4.2 cm and held, 75-95
  steps from the block to the lift (3 eps). Pitch 115 from a hover 4-5 cm to +x held too (2
  eps). Turning to yaw 90, pitch 60 or yaw 40 instead kept slipping; it ran out (1 ep).
- With the open behind `if m.ok:`, the stop over the basket skipped it; lifting and lowering
  again to 0.15-0.19 before opening cost 34-52 steps (7 eps). Open where it stopped (8 eps).
- Crossing from the release to the butter at 0.15-0.19 without going up first, the hand caught
  the basket's -y wall and dragged the basket away; freeing it took the rest of the episode
  (1 ep). Go up to 0.25 first, as in step 3 (15 eps).

## Typical step costs
- Horizon 520. Successes 327-397 (16 eps), 507 after a butter block (1 ep). Box: hover 33-42,
  down 25-34, close 10, up 27-29, across 44-50; butter: down 22-45, close 10, up 27-30 (13 eps).
