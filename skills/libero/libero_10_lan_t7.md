# libero_10_lan_t7

## Judging
- What counted: with the blue can and the box in the basket the episode ended with success (18
  eps). "alphabet soup": the can with the blue label and yellow print left of the agentview
  picture's centre (-y), in its lower part: about 0.55-0.59 ahead, 0.08-0.12 to -y, 6 cm wide,
  top 0.096. "cream cheese box": the small flat blue-and-white box just right of the picture's
  centre (+y): 0.34-0.39 ahead, 0.04-0.08 to +y, 7.6 × 3.7 cm, top 0.035.
- The red-and-green can at the far lower left of the picture (+x and -y of the blue can, about
  0.59-0.63 ahead, 0.19-0.21 to -y) never counted as the soup: with it and the box in the basket
  the episode ran out (1 ep).
- 'alphabet soup' found nothing (20 eps); 'cream cheese box' found the box in 8 episodes and
  nothing in 12: point at both.

## Flow
1. Before any motion (free): `s = robot.measure(('agentview', (x, y)))` on the blue can, `b = ...`
   on the box, `robot.measure('basket')`: whole, about 16-17 × 12-15 cm, centred 0.51-0.52 ahead,
   0.23-0.24 to +y (18 eps).
2. Can: above it at 0.20, pitch 90, yaw 0, `grip='open'`; down to 0.07; close (about 6.3 cm); up
   to 0.25 (19 eps).
3. Across at 0.25-0.30 to over 0.49-0.55 ahead, 0.25-0.28 to +y; lower toward 0.12; open where the
   lowering stops; up to 0.25 (19 eps).
4. Box: above it at 0.14-0.21, down to 0.026 at yaw 0, close, up to 0.25, across to over 0.45-0.53
   ahead, 0.23-0.29 to +y, lower toward 0.12-0.16: success came there (14 eps; box first: 4 eps).

## Verified parameters
- Held over the basket, the can or the box stopped going down at 0.120-0.125, "short of target"
  when asked lower (12 eps); opening right there dropped it in (15 eps).
- At the box the close printed "~8.0 cm (holding)"; on the lift the gap went to 4.2 cm and it
  stayed held (20 eps).
- A basket measure of only about 14-15 × 7-9 cm, centred 0.55-0.56 ahead, 0.21-0.22 to +y, covers
  just its camera side (9 eps); use the whole one or the release spots above (19 eps).

## Common failures and fixes
- With the open behind `if r.ok:`, the "short" stop at about 0.12 skipped it (8 eps). Opened at
  that spot in the next reply: in (4 eps). Lifted and lowered again to 0.15-0.17: 38-43 steps
  more (3 eps). Went on in the same cell toward the box with `grip='open'`: the open came at the
  start of that move, 0.27 high, the can landed beside the box, and getting it back ran the
  episode out (1 ep). Fix: open where the lowering stopped, before any other move (15 eps).
- The red-and-green can placed as the soup (1 ep); one more hovered over it first (39 steps)
  before pointing at the blue can (1 ep). Check the blue label before the first move (18 eps).

## Typical step costs
- Horizon 520. Successes 320-419 (18 eps), e.g. hover 24, down 27, close 10, up 26, over 39,
  down 29, open 13, up 24, to the box 28, down 24, close 10, up 28, over 28, down 10: 320 (1 ep).
