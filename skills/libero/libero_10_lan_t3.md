# libero_10_lan_t3

## Judging
- The bottom drawer starts open ~15 cm, its box to +y of the bowl (right of it in the agentview
  picture), its face (a low bar ~4 cm up) looking to -y. Done = bowl inside, drawer pushed shut
  to +y: in our runs the episode ended during that push. A bowl on a drawer wall kept it open.

## Flow
1. Free: `robot.measure('black bowl')` (~9.5 cm across, top ~0.036, y -0.03..-0.07). The bar:
   point at the lowest bar in the agentview picture, around pixel (285, 321), with
   `robot.measure(('agentview', (u, v)))`: ~8 cm long, ~4 cm up, x 0.65-0.66, y +0.05..+0.06
   (2 eps). Call it h. Bars ~0.11 and ~0.18 up belong to the shut drawers above (3 eps).
2. Bowl at y above -0.05: `robot.goto((b.x, b.y - 0.04, 0.12), pitch=90, yaw=0, grip='open')`,
   straight down to 0.024, close: 0.8-0.9 cm on the -y rim (9 eps, 1 probe). Else step 5.
3. Up to 0.165, level to (h.x, h.y + 0.05, 0.165), then down toward 0.05: BLOCKED at ~0.153 is
   normal; open there: flat inside, ~0.02 up, top ~0.063, in 14 of 15 (15 eps, 3 probes).
4. Close it fingertips first, pointing to +y (yaw 90), tilted down: `robot.goto((h.x, h.y - 0.09,
   0.16), pitch=30, yaw=90, grip='close')` (may end short), then (h.x, h.y - 0.04, 0.045), then
   (h.x, h.y + 0.19, 0.045): it stops ~13 cm to +y of h and the episode ends (4 probes, 2 eps).
5. Bowl at y -0.05 or lower: take its -x rim, `robot.goto((b.x - 0.04, b.y, 0.12), pitch=90,
   yaw=90, grip='open')`, straight down to 0.025, close: ~0.8 cm, and it lifts (3 eps, 12 probes).
6. Up to 0.165, level to (h.x - 0.04, h.y + 0.10, 0.165), down toward 0.055. If that stops
   BLOCKED 0.10-0.12 up, open there anyway; then up (6 probes).
7. As step 4 but pitch=45, first point (h.x, h.y - 0.08, 0.10): the episode ended (6 probes, 1 ep).

## Verified parameters
- Every success ended with a push at pitch 20-45, yaw 90, 3-6 cm up (18 eps).
- `robot.measure('bottom drawer')` gave a box well off the table in 19 of 24 tries (24 eps).

## Common failures and fixes
- Bowl at y -0.05 or lower: the yaw-0 descent stopped BLOCKED ~0.08 up in 6 of 7, also on retries
  and at pitch 65-75; tries and switch cost 75-170 steps (7 eps). Go to step 5 at once.
- Yaw 90 over the bowl's middle: stopped ~0.03 up, closes read 0.8-1.3 cm, the bowl slipped out
  on the lift, later half-open tries closed empty; out of steps (1 ep). Use the -x rim.
- After the yaw-90 hold, the pitch-30 push stopped BLOCKED 0.11-0.14 up on its way down (3 probes,
  1 ep): push as in step 7. Let go at h.y + 0.08 instead, a push stuck ~4 cm short (1 probe).
- Pitch 90 pushes to +y stopped after 3-7 cm, the drawer open (8 eps); level ones (pitch 0) were
  BLOCKED with an elbow note (7 eps); a slanted move to the push start was BLOCKED (1 probe).
- A grip that widened to 3.1 cm in the carry left the bowl on a wall ~6 cm up; no push shut the
  drawer (1 ep). Gap notes of 1.7-2.6 cm in the carry did no harm (7 eps).

## Typical step costs
- Horizon 520. Steps 2-4, probe 250: approach 38, down 20, close 12, up 23, carry 24, lower 10,
  open 14, push points 50 + 41, push 18 (1 probe). Steps 5-7, probes 230-261: carry 25-43, lower
  13-22, push points 26-34 + 15 (8 probes). Successes 245-417 now, 359-512 before the note (18 eps).
