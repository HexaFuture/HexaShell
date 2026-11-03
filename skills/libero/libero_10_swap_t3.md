# libero_10_swap_t3

## Judging
- The bottom drawer starts open ~15 cm, its box to -y of the bowl (left of it in the agentview
  picture), its face (a low bar ~4 cm up) looking to +y. Done = bowl inside, drawer pushed shut
  to -y: in our runs the episode ended during that push.

## Flow
1. Free: `robot.measure('black bowl')` (x 0.47-0.52, y +0.025..+0.075, top ~0.037) (10 eps).
   The bar: point at the lowest bar in the agentview picture, around pixel (220, 277), with
   `robot.measure(('agentview', (u, v)))`: ~8 cm long, ~4 cm up, x 0.55-0.56, y about -0.06
   (2 eps). Call it h. 'drawer handle' gave a shut drawer's bar ~0.107 up (2 eps).
2. `robot.goto((b.x, b.y + 0.04, 0.12), pitch=90, yaw=0, grip='open')`, straight down to 0.023,
   `robot.gripper('close')`: 0.8-0.9 cm on the +y rim (7 eps, 2 probes).
3. Up to 0.165, level to (h.x, h.y - 0.05, 0.165), down toward 0.05: BLOCKED at ~0.152 is
   normal; open there. The bowl lands flat inside, resting ~0.02 up (7 eps, 2 probes).
4. Close it fingertips first, pointing to -y (yaw -90), tilted down, roll 180: closed hand,
   pitch=40, yaw=-90, roll=180 (the same hand as pitch=140, yaw=90): (h.x, h.y + 0.09, 0.16),
   then (h.x, h.y + 0.04, 0.045), then (h.x, h.y - 0.19, 0.045): it ends short ~13 cm to -y of h
   and the episode ends (2 probes; 5 eps closed at pitch 20-45 roll 180 or pitch 135-150 yaw 90).

## Verified parameters
- `robot.measure('bottom drawer')` found the open box (x 0.55-0.57, y -0.14..-0.16, top ~0.072)
  in 6 of 6 tries (6 eps); the bar runs along its +y edge, ~10 cm from its centre (2 eps).
- Grasp: hand 3.1-4.3 cm to +y of the bowl's centre, 0.023-0.027 up (7 eps).

## Common failures and fixes
- Pitch 90 pushes to -y stopped after 2-10 cm, the drawer still open (10 eps).
- Pointing to -y with roll 0 (pitch 0-35, yaw -90) ran into the wrist's twist or bend limit or
  the elbow, though dry runs said reachable (9 eps). Level with roll 180: BLOCKED (4 eps).
- Held by its -x rim (yaw 90), lowered to 0.11 with its centre ~11 cm to -y of the bar: it tilted
  against the drawer's -y end; no push closed the drawer (1 ep). Use the flow's grasp (2 probes).
- Three failures ran out of steps while trying close angles; `robot.home()` plus one move cost
  138 steps (3 eps).

## Typical step costs
- Horizon 520. Probes 209-234: approach 24-25, down 17-36, close 12-13, up 22, carry 24-26,
  lower 10, open 14, push points 48-49 + 21, push 17-18 (2 probes).
- Successes 405-490, of which the close took 157-344 (7 eps).
