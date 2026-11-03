# libero_10_object_t9

## Judging
- `robot.measure('yellow and white mug')` found a mug at y 0: round, 0.63-0.68 ahead, within
  3 cm of y 0, top 0.10 (10 eps). Put inside with the door shut, that one ended the episode with
  SUCCESS (1 ep, 2 probes); the other mug, at -y, was not moved.
- The microwave stands at +y (right in the agentview picture), opening toward -y; its door hangs
  on the opening's -x edge and swings out toward -y. The words 'microwave' found nothing (9 eps).
- SUCCESS came as the closing arc stopped against the shut door, the mug inside (2 probes).

## Flow
1. Free first: `m = robot.measure('yellow and white mug')` (10 eps). Hinge hx, hy: to_world on
   the edge where the door meets the box, 0.48-0.50 ahead, 0.25-0.275 to +y (17 eps).
   Door: a to_world spot ex, ey on its free vertical edge (middle of the agentview picture) lies
   0.23-0.26 m from the hinge; open by `a = -math.degrees(math.atan2(ey - hy, ex - hx))` (18 eps).
2. If a < 90 the arm meets the door on the way in: BLOCKED (probe). Push it wider first
   (2 probes): `t = math.radians(10 - a)`; closed hand, pitch 90, yaw 90 - a, over
   `(hx + 0.20*math.cos(t), hy + 0.20*math.sin(t), 0.25)`; `robot.goto(by=(0, 0, -0.10))`;
   `robot.arc((hx, hy, 0.15), -40, axis='z')`; then close as for a door at a + 35 (probe).
3. `robot.goto((m.center[0] - 0.04, m.center[1], 0.16), pitch=90, yaw=90, grip='open')`, down
   to 0.078, `robot.gripper('close')` on the mug's -x wall, up to 0.20 (2 probes).
4. Carry and turn in one move: `robot.goto((hx + 0.12, hy - 0.13, 0.125), pitch=0, yaw=90,
   roll=90)`; in to hy + 0.03 with speed=0.1; `robot.gripper(0.6)`; out to hy - 0.13 (2 probes).
5. Close: up in place to 0.24 with pitch=90, yaw=90 - a, grip='close'; `t = math.radians(-a)`;
   `P = (hx + 0.20*math.cos(t) + 0.05*math.sin(t), hy + 0.20*math.sin(t) - 0.05*math.cos(t))`;
   over P at 0.24-0.26, down to 0.15, then `robot.arc((hx, hy, 0.15), a + 19, axis='z')`: it
   ended short or BLOCKED as the door shut, with SUCCESS (2 probes; 7 in the sibling variants).

## Verified parameters
- Held by its -x wall, the mug turns with the hand: at pitch 0, yaw 90, roll 90 it went in
  (2 probes), at roll -90 it was BLOCKED (probe). `robot.gripper(0.6)` let it go (2 probes);
  with `robot.gripper(0.3)` instead no SUCCESS came (probe).

## Common failures and fixes
- Insert tries BLOCKED again and again (4 eps): push a door under 90 first, then step 4 (probes).
- Grasps 7.4-7.7 cm wide across the body, or slips (4 eps): close on the -x wall (2 probes).
- Door arcs came late or were BLOCKED (4 eps); twice the mug was inside with too few steps left
  to close (2 eps): use step 5. The one success swept the door shut by arcs, 65 then 32 (1 ep).
- With the door at 94 by the spots, going from the exit straight over P saved about 35 steps
  (probe); after a push, the descent at P that way was BLOCKED (probe).

## Typical step costs
- Horizon 520. Push 61, over the mug 55, down 15, close 12, up 20, carry and turn 52, in 38,
  open 10, out 24, up and turn 46, over P 34, down 16, arc 43: 426 (probe); with no push and
  straight from the exit to P: 317 (probe). The one learn1 success took 484 (1 ep).
