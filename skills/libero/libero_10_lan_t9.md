# libero_10_lan_t9

## Judging
- `robot.measure('yellow and white mug')` found the mug at y 0 every time, 0.63-0.68 ahead,
  within 3 cm of y 0, top 0.09 (10 eps); put inside with the door shut, it ended the episode
  with SUCCESS (1 ep, 3 probes). The other mug, at -y, is not moved.
- The microwave stands at +y (right in the agentview picture), opening toward -y; its door hangs
  on the opening's -x edge and swings out toward -y. The words 'microwave' found nothing (10 eps).
- SUCCESS came as the closing arc stopped against the shut door, the mug inside (3 probes).

## Flow
1. Free first: `m = robot.measure('yellow and white mug')` (10 eps). Hinge hx, hy: to_world on
   the edge where the door meets the box, 0.48-0.50 ahead, 0.25-0.275 to +y (17 eps).
   Door: a to_world spot ex, ey on its free vertical edge (middle of the agentview picture) lies
   0.23-0.26 m from the hinge; open by `a = -math.degrees(math.atan2(ey - hy, ex - hx))` (18 eps).
2. Rare here (97-112 by the spots, 4 eps): if a < 90 the arm meets the door, so push it wider
   first (2 probes): `t = math.radians(10 - a)`; closed hand, pitch 90, yaw 90 - a, over
   `(hx + 0.20*math.cos(t), hy + 0.20*math.sin(t), 0.25)`; `robot.goto(by=(0, 0, -0.10))`;
   `robot.arc((hx, hy, 0.15), -40, axis='z')`; close as for a + 35, up in place first (3 probes).
3. `robot.goto((m.center[0] - 0.04, m.center[1], 0.16), pitch=90, yaw=90, grip='open')`, down
   to 0.075, `robot.gripper('close')` on the mug's -x wall, up to 0.20 (3 probes).
4. Carry and turn in one move: `robot.goto((hx + 0.145, hy - 0.13, 0.125), pitch=0, yaw=90,
   roll=90)`; in to hy + 0.03 with speed=0.1; `robot.gripper(0.6)`; out to hy - 0.13 (3 probes).
5. Close: `t = math.radians(-a)`;
   `P = (hx + 0.20*math.cos(t) + 0.05*math.sin(t), hy + 0.20*math.sin(t) - 0.05*math.cos(t))`;
   from there straight over P at 0.26 with pitch 90, yaw 90 - a, grip='close'; down to 0.15;
   `robot.arc((hx, hy, 0.15), a + 19, axis='z')`: it ended short or BLOCKED as the door shut,
   with SUCCESS (3 probes; the same close worked in 6 probes of the sibling variants).

## Verified parameters
- Held by its -x wall, the mug turns with the hand: at pitch 0, yaw 90, roll 90 it went in
  (3 probes), at roll -90 it was BLOCKED (probe). `robot.gripper(0.6)` let it go (3 probes);
  with `robot.gripper(0.3)` instead no SUCCESS came (probe).

## Common failures and fixes
- Insert tries BLOCKED again and again (5 eps): the side-lying mug of step 4 went in at the first
  try (3 probes).
- The mug slipped out of the grasp (2 eps): close on the -x wall (3 probes).
- Door arcs came late or were BLOCKED (6 eps): the arc of step 5 shut it in 39-43 steps
  (3 probes). The one success held the mug by its handle (1 ep).

## Typical step costs
- Horizon 520. Over the mug 45, down 15, close 11-13, up 20-21, carry and turn 52, in 53-62, open
  10, out 23-24, over P 43-45, down 18, arc 39-43: 331-344 (3 probes); a push adds 61-65 (2 probes).
  The one learn1 success took 508 (1 ep).
