# libero_10_task_t9

## Judging
- "white mug" is the plain white one standing alone at -y (near the left edge of the agentview
  picture): pointed at there it measured 0.53-0.59 ahead, 0.23-0.29 to -y, round, top 0.10; the
  words 'white mug' found the yellow-and-white mug at y 0 instead (8 eps). Put inside with the
  door shut, the plain white one ended the episode with SUCCESS (1 ep, 4 probes).
- The microwave stands at +y (right in the agentview picture), opening toward -y; its door hangs
  on the opening's -x edge and swings out toward -y. The words 'microwave' found nothing (9 eps).
- SUCCESS came as the closing arc stopped against the shut door, the mug inside (4 probes).

## Flow
1. Free first: `m = robot.measure(('agentview', (x, y)))` on the white mug (10 eps). Hinge hx, hy:
   to_world on the edge where the door meets the box, 0.48-0.50 ahead, 0.25-0.275 to +y (17 eps).
   Door: a to_world spot ex, ey on its free vertical edge (middle of the agentview picture) lies
   0.23-0.26 m from the hinge; open by `a = -math.degrees(math.atan2(ey - hy, ex - hx))` (18 eps).
2. If a < 90 the arm meets the door on the way in (probe); push it wider first (2 probes):
   `t = math.radians(10 - a)`; closed hand, pitch 90, yaw 90 - a, over `(hx + 0.20*math.cos(t),
   hy + 0.20*math.sin(t), 0.25)`; `robot.goto(by=(0, 0, -0.10))`;
   `robot.arc((hx, hy, 0.15), -40, axis='z')`; then close as for a door at a + 35 (probe).
3. `robot.goto((m.center[0] - 0.04, m.center[1], 0.16), pitch=90, yaw=90, grip='open')`, down
   to 0.078, `robot.gripper('close')` on the mug's -x wall, up to 0.22 (4 probes).
4. Carry and turn in one move: `robot.goto((hx + 0.085, hy - 0.13, 0.125), pitch=0, yaw=90,
   roll=90)` (probe); in to hy + 0.03 with speed=0.1, `robot.gripper(0.6)`, out to hy - 0.13
   (4 probes). Turned 0.21-0.32 up, then lowered there: 3 worked, 1 was BLOCKED (4 probes).
5. Close: from the exit straight up to 0.24 with pitch=90, yaw=90 - a, grip='close'. With
   `t = math.radians(-a)`, `P = (hx + 0.20*math.cos(t) + 0.05*math.sin(t), hy + 0.20*math.sin(t)
   - 0.05*math.cos(t))`; over P at 0.25, down to 0.15, `robot.arc((hx, hy, 0.15), a + 19,
   axis='z')`: it ended short or BLOCKED as the door shut, with SUCCESS (4 probes).

## Verified parameters
- Held by its -x wall, the mug turns with the hand: at pitch 0, yaw 90, roll 90 it went in
  (4 probes), at roll -90 it was BLOCKED (probe). `robot.gripper(0.6)` let it go (4 probes);
  with `robot.gripper(0.3)` instead no SUCCESS came (probe).

## Common failures and fixes
- Insert tries BLOCKED again and again (4 eps); with a door under 90 by the spots, step 4 was
  BLOCKED too (probe); after the push of step 2 it went in (2 probes).
- Grasps 7.4-7.7 cm wide across the body slid out (2 eps): close on the -x wall (4 probes).
- After a push, going from the exit straight over P, the descent there was BLOCKED (probe).
- Door arcs came late or were BLOCKED (4 eps): step 5's arc shut it in 39-43 steps (4 probes).

## Typical step costs
- Horizon 520. Push 64, mug 104, carry 52, down 17, in 49, open 10, out 23, up 44, over P 31,
  down 16, arc 42: 452 (probe); no push, one move in: 366 (probe); learn1's success 514 (1 ep).
