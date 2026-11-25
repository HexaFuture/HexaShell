# libero_10_t5

## Judging
- "The caddy" is the dark organizer to -y of the book: 'caddy' by name found nothing (129 eps),
  'brown organizer' did (126 of 129 probes). "The back compartment" is the -x half of its middle
  part, from the back wall (7 cm -x of its centre, 60 eps) to a low divider at the centre (36 eps).

## Flow
1. Free: `b = robot.measure('book')`: 2 views, 11 x 3 x 13 cm, top 0.105-0.110 (97 of 128
   probes); `c = robot.measure('brown organizer')`: 2 views, warned, about 3 cm off (100 probes).
   It found nothing: `robot.find('agentview', at=(170, 240))`, the largest mask (6 probes, 5 eps).
2. Look from 10 cm +x and 4 cm +y of `c.center`, z 0.32, `pitch=90, yaw=0, roll=0`,
   `c = robot.measure(c)`: 41-44 x 14-15 cm, top 0.135-0.147, grip_yaw ±90 (102 of 129 probes).
3. `robot.goto((b.center[0], b.center[1], b.top + 0.12), pitch=90, yaw=b.grip_yaw, roll=0,
   grip='open')`, to `b.top + 0.03`, then `b.top - 0.025` at `speed=0.05`, `robot.wait(10)`,
   `robot.gripper('close')`: holding 2.9-3.2 cm (86 of 94 probes; 136 of 139 eps).
4. Up to `c.top + 0.15`, across to `(c.center[0] - 0.035, c.center[1])`, yaw -90 if the grasp yaw
   was 0 or less, else 90, down toward `c.top - 0.02`: SUCCESS 6-7 steps in (139 eps), else open.

## Verified parameters
- Lowered 2.5-4.5 cm -x of the measured centre, within 1 cm of its y, yaw ±90: 58 of 58 SUCCESS,
  else 8 of 19, in the end or front parts 0 of 14 (eps); also with the caddy 27-42 cm long or raised
  (top 0.23), or the book 14.5 x 4.5 x 9 cm (24 of 24 probes).
- Book top 0.135-0.14: closed 2.5 cm below it 3 of 5 "slipped out"; 6 cm below 2 of 2 held (5 eps).

## Common failures and fixes
- At full speed the descent onto the book stopped "short of target" 2.3-5.8 cm, knocking it (9
  probes, 4 eps); at `speed=0.05` a stop 2-3 cm high left it standing: close there, held (5 eps).
- Book lost ("closed fully (empty)", or "the gap between the fingers changed from 2.9 to 7.9 cm"):
  up 10 cm, `robot.measure('book')`; upright, slowly down 6.5 cm from its top + 0.03, close where
  BLOCKED: SUCCESS (5 of 5 eps). NOT FOUND, 14x11x3 or 17x10x1 cm: flat, never lifted (33 eps).
- Every reading "1 view (agentview)": aim by `robot.to_world('agentview', px, z=0.105)` on the
  book's top, `z=0.14` on the back part (4 of 4 eps); 5 cm +x of that BLOCKED, opened: flat (1 ep).
- A first book top of 0.16: from above it read 0.106-0.108, held by that (6 probes).
- BLOCKED at hand z 0.24 over the back part, aimed right: a can there, unseen before (17 eps). Open
  there: none (6 probes); can closes at 4.9-6.1 cm lifted none (12 eps); `gripper(0.75)` (untested).
- Lowering BLOCKED at hand z 0.20-0.26: aimed over 4.5 cm -x of the centre, 1.2 cm or more off its
  y, or at yaw 72 (16 lowerings, 11 eps). Lift, re-aim about 2 cm, lower or open: 6 of 8 eps.
- Lowered all the way, still held, no SUCCESS: the caddy had moved ("agree only roughly"). Read its
  middle, `m = robot.to_world('agentview', px, z=0.14)`; open at hand z 0.29-0.31 and x 3-6 cm
  less than m's (x 0.19-0.23): SUCCESS 9 of 9 eps; within 2.3 cm of m or +x of it: flat (10 eps).

## Typical step costs
- Horizon 520. Route: look 17-27, hover 31-34, to 3 cm 15-16, slow 28-36, wait 10, close 8-9, up
  27-28, across 43-44, lower 6-7: 186-205 (139 eps); all 172-408, median 189 (197 of 215 eps).
