# libero_goal_t6

## Judging
- "The cream cheese" is the small blue box labelled "Cream Cheese", right of the bowl in the
  agentview picture, about (336, 311): 8 x 4 x 1.6 cm, long side along x, top about 0.005,
  grip_yaw 0 (96 of 129 probes). 'cream cheese' by name found nothing (128 calls).
- SUCCESS came 5-7 steps into the open, the box over the bowl's centre (133 of 153 eps).

## Flow
1. Free: `bowl = robot.measure('bowl')`: 11 x 11 cm, top 0.035-0.037 (107 of 129 probes);
   `c = robot.measure(robot.find('agentview', at=(336, 311))[0])`.
2. `robot.goto((c.center[0], c.center[1], 0.12), pitch=90, yaw=0, roll=0, grip='open')` (35-44
   steps), `c = robot.measure(c)`: within 2 cm of the first, keep it (104 of 123 probes).
3. `robot.goto((c.center[0], c.center[1], 0.004))` (19-20), `robot.gripper('close')`: holding
   4.2 cm (194 closes, 215 eps); `robot.goto(by=(0, 0, 0.12))` (21-22).
4. `bowl = robot.measure(bowl)`, `robot.goto((bowl.center[0], bowl.center[1], 0.14))` (22-26),
   down to 0.06 (15), `robot.gripper('open')`: SUCCESS (206 of 215 eps).

## Verified parameters
- Held at yaw 0 within 0.4 cm of the box's centre, closed at z 0.002-0.0075: 4.21 cm (103 eps).

## Common failures and fixes
- The thing at that point not about 8 x 4 cm (a mask over most of the picture, a round thing 15
  cm tall): the box stood elsewhere; pointed at in either picture and taken: SUCCESS (5 of 6 eps).
- The move above the box ended short or BLOCKED 2.6-7.8 cm off, the hand 1-6 cm +x and +y of it;
  going down from there missed (9 probes). Keep sending: up or toward the robot (home: 97 steps, 4
  eps), then above the box at 0.07-0.12, then down: SUCCESS at 134-287 (23 of 24 eps).
- Down onto the box BLOCKED 6-8 cm above it (10 eps): 3 cm to +x of its centre, or pitch 115 over
  it, then down to 0.007: held 4.2 (6 eps, 5 SUCCESS); yaw 15-20, 55 or 90 BLOCKED or slipped (5).
- Down to the box "short of target" 3.6-3.9 cm at hand z 0.040, things reading "rests on something
  4-8 cm above the table": closing there held 4.2-4.3, SUCCESS at 163-219 (5 eps).
- 'close' holding 7.3-8.0 cm ("nearly fully open") or "closed fully (empty)": nothing held (7
  probes, 3 eps). Up to 0.10 if the box is seen 8-9 cm away, point at it again, down to 0.005-0.009
  at its grip_yaw, close: 4.2 cm after 1-3 more closes, SUCCESS at 187-293 (6 eps).
- Readings that disagree ("agree only roughly", a bowl 5-7 cm wide, the box 4-6 cm tall): agentview
  was off; the hand marked in it showed by how much. Corrected by that, or pointed at in the wrist
  picture, the box was held, even 2-3 cm "short of target" (8 of 9 eps, SUCCESS at 136-278).
- A raised bowl (top 0.146-0.167): moving from above it down to the box caught the bowl (3 eps),
  hovering first at 0.24-0.26 none did (5 eps); opening 1.5-2 cm above its top counted (4 eps).
- A thin board along the box's -y side: yaw 0 BLOCKED 10-14 cm up, from +y tilted it stopped 2-3
  cm short, yaw 45-70 held 0.7-2.9 cm and slipped; no SUCCESS (8 eps).

## Typical step costs
- Horizon 300. Route: hover 35-44, down 19-20, close 8-9, up 21-22, across 22-26, down 15, open
  5-7: SUCCESS at 127-160, median 128 (153 eps); no note 119-167 on clean runs (95 eps).
