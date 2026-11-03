# SearingMeat
## Judging
- Done when the pan stands on the named burner, that burner is lit (only that one), the meat
  lies in the pan near its middle and the hand is well away, at once.
## Flow
1. Before the pan, arm home: find the stove in the start pictures, else turn the base 90 deg, look,
   turn back (2 eps); robot.to_world on the named burner, counted on the whole cooktop (17 eps).
2. Pan, from the start: p = robot.measure('pan'); R = min(p.size[0], p.size[1]) / 2, H = p.size[2].
   Middle c = p.center moved max(p.size[0], p.size[1]) / 2 - R along grip_yaw, away from the handle
   (12 eps). c over 6 cm to a side: home, drive sideways by that, measure again (prior tool).
3. Drag (prior tool, 29/30; rim out 7 eps): u = the heading's unit vector; edge = robot.to_world on
   the shelf's front below the pan; drag = (c - u*R - edge) along u + 3.5 cm, 6-16 cm (no edge:
   9.5). Closed, pitch 45, yaw = heading, roll 0 (90 if p.on_z > 1.47), turned 0.30 m ahead, 8 cm
   over p.top; d = c - u*(R - 0.05) at p.on_z + 0.012: goto d - u*0.057 + 0.057 up, then d (short is
   fine below p.top); goto(by=) -u*(0.05 + drag) and 0.003 up; then up by H + 0.05 (prior tool).
4. Never skip: hand 0.30 m ahead at 1.25 m, measure, robot.to_world on 4-5 near-rim pixels off the
   handle; rim = the nearest one 0.5 cm+ over p.on_z (1 ep; c - u*R erred 2.8-4 cm, 2 eps). Rim
   under 2 cm past the edge: pull 4.5 cm minus that (a full drag tipped it off, 1 ep). Pinch (prior
   tool): g = rim at p.top - min(0.018, 0.6 * H); handle within 50 deg of pointing back: rim 50 deg
   round, yaw at c. Pitch 45, roll 90: turn at g - u*0.085 + 0.115 up; grip='open' to
   g - u*0.071 + 0.071 up, then g; close. Up 4 cm, robot.wait(4). A gap note now or a refused drive:
   set it down, open, grasp again (1 ep; driven on, it fell, 3 eps). robot.drive(forward=-0.40);
   hand to 1.10 m, 0.36-0.55 m ahead, under 0.15 m aside, robot.wait(6), before any other drive
   (8 eps; driven raised, it fell, 2 eps).
5. Place: straight drives till t = burner - u*(R - 0.006) is 0.42-0.62 m ahead, under 0.22 m aside;
   rest = burner z + H - min(0.018, 0.6 * H); goto t at rest + 0.14, rest + 0.08, then rest - 0.015;
   robot.wait(4), open, robot.wait(8); 6 cm back along the pointing line, up 12 cm (6 eps).
6. Meat: pitch 90, yaw = grip_yaw, its middle; grasp point at `.on_z` or 2 cm below `.top`,
   whichever is lower (1-4.4 cm below held 13 of 15, 13 eps; within 0.5 cm, 19 of 28 slipped at
   once, 10 eps). A move short or BLOCKED under 1 cm below `.top` rests on it: do not close
   (18 of 22 failed, 9 eps); then, or narrow side over 6.4 cm: 3-6 cm toward its thinner end, or
   pitch 45 from the robot's side (3 eps). Up 9 cm; open 5 cm over the pan's centre (prior tool).
7. Knob: home, base 0.35 m back, look. Left to right: front left, rear left, (front centre, rear
   centre,) rear right, front right; count from the outer ends (prior tool). On top: pitch 90
   (6 eps). Front panel (14 eps; from behind or above: BLOCKED, 3 eps): base 0.38-0.5 m out,
   0.21-0.32 m aside, turned 25-45 deg (elbow limit: +0.15 m); fingers square into the panel, pitch
   0-20, from 6-9 cm out, on, close, robot.goto(roll=-60) (if roll 0 fails: 180, then 120), open,
   back off.
## Verified parameters
- Held: pinch 0.9-1.8 cm (7 eps), meat at grip_yaw 4.3-7.7 cm (9 eps), knob 0.9-6.7 cm (16 eps).
## Common failures and fixes
- Shelf 1.46 m+, 2 drags short: grip along the handle, pitch 30-45, roll 0 (2 eps). On a counter:
  top-down on the handle, yaw along it, 1.8-3.3 cm (7 eps). Wrong knob lit: turn it back (2 eps).
## Typical step costs
- Pan by 440-1500 steps, meat 240-1100 more, knob 130-410 (8 eps); after 2 misses, switch (1 ep).
