# OpenDrawer
## Judging
- The drawer pulled well out, about half its depth or more; further is fine, no release needed.
  Still running: not out far enough.
- "left drawer" / "right drawer" = the top drawer right beside the base on that side, its bar
  centred on its front, a little ahead (20 eps). Not the drawer or oven straight ahead, nor the
  next drawer along.
## Flow
1. Free: `robot.measure(('left', (x, y)))` (the drawer's side) pointing at the bar: a horizontal
   bar 0.11-0.21 m long (16 eps). Wider or flat = it grew into the front or the counter: point
   again or use `robot.to_world` on the bar (5 eps). Call its centre bx, by, bz.
2. Stand BESIDE the bar, not behind it: `robot.drive(to=(bx-0.27, by-0.42))` for a left drawer,
   `by+0.42` for a right one: the bar 0.27 m ahead and 0.42 m to the side of the base (prior tool,
   30/30); a short drive from the start pose, heading kept. Measure the bar again.
3. Angles: contact point c = (bx+0.01, by, bz). For pitch p in 10, 20, 30 and roll r in 90, -90
   (yaw 0), dry-run `robot.goto(c, ...)`; take the first that is reachable with no 'joint limit'
   in its text (prior tool: pitch 5-10 in 46 of 50).
4. Pre-grasp 13 cm back along the hand, fingers at 0.6: `robot.goto((c[0]-dx, by, bz+dz),
   pitch=p, yaw=0, roll=r, grip=0.6)`, dx, dz = 0.13 cos p, 0.13 sin p (pitch 10: 0.128, 0.023)
   (prior tool, 84 of 90 clear).
5. Straight in: `robot.goto(c, pitch=p, yaw=0, roll=r)`. Ending 0-3 cm short with the fingertips
   on the drawer front is normal (prior tool, 59 of 88). `robot.gripper('close')`; check holding.
6. Pull with the base: `robot.drive(forward=-0.30)`, then `robot.drive(forward=-0.15)`. Success
   came during the first or the second drive (12 eps).
## Verified parameters
- Pitch 0-10, roll +-90, the bar fully between the pads: held 1.2-1.6 cm, the gap did not shrink
  in the pull (3 eps, all successes; there the bar was straight ahead, 0.49-0.62 m).
- Pitch 45 from 0.62-0.67 m straight behind (the old recipe): 12-84 mm short, the open upper finger
  on the front; of 19 first closes 7 empty, 4 slipped 10-14 steps into the pull, 8 held (20 eps).
  Behind the bar pitch 0-20 was often out of reach (5 eps), at joint limits (prior tool).
## Common failures and fixes
- Closed fully: `robot.gripper(0.6)`, back to the pre-grasp, in again to c + 1.5 cm in x, close
  (prior tool, 9 of 88 grasps). Never nudge the base forward onto the bar: 6 of 6 stayed empty
  or slipped, 3 BLOCKED against the front (6 eps).
- Slipped in the pull (the drive stops, 'slipped out'): open, `robot.goto(by=(-0.10, 0, 0))`,
  measure the bar by pointing BEFORE moving (it is nearer now; aiming at its old place pushed the
  drawer shut, 1 ep), stand at 0.27 / 0.42 again, repeat steps 3-6 (prior tool).
- The side drive BLOCKED near the start (1 ep): back 0.25 m, sideways onto the stance line, then
  forward (prior tool). Bar not in the start pictures: drive 0.30 m that way, look (prior tool).
- Failures spent 500-575 steps on 7-12 re-approaches from behind at pitch 40-90 (8 eps): redo
  steps 2-5 instead. No `robot.home()`: it cost 95-143 steps or was BLOCKED (4 eps).
## Typical step costs
- Horizon 750. Parts: side drive 25-40, pre-grasp 55-65, onto 20, close 12, pulls 25 + 15
  (20 eps). The prior tool's whole attempt took 154-208 steps (prior tool, 40 of 50). A re-grasp
  as above costs ~100: room for four; no surveys (untested).
