# PickPlaceDrawerToCounter
## Judging
- Done when the object rests on any counter (top, front, island) and the hand is well away from
  it; it fired in the move away after release (13 eps).
## Flow
1. The object (a utensil or tool) lies in the front half of an open top drawer, its far end
   under the counter top. Pull the drawer 0.20 m farther out by its front's rim first
   (prior tool, 45/50; 10 eps). rz = the rim's top, fx its x: `robot.to_world` on its top edge or
   a thin Thing (1-3 cm) on the front's face (10 eps). Grip y: the object's y, but >= 0.16 m from
   the base's centre line, >= 0.10 m inside the front's ends; refused: 4 cm along, then 8 (2 eps).
2. `robot.goto((fx, y, rz+0.08), pitch=90, yaw=90, grip='open')`, `robot.goto((fx, y,
   rz-0.024))`, `robot.gripper('close')`: holding 2.9-4.5 cm; `robot.drive(forward=-0.20)`,
   `robot.gripper('open')`, `robot.goto(by=(0, 0, 0.09))` (prior tool, 49 of 50; 10 eps).
3. Stance (prior tool): before step 2, a front at or behind the base's x: back 0.25 m first; after
   it, a front over 0.20 m ahead (0.27, 1 ep): hand above the rim, drive up till it is 0.11 m ahead.
4. `robot.to_world` on the top's front edge: ex. Hover 12 cm above the rim, yaw = grip_yaw, open;
   measure all of it with the wrist (5 eps; from 10 cm over it a third showed, the grasp went 2 cm
   off its middle, 1 ep). Point at it, `robot.measure(('wrist', (x, y)))`: names failed (6 eps).
5. Grasp its narrowest part (at most 6.5 cm across) nearest the middle of its whole length; a ladle
   on the handle right at its bowl (prior tool, 8 of 8), never the bowl (slid out twice, 1 ep). Aim
   at `t.on_z + 0.010`; Move line over on_z + 0.012: ask the excess lower (untested). Grasps ending
   1.6-2.2 cm over on_z slid or crept (5 of 6 eps), 0.6-1.1 held (6 eps). Lift 0.32-0.36 m (7 eps).
6. Land all of it >= 3 cm beyond ex and >= 4 cm from the other object at the spot nearest the base,
   keeping the grasp's yaw: the grasp point at most 0.72 m from the base in a straight line (5 eps;
   0.76-0.89 refused, 6 eps). Lower to a grasp point 3-9 cm above the top (12 eps); open;
   `robot.goto(by=(-0.27, 0, 0.17))` (10 eps), not straight up (refused, 6 eps).
## Verified parameters
- Held to the release: handles 1.1-3.9 cm, cups 3.5-5.9 cm (18 eps); the rim 2.9-4.5 cm (10 eps).
## Common failures and fixes
- The gap shrank since the close: carry with the arm only, no drive, no hand turn. Drives lost or
  loosened it 4 of 4 times, turns 2 of 3; arm-only carries held 3 of 3 (7 eps). Drive with it only
  if the gap is steady (3 eps); else set it down in the drawer, drive empty, grasp again (untested).
- Without the pull the far part sat under the top: BLOCKED descents, 300+ steps on the bar (4 eps).
  Rim grip closed fully: up, in 1.5 cm farther (untested); pull slipped: grip again (prior tool).
- Narrow drawer: descents stop 3-9 cm high, closes empty (3 eps): the hand body (+-10.6 cm along
  the closing line) sits on its side walls. Grip it nearest the middle of the front's two ends (1
  ep; 4.5-7 cm off failed even turned 40-70 degrees, 2 eps); along a wall: yaw 60 off (prior tool).
- A grasp over 0.27 m to the side of the base: stops short or BLOCKED at pitch 78-85 (6 eps); step
  the base till it is 0.20 m to the side (prior tool, 1 ep). That step BLOCKED (2 eps): from 12 cm
  above the rim at grip_yaw or grip_yaw + 180 (1 ep); 60 degrees off grip_yaw slipped (1 ep).
- Slipped (11 eps): regrasp another part >= 4 cm from the first (prior tool). 'holding' at 7-8 cm:
  lift 10 cm slowly, read the gap (1 of 2 eps held). No `robot.home()`: 80-122 steps (4 eps).
## Typical step costs
- Successes 183-734 of 750 (11 eps): pull 95-132, hover 24-48, down 13-37, close 10-14, lift 32-39,
  carry 29-53, lower 11-22 (5 eps). Replies 14-29; 2 ran out after 22-27 that moved nothing (2 eps).
