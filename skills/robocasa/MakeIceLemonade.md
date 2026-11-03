# MakeIceLemonade
## Judging
- Done when the wedge and 1+ ice cube are in the glass, the hand away from all three; the doors
  do not matter. The wedge sits at the front of a fridge shelf, up to ~0.5 m over the counter top.
## Flow
1. Doors (prior tool): a pair over drawers: both, right first, display or not; its bar is the right
   (lower y) one of the two at the seam (2 eps). Two doors to the floor: the one with no water/ice
   panel, else the right (2 eps); one tall door: it. Glass and bowl: measure them now (untested).
2. Door grip: base square, the bar's front 0.51-0.54 m ahead (7 eps); at 1.10-1.30 m, pitch 20-25,
   yaw square, roll 0: open, 9 cm back along the hand, in to 0.5 cm past its front (3+ cm short: 4
   of 4 slid, 2 eps), close (prior tool) on its upright part; one ending under 1.2 m: pitch 45, 5 cm
   under its end (2 eps). Two doors to the floor: open 0.6, grip at 1.00-1.09 m: 78-90 deg (15 eps).
3. Swing, `robot.drive`: right door turn=85, about=(hand x + 0.08, hand y - 0.36); left turn=-85,
   hand y + 0.36; wide 0.60-0.66 aside (prior tool). 0.32-0.38 aside slid 0 of 22, 0.46+ 5 of 13
   (43 eps); 0.12-0.27, not deeper than the bar: 5 of 5 failed (2 eps). Stopped at 13-36 deg: grip
   again at pitch 65, `robot.arc` 30-35 deg steps (3 of 3 went on, 1 ep; at pitch 25: 7 of 7
   stopped, 5 eps). Reach in only with no door edge before the wedge (else stuck 600-1700, 8 eps).
4. Leave: open, `robot.wait(6)`, back 11 cm level along the hand at the same pitch and yaw,
   by=(-0.11*cos(yaw), -0.11*sin(yaw), 0), home (5 eps). Next door or the wedge: out square to
   0.75 m before the bars, across, in (probe, 7 eps). Pass an open wide door with the base 0.35 m
   further out than its end (prior tool); crossing 0.04-0.10 m out pushed it shut (1 ep).
5. Wedge: 'green wedge' or 'yellow wedge' (not 'lemon wedge', 2 eps); base square 0.60-0.65 m
   behind, pitch 30-45, 30 above ~1.33 m (prior tool; held off square: 0.15 m aside, away from the
   hinge, pitch 50, 1 ep); hand open 15 cm back along the hand, measure to 2+ views (4 eps); yaw
   within ~15 of grip_yaw, straight in to the 2-view `.center`, above `.on_z` + 1.5; close (2 eps).
6. Out: lift 3 cm, back 20 cm along the hand, base straight back to 1.1 m behind it (1.5 m by an
   open wide door) before any slide (prior tool): slides at 0.77-1.34 m stuck, wedge lost (4 eps).
   No home: grasp pitch and yaw kept, hand 0.25-0.3 m ahead at 1.25-1.4 m (16 eps; homed: 1/10).
   Glass beside the fridge: slide, same heading, 0.15-0.3 m past its line, then straight (prior
   tool; 1 ep); behind: one drive turning <=90 deg (3 eps); more turning or home lost it (4 eps).
7. Release: pitch 90 over the glass's centre (clear: rim by `robot.to_world`, pair=, 2 eps), grasp
   point 23-30 cm over the counter, open (7 eps). Ice: pitch 90, yaw square to a face (diagonals
   slipped, 3 eps), 0.4-1.2 cm below its top, `robot.gripper('light')` (replays: 0 of 20 lost),
   lift 0.15, release as the wedge (5 eps); read flat: `robot.to_world` at the tops' z, ~5.4 cm up.
## Verified parameters
- Bars close 1.9-3.8 cm, a few 4.1-4.8 (27 eps), a wide door's 4.3-4.5 (4 eps). Ice held 3.9-5.3.
- Wedge 6.5-8.8 x 1.9-5.3 x 2.4-4.0 cm in 2+ views, one view 0.5-1.3 across (7 eps); glass top
  7.7-13.4 or flat (9 eps). Wedge closes of 1.3-4.7 cm shrink ~5% per 10 steps, still or moving,
  to 0.2-1.4 at the glass 200-300 steps on (18 eps): plan the route first; looking takes no steps.
## Common failures and fixes
- Closes of 0.6-1.4 cm (one-view centre, near the tip, yaw 40 off grip_yaw) slipped (5 eps). Slip
  refusal: drive on, straight, as planned (24 of 35 reached the glass, 35 eps). "Slipped out" but
  seen at the fingers: pinched; open only over the glass (9 of 9 did, 9 eps; sooner: lost, 1 ep).
## Typical step costs
- Horizon 3000. Successes 1114-2595 (18 eps): wedge closed at 680-1360, carry 200-300, ice 150-360.
