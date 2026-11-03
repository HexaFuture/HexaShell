# PrepareCoffee
## Judging
- Done when the mug stands right under the outlet (its middle within a few cm of S, the spot
  under the outlet), the machine has been started (a start button pressed, step 9) and the hand
  is well away from the mug and the buttons.
## Flow
1. Priors: the mug stands at the FRONT of a shelf ~0.5 m over the counter (take that one, not
   one behind it), about half a metre ahead, its handle turned roughly toward you; the machine
   below or up to 0.75 m aside (20 eps). S: to_world on the tray's middle under the outlet (13 eps).
2. Head check, before driving: to_world on the lowest point of any part reaching out over or in
   front of the tray (a round holder with its handle, a spout) and on the tray. Free height under
   the mug's height + 7 cm: the step 3 hand rides 5-8 cm over the mouth (prior tool) and stuck
   under 6 such heads (6 eps): take steps 6-8 for step 5, and 7-8 after a drop there (6 eps).
3. Grasp: 16-18 cm in front of the mug at its top height, pitch 20-30, yaw 0, open; measure again;
   to its axis 1.6-1.8 cm under its top; close; slide straight back 20-25 cm (20 eps). Not level
   at half height: the handle faces you there. A close under 6.3 cm is the pinch here:
   7 of 9 such fell, 2 of 11 wider (20 eps): open, 1 cm further in, close.
4. Carry: back 0.15-0.28 m first (19 eps). S 0.25+ m aside: no sideways leg at shelf height (none
   placed, 12 of 15 lost: 15 eps); set it down (untested), re-grasp as below, carry it low (4 eps).
5. Top route: the grasp point over S, 6-10 cm over the tray, pitch 30, 0.48-0.60 m level from the
   base (13 eps); dry_run it; lower ~12 cm in front, slide in, open, `goto(by=(-0.18, 0, 0.08))`.
   Stopped 4+ cm short of S: keep holding, back out, steps 6-8 (let go there, none placed: 6 eps).
6. Level route (untested here): base square to the machine, S 0.58-0.70 m ahead; turn the hand to
   yaw +70 at its pitch (dry_run; +95 for a handle heading to your left, +45 to your right), so the
   handle heads -85..-135; set the mug upright 0.40-0.47 m ahead, clear of the head; open; lift.
7. Level grasp (29 eps, from the counter): m = robot.measure('mug'), r = min(m.size[:2])/2, axis =
   m.center moved (max(m.size[:2]) - 2r)/2 away from the handle; pitch 0, roll 180 (0 for a mug 7+
   cm left of the base's line), yaw = handle heading + 60 in -70..-45, u = (cos yaw, sin yaw); g =
   axis - (r - 0.021)*u at m.on_z + 0.046; open 8-18 cm over g - 0.09*u, down, along u to g, close.
8. Slide (19 eps, from the counter; won under 3 of 4 such heads): lift till the bottom is 3 cm over
   the tray; target = S moved 2 cm left, less (r - 0.021)*u; from 12 cm short of it in x, bottom
   1.2 cm over the tray, in at speed 0.08, down at 0.05 till it stops; open only with the mug under
   the outlet in the wrist picture, then `goto(by=...)` 6 cm along -u, 15 back, 20 up.
9. Press (13 eps): gripper closed, pitch 10-30, from ~11 cm in front of a round button 20-35 cm over
   the counter (to_world; robot.find gave tiny masks or none), straight in till short of target,
   out; a press not ended on the way out: the next button (2 eps).
## Verified parameters
- A part 2+ cm over the tray within 10 cm left of S hit the palm at yaw -60 (2 eps): grasp at yaw
  -45, turn to yaw -15 going in (prior tool, 4/4).
## Common failures and fixes
- Dropped, or set down (step 4): from above by its near wall, one finger inside, 1.5 cm under its
  top; a 1.6-2.1 cm close held 10 of 11, else 5 of 16 (16 eps); lift, pitch 30, slide in, open.
## Typical step costs
- Horizon 1800, 100 replies. Clean 239-546 (8 eps): grasp ~85, carry 60-100, release ~50, press
  40-230; with a drop 423-1729 (5 eps); the level route from the counter took 212-343 (9 eps).
