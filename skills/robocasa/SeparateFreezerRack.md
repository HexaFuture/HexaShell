# SeparateFreezerRack
## Judging
- Done when the vegetable tray is on the HIGHEST freezer shelf, the meat tray on the one right
  below, the items still in their trays, and the hand well away from both trays, at once.
- The door starts open. Pushed shut, it was never back open in time: handle
  pulls slipped, 800+ steps lost (11 eps). A tray or item on the floor is lost (17 eps).
## Flow
1. Scout empty-handed, arm home: back ~0.9 m BEFORE sliding to face the opening (prior tool); slid
   at 0.5 m back, it swept the open door shut (2 eps). Read the top two shelves' heights and front
   edges, the opening's sides, the open door's bins and free end, closed fronts' handles (2 eps).
2. Grasp the meat tray on its near short wall (below), never the measured centre: that closed on
   the food or slipped (2 eps). Lift 6-9 cm, hand 0.22-0.26 m ahead of the base (2 eps).
3. Carry: back until the far end (hand + tray length) is 13 cm or more before the closed fronts
   (handles stand 6-9 cm proud) and outside the open door's free end: ~0.9 m (2 eps); at 0.3-0.5 m
   it hit a handle or swept the door shut (6 eps). Slide at one heading, no turns, until the base is
   0.15-0.2 m off the opening's centre line AWAY from the door, the hand mid free width (2 eps).
4. Tilt there, still far back (2 eps): pitch 60, yaw = the heading, roll = heading minus the grasp
   yaw (-90 after a grasp at heading+90); grasp point 6 cm above the shelf, 0.28-0.40 m ahead of
   the base; dry_run first, else pitch 68 or 52 (prior tool). Tilted by the fronts: hit bins (1 ep).
5. Drive the BASE straight in, arm fixed, in two legs of 0.31-0.38 m with a look between, until
   the grasp point is 3.5 cm before the shelf's front edge (2 eps, 4 trays). BLOCKED 10 cm or more
   short: back 5 cm, look, shift 2-3 cm away from what touched, retry once (prior tool).
6. Release (2 eps, 4 trays): lower at speed=0.06-0.08 to 1.3-4 cm above the shelf, open;
   `robot.goto(by=...)` up 6.5 cm and 1.2 cm forward, then up 8 cm, 3 cm forward; back 0.4 m.
7. `robot.home()`, back to ~0.9 m, slide, forward to the counter; then the vegetable tray (2 eps).
## Verified parameters
- Grasp (27 eps held): pitch 90, yaw heading+90 (fingers close front-to-back) on the near short
  wall's middle; grip=0.9 from 10 cm above; down to 3-3.5 cm above `.on_z`; close: 0.7-3.0 cm.
  A gap shrinking 0.6-1.4 cm on the lift right after the close still held (2 eps). A descent ending
  10 mm+ off, even `.ok`: do not close (1 of 9 placed, 7 eps); go 3 cm along the wall (prior tool).
- Held trays droop 6-26 deg far end down: tilted 8 deg the tray caught the shelf front, 30 deg went
  in (prior tool). A tilt 0.24 m ahead of the base was out of reach (1 ep).
- Top two shelves 29-38 cm apart (5 eps); one freezer's second was glass, 0.65 m below the top
  (1 ep): there stand up to 7 cm nearer the middle, the low elbow met the handle (prior tool).
  Glass reads as nothing in depth: `robot.to_world` with `pair=` on its front edge (2 eps).
## Common failures and fixes
- A gap-change note in a carry, tilt or drive: the tray hit something and turned; moving on lost
  it (5 eps). Stop, back 5-10 cm, look; askew: back to the counter, set down, regrasp (untested).
- Backing BLOCKED short of ~0.9 m: forward 5 cm, slide; still BLOCKED, 5 cm more (prior tool). The
  slide went through only 0.12-0.2 m forward (3 eps); there the far end met the closed handle
  (3 eps): slide in legs of 0.3 m or less, a look after each (12 eps; 9 still got a gap note).
- A level hand pushed in by the arm: BLOCKED again and again, then it fell (5 eps).
## Typical step costs
- Horizon 2400, 100 replies. Successes 1275-1392 steps, 27 replies (2 eps): scout 260-390; per
  tray grasp 75-90, carry 130-150, tilt and drive in 75-95, release 75-95; return, regrasp ~310.
