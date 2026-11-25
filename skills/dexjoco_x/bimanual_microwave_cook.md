# bimanual_microwave_cook

## Judging
- The start press counted only with the food inside the oven and the door shut: with the door left
  ajar (the food in the doorway) the same press did not end the episode (L7, L8, L10). With both
  in place the episode ended during the press motion (L1-L10).
- Below, every place is given relative to the door handle's measured centre, `h =
  robot.measure('door handle').center` (+x deeper into the oven, +y toward the hinge). Between
  seeds only the microwave's height changed, so these offsets held on L1-L10.

## Flow
1. Measure the handle and the food from the start pictures; no wrist-camera views were needed
   (L1-L10). Measured so, the food's centre reads about 1 cm nearer the robot and 1 cm lower than
   from close by; step 4's grasp point allows for that.
2. Handle, left hand: `robot.fingers('left', 'power', amount=0.45)` first (part-curled), then the
   grasp point to 8 cm in front of h (-x) and 8 cm toward +y, `normal=(0, -1, 0), along=(0, 0, 1)`,
   `speed=0.18`; then straight to 1.5 cm in front of h at `speed=0.08`. This move stops BLOCKED
   about 1 cm short with the little finger against the door: that is the grip, not a failure.
   `robot.fingers('left', 'power')` then held with four digits (L1-L10).
3. Open: `robot.arc('left', hinge, -65, axis='z', speed=40)` with `hinge = h + (0.04, 0.43, 0)`:
   the full 65 degrees on L1-L10. Keep the left hand on the handle until the door is shut again.
4. Food, right hand: the prompt's food grasp (both vectors turned by d), the grasp point 2 cm nearer
   the robot than the measured centre and 1 cm below it, `approach=0.08`; `'tripod'` with
   `seconds=0.6`; lift 5 cm (`by=(0, 0, 0.05)`, `speed=0.08`). Held on L1-L10; check `.holding`
   anyway: on V9 (food turned 16 degrees) a scripted run of this grasp closed on nothing.
5. Insert, palm down:
   - move the grasp point with `facing='down'` to 10 cm in front of h, 9 cm toward +y and 2 cm below
     h (about 12 cm above the oven's floor, which lies about 14 cm below h), `speed=0.18`;
   - then straight in to 16 cm past h, `speed=0.10`;
   - lower 4 cm (`by=(0, 0, -0.04)`, `speed=0.08`); it stops BLOCKED when the food meets the floor,
     which is fine;
   - `robot.fingers('right', 'open')`, move up 3 cm, then straight back out to 12 cm in front of h,
     1 cm lower than the way in.
   Turned palm down, the food swings in the fingers until it hangs steeply; it still went in (L1-L10).
6. To the start button: `robot.fingers('right', 'point')`, then `point='index'` to 2.5 cm in front of
   the button. The button lies about 3.5 cm deeper than h (+x), 10.5 cm toward -y and 8 cm below
   h. Keep the palm-down orientation (no `normal`/`finger`): the index then points forward along +x
   (L1-L10).
7. Close: `robot.arc('left', hinge, 68, axis='z', speed=40)` while the left hand still holds the
   handle. It stops BLOCKED after about 66 degrees, where the door meets its frame: the door is
   shut (L1-L10).
8. Press: `robot.hand('right', by=(0.045, 0, 0), speed=0.04)`. The episode ended during this move
   (L1-L10).

## Verified parameters
- The whole flow as one cell per step (steps 2-8 checking `.ok` and `.holding` where it matters):
  10/10 on L1-L10.
- Insertion height: 12 cm above the floor worked on every seed; 8 cm did not (see below).

## Common failures and fixes
- Inserting with the hand kept in the tilted food-grasp orientation: the arm came to the end of its
  reach 17 cm short of the goal, still at the oven's opening (BLOCKED, "out of the arm's reach with
  the hand held this way", L1). Turned palm down first, it went the full way in (L1-L10).
- Inserting 8 cm above the floor: the hanging food's lower end caught the floor's front edge and
  stayed in the doorway (L8). At 12 cm it went in (L1-L10).
- Opening the fingers and pulling straight back: the fingers dragged the food back out (L7, L10).
  Lowering 4 cm, opening, rising 3 cm and then backing out left it inside (L1-L10).
- In each of these failures the closing arc stopped early (BLOCKED after 31-56 degrees, the food
  or the hand in the doorway). With a clear doorway it reported 66-67 degrees in all 19 model runs
  with these notes that closed it (L1-L10, V1-V8, V10). In V9 the straight move in had stopped
  BLOCKED 1.6 cm short, the food ended in the doorway with its centre about 9 cm deeper than h,
  and the arc stopped at 61 degrees. The press there did not count.
- So a close that stops below about 65 degrees means the doorway is not clear: do not press.
  Swing the door open again (`arc` back) and measure the food. Pushing it deeper with the
  fingertips did not work: three pushes in V9 were each BLOCKED, and the steps ran out. Taking it
  again with step 4's grasp and inserting it as in step 5 was not tried.

## Final checks
- Before the press: the closing arc reported 65 degrees or more (66-67 when the doorway was clear),
  and the right hand holds nothing.
- No other check was needed: the episode ends by itself on success.

## Typical step costs
- Handle 139-140 (part-curl 15, two moves 109, 'power' 15), open 62-63, food 110-131,
  insert 186-203, to the button 46-47, close 65-67, press 13-14: done at steps 640-671 of the
  1100 (L1-L10).
- The models' own runs (wrist views, retreats, regrasps) ended at steps 826-1100, so wrist views
  and extra retreats cost most of the spare budget.
