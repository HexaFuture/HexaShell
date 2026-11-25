# place_block_into_box

## Judging
- Stages in order: Follow; PickUpOnGripper (the named arm holds the block); Inside (the block in
  the box through the round hole). Follow counted in all 12 episodes; each used only the named
  arm (L0-L15, L0a-L15a, L0b-L15b).
- Inside counted during a later wait or move after the release (L0-L15, L0a-L15a, L0b-L10b;
  11/11). A miss (block beside or behind the box, or left on its top) did not end the episode at
  once, and a block picked up again still counted (L15). A stage that takes too long, or
  something out of the allowed region, ends it: L15b ended after its third miss (t38).

## Flow
Do each step in one cell (L0a, L5a, L10a, L15a: 7-10 turns; split up, L0b, L5b, L10b: 11-16).
1. Measure the block and 'block box' and take a zoomed head crop of the box top (round, star and
   half-round holes); `robot.find('head', 'round hole')` gave the round one first (L5b, L15b).
   Take `robot.to_world('head', (x, y), z=<box top>)`: it lay 2-5 cm off the box centre (12/12).
2. Named arm `pitch=90, yaw=0`, open, 10 cm above the block's top; measure the block again from
   there. Lower to about 2 cm above the table, close ("holding" at about 1.8 cm),
   `robot.wait(12)`, lift with `by=(0, 0, 0.15)` (12/12 held), dry-run the pose over the hole.
   If that dry run is NOT reachable, rise straight up about 10 cm more, then go over (L5b t8-t11).
3. If the dry run says "reaches that pose only to within" about 1 cm, first turn the waist about
   20° toward the box (+ = left): `robot.waist(yaw=20)`, box left of the right hand (L15a; in at
   the first release; other sides untested). Without it the hand arrived 11 mm / 2° off, and a
   release at `pitch=85` left the block on the hole's mouth (L15b t8-t16).
4. Go over the hole pointing down, about 10 cm above the box top. Refine in the holding hand's
   wrist look: `robot.to_world('<arm>_wrist', (x, y), z=<box top>)` of the hole's centre moved
   the target 0.5-2 cm (L0, L5, L10, L0a-L15a, L0b-L15b; 11/11).
5. Lower until the grasp point is as high above the box top as it was above the table at the
   grasp, or a little less (1.5-2 cm), still pointing down; open, `robot.wait(12)`, lift about
   10 cm, `robot.wait(20)`, home that arm and wait (in at the first release: L0, L5, L10,
   L0a-L15a, L0b-L10b; 10/10). If the lift is NOT executed, go up 8 cm and 1 cm toward the robot
   (L5a t7, L5b t15; 2/2).

## Common failures and fixes
- Level-hand releases went in 1 of 5 (L15 1 of 3; L15b: fell behind the box t17-t20, stayed on
  the box top beside the hole t32-t37). Release pointing down instead.
- Block left on the hole's mouth: grasped from above again, it held (L15b t17). Untested: turn
  the waist as in step 3, then release as in step 5.
- Block behind the box: tilted grasps (`pitch` 45-75) were BLOCKED or NOT reachable and the other
  arm was out of reach (L15). After `robot.waist(yaw=40)` the `pitch=90` grasp was reachable and
  held (L15); in L15b, after `yaw=35`, it was NOT reachable, `pitch=75` slipped at the lift and
  `pitch=60` held (t22-t30). Untested: set it down within reach, grasp it again with `pitch=90`.

## Final checks
- Both hands empty and the holding hand lifted clear of the box; then wait (11/11).
