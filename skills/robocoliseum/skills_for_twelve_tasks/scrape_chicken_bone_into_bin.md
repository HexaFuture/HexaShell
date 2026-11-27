# scrape_chicken_bone_into_bin

## Judging
- Stage 1: the named gripper lifts the scraper by its handle; motions of either arm before it
  use this stage's steps. Stage 2: every named item inside the can at once (one of two scored
  nothing); an item that fell in stayed in; motions of both arms use its steps, and six runs
  ran out with the items lined up or one of two in.
- The can stands at the near edge on the named arm's side, its rim below the table top: measure it.

## Flow
1. At t1: bone and apple core from head pixels, the can as the robot note says. Plan it all:
   far item onto the can's line, then one pull for all (arx V1, V6, L5; g2op V1, L0; 5/5).
2. Pick first, at pitch 90 or the offered pitch nearest 90 (robot note); close, lift 10-14 cm
   (31/32 picked in V1-V16, 27/32 in L0-L15).
3. After the pick move only for the plan: agilex V1 spent 582 steps on free-arm looks, V16 290
   on a turn and a move-away, arx V11 220 on looks; all three ran out (L untested).
4. Turn the wrist in the lift or carry move: blade across the move, yaw 0 for a pull (arx 14/14
   held; g2op L0 t7). to_world the blade's middle in a wrist picture: 6-10 cm from the grasp
   point (arx V16 t9, L5 t9; g2op V1 t9, V6 t11, L15 t9).
5. Item 10 cm or more off the can's line: put it on the line first, in one move, the robot
   note's way (arx V1 t11, V6 t19, L5 t9; g2op V1 t11, L0; g1 V1 t12, V11, L10; agilex V16 t16).
6. Place the blade, not the hand: blade beyond the item, item between blade and goal, hand clear
   above it; look, then move (arx V1 t10, V11 t11, L0 t7; g2op V1 t10; 4/4).
7. Contact moves at the height the lowering reached (arx 12/13 held, L0, L5; g2op V1 t11, L0 t7);
   0.7-1.5 cm lower the scraper fell (arx V16 t11; g2op V6 t14, V11 t12; 3/3).
8. Pull straight along the can's line, items near the blade's middle, on past the near edge to
   the can's centre; last leg as the robot note says (arx V1, V6, L0, L5; g2op V1, L0; g1 V11).
9. Look, and wait only once every item hangs over the can (arx V1 t15, L0; g2op V16 t21 did not).

## Verified parameters
- Free-arm closed push, grasp point 2-4 cm above the table top (g1) or about 4 cm (agilex), one
  straight move: 33-44 cm (g1 V1 t12, V11, L0, L10, L15; agilex V16 t16, V11 t17).

## Common failures and fixes
- "Dropped": look at that wrist first: 4 of 17 were false, the handle still in the fingers; arx
  won both such runs (agilex V1 t14, V6 t7; arx V1 t4, V6 t12; L untested).
- Blade or hand set on or run into an item: the scraper stopped or fell (arx V6 t11, L10 t6;
  g2op V11 t11-t12, V6 t25, V16 t14, t18, L10 t7, L15 t9).
- After a real drop near the can's line push on with the closed gripper (g2op V16 t20-t22, g1
  V11 t34, arx L10 t10-t11; 3/3); regrasps cost 110-375 steps (g1 V16, L0, L10; g2op L10).
- Diagonal pulls lost the scraper or the bone (arx V16 t19, no-notes V16 t12; g1 L10 t36).

## Final checks
- Every named item seen inside the can; one still on the table: go for it at once (g1 L0).
