# cover_cup_lid
## Judging
- Set the lid centred and level on the cup. The cup tips over at a touch: touch only the lid.
- The task's steps name an arm picking up the cup: leave the cup standing, never grasp or lift it.
- The lid: a flat, very light disc about 7 cm across and 1.5 cm tall; measure reads 5.3-6 cm across.
## Flow
1. measure('cup lid', cams=['head']) and measure('cup', cams=['head']) (probe L0-L15; 37/37).
2. Raise both 15 cm as they start; then, in the next cell, hand(arm, 'point', fingers={'middle':
   0}) for each (probe L15; lifted 9/10). Shaping first gripped a nearby cup (V1, V6 r3; 4/4).
3. Hands empty and high, turn the waist to face midway between the lid and the cup (heading seen
   from 25 cm in front of the robot's origin); a lid 25 cm or more to one side: face the lid
   (probe L0; 3/3; V11 r3). Measure the cup right after the turn and use that (probe L0-L15; 17/17).
4. Each hand above its side of the lid: pitch=90, left yaw=14, right yaw=-14 (pads face the lid),
   path='swing', 23 cm above the table (if refused: 19, then 16 cm); index tip 1.8 cm farther
   than the lid's centre and 6.5 cm to its side (probe L0-L15; 39/41 reached).
5. Each hand straight down with contact=True to 0.5 cm below the table top (the lid's on_z), then
   each in with contact=True to 3.1 cm from the lid's centre (probe L0-L15; 37/37 lifted).
6. goto('both', by=(0, 0, 0.03)); the wrist pictures show the lid level between both index tips,
   else set it down, release (11) and squeeze again (V6, V11 r3). Its centre: the tips' midpoint
   moved 1.8 cm toward the robot (probe L0-L15; within 2 mm); a wrist reading was 6 mm off (V11 r3).
7. Dry-run, then lift until the index tips are 2.5 cm above the rim; dry-run, then one goto('both',
   by=...) putting that centre over the cup's reading (probe L5-L15; 12/12).
8. One goto('both', by=(0, 0, -d), contact=True) to the index tips 0.5 cm above the rim; keep
   holding (probe L0-L15; 20/20; from 4 cm above it 4/4). Hand by hand tilted the lid on the rim
   (probe L15; 1/1; L15 r3). If the task goes on: release (11), raise both (untested).
9. Two legs if lid and cup are over 30 cm apart sideways or a dry run refused. Lid far out: carry
   it 13 cm toward the cup, 4 cm nearer; one goto('both', contact=True) down to the table; release
   (11); raise each hand 15 cm (8 if refused); face midway between that lid centre (6) and the cup;
   4-8 there (probe L0; 5/5).
10. Cup far out: set the lid down halfway, 4 cm nearer, 13 cm or more from the cup; raise both 15
   cm; face the cup; squeeze along the torso's lines, yaw = waist yaw +14/-14 (probe L5, L10; 3/3).
11. Release: six times left then right 1.5 mm out, contact=True; 1.5 cm out (probe L0-L15; 12/12).
## Verified parameters
- Squeeze 9-14 mm past first touch held 37/37, 5 mm did not (probe L5); carries stayed level.
## Common failures and fixes
- A waist turn while holding tilted the lid 10-21 degrees: turn with empty hands (probe L5, V16 sk).
- With the hands 8 cm up, turns and home() swept a hand over the cup (probe L0; 4/4): raise 15 cm.
- measure missed the cup behind the arms (probe L0-L15; 16/18 before the carry) or read the lid as
  the cup (V1 r3): keep the first reading after the turn. No lid found: measure(('head', (x, y))).
- Approach refused: the outer hand's yaw 10 degrees nearer 0 reached (V11 r3 t13); else the heading
  5 degrees at a time (L10: +3/+4). A swing descent hit the lid, other yaws held it tilted (V11 r3).
- Cup caught in a hand: lower the whole raise, then flat (stood V6 r3 2/2; 1 cm short fell V1 r3).
- One fingertip per hand, flat palms, pinches or wraps never held it (probe L15, V6 sk, r2; 0/29).
## Final checks
- Before lowering: lid level, its centre over the cup's reading, the cup standing (probe L0-L15).
