# bimanual_hold_pot

## Judging
- Grasp both handles: counted after one `robot.goto('both', by=(0, 0, 0.12))` with both holding
  (L0, L5, L10, L0b, L5a, L5b, L10a, L10b, L15b, L0c-e, L5c-e, L10c-e, L15c-e; 21/21), never after
  lower lifts, in one move or in parts (L0a, L15a, L15b; 0/4). Until then the burner did not (L0a).
- Put down: counted upright on the near burner (L0, L0b, L5a, L5b, L15b, L0c, L0e, L5c-e, L10d,
  L10e, L15c-e; not L10b, which counted at the 'stove' centre), once it had rested there until seen.

## Flow
1. Measure 'pot' and 'stove'. Take each handle point and the near burner's centre with
   `robot.to_world('head', (x, y))`; re-pick handle spots at the counter's height (L0a, L10b).
2. Grasp from outside along each handle: left `yaw=-90`, right `yaw=90`, `pitch=30, roll=90`, from
   6-8 cm out at handle height, `path='line'` in to just below the handle point. Close both,
   `robot.wait(10)`: holding at 0.5-1.0 cm (L0, L0a, L0b, L5a, L5b, L10b, L15a, L15b, L0c-e, L5c-e,
   L10c-e, L15c-d; 19/21). On 2.6-3.0 cm: open, in again at pitch 30, close (L0a, L10a, L15e; 3/3).
3. Start point NOT reachable at pitch 30: start at pitch 45, go in at 30. The 0.12 lift is then NOT
   executed (L0a, L15a-e; 6/6): turn that hand in place, `robot.goto('left', robot.state().left.at,
   pitch=45, yaw=-90, roll=90)`; then the lift ran and counted (L15c t9, L15d t11, L15e t10; 3/3).
4. `robot.goto('both', by=(0, 0, 0.12))`, `robot.wait(10)`: it held through lift, carry and
   set-down (L0, L0b, L5a, L5b, L10a, L10b, L15b, L0c-e, L5c-e, L10c-e, L15c-e; 19/19).
5. Carry toward the near burner in 'both' legs of 10-13 cm; halve a leg NOT executed. Half refused
   too: turn both hands in place to yaw -45..-65 / 45..65 and carry on: the legs then reached the
   burner (L15c t14, L15d t18, L15e t15-t16; 3/3; untested with both at pitch 30). Else set it down
   where the legs stop: it stood, short and left of the burner centre (L0c-e, L5c-e, L10c-e; 9/9).
6. Lower with 'both' until both Moves end "short of target" (L0, L0b, L5a, L5b, L10b, L15b, L0c-e,
   L5c-e, L10c-e, L15c-e; 18/18). Open both; move each hand 7 cm out along its handle, 4 cm up,
   same angles; NOT executed: back and out, e.g. left `by=(-0.03, 0.05, 0.07)` (L0e t13, L15e t20).
   Not at pitch 60: then `robot.home('left')` "could not lift straight up first" and knocked the pot
   over, never righted (L0d t14-t27, L10c t12-t72; 2 of 4). Then `robot.home('left')`, then
   `robot.home('right')`; `robot.home('both')` moved or lost it 4 of 8 times (L10, L0b, L10a, L15a).
7. Measure; compare its centre with the burner centre. Short and left of it, as where the pitch-30
   legs stop: push it on (below) to the centre or a little past, home that arm (L0c t16, L0e t16,
   L5c t13, L5d t13, L5e t13, L10e t20; 6/6 counted). Nearly centred (L10d, L15c-e): go to step 8.
8. Both hands home: `robot.wait(25)` three times in one cell, then measure (untested as three; one
   or two waits did not end L0e, L5c-e, L10d, L10e, later moves did). Do not regrasp or push again
   a pot upright on the centre (4-6 turns lost each: L0c, L0e, L5c-e, L10e). If the lift was not
   one 0.12 move, regrasp at `yaw=-45`/`45` from 4-5 cm back and out, lift 0.12 (L15b t31-t37).

## Common failures and fixes
- From above (`pitch=90`) 10 of 12 two-hand lifts slipped (L0, L5, L10, L15): grasp as in step 2.
- Push from the robot's side: gripper closed, `pitch` 75-90 (take "reachable instead"), fingertips
  low on the near wall, straight at the place, `contact=True`, 12-18 cm legs, measure after each
  (L0, L0a, L5a, L5b, L10b, L15b, L0c, L0e, L5c-e, L10e; 12/12 upright on the burner). From the
  left side, the back or at handle height it went astray or tipped (L0a t48, L0b, L15a, L10b t24).
