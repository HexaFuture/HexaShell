# open_the_roller_shutter

## Judging
- Stages: base near and facing the shutter; key off the floor; key turned; shutter up; base inside.

## Flow
1. At once, before anything else: measure(('head', (x, y))) on the key's black bow in the head
   picture (V1-V16 sk; 4/4; the word 'key' 0/4); drive straight forward till it is about 0.8 m
   ahead, legs of at most 1.5 m; measure it again after every drive (probe L0-L15; 7/10).
2. Then, also when the key looks right and before any other motion, square up: point at two
   spots far apart on the shutter's foot, turn by their line's angle (20-41 degrees), measure,
   drive sideways till the key is about 0.8 m ahead, 0.16-0.2 m left (L0-V6 r5-r8; 16/16, else 0/4).
3. body(lean=45, height=0.80) with the lock about 1.0 m ahead: floor near -0.04 (probe L0-L15;
   17/18). Read low, measure again rather than back off: from 1.11 m pushes stopped (V11 r3).
4. Key at most about 0.84 m ahead (probe L15). Left hand on the black bow, not the measured middle
   (the blade; probe L5), pitch 90, yaw along the blade, contact=True to the floor, close: about
   5.5 cm (probe L0-L15; 14/14). Lift 0.15 m, path='swing', then yaw 0 (probe L0-L15; 13/14).
5. Lock: small round silver disc with a level slot, 13.4-14.6 cm above the floor read with it
   (probe L0-L15; 17/17). Touch it 6 mm above the slot: tip 8.5-11.9 cm ahead of the grasp (12/12).
6. Light presses (about 1.5 mm into the face), the grasp point 1.5-2 cm above the head reading of
   the disc's middle, from the reading leftward 1 mm apart: the key went in 6-10 mm left of it (V6,
   L0, L5 r3; 4/4), 0.5-4 mm left (probe L5, L15; 4/4); at or right of it, or the grasp point 0-1
   or 3 cm above it, the key never turned (V11, V16, L10, L15 r3; 0/5; probe L5-L15; 0/3).
7. Push on in 6 mm steps, contact=True (stuck: tilt the key 0.7-2 degrees about its tip), holding
   till the tip is about 5 cm past the touch (V6, L0, L5 r3; probe L5, L15; 6/6). No hand turn.
8. Let go, lift the hand 6 cm, close it. From 2-3.5 cm above the grasp point of the seat and 1.1-2.2
   cm left of it (2.4-3.4 cm sprang back; V6 r3), move straight down, pitch 90, yaw 90, to 3.5-5 cm
   below it, contact=True: the key turned to about 90 degrees (V6, L0, L5 r3; probe L5, L15; 5/5;
   V1, V6, L0 r7; 3/5); without it, it did not (L5 r7). Then lift at once: no message tells that
   it turned, and runs that stayed at the key and the foot used 74-102 motions (L0, V1, L5, V6 r5).
9. body(lean=45, height=0.73); close both hands; pitch -45 (fingertips up), yaw 0, roll 0, y 0.15
   and -0.15, grasp point 0.49 m above the floor (0.53 if refused): from 5 cm before the face to 1.6
   cm before it, contact=True (probe L0 r5; V1, V6 r7; probe L0, L5 r9; 23/23).
10. Torso up to height 1.05 in 8 cm steps; before each, goto('both', by=(d, 0, 0.01), contact=True),
   d = 1.2 cm plus how far body(lean=45, height=h, dry_run=True) says the held hands go back; from
   0.49 m, after the first also by=(0.02, 0, 0.01). Then only goto('both', by=(0.012, 0, 0.08),
   contact=True) on and on; refused: 4, 2, 1 cm up, till 1 cm is refused (probe L0, L5 r9; 12/12).
11. Still pressing, to_world('head', (320, 8)): the floor 1.6-1.8 m beyond the face = up, the face
   1.1 m up = not (probe L0, L5 r9; 13/13). Up: hands off one at a time, 5 cm back each, then at
   once drive forward 1.3 m, the hands left where they are (L0, V6 r10; 2/2). Hands lowered or home
   first: it came down (V1, L5 r10, two runs each; 4/4; probe L0, L5 r9; 16/16): step 9 again.

## Common failures and fixes
- Turning, tilting or swinging the key held at the lock dropped it (V11, V16, L10, L15 r3; 6 drops).
- Stopped at 1.65-1.74 m, no smaller rise tried: it fell (L5, V6 r9; 3/3). Not up: step 9 again.
