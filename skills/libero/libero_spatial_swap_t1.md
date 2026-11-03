# libero_spatial_swap_t1

## Judging
- In our episodes the bowl that counted stood next to the ramekin: x 0.465-0.495, y +0.305 to
  +0.335, top 0.037 (20 eps). The plate lies far out: x 0.775-0.82, y -0.05 to -0.105 (20 eps).
  Success came with that bowl lowered onto the plate (19 eps).

## Flow
1. Measure bowl and plate (free) (20 eps).
2. Above its +y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] + 0.04, 0.17),
   pitch=90, yaw=0, grip='open')`, c = bowl centre (34-39 steps) (18 eps, 1 probe).
3. Straight down to 0.025 (22-28 steps), close (12-13), up 12 cm (20-24) (18 eps, 1 probe).
4. Carry with the hand tilted to pitch 75, to the plate's centre with y + 0.035, 0.13 up:
   `robot.goto((p[0], p[1] + 0.035, 0.13), pitch=75, yaw=0)` (48-60 steps) (9 eps, 1 probe).
5. Down at pitch 75 to 0.045 there: success came on the way down or within 2 steps of opening
   (8-23 steps) (9 eps, 1 probe).

## Verified parameters
- The +y rim grasp put the hand 3.0-4.0 cm to +y of the bowl's centre, 0.023-0.027 up, holding
  (18 eps, 1 probe).
- At pitch 90 a pose 4.5-8.5 cm up at the plate's x was refused as stretched (2 eps); at pitch
  70-80 the low place poses ran (12 eps, 1 probe).

## Common failures and fixes
- A carry at pitch 90 cost 57-74 steps and the descent 16-28 more: 185-199 in all (7 eps). One
  such run, refused low at the plate, re-aimed about 3 cm off the plate's centre plus the grasp's
  offset, opened with 15 steps left and no success came (1 ep). Another, refused low, came down
  at pitch 75 and was done on step 163 (1 ep). Carry at pitch 75 as in step 4 (9 eps, 1 probe).
- Over the plate, measure() also noted "other 'plate' seen" 7-16 cm up, 5-7 cm from the hand:
  that is the held bowl (7 eps). Place by the grasp's offset; 6 of them did and succeeded (6 eps).

## Typical step costs
- Horizon 220. Probe 170: approach 39, down 22, close 12, lift 20, carry 60, down 17 (1 probe).
- Pitch-75 carries: 154-173 in all (9 eps); pitch 90: 185-199, and one ran out (8 eps).
