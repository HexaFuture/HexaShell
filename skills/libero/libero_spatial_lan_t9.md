# libero_spatial_lan_t9

## Judging
- In our episodes the bowl that counted sat on top of the cabinet: x 0.64-0.72, y -0.255 to
  -0.30, top 0.263-0.266 (8 eps). The plate lies at x 0.70-0.74, y +0.19 to +0.22 (10 eps).
  Success came with the bowl set down on the plate (8 eps).

## Flow
1. Measure bowl and plate (free). The first measure from the start pose had its x 1.6-1.9 cm
   too small (3 eps): measure again once the wrist sees the bowl.
2. Above its +y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] + 0.04, 0.30),
   pitch=90, yaw=0, grip='open')`, c = bowl centre (37-41 steps; 53 for a bowl at x 0.69)
   (3 probes).
3. Straight down to 1.5 cm below its top, about 0.249 (15-22 steps), close (12)
   (9 eps, 3 probes).
4. Lift 8 cm with `robot.goto(by=(0, 0, 0.08))` (17-18 steps) (2 probes). With the grasp at
   x 0.69 or more, pull in while lifting instead: `robot.goto((g[0] - 0.04, g[1], 0.33))`,
   g = grasp pose (20 steps) (1 probe).
5. One straight move to the plate's centre with y + 0.04 (the grasp's offset), 0.045 up (69-74
   steps), then open; success came within 3 steps of opening (3 probes).

## Verified parameters
- Grasp on the +y rim: hand 3.5-4.8 cm to +y of the bowl's centre, 0.247-0.251 up; it closes at
  about 0.8-1.0 cm, holding (9 eps, 3 probes).
- A carry to the place pose 30-34 cm up was refused as stretched (4 eps), while nearly the same
  pose ran in others (5 eps): it is at the edge of reach. The straight move from an 8 cm lift or
  a pull-in down to 0.045 was neither refused nor blocked (3 probes).
- During the carry a move may note "the fingers closed fully: what they held slipped out" while
  the bowl still hangs in the hand: go on with the place (5 eps).

## Common failures and fixes
- Lift to 0.36, level carry 29-32 cm up (70-72 steps), then straight down: the steps ran out
  with the bowl still held and the hand 6-9 cm up (2 eps). The successes on that path took
  191-220 steps, one ending on the last step (8 eps). One straight move after an 8 cm lift or a
  pull-in: 158-179 (3 probes).

## Typical step costs
- Horizon 220. Probes 158-179: approach 37-53, down 15-22, close 12, lift or pull-in 17-20,
  straight move 69-74, open 3 (3 probes). Learning-round successes 191-220 (8 eps).
