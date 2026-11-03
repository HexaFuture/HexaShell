# libero_spatial_lan_t0

## Judging
- In our episodes the bowl that counted stood between the ramekin (to -x of it) and the plate
  (to +x of it), all near y +0.20: from the top of the agentview picture down, ramekin, bowl,
  plate (3 eps). Bowl x 0.54-0.61, y +0.19 to +0.225 (10 eps); plate x 0.695-0.74, y +0.185 to
  +0.21 (9 eps); the ramekin, a small round grey cup, x 0.455, y +0.21, top 0.030 (1 ep). The
  other black bowl stood to +y of the ramekin (right of it in the agentview picture, 3 eps), at
  x 0.48, y +0.32 (1 ep).
- Its first measure may read top 6.5-8.3 cm (3 eps), not round (2 eps); measured again it read
  3.7 (2 eps) or stayed at 6.5, tilted (1 ep). It is still the one. Success came with it set down
  at the plate's centre (7 eps); landing 2-2.4 cm off it did not count (2 eps).

## Flow
1. Measure bowl and plate (free) (9 eps).
2. The -y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] - 0.04, 0.07), pitch=90,
   yaw=0, grip='open')`, c = bowl centre; straight down to 0.024 (a tilted bowl: 1.5 cm below
   the rim's height at that spot, from `robot.to_world` on the wrist picture), close, lift 10 cm
   (2 eps, 2 probes).
3. Carry 12 cm up to the plate's centre with the same y - 0.04, then straight down to 0.04-0.045
   and open; success came during that descent or within 2 steps of opening (2 eps, 2 probes).

## Verified parameters
- Grasp on the -y rim: hand 4 cm to -y of the bowl's centre, 0.025 up; it closes at about
  0.8 cm, holding (2 eps, 1 probe).
- The -x rim at yaw 90 (hand 4 cm to -x of the centre, place at plate centre x - 0.04) also
  works, but turning the hand costs about 14 steps more (4 eps, 1 probe).

## Common failures and fixes
- The +x rim at yaw 90 (hand 3.3 cm to +x of the centre): the place pose as far to +x of the
  plate's centre was refused as stretched; let go 2 cm to -x of it, the bowl landed 2 cm to -x
  of the plate's centre (1 ep). Grip the -y rim or the -x rim, so the hand stays at or to -x of
  the plate's centre (2 probes).
- After a correct carry, re-aiming from an agentview measure of the hanging bowl moved the hand
  about 2 cm the wrong way; let go 6.7 cm up, the bowl landed 2.4 cm off-centre (1 ep). Keep the
  grasp's offset and go down to 0.045 before opening (1 probe).

## Typical step costs
- Horizon 220. The -y rim 116-120: approach 30-32, down 11-19, close 12, lift 18-20, carry 22-33,
  down 11-20 (2 eps, 2 probes). The -x rim at yaw 90: 126-149 (4 eps, 1 probe).
