# libero_10_object_t5

## Judging
- No book is on the table: the loose things are a grey patterned bowl (on the +y side) and a
  mug (further +x). "The back compartment" (agentview back: nearer the robot, -x): the bowl,
  hanging -x of the hand, lowered with the hand 3.5 cm +x of the organizer's measured centre and
  released, ended the episode with SUCCESS (2 probes; 1 more from a rough centre).
- There it came to rest on the walls ("short" at z 0.14-0.145) and still counted (3 probes).
  Released with the hand over the middle part's +y wall, it did not (1 probe). The mug was not
  tried.

## Flow
1. `org = robot.measure('brown organizer')` or point at it: 0.40 x 0.18 m, top 0.14, centre
   0.34-0.37 m ahead, 13-15 cm to -y (7 eps). 'book' and 'caddy' found nothing (10 eps). The
   table read z -0.031 to -0.032 (6 eps); spots inside the organizer, -0.004 to -0.018 (7 eps).
2. Measure the bowl by pointing at it (untested). Take its +x rim at yaw 90, the fingers closing
   along x: open at z 0.12 about 7 cm +x of its centre, straight down to about 5 cm above the
   table, then close (3 probes; both offsets are rough).
3. `robot.goto(by=(0, 0, 0.20))`: the bowl hangs from its rim, a little tilted (3 probes).
4. With cx, cy the organizer's centre: `robot.goto((cx + 0.035, cy, 0.225))`. The bowl hangs
   -x of the hand, nearer the robot (2 probes; 1 more from a rough centre).
5. `robot.goto((cx + 0.035, cy, 0.13), speed=0.10)`: "short" at z 0.14-0.145 as the bowl sits on
   the walls; `robot.gripper('open')`: SUCCESS (2 probes; 1 more lowered in 2-4 cm legs).

## Verified parameters
- The +x-rim grasp at yaw 90, about 5 cm above the table, held the bowl through the 20 cm lift
  and a 37-40 cm carry at z 0.225 (3 probes). Released with the hand at z 0.13-0.145: SUCCESS
  on the open (3 probes).

## Common failures and fixes
- All episodes hunted the book: dark strips at the table's +x corners, 0.69-0.76 m ahead and
  0.48-0.69 m to +y or -y, were out of reach (3-25 refused poses each) or closed empty; they
  used 384-520 steps and never moved the bowl (10 eps).
- A dark divider inside the organizer, gripped as the book, slipped out on the lift (1 ep).
- The bowl taken by its +y rim at yaw 0 hangs -y of the hand. Lowered with the hand over the
  middle part's +y wall, it stopped "short" at z 0.156 and the open gave no SUCCESS (1 probe).

## Typical step costs
- Horizon 520. Pre-pose 46, down 17, close 11, lift 27, carry 42-44, lower 32, open 7: SUCCESS
  at 182-183 (2 probes); 211 with the lowering in four legs (1 probe).
