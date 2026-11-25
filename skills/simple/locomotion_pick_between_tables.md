# locomotion_pick_between_tables

## Judging
- "Place on table2" means: into the basket that stands on table 2. It counted only once the box
  stood inside the basket near its middle and the hand had let go and moved off (scripted 30/30).
  Set down on the table top, or let go above the table's edge, it never counted (4/4 runs failed).
- Table 2 is behind the robot at the start, about 1.7 m away; the basket stands right at its
  near edge, its near half out over the edge: pushed toward the robot it falls off (scripted).

## Flow
1. Measure the box on table 1 (upright, long side across, 0.42-0.45 m ahead; 2 runs). Before
   walking, clear both hands (one cell): the left palm about 0.25 m ahead, 0.2 m left, 0.95 m high,
   facing in; the right `'wide'`, palm `facing='inward'` about 0.22 m ahead, 10-12 cm right of the
   box's right end, 5 cm above its top (scripted 30/30). At the start the palms are 0.28 m ahead,
   above the table's top: walking 0.18 m with them there knocked the box over (4/4 runs).
2. Walk so the pelvis ends 0.22 m behind the grip point and 0.10 m left of it (scripted 30/30);
   measure again. Grip it as in xmove_pick but standing upright (this table is higher): right
   palm `facing='inward'` on its right end, 4-6 cm below its top, then close (4/4 runs held it).
3. Lift the box 20-22 cm, `robot.hand('right', by=(0, 0, 0.21), seconds=0.6, hold='pelvis')`:
   35-75 steps (12 runs). Its bottom then clears the basket's rim later (scripted 30/30).
4. Step back and turn round in one walk, `robot.walk(by=(-0.12, 0), turn=-176)`: 284-306 steps
   (12 runs; scripted 4/4). Two quarter turns (`turn=-90` twice) took 366-379 steps (6 runs), and
   the box counted 16-56 steps past the 1200 limit (V1-V4). Turning in place where it picked, a
   hip can catch table 1 (scripted).
5. Look for the basket: `robot.gaze(0, 25)` facing table 2, `robot.measure('basket')`: its
   centre, rim height (top) and floor. Measure it again from near (step 6).
6. Walk so that the pelvis ends 0.25-0.28 m short of the basket's centre, facing it, the centre
   0-5 cm left of the pelvis's line (scripted 30/30). Measure the basket again.
7. The box over the basket, as a move of its own: its centre 1-2 cm past the basket's centre (away
   from the robot; the right palm then about 9 cm right of that centre), its bottom about 4 cm above
   the rim. Then straight down until its bottom is about 2 cm above the basket's floor (scripted
   30/30). The hand stopped 2-4 cm short of that (the box on the floor): let go there (8/8 runs).
8. `robot.fingers('right', False)`, the hand 2 cm out to the right, then up; `robot.wait(15)`.

## Verified parameters
- The basket's centre in the robot's frame at the place: 0.25-0.28 m ahead, 0-5 cm left, rim
  about 0.15 m above table 2's top (scripted 30/30).
- Walks of 1.1-1.5 m with the box held `hold='pelvis'` kept it in the hand (4/4 runs).

## Common failures and fixes
- Never put it into the basket (4/4 runs): the box let go on the table top or over the edge,
  then regrasps used up the steps. Measure the basket first and place into it (step 7).
- Stood 0.4-0.5 m short of where the box had to go: every palm point there was NOT reachable,
  4-17 cm short (4 runs). Walk to the place of step 6, not to the table's edge.
- Leaning 25-42 degrees to reach over table 2 did not help (2 runs): walk nearer instead.
- From the turn the basket looks small and dark: once 'basket' outlined nothing (scripted).
  Measure it by its pixel then, `robot.measure(('gaze', (x, y)))`.
- Over and down in one move (`via=[over]`) saved 60-75 steps (scripted 4/4) but came in slanted:
  set low, the box caught the rim, the arm pressed on and the basket was pulled off the table (V4).

## Final checks
- After letting go, look: the box inside the basket, the basket still on the table (scripted).

## Typical step costs
- Grip and lift about 200-300, step back and turn 210-230, walk to table 2 about 200-360, place
  about 240; done at steps 1036-1157 of the official 1200 (scripted 30/30). Every extra regrasp
  cost 150-300 (4 runs).
- Through `robot`: hands clear and walk about 235, grip and lift about 125, the turn as step 4, to
  table 2 about 255, over the basket 49-106, down 86-102, open 50, out 25: done at 1153-1220 (8
  runs). No room for a regrasp or a second walk: look and measure instead, they cost nothing.
