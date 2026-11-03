# CloseFridge

## Judging
- Every fridge-compartment door pushed fully shut, onto its stop. "Close the fridge doors." = two
  doors side by side: both. "door" = one; a freezer drawer below or the freezer's own tall door
  does not matter.
- No release needed, the hand may stay on the door. Doors start wide open and do not swing shut
  by themselves.

## Flow
1. Priors (`robot.to_world` at the start, 10 eps): the front 0.55-0.7 m ahead; the fridge's middle
   within ~0.1 m of the base's line (8 eps), a one-door fridge's centre 0.2-0.3 m right (2 eps).
   From the start pictures: which doors stand open (they stick out toward the base from its side
   edges); `robot.to_world` on each door's inner face (toward the middle) and on the front.
2. Push each door from OUTSIDE it, never reaching across it: a door hinged at the fridge's left
   edge from its left, a right-hinged one from its right. Back up, slide sideways, drive in:
   the base 0.68-0.77 m from the front and 0-0.3 m outside the door's inner face (7 eps).
3. Closed gripper, hand 1.25 m up, mid-door (7 eps). Waypoints as front/side (m in front of the
   fridge front / m outside the door's inner face; negative = past it toward the middle), yaw for
   a left-hinged door, the opposite sign for a right-hinged one, one `robot.goto` each:
   0.40/0.25 and 0.24/0.22 pitch 30 yaw -90; 0.18/0.0 pitch 25 yaw -60;
   0.08/-0.18 pitch 15 yaw -30; -0.02/-0.20 pitch 5 yaw -10.
4. Leg 4 or 5 ending BLOCKED 8-15 cm short of the front = the door is on its stop: leave it
   (2 eps ended a success with no more pushes on it). If leg 5 is NotExecuted, back the hand
   12 cm, `robot.drive(forward=0.12)`, repeat leg 5 (1 ep).
5. Second door: `robot.goto(by=(-0.20, 0, 0))`, then ONE drive back 0.1-0.35 and sideways to
   the second stance, then forward; no `robot.home()` (1 ep, 73 steps; with home 139-179, 3 eps).
6. One wide door reaching back to about the base: back up 0.45-0.55, sideways ~0.9 to its
   outside, forward ~0.4, then the sweep (2 eps). A narrower single door: 0.35 / 0.72 / 0.2 (1 ep).

## Verified parameters
- The sweep of step 3 shut every door at leg 4 or 5, 110-200 steps a door (7 eps, 11 doors).

## Common failures and fixes
- Base ~0.80-0.85 m from the front: the first leg BLOCKED after ~68 steps (3 eps), and the next
  NotExecuted, stretched (2 eps). `robot.drive(forward=0.10-0.12)` and redo it.
- Re-pushing a door already BLOCKED at its stop (pitch 5-90): 177-290 steps lost (2 eps).
- From the start pose, pitch-90 moves across the left door hit it 5 times (178 steps, 1 ep).
- After the first door: home BLOCKED (hand on the shut door), then a drive with the arm out
  dragged the hand into the other open door (~115 steps, 1 ep). Back the hand out first.
- `measure` of a door returned a blob spanning both doors (1 ep): use `robot.to_world` spots.

## Typical step costs
- Horizon 900. One door 226-245 (3 eps): drives ~90, sweep ~140. Two doors 497-707 (7 eps):
  each sweep 110-200, the crossing 73-180; the slow ones lost 180-290 on extra pushes.
