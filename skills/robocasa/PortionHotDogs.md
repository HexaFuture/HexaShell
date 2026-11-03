# PortionHotDogs

## Judging
- Done when each plate holds EXACTLY one bun and one sausage, each lying on the plate itself,
  well inside its rim, and the hand is well away from all four items, at once.
- Lay the bun and the sausage side by side on the plate, not the sausage in the bun. Two buns or
  two sausages on one plate fail.
- Still running after the last release: the hand is still near an item, or an item slid
  off-centre or onto the other.

## Flow
1. Priors: the robot faces the dining counter from the side nearest the bowl (a wide one; two
   buns and two sausages in it). Each plate is at the counter edge in front of a stool; the two
   can be far apart or on another side of the counter.
2. If the bowl is out of reach, `robot.drive(forward=0.18)`: the counter stops the base.
3. Hand ~0.25 m above the bowl, pitch 90, yaw 0; `robot.look('wrist', grid=True)`; measure each
   item by pointing in that picture. Seen from above, sausages are ~11-12 x 2.2 cm and buns
   ~13.5 x 3-5 cm; the word 'sausage' also scored the buns.
4. Sausage: `robot.to_world('wrist', (x, y))` on its middle; pitch 90, yaw = its grip_yaw, 10 cm
   above; down to 1.5 cm below that spot; close; holding 2.5-3.5 cm; lift 0.17.
5. Bun: same, down to 1.5-2 cm below its measured top; holding 5.5-7 cm; lift 0.17.
6. Place: the sausage ~3 cm to one side of the plate's centre, the bun ~3.5 cm to the other;
   grasp point ~5 cm above the plate top; open; lift 0.2.
7. Far plate: pull the hand in to ~0.3 m ahead of the base, then `robot.drive(left=...)` with the
   item held; measure the plate from there; come back for the second item.
8. After the last release lift 0.2 m, then pull the hand in: success fired only on that move.

## Verified parameters
- Sausages at their middle, 1.4-1.7 cm below the to_world spot (yaw 0): holding 2.9 and 3.5 cm;
  both held through drives of 0.72 and 0.48 m, 46 and 34 steps (1 ep).
- Buns 1.4-1.9 cm below the measured top: holding 5.9 and 6.6 cm. One descent stopped "short of
  target" 1.7 cm high; closing there held (1 ep).
- Releases 4.6-5.8 cm above the plate top, the two items 6.4-6.5 cm apart: all four counted (1 ep).

## Common failures and fixes
- A sausage grasp aimed from its measured top (the measure read it 6-9 cm tall, though it is
  ~2.2 cm thick) caught it 3.1 cm below that top: holding 2.8, then it slipped out in the next
  move; the rest of the cell went on with an empty hand (~216 steps). Aim from the to_world spot.
- Scouting the far plate by driving there and back empty cost ~130 steps: carry an item instead.
  The drive back was BLOCKED after 0.48 of 0.72 m; the bowl was still in reach.

## Typical step costs
- Horizon 2250. 1 ep succeeded at 1558 in 25 replies, ~350 of it on the slip and the scouting.
  Per item: pick 80-90, carry and place 100-170; far-plate drives 30-46. Aim for ~1200.
