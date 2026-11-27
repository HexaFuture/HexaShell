# transport_cartons_to_table
## Judging
- Stages in order: two parcels lying on the cart's deck, then all four; the cart before the
  sorting tables; white parcels on the right tables, yellow on the left.
## Flow
1. Parcels stand on a shelf ahead: measure each at a head pixel. Raise both hands: pitch 45,
   0.65 m ahead, 0.2 m out, 15 cm above the parcels' tops, path='swing' (V1-V16 sk; probe L0-L15).
2. Per pick drive so the parcel is 0.76 m ahead, 0.17 m to that hand's side. Of grip_yaw and
   grip_yaw +-90 the left hand takes the one in -90..0, the right the one in 0..90. Dry-run the
   straight move to 8 cm above the grasp point at pitch 45, else 30; go, then straight down to
   1.4 cm below the top; close; lift 12 cm, 8 if refused (probe L0-L15; 66/66 held).
3. To the cart (untested): back 0.45 m, body(lean=0, height=1.30); find 'cart' in the head picture
   (head yaw up to 44), else turn in place by 90 and look again; head(pitch=25), measure it at a
   pixel in its deck's middle. Go to its long side in legs, measuring it after each; to turn near
   it, turn pairs: for a way a ahead, b left, drive(a/2, b/2, turn=-90), drive(-b/2, a/2, turn=-90).
4. Stance: the deck's centre 0.97-0.99 m ahead, 0.06 m to your left, its long side facing you;
   else drive sideways, then straight, to that, and measure again (L0-L15; 18/18). head(pitch=0).
5. body(lean=65, height=0.72): the head looks down at the deck; to_world on the deck's middle: its
   top. Place each at pitch 80, yaw 0, 8.5 cm above that top, 0.18 m to its side; the first pair
   6 cm beyond the deck's centre, the second 10 cm short of it. Open only with the hand there;
   raise it 15 cm; from the near places 10 cm (or 5) ahead and 12 up (probe L0-L15; 67/67).
6. Second trip: raise a low hand, body(lean=19, height=1.24); to the shelf by the drives you made
   (as the replies read them) in reverse, measure the parcels again; pick; steps 3-5 (untested).
7. Cart: face its handle end, the top bar's top (to_world) 0.55 m ahead, hands home. Grasp the bar
   0.15 m each side of its middle at pitch 90, yaw -90 left, 90 right, from 10 cm above straight
   down to 2.5 cm below its seen top (probe L0-L15; 10/12 held). Find 'table' (head yaw up to 44):
   the sorting tables. Turn the cart toward them about its fixed wheels (far end) by drives of
   forward 0.05, left -0.37, turn 15 (six: 90; probe L0-L15; 4/4); push by drives of at most 1.46 m,
   measuring the tables after each, to 0.3-0.5 m from them; whites right, yellows left (untested).
## Verified parameters
- Lean 65: the head sees 0.63-1.45 m ahead; places 0.86-1.04 m ahead went, 0.76 not (probe L0-L15).
- Costs in steps: hands up 179, a pick 107-238, carry and step 4 643-792, lean and two places
  346-427, back to the next pick 613-703, a second trip 1811-2154 (probe L0-L15).
## Common failures and fixes
- Left yaw +17 to +40, right -10 to -21: no straight line at pitch 45; ten turns lost on retries
  (V1 t23, V6 t2-t10, V16 t3; probe L0, L10, L15). Right +70 to +79 took pitch 30 (probe L0, L10).
- A swing lift with a parcel held dropped it (V1 t26); opening after a refused place dropped them
  off the deck (probe L0, L5). Lift straight; keep holding.
- One drive moving and turning 0.3-0.6 m from the cart pushed it (probe L0, L5, L10; 3/3); a turn
  of 180 in place close to it moved or lost it (probe L0, L10, L15; 3/3). Use turn pairs.
- Ways back driven without looking got stuck, lost or in the next aisle (V6 t32, V11, V16 t18).
- A measure under a raised hand read the hand, 0.47 m up (probe L0-L15; 14/15): move it aside.
## Final checks
- Each parcel lies released on the deck before that hand leaves; after each rise and drive the
  cart stands level on its wheels where it was (V1 t17-t20; probe L0-L15).
