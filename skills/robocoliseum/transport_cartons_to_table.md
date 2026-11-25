# transport_cartons_to_table
## Judging
- Stages in order: two parcels lying on the cart's deck, then all four; the cart before the
  sorting tables; white parcels on the right tables, yellow on the left.
## Flow
1. Parcels (5-7 cm boxes) stand on a shelf ahead, tops 1.27-1.29 m up; the cart is behind you on
   your left. Measure each at a head pixel. Raise both hands: pitch 45, 0.65 m ahead, 0.2 m out,
   1.43 m up, path='swing' (V1-V16 sk; probe L0-L15).
2. Per pick drive so the parcel is 0.76 m ahead, 0.17 m to that hand's side. Of grip_yaw and
   grip_yaw +-90 the left hand takes the one in -90..0, the right the one in 0..90. Dry-run the
   straight move to 8 cm above the grasp point at pitch 45, else 30; go, then straight down to
   1.4 cm below the top; close; lift 12 cm, 8 if refused (probe L0-L15; 66/66 held).
3. To the cart: back 0.45 m, body(lean=0, height=1.30). Turn pair to the turn spot, 0.3 m ahead of
   your start and 1.0 m left: for a way a m ahead, b m left, drive(a/2, b/2, turn=-90), then
   drive(-b/2, a/2, turn=-90). Slide right 0.8 m to face the deck's long side (probe L0-L15; 34/34).
4. head(pitch=25); measure at a pixel in the deck's middle: the Thing is the cart, 0.96 x 0.59 m.
   Its centre should read 0.97-0.99 m ahead, 0.06 m to your left (deck's near edge 0.70 m ahead);
   else drive sideways, then straight, to that (probe L0-L15; 18/18). head(pitch=0).
5. body(lean=65, height=0.72): the head looks down at the deck (reads 0.15 m). Place each at pitch
   80, yaw 0, 0.235 m up, 0.18 m to its side: first pair 1.04 m ahead, second 0.88 m. Open only
   with the hand there; raise it 15 cm, at 0.88 m 10 cm or 5 ahead and 12 up (probe L0-L15; 67/67).
6. Second trip: raise a low hand, body(lean=19, height=1.24), slide left 0.8 m, turn pair to the
   next pick, hands up; pick; back by step 3, step 4's drives with no look (probe L0-L15; 17/17).
7. Cart: from the turn spot face its handle end, bar 0.55 m ahead, hands home; grasp the top bar
   0.15 m each side of its middle at pitch 90, yaw -90 left, 90 right, from 10 cm above straight
   down to 2.5 cm below its seen top (probe L0-L15; 10/12 held). Turn it 90 about its fixed wheels
   (far end): six drives of forward 0.05, left -0.37, turn 15; push by drives of 1.46 m, 7.3 m
   to the tables (probe L0-L15; 4/4). Sorting onto the tables (untested).
## Verified parameters
- Deck reads 0.15 m, parcels on it tops 0.19-0.22, the bar's top 0.91-0.92 (2.8 cm thick); at
  lean 65 the head sees 0.63-1.45 m ahead; places 0.86-1.04 m ahead went, 0.76 m not (probe L0-L15).
- Costs in steps: hands up 179, a pick 107-238, carry and step 4 643-792, lean and two places
  346-427, back to the next pick 613-703, a second trip 1811-2154 (probe L0-L15).
## Common failures and fixes
- Left yaw +17 to +40, right -10 to -21: no straight line at pitch 45; ten turns lost on retries
  (V1 t23, V6 t2-t10, V16 t3; probe L0, L10, L15). Right +70 to +79 took pitch 30 (probe L0, L10).
- A swing lift to 0.73 m ahead, 1.40 m up dropped the parcel (V1 t26); opening after a refused
  place dropped them off the deck (probe L0, L5). Lift straight; keep holding.
- One drive moving and turning 0.3-0.6 m from the cart pushed it (probe L0, L5, L10; 3/3); a turn
  of 180 in place at the turn spot moved or lost it (probe L0, L10, L15; 3/3). Use turn pairs.
- Unplanned ways back failed: stuck at the shelf (V6 t32), lost (V11 t11-t16), next aisle (V16 t18).
- A measure under a raised hand read the hand, 0.47 m up (probe L0-L15; 14/15): move it aside.
## Final checks
- Each parcel lies released on the deck before that hand leaves; after each rise and drive the
  cart stands level on its wheels where it was (V1 t17-t20; probe L0-L15).
