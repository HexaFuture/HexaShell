# PickPlaceCounterToCabinet

## Judging
- Done when the object is inside the cabinet, between a level's floor and the shelf above (it may
  stick out of the front a little), let go, and the hand well away from it, at once.
- Upright and the doors (already open) do not matter; open shelves count. Another object may
  stand at the back of a level. A 26 cm retreat was not yet far enough (1 ep).

## Flow
1. Measure the object; `robot.to_world` on the lowest level's floor at its front edge and on the
   edge above. Floor about 0.5 m above the counter, its edge ~0.4-0.55 m ahead (9 eps). The
   object lay ~0.3-0.45 m ahead, mostly within 0.3 m to the side (9 eps); once further (1 ep).
2. From above with `grip_yaw`, a long thing at mid-length; lift 15-20 cm (8 eps).
3. Carry by shape, the hand level (pitch 0-20):
   - compact or flat: roll 0, the grasp point 6-13 cm above the floor (3 eps);
   - long (14-32 cm): roll 90 so it lies across the opening, its middle in line with the
     opening's middle (1 ep). At roll 90 the hand's body stands ~10 cm above and below the grasp
     point: carry it at the level's mid-height, 11-21 cm above the floor (4 eps).
4. Straight in until the grasp point is 8-15 cm inside the front edge. Roll 0: lower until "short
   of target" (~5 cm above the floor). Open; back straight out 30 cm, then down (7 eps).

## Verified parameters
- Closed widths that held: ladle handle 0.9 cm, brush 1.4, banana 3.2, mushroom 5.5, foil box
  5.7, lime 5.9, sponge across its width 6.0 (7 eps). Long things released at roll 90 with the
  grasp point 12-17 cm above the floor settled inside (4 eps).

## Common failures and fixes
- Roll 90 with the grasp point 7-8 cm above the floor: BLOCKED at the front edge (4 eps); at
  17-19 cm: BLOCKED by the hand's top (3 eps). Each retry cost 30-60 steps.
- A 30 cm box across the opening stopped until moved 12 cm sideways to the middle (1 ep).
- Tall bottle (17-23 cm): top-down pre-poses above it were refused or BLOCKED under the cabinet
  (2 eps); a side grasp aimed past its axis pushed it 10 cm away; grasps on the head or shoulder
  slipped on the lift (1 ep, 4 tries). Instead (untested): side grasp low on the body, pitch 0-10,
  roll 0, the grasp point 1 cm short of the axis; carry it upright, its bottom 2 cm over the floor.
- A close at 7.9 cm (the full opening) on a 7-8 cm jar is a wedge: the open freed nothing and the
  hand pulled it off the shelf onto the floor (1 ep). Grasp across a side under 7 cm (untested).
- A thin flat thing closed at its measured top (0.9 cm) slipped; 0.7 cm lower the fingers closed
  across its width and held (1 ep).
- A round flat thing levelled at roll 90 turned in the fingers (gap 4.3 -> 2.2 cm), then fell in
  a sideways drive with the arm held out (1 ep). Carry it at roll 0 (untested).

## Typical step costs
- Horizon 750. Clean, about 200 (3 eps): pre-pose 24-36, descend and close 26-35, lift 19-21,
  level 37-48, in 27-40, open 13, out 20-30. Blocked entries added 50-170 (4 eps).
