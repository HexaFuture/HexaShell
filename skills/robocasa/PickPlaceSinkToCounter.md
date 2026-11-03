# PickPlaceSinkToCounter

## Judging
- Done when the food rests on the plate (its centre over the plate), the plate still on the
  counter by the sink, and the hand well away from the food, at once. It fired in the moves after
  the release (9 eps).

## Flow
1. Priors: the food lies at the back of the basin, centred sideways, often under the faucet's
   spout; the plate is on the counter left or right of the sink, at the front edge; another
   object may stand behind the plate.
2. Measure both; the words missed the food or the plate in 4 eps. Hover over the food, pitch 90,
   grip='open', up to 25 cm above it (7 eps); measure it again and use that grip_yaw.
3. Spout over the food (wrist picture), or the hover stopped BLOCKED or short (3 eps): enter low.
   Go down over the basin's front half to 5-15 cm below the counter top, slide back at that height
   to above the food, then down (2 eps). Leave the same way before going up (1 ep).
4. Grasp point 1.5-3 cm below the measured top, not below `on`; close; lift 0.25 m. Gap shrank
   4 mm or more: set it down and regrasp lower (untested; carried on after 7 mm, it fell, 1 ep).
5. Carry 12-16 cm above the plate's top (8 eps); `dry_run` first. Plate over ~0.65 m to the side:
   not reachable (3 eps): drive 0.30 m sideways toward it while holding (3 eps). Measure the plate
   again from there: its first centre was 1.9-3.4 cm off (6 eps).
6. Lower to a grasp point 3.5-5 cm above the plate's top (7 eps; a flat steak 3 cm, 1 ep); open;
   `robot.goto(by=(0, 0, 0.16))`; `robot.home()`.

## Verified parameters
- Held to the release, pitch 90: grasp point 1.4-1.7 cm below the measured top for orange, mango,
  bell pepper, apple, yogurt, gap 5.2-6.7 cm (5 eps); broccoli 3 cm below, 6.8 cm (1 ep); a steak
  (measured 0.7 cm thick) at `on`, 6.2 cm (1 ep); a milk carton 2 cm below its top, 5.9 cm (1 ep).
- Round food's measured height read 0.6-3.5 cm, the grip closed at 5.2-6.8 cm across it (7 eps):
  its centre lies lower than measured. A 0.24-0.30 m lift cleared the rim (8 eps).

## Common failures and fixes
- Spout: the 25 cm hover or the descent from it was BLOCKED or stopped short, 110-350 steps lost
  (3 eps). Fix as in step 3; driving the base 0.32 m sideways also cleared it (1 ep).
- First grasps that fell: garlic, broccoli 1.4 cm below the top (2 eps; the garlic was taken ~80
  degrees off grip_yaw, turned, gap 5.2 to 4.1 cm); a potato after a descent stopped on its top
  (1 ep); a steak pinched 1 cm above its top (1 ep). Regrasps lower, at grip_yaw, held (2 eps).
- Food fallen against the basin's front wall: pitch-90 descents BLOCKED 10-11 cm above it, tilted
  ones (pitch 20-65) 3-5 cm short; out of steps (1 ep). Avoid the drop (step 4).
- Pitch-120 grasp: 7 moves up hit the basin wall (~140 steps); `robot.home()` pulled it out (1 ep).

## Typical step costs
- Horizon 900. Successes 209-675 (9 eps; the 4 clean ones 209-257); the failure ran out at 834.
- Clean: hover 32-48, down+close+lift 63-65, raise 16-18, drive 26, carry 19-66, lower+open+lift
  46-58, home 10-20. Sinks: the spout 110-350; a drop and regrasp 60-150, 430 with tilted lifts.
