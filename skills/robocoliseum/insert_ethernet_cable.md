# insert_ethernet_cable
## Judging
- Lift the plug with the hand the task names, push its front into the router's network port and
  keep holding it there: hands opened at the port let it slide out (L10 t15, V1 ns t17, V11 ns t19).
## Flow
1. The plug moved on its own, 4-26 cm and up to 56° in its first 0.5-2 min, between motions too
   (L0, V1, V6 r5). Measure it from the head, then again in the next cell without robot.wait;
   pick once both agree to about 5 mm (L5 r6), else measure again in the next cell.
2. Keep its front from the look before: it turned less than 60°, and its cable can curl up beside
   the front end, so the cable's side does not mark the back (probe L0, L5 r5).
3. `find(cam, 'white plug')` outlines it with part of its cable; 'ethernet plug', 'port' and 'rj45
   port' found nothing (V1 r5 t7; probe L0, L5 r5). Its white body stands 1.2-4 cm over the table.
4. Pick from straight above: pre-pose 8 cm over the grasp point, straight down, `pitch=25`, `yaw`
   along the body toward its front; grasp point about 8 cm behind the front tip, on the boot, 1.2
   cm over its top (closes to 1.5 cm). Lift 8 cm; H = the grasp height (probe L0 r5; 3/3).
5. In the wrist picture its front must read well over 6 cm ahead of the grasp point (7.5-7.8 from
   the boot: probe L0 r5, 2/2): the closed fingers meet the router's face with the grasp point
   about 4.5 cm before it. Grasped about 6 cm behind its tip, it stopped short nine times and
   went in once (L5 r5 t5-t25); body grasps 5.5 cm behind it stopped short (probe L5 r5, 2/2).
6. Port: the "<-->" socket right of the round power socket; heading perpendicular to two
   `to_world` points on the panel's bottom edge, into the router (L5 r5 t8, V1 r5 t3).
7. Go 11 cm before the socket on the heading, yaw on it, grasp point at H + 3.2-3.7 cm: from there
   the front entered the socket (L5 r5 t24-t25; probe L5 r5, 2/2, both then stopped by the hand).
8. Sideways only: `to_world` on the socket and on the plug's front in the wrist picture, z at the
   socket's height; move across the heading by their difference (probe L0, L5 r5: 1-2 mm).
9. One `contact=True` push of 4.5 cm on the heading, stay closed, `robot.wait(40)` (L5 r5 t25).
## Verified parameters
- Closes: body 2.1 cm (L5 r5 t6, t23; probe L0, L5 r5), boot 1.5 cm (probe L0 r5); both carried.
## Common failures and fixes
- "short of target" at the same hand spot again and again: the fingers are on the router's face;
  pushing more never helped (L5 r5 t12-t20). Set it down and grasp it again as in step 4
  (untested; a second grasp 6 cm behind the tip went in: L5 r5 t21-t25).
- Socket far out on the left hand's side (L0): the pushing hand sank 5-6 mm and tipped 2° down,
  the front caught at the socket's mouth 2-3 mm low, tipped down in the hand and fell at the next
  move (probe L0 r5, 3/3). Start that push about 4 mm higher (untested).
- Moves while the plug pressed on the router's face (a further push, a height change, a back-off)
  lost it (L15 t20, t30, V6 sk t20, V16 sk t34, t42; probe L0, L5 r5).
- A hand moved sideways low beside the creeping plug pushed it over the table's near edge
  (V1 r5 t9-t11): rise first, then come down from straight above it.
- `to_world` points on the held plug's front read 7-9 mm high (V1 sk t19, V6 sk t16, t27): keep H.
- Weak closes: 0.3 cm "holding" was empty (L5 t3, V6 sk t4); 0.7-0.9 cm lost it (L5, L15, V6 ns
  t5); 5.3-7.0 cm "holding" lost it at the lift (L5 t10, V16 sk t12, V6 ns t10).
## Final checks
- Before the push: yaw on the heading, the plug's front well over 6 cm ahead of the grasp point
  and on the socket's line in the wrist picture, grasp point at H + 3.2-3.7 cm (L5 r5 t24-t25).
