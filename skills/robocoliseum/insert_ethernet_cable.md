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
   port' found nothing (V1 r5 t7; probe L0, L5 r5). Measure its body's top and front end.
4. Pick from straight above: pre-pose 8 cm over the grasp point, straight down, `pitch=25`, `yaw`
   along the body toward its front; grasp point about 8 cm behind the front tip, on the boot, 1.2
   cm over its top (closes to 1.5 cm). Lift 8 cm; H = the grasp height (probe L0 r5; 3/3). Just
   before the pick, to_world on the middle of the plug's front end on the table: its height F.
5. In the wrist picture its front must read well over 6 cm ahead of the grasp point (7.5-7.8 from
   the boot: probe L0 r5, 2/2): the closed fingers meet the router's face 4.5 cm ahead of it. Grasps
   6 cm behind the tip stopped short 9 of 10 (L5 r5 t5-t25), 5.5 cm behind 2/2 (probe L5 r5).
6. Port: the "<-->" socket right of the round power socket; heading perpendicular to two
   `to_world` points on the panel's bottom edge, into the router (L5 r5 t8, V1 r5 t3).
7. Socket height S: to_world on the router's face just beside the socket's middle (zoomed head
   picture). Go 11 cm before the socket on the heading, yaw on it, the grasp point at S + H - F,
   the plug's front level with the socket (untested in this form; L5 r5 t24-t25, probe L5 r5).
8. Sideways only: `to_world` on the socket and on the plug's front in the wrist picture, z at the
   socket's height; move across the heading by their difference (probe L0, L5 r5: 1-2 mm).
9. One `contact=True` push of 4.5 cm on the heading, stay closed, `robot.wait(40)` (L5 r5 t25).
## Common failures and fixes
- "short of target" at the same hand spot again and again: the fingers are on the router's face;
  pushing more never helped (L5 r5 t12-t20). Set it down and grasp it again as in step 4
  (untested; a second grasp 6 cm behind the tip went in: L5 r5 t21-t25).
- Socket far out on the left hand's side (L0): the pushing hand sank 5-6 mm and tipped 2° down,
  the front caught at the socket's mouth 2-3 mm low, tipped down in the hand and fell at the next
  move (probe L0 r5, 3/3). Read the hand's height (robot.state) there and add its sag (untested).
- Moves while the plug pressed on the router's face (a further push, a height change, a back-off)
  lost it (L15 t20, t30, V6 sk t20, V16 sk t34, t42; probe L0, L5 r5).
- A hand moved sideways low beside the creeping plug pushed it over the table's near edge
  (V1 r5 t9-t11): rise first, then come down from straight above it.
- `to_world` on the held plug's front read 7-9 mm high (V1 sk t19, V6 sk t16, t27): use F and S.
- Closes on the body read 2.1 cm, on the boot 1.5 cm, both carried (L5 r5 t6, t23; probe L0, L5
  r5). Weak: 0.3 cm "holding" was empty (L5 t3, V6 sk t4); 0.7-0.9 cm lost it (L5, L15, V6 ns
  t5); 5.3-7.0 cm "holding" lost it at the lift (L5 t10, V16 sk t12, V6 ns t10).
## Final checks
- Before the push: yaw on the heading, the plug's front well over 6 cm ahead of the grasp point
  and on the socket's line in the wrist picture, grasp point at S + H - F of step 7.
