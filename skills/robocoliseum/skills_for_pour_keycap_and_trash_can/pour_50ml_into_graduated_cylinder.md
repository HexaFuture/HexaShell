# pour_50ml_into_graduated_cylinder
## Judging
- Card: the named arm grasps the jug by its handle (passed 32/32 r6), then pours until the
  graduated cylinder holds 50 ml (passed 3/32 r6: g1 L0, g1 L5, g2op L0; probes 4/38). Fails r6:
  liquid outside 21, a second pour or regrasp not finished 6, the jug dropped 1, unclear 1.
## Flow
1. find gave the jug 1/8 and the tube 4/8 (probe L0, L5 r6): take a pixel on each in the head
   picture. A ring of depth points on the jug's rim gave its centre within 5 mm (probe L0, L5 r6;
   19/20); points 6.5-11.5 cm out gave the handle's heading within about 3 degrees (16/20). Leave
   out points near the tube (taken for the handle once); if an arm hides the handle, see robot note.
2. Grasp and lift as the robot note says, by lines: a 291-step swing to the pre pose knocked the jug
   over (g1 probe L0 r6); a 360-step swing holding it dropped it (agilex V6 r6).
3. Park the other wrist by the tube as the robot note says and fit a ring of depth points on the
   tube's rim in its picture: the mouth's centre and height within 2 mm (probe L0, L5 r6; 36/36).
   Keep that wrist still and take a picture of the empty tube for the column reads (probe L0 r6).
4. One line to tilt 30 or 45 over the mouth, then 60, the lip at the robot note's heights and
   offset: nothing reached the tube before the tip (probe L0, L5 r6; 36/36).
5. Tip in one move to the robot note's tilt and read back the reached pitch: one degree changed the
   flow a lot (g2op probe L5 r6; 4 runs). Correct the height only: shifts across the side picture
   taken from the jug's outline put the stream beside the tube (probe L5 r6; 2 of 3).
6. Hold still; nothing moves between calls (probe L0 r6). First liquid came 0.75-2 s after the tip,
   often a wave of 0.6-1.4 cm of column in 0.25 s, then 0.4-3.7 cm a second (probe L0, L5 r6).
7. Column reads: the side picture against the empty one along the tube's far inner wall; the highest
   changed point is the column's top. Start back by the robot note's rule: on one move back to 60,
   0.3-1.4 cm more came in (probe L0, L5 r6; 14 runs).
8. Return as the robot note says, by lines: back to 60, wait about 1.75 s, toward upright and up,
   over the jug's own spot, down to the grasp height, open, raise 8 cm; wait about 3 s, finish:
   nothing was outside in 16 of 17 runs and the jug stood within 3 mm of its start in 13 of 14
   set-downs (probe L0, L5 r6).
## Verified parameters
- 120 steps = 1 s; one wait takes at most 150 steps (probe L0 r6). Jug about 9.2 cm tall; the tube's
  mouth about 20 cm above the table, its rim ring about 4.6 cm across (probe L0, L5 r6); 50 ml is
  about 3.6 cm of column, the first printed mark (L0-L15; probe L0, L5 r5).
## Common failures and fixes
- Liquid outside (L0, V1, L5, V6 r6; 21/32): lips computed from an upright estimate, 1.8-4.6 cm
  off; a jug moved or turned while tilted; a second tip. Measure the lip, tip once, hold still.
- While it flows the stream shows as liquid: head reads put the column up to 6 cm too high and two
  head-read stops came at about 1.4 cm of column (agilex probe L0, L5 r6; 2/2); side reads lagged
  0.4-1.4 cm or jumped 1-3 cm high (probe L0, L5 r6). Never stop on one read.
- Settled reads came within 0.2 cm of the column in 10 of 46 and up to 3.7 cm off otherwise; an
  empty tube once read full (probe L0, L5 r6): never pour again on a read alone.
- Second pours and regrasps did not finish (L0, V1, L5, V6 r6; 9 of 10): one tip, one return.
## Final checks
- Side picture: column near the 50 mark; nothing on the tube's rim, outside or base or on the
  table; tube upright; jug on its own spot; hand open and raised (probe L0, L5 r6).
