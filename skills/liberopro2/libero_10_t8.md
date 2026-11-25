# libero_10_t8

## Judging
- Both pots on the stove at once: SUCCESS came as the second was let go, 3-11 steps into the open
  (135 eps), or while it was still held, lowering (26 eps). Pots that counted stood about 10 cm
  apart, one each side of the stove's centre along x (54 of 73 eps); one lying at its edge did not.

## Flow
1. Free: `s = robot.measure('stove')`: 2 views, 0.19 x 0.17-0.18, top 0.011-0.045 (141 of 205
   eps). 'moka pot' found nothing (107 of 121 eps): measure `robot.find('agentview', at=(280,
   333))[0]` (the middle pot, first: 70 of 75 successes) and `at=(418, 301)` (123 probes).
2. `robot.goto((p.center[0], p.center[1] - 0.068, 0.23), pitch=90, yaw=90, roll=0, grip='open')`,
   `p = robot.measure(p)`: 2 views, 7.2-8.6 cm across, top 0.117-0.123 (223 probes); the handle
   sits 6.6-7.5 cm -y of that centre (102 holds); the other pot's first x read 3 cm short (44 eps).
3. Each pot: over `p.center[1] - 0.068`, down to 0.09 (the first pot: to 0.20, to 0.15 at
   `speed=0.05`, on: 20 steps more, 10 of 10 probes), close: holding 1.4 cm (264 of 270 closes, 135
   eps); up 0.15, across at 0.24 with the pot centre (hand + 0.068 in y) 0.041 +x of s.center, the
   second 0.053 -x and 0.016 +y of it; down to s.top + 0.115, open, `robot.goto(by=(0, 0, 0.10))`:
   SUCCESS (135 eps).

## Verified parameters
- Knob at yaw 90, down to 0.12-0.14: on the stove 35 of 43; the lid at yaw 0: 26 of 67 (110 tries).
- A stove read 0.08-0.15 taller or 26-28 cm wide: releasing at hand z 0.225-0.272, or where the
  lowering stopped, worked (8 eps). Re-read after a lift: 1 view, 20-33 cm wide, wrong (9 probes);
  from over its middle at z 0.25 it read 9-21 cm further +x (6 eps), no better (4 probes).

## Common failures and fixes
- Down to the handle stopped "short of target" 2 cm high: sent again, it got there (5 eps). Stopped
  3.5-4.5 cm high, within 2.2 cm in x and y: close there, holding 1.7-2.0 (5 closes, 3 eps), then
  open where the lowering onto the stove stops 4-5 cm high (3 eps); re-aiming instead lost 1 of 2.
- First descent onto a handle "short of target" 70 mm off, 2 cm low, the lift BLOCKED: the pot lay
  knocked over (10 of 10 eps), 5 done at 468-509. Slowed as in Flow 3 it stood: SUCCESS (10 probes).
- Second pot "closed fully (empty)" after a clean descent: it read 9-11 cm +x. Up 14 cm, measure
  it again, close at `robot.to_world('wrist', px, z=0.09)` on the handle: SUCCESS (5 of 5 eps).
- 'close' reported holding at 7.7-8.0 cm: nothing held (9 probes, 5 eps); open, go down, close
  again: held 1.4 cm 52-63 steps after the first, SUCCESS at 423-437 (5 eps).
- Every reading "1 view (agentview)": handle by `robot.to_world('agentview', px, z=0.09)`, stove
  middle at `z=0.02` (4 of 5 eps). Stove NOT FOUND ("the outline does not reach the table") or not
  0.19 x 0.19 (5 probes, 20 eps): point at its middle, measure that: 14 of 20, then 5 of 5 SUCCESS.
- Something on the stove: lowering onto it failed 2 of 3 (eps). The second lowering stopped 1.6-4.3
  cm high, pot still held: SUCCESS came there (2 of 5 eps); open there if none came (untested).

## Typical step costs
- Horizon 520. Per pot: hover 46-50, again 9-13, down 21, close 10, up 23-24, across 30-31 (second
  44-46), down 15-18, open 13, up 18: SUCCESS at 362-472, median 373 (135 eps); all 351-505 (195).
