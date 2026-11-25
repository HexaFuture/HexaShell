# put_the_cap_on_the_pen_new
## Judging
- Four stages in order: the arm the card names picks up the pen, the other arm the cap, the cap
  goes onto the pen, the pen ends lying on the table (L0-L15, V1-V16).
- Each pick counted at the first look after a 14 cm lift (32/32). The cap stage passed during one
  push that took the cap fully on (L0-L15, V1 sk, V11 sk, V16); caps that met the nib
  off its line slid past and fell when let go (L0, L10, V1 ns, V6, V11 ns). A pen left standing
  on its end failed the last stage (V1 sk).
## Flow
1. Measure 'pen' and 'pen cap' by words (cap found 6/8 in V, not in V6): else a head pixel on it
   (L0-L15, V1-V16).
2. Pen: from 11 cm above a point 2 cm behind its middle, `pitch=90`, `yaw=grip_yaw` (then pass the
   yaw the result gives); grasp point 2.2-3 cm above its top; close (1.8 cm); lift 14 cm (L0-L15,
   V1-V16; 16/16).
3. Cap: the other arm the same way over its centre; grasp point 1.5-2 cm above its top, head or
   wrist measured; close (1.9 cm); lift to the pen hand's height (L0-L15, V; 12/12). At 3 cm above
   the head-measured top it closed on nothing (V1-V16 sk; 4/4).
4. `to_world` on the nib tip and the back end in the pen hand's wrist picture, and on the cap's
   open and closed ends in the cap hand's: the two axes (L0-L15, V1-V16).
5. Cap hand on the pen's axis at the pen hand's height, the opening 2.5-3 cm beyond the nib; yaw
   180 degrees from the pen hand's when the left holds the pen (L5, L15, V1-V16: -72 and 108,
   -111 and 69), equal to it when the right does (L0, L10); if refused, `path='swing'` (L0,
   L10). Measure nib and opening again in the cap hand's picture; re-centre (L15 t11, V16 sk t13).
6. One `contact=True` move of 4.5-6.5 cm along the axis toward the pen's back end (L5, L15, V1 sk,
   V11 sk, V16: on at the first push).
7. Look at the pen in its hand's wrist picture: still level (untested)? Open the cap hand; lift it
   7-9 cm at the yaw a refusal offers (92), `home` it. Lower the pen hand until the grasp point is
   0.6-1.1 cm above where it grasped on the table, open, raise it (L0-L15, V11 sk, V16; 9/9).
## Verified parameters
- Closes on: pen 1.7-1.8 cm, cap 1.9 cm (L0-L15, V1-V16).
## Common failures and fixes
- A push off the nib's line went past the nib; the cap fell when let go (V1 ns, V6 ns, V11 ns),
  and further pushes while holding never helped (L0, L10; 0/3). Re-centre before the one push.
- The pen tilted in its pinch: 25 degrees before the push (V6 sk; it missed), 35 after it
  (V1 sk t11); once the cap hand let go it hung upright and stood on its end when set down (V1 sk
  t15-t16). Pen not level before opening: set it down level with both hands first (untested).
- A fallen cap stood on its closed end (L0 t14; V1 ns): a sweep of the open hand laid it down on
  the second try (L0 t16-t18); gripped from above it hangs upright.
- Wrist cameras went dark when the hands met: the right (L0-L15; 4/4), the left too (V1 sk t10).
- Once the pen was up, re-measuring the cap returned the held nib (L15 t4) and a head pixel under
  the raised hand gave the hand (L0 t4): measure a pixel in the cap hand's wrist picture (L0, L15).
## Final checks
- Before opening the cap hand: the cap reaches the pen's white body, no dark nib section between
  them (L10; L0 t11 showed one: fell off), and the pen lies level (V1 sk; untested remedy).
- After the last open: the pen lies on the table with the cap on (L5, L10, L15, V11, V16).
