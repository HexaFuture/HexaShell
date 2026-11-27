# install_bearing_shaft_seat5mm
## Judging
- Three stages: the hand the card names lifts the gear (counted after 15-17 cm lifts, 16/16); the
  gear goes a few cm down the central shaft; it sits fully down on the disc.
- The seat counts only after the gear has lain fully down, untouched, for a long while: it counted
  in the second `robot.wait(150)` with the hands away (L5, L10, V16 sk). No message says it is
  seated: seated gears touched again were lost (V1 ns, V6 ns); one seated late ran out (V11 sk).
  Gears caught 1-2.5 cm up never counted, however long they waited (L5, L15, V1 sk, V6 sk).
## Flow
1. Point at the gear in the head picture (words found it twice, each in one wrist view only).
   The shaft is the post at the disc's centre (V6 had a second post). Axis: `to_world('head',
   ring-centre px, z=<shaft top>)` (L0-L15, V1-V16 sk; 8/8).
2. Pick at `pitch=60` (L0-L15, V; 10/10); `pitch=90` reached the near gears too (V1 ns, V11 ns).
   A gear far on the other side of the body: turn the waist 29-39 degrees toward it, back after
   the lift (V6, V16; 4/4; untested on L). Centre = mean of four rim points seen from 10 cm above
   in the wrist picture (L5, L10, L15). Its thickness T: to_world on its top face and on the table
   right beside it, in that picture (untested).
3. Grasp point about 1 cm above the gear's top, 0.5-1 cm off its centre toward the hand's own
   side; close; lift 16-17 cm; offset = grasp point minus gear centre (L0-L15, V1-V16 sk; 8/8).
4. Over the shaft, same angles: hand = shaft axis + offset, 8-9 cm above the shaft top; then
   `contact=True` down toward the disc's top + T + 1 cm, where the grasp point would be with the
   gear seated (L5, L15, V1-V16; 11/11 went on the shaft; this goal untested).
5. The hand stops 3.5-7.5 cm above that goal (V; 7/8): open there, lift 15-20 cm, and measure at
   once from a wrist 15-20 cm above the gear and the disc beside it (V6, V16 ns; untested on L).
6. Gear top within 0.5 cm of the disc's top + T: seated. Hands away, `robot.wait(150)` twice, touch
   nothing more (L5, L10, V16 sk; 3/3 counted).
7. 1 cm or more above that: caught. Press at once, no wait: closed hand or `grip=0.4`, `pitch=60`,
   `yaw=0`, grasp point 2-4 cm nearer the robot than the axis, `contact=True` down to the disc's
   top + T + 1.5 cm; lift; measure; then step 6 (V11 sk, V1 ns; 2/2; untested on L).
## Verified parameters
- Closes on the gear from the table read about its measured width (L0-L15, V1-V16; 16/16).
- A `pitch=90` hand stops with its palm on the shaft's top when the grasp point is about 5.5 cm
  below it (L5, L15; V1, V11, V16, 7 times).
## Common failures and fixes
- Fingertip presses at `pitch=90` on a caught gear: `Dropped` or no change (V1 sk t15, t18, V16
  ns t19; 0/3); a `pitch=45` press missed it (V11 ns t12-t13; both untested on L).
- Grasping a caught gear again and pushing it down lowered it 0.5-1.5 cm, never fully (L15, V1 sk,
  V6 sk, V16 ns; 0/4). Waiting on it never helped (L5, L15, V1 sk t13, V6 sk t24-t28).
- Stopped or blocked moves shove the whole gearbox 3.4-8.5 cm (L10 t8, V11 sk, V16 sk, V6 ns
  t22-t23): measure the axis again after one (V11 sk t20, V16 sk t29). Near a seated gear, none.
## Final checks
- After each release the wrist measure of step 5 against the disc's top + T decides; press only a
  caught gear (V1, V6, V11; untested on L).
- Still going after the second wait: measure again; seated, wait again untouched (untested).
