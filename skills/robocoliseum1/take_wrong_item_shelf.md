# take_wrong_item_shelf

## Judging
- Two stages in order: the named arm holding the misplaced item, then the item in the basket
  (L0, L5, L10, L15; 4/4).
- The grasp counted at the close. The basket counted a while after the release, during the next
  turn's move away (4/4). The end message came in that turn (L0, L5) or one or two turns later
  (L15, L10).
- Counted: the item let go upright over the basket's middle with its lower end at or just above
  the basket floor (4/4); it then stood in the basket (L10, L15).

## Flow
1. The misplaced item is the one unlike the others in its row (4/4). Measure it and the basket at
   head pixels: by words the item matched the wrong thing or was seen by one wrist camera only,
   and the basket was NOT FOUND (L5, L15).
2. Use the arm the instruction names (left in L0, L15; right in L5, L10). Dry-run the start and
   grasp points first (4/4).
3. Level grasp, `pitch=0, roll=0`, `yaw` along the approach: 0 with the item straight ahead of the
   arm (L5, L10), -15 to -18 for the left arm reaching in toward the middle (L0, L15). Start
   11-12 cm in front of the item's centre at its centre height, gripper open; line in to the
   centre; close; `robot.wait(12)`: holding at 3.0-3.2 cm (4/4).
4. Lift 7.5-8 cm, wait, then pull straight back about 25 cm at that height (4/4).
5. Move over the basket's middle (within 3 cm of its measured centre) and lower the grasp point to
   the basket's measured centre height: the item's lower end reaches the floor (the move stopped
   short there in L5). Open; up 16-20 cm; wait (4/4).
6. Move the arm clear: `robot.home(arm)` (L0, L10, L15) or a level move aside (L5). Moves aside at
   `pitch` 25 and 55 were NotExecuted (L10).
7. Then only wait and look. Do not reach into the basket again: a re-grasp was under way when the
   end message came (L10).
