# open_door

## Judging
- One stage: the handle turned and the door pushed open with the right arm (4/4). The door has to
  stand clearly open until it is seen: it counted during a later wait or move, never during a
  push (4/4).
- The handle pushes alone counted once (L5). In the other three it counted only after a further
  push on the door's face (L0, L10, L15; 3/3).

## Flow
1. Find the doorknob with `robot.measure('door handle')`, then `robot.to_world('head', (x, y))`
   on 4-5 pixels along the handle (4/4). The measured centre moved 2-3 cm on a closer look (L0,
   L10): aim by the pixels.
2. Right hand level and open, `pitch=0, yaw=0, roll=90`, 8-10 cm in front of the handle at its
   height, 2-4 cm from its centre toward the door's middle. Go straight in (`path='line'`), close
   ("holding" at 1.1-1.4 cm), `robot.wait(12)` (4/4).
3. Turn: 3.5-4 cm lower and 1.5 cm away from the door's middle, `roll=45`, `contact=True`, then
   `robot.wait(10)` (4/4).
4. Push 10-13 cm straight ahead, `yaw` 8-10, `roll=45`, `contact=True`: the Move ends "short of
   target" by 1-2 cm, still holding (4/4). Open; back the hand off 10-20 cm (4/4).
5. `robot.waist(yaw=25)` to `30`: from the start heading, points near the door's middle were NOT
   reachable (L0, L10, L15; 3/3). Close the right gripper.
6. Face push: closed level hand (`roll=0`) at the handle's height, 43-56 cm from the handle toward
   the door's middle, `yaw` 10-20, `contact=True`, from about 5 cm deeper than the handle grasp
   point to 2-5 cm past where the handle push ended (L0, L10, L15; 3/3). One that ended short of
   the handle push's end did not count (L10). Then `robot.wait(8)`.
7. Back the hand off and wait (L0, L15), or just wait (L10): it counted there.

## Common failures and fixes
- Handle pushes past step 4 were NOT reachable or NOT executed, "beyond the arm's reach" (4/4),
  a turn each; only an extra 2 cm ran (L5). Release there; the face push counted (3/3).

## Final checks
- Head look: the door stands open and the hand is off it; then wait (L0; L15 counted during the
  next move).
