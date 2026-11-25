# hammer_nail

## Judging
- One downward strike was enough: on L1-L10 the episode ended during the first strike of step 7. In
  that one move the head hit the nail, bounced and hit it again.
- What decides the episode is the first grasp of the handle. With the index finger pressing after
  'power', the hammer stayed in the hand through the lift, the carry and the strike in every
  scripted run (55 of 55). Without the index it slipped out on the lift or was never held (10 of 10).

## Flow
1. Nail: `n = robot.measure('nail')` from the start pictures. It was found on L1-L10, its centre
   within 0.7 cm of the true one and its top within 0.2 cm. Keep `n.center` and `n.top`.
2. Look at the hammer from straight above.
   - In the scene picture the hammer lies at the robot's right (-y) with its head standing upright;
     the head hides most of the handle.
   - Put the right wrist camera 46 cm above the hammer, looking straight down. On L1-L10 the
     camera at (-0.32, -0.45, 1.46) with `look_at=(-0.32, -0.45, 1.00)` saw the whole hammer.
   - Then `h = robot.measure('wooden handle')`. From there the words outlined the handle on
     L1-L10. In the start pictures no handle words found anything on L1, and 'metal head' outlined
     the nail's top on L1 and L10, so do not measure the hammer from the start pictures.
   - Measured so, `h.grip_yaw` was within 0.3 degrees of the handle's direction and `h.center` lay
     2.3-2.5 cm above the handle's axis, 0.5-1 cm from the handle's middle toward the head (L1-L10).
3. Grasp with the right hand.
   - The grasp point `g` is 3.5 cm below `h.center`, about 1 cm below the handle's axis.
   - `along` points along the handle toward the head and rises as the handle does (it lies tilted
     16 degrees): `A = (0.96 cos(y), 0.96 sin(y), 0.27)` with `y = h.grip_yaw` in radians. If
     `A[0] < 0`, flip its x and y signs (keep z at 0.27). On L1-L10 the head lay toward +x.
   - First move the grasp point 15 cm straight above `g` with `along=A, facing='down'`,
     `speed=0.15`.
   - Then `robot.hand('right', g, point='grasp', along=A, facing='down', approach=0.08)`.
   - Then `robot.fingers('right', 'power', seconds=0.7)`.
4. Before lifting, check `robot.state().right.pressing`. It should include the index. The held
   grasps reported thumb, index, middle and ring.
5. Lift 15 cm: `robot.hand('right', by=(0, 0, 0.15), speed=0.10)`.
6. Bring the head over the nail without looking. The head's lowest point follows from the hand's
   state: with `s = robot.state().right`, `P, N, F = s.palm, s.normal, s.finger`,
   `low = P + 0.18*N + 0.13*F - 0.08*np.cross(N, F)` (fitted on L1-L10, within 0.3 cm).
   - If `low` is less than 6 cm above `n.top`, move up by the difference first.
   - Then move straight across so that `low` is over the nail's centre:
     `by=(n.center[0] - low[0], n.center[1] - low[1], 0)`, `speed=0.15`.
7. Strike: `robot.hand('right', by=(0, 0, -0.12), speed=0.20)`. The episode ended during this move
   on L1-L10. If it goes on, move up 12 cm and strike again 14 cm down.

## Verified parameters
These were measured on L1, L3, L5, L7 and L9 (all ten where noted):
- Grasp point below `h.center`:
  - 2.5 cm: 5/5 held and succeeded.
  - 3.5 cm: 10/10 (all ten).
  - 4.5 cm: 5/5.
  - 1.5 cm: 0/5. 'power' pressed with thumb, middle and ring (no index), and the hammer slipped out
    on the lift.
  - 0 cm (at `h.center`): 0/5, no index either.
- `along` horizontal instead of rising: held on all ten. On L9, though, the lift stopped BLOCKED
  13 cm short of its 15 cm, and the carry toward the nail stopped BLOCKED 26 cm short.
- Aim at the strike (where `low` was put relative to the nail):
  - 2 cm toward +x: 5/5.
  - 3 cm toward -x: 5/5.
  - 2 cm toward +y: 2/5. The head came down beside the nail and the strikes stopped BLOCKED.
  So the step 6 formula needs no correction, and an error in y matters more than in x.

## Common failures and fixes
- Grasping too high (at `h.center`, which lies near the handle's upper surface, or anywhere 1.5 cm
  or less below it): 'power' pressed without the index, and the hammer slipped out on the lift or
  was not held at all.
- A first grasp that closed without the index already moved the hammer. In 6 of 10 seeds it fell
  flat, and in 4 it turned and tilted.
  - Grasping again at the first measure: 0/10.
  - Opening, moving up 6 cm, looking again from 46 cm above the handle, measuring it again and
    grasping 3.5 cm below the new centre: 6/10, done at steps 562-635.
  - So make the first grasp as in step 3; a second grasp is a poor rescue.

## Final checks
- Before the strike: `robot.state().right.holding` is True and the index still presses.
- Nothing else is needed: the episode ends by itself on success.

## Typical step costs
- Look 73, grasp 131-135 (two moves and 'power'), lift 30, up and across 78-93, strike about 10-20:
  done at steps 329-353 of the 1000 (L1-L10).
- The models' own successful runs on L1-L10 that kept the hammer at the first grasp took 323-361
  steps. Those that grasped again took 427-619.
