# 2. The world and the robot

Units are metres and degrees. One fixed frame for everything, the **scene frame**: z up, the robot
faces +x, y to its left. The robot does not move about: its arms stand where they are mounted.

The scene:
- A table in front of the robot: its top spans x -0.55 to 0.25 and y -0.8 to 0.8, and its height
  differs from scene to scene (0.92-0.98 m): measure it, do not assume it.
- **The arms** (Franka Panda, seven joints each) are mounted 0.92 m high, 25 cm behind the table's
  near edge: one arm at (-0.8, 0) in the one-arm tasks; a left arm at (-0.8, 0.4) and a right arm at
  (-0.8, -0.4) in the two-arm tasks (the microwave task: x -1.05, the right arm at y -0.32). The task
  card names the arms of this task and where each hand starts: palm down, fingers forward, about
  0.55 m above the table.

The hands (Wuji: five fingers, four joints each):
- A hand command moves one **point** of the hand: 'palm' (the centre of the palm's inner face),
  'grasp' (where a closed hand holds a handle or a box: 5.5 cm along the fingers from the palm's
  centre and 5 cm out from the palm's face), a fingertip ('thumb', 'index', 'middle', 'ring',
  'little': the middle of its pad) or 'camera' (the wrist camera, below).
- The orientation is two directions: the palm's **normal** (the way the palm's inner face looks) and
  the **finger** direction (from the wrist toward the finger roots). Named ones: 'down' (palm down,
  fingers forward: the start), 'up' (palm up, fingers forward), 'forward' (palm ahead, fingers down),
  'inward' (palm toward the robot's middle: the right palm faces +y, the left -y; fingers forward),
  'outward'. Or give either direction as a vector.
- With the fingers open, the four fingertips reach 12-14 cm beyond the palm's centre along the
  fingers, and stand 3-4 cm back from the palm's face (the open fingers bend back a little). The
  thumb lies beside the palm, its tip 12 cm to the thumb's side (with the palm down: toward the
  robot's middle - the right hand's thumb at +y, the left hand's at -y) and level with the palm's
  face. From thumb tip to little fingertip the open hand spans about 16 cm.
- **The fingers** take named shapes or a curl per finger (section 3). A finger that something stops
  keeps pressing toward where it was set: that is a grip's squeeze.
- **The hands have no touch sensors.** What the robot feels comes from its finger joints: a finger
  that something outside the hand pushes on (felt as a push on its joints) **presses on** something;
  a hand whose thumb and at least one more finger press on something is **holding** it; a finger
  pushed on during a motion touched something on the way. Fingers meeting each other or the palm
  (curled far, or the thumb swung across onto the index or middle finger) are not a touch. A push
  straight along a finger, toward its root, turns none of its joints and is not felt. A palm, a back
  of the hand or an arm that bumps into something is felt only as a motion that stops short
  (section 3: BLOCKED).

Reach (from the reach check that `dry_run` uses, which real motions bore out; x for arms mounted at
x -0.8 - in the microwave task, mounted at -1.05, take 0.25 off every x):
- Palm down, in line with the arm (y 0 in the one-arm tasks, +-0.4 in the two-arm tasks): x -0.40
  to 0.10 with the palm at z 0.97-1.22, x -0.60 to 0.05 at z 1.42. Nearer the mount the elbow
  (joint 4) is at the end of its travel; past x 0.10 the arm is stretched out.
- To the side (palm down, z 1.07): 0.2 m off the arm's line x -0.45 to 0.10, 0.4 m off x -0.65 to
  0.00, 0.6 m off x -0.65 to -0.20. In the two-arm tasks each arm works on its own side: the left
  hand from y 0.2 to 0.6, the right from -0.6 to -0.2.
- How the hand is turned matters as much as where it is:
  - palm sideways ('inward', 'outward'): about as far out as palm down (to x 0.05), and nearer the
    mount (to x -0.65 at z 0.97, -0.45 at z 1.22);
  - palm tilted 30-45 degrees from down toward forward (normal (0.5, 0, -0.87) to (0.71, 0, -0.71),
    the fingers forward and up): out to x 0.15, but low down not nearer than x -0.25 (30 degrees) or
    -0.05 (45 degrees): joint 6 is at the end of its travel there;
  - palm tilted 45 degrees down and back toward the robot, the fingers forward and down (normal
    (-0.71, 0, -0.71), finger (0.71, 0, -0.71)), also with the fingers turned up to 30 degrees
    toward the robot's middle (the grasp for small things, section 5): the grasp point from x -0.50
    to -0.05 at z 0.97-1.10, up to 0.15 m off the arm's line; nearer than x -0.50 the elbow (joint 4)
    is at the end of its travel. Tilted 30 degrees: x -0.40 to 0.00, and 40 degrees in between;
  - 'forward' (palm ahead, fingers down) is at the edge of reach almost everywhere, and at z
    1.07-1.22 joint 5 stops it over most of the table; palm ahead with the fingers up or sideways
    does not reach at all;
  - 'up' is a half turn of the hand from the start: on the way joint 5 may come to the end of its
    travel.
- `dry_run=True` checks a motion before it is made: it follows the motion's path with the arm's
  joints - not what the hand or arm would touch on the way - and says reachable, at the edge of
  reach (a real motion may stop short), or NOT reachable, and which joint stops it. Nothing moves.

Cameras (colour and depth pictures, 512 x 512):
- **scene**: fixed. In the one-arm tasks it stands across the table, facing the robot (the robot's
  right is on the picture's left); in the two-arm tasks it stands behind and above the robot and
  looks over it at the table (the robot's right is on the picture's right). It sees the whole table,
  but from 1.5-2 m away: small things are a few dozen pixels wide in it.
- **right_wrist** / **left_wrist**: one on each hand of the task, 23.5 cm behind the palm's centre,
  looking along the fingers, tilted 30 degrees toward the palm's side, 60 degrees across. With the
  palm down and the fingers forward it looks ahead and 30 degrees down: it sees what lies in front of
  the fingers, not what is under the hand. It moves with the hand; `robot.hand(side, xyz,
  point='camera', look_at=target)` places it to look at a point from close by (section 4).
- Pixels are (x, y) = (column, row) from the top-left corner. Every tool that takes or gives pixels
  uses this order.
