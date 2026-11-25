# 2. The world and the robot

Units are metres and degrees. One frame for everything, the **start frame**: where the robot stood at
the start of the episode - origin on the ground under its pelvis, x forward (the way it faced at the
start), y to its left, z up. It stays fixed while the robot walks; `robot.state()` gives where the
pelvis is now and its heading (0 = facing +x, 90 = facing +y). The robot knows its own position
exactly: after a walk, points measured before it are still right as long as nothing moved them.

The robot:
- **Legs**, run by the benchmark's own controller: it keeps the robot balanced, walks where you send
  the pelvis (`robot.walk`) and holds the posture you ask for (`robot.body`): the pelvis's height above
  the ground (about 0.77 m standing), the torso's forward lean (pitch) and its turn left or right of
  the pelvis (yaw). How low the pelvis really goes depends on the controller (a note says so); leaning
  forward brings the hands lower and farther out.
- **Two arms with three-fingered hands** (Dex3: a thumb, an index and a middle finger). Each palm is
  moved on its own (`robot.hand`): to a point, with an orientation, through points on the way. The
  fingers have three settings (`robot.fingers`): open; wide, open with the thumb swung back from the
  fingers; and closed: closing curls them until they touch and then squeezes. The hand feels what it touches: results list the parts in contact ('thumb',
  'index', 'middle', 'palm'), and a closed hand **holds** something when its thumb and one more part
  touch it.
- **A gaze camera** on the head, on a pan-tilt (`robot.gaze`): yaw -45 to 45 degrees left of the
  torso's forward, pitch 0 to 60 down from the torso's horizontal; it turns with the torso. It gives a
  colour and a depth picture, 640 x 400, 90 degrees across; at the start it looks 45 degrees down,
  straight ahead. It is the robot's only camera: to see something, point it there.
- Pixels are (x, y) = (column, row) from the top-left corner. Every tool that takes or gives pixels
  uses this order.

The palm and its orientation:
- A hand command moves the **palm's centre** (the middle of the palm's inner face). The index and
  middle fingers reach about 13 cm beyond it along the finger direction; the thumb stands about 3 cm
  behind it and sticks out about 10 cm along the palm's normal. Closing, the two fingers curl toward
  the thumb: an object is held between them, about 3 cm ahead of the palm's centre and 3-5 cm out
  from the palm.
- The orientation is two directions: the palm's **normal** (the way the inner face of the palm looks)
  and the **finger** direction (from the wrist toward the finger roots). Named ones, in the robot's
  own directions now: 'down' (palm faces the ground, fingers forward), 'up', 'forward' (palm faces
  ahead, fingers up), 'inward' (palm faces the robot's middle: the left palm faces right, the right
  palm faces left; fingers forward), 'outward'. Or give either direction as a start-frame vector.

Reach: each arm reaches about 0.5-0.6 m from its shoulder. What it can reach depends on where the
robot stands and how it holds its body: when a target is out of reach, walk closer, lean, lower the
pelvis or turn the torso rather than stretching. `robot.hand(..., dry_run=True)` says whether the
palm gets there from the body's pose now, without moving anything. The legs need room: the robot
cannot walk into a table; it stops at the table's edge.
