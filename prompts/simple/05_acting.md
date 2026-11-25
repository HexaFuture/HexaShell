# 5. Acting with a humanoid

Plan the body first, then the hands. A hand only gets where the body lets it: where the robot stands
(walk), how high and how far forward the torso is (body), which way it faces (heading, torso yaw).

Standing in the right place:
- Measure the object from where the robot starts (point the camera at the table first), then walk so
  that it ends about 0.3-0.45 m ahead of the pelvis and a little to the side of the hand that will
  take it (the right hand works best on things just right of the pelvis's line, the left on things
  just left of it). Face it: give the walk a heading toward it.
- The legs need room: the robot stops at a table's edge (the walk comes back BLOCKED). Stand with the
  pelvis about 0.25-0.3 m back from the edge and reach over it.
- A walk ends a few centimetres from its target (up to about 5): measure again after it before a
  precise move.
- Walk standing up, then lower and lean. Stepping from a squat or a deep lean is slow and lands far
  off (16 cm at 0.55 m and 45 degrees took 21 s and stopped 11 cm short): stand up first
  (`robot.body(0.75, pitch=0)`), walk, then take the low pose again.

Low and far things:
- Lower the pelvis and lean the torso forward (`robot.body(0.6, pitch=25)`): the shoulders go down and
  forward, and the arms with them (with hold 'torso' a palm moves along). How low the pelvis really
  goes depends on the legs' controller; the task card says which one this task has. Leaning reaches
  farther than squatting; the torso leans up to 60 degrees, but past about 30 the robot is less steady
  and the camera looks down more.
- Check a doubtful reach with `robot.hand(side, xyz, dry_run=True)` before moving; a joint limit note
  means: change the body's pose or where the robot stands, not the hand's target.

Grasping:
1. Choose the grasp: the hand holds a thing between the thumb (out along the palm's normal) and the
   two curled fingers, across a side up to about 7 cm thick. For a box, put the palm against one end
   or side face with the fingers along the thin side, so the thumb and the fingers close around it;
   a round thing (a can) is held from the side with the palm facing it. Pick the face the palm can
   reach with the wrist straight from where the robot stands.
2. Bring the palm to a pre-grasp point 6-10 cm back from the grasp point along the palm's normal,
   with the grasp orientation and the fingers open (`seconds` about 1). The palm moves in straight
   lines from where it is, and the fingers reach about 13 cm ahead of it: after a squat or a lean the
   hands hang low, near the table's edge, and a straight line from there to a point above the object
   can pass through it. Give such a move a `via` point first (straight up from where the hand is, or
   to the side of the object). When the hand comes around an end of the object (the thumb has to
   pass one face on the way in), open it wide first (`robot.fingers(side, 'wide')`), so the thumb
   does not knock the object; close as usual.
3. Move in to the grasp point slowly (`seconds` 1-1.5) and close: `robot.fingers(side, True)`. Check
   `.holding`; if it is False, open, change what made it miss (1-2 cm closer, deeper, lower; another
   face) and close again.
4. Lift a little first (`robot.hand(side, by=(0, 0, 0.05))`) and check `.holding` again before going
   on: a grasp held by two parts only may slip.

Carrying, placing, handing over:
- To walk or turn with something in the hand, give the hand `hold='pelvis'` on its last move before
  the walk; it then moves with the pelvis. With 'torso' it also comes along but swings with every
  lean and torso turn.
- To place: move the object above the spot, lower it until it is a centimetre or two over the
  surface, open the fingers, then move the hand up and away.
- To pass an object from one hand to the other: bring it in front of the chest, bring the other palm
  to the free side of it facing it, close that hand and check `.holding`, then open the first hand and
  move it away.

Using the budget well:
- The official limit is short (8-24 s of simulated time). Thinking, looking and measuring cost
  nothing; motions do. Put several motions in one cell when the code checks each result before the
  next, and avoid motions that only wait or undo each other.
- Typical costs: a 0.4 m walk about 100-150 steps; a body change 40-80 steps; a hand move 30-150
  steps; closing or opening the fingers 25 steps (AMO) or 50 (GR00T's controller); a camera turn a few steps.
