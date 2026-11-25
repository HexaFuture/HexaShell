# 3. robot API

- robot.goto(xyz=None, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto", dry_run=False, contact=False, gentle=False): xyz accepts a 3-tuple, Spot or ArmStatus. Without xyz the measured TCP is used; by is a world-frame offset from that position. grip is done before the motion starts; to put an object down, call gripper separately after arriving. Returns Move: check ok, the errors and notes.
- path="line" asks for a strict Cartesian straight line; "swing" asks for a joint-space curve; "auto" tries the line first and, if that fails, accepts only a joint curve that stays within 3 cm of the line and no more than 1 cm below the lower of the two ends; beyond that you need an intermediate point or an explicit swing.
- dry_run=True only plans: it sends no motion or gripper command and uses no budget; Move.steps gives the expected number of steps, and a refusal returns reached=False. An ordinary planning refusal raises NotExecuted and cuts the current cell short. An execution error, such as the robot's collision reflex, ends the cell with franky's message; an operator must then recover the robot.
- gentle=True caps the whole motion at 0.03 m/s; use it for the last centimetres towards contact. contact=True adds no stall check; the robot's own reflexes still apply.
- robot.gripper(opening): open/close/0..1; changes the opening while holding the pose and returns Grip. Closing on air is legal; check holding and width_m before lifting.
- robot.home(): plans back to the session's initial joints, keeps the gripper, returns Homing.
- robot.wait(n): n=1..200; holds for n*0.04 s; returns Waited.
- robot.to_world(cam, xy, z=None, pair=None): reads the frame's full depth by default; z is a horizontal world plane; pair=(cam2,xy2) triangulates from two views. Returns Spot; with missing depth xyz=None, and the table is not assumed.
- robot.to_pixel(xyz, cam="external"): projects to a pixel; returns None outside the view or behind the lens.
- robot.task: the task, budget and session start pose. robot.state(): measured TCP, angles, joints, opening and holding.
- robot.image(cam="external"): the current uint8 RGB array. robot.camera(cam="external"): the camera's full K and world pose.
- robot.look(cams="external", grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False, note=None): attaches pictures to the cell. cams can be a list or all; world marks use ('world',(x,y,z),'label'); masks accepts supplied outlines; crop=(x0,y0,x1,y1) keeps original image coordinates; tile puts them side by side.
