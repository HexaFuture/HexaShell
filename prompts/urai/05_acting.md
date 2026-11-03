# 5. Acting

Grasp in small, observable stages: open and move above the object, descend, close, check the opening and holding, lift, confirm the object moves with the hand, move above the target, descend, open, back away. An ordinary motion holds its final pose, and later cells can carry on from it. Several motions can run in one cell, but check the return values at key steps such as grasping and releasing.

URAI uses the original IK, joint/table/self/inter-arm collision checks, time scaling and local 50 Hz execution. There is no general collision-planning guarantee against arbitrary scene objects; lift above visible obstacles before moving across. When a line is refused, read the reason and choose a new target or intermediate point instead of repeating the same unreachable parameters. An explicit swing allows a curved path, and you must make sure the space around it is clear yourself.

Do not copy ARX X5 joint values, table heights, grasp offsets, gripper fits or step counts. Omit the angles to keep an object's orientation; changing the angles rotates the object and the wrist camera. The other arm takes part in URAI's checks even when it does not move. home keeps the gripper and may carry an object back to the start; when it must be released, put it down and open the gripper explicitly first.
