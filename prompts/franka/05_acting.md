# 5. Acting

Grasp in small, observable stages: open and move above the object, descend, close, check the opening and holding, lift, confirm the object moves with the hand, move above the target, descend, open, back away. An ordinary motion holds its final pose. Several motions can run in one cell, but check the return values at key steps such as grasping and releasing.

The service plans with the FR3's inverse kinematics and checks the joint limits, the table and the arm's own links. It knows nothing else in the scene, so lift above visible obstacles before moving across. When a line is refused, read the reason and choose a new target or intermediate point instead of repeating the same parameters. An explicit swing moves the TCP on a curve; check it with dry_run near obstacles.

The arm has seven joints, and the service picks the elbow nearest the current one. Omit the angles to keep an object's orientation; changing the angles rotates the object and the wrist camera. home keeps the gripper and may carry an object back to the start; when it must be released, put it down and open the gripper explicitly.
