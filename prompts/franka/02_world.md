# 2. World, control point and units

World coordinates are the rig's: metres, z up. t_world_base, the table height table_z, the TCP offset and the maximum gripper opening are those in the task card's model data. Do not assume any other table height.

The control point is the gripping centre on the hand's axis, between the fingers, tcp_offset_mm along the flange; the finger tips reach tip_past_tcp_mm past it (in the same model data). A TCP closer to the table than the tips' reach plus a small margin is refused. When grasping, choose the depth from the object's thickness and visible position.

Angles follow HexaShell's pitch/yaw/roll in degrees: pitch 0 is horizontal, 90 points down; when pointing down, yaw 0 puts the closing axis along world x and yaw 90 along world y; roll turns about the approach axis. An omitted angle keeps the full rotation of the previous request.

Gripper commands: open=1, close=0; values in between are multiplied by the maximum opening on the task card. Grip.width_m is the measured gap; holding is inferred from the difference between commanded and measured opening, and a thin object can look fully closed. Camera names are external (fixed) and wrist (on the hand). Pixels are always (u,v) = (column,row) with the origin at the top left.
