# 4. Seeing and locating points

Look before you move, and confirm from two cameras when needed. Use look with a grid, marks, crop or zoom to inspect the object, choose pixels on visible surfaces, and convert them with to_world. There is no segmentation service, automatic object measurement or tracked object list; find, measure and things are not available.

Use to_world's measured-depth route first. If it returns xyz=None, obtain a new view, use an explicitly known horizontal plane with z, or triangulate the same visible point with pair=(cam2,xy2). Never assume missing depth means the point lies on the table. Check Spot.warning and mm_per_px: transparency, reflections, occlusion and mixed depth at boundaries can make a point unreliable. A single surface point does not establish the object's full size or hidden shape.

Cross-check world points by projecting them with to_pixel and marking them in look. In the full K, fx and fy can differ and the principal point can be off-centre. The robot's own pixels come from projecting URAI's collision capsules plus measured depth; the model is coarser than the real surfaces, so do not grasp from hidden pixels in occluded areas.

After moving the arm or an object, inspect a fresh picture and locate the relevant points again. Old coordinates are not evidence that an object stayed put, and there is no automatic object tracking. The conversation keeps the starting pictures and the most recent ones; older pictures are replaced by a note, so use look to see the current state.
