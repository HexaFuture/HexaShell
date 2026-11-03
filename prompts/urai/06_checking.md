# 6. Checking, recovering and finishing

Read Move/Grip at every key stage: check ok, the errors, holding, the measured width and notes. A refusal or execution error cuts the current cell short; do not assume the gripper operations after it have happened. After looking, name the cause and change the position, orientation, opening or path that made it fail. URAI's original execution error messages are kept, and the same action is never retried automatically.

After grasping, see whether the object lifts with the hand; after releasing, see whether it stays on its support. When the task asks for relations such as "inside", "aligned" or "stacked", check those geometric relations and the other objects the task involves. Pictures and measurements are the evidence; an HTTP completed only means the motion chain ended.

The task budget is checked before each call; one motion may use up or exceed the remaining steps. Consider the time the current stage and the placement still need. There is no automatic success ending and no old benchmark score table. When you have confirmed completion, call finish(done, summary) with an English summary stating what you verified and what remains uncertain; if you cannot continue, use the matching stop status. Without an evaluator the final success stays null.
