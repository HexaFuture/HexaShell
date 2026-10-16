Legacy disabled commands require calibrated RGB-D observations; physical motions and settling consume action steps.
`robo compose_transfer {left,right} --source_u U --source_v V --target_u U --target_v V --floor Z [--via_x X --via_y Y --via_z Z]` prepares and executes single-use lift/place phases with public effect checks.
`robo compose_bridge {left,right} ... --second_u U --second_v V` adds a measured two-support contact gate; selected geometry refusals permit up to three preparation retries with two-step settling.
`robo compose_lift_carry {left,right} ... [--carry_yaw DEG] [--landing vertical|diagonal] [--carry_to_x X --carry_to_y Y --carry_to_z Z]` verifies a prepared lift, then invokes carry and a placement-effect check; partial target overrides fall back to the recipe target.
Carry turns with magnitude >=90 degrees permit one refreshed zero-yaw waypoint recovery before release, then one endpoint recovery on IK refusal; consumed lift tickets are never replayed.
Returns plan_ok, plan_fail_reason, motion_sent, phase results and object_effect_verified; lift/preparation failures stop dependent motion. Carry output includes carry_attempts and release status.
A requested release can return plan_ok=true with object_effect_verified=false and released_after_retreat_error=true; this does not certify stable contact or official completion.
Pixels, coordinates and tickets depend on the current observation and episode; missing depth, invalid geometry, reach failures and unverified effects can refuse execution.
