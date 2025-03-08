@tool
class_name MaxRotationBoneConstraint extends GodotIKConstraint
## Constrains the rotation of a bone within a maximum angle.
##
## This constraint limits how much a bone can rotate from its initial orientation,
## ensuring it stays within a given angular range.

## Whether the constraint applies when the IK chain moves forward.
@export var forward : bool

## Whether the constraint applies when the IK chain moves backward.
@export var backward : bool

## Whether the constraint is currently active.
@export var active : bool

## The maximum allowable rotation angle (in radians) from the initial pose.
@export var max_rotation : float

var _initial_rotation : Quaternion

## This function overwrite limits the bone's rotation by ensuring it does not exceed
## the specified `max_rotation` angle from its initial pose.
func apply(
	pos_parent_bone: Vector3,
	pos_bone: Vector3,
	pos_child_bone: Vector3,
	dir: int
	) -> PackedVector3Array:

	var result = [pos_parent_bone, pos_bone, pos_child_bone]
	if not active:
		return result

	var dir_pb = pos_parent_bone.direction_to(pos_bone)
	var dir_bc = pos_bone.direction_to(pos_child_bone)

	## Store the initial rotation on the first iteration.
	if get_ik_controller() and get_ik_controller().get_current_iteration() == 0:
		_initial_rotation = calculate_initial_rotation()

	## Compute the current rotation and the deviation from the initial pose.
	var current_rotation : Quaternion = Quaternion(dir_pb, dir_bc)
	var rotation_to_initial = _initial_rotation * current_rotation.inverse()
	var phi = rotation_to_initial.get_angle()

	## If within the max rotation, no correction is needed.
	if phi <= max_rotation:
		return result

	## Adjust the rotation based on direction and constraints.
	if dir == FORWARD and forward:
		var d = phi - max_rotation
		var adj = Quaternion(rotation_to_initial.get_axis(), d)
		result[2] = pos_bone + adj * dir_bc * (pos_child_bone - pos_bone).length()

	if dir == BACKWARD and backward:
		var d = phi - max_rotation
		var adj = Quaternion(-rotation_to_initial.get_axis(), d)
		result[0] = pos_bone + adj * -dir_pb * (pos_parent_bone - pos_bone).length()

	return result

## Computes the initial rotation of the bone.
##
## This function calculates the initial relative rotation between the parent and child bones
## before any IK solving takes place. This serves as a reference for enforcing the rotation constraint.
##
## @return The initial relative rotation as a Quaternion.
func calculate_initial_rotation() -> Quaternion:
	var bone_parent = get_skeleton().get_bone_parent(bone_idx)
	var bone_children = get_skeleton().get_bone_children(bone_idx)
	assert(bone_children.size() == 1)

	var pos_p = get_skeleton().get_bone_global_pose(bone_parent).origin
	var pos_b = get_skeleton().get_bone_global_pose(bone_idx).origin
	var pos_c = get_skeleton().get_bone_global_pose(bone_children[0]).origin

	var dir_pb = pos_p.direction_to(pos_b)
	var dir_bc = pos_b.direction_to(pos_c)

	return Quaternion(dir_pb, dir_bc)
