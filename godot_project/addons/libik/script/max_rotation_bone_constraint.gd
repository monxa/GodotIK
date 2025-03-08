@tool
extends GodotIKConstraint

@export var forward : bool
@export var backward : bool
@export var active : bool

## Constrain rotation around this axis
@export var max_rotation : float

var initialized = false

var initial_rotation : Quaternion

func apply(pos_parent_bone: Vector3,
	pos_bone: Vector3,
	pos_child_bone: Vector3,
	dir: int
	) -> PackedVector3Array:
	var result = [pos_parent_bone, pos_bone, pos_child_bone]
	if not active:
		return result

	var dir_pb = pos_parent_bone.direction_to(pos_bone)
	var dir_bc = pos_bone.direction_to(pos_child_bone)

	if get_ik_controller().get_current_iteration() == 0:
		initial_rotation = calculate_initial_rotation()

	var current_rotation : Quaternion = Quaternion(dir_pb, dir_bc)
	var rotation_to_initial = initial_rotation * current_rotation.inverse()
	var phi = rotation_to_initial.get_angle()
	if phi <= max_rotation:
		return result

	if dir == FORWARD and forward:
		var d = phi - max_rotation
		var adj = Quaternion(rotation_to_initial.get_axis(), d)
		result[2] = pos_bone + adj * dir_bc * (pos_child_bone - pos_bone).length()

	if dir == BACKWARD and backward:
		var d = phi - max_rotation
		var adj = Quaternion(-rotation_to_initial.get_axis(), d)
		result[0] = pos_bone + adj * -dir_pb * (pos_parent_bone - pos_bone).length()
	return result

func calculate_initial_rotation() -> Quaternion:
	var bone_parent = get_skeleton().get_bone_parent(bone_idx)
	var bone_childen = get_skeleton().get_bone_children(bone_idx)
	assert(bone_childen.size() == 1)
	var pos_p = get_skeleton().get_bone_global_pose(bone_parent).origin
	var pos_b = get_skeleton().get_bone_global_pose(bone_idx).origin
	var pos_c = get_skeleton().get_bone_global_pose(bone_childen[0]).origin

	var dir_pb = pos_p.direction_to(pos_b)
	var dir_bc = pos_b.direction_to(pos_c)

	return Quaternion(dir_pb, dir_bc)
