@tool
class_name StraightBoneConstraint extends GodotIKConstraint
## Ensures a joint remains straight between the parent and child bones.
##
## This constraint forces the mid-bone to align perfectly along the axis
## between the parent and child bones, removing any bending.

## Whether the constraint is currently active.
@export var active : bool = true

## If true, the constraint applies when the IK chain moves forward.
@export var forward : bool = true

## If true, the constraint applies when the IK chain moves backward.
@export var backward : bool = true


## This function overwrite modifies the mid-bone position to ensure it stays in a straight line
## between the parent and child bones.
func apply(
		pos_parent_bone: Vector3,
		pos_bone: Vector3,
		pos_child_bone: Vector3,
		chain_dir : Dir
	) -> PackedVector3Array:
	var result : PackedVector3Array = [pos_parent_bone, pos_bone, pos_child_bone]

	if not active: return result
	if not forward and FORWARD or not backward and BACKWARD:
		return result

	var dir_parent_child = pos_parent_bone.direction_to(pos_child_bone)
	var len_parent_bone = pos_parent_bone.distance_to(pos_bone)
	var len_bone_child = pos_bone.distance_to(pos_child_bone)
	var vec_parent_bone = pos_bone - pos_parent_bone

	match chain_dir:
		FORWARD:
			result[1] = pos_parent_bone + dir_parent_child * len_parent_bone
			result[2] = pos_parent_bone + dir_parent_child * (len_parent_bone + len_bone_child)
		BACKWARD:
			result[1] = pos_child_bone - dir_parent_child * len_bone_child
			result[0] = pos_child_bone - dir_parent_child * (len_parent_bone + len_bone_child)
	return result
