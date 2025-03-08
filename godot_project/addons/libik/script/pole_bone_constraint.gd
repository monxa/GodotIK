@tool
class_name PoleBoneConstraint extends GodotIKConstraint
## Applies a pole vector constraint to an IK bone chain.
##
## This constraint ensures that the mid-bone follows a specific pole direction,
## controlling the plane in which the chain bends.

## Whether the constraint is currently active.
@export var active : bool = true

## The pole direction vector that influences the bending plane.
@export var pole_direction : Vector3

## If true, the constraint applies when the IK chain moves forward.
@export var forward : bool = true

## If true, the constraint applies when the IK chain moves backward.
@export var backward : bool = true

func apply(
		pos_parent_bone: Vector3,
		pos_bone: Vector3,
		pos_child_bone: Vector3,
		chain_dir : Dir
	) -> PackedVector3Array:
	var result : PackedVector3Array = [pos_parent_bone, pos_bone, pos_child_bone]
	if not active:
		return result
	if chain_dir == Dir.FORWARD and not forward:
		return result
	if chain_dir == Dir.BACKWARD and not backward:
		return result

	# Compute the primary bone direction
	var dir = (pos_child_bone - pos_parent_bone).normalized()

	# Project pole direction onto the plane perpendicular to dir
	var pole_direction_normalized = pole_direction.normalized()  # Ensure it's a unit vector
	var pole_direction_projected = pole_direction_normalized - pole_direction_normalized.dot(dir) * dir

	# If the projected pole direction is too small, return early
	if pole_direction_projected.length_squared() < 0.002:
		return result

	# Normalize the projected pole direction
	pole_direction_projected = pole_direction_projected.normalized()

	# Find the perpendicular foot of the bone to the line
	var mid_point = foot_of_the_perpendicular(pos_bone, pos_parent_bone, pos_child_bone)

	# Adjust the bone position to follow the pole constraint
	var mid_to_bone = pos_bone - mid_point
	var bone_length = mid_to_bone.length()  # Store instead of computing twice
	result[1] = mid_point + pole_direction_projected * bone_length

	return result

## Compute the foot of the perpendicular from a point to a line
func foot_of_the_perpendicular(point_tip: Vector3, point_line1: Vector3, point_line2: Vector3) -> Vector3:
	var bc: Vector3 = point_line2 - point_line1  # Vector along the line
	var t: float = (point_tip - point_line1).dot(bc) / bc.length_squared()  # Projection factor
	return point_line1 + t * bc
