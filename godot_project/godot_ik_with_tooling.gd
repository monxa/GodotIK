@tool
class_name GodotIKWithTooling extends GodotIK

@export_tool_button("Reset all effectors") var t = func():
	var skeleton = get_skeleton()
	skeleton.reset_bone_poses()

	for effector : GodotIKEffector in get_children():
		var trans = skeleton.global_transform * skeleton.get_bone_global_pose(effector.bone_idx)
		effector.global_transform = trans
