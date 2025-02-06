@tool
class_name GodotIKWithTooling extends GodotIK

## This only works with godot 4.4+ WE NEED SOME EditorPlugin to provide tooling, please <3
#@export_tool_button("Reset all effectors") var t = func():
	#var skeleton = get_skeleton()
	#skeleton.reset_bone_poses()
#
	#for effector : GodotIKEffector in get_children():
		#var trans = skeleton.global_transform * skeleton.get_bone_global_pose(effector.bone_idx)
		#effector.global_transform = trans
