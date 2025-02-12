#include "godot_ik_constraint.h"

using namespace godot;

#include <godot_cpp/classes/skeleton3d.hpp>

PackedVector3Array GodotIKConstraint::apply(Vector3 p_pos_parent_bone, Vector3 p_pos_bone, Vector3 p_pos_child_bone, int p_chain_direction) {
	PackedVector3Array result;
	bool success = GDVIRTUAL_CALL(apply, p_pos_parent_bone, p_pos_bone, p_pos_child_bone, p_chain_direction, result);
	if (success == false || result.size() != 3) {
		if (apply_method_implemented) {
			ERR_PRINT("Implement GodotIKConstraint::apply in derived class, that returns (modified) {parent_pos_bone, pos_bone, pos_child_bone}.");
		}
		apply_method_implemented = false;
		return { p_pos_parent_bone, p_pos_bone, p_pos_child_bone };
	}
	// so that error only prints once.
	apply_method_implemented = true;
	return result;
}

int GodotIKConstraint::get_bone_idx() const {
	return bone_idx;
}

void GodotIKConstraint::set_bone_idx(int p_bone_idx) {
	int old_bone_idx = bone_idx;
	bone_idx = p_bone_idx;
	if (old_bone_idx != bone_idx) {
		emit_signal("bone_idx_changed", bone_idx);
	}
}

void GodotIKConstraint::set_skeleton(godot::Skeleton3D *p_skeleton) {
	skeleton = p_skeleton;
}

Skeleton3D *godot::GodotIKConstraint::get_skeleton() const {
	return skeleton;
}

void godot::GodotIKConstraint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_bone_idx"), &GodotIKConstraint::get_bone_idx);
	ClassDB::bind_method(D_METHOD("set_bone_idx", "bone_idx"), &GodotIKConstraint::set_bone_idx);
	GDVIRTUAL_BIND(apply, "pos_parent_bone", "pos_bone", "pos_child_bone", "chain_direction");
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "bone_idx"), "set_bone_idx", "get_bone_idx");
	BIND_ENUM_CONSTANT(BACKWARD);
	BIND_ENUM_CONSTANT(FORWARD);
	ADD_SIGNAL(MethodInfo("bone_idx_changed", PropertyInfo(Variant::Type::INT, "bone_idx")));
	ClassDB::bind_method(D_METHOD("get_skeleton"), &GodotIKConstraint::get_skeleton);
}
