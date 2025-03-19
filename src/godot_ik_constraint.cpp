#include "godot_ik_constraint.h"
#include "godot_ik.h"

#include <godot_cpp/classes/skeleton3d.hpp>

using namespace godot;

void GodotIKConstraint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_bone_name"), &GodotIKConstraint::get_bone_name);
	ClassDB::bind_method(D_METHOD("set_bone_name", "bone_name"), &GodotIKConstraint::set_bone_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "bone_name"), "set_bone_name", "get_bone_name");

	ClassDB::bind_method(D_METHOD("get_bone_idx"), &GodotIKConstraint::get_bone_idx);
	ClassDB::bind_method(D_METHOD("set_bone_idx", "bone_idx"), &GodotIKConstraint::set_bone_idx);
	GDVIRTUAL_BIND(apply, "pos_parent_bone", "pos_bone", "pos_child_bone", "chain_direction");
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "bone_idx"), "set_bone_idx", "get_bone_idx");
	BIND_ENUM_CONSTANT(BACKWARD);
	BIND_ENUM_CONSTANT(FORWARD);
	ADD_SIGNAL(MethodInfo("bone_idx_changed", PropertyInfo(Variant::Type::INT, "bone_idx")));
	ClassDB::bind_method(D_METHOD("get_skeleton"), &GodotIKConstraint::get_skeleton);
	ClassDB::bind_method(D_METHOD("get_ik_controller"), &GodotIKConstraint::get_ik_controller);
}

PackedVector3Array GodotIKConstraint::apply(Vector3 p_pos_parent_bone, Vector3 p_pos_bone, Vector3 p_pos_child_bone, int p_chain_direction) {
	PackedVector3Array result;
	bool success = GDVIRTUAL_CALL(apply, p_pos_parent_bone, p_pos_bone, p_pos_child_bone, p_chain_direction, result);
	if (success == false || result.size() != 3) {
		if (apply_method_implemented) {
			ERR_PRINT("Implement GodotIKConstraint::apply in derived class that returns (modified) {parent_pos_bone, pos_bone, pos_child_bone}.");
		}
		apply_method_implemented = false;
		return { p_pos_parent_bone, p_pos_bone, p_pos_child_bone };
	}
	// so that error only prints once.
	apply_method_implemented = true;
	return result;
}

void GodotIKConstraint::_validate_property(PropertyInfo &p_property) const {
	if (p_property.name == StringName("bone_name")) {
		Skeleton3D *skeleton = get_skeleton();
		if (skeleton) {
			p_property.hint = PROPERTY_HINT_ENUM;
			p_property.hint_string = skeleton->get_concatenated_bone_names();
		} else {
			p_property.hint = PROPERTY_HINT_NONE;
			p_property.hint_string = "";
		}
	}
}

String GodotIKConstraint::get_bone_name() const {
	return bone_name;
}

void GodotIKConstraint::set_bone_name(String p_name) {
	bone_name = p_name;
	Skeleton3D *skeleton = get_skeleton();

	if (skeleton) {
		set_bone_idx(skeleton->find_bone(bone_name));
	}
}

int GodotIKConstraint::get_bone_idx() const {
	return bone_idx;
}

void GodotIKConstraint::set_bone_idx(int p_bone_idx) {
	int old_bone_idx = bone_idx;
	bone_idx = p_bone_idx;

	Skeleton3D *skeleton = get_skeleton();

	if (skeleton) {
		if (bone_idx < 0 || bone_idx >= skeleton->get_bone_count()) {
			WARN_PRINT("Bone index for IKConstraint not in Skeleton! Constraint " + get_name() + " will be ignored.");
			bone_idx = 0;
		} else {
			bone_name = skeleton->get_bone_name(bone_idx);
		}
	}

	if (old_bone_idx != bone_idx) {
		emit_signal("bone_idx_changed", bone_idx);
	}
}

void GodotIKConstraint::set_ik_controller(GodotIK *p_ik_controller) {
	ik_controller = p_ik_controller;
}

GodotIK *GodotIKConstraint::get_ik_controller() const {
	return ik_controller;
}

Skeleton3D *GodotIKConstraint::get_skeleton() const {
	if (!ik_controller) {
		return nullptr;
	}
	return ik_controller->get_skeleton();
}
