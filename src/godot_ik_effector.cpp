#include "godot_ik_effector.h"
#include "godot_cpp/core/error_macros.hpp"
#include "godot_ik.h"

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void GodotIKEffector::_bind_methods() {
	// TransformMode
	BIND_ENUM_CONSTANT(POSITION_ONLY);
	BIND_ENUM_CONSTANT(PRESERVE_ROTATION);
	BIND_ENUM_CONSTANT(STRAIGHTEN_CHAIN);
	BIND_ENUM_CONSTANT(FULL_TRANSFORM);

	ClassDB::bind_method(D_METHOD("set_bone_name", "bone_name"), &GodotIKEffector::set_bone_name);
	ClassDB::bind_method(D_METHOD("get_bone_name"), &GodotIKEffector::get_bone_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "bone_name"), "set_bone_name", "get_bone_name");

	ClassDB::bind_method(D_METHOD("set_bone_idx", "bone_idx"), &GodotIKEffector::set_bone_idx);
	ClassDB::bind_method(D_METHOD("get_bone_idx"), &GodotIKEffector::get_bone_idx);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "bone_idx"), "set_bone_idx", "get_bone_idx");

	ClassDB::bind_method(D_METHOD("set_chain_length", "chain_length"), &GodotIKEffector::set_chain_length);
	ClassDB::bind_method(D_METHOD("get_chain_length"), &GodotIKEffector::get_chain_length);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "chain_length", PROPERTY_HINT_RANGE, "1, 32, 1"), "set_chain_length", "get_chain_length");

	ClassDB::bind_method(D_METHOD("set_transform_mode", "transform_mode"), &GodotIKEffector::set_transform_mode);
	ClassDB::bind_method(D_METHOD("get_transform_mode"), &GodotIKEffector::get_transform_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "transform_mode", PROPERTY_HINT_ENUM, "Position Only, Preserve Rotation, Straighten Chain, Full Transform"), "set_transform_mode", "get_transform_mode");

	ClassDB::bind_method(D_METHOD("set_active", "active"), &GodotIKEffector::set_active);
	ClassDB::bind_method(D_METHOD("is_active"), &GodotIKEffector::is_active);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "active"), "set_active", "is_active");

	ClassDB::bind_method(D_METHOD("set_influence", "influence"), &GodotIKEffector::set_influence);
	ClassDB::bind_method(D_METHOD("get_influence"), &GodotIKEffector::get_influence);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "influence", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_influence", "get_influence");

	ADD_SIGNAL(MethodInfo("ik_property_changed"));

	ClassDB::bind_method(D_METHOD("get_skeleton"), &GodotIKEffector::get_skeleton);

	ClassDB::bind_method(D_METHOD("set_transform_to_bone"), &GodotIKEffector::set_transform_to_bone);

	ClassDB::bind_method(D_METHOD("get_ik_controller"), &GodotIKEffector::get_ik_controller);
}

void GodotIKEffector::_validate_property(PropertyInfo &p_property) const {
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

// Setters/Getters -------------------------
void GodotIKEffector::set_bone_name(String p_name) {
	bone_name = p_name;
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton) {
		set_bone_idx(skeleton->find_bone(bone_name));
	}
}

String GodotIKEffector::get_bone_name() const {
	return bone_name;
}

int GodotIKEffector::get_bone_idx() const {
	return bone_idx;
}

void GodotIKEffector::set_bone_idx(int p_bone_idx) {
	int prev_bone_idx = bone_idx;
	bone_idx = p_bone_idx;

	Skeleton3D *skeleton = get_skeleton();

	if (skeleton) {
		if (bone_idx < 0 || bone_idx >= skeleton->get_bone_count()) {
			WARN_PRINT("Bone index out of range! Cannot connect IK Effector to node!");
			bone_idx = 0;
		} else {
			bone_name = skeleton->get_bone_name(bone_idx);
		}
	}

	if (prev_bone_idx != bone_idx) {
		emit_signal("ik_property_changed");
	}
}

int GodotIKEffector::get_chain_length() const {
	return chain_length;
}

void GodotIKEffector::set_chain_length(int p_chain_length) {
	bool changed = p_chain_length != chain_length;
	chain_length = p_chain_length;
	if (changed) {
		emit_signal("ik_property_changed");
	}
}

GodotIKEffector::TransformMode GodotIKEffector::get_transform_mode() const {
	return transform_mode;
}

void GodotIKEffector::set_transform_mode(TransformMode p_transform_mode) {
	transform_mode = p_transform_mode;
}

void GodotIKEffector::set_ik_controller(GodotIK *p_ik_controller) {
	ik_controller = p_ik_controller;
}

GodotIK *GodotIKEffector::get_ik_controller() const {
	return ik_controller;
}

Skeleton3D *GodotIKEffector::get_skeleton() const {
	if (ik_controller == nullptr) {
		return nullptr;
	}
	return ik_controller->get_skeleton();
}

void GodotIKEffector::set_transform_to_bone() {
	Skeleton3D *skeleton = get_skeleton();
	ERR_FAIL_NULL_MSG(skeleton, "[GodotIK] set_transform_to_pose failed - Skeleton could not be retrieved.");

	ERR_FAIL_INDEX_MSG(bone_idx, skeleton->get_bone_count(), "[GodotIK] set_transform_to_pose failed - Effectors bone_idx not in skeletons bones.");
	set_global_transform(skeleton->get_global_transform() * skeleton->get_bone_global_pose(bone_idx));
}

void GodotIKEffector::set_active(bool p_active) {
	active = p_active;
}

bool GodotIKEffector::is_active() const {
	return active;
}

void GodotIKEffector::set_influence(float p_influence) {
	bool changed = influence != p_influence;
	influence = p_influence;
}

float godot::GodotIKEffector::get_influence() const {
	return influence;
}

PackedStringArray GodotIKEffector::_get_configuration_warnings() const {
	PackedStringArray result;
	if (get_ik_controller() == nullptr) {
		result.push_back("Needs to be parented by a GodotIK node. Can be nested.");
	}
	return result;
}
