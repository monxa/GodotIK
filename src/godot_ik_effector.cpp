#include "godot_ik_effector.h"
#include "godot_ik.h"

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/godot.hpp>

#include <godot_cpp/classes/skeleton3d.hpp>

using namespace godot;

void GodotIKEffector::_bind_methods() {
	// TransformMode
	BIND_ENUM_CONSTANT(POSITION_ONLY);
	BIND_ENUM_CONSTANT(PRESERVE_ROTATION);
	BIND_ENUM_CONSTANT(STRAIGHTEN_CHAIN);
	BIND_ENUM_CONSTANT(FULL_TRANSFORM);

	// TwistMode
	BIND_ENUM_CONSTANT(DYNAMIC);
	BIND_ENUM_CONSTANT(PRESERVE_TWIST);

	ClassDB::bind_method(D_METHOD("set_bone_idx", "bone_idx"), &GodotIKEffector::set_bone_idx);
	ClassDB::bind_method(D_METHOD("get_bone_idx"), &GodotIKEffector::get_bone_idx);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "bone_idx"), "set_bone_idx", "get_bone_idx");

	ClassDB::bind_method(D_METHOD("set_chain_length", "chain_length"), &GodotIKEffector::set_chain_length);
	ClassDB::bind_method(D_METHOD("get_chain_length"), &GodotIKEffector::get_chain_length);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "chain_length", PROPERTY_HINT_RANGE, "1, 32, 1"), "set_chain_length", "get_chain_length");

	ClassDB::bind_method(D_METHOD("set_transform_mode", "transform_mode"), &GodotIKEffector::set_transform_mode);
	ClassDB::bind_method(D_METHOD("get_transform_mode"), &GodotIKEffector::get_transform_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "transform_mode", PROPERTY_HINT_ENUM, "Position Only, Preserve Rotation, Straighten Chain, Full Transform"), "set_transform_mode", "get_transform_mode");

	ClassDB::bind_method(D_METHOD("set_twist_mode", "twist_mode"), &GodotIKEffector::set_twist_mode);
	ClassDB::bind_method(D_METHOD("get_twist_mode"), &GodotIKEffector::get_twist_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "twist_mode", PROPERTY_HINT_ENUM, "Dynamic, Preserve Twist"), "set_twist_mode", "get_twist_mode");

	ADD_SIGNAL(MethodInfo("bone_idx_changed", PropertyInfo(Variant::Type::INT, "bone_idx")));
	ADD_SIGNAL(MethodInfo("chain_length_changed", PropertyInfo(Variant::Type::INT, "chain_length")));

	ClassDB::bind_method(D_METHOD("get_skeleton"), &GodotIKEffector::get_skeleton);

	ClassDB::bind_method(D_METHOD("get_ik_controller"), &GodotIKEffector::get_ik_controller);
}

// Setters/Getters -------------------------

int GodotIKEffector::get_bone_idx() const {
	return bone_idx;
}

void GodotIKEffector::set_bone_idx(int p_bone_idx) {
	int prev_bone_idx = bone_idx;
	bone_idx = p_bone_idx;
	if (prev_bone_idx != bone_idx) {
		emit_signal("bone_idx_changed", bone_idx);
	}
}

int GodotIKEffector::get_chain_length() const {
	return chain_length;
}

void GodotIKEffector::set_chain_length(int p_chain_length) {
	int prev_chain_length = chain_length;
	chain_length = p_chain_length;
	if (prev_chain_length != chain_length) {
		emit_signal("chain_length_changed", chain_length);
	}
}

GodotIKEffector::TransformMode GodotIKEffector::get_transform_mode() const {
	return transform_mode;
}

void GodotIKEffector::set_transform_mode(TransformMode p_transform_mode) {
	transform_mode = p_transform_mode;
}

void godot::GodotIKEffector::set_twist_mode(TwistMode p_twist_mode) {
	twist_mode = p_twist_mode;
	update_configuration_warnings();
}

GodotIKEffector::TwistMode godot::GodotIKEffector::get_twist_mode() const {
	return twist_mode;
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

PackedStringArray godot::GodotIKEffector::_get_configuration_warnings() const {
	PackedStringArray result;
	if (get_ik_controller() == nullptr){
		result.push_back("Needs to be parented by a GodotIK node. Can be nested.");
	}
	if (get_twist_mode() == TwistMode::PRESERVE_TWIST && !has_one_pole){
		result.push_back("Exactly one GodotIKPole child required when using twist_mode = PRESERVE_TWIST for stable twisting.");
	}
	return result;
}

GodotIKPole *godot::GodotIKEffector::get_pole() {
	return pole;
}

void godot::GodotIKEffector::set_pole(GodotIKPole *p_pole) {
	pole = p_pole;
}
