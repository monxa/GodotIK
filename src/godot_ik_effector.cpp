#include "godot_ik_effector.h"

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;
void GodotIKEffector::_bind_methods() {
	BIND_ENUM_CONSTANT(POSITION_ONLY);
	BIND_ENUM_CONSTANT(PRESERVE_ROTATION);
	BIND_ENUM_CONSTANT(STRAIGHTEN_CHAIN);
	BIND_ENUM_CONSTANT(FULL_TRANSFORM);

	ClassDB::bind_method(D_METHOD("set_bone_idx", "bone_idx"), &GodotIKEffector::set_bone_idx);
	ClassDB::bind_method(D_METHOD("get_bone_idx"), &GodotIKEffector::get_bone_idx);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "bone_idx"), "set_bone_idx", "get_bone_idx");

	ClassDB::bind_method(D_METHOD("set_chain_length", "chain_length"), &GodotIKEffector::set_chain_length);
	ClassDB::bind_method(D_METHOD("get_chain_length"), &GodotIKEffector::get_chain_length);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "chain_length", PROPERTY_HINT_RANGE, "1, 32, 1"), "set_chain_length", "get_chain_length");

	ClassDB::bind_method(D_METHOD("set_leaf_behavior", "transform_mode"), &GodotIKEffector::set_leaf_behavior);
	ClassDB::bind_method(D_METHOD("get_leaf_behavior"), &GodotIKEffector::get_leaf_behavior);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "transform_mode", PROPERTY_HINT_ENUM, "Position Only, Preserve Rotation, Straighten Chain, Full Transform"), "set_leaf_behavior", "get_leaf_behavior");
}

int GodotIKEffector::get_bone_idx() {
	return bone_idx;
}

void GodotIKEffector::set_bone_idx(int p_bone_id) {
	bone_idx = p_bone_id;
}

int GodotIKEffector::get_chain_length() {
	return chain_length;
}

void GodotIKEffector::set_chain_length(int p_chain_length) {
	chain_length = p_chain_length;
}

GodotIKEffector::TransformMode GodotIKEffector::get_leaf_behavior() {
	return transform_mode;
}

void GodotIKEffector::set_leaf_behavior(TransformMode p_leaf_behavior) {
	transform_mode = p_leaf_behavior;
}
