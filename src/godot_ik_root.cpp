#include "godot_ik_root.h"
#include "godot_ik.h"

using namespace godot;

void GodotIKRoot::_notification(int p_notification) {
	if (p_notification == NOTIFICATION_READY) {
		if (ik_controller_path.is_empty()) {
			return;
		}
		GodotIK *new_ik_controller = get_node<GodotIK>(ik_controller_path);
		if (new_ik_controller) {
			new_ik_controller->add_external_root(this);
		}
		ik_controller = new_ik_controller;
	}
	if (p_notification == NOTIFICATION_EXIT_TREE){
		if (ik_controller){
			ik_controller->remove_external_root(this);
		}
	}
}

void GodotIKRoot::set_ik_controller(const NodePath &p_ik_controller) {
	ik_controller_path = p_ik_controller;
	if (!is_node_ready()) {
		return;
	}

	GodotIK *new_ik_controller = get_node<GodotIK>(ik_controller_path);
	GodotIK *old_ik_controller = ik_controller;
	if (new_ik_controller == old_ik_controller) {
		return;
	}
	if (old_ik_controller) {
		old_ik_controller->remove_external_root(this);
	}
	if (new_ik_controller) {
		new_ik_controller->add_external_root(this);
	}
	ik_controller = new_ik_controller;
}

NodePath GodotIKRoot::get_ik_controller() const {
	return ik_controller_path;
}

void GodotIKRoot::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_ik_controller", "ik_controller"), &GodotIKRoot::set_ik_controller);
	ClassDB::bind_method(D_METHOD("get_ik_controller"), &GodotIKRoot::get_ik_controller);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "ik_controller", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "GodotIK"),
			"set_ik_controller",
			"get_ik_controller");
}
