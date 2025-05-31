#include "godot_ik_editor_plugin.h"
#include "godot_cpp/classes/button.hpp"
#include "godot_cpp/classes/editor_inspector_plugin.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/callable_method_pointer.hpp"
#include "godot_cpp/variant/string_name.hpp"
#include "godot_ik.h"

using namespace godot;

GodotIKEditorPlugin::GodotIKEditorPlugin() {
	inspector_plugin = Ref(memnew(GodotIKEditorInspectorPlugin));
	add_inspector_plugin(inspector_plugin);
}

void GodotIKEditorPlugin::_bind_methods() {
}

// --------------------------------------------------

GodotIKEditorInspectorPlugin::GodotIKEditorInspectorPlugin() {
}

bool GodotIKEditorInspectorPlugin::_can_handle(Object *p_object) const {
	const StringName obj_name = p_object->get_class();
	if (obj_name == GodotIK::get_class_static()) {
		return true;
	}
	if (obj_name == GodotIKEffector::get_class_static()){
	    return true;
	}
	return false;
}

void GodotIKEditorInspectorPlugin::_parse_end(Object *p_object) {
	const StringName obj_name = p_object->get_class();
	Button *reset_button = memnew(Button);
	reset_button->set_h_size_flags(Control::SizeFlags::SIZE_EXPAND_FILL);
	if (obj_name == GodotIK::get_class_static()) {
		GodotIK *ik_controller = cast_to<GodotIK>(p_object);
	    reset_button->set_text("Reset Effectors");
		reset_button->connect("pressed", callable_mp(ik_controller, &GodotIK::set_effector_transforms_to_bones));
	}
	if (obj_name == GodotIKEffector::get_class_static()) {
		GodotIKEffector *effector = cast_to<GodotIKEffector>(p_object);
		reset_button->set_text("Reset Effector");
		reset_button->connect("pressed", callable_mp(effector, &GodotIKEffector::set_transform_to_bone));
	}

	add_custom_control(reset_button);
}

void GodotIKEditorInspectorPlugin::_bind_methods() {
}
