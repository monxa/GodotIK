#include "godot_ik_editor_plugin.h"
#include "godot_cpp/classes/button.hpp"
#include "godot_cpp/classes/editor_inspector_plugin.hpp"
#include "godot_cpp/classes/editor_undo_redo_manager.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/undo_redo.hpp"
#include "godot_cpp/core/error_macros.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/core/object.hpp"
#include "godot_cpp/variant/callable.hpp"
#include "godot_cpp/variant/callable_method_pointer.hpp"
#include "godot_cpp/variant/string_name.hpp"
#include "godot_cpp/variant/typed_array.hpp"
#include "godot_ik.h"

using namespace godot;

GodotIKEditorPlugin::GodotIKEditorPlugin() {
	inspector_plugin = Ref(memnew(GodotIKEditorInspectorPlugin));
	add_inspector_plugin(inspector_plugin);

	inspector_plugin->connect("request_reset_effector", callable_mp(this, &GodotIKEditorPlugin::reset_single_effector));
	inspector_plugin->connect("request_reset_all_effectors", callable_mp(this, &GodotIKEditorPlugin::reset_all_effectors));
}

void GodotIKEditorPlugin::_bind_methods() {
}

void GodotIKEditorPlugin::reset_single_effector(Object *p_effector) {
	GodotIKEffector *effector = Object::cast_to<GodotIKEffector>(p_effector);
	ERR_FAIL_NULL_MSG(effector, "[GodotIK] reset_single_effector: Effector is null or of the wrong type.");


	get_undo_redo()->create_action("Reset Single Effector Transform");
	get_undo_redo()->add_do_method(p_effector, "set_transform_to_bone");
	get_undo_redo()->add_undo_property(p_effector, "transform", effector->get_transform());
	get_undo_redo()->commit_action();
}

void GodotIKEditorPlugin::reset_all_effectors(Object *p_ik_controller) {
	GodotIK *ik_controller = Object::cast_to<GodotIK>(p_ik_controller);
	ERR_FAIL_NULL_MSG(ik_controller, "[GodotIK] reset_all_effectors: IK controller is null or of the wrong type.");

	get_undo_redo()->create_action("Reset All GodotIK Effector Transforms");
	get_undo_redo()->add_do_method(ik_controller, "set_effector_transforms_to_bones");
	TypedArray<GodotIKEffector> all_effectors = ik_controller->get_effectors();
	for (int i = 0; i < all_effectors.size(); i++) {
		GodotIKEffector *effector = cast_to<GodotIKEffector>(all_effectors[i]);
		get_undo_redo()->add_undo_property(effector, "transform", effector->get_transform());
	}
	get_undo_redo()->commit_action();
}

// --------------------------------------------------

GodotIKEditorInspectorPlugin::GodotIKEditorInspectorPlugin() {
}

void GodotIKEditorInspectorPlugin::_bind_methods() {
	ADD_SIGNAL(MethodInfo("request_reset_effector", PropertyInfo(Variant::OBJECT, "effector")));
	ADD_SIGNAL(MethodInfo("request_reset_all_effectors", PropertyInfo(Variant::OBJECT, "ik_controller")));
}

bool GodotIKEditorInspectorPlugin::_can_handle(Object *p_object) const {
	const StringName obj_name = p_object->get_class();
	if (obj_name == GodotIK::get_class_static()) {
		return true;
	}
	if (obj_name == GodotIKEffector::get_class_static()) {
		return true;
	}
	return false;
}

void GodotIKEditorInspectorPlugin::_parse_end(Object *p_object) {
	const StringName obj_name = p_object->get_class();
	Button *reset_button = memnew(Button);

	reset_button->set_h_size_flags(Control::SizeFlags::SIZE_EXPAND_FILL);
	if (obj_name == GodotIK::get_class_static()) {
		reset_button->set_text("Reset Effectors");

		Callable c_emit_signal = Callable(this, "emit_signal").bind("request_reset_all_effectors", p_object);
		reset_button->connect("pressed", c_emit_signal);
	}
	if (obj_name == GodotIKEffector::get_class_static()) {
		reset_button->set_text("Reset Effector");

		Callable c_emit_signal = Callable(this, "emit_signal").bind("request_reset_effector", p_object);
		reset_button->connect("pressed", c_emit_signal);
	}

	add_custom_control(reset_button);
}
