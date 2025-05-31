#include "editor/godot_ik_editor_plugin.h"
#include "godot_cpp/classes/editor_plugin_registration.hpp"
#include "godot_ik.h"
#include "godot_ik_constraint.h"
#include "godot_ik_effector.h"
#include "godot_ik_root.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_lib_ikworks(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(GodotIK);
		GDREGISTER_CLASS(GodotIKEffector);
		GDREGISTER_CLASS(GodotIKConstraint);
		GDREGISTER_CLASS(GodotIKRoot);
	}

	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		GDREGISTER_INTERNAL_CLASS(GodotIKEditorPlugin);
		GDREGISTER_INTERNAL_CLASS(GodotIKEditorInspectorPlugin);
		EditorPlugins::add_by_type<GodotIKEditorPlugin>();
	}
}

void terminate_lib_ikworks(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		EditorPlugins::remove_by_type<GodotIKEditorPlugin>();
	}
}

extern "C" {
GDExtensionBool GDE_EXPORT ikworks_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_lib_ikworks);
	init_obj.register_terminator(terminate_lib_ikworks);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}
