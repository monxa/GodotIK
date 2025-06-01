#ifndef GODOT_IK_EDITOR_PLUGIN_H
#define GODOT_IK_EDITOR_PLUGIN_H
#include <godot_cpp/classes/editor_inspector_plugin.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/wrapped.hpp>

namespace godot {

class GodotIKEditorInspectorPlugin;

// ---------------------------------------------------------------

class GodotIKEditorPlugin : public EditorPlugin {
	GDCLASS(GodotIKEditorPlugin, EditorPlugin)
public:
	GodotIKEditorPlugin();

protected:
	static void _bind_methods();

private:
	void reset_single_effector(Object *p_effector);
	void reset_all_effectors(Object *p_ik_controller);

private:
	Ref<GodotIKEditorInspectorPlugin> inspector_plugin;
}; // ! GodotIKEditorPlugin

// ------------------------------------------------------------------

class GodotIKEditorInspectorPlugin : public EditorInspectorPlugin {
	GDCLASS(GodotIKEditorInspectorPlugin, EditorInspectorPlugin)
public:
	GodotIKEditorInspectorPlugin();
	bool _can_handle(Object *p_object) const override;
	void _parse_end(Object *p_object) override;

protected:
	static void _bind_methods();

}; //! GodotIKEditorInspectorPlugin

// ---------------------------------------------------------------

} //namespace godot
#endif // ! GODOT_IK_EDITOR_PLUGIN
