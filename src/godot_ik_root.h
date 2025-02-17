#ifndef GODOT_IK_ROOT_H
#define GODOT_IK_ROOT_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/node_path.hpp>

namespace godot {
class GodotIK;

class GodotIKRoot : public Node3D {
	GDCLASS(GodotIKRoot, Node3D);

public:
	void _notification(int p_notification);
	void set_ik_controller(const NodePath &ik_controller);
	NodePath get_ik_controller() const;

protected:
	static void _bind_methods();

private:
	GodotIK *ik_controller = nullptr;
	NodePath ik_controller_path;
};

} //namespace godot
#endif // ! GODOT_IK_ROOT_H