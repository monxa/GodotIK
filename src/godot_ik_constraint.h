#ifndef GODOT_IK_CONSTRAINT
#define GODOT_IK_CONSTRAINT

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>

namespace godot {
class GodotIKConstraint : public Node {
	GDCLASS(GodotIKConstraint, Node);

public:
	enum Dir {
		BACKWARD = -1,
		FORWARD = 1
	};
	virtual PackedVector3Array apply(Vector3 p_parent_bone_pos, Vector3 p_bone_pos, Vector3 p_child_bone_pos, int direction);
	int get_bone_idx();
	void set_bone_idx(int p_bone_id);

protected:
	static void _bind_methods();
	GDVIRTUAL4RC(PackedVector3Array, apply, Vector3, Vector3, Vector3, int);
	bool apply_method_implemented = false;

private:
	int bone_idx;
}; // ! class GodotIKConstraint
} //namespace godot

VARIANT_ENUM_CAST(GodotIKConstraint::Dir);

#endif // ! GODOT_IK_CONSTRAINT