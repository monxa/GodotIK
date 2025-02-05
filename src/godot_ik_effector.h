#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/wrapped.hpp"

#ifndef GODOT_IK_EFFECTOR_H
#define GODOT_IK_EFFECTOR_H

class GodotIKEffector : public godot::Node3D {
	GDCLASS(GodotIKEffector, godot::Node3D)
public:
	enum TransformMode {
		POSITION_ONLY, // Only the position is applied.
		PRESERVE_ROTATION, // Preserve local rotation of the leaf.
		STRAIGHTEN_CHAIN, // Straighten the IK chain at the effector.
		FULL_TRANSFORM // Apply this node3ds transform.
	};

	int get_bone_idx();
	void set_bone_idx(int p_bone_id);

	int get_chain_length();
	void set_chain_length(int p_chain_length);

	TransformMode get_leaf_behavior();
	void set_leaf_behavior(TransformMode p_leaf_behavior);

protected:
	static void _bind_methods();

private:
	int bone_idx = 0;
	int chain_length = 2;
	TransformMode transform_mode = TransformMode::POSITION_ONLY;
}; // ! class GodotIKEffector

VARIANT_ENUM_CAST(GodotIKEffector::TransformMode);

#endif // ! GODOT_IK_EFFECTOR_H