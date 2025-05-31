#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/wrapped.hpp"

#ifndef GODOT_IK_EFFECTOR_H
#define GODOT_IK_EFFECTOR_H

namespace godot {
class Skeleton3D;
class GodotIK;

class GodotIKEffector : public Node3D {
	GDCLASS(GodotIKEffector, Node3D)
public:
	enum TransformMode {
		POSITION_ONLY, // Only the position is applied.
		PRESERVE_ROTATION, // Preserve local rotation of the leaf.
		STRAIGHTEN_CHAIN, // Straighten the IK chain at the effector.
		FULL_TRANSFORM // Apply this node3ds transform.
	};

	String get_bone_name() const;
	void set_bone_name(String p_name);

	int get_bone_idx() const;
	void set_bone_idx(int p_bone_idx);

	int get_chain_length() const;
	void set_chain_length(int p_chain_length);

	void set_transform_mode(TransformMode p_transform_mode);
	TransformMode get_transform_mode() const;

	void set_ik_controller(GodotIK *p_ik_controller);
	GodotIK *get_ik_controller() const;

	Skeleton3D *get_skeleton() const;

	void set_transform_to_bone();

	void set_active(bool p_active);
	bool is_active() const;

	void set_influence(float p_influence);
	float get_influence() const;

	PackedStringArray _get_configuration_warnings() const override;

protected:
	void _validate_property(PropertyInfo &p_property) const;

	static void _bind_methods();

private:
	String bone_name = "";
	int bone_idx = 0;
	int chain_length = 2;
	TransformMode transform_mode = TransformMode::POSITION_ONLY;
	GodotIK *ik_controller = nullptr;
	float influence = 1.;
	bool active = true;
}; // ! class GodotIKEffector
} //namespace godot

VARIANT_ENUM_CAST(GodotIKEffector::TransformMode);

#endif // ! GODOT_IK_EFFECTOR_H
