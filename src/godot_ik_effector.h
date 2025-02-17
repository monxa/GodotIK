#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/wrapped.hpp"

#ifndef GODOT_IK_EFFECTOR_H
#define GODOT_IK_EFFECTOR_H

namespace godot {
class Skeleton3D;
class GodotIK;
class GodotIKPole;

class GodotIKEffector : public Node3D {
	GDCLASS(GodotIKEffector, Node3D)
public:
	enum TransformMode {
		POSITION_ONLY, // Only the position is applied.
		PRESERVE_ROTATION, // Preserve local rotation of the leaf.
		STRAIGHTEN_CHAIN, // Straighten the IK chain at the effector.
		FULL_TRANSFORM // Apply this node3ds transform.
	};

	enum TwistMode {
		DYNAMIC,
		PRESERVE_TWIST
	};

	int get_bone_idx() const;
	void set_bone_idx(int p_bone_idx);

	int get_chain_length() const;
	void set_chain_length(int p_chain_length);

	void set_transform_mode(TransformMode p_transform_mode);
	TransformMode get_transform_mode() const;
	
	void set_twist_mode(TwistMode p_twist_mode);
	TwistMode get_twist_mode() const;

	void set_ik_controller(GodotIK *p_ik_controller);
	GodotIK *get_ik_controller() const;

	Skeleton3D *get_skeleton() const;
	
	GodotIKPole * get_pole();
	void set_pole(GodotIKPole * p_pole);
	bool has_one_pole = false;



	PackedStringArray _get_configuration_warnings() const override;

protected:
	static void _bind_methods();

private:
	int bone_idx = 0;
	int chain_length = 2;
	TransformMode transform_mode = TransformMode::POSITION_ONLY;
	TwistMode twist_mode = TwistMode::DYNAMIC;
	GodotIK *ik_controller = nullptr;
	GodotIKPole * pole;

}; // ! class GodotIKEffector
} //namespace godot

VARIANT_ENUM_CAST(GodotIKEffector::TransformMode);
VARIANT_ENUM_CAST(GodotIKEffector::TwistMode);

#endif // ! GODOT_IK_EFFECTOR_H