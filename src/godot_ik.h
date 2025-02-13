#ifndef GODOT_IK_H
#define GODOT_IK_H

#include "godot_ik_constraint.h"
#include "godot_ik_effector.h"

#include <cstdint>
#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

namespace godot {

class IKChain;

class GDE_EXPORT GodotIK : public SkeletonModifier3D {
	GDCLASS(GodotIK, SkeletonModifier3D)

private:
	struct IKChain {
		// From parent to child
		PackedInt32Array bones;
		GodotIKEffector *effector;
		Vector3 effector_position;
		Vector<GodotIKConstraint *> constraints;
	};

public:
	void _notification(int p_notification);

	void _process_modification() override;
	void set_position_group(int p_idx_bone, const Vector3 &p_pos_bone);

	PackedStringArray _get_configuration_warnings() const override;

	// public setters/getters
	void set_iteration_count(int p_iteration_count);
	int get_iteration_count() const;

	PackedVector3Array get_positions() {
		PackedVector3Array result;
		for (Vector3 p : positions) {
			result.push_back(p);
		}
		return result;
	}
	float get_time_iteration() { return time_iteration; }

	void set_use_global_rotation_poles(bool p_use_global_rotation_poles);
	bool get_use_global_rotation_poles();

protected:
	static void _bind_methods();

private:
	int iteration_count = 8;

	bool dirty = true;
	// Bone length relative to parent. Root has no bone length.
	Vector<float> bone_lengths;
	HashMap<int, Vector<int>> grouped_by_position;
	Vector<int> bone_depths;
	Vector<int> indices_by_depth;

	Vector<bool> needs_processing;
	Vector<Vector3> positions;
	Vector<Transform3D> initial_transforms;
	// identity_idx is used as an extra element (at index bone_count) to represent a "null" or identity transform.
	// This extra element is initialized as follows:
	//   needs_processing[identity_idx] = false
	//   positions[identity_idx] = Vector3(0, 0, 0)   // or Vector3.ZERO if defined
	//   initial_transforms[identity_idx] = Transform3D()  // or Transform3D.IDENTITY if defined
	int identity_idx;

	// this is mainly for compatibility with version 1.0.0:
	bool use_global_rotation_poles = false;

	float time_iteration = 0.;
	Vector<IKChain> chains;
	Vector<Vector<int>> chain_forward_processing_order;

	void initialize_if_dirty();
	void initialize_groups();
	void initialize_bone_lengths();
	void initialize_chains();
	void set_effector_transforms_to_bones();

	void initialize_deinitialize_connections();
	void make_dirty();
	Callable callable_deinitialize;

	void solve_forward();
	void solve_backward();

	void apply_positions();

	_ALWAYS_INLINE_ void apply_constraint(const IKChain &p_chain, int p_idx, GodotIKConstraint::Dir p_direction);

	Vector<int> calculate_bone_depths(Skeleton3D *p_skeleton);
	bool compare_by_depth(int p_a, int p_b, const Vector<int> &p_depths);

	Vector<Node *> get_nested_children_dsf(Node *base);

}; // ! class GodotIK

} // namespace godot

#endif // ! GODOT_IK_H