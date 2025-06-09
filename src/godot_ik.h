#ifndef GODOT_IK_H
#define GODOT_IK_H

#include "godot_cpp/variant/basis.hpp"
#include "godot_ik_constraint.h"
#include "godot_ik_effector.h"
#include "godot_ik_root.h"

#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/templates/local_vector.hpp>

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
		int32_t ancestor_chain_bone_idx = -1;
		int16_t ancestor_child_bone_index = -1;
		bool ancestor_is_effector = false;
		const IKChain * ancestor_chain = nullptr;

		inline int size() const {
			return bones.size();
		}
		inline bool is_empty() const {
			return size() == 0;
		}
	};
	struct BoneCache {
		Basis basis;
		int16_t iteration = -1;
		int16_t child_index;
		void invalidate(){
			iteration = -1;
		}
		bool is_dirty() const {
			return iteration == -1;
		}
	};

public:
	void _notification(int p_notification);

	void _process_modification() override;
	void set_position_group(int p_idx_bone, const Vector3 &p_pos_bone);

	PackedStringArray _get_configuration_warnings() const override;

	// public setters/getters
	void set_iteration_count(int p_iteration_count);
	int get_iteration_count() const;

	Vector3 get_bone_position(int p_bone_idx) const {
		ERR_FAIL_INDEX_V(p_bone_idx, positions.size(), Vector3());
		return positions[p_bone_idx];
	}

	float get_time_iteration() const { return time_iteration; }

	void set_use_global_rotation_poles(bool p_use_global_rotation_poles);
	bool get_use_global_rotation_poles() const;

	TypedArray<GodotIKEffector> get_effectors();

	int get_current_iteration();

	void add_external_root(GodotIKRoot *p_root);
	void remove_external_root(GodotIKRoot *p_root);

	void set_effector_transforms_to_bones();

protected:
	static void _bind_methods();

private:
	int iteration_count = 8;
	int current_iteration = -1;
	bool dirty = true;
	Vector<GodotIKRoot *> external_roots;
	// Bone length relative to parent. Root has no bone length.
	Vector<float> bone_lengths;
	HashMap<int, Vector<int>> grouped_by_position;
	
	// Breath (shallowest bone) first THEN in chain first.
	Vector<int> indices_by_process_order;

	Vector<bool> needs_processing;
	Vector<Transform3D> initial_transforms;
	Vector<Vector3> positions;
	Vector<int> primary_child_list;
	Vector<GodotIKEffector *> effectors;
	StringName performance_monitor_name;
	/** identity_idx is used as an extra element (at index bone_count) to represent a "null" or identity transform.
	*** This extra element is initialized as follows:
	*** needs_processing[identity_idx] = false
	*** positions[identity_idx] = Vector3(0, 0, 0)   // or Vector3.ZERO if defined
	*** initial_transforms[identity_idx] = Transform3D()  // or Transform3D.IDENTITY if defined */
	int identity_idx;

	// this is mainly for compatibility with version 1.0.0:
	bool use_global_rotation_poles = false;

	float time_iteration = 0.;
	Vector<IKChain> chains;
	TightLocalVector<BoneCache> bone_caches;
	TightLocalVector<bool> dirty_bones;
	Callable callable_deinitialize;

	// update ------
	void propagate_positions_from_chain_ancestors();
	void solve_forward();
	void solve_backward();
	void post_pass();

	// ! update --------

	// initialization -----------------------------

	// if dirty -----------
	void initialize_if_dirty();
	void initialize_groups();
	void initialize_bone_lengths();

	void initialize_effectors();
	void set_effector_properties(GodotIKEffector *effector, GodotIK *ik_controller);

	void initialize_chains();
	void update_all_transforms_from_skeleton();

	void initialize_connections(Node *p_root);
	// ! if dirty --------

	// ! initialization ------------------------------

	// helpers ------

	void make_dirty();

	_ALWAYS_INLINE_ void apply_constraint(const IKChain &p_chain, int p_idx, GodotIKConstraint::Dir p_direction);

	Vector<int> calculate_bone_depths(Skeleton3D *p_skeleton);
	bool compare_by_depth(int p_a, int p_b, const Vector<int> &p_depths);
	Vector<Node *> get_nested_nodes(Node *p_base) const;
	float compute_influence_in_step(float p_total_influence, int p_iteration_count);

	Basis get_effector_target_global_basis(
			const GodotIKEffector *p_effector,
			const Skeleton3D *p_skeleton,
			const Vector3 basis_scale,
			const Transform3D &p_parent_global_transform) const;
	void apply_effector_rotation(const GodotIKEffector *effector, Vector<Transform3D> &transforms, const Skeleton3D *skeleton);
	Basis get_working_global_basis(
			int p_bone_idx,
			int p_child_idx,
			const Skeleton3D *p_skeleton,
			const Vector<Vector3> &p_positions,
			const Vector<Transform3D> &p_initial_transforms
		);
	Basis get_working_global_basis_unsafe(
		int p_bone_idx,
		int p_child_idx,
		const Skeleton3D *p_skeleton,
		const Vector<Vector3> &p_positions,
		const Vector<Transform3D> &p_initial_transforms
	);
}; // ! class GodotIK

} // namespace godot

#endif // ! GODOT_IK_H
