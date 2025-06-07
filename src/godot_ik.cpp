#include "godot_ik.h"

#include <godot_ik_effector.h>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform2d.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// ----- Godot (Node) bindings -------

static int total_instance_count = 0;

void GodotIK::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_iteration_count", "iteration_count"), &GodotIK::set_iteration_count);
	ClassDB::bind_method(D_METHOD("get_iteration_count"), &GodotIK::get_iteration_count);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "iteration_count", PROPERTY_HINT_RANGE, "1, 32, 1"),
			"set_iteration_count",
			"get_iteration_count");
	ClassDB::bind_method(D_METHOD("get_bone_position", "bone_idx"), &GodotIK::get_bone_position);

	ClassDB::bind_method(D_METHOD("set_effector_transforms_to_bones"), &GodotIK::set_effector_transforms_to_bones);

	ClassDB::bind_method(D_METHOD("set_use_global_rotation_poles", "global_rotation_poles"), &GodotIK::set_use_global_rotation_poles);
	ClassDB::bind_method(D_METHOD("get_global_rotation_poles"), &GodotIK::get_use_global_rotation_poles);
	ADD_PROPERTY(PropertyInfo(Variant::Type::BOOL, "use_global_rotation_poles"),
			"set_use_global_rotation_poles",
			"get_global_rotation_poles");

	ClassDB::bind_method(D_METHOD("get_effectors"), &GodotIK::get_effectors);

	ClassDB::bind_method(D_METHOD("get_current_iteration"), &GodotIK::get_current_iteration);
}

void GodotIK::_notification(int p_notification) {
	if (p_notification == NOTIFICATION_READY) {
		Callable callable_initialize = callable_mp(this, &GodotIK::initialize_if_dirty);
		callable_deinitialize = callable_mp(this, &GodotIK::make_dirty);
		connect("child_order_changed", callable_initialize);

		performance_monitor_name = "IK/" + get_parent()->get_name() + " (" + Variant(total_instance_count).stringify() + ")";

		total_instance_count += 1;
		if (!Performance::get_singleton()->has_custom_monitor(performance_monitor_name)) {
			Performance::get_singleton()->add_custom_monitor(performance_monitor_name, callable_mp(this, &GodotIK::get_time_iteration));
		}
		if (get_skeleton()) {
			initialize_effectors();
		}
	}
	if (p_notification == NOTIFICATION_EXIT_TREE) {
		for (GodotIKRoot *root : external_roots) {
			root->set_ik_controller(NodePath());
		}
		external_roots.clear();
		if (Performance::get_singleton()->has_custom_monitor(performance_monitor_name)) {
			Performance::get_singleton()->remove_custom_monitor(performance_monitor_name);
		}
	}
}

PackedStringArray GodotIK::_get_configuration_warnings() const {
	PackedStringArray result;
	if (effectors.size() == 0) {
		result.push_back("At least one child effector is required.");
	}
	return result;
}

// ! Godot (Node) bindings

// ------ Update --------

void GodotIK::_process_modification() {
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton == nullptr) {
		return;
	}
	if (!is_node_ready()) {
		return;
	}
	float t1 = Time::get_singleton()->get_ticks_usec();

	// Initialize positions with base
	if (dirty) {
		initialize_if_dirty();
	}

	update_all_transforms_from_skeleton();

	for (IKChain &chain : chains) {
		if (!chain.effector->is_inside_tree()) {
			// ! RETURN. If any effector is outside the tree, we are not ready yet, to process any modifications.
			return;
		}
		chain.effector_position = skeleton->to_local(chain.effector->get_global_position());

		// see https://github.com/monxa/GodotIK/issues/52
		float influence = chain.effector->get_influence();

		int bone_idx = chain.effector->get_bone_idx();
		if (chain.effector->is_active() && influence < 1. && bone_idx >= 0 && bone_idx < initial_transforms.size()) {
			Vector3 global_pose_pos = initial_transforms[bone_idx].origin;
			chain.effector_position = global_pose_pos.lerp(chain.effector_position, influence);
		}
	}

	for (current_iteration = 0; current_iteration < iteration_count; current_iteration++) {
		propagate_positions_from_chain_ancestors();
		solve_backward();
		solve_forward();
	}
	apply_positions();

	current_iteration = -1;

	float t2 = Time::get_singleton()->get_ticks_usec();
	time_iteration = (t2 - t1);
}

void GodotIK::set_position_group(int p_idx_bone_in_group, const Vector3 &p_pos_bone) {
	ERR_FAIL_COND_MSG(!grouped_by_position.has(p_idx_bone_in_group), "[GodotIK] grouped_by_position should have every index.");
	Vector<int> group = grouped_by_position.get(p_idx_bone_in_group);
	ERR_FAIL_COND_MSG(group.size() == 0, "[GodotIK] grouped_by_position should be reflexive.");
	for (int i = 0; i < group.size(); i++) {
		positions.write[group[i]] = p_pos_bone;
	}
}

// Apply ancestor's transform offset to all intermediate bones (from root up to—but not including—ancestor)
void GodotIK::propagate_positions_from_chain_ancestors() {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}
	for (const IKChain &chain : chains) {
		if (chain.closest_parent_in_chain == -1) {
			continue;
		}

		ERR_FAIL_INDEX(0, chain.bones.size());
		int root_idx = chain.bones[chain.bones.size() - 1];
		int ancestor_idx = root_idx; // include root_idx in updates

		Vector<int> list_to_closest_parent; // TODO: Cache root to ancestor list for performance
		while (ancestor_idx != chain.closest_parent_in_chain) {
			list_to_closest_parent.push_back(ancestor_idx);
			ancestor_idx = skeleton->get_bone_parent(ancestor_idx);
			ERR_FAIL_COND_MSG(ancestor_idx == -1, "[GodotIK] Invalid hierarchy: closest_parent_in_chain not actually reachable from root_idx.");
		}

		ERR_FAIL_INDEX_MSG(0, list_to_closest_parent.size(), String("[GodotIK] Panic: Dependency propagation from invalid state. Please open an issue!") + (list_to_closest_parent.size()));

		int idx_ancestor_bone = chain.closest_parent_in_chain;
		int child_idx_of_ancestor_bone = chain.pivot_child_in_ancestor;

		ERR_FAIL_INDEX(chain.closest_parent_in_chain, initial_transforms.size());
		Transform3D rest_ancestor = initial_transforms[chain.closest_parent_in_chain];
		
		ERR_FAIL_INDEX_EDMSG(child_idx_of_ancestor_bone, initial_transforms.size(), chain.effector->get_name());
		Transform3D rest_ancestor_child = initial_transforms[child_idx_of_ancestor_bone];

		Vector3 rest_dir = rest_ancestor.origin.direction_to(rest_ancestor_child.origin);
		Vector3 cur_dir = positions[idx_ancestor_bone].direction_to(positions[child_idx_of_ancestor_bone]);

		ERR_FAIL_COND_MSG(rest_dir.length() == 0 || cur_dir.length() == 0, "[GodotIK] Can't propergate transforms with zero length base bone. Please open an issue.");

		// This simulates the actual end transform and propagates it through the chain.
		// Same as in final apply_positions post-pass. Note: Might not work if deprecated use_global_poles = true.
		Quaternion adjustment = Quaternion(rest_dir, cur_dir);
		Basis new_rotation = adjustment * rest_ancestor.basis;
		Transform3D new_transform = Transform3D(new_rotation, positions[idx_ancestor_bone]);
		Transform3D delta_transform = new_transform * rest_ancestor.inverse();

		Vector<int> list_from_ancestor = list_to_closest_parent;
		list_from_ancestor.reverse();
		for (int index : list_to_closest_parent) {
			Transform3D rest_transform = initial_transforms[index];
			Transform3D adjusted_transform = delta_transform * rest_transform;
			delta_transform = adjusted_transform * rest_transform.inverse();
			positions.write[index] = (adjusted_transform).origin;
		}
	}
}

void GodotIK::solve_backward() {
	// Chains are sorted by shallowest bone; iterate in reverse for backward pass
	for (int idx_chain = chains.size() - 1; idx_chain >= 0; idx_chain--) {
		IKChain &chain = chains.write[idx_chain];
		if (chain.bones.size() == 0 || chain.effector->get_influence() == 0.) {
			continue;
		}
		if (!chain.effector->is_active()) {
			continue;
		}
		const int leaf_idx = 0;
		set_position_group(chain.bones[leaf_idx], chain.effector_position);

		if (chain.constraints[leaf_idx]) {
			apply_constraint(chain, leaf_idx, GodotIKConstraint::Dir::BACKWARD);
		}
		for (int i = 1; i < chain.bones.size() - 1; ++i) {
			int idx_child = chain.bones[i - 1];
			int idx_bone = chain.bones[i];
			float length = bone_lengths[idx_child];
			Vector3 pos_child = positions[idx_child];
			Vector3 pos_bone = positions[idx_bone];

			pos_bone = pos_child + pos_child.direction_to(pos_bone) * length;
			set_position_group(idx_bone, pos_bone);
			if (chain.constraints[i]) {
				apply_constraint(chain, i, GodotIKConstraint::Dir::BACKWARD);
			}
		}
	}
}

void GodotIK::solve_forward() {
	// Forward pass uses sorted order (shallowest chains first)
	for (int idx_chain = 0; idx_chain < chains.size(); idx_chain++) {
		IKChain &chain = chains.write[idx_chain];
		if (!chain.effector->is_active() || chain.effector->get_influence() == 0.) {
			continue;
		}
		int root_idx = chain.bones.size() - 1;
		if (root_idx >= 0 && chain.constraints[root_idx]) {
			apply_constraint(chain, chain.bones.size() - 1, GodotIKConstraint::Dir::FORWARD);
		}
		for (int i = chain.bones.size() - 2; i >= 0; --i) {
			int idx_parent = chain.bones[i + 1];
			int idx_bone = chain.bones[i];
			float length = bone_lengths[idx_bone];
			Vector3 pos_parent = positions[idx_parent];
			Vector3 pos_bone = positions[idx_bone];
			pos_bone = pos_parent + pos_parent.direction_to(pos_bone) * length;
			set_position_group(idx_bone, pos_bone);
			if (chain.constraints[i]) {
				apply_constraint(chain, i, GodotIKConstraint::Dir::FORWARD);
			}
		}
	}
}

void GodotIK::apply_positions() {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}
	Vector<Transform3D> transforms = initial_transforms.duplicate();

	// -------- Apply Positions -----------
	for (int bone_idx = 0; bone_idx < initial_transforms.size(); bone_idx++) {
		transforms.write[bone_idx].origin = positions[bone_idx];
	}

	// ------- Apply Rotations ------------
	Vector<bool> rotation_processed;
	rotation_processed.resize(transforms.size());
	rotation_processed.fill(false);
	const Transform3D identity_transform; // Default identity transform

	for (int bone_idx : indices_by_depth) {
		// Get parent's index; if none, use identity_idx.
		int parent_idx = skeleton->get_bone_parent(bone_idx);
		parent_idx = (parent_idx < 0) ? identity_idx : parent_idx;

		// If neither this bone nor its parent needs processing, skip it.
		if (!needs_processing[bone_idx] && !needs_processing[parent_idx]) {
			continue;
		}

		// Only apply rotation correction once to each parent_idx. All children will share the same position.
		// We greedily process the first child->parent relation only.
		if (rotation_processed[parent_idx]) {
			continue;
		}
		rotation_processed.write[parent_idx] = true;

		// Parent idx can't escape processing now ------------------

		// Determine the grandparent index.
		int grandparent_idx = identity_idx;
		if (parent_idx != identity_idx) {
			grandparent_idx = skeleton->get_bone_parent(parent_idx);
			if (grandparent_idx < 0) {
				grandparent_idx = identity_idx;
			}
		}

		// Cache the necessary transforms.
		const Transform3D &gp_transform = transforms[grandparent_idx];
		const Transform3D &gp_init_transform = initial_transforms[grandparent_idx];

		// Retrieve positions from initial and current transforms.
		const Vector3 &old_bone_pos = initial_transforms[bone_idx].origin;
		const Vector3 &old_parent_pos = initial_transforms[parent_idx].origin;
		const Vector3 &new_bone_pos = transforms[bone_idx].origin;
		const Vector3 &new_parent_pos = transforms[parent_idx].origin;

		// Compute the direction vectors (from parent to bone).
		Vector3 old_direction = old_parent_pos.direction_to(old_bone_pos);
		Vector3 new_direction = new_parent_pos.direction_to(new_bone_pos);

		// Compute the additional rotation for adjustment.
		Quaternion additional_rotation;

		if (use_global_rotation_poles) { // Old approach
			additional_rotation = Quaternion(old_direction, new_direction);
			// Handle singularity: Anti parallel vectors.
			float dot = old_direction.dot(new_direction);
			if (fabs(dot + 1.0f) < CMP_EPSILON) { // Old approach requires pole fix
				Vector3 chosen_axis;

				// Try to use parent's information if available.
				if (grandparent_idx != -1) {
					// Use grandparent's position to influence the twist axis.
					Vector3 parent_dir = positions[grandparent_idx].direction_to(positions[parent_idx]);
					chosen_axis = parent_dir.cross(old_direction);
				}

				// If parent's data didn't yield a valid axis, fall back to a default arbitrary axis.
				if (chosen_axis.length_squared() < CMP_EPSILON) {
					chosen_axis = old_direction.cross(Vector3(1, 0, 0));
					if (chosen_axis.length_squared() < CMP_EPSILON) {
						chosen_axis = old_direction.cross(Vector3(0, 0, 1));
					}
				}

				chosen_axis = chosen_axis.normalized();
				additional_rotation = Quaternion(chosen_axis, Math_PI); // 180 deg rotation.
			}
		} else { // New approach: use_global_rotation_poles = false
			// Transform directions into the grandparent's local space.
			old_direction = gp_init_transform.basis.xform_inv(old_direction);
			new_direction = gp_transform.basis.xform_inv(new_direction);
			additional_rotation = Quaternion(old_direction, new_direction);

			// Bring the rotation back into global space.
			additional_rotation = gp_transform.basis * additional_rotation * gp_init_transform.basis.inverse();
		}

		// Update the parent's transform with the computed rotation.
		Transform3D parent_transform = transforms[parent_idx];
		parent_transform.basis = additional_rotation * parent_transform.basis;
		transforms.write[parent_idx] = parent_transform;

		// Ensure the identity index is kept at identity.
		transforms.write[identity_idx] = identity_transform;
	}

	// ------ Apply previously calculated global rotations in local space -------

	for (int bone_idx : indices_by_depth) {
		if (!needs_processing[bone_idx]) {
			continue;
		}
		Transform3D local_transform = transforms[bone_idx];
		int parent_idx = skeleton->get_bone_parent(bone_idx);
		if (parent_idx >= 0) {
			local_transform = transforms[parent_idx].affine_inverse() * local_transform;
		}
		skeleton->set_bone_pose(bone_idx, local_transform);
	}

	// ----- Effector leaf behavior ---------
	// TODO: Refactor this into the solving step. This needs to be done very early so that non-leaf effectors influence others correctly.
	for (const IKChain &chain : chains) {
		GodotIKEffector *effector = chain.effector;
		if (effector == nullptr) {
			continue;
		}
		if (effector->get_bone_idx() < 0 || effector->get_bone_idx() >= skeleton->get_bone_count()) {
			continue;
		}

		GodotIKEffector::TransformMode transform_mode = effector->get_transform_mode();
		if (transform_mode == GodotIKEffector::TransformMode::POSITION_ONLY) {
			continue;
		}
		int bone_idx = effector->get_bone_idx();
		int parent_idx = skeleton->get_bone_parent(effector->get_bone_idx());
		Transform3D trans_bone = transforms[bone_idx];
		Transform3D trans_skeleton = skeleton->get_global_transform();
		Transform3D trans_effector = effector->get_global_transform();
		Transform3D trans_parent;
		if (parent_idx != -1) {
			trans_parent = transforms[parent_idx];
		}
		switch (transform_mode) {
			case GodotIKEffector::TransformMode::FULL_TRANSFORM: {
				trans_bone.basis = (trans_skeleton.affine_inverse() * trans_effector).basis;
				Transform3D local_trans_bone = trans_parent.affine_inverse() * trans_bone;

				skeleton->set_bone_pose(bone_idx, local_trans_bone);
				break;
			}
			case GodotIKEffector::TransformMode::STRAIGHTEN_CHAIN: {
				if (parent_idx == -1) {
					continue;
				}
				Transform3D local_trans_bone = trans_parent.affine_inverse() * trans_bone;
				Vector3 prev_basis_scale = local_trans_bone.basis.get_scale();
				local_trans_bone.basis = Basis().scaled(prev_basis_scale);

				skeleton->set_bone_pose(bone_idx, local_trans_bone);
				break;
			}
			case GodotIKEffector::TransformMode::PRESERVE_ROTATION: {
				Transform3D init_transform_parent;
				if (parent_idx != -1) {
					init_transform_parent = initial_transforms[parent_idx];
				}
				Transform3D init_trans_bone = initial_transforms[bone_idx];
				Transform3D init_local_trans_bone = init_transform_parent.affine_inverse() * init_trans_bone;

				Transform3D local_trans_bone = trans_parent.affine_inverse() * trans_bone;
				local_trans_bone.basis = init_local_trans_bone.basis;
				skeleton->set_bone_pose(bone_idx, local_trans_bone);
				break;
			}
			default: {
				continue;
			}
		}
	}
}

void GodotIK::apply_constraint(const IKChain &p_chain, int p_idx_in_chain, GodotIKConstraint::Dir p_dir) {
	if (p_idx_in_chain >= p_chain.bones.size()) {
		return;
	}
	GodotIKConstraint *constraint = p_chain.constraints[p_idx_in_chain];
	if (constraint == nullptr) {
		return;
	}

	float influence = compute_constraint_step_influence(p_chain.effector->get_influence(), iteration_count);

	int idx_bone = p_chain.bones[p_idx_in_chain];
	int idx_child = -1;
	int idx_parent = -1;

	Vector3 pos_parent;
	Vector3 pos_bone = positions[idx_bone];
	Vector3 pos_child;

	if (p_idx_in_chain < p_chain.bones.size() - 1) {
		idx_parent = p_chain.bones[p_idx_in_chain + 1];
		pos_parent = positions[idx_parent];
	} else { // no parent. We are the root. Check skeleton for parent then.
		int idx_parent_proxy = get_skeleton()->get_bone_parent(idx_bone); // proxy, so we don't write outside the chain.
		if (idx_parent_proxy != -1) {
			pos_parent = positions[idx_parent_proxy];
		}
	}

	if (p_idx_in_chain > 0) {
		idx_child = p_chain.bones[p_idx_in_chain - 1];
		pos_child = positions[idx_child];
	}

	PackedVector3Array result = constraint->apply(pos_parent, pos_bone, pos_child, p_dir);

	if (influence < 1. && result.size() == 3) {
		PackedVector3Array base = { pos_parent, pos_bone, pos_child };
		for (int i = 0; i < 3; i++) {
			result[i] = base[i].lerp(result[i], influence); // TODO: Suboptimal since dependent on iteration count.
		}
	}

	if (idx_parent != -1 && pos_parent != result[0]) {
		set_position_group(idx_parent, result[0]);
	}
	if (pos_bone != result[1]) {
		set_position_group(idx_bone, result[1]);
	}
	if (idx_child != -1 && pos_child != result[2]) {
		set_position_group(idx_child, result[2]);
	}
}

// ! Update

// ------ Initialization ------------- /

void GodotIK::initialize_if_dirty() {
	dirty = false;
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}
	// Connect to bone_list_changed
	Callable callable_initialize = callable_mp(this, &GodotIK::initialize_if_dirty);
	if (!get_skeleton()->is_connected("bone_list_changed", callable_initialize)) {
		get_skeleton()->connect("bone_list_changed", callable_initialize);
	}

	initialize_groups();

	initialize_bone_lengths();
	initialize_effectors();
	initialize_chains();

	{ // Indices by depth creation.
		HashSet<int> in_chain;
		for (const IKChain &chain : chains) {
			for (int idx : chain.bones) {
				in_chain.insert(idx);
			}
		}
		Vector<int> bone_depths = calculate_bone_depths(skeleton);
		Vector<int> indices;
		indices.resize(bone_depths.size());
		for (int i = 0; i < bone_depths.size(); i++) {
			indices.write[i] = i;
		}

		struct DepthComparator {
			const Vector<int> &comp_depths;
			const HashSet<int> &comp_in_chain;
			DepthComparator(const Vector<int> &p_depths, const HashSet<int> &p_in_chain) :
					comp_depths(p_depths), comp_in_chain(p_in_chain) {
			}

			// Respect depth and make bones that are in chains shallower. Shallower -> Processed first.
			bool operator()(int a, int b) const {
				return comp_depths[a] < comp_depths[b] || (comp_depths[a] == comp_depths[b] && comp_in_chain.has(a) && !comp_in_chain.has(b));
			}
		};

		indices.sort_custom<DepthComparator>(DepthComparator(bone_depths, in_chain));
		indices_by_depth = indices;
	}

	{ // Sort chains by depth of shallowest bone.
		struct DepthComparator {
			bool operator()(const IKChain &a, const IKChain &b) const {
				if (a.bones.size() == 0) {
					return true;
				}
				if (b.bones.size() == 0) {
					return false;
				}
				return a.bones[a.bones.size() - 1] < b.bones[b.bones.size() - 1];
			};
		};
		chains.sort_custom<DepthComparator>(DepthComparator());
	}

	// Recalculate a mapping on the sorted chains
	HashMap<int, Pair<int, int>> bone_to_chain_map; // bone_idx -> (chain_index, index_in_chain)
	for (int i = 0; i < chains.size(); i++) {
		bone_to_chain_map.reserve(skeleton->get_bone_count());
		for (int idx_chain = 0; idx_chain < chains.size(); idx_chain++) {
			const IKChain &chain = chains[idx_chain];
			if (chain.effector->get_bone_idx() == 0 || chain.bones.size() == 0){
				continue;
			}
			for (int idx_in_chain = 0; idx_in_chain < chain.bones.size(); ++idx_in_chain) {
				int bone_idx = chain.bones[idx_in_chain];
				if (bone_to_chain_map.has(bone_idx)) {
					continue;
				}
				bone_to_chain_map.insert(bone_idx, Pair(idx_chain, idx_in_chain));
			}
		}
	}

	// assign closest_parent in chain
	for (IKChain &chain : chains) {
		if (chain.bones.size() == 0) {
			continue;
		}
		int root_idx = chain.bones[chain.bones.size() - 1];
		int ancestor_idx = skeleton->get_bone_parent(root_idx);
		while (ancestor_idx != -1 && !bone_to_chain_map.has(ancestor_idx)) {
			ancestor_idx = skeleton->get_bone_parent(ancestor_idx);
		}
		chain.closest_parent_in_chain = ancestor_idx;
		if (ancestor_idx != -1) {
			Pair placemnt_pair = bone_to_chain_map.get(ancestor_idx);
			if (placemnt_pair.second <= 0){
				chain.closest_parent_in_chain = -1;
				WARN_PRINT("[GodotIK] Effectors inbetween chains not supported. Please open an issue.");
				continue;
			}
			chain.pivot_child_in_ancestor = chains[placemnt_pair.first].bones[placemnt_pair.second - 1];
		}
	}

	// fill needs_processing
	// + 1 for identity_idx
	needs_processing.resize(skeleton->get_bone_count() + 1);
	needs_processing.fill(false);
	for (auto chain : chains) {
		for (int bone_idx : chain.bones) {
			if (!grouped_by_position.has(bone_idx)) {
				print_error("NO Group found for ", bone_idx);
				continue;
			}
			for (int idx_in_group : grouped_by_position.get(bone_idx)) {
				needs_processing.write[idx_in_group] = true;
			}
		}
	}
	
	initialize_connections(this);
	for (GodotIKRoot *ext : external_roots) {
		initialize_connections(ext);
	}
	identity_idx = skeleton->get_bone_count();

	bone_effector_map.resize(identity_idx + 1);
	bone_effector_map.fill(Vector<GodotIKEffector *>());
	for (const IKChain &chain : chains) {
		for (const int bone_idx : chain.bones) {
			bone_effector_map.write[bone_idx].push_back(chain.effector);
		}
	}
	dirty = false;
}

void GodotIK::initialize_groups() {
	Skeleton3D *skeleton = get_skeleton();
	grouped_by_position = HashMap<int, Vector<int32_t>>();

	if (skeleton == nullptr) {
		return;
	}

	// for each bone
	for (int i = 0; i < skeleton->get_bone_count(); i++) {
		HashMap<Vector3, Vector<int32_t>> pos_groups;
		PackedInt32Array children = skeleton->get_bone_children(i);

		// Construct <snapped(position), child-indices>
		for (int child_index : children) {
			Vector3 pos_j = skeleton->get_bone_pose_position(child_index);
			pos_j.snapf(CMP_EPSILON);

			if (!pos_groups.has(pos_j)) {
				pos_groups[pos_j] = Vector<int32_t>();
			}
			pos_groups[pos_j].push_back(child_index);
		}

		// Construct <index, groups>
		for (KeyValue<Vector3, Vector<int32_t>> kv : pos_groups) {
			for (int k : kv.value) {
				grouped_by_position[k] = kv.value;
			}
		}
	}

	for (int bone : skeleton->get_parentless_bones()) {
		grouped_by_position[bone] = { bone };
	}
}

void GodotIK::initialize_bone_lengths() {
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton == nullptr) {
		return;
	}
	bone_lengths = Vector<float>();
	bone_lengths.resize(skeleton->get_bone_count());
	for (int i = 0; i < skeleton->get_bone_count(); i++) {
		bone_lengths.write[i] = skeleton->get_bone_pose_position(i).length();
	}
}

void GodotIK::initialize_effectors() {
	// Collect all nested effectors
	Vector<Node *> child_list = get_nested_children_dsf(this);
	Vector<GodotIKEffector *> new_effectors;
	// TODO: We could do some deinitialization here

	for (GodotIKRoot *external_root : external_roots) {
		Vector<Node *> external_child_list = get_nested_children_dsf(external_root);
		for (Node *child : external_child_list) {
			child_list.push_back(child);
		}
	}
	for (Node *child : child_list) {
		GodotIKEffector *effector = Object::cast_to<GodotIKEffector>(child);
		if (effector) {
			new_effectors.push_back(effector);
		}
	}
	// clean up old effectors
	for (GodotIKEffector *effector : effectors) {
		set_effector_properties(effector, nullptr);
	}

	for (GodotIKEffector *effector : new_effectors) {
		set_effector_properties(effector, this);
	}

	effectors = new_effectors;
}

void GodotIK::set_effector_properties(GodotIKEffector *effector, GodotIK *ik_controller) {
	effector->set_ik_controller(this);
	for (int i = 0; i < effector->get_child_count(); i++) {
		GodotIKConstraint *constraint = Object::cast_to<GodotIKConstraint>(effector->get_child(i));
		if (constraint) {
			constraint->set_ik_controller(this);
		}
	}
	update_configuration_warnings();
}

void GodotIK::initialize_chains() {
	// Ensure chains are clear for initialization
	chains.clear();

	// Collect skeleton and ensure it's valid
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		ERR_PRINT("Skeleton not found for IK initialization.");
		return;
	}

	// Process each child if child is effector
	for (GodotIKEffector *effector : effectors) {
		IKChain new_chain;
		new_chain.effector = effector;
		if (skeleton->get_bone_parent(effector->get_bone_idx()) == -1){
			continue; // Don't build chains from root up.
		}
		int bone_idx = effector->get_bone_idx();

		for (int chain_step = 0; chain_step < effector->get_chain_length() && bone_idx >= 0; chain_step++) {
			new_chain.bones.push_back(bone_idx);
			bone_idx = skeleton->get_bone_parent(bone_idx);
		}

		// add constraints to current effector
		new_chain.constraints.resize(new_chain.bones.size());
		new_chain.constraints.fill(nullptr);
		int effector_pole_count = 0;
		for (int i = 0; i < effector->get_child_count(); i++) {
			Node *child = effector->get_child(i);
			if (child->is_class("GodotIKConstraint")) {
				GodotIKConstraint *constraint = Object::cast_to<GodotIKConstraint>(child);
				int placement_in_chain = -1;
				for (int idx_chain = 0; idx_chain < new_chain.bones.size(); idx_chain++) {
					if (new_chain.bones[idx_chain] == constraint->get_bone_idx()) {
						placement_in_chain = idx_chain;
						break;
					}
				}
				if (placement_in_chain == -1) {
					UtilityFunctions::push_error("Constraint: ", constraint->get_name(), "with bone_idx: ", bone_idx, " not in parent effectors chain.");
					continue;
				}
				new_chain.constraints.write[placement_in_chain] = constraint;
			}
		}
		chains.push_back(new_chain);
	}
}

void GodotIK::initialize_connections(Node *p_root) {
	Vector<Node *> child_list = get_nested_children_dsf(p_root); // First in, first out through iteration -> BSF
	if (!p_root->is_connected("child_order_changed", callable_deinitialize)) {
		p_root->connect("child_order_changed", callable_deinitialize);
	}
	for (Node *child : child_list) {
		if (!child->is_connected("child_order_changed", callable_deinitialize)) {
			child->connect("child_order_changed", callable_deinitialize);
		}
		if (child->is_class("GodotIKEffector")) {
			if (!child->is_connected("ik_property_changed", callable_deinitialize.unbind(1))) {
				child->connect("ik_property_changed", callable_deinitialize);
			}
		}
		if (child->is_class("GodotIKConstraint")) {
			if (!child->is_connected("bone_idx_changed", callable_deinitialize.unbind(1))) {
				child->connect("bone_idx_changed", callable_deinitialize.unbind(1));
			}
		}
	}
}

void GodotIK::update_all_transforms_from_skeleton() {
	Skeleton3D *skeleton = get_skeleton();
	ERR_FAIL_NULL_MSG(skeleton, "[GodotIK] No skeleton during _update_all_transforms_from_skeleton.");

	initial_transforms.resize(identity_idx + 1); // +1 for identity index

	for (int i = 0; i < skeleton->get_bone_count(); i++) {
		initial_transforms.set(i, skeleton->get_bone_global_pose(i));
	}
	initial_transforms.set(identity_idx, Transform3D()); // buffer an identity transform
	positions.resize(identity_idx + 1);
	for (int i = 0; i < skeleton->get_bone_count(); i++) {
		positions.set(i, initial_transforms[i].origin);
	}
	positions.set(identity_idx, Vector3());
}

void GodotIK::make_dirty() {
	dirty = true;
}
// ! Initialization

// ------------ Helpers ----------------

Vector<int> GodotIK::calculate_bone_depths(Skeleton3D *p_skeleton) {
	ERR_FAIL_NULL_V_MSG(p_skeleton, Vector<int>(), "[GodotIK] No skeleton during calculate_bone_depths.");

	int bone_count = p_skeleton->get_bone_count();
	if (bone_count == 0) {
		return Vector<int>();
	}

	Vector<int> depths;
	depths.resize(bone_count);
	PackedInt32Array process_list = p_skeleton->get_parentless_bones();
	for (int root_idx : process_list) {
		depths.set(root_idx, 0); // Root depth is 0
	}
	for (int i = 0; i < process_list.size(); i++) {
		int idx = process_list[i];
		int depth = depths.get(idx);

		PackedInt32Array children = p_skeleton->get_bone_children(idx);
		for (int child_idx : children) {
			process_list.push_back(child_idx);
			depths.set(child_idx, depth + 1);
		}
	}
	return depths;
}

Vector<Node *> GodotIK::get_nested_children_dsf(Node *base) const {
	Vector<Node *> child_list; // First in, first out through iteration -> BSF

	for (int i = 0; i < base->get_child_count(); i++) {
		child_list.push_back(base->get_child(i));
	}
	for (int i = 0; i < child_list.size(); i++) {
		Node *child = child_list[i];
		for (int j = 0; j < child->get_child_count(); j++) {
			child_list.push_back(child->get_child(j));
		}
	}
	return child_list;
}

// Computes the per-iteration influence step to reach a total influence after N iterations
float GodotIK::compute_constraint_step_influence(float total_influence, int iteration_count) {
	if (total_influence == 0 || total_influence == 1. || iteration_count == 0) {
		return total_influence;
	}

	return 1.0f - powf(1.0f - total_influence, 1.0f / float(iteration_count));
}

// For editor tooling:
void GodotIK::set_effector_transforms_to_bones() {
	for (GodotIKEffector *effector : effectors) {
		effector->set_transform_to_bone();
	}
}

// !Helpers

/*-------- Setters & Getters ------------*/
void GodotIK::set_iteration_count(int p_iteration_count) {
	iteration_count = p_iteration_count;
}

int GodotIK::get_iteration_count() const {
	return iteration_count;
}

bool GodotIK::compare_by_depth(int p_a, int p_b, const Vector<int> &p_depths) {
	return p_depths[p_a] < p_depths[p_b];
}

void GodotIK::set_use_global_rotation_poles(bool p_use_global_rotation_poles) {
	use_global_rotation_poles = p_use_global_rotation_poles;
}

bool GodotIK::get_use_global_rotation_poles() const {
	return use_global_rotation_poles;
}

TypedArray<GodotIKEffector> GodotIK::get_effectors() {
	TypedArray<GodotIKEffector> result;
	result.resize(effectors.size());
	for (int i = 0; i < effectors.size(); i++) {
		result[i] = effectors[i];
	}
	return result;
}

int GodotIK::get_current_iteration() {
	return current_iteration;
}

void GodotIK::add_external_root(GodotIKRoot *p_root) {
	ERR_FAIL_COND_MSG(this->is_ancestor_of(p_root) || p_root->is_ancestor_of(this), "[GodotIK] can't be ancestor of its external root or vise versa.");
	external_roots.push_back(p_root);
	if (get_skeleton()) {
		initialize_if_dirty();
	}
}

void GodotIK::remove_external_root(GodotIKRoot *p_root) {
	external_roots.erase(p_root);
	if (!p_root) {
		return;
	}

	Vector<Node *> child_list = get_nested_children_dsf(p_root); // First in, first out through iteration -> BSF
	if (p_root->is_connected("child_order_changed", callable_deinitialize)) {
		p_root->disconnect("child_order_changed", callable_deinitialize);
	}
	for (Node *child : child_list) {
		if (child->is_connected("child_order_changed", callable_deinitialize)) {
			child->disconnect("child_order_changed", callable_deinitialize);
		}
		if (child->is_class("GodotIKEffector")) {
			if (child->is_connected("ik_property_changed", callable_deinitialize)) {
				child->disconnect("ik_property_changed", callable_deinitialize);
			}
		}
		if (child->is_class("GodotIKConstraint")) {
			if (child->is_connected("bone_idx_changed", callable_deinitialize.unbind(1))) {
				child->disconnect("bone_idx_changed", callable_deinitialize.unbind(1));
			}
		}
	}
	make_dirty();
}
