#include "godot_ik.h"
#include "godot_ik_effector.h"

#include <cstdint>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void godot::GodotIK::_notification(int p_notification) {
	if (p_notification == NOTIFICATION_READY) {
		connect("child_order_changed", callable_mp(this, &GodotIK::initialize));
		StringName name = "IK/" + get_parent()->get_name();
		if (!Performance::get_singleton()->has_custom_monitor(name)) {
			Performance::get_singleton()->add_custom_monitor(name, callable_mp(this, &GodotIK::get_time_iteration));
		}
	}
}

PackedStringArray GodotIK::_get_configuration_warnings() const {
	int c = get_child_count();
	PackedStringArray result;
	if (c == 0) {
		result.push_back("At least one child effector is required.");
	}
	return result;
}

void GodotIK::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_iteration_count", "iteration_count"), &GodotIK::set_iteration_count);
	ClassDB::bind_method(D_METHOD("get_iteration_count"), &GodotIK::get_iteration_count);
	ADD_PROPERTY(PropertyInfo(Variant::Type::INT, "iteration_count", PROPERTY_HINT_RANGE, "1, 32, 1"),
			"set_iteration_count",
			"get_iteration_count");
	ClassDB::bind_method(D_METHOD("get_positions"), &GodotIK::get_positions);
}

// ------ Update --------

void GodotIK::_process_modification() {
	Skeleton3D *skeleton = get_skeleton();
	if (skeleton == nullptr) {
		return;
	}
	float t1 = Time::get_singleton()->get_ticks_usec();

	// Initialize positions with base
	initial_transforms.resize(skeleton->get_bone_count());
	positions.resize(skeleton->get_bone_count());
	for (int i = 0; i < skeleton->get_bone_count(); i++) {
		initial_transforms.set(i, skeleton->get_bone_global_pose(i));
	}
	for (int i = 0; i < skeleton->get_bone_count(); i++) {
		positions.set(i, initial_transforms[i].origin);
	}
	if (!initialized) {
		initialize();
	}

	// update effector positions
	for (int ch = 0; ch < chains.size(); ch++) {
		chains.write[ch].effector_position = skeleton->to_local(chains[ch].effector->get_global_position());
	}

	for (int i = 0; i < iteration_count; i++) {
		solve_backward();
		solve_forward();
	}
	apply_positions();

	float t2 = Time::get_singleton()->get_ticks_usec();
	time_iteration = (t2 - t1);
}

void GodotIK::set_position_group(int p_idx_bone_in_group, const Vector3 &p_pos_bone) {
	ERR_FAIL_COND_MSG(!grouped_by_position.has(p_idx_bone_in_group), "grouped_by_position should have every index.");
	Vector<int> group = grouped_by_position.get(p_idx_bone_in_group);
	ERR_FAIL_COND_MSG(group.size() == 0, "grouped_by_position should be reflexive.");
	for (int i = 0; i < group.size(); i++) {
		positions.write[group[i]] = p_pos_bone;
	}
}

void GodotIK::solve_backward() {
	for (IKChain &chain : chains) {
		if (chain.bones.size() == 0) {
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
	for (IKChain &chain : chains) {
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

void godot::GodotIK::apply_positions() {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}
	Vector<Transform3D> transforms = initial_transforms.duplicate();
	int skip = 0;
	for (int bone_idx : indices_by_depth) {
		int parent_idx = skeleton->get_bone_parent(bone_idx);
		if (parent_idx < 0) {
			if (positions[bone_idx] != transforms[bone_idx].origin) {
				print_error("IK Chain includes a root bone. This is not supported.");
				return;
			}
			continue;
		}
		if (!needs_processing[bone_idx] && !needs_processing[parent_idx]) {
			skip++;
			continue;
		}
		const Transform3D &parent_transform = transforms[parent_idx];
		const Transform3D &bone_transform = transforms[bone_idx];

		Vector3 old_position_bone = initial_transforms[bone_idx].origin;
		Vector3 old_position_parent = initial_transforms[parent_idx].origin;

		Vector3 new_position_bone = positions[bone_idx];
		Vector3 new_position_parent = positions[parent_idx];

		Vector3 old_direction = old_position_parent.direction_to(old_position_bone);
		Vector3 new_direction = new_position_parent.direction_to(new_position_bone);

		Quaternion additional_rotation = Quaternion(old_direction, new_direction);

		Transform3D new_parent_transform = transforms[parent_idx];
		Transform3D new_bone_transform = transforms[bone_idx];
		new_bone_transform.origin = new_position_bone;

		new_parent_transform.origin = positions[parent_idx];
		new_parent_transform.basis = additional_rotation * new_parent_transform.basis;
		transforms.write[parent_idx] = new_parent_transform;
		transforms.write[bone_idx] = new_bone_transform;
	}

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

		GodotIKEffector::TransformMode transform_mode = effector->get_leaf_behavior();
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
				Vector3 old_forward = -trans_bone.basis[2];
				Vector3 new_forward = trans_parent.origin.direction_to(trans_bone.origin);

				Quaternion rotation_adjustment = Quaternion(old_forward, new_forward);
				trans_bone.basis = rotation_adjustment * trans_bone.basis;
				Transform3D local_trans_bone = trans_parent.affine_inverse() * trans_bone;

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

void godot::GodotIK::apply_effector_transforms() {
}

void godot::GodotIK::apply_constraint(const IKChain &p_chain, int p_idx_in_chain, GodotIKConstraint::Dir p_dir) {
	if (p_idx_in_chain >= p_chain.bones.size()) {
		return;
	}
	GodotIKConstraint *constraint = p_chain.constraints[p_idx_in_chain];
	if (constraint == nullptr) {
		return;
	}
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

void GodotIK::initialize() {
	initialized = false;
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}
	// Connect to bone_list_changed
	Callable callable_initialize = callable_mp(this, &GodotIK::initialize);
	if (!get_skeleton()->is_connected("bone_list_changed", callable_initialize)) {
		get_skeleton()->connect("bone_list_changed", callable_initialize);
	}

	initialize_groups();
	bone_depths = calculate_bone_depths(skeleton);
	{ // Indices by depth creation.
		Vector<int> indices;
		Vector depths = bone_depths;
		indices.resize(depths.size());
		for (int i = 0; i < depths.size(); i++) {
			indices.write[i] = i;
		}

		struct DepthComparator {
			const Vector<int> &depths;
			DepthComparator(const Vector<int> &p_depths) :
					depths(p_depths) {}

			bool operator()(int a, int b) const {
				return depths[a] < depths[b];
			}
		};

		indices.sort_custom<DepthComparator>(DepthComparator(depths));
		indices_by_depth = indices;
	}
	initialize_bone_lengths();
	initialize_chains();
	needs_processing.resize(skeleton->get_bone_count());
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
	initialized = true;
}

void GodotIK::initialize_groups() {
	godot::Skeleton3D *skeleton = get_skeleton();
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
			pos_j.snapf(snap_length);

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
	return;
}

void GodotIK::initialize_bone_lengths() {
	godot::Skeleton3D *skeleton = get_skeleton();
	if (skeleton == nullptr) {
		return;
	}
	bone_lengths = Vector<float>();
	bone_lengths.resize(skeleton->get_bone_count());
	for (int i = 0; i < skeleton->get_bone_count(); i++) {
		bone_lengths.write[i] = skeleton->get_bone_pose_position(i).length();
	}
}

void GodotIK::initialize_chains() {
	// Ensure chains are clear for initialization
	chains.clear();
	chain_forward_processing_order.clear();

	// Collect skeleton and ensure it's valid
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		ERR_PRINT("Skeleton not found for IK initialization.");
		return;
	}

	// Collect Effector child nodes and initialize chains
	for (int child_index = 0; child_index < get_child_count(); child_index++) {
		Node *child = get_child(child_index);
		GodotIKEffector *effector = Object::cast_to<GodotIKEffector>(child);
		if (effector == nullptr) {
			ERR_PRINT("Effector could not be converted");
			continue;
		}
		IKChain new_chain;
		new_chain.effector = effector;

		int bone_idx = effector->get_bone_idx();

		for (int chain_step = 0; chain_step < effector->get_chain_length() && bone_idx >= 0; chain_step++) {
			new_chain.bones.push_back(bone_idx);
			bone_idx = skeleton->get_bone_parent(bone_idx);
		}

		// add constraints
		new_chain.constraints.resize(new_chain.bones.size());
		new_chain.constraints.fill(nullptr);
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

	// Calculate bone depths for sorting

	// Prepare to process chains in forward order
	// Vector<int> chain_indices;
	// chain_indices.resize(chains.size());
	// for (int i = 0; i < chains.size(); i++) {
	// 	chain_indices.write[i] = 0;
	// }

	// bool done_processing_chains = false;
	// while (!done_processing_chains) {
	// 	int best_depth = INFINITY;
	// 	done_processing_chains = true;
	// 	for (int ch = 0; ch < chains.size(); ++ch) {
	// 		int chain_idx = chain_indices[ch];
	// 		if (chain_indices[ch] >= chains[ch].bones.size()) {
	// 			continue;
	// 		}
	// 		done_processing_chains = false;
	// 		int bone_idx = chains[ch].bones[chain_idx];
	// 		int depth = bone_depths[bone_idx];
	// 		best_depth = MIN(best_depth, depth);
	// 	}

	// 	Vector<int> process_at_step;
	// 	int selected_bone_idx = -1;
	// 	for (int ch = 0; ch < chains.size(); ++ch) {
	// 		if (chain_indices[ch] >= chains[ch].bones.size()) {
	// 			continue;
	// 		}
	// 		int chain_idx = chain_indices[ch];
	// 		int bone_idx = chains[ch].bones[chain_idx];
	// 		if (selected_bone_idx == bone_idx) {
	// 			ERR_FAIL_COND(process_at_step.size() == 0);
	// 			++chain_indices.write[ch];
	// 			process_at_step.push_back(ch);
	// 			continue;
	// 		}

	// 		int depth = bone_depths[bone_idx];
	// 		if (depth == best_depth) {
	// 			ERR_FAIL_COND(process_at_step.size() > 0);
	// 			process_at_step = { ch };
	// 			++chain_indices.write[ch];
	// 		}
	// 	}
	// 	chain_forward_processing_order.push_back(process_at_step);
	// }
	// for (int i = 0; i < chain_forward_processing_order.size(); i++){
	// 	Vector<int> step = chain_forward_processing_order[i];
	// 	godot::UtilityFunctions::print("-");
	// 	for (int j = 0; j < step.size(); j++){
	// 		UtilityFunctions::print(step[j]);
	// 	}
	// }
}
void godot::GodotIK::reset_effector_positions() {
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton)
		return;
	for (auto chain : chains) {
		if (chain.bones.size() == 0)
			continue;
		Vector3 bone_position = positions[chain.bones[0]];
		chain.effector->set_global_transform(skeleton->get_global_transform() * initial_transforms[chain.bones[0]]);
	}
}
// ! Initialization

// ------------ Helpers ----------------

Vector<int> GodotIK::calculate_bone_depths(Skeleton3D *p_skeleton) {
	if (!p_skeleton) {
		print_error("Skeleton not initialized while calculating bone depths");
		return Vector<int>();
	}

	int bone_count = p_skeleton->get_bone_count();
	if (bone_count == 0) {
		return Vector<int>();
	}

	Vector<int> depths;
	depths.resize(bone_count);

	Vector<int> process_list = { 0 }; // Start with the root bone.
	depths.set(0, 0); // Root depth is 0

	for (int i = 0; i < process_list.size(); i++) {
		int idx = process_list.get(i);
		int depth = depths.get(idx);

		PackedInt32Array children = p_skeleton->get_bone_children(idx);
		for (int child_idx : children) {
			process_list.push_back(child_idx);
			depths.set(child_idx, depth + 1);
		}
	}
	return depths;
}
// !Helpers

/*-------- Setters & Getters ------------*/
void GodotIK::set_iteration_count(int p_iteration_count) {
	iteration_count = p_iteration_count;
}

int GodotIK::get_iteration_count() const {
	return iteration_count;
}

bool godot::GodotIK::compare_by_depth(int p_a, int p_b, const Vector<int> &p_depths) {
	return p_depths[p_a] < p_depths[p_b];
}
