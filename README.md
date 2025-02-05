# GodotIK

## 3D Inverse Kinematics for Godot 4.3+

> I needed IK. People said IK was hard. I did it anyway.

This project provides three classes:

- **`GodotIK`**  
  A `SkeletonModifier` that does all the magic.
  
- **`GodotIKEffector`**  
  Parented by `GodotIK` in the SceneTree, these nodes position the bones, define the chain length, and rotate the affected bone according to their `transform_behavior` property.
  
- **`GodotIKConstraint`**  
  An abstract class that is parented by `GodotIKEffector`s. Two examples are provided as GDScript implementations: `PoleConstraint` and `StraightBoneConstraint`.

This is the best implementation I could come up with. It's stable—at least as stable as it gets with constraints.

For further reference, check out the `doc_classes/` directory; it tells it all.

<sup>SceneTree example: Skeleton3D -> GodotIK -> [GodotIKEffectors] -> GodotIKConstraint</sup>

### Getting started
Binaries soon.

In the meantime, to use this, you can run
1. git clone
2. git submodule init # for godot-cpp
3. git submodule update # for godot-cpp
4. scons # compile
5. cp -r godot_project/addons/libik <your-project-path>/addons
