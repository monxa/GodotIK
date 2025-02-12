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


### Demo

[demo_libik.webm](https://github.com/user-attachments/assets/f0c0ee04-3cf1-4af9-b92a-091fd7bb769b)

[demo_local_rotations.webm](https://github.com/user-attachments/assets/97054a20-d62d-48fd-94d2-75b65f4042ad)

<sup>(Video sometimes doesn't load. Refreshing page often helps.)</sup>

### Getting started
We have binaries!
#### The easy way
1. Download release [https://github.com/monxa/GodotIK/releases](https://github.com/monxa/GodotIK/releases)
2. Unpack and copy /addons into \<your-project-path\>

#### The hard way (build it yourself)
1. git clone
2. git submodule init # for godot-cpp
3. git submodule update # for godot-cpp
4. scons # compile
5. cp -r godot_project/addons/libik \<your-project-path\>/addons

<sup> If you recently unsuccessfully cloned or tried to build the repository, it was probably because the godot-cpp submodule was configured to use the SSH origin, which you needed to set up. We have now switched to the HTTPS origin. To fix this, run git submodule sync followed by git submodule update. </sup>
