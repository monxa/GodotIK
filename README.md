# GodotIK

## 3D Inverse Kinematics for Godot 4.3+

> I needed IK. People said IK was hard. I did it anyway.

This project provides four classes:

- **`GodotIK`**  
  A `SkeletonModifier` that does all the magic.
  
- **`GodotIKEffector`**  
  Parented by `GodotIK` in the SceneTree, these nodes position the bones, define the chain length, and rotate the affected bone according to their `transform_behavior` property.
  
- **`GodotIKConstraint`**  
  An abstract class that is parented by `GodotIKEffector`s. Four examples are provided as GDScript implementations: `PoleBoneConstraint`, `StraightBoneConstraint`, `SmoothBoneConstraint`, `MaxRotationBoneConstraint`.

- **`GodotIKRoot`**
  An alternative root node, under which effectors and constraints can be placed. Links into `GodotIK` through a `Nodepath` refernce.

This is the best implementation I could come up with. It's stable - at least as stable as it gets with constraints.

For further reference, check out the In-Editor-Documentation: The `doc_classes/` directory; it tells it all.

<sup>SceneTree example: Skeleton3D -> GodotIK -> [GodotIKEffectors] -> GodotIKConstraint</sup>


### Demo
[demo_libik.webm](https://github.com/user-attachments/assets/f0c0ee04-3cf1-4af9-b92a-091fd7bb769b)

<sup>(Video sometimes doesn't load. Refreshing page often helps.)</sup>

### Getting started
We have binaries!
#### The easy way
1. Grab the binaries [https://github.com/monxa/GodotIK/releases](https://github.com/monxa/GodotIK/releases)
2. Unpack and copy /addons into \<your-project-path\>

#### The hard way (build it yourself)
1. git clone
2. git submodule init # for godot-cpp
3. git submodule update # for godot-cpp
4. scons target=template_release # compile.
5. cp -r godot_project/addons/libik \<your-project-path\>/addons

<sup> If you recently unsuccessfully cloned or tried to build the repository, it was probably because the godot-cpp submodule was configured to use the SSH origin, which you needed to set up. We have now switched to the HTTPS origin. To fix this, run git submodule sync followed by git submodule update. </sup>

### Tradeoffs

This solution uses FABRIK, a positional solver. Because it solves positions first, twist is dynamically calculated in a post-processing step. Positions translate into rotations, then into local transforms.

Pros:
- Extremely flexible and stable.
- Fully customizable constraint interface: Overwrite solver behavior, limit rotations, smooth motion, introduce bone stretch, etc.

Cons:
- Constraints can’t apply twist, as it's inferred dynamically.
- A pole bone-based alternative for twist is under consideration.

I’ve tried to make this as easy to use as possible and am actively improving it. Contributions are welcome!
