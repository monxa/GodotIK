#ifndef GODOT_IK_POLE
#define GODOT_IK_POLE

#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/classes/node3d.hpp>

namespace godot {
    class GodotIKPole : public Node3D{
        GDCLASS(GodotIKPole, Node3D);

        protected:
        static void _bind_methods(){}
    };
}

#endif // ! GODOT_IK_POLE