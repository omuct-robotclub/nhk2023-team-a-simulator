/* godot-cpp integration testing project.
 *
 * This is free and unencumbered software released into the public domain.
 */

#ifndef GODOT_ROS_REGISTER_TYPES_H
#define GODOT_ROS_REGISTER_TYPES_H

#include "godot_cpp/godot.hpp"

namespace godot::robosim {

void initialize_module(ModuleInitializationLevel p_level);
void finalize_module(ModuleInitializationLevel p_level);

} // namespace godot::ros

#endif
