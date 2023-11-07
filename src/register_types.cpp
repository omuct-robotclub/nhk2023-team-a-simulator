/* godot-cpp integration testing project.
 *
 * This is free and unencumbered software released into the public domain.
 */

#include <gdextension_interface.h>

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "godot_cpp/variant/dictionary.hpp"
#include "godot_cpp/variant/variant.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/init_options.hpp"

#include "register_types.h"

#include "ros_bridge.hpp"
#include "lidarsim.hpp"
#include "control_interface.hpp"

using namespace std;

static bool initialized = false;

namespace godot::robosim {

Variant add_project_setting(String name, Variant default_value, int type,
                            int hint = PROPERTY_HINT_NONE,
                            String hint_string = "") {
  if (ProjectSettings::get_singleton()->has_setting(name))
    return ProjectSettings::get_singleton()->get_setting(name);

  Dictionary setting_info;
  setting_info["name"] = name;
  setting_info["type"] = type;
  setting_info["hint"] = hint;
  setting_info["hint_string"] = hint_string;

  ProjectSettings::get_singleton()->set_setting(name, default_value);
  ProjectSettings::get_singleton()->add_property_info(setting_info);
  ProjectSettings::get_singleton()->set_initial_value(name, default_value);

  return default_value;
}

void initialize_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }
  if (initialized)
    return;
  initialized = true;

  rclcpp::InitOptions opts{};
  rclcpp::init(0, nullptr, opts, rclcpp::SignalHandlerOptions::SigTerm);

  ClassDB::register_class<ROSBridge>();
  Engine::get_singleton()->register_singleton("ROSBridge", memnew(ROSBridge));
  ClassDB::register_class<LidarSim>();
  ClassDB::register_class<ControlInterface>();
}

void finalize_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }
  if (!initialized)
    return;
  initialized = false;

  Engine::get_singleton()->unregister_singleton("ROSBridge");
  memdelete(ROSBridge::get_singleton());
  rclcpp::shutdown();
}

} // namespace godot::robosim

extern "C" {
// Initialization.
GDExtensionBool GDE_EXPORT robosim_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(godot::robosim::initialize_module);
	init_obj.register_terminator(godot::robosim::finalize_module);
	init_obj.set_minimum_library_initialization_level(godot::MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}

}
