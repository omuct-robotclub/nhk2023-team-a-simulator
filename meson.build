project('robosim', 'cpp', default_options : ['cpp_std=c++20'])

sources = [
  'src/register_types.cpp',
  'src/ros_bridge.cpp',
  'src/lidarsim.cpp',
  'src/control_interface.cpp'
]

deps = [
  dependency('godot-cpp'),
  dependency('rclcpp'),
  dependency('sensor_msgs'),
  dependency('geometry_msgs'),
  dependency('nav_msgs')
]

library('robosim', sources, dependencies: deps, cpp_args: ['-Wno-non-virtual-dtor'])

# lib = library('godot-ros', sources, dependencies: deps, include_directories: incdir, cpp_args: ['-Wno-non-virtual-dtor'], install : true)
# install_data('godot-ros.gdextension', install_dir: '.')
