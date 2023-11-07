#pragma once

#include "godot_cpp/classes/object.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "godot_cpp/classes/worker_thread_pool.hpp"

namespace godot::robosim {

class ROSBridge : public Object {
  GDCLASS(ROSBridge, Object)

public:
  ROSBridge();
  ~ROSBridge();

  static ROSBridge* get_singleton() { return singleton_; }
  rclcpp::Executor::SharedPtr get_executor() { return executor_; }

protected:
  static void _bind_methods();

  std::unique_ptr<std::thread> thread_;
  std::atomic_bool is_shutting_down_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  static ROSBridge* singleton_;
};

} // namespace godot::ros
