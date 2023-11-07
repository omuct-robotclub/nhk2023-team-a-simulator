#include "ros_bridge.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

using namespace std;

namespace godot::robosim {

ROSBridge* ROSBridge::singleton_ = nullptr;

ROSBridge::ROSBridge() {
  if (singleton_ == nullptr)
    singleton_ = this;
  executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
  thread_ = make_unique<thread>([this](){
    while (!is_shutting_down_.load()) {
      try {
        executor_->spin_some();
      } catch (std::exception& e) {
        UtilityFunctions::push_error("error while executing spin_some() ", e.what());
      }
    }
  });
}

ROSBridge::~ROSBridge() {
  if (singleton_ == this)
    singleton_ = nullptr;
  is_shutting_down_.store(true);
  thread_->join();
}

void ROSBridge::_bind_methods() {
}

}
