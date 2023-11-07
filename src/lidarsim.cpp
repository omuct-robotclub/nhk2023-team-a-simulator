#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/world3d.hpp"
#include "godot_cpp/classes/physics_direct_space_state3d.hpp"
#include "godot_cpp/classes/physics_ray_query_parameters3d.hpp"
#include "godot_cpp/variant/basis.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/dictionary.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/classes/worker_thread_pool.hpp"

#include "lidarsim.hpp"
#include "ros_bridge.hpp"
#include "utils.hpp"

#include <sstream>

using namespace std;
using sensor_msgs::msg::LaserScan;

namespace godot::robosim {


LidarSim::~LidarSim() {
  if (node_)
    ROSBridge::get_singleton()->get_executor()->remove_node(node_);
}

void LidarSim::set_topic_name(String topic_name) {
  if (topic_name.length() == 0) return;
  topic_name_ = topic_name;
  if (!Engine::get_singleton()->is_editor_hint() && node_)
    pub_ = node_->create_publisher<LaserScan>(topic_name_.ascii().get_data(), rclcpp::SensorDataQoS());
}

String LidarSim::get_topic_name() {
  return topic_name_;
}

void LidarSim::_notification(int what) {
  switch(what) {
    case NOTIFICATION_READY: {
      if (!Engine::get_singleton()->is_editor_hint()) {
        ostringstream ss;
        ss << hex << (size_t)this;
        string node_name = "lidar_node_" + ss.str();
        node_ = rclcpp::Node::make_shared(node_name);
        ROSBridge::get_singleton()->get_executor()->add_node(node_);
        last_scan_ = node_->get_clock()->now();
        set_topic_name(topic_name_);
        set_physics_process(true);
      }
    } break;

    case NOTIFICATION_PHYSICS_PROCESS: {
      auto now = node_->get_clock()->now();
      auto diff = now - last_scan_;
      auto period = rclcpp::Duration::from_seconds(1 / publish_rate_);

      if (diff >= period) {
        scan();
        last_scan_ = now;
      }

      auto delayed_now = now - rclcpp::Duration::from_seconds(delay_);
      while (!publish_queue_.empty() && publish_queue_.front().first <= delayed_now) {
        pub_->publish(std::move(publish_queue_.front().second));
        publish_queue_.pop();
      }
    } break;
  }
}

void LidarSim::scan() {
  if (pub_ == nullptr) return;

  LaserScan::UniquePtr msg = make_unique<LaserScan>();
  msg->header.frame_id = frame_id_.ascii().get_data();
  msg->header.stamp = node_->get_clock()->now();
  msg->angle_max = 2 * numbers::pi;
  msg->angle_min = 0;
  msg->angle_increment = (msg->angle_max - msg->angle_min) / point_count_;
  msg->time_increment = 0;
  msg->range_max = range_max_;
  msg->range_min = range_min_;
  msg->ranges.resize(point_count_, std::numeric_limits<double>::quiet_NaN());

  auto world = get_world_3d();
  if (world == nullptr) return;
  auto state = world->get_direct_space_state();
  ERR_FAIL_COND(state == nullptr);

  Transform3D tr = get_global_transform();

  for (int64_t i = 0; i < point_count_; i++) {
    Ref<PhysicsRayQueryParameters3D> q = memnew(PhysicsRayQueryParameters3D);
    double theta = msg->angle_increment * i;
    Vector3 ray_to = tr.xform(range_max_ * Vector3(cos(theta), sin(theta), 0));
    q->set_from(tr.origin);
    q->set_to(ray_to);
    q->set_collision_mask(collision_mask_);
    auto result = state->intersect_ray(q);
    if (result.has("position")) {
      double range = Vector3(result["position"]).distance_to(tr.origin);
      if (range_min_ <= range && range <= range_max_)
        msg->ranges[i] = UtilityFunctions::randfn(range, stddiv_);
    }
  }

  publish_queue_.push({node_->get_clock()->now(), std::move(msg)});
}

#define DEF_PROP(m_property_name, m_property_type) \
  ClassDB::bind_method(D_METHOD("set_"#m_property_name), &LidarSim::set_##m_property_name); \
  ClassDB::bind_method(D_METHOD("get_"#m_property_name), &LidarSim::get_##m_property_name); \
  ADD_PROPERTY(PropertyInfo(Variant::m_property_type, #m_property_name), "set_"#m_property_name, "get_"#m_property_name)

void LidarSim::_bind_methods() {
  DEF_PROP(publish_rate, FLOAT);
  DEF_PROP(delay, FLOAT);
  DEF_PROP(topic_name, STRING);
  DEF_PROP(point_count, INT);
  DEF_PROP(frame_id, STRING);
  DEF_PROP(range_max, FLOAT);
  DEF_PROP(range_min, FLOAT);
  DEF_PROP(stddiv, FLOAT);

  ClassDB::bind_method(D_METHOD("set_laser_collision_mask"), &LidarSim::set_laser_collision_mask);
  ClassDB::bind_method(D_METHOD("get_laser_collision_mask"), &LidarSim::get_laser_collision_mask);
  ADD_PROPERTY(PropertyInfo(Variant::INT, "laser_collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_laser_collision_mask", "get_laser_collision_mask");

  ClassDB::bind_method(D_METHOD("scan"), &LidarSim::scan);
}

#undef DEF_PROP

}
