#pragma once

#include "godot_cpp/classes/ref_counted.hpp"

#include "rclcpp/subscription.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define SET_GET(m_type, m_prop_name, m_default) \
  public: \
    m_type get_##m_prop_name() { return m_prop_name##_; } \
    void set_##m_prop_name(m_type value) { m_prop_name##_ = value; } \
  protected: \
    m_type m_prop_name##_ = m_default; \
  public:

namespace godot::robosim {

class ControlInterface: public RefCounted {
  GDCLASS(ControlInterface, RefCounted)

public:
  ControlInterface();
  ~ControlInterface();

  Vector2 get_linear_cmd() { return linear_cmd_; };
  double get_angular_cmd() { return angular_cmd_; };

  void publish_odom(Vector2 linear_vel, double angular_vel);

  SET_GET(String, base_link_frame, "base_link")
  SET_GET(double, covariance_x, 0.1)
  SET_GET(double, covariance_y, 0.1)
  SET_GET(double, covariance_angular, 0.1)

protected:
  static void _bind_methods();

  Vector2 linear_cmd_;
  double angular_cmd_ = 0;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

} // namespace godot::ros

#undef SET_GET
