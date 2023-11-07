#include "control_interface.hpp"
#include "ros_bridge.hpp"
#include "utils.hpp"

#include <sstream>

using namespace std;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

namespace godot::robosim {

ControlInterface::ControlInterface() {
  node_ = rclcpp::Node::make_shared(Utils::get_unique_node_name("control_interface_node_", this));
  ROSBridge::get_singleton()->get_executor()->add_node(node_);

  cmd_vel_sub_ = node_->create_subscription<Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), [this](Twist::ConstSharedPtr msg){
    linear_cmd_ = Vector2(msg->linear.x, msg->linear.y);
    angular_cmd_ = msg->angular.z;
    call_deferred("emit_signal", "cmd_vel_received");
  });
  odom_pub_ = node_->create_publisher<Odometry>("odom", rclcpp::SensorDataQoS());
}

ControlInterface::~ControlInterface() {
  ROSBridge::get_singleton()->get_executor()->remove_node(node_);
}

void ControlInterface::publish_odom(Vector2 linear_vel, double angular_vel) {
  auto msg = make_unique<Odometry>();
  msg->header.frame_id = "odom";
  msg->child_frame_id = base_link_frame_.ascii().get_data();
  msg->header.stamp = node_->get_clock()->now();
  msg->twist.twist.linear.x = linear_vel.x;
  msg->twist.twist.linear.y = linear_vel.y;
  msg->twist.twist.angular.z = angular_vel;
  msg->twist.covariance[0*6+0] = covariance_x_;
  msg->twist.covariance[1*6+1] = covariance_y_;
  msg->twist.covariance[5*6+5] = covariance_angular_;
  odom_pub_->publish(std::move(msg));
  // Utils::publish_deferred(odom_pub_, std::move(msg));
}

#define DEF_PROP(m_property_name, m_property_type) \
  ClassDB::bind_method(D_METHOD("set_"#m_property_name), &ControlInterface::set_##m_property_name); \
  ClassDB::bind_method(D_METHOD("get_"#m_property_name), &ControlInterface::get_##m_property_name); \
  ADD_PROPERTY(PropertyInfo(Variant::m_property_type, #m_property_name), "set_"#m_property_name, "get_"#m_property_name)


void ControlInterface::_bind_methods() {
  ADD_SIGNAL(MethodInfo("cmd_vel_received"));
  ClassDB::bind_method(D_METHOD("get_linear_cmd"), &ControlInterface::get_linear_cmd);
  ClassDB::bind_method(D_METHOD("get_angular_cmd"), &ControlInterface::get_angular_cmd);
  ClassDB::bind_method(D_METHOD("publish_odom", "linear_vel", "angular_vel"), &ControlInterface::publish_odom);

  DEF_PROP(base_link_frame, STRING);
  DEF_PROP(covariance_x, FLOAT);
  DEF_PROP(covariance_y, FLOAT);
  DEF_PROP(covariance_angular, FLOAT);
}

#undef DEF_PROP

}
