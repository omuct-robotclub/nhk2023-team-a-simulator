#pragma once

#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/timer.hpp"
#include "godot_cpp/variant/string.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <queue>

namespace godot::robosim {

class LidarSim : public Node3D {
  GDCLASS(LidarSim, Node3D)

public:
  ~LidarSim();

  void set_topic_name(String topic_name);
  String get_topic_name();

  void set_publish_rate(double hz) { publish_rate_ = hz; }
  double get_publish_rate() { return publish_rate_; }

  void set_delay(double delay) { delay_ = delay; }
  double get_delay() { return delay_; }

  void set_point_count(int64_t c) { point_count_ = c; }
  int64_t get_point_count() { return point_count_; };

  void set_frame_id(String name) { frame_id_ = name; }
  String get_frame_id() { return frame_id_; }

  void set_range_max(double max) { range_max_ = max; }
  double get_range_max() { return range_max_; }

  void set_range_min(double min) { range_min_ = min; }
  double get_range_min() { return range_min_; }

  void set_stddiv(double stddiv) { stddiv_ = stddiv; }
  double get_stddiv() { return stddiv_; }

  void set_laser_collision_mask(uint32_t layer) { collision_mask_ = layer; }
  uint32_t get_laser_collision_mask() { return collision_mask_; }

  void _notification(int what);
  void scan();

protected:
  static void _bind_methods();

  String topic_name_;
  double publish_rate_ = 10;
  double delay_ = 0.1;
  int64_t point_count_ = 200;
  String frame_id_;
  double range_max_ = 10.0;
  double range_min_ = 0.1;
  double stddiv_ = 0.0;
  uint32_t collision_mask_ = 1;
  Timer* timer_;
  rclcpp::Time last_scan_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  std::queue<std::pair<rclcpp::Time, sensor_msgs::msg::LaserScan::UniquePtr>> publish_queue_;
  // rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace godot::ros
