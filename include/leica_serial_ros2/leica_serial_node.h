#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "leica_serial_ros2/serial_total_station_interface.h"

namespace leica_serial_ros2 {

class LeicaSerialNode : public rclcpp::Node {
 public:
  LeicaSerialNode();
  ~LeicaSerialNode();

 private:
  void positionCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void startStopCb(const std_msgs::msg::Bool::SharedPtr msg);
  void locationTSCallback(double x, double y, double z);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr prism_pos_pub_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
  tf2::Quaternion q_;

  std::unique_ptr<SerialTSInterface> ts_;
  
  // Parameters
  std::string comport_;
};

} // namespace leica_serial_ros2