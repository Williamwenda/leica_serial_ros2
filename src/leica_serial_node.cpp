#include <iostream>
#include <string>
#include <memory>
#include <functional>

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
  LeicaSerialNode() : Node("leica_serial_node") {
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Declare parameters
    this->declare_parameter("comport", "/dev/ttyUSB0");
    comport_ = this->get_parameter("comport").as_string();

    // Initialize the total station interface with callback
    ts_ = std::make_unique<SerialTSInterface>(
        std::bind(&LeicaSerialNode::locationTSCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));

    // Connect to the serial port
    ts_->connect(comport_);

    // Create publisher
    prism_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/leica/position", 10);

    // Create subscribers
    pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/paintcopter/position", 10,
        std::bind(&LeicaSerialNode::positionCb, this, std::placeholders::_1));

    start_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/leica/start_stop", 10,
        std::bind(&LeicaSerialNode::startStopCb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Leica Serial Node initialized with comport: %s", comport_.c_str());
  }

  ~LeicaSerialNode() {
    if (ts_) {
      ts_.reset();
    }
  }

 private:
  void positionCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (ts_) {
      ts_->setPrismPosition(msg->pose.pose.position.x, 
                           msg->pose.pose.position.y, 
                           msg->pose.pose.position.z);
    }
  }

  void startStopCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (ts_) {
      if (msg->data) {
        ts_->start();
        RCLCPP_INFO(this->get_logger(), "Starting Leica measurements");
      } else {
        ts_->end();
        RCLCPP_INFO(this->get_logger(), "Stopping Leica measurements");
      }
    }
  }

  void locationTSCallback(const double x, const double y, const double z) {
    // Publish position message
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = z;

    prism_pos_pub_->publish(msg);

    // Publish transform
    transform_stamped_.header.stamp = this->get_clock()->now();
    transform_stamped_.header.frame_id = "world";
    transform_stamped_.child_frame_id = "leica_pos";
    transform_stamped_.transform.translation.x = x;
    transform_stamped_.transform.translation.y = y;
    transform_stamped_.transform.translation.z = z;

    q_.setRPY(0, 0, 0);
    transform_stamped_.transform.rotation.x = q_.x();
    transform_stamped_.transform.rotation.y = q_.y();
    transform_stamped_.transform.rotation.z = q_.z();
    transform_stamped_.transform.rotation.w = q_.w();

    tf_broadcaster_->sendTransform(transform_stamped_);

    RCLCPP_DEBUG(this->get_logger(), "Prism position: x=%f, y=%f, z=%f", x, y, z);
  }

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<leica_serial_ros2::LeicaSerialNode>();
  
  RCLCPP_INFO(node->get_logger(), "Leica Serial ROS2 Node started");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}