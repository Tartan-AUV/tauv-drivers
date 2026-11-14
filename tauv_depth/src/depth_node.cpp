#include "tauv_depth/depth_node.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>

namespace tauv_depth
{

Bar02Node::Bar02Node()
: Node("bar02_node")
{
  this->declare_parameter<int>("i2c_bus", 7);
  this->declare_parameter<double>("freq_hz", 10.0);
  this->declare_parameter<int>("max_retries", 5);
  this->declare_parameter<double>("variance", 0.001);

  bus_id_ = this->get_parameter("i2c_bus").as_int();
  freq_ = this->get_parameter("freq_hz").as_double();
  max_retries_ = this->get_parameter("max_retries").as_int();
  variance_ = this->get_parameter("variance").as_double();

  initializeSensor();

  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/os/sensors/depth", 10);

  auto period = std::chrono::duration<double>(1.0 / freq_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&Bar02Node::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Bar02 node started on I2C bus %d", bus_id_);
}

void Bar02Node::initializeSensor()
{
  try {
    sensor_ = std::make_unique<MS5837_02BA>(bus_id_);
    if (!sensor_->init()) {
      RCLCPP_ERROR(this->get_logger(), "Sensor init failed (check wiring/power)");
      sensor_.reset();
      return;
    }
    sensor_->setFluidDensity(DENSITY_FRESHWATER);
    error_count_ = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "I2C bus %d unavailable: %s", bus_id_, e.what());
    sensor_.reset();
  }
}

void Bar02Node::timerCallback()
{
  if (!sensor_) {
    initializeSensor();
    return;
  }

  try {
    if (!sensor_->read()) {
      throw std::runtime_error("read() returned false");
    }
  } catch (const std::exception & e) {
    error_count_++;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Bar02 read failed (%d/%d): %s",
      error_count_,
      max_retries_,
      e.what());

    if (error_count_ >= max_retries_) {
      RCLCPP_ERROR(this->get_logger(), "Too many failures, re-initializing sensor.");
      sensor_.reset();
    }
    return;
  }

  const double depth = sensor_->depth();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->get_clock()->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "depth_link";

  // Convert depth (positive down) to ROS Z (positive up)
  odom.pose.pose.position.z = -depth;

  // High covariance on everything except Z (index 14 = z,z in row-major 6x6)
  odom.pose.covariance.fill(1e6);
  odom.twist.covariance.fill(1e6);
  odom.pose.covariance[14] = variance_;

  publisher_->publish(odom);

  if (error_count_ > 0) {
    RCLCPP_INFO(this->get_logger(), "Bar02 recovered.");
    error_count_ = 0;
  }
}

}  // namespace tauv_depth

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tauv_depth::Bar02Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
