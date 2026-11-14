#ifndef TAUV_DEPTH__DEPTH_NODE_HPP_
#define TAUV_DEPTH__DEPTH_NODE_HPP_

#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tauv_depth/ms5837.hpp"

namespace tauv_depth
{

class Bar02Node : public rclcpp::Node
{
public:
  Bar02Node();
  ~Bar02Node() override = default;

private:
  void initializeSensor();
  void timerCallback();

  int bus_id_{7};
  double freq_{10.0};
  int max_retries_{5};
  double variance_{0.001};

  std::unique_ptr<MS5837_02BA> sensor_;
  int error_count_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tauv_depth

#endif  // TAUV_DEPTH__DEPTH_NODE_HPP_

