// NOT USED ANYMORE

#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tauv_msgs/msg/depth.hpp>

class DepthConverter : public rclcpp::Node {
   public:
    DepthConverter(std::string prefix);

   private:
    void depthCallback(const tauv_msgs::msg::Depth::SharedPtr msg);

    rclcpp::Subscription<tauv_msgs::msg::Depth>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

    std::string prefix_;
};
