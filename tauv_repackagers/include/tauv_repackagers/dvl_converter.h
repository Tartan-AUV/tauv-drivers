#pragma once

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <dvl_msgs/msg/dvl.hpp>

class DvlConverter : public rclcpp::Node {
   public:
    DvlConverter(std::string prefix);

   private:
    void dvlCallback(const dvl_msgs::msg::DVL::SharedPtr msg);

    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
    uint64_t dropped_{0};

    std::string prefix_;
};