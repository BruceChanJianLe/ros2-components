#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace component
{
    class component_listener : public rclcpp::Node
    {
    public:
        explicit component_listener(const rclcpp::NodeOptions & options);
        ~component_listener();

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        void listenerCallback(const std_msgs::msg::String::SharedPtr);
    };

} // namespace component
