#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// STL
#include <chrono>

namespace component
{
    class component_talker : public rclcpp::Node
    {
    public:
        explicit component_talker(const rclcpp::NodeOptions & options);
        ~component_talker();

    protected:
        void on_timer();

    private:
        size_t count_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

} // namespace component