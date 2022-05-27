#include "ros2-components/component_listener.hpp"

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(component::component_listener)

namespace component
{
    component_listener::component_listener(const rclcpp::NodeOptions & options)
    : rclcpp::Node("component_listener_node", options)
    {
        sub_ = this->create_subscription<std_msgs::msg::String>("component_talker/says", 1, [this](const std_msgs::msg::String::SharedPtr msg){this->listenerCallback(msg);});
    }

    component_listener::~component_listener()
    {
    }

    void component_listener::listenerCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << ": I heard \""
            << msg->data
            << "\"."
        );
    }
} // namespace component