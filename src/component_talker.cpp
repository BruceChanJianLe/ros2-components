#include "ros2-components/component_talker.hpp"

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(component::component_talker)

namespace component
{
    component_talker::component_talker(const rclcpp::NodeOptions & options)
    : rclcpp::Node("component_talker_node", options)
    , count_(0)
    {
        pub_ = this->create_publisher<std_msgs::msg::String>("component_talker/says", 1);
        timer_ = this->create_wall_timer(std::chrono::seconds (1), [this](){this->on_timer();});
    }

    component_talker::~component_talker()
    {
    }

    void component_talker::on_timer()
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Talker says " + std::to_string(++count_);
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << ": "
            << msg->data
        );

        pub_->publish(std::move(msg));
    }
} // namespace component