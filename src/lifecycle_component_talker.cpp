#include "ros2-components/lifecycle_component_talker.hpp"

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_component::component_talker)

namespace lifecycle_component
{
    component_talker::component_talker(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("component_talker_node", options)
    , count_(0)
    , param1(0)
    , param2(0)
    {
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
            << "\nparam1 "
            << param1
            << ", param2 "
            << param2
        );

        pub_->publish(std::move(msg));

        // Display params
        this->get_parameter("param1", param1);
        this->get_parameter("param2", param2);

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn component_talker::on_configure(const rclcpp_lifecycle::State &)
    {
        try
        {
            // Declare params
            this->declare_parameter<int>("param1", 0);
            this->declare_parameter<int>("param2", 0);

            pub_ = this->create_publisher<std_msgs::msg::String>("component_talker/says", 1);
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": caught rclcpp exceptions - "
                << e.what()
            );
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": caught an exception."
            );
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn component_talker::on_activate(const rclcpp_lifecycle::State &)
    {
        try
        {
            pub_->on_activate();
            // Start timer during on_active as it will automatically start once create_wall_timer is called
            timer_ = this->create_wall_timer(std::chrono::seconds (1), [this](){this->on_timer();});
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": throw an error - "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": caught an exception."
            );
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << " "
            << __func__
            << " is called."
        );

        // Emulate we are doing sth important
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn component_talker::on_deactivate(const rclcpp_lifecycle::State &)
    {
        try
        {
            pub_->on_deactivate();
            timer_->cancel();
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": throw an error - "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": caught an exception."
            );
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << " "
            << __func__
            << " is called."
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn component_talker::on_cleanup(const rclcpp_lifecycle::State &)
    {
        try
        {
            this->undeclare_parameter("param1");
            this->undeclare_parameter("param2");
            timer_.reset();
            pub_.reset();
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(), this->get_name()
                << " "
                << __func__
                << ": throw an error - "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": caught an exception."
            );
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << " "
            << __func__
            << " is called."
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn component_talker::on_shutdown(const rclcpp_lifecycle::State &)
    {
        try
        {
            this->undeclare_parameter("param1");
            this->undeclare_parameter("param2");
            if(!timer_->is_canceled())
                timer_->cancel();
            timer_.reset();

            if(pub_->is_activated())
                pub_->on_deactivate();
            pub_.reset();
        }
        catch(const rclcpp::exceptions::RCLError & e)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": throw an error - "
                << e.what()
            );

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                this->get_name()
                << " "
                << __func__
                << ": caught an exception."
            );
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << " "
            << __func__
            << " is called."
        );

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

} // namespace lifecycle_component