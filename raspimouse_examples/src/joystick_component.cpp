
#include "raspimouse_examples/joystick_component.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;


namespace joystick
{
JoystickComponent::JoystickComponent(const rclcpp::NodeOptions & options)
    : Node("joystick", options)
{
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    auto callback = 
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
        {
            double velLinerX = 0.5 * msg->axes[1];
            double velAngularZ = 2.0* msg->axes[2];
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = velLinerX;
            cmd_vel.angular.z = velAngularZ;
            pub_->publish(cmd_vel);

        };
    sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

} // namespace joystick

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
