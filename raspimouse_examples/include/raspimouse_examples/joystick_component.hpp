
#ifndef JOYSTICK_COMPONENT_HPP_
#define JOYSTICK_COMPONENT_HPP_

#include "visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace joystick
{

class JoystickComponent : public rclcpp::Node
{
public:
    COMPOSITION_PUBLIC
    explicit JoystickComponent(const rclcpp::NodeOptions & options);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pubBuzzer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    geometry_msgs::msg::Twist prevCmdVel_;

    void publishCmdVel(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publishBuzzer(const sensor_msgs::msg::Joy::SharedPtr msg);
};

} // namespace joystick

#endif
