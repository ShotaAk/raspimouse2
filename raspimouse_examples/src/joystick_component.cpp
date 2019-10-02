
#include "raspimouse_examples/joystick_component.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <cmath>

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
            publishCmdVel(msg);
        };
    sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

void JoystickComponent::publishCmdVel(const sensor_msgs::msg::Joy::SharedPtr msg){
    // joyの受信周期は約20Hz
    
    const double frame_rate = 20.0; // Hz
    const double ACC_LINEAR = 0.2 * 1.0/frame_rate; // m/ss
    const double ACC_ANGULAR = 2.0 * 1.0/frame_rate; // rad/ss
    const double GAIN_LINEAR = 0.5;
    const double GAIN_ANGULAR = 2.0;

    double targetVelX = GAIN_LINEAR * msg->axes[1];
    double targetVelZ = GAIN_ANGULAR * msg->axes[2];

    // 加速度リミッタ
    geometry_msgs::msg::Twist cmd_vel;

    double diffX = targetVelX - prevCmdVel_.linear.x;
    if(std::fabs(diffX) > ACC_LINEAR){
        cmd_vel.linear.x = prevCmdVel_.linear.x + std::copysign(ACC_LINEAR, diffX);
    }else{
        cmd_vel.linear.x = targetVelX;
    }
    double diffZ = targetVelZ - prevCmdVel_.angular.z;
    if(std::fabs(diffZ) > ACC_ANGULAR){
        cmd_vel.angular.z = prevCmdVel_.angular.z + std::copysign(ACC_ANGULAR, diffZ);
    }else{
        cmd_vel.angular.z = targetVelZ;
    }

    // 前回速度を保存
    prevCmdVel_ = cmd_vel;

    pub_->publish(cmd_vel);
}

} // namespace joystick

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
