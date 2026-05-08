#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode() : Node("Teleop")
  {
    RCLCPP_INFO(this->get_logger(), "TeleopNode initialized");
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Subscribed to /joy and publishing to /cmd_vel");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  geometry_msgs::msg::Twist cmd;

  if (msg->axes.size() > 3) {

    // Negative sign fixes PS4 axis inversion
    // forward stick push → negative axes[1] → needs negation to become positive
    float lin =  -msg->axes[0] * 0.8f;
    float ang =   msg->axes[1] * 0.8f;

    // Deadband on RPi side — dont publish noise when stick is at rest
    const float DB = 0.05f;
    if (fabsf(lin) < DB) lin = 0.0f;
    if (fabsf(ang) < DB) ang = 0.0f;

    cmd.linear.x  = lin;
    cmd.angular.z = ang;

    RCLCPP_INFO(this->get_logger(),
      "Twist: linear.x=%.2f  angular.z=%.2f", lin, ang);
  } else {
    RCLCPP_WARN(this->get_logger(), "Not enough axes in Joy message");
  }

  cmd_pub_->publish(cmd);
}

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting TeleopNode main");
  rclcpp::spin(std::make_shared<TeleopNode>());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down TeleopNode main");
  rclcpp::shutdown();
  return 0;
}
